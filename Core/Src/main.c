/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>
#include "lcd_i2c.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef enum
{
  WAIT_AA,
  WAIT_55,
  WAIT_LEN,
  WAIT_PAYLOAD,
  WAIT_CHECKSUM,
  /* Target-name message: a second framing (header 0xA5 0x5A, never 0xAA 0x55)
     that shares this same parser/ISR so the two message types can never race on
     rx_byte. Low rate only - sent on target change, never per tracking frame. */
  WAIT_NAME_A5,
  WAIT_NAME_LEN,
  WAIT_NAME_PAYLOAD,
  WAIT_NAME_CHECKSUM
} uart_rx_state_t;

/* Per-axis controller state for the dead-time damping and the anti-ring
   watchdog described next to VISION_LATENCY_MS and RING_HALF_PERIOD_MS. */
typedef struct
{
  float    inflight_us;      /* motion already commanded that vision can't see yet */
  float    gain_scale;       /* 1.0 normally, cut by the watchdog, handed back slowly */
  float    peak_err;         /* |error| peak so far in the current half-cycle */
  float    prev_peak_err;    /* the previous half-cycle's peak, to spot growth vs decay */
  uint32_t last_cross_tick;  /* when the error last changed sides */
  uint8_t  ring_count;       /* consecutive fast, non-decaying half-cycles */
  int8_t   err_sign;         /* which side of centre the error was on last sample */
} axis_ctl_t;

/* What the autonomous follower last commanded the drivetrain to do. Kept so an
   unchanged decision doesn't re-write the motor GPIOs every pass, and so it can
   be shown on the debug line. STOP is the safe default. */
typedef enum
{
  DRIVE_STOP = 0,
  DRIVE_LEFT,
  DRIVE_RIGHT,
  DRIVE_FORWARD,
  DRIVE_BACKWARD
} drive_state_t;

/* One snapshot of everything the follower decides on, taken in a single
   critical section. Unlike vision_take_sample() this does not consume
   new_target_data - the pan/tilt controller owns that flag and must stay its
   only consumer, or the two loops would silently steal each other's frames. */
typedef struct
{
  int16_t  err_x;
  uint8_t  size_pct;
  uint8_t  flags;
  uint32_t packet_tick;
  uint32_t status_tick;
} vehicle_vision_t;

/* Everything that differs between the pan and tilt axes, in one place, so a new
   difference has to be added here rather than scattered through the file. */
typedef struct
{
  float   sign;           /* direction calibration, +/-1.0f */
  float   gain;           /* us of pulse per pixel of error */
  float   damp;           /* fraction of the in-flight move discounted per step */
  float   band_px;        /* dead-zone half-width */
  float   delta_max_us;   /* cap on one step */
  float   pulse_min_us;
  float   pulse_max_us;
  uint8_t ring_trip;      /* fast non-decaying half-cycles tolerated before backing off */
} axis_cfg_t;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* Pan/tilt visual tracking. The laptop's vision program sends target position
   over USART2 at 115200 baud; the RX interrupt parses each packet and the main
   control loop (every CONTROL_PERIOD_MS) turns that into servo pulses on TIM3
   CH1 (pan, PA6) and CH2 (tilt, PA7).
   Sign convention from the vision side: err_x > 0 means the target is right of
   frame centre, err_y > 0 means it is below frame centre. */

/* Hard travel limits - every pulse written to a servo is clamped to these. */
#define PAN_PULSE_MIN_US      700.0f
#define PAN_PULSE_MAX_US      2300.0f
/* Tilt's home position sits close to one end of its travel, so upward room is
   scarce: with the floor at 400us there was only about 9 degrees of upward
   travel. Lowering the floor to 350us gives about 150us (~13 degrees) of real
   upward authority - enough to re-centre a target near the top of frame.
   400.0f was never a measured servo limit, just an old guess, so 350us is
   stepping into unverified territory on purpose. If tilt buzzes or hums at the
   top of its travel, or the board resets while looking up, the servo is
   stalling against its internal stop - put this back to 400.0f. Full
   symmetric up/down travel would need the tilt horn remounted, not a software
   change. */
#define TILT_PULSE_MIN_US     350.0f
#define TILT_PULSE_MAX_US     2200.0f

/* Calibrated home position, applied once at boot and never touched again by
   the tracking loop. Pan's neutral is PAN_CENTER_US + PAN_TRIM_US. Tilt is
   driven to its known bottom stop (TILT_BOTTOM_US), held there briefly, then
   moved up by TILT_TRIM_US - that makes the boot position repeatable no
   matter where the horn happened to power up.
   TILT_TRIM_US=1700 puts home at 500us, leaving ~150us of upward room above
   the 350us floor above. Prefer lowering TILT_PULSE_MIN_US over raising this
   if more upward room is needed - lowering TILT_TRIM_US instead tilts the
   boot framing further down and can lose a standing person entirely.
   This block is hand-calibrated against the physical rig; don't change it
   without a reason. */
#define PAN_CENTER_US         1500U
#define PAN_TRIM_US           0
#define TILT_CENTER_US        1500U  /* reference only - tilt neutral comes from the bottom stop */
#define TILT_BOTTOM_US        TILT_PULSE_MAX_US /* increasing pulse tilts down; this is the down limit */
#define TILT_TRIM_US          1700
#define TILT_BOTTOM_HOLD_MS   300U   /* hold at the bottom limit before moving to the trimmed centre */
#define SERVO_HOME_SETTLE_MS  200U   /* hold neutral before any tracking command is honoured */

/* Direction calibration - the only two knobs that decide which way the camera
   turns. Confirmed on hardware: pan converges (rings slightly rather than
   running away) and tilt tracks downward correctly, so leave these as they
   are unless the camera starts running away from the target on an axis. */
#define PAN_SIGN             -1.0f
#define TILT_SIGN             1.0f

/* Incremental controller: each fresh vision sample nudges the pulse by
   gain * error, capped per step, with damping to fight the lag between a move
   being commanded and it actually showing up in the next camera frame (the
   vision pipeline runs roughly 150-200ms behind: capture, inference, then
   serial). Without that damping, the controller kept "correcting" a move the
   camera had already made because the frame proving it hadn't arrived yet,
   which shows up as a growing left-right (or up-down) oscillation. Pan and
   tilt share the same gain and damping because they share the same pipeline
   and the same lag - there was never a good reason for them to differ.
   If tracking feels sluggish, raise the gain a little; if it starts ringing,
   lower it. Change the damping value first though - it targets the actual
   cause. */
#define PAN_GAIN              0.12f
#define TILT_GAIN              0.12f
#define PAN_DELTA_MAX_US      60.0f  /* per-step cap so one bad frame can't cause a lurch */
/* Tilt's cap has to be read against how much travel it actually has: 150us of
   upward room at 50us/step is only three steps, so a large cap makes upward
   correction slam-to-the-stop instead of smooth. 25us gives about six steps
   of up-room and doesn't slow real tracking - a person standing up shifts the
   box fast enough that the cap never binds during a real move. */
#define TILT_DELTA_MAX_US     25.0f

/* Dead-zone half-width in pixels, per axis - must be wider than normal
   detector jitter or the servo hunts on a stationary target. Tilt's box
   height is noisier frame to frame than its box width (posture, limbs), so it
   gets the wider band. Tune against the ex=/ey= fields in the TRACKING_DEBUG
   line: pick a little above the resting jitter you see there. */
#define PAN_DEADBAND_PX       35.0f
#define TILT_DEADBAND_PX      35.0f
#define ERR_FILTER_OLD        0.15f  /* light smoothing only - heavy filtering just adds lag */
#define ERR_FILTER_NEW        0.85f

/* Dead-time damping: the fix for a slow, growing left-right (or up-down)
   swing. This loop is incremental, so the loop gain scales with how many
   frames arrive per second - a faster laptop means bigger effective gain even
   though no constant changed. rate_scale below normalises for that.
   Separately, the controller keeps a running total of motion it has commanded
   that the camera hasn't shown yet (inflight_us) and subtracts a fraction of
   it (DAMP) from every new step, so it stops re-commanding a correction that
   has already happened but hasn't appeared in a frame yet. This is a cut-down
   Smith predictor, built from our own command history (exact, no lag) rather
   than differentiating the noisy vision error, which would mostly amplify
   jitter.
   If it still rings, raise DAMP; if it feels sluggish, lower it - and change
   DAMP before GAIN, since GAIN is already confirmed on hardware. */
#define VISION_LATENCY_MS       180.0f /* capture -> inference -> serial, measured 150-200ms */
#define VISION_NOMINAL_FRAME_MS 66.0f  /* ~15fps, the rate the gains above were tuned at */
#define RATE_SCALE_MIN          0.35f
#define RATE_SCALE_MAX          1.25f  /* a dropped frame must not turn into a lurch */
#define PAN_DAMP                0.30f
#define TILT_DAMP               0.30f

/* Anti-ring watchdog: a backstop for the rare case where an oscillation
   starts anyway (slower laptop, bigger target, extra servo friction). It
   looks for the one pattern only a limit cycle produces - the error crossing
   zero fast and repeatedly, with peaks that aren't shrinking - and cuts that
   axis's gain until it settles, handing the gain back gradually once it does.
   A real moving target crosses far more slowly than a delay-driven ring does,
   so this never mistakes normal tracking for ringing.
   Tilt gets fewer chances than pan before backing off, since it has much less
   travel to lose the target off the edge of the frame in. */
#define RING_HALF_PERIOD_MS   900U   /* crossings farther apart than this are a real target */
#define RING_PEAK_DECAY_OK    0.70f  /* a healthy overshoot decays; above 70% of last peak rings */
#define PAN_RING_TRIP         3U
#define TILT_RING_TRIP        2U
#define RING_GAIN_CUT         0.60f
#define RING_GAIN_MIN         0.35f  /* never cut below this - it still has to be able to track */
#define RING_GAIN_RECOVER     0.010f /* handed back per settled sample, about 4s to fully recover */
#define CONTROL_PERIOD_MS     20U    /* control loop rate; a step also needs a fresh sample */

/* How old the newest vision packet may be before the link counts as dead.
   This one number gates the pan/tilt freeze in tracking_update(), both status
   LEDs, the LCD, and autonomous forward motion, so they can never disagree
   about whether the link is alive.
   This is a link-death timeout, not a target-lost timeout - a packet saying
   "not detected" stops things immediately through the status flags, with no
   reference to this timer at all. So this number only needs to comfortably
   clear the slowest healthy frame interval.
   The Pi 4 doing YOLO on CPU takes roughly 150-700ms per frame depending on
   image size, so 4000ms survives several dropped frames or a GC pause
   without ever masking a genuinely dead link (unplugged cable, killed vision
   process). If the vision side's frame rate changes a lot, recheck this
   against the PERF line it prints and keep it at roughly 3-4x the measured
   interval. */
#define FAILSAFE_TIMEOUT_MS   4000U

#define TRACKING_DEBUG        1      /* 1 = print a compact status line once a second over USART2 */

/* L9110S channel A ("A-1A"/"A-1B") drives one wheel, channel B ("B-1A"/"B-2A")
   drives the other. Both channels are confirmed working: driving a channel's
   pin_1 HIGH / pin_2 LOW spins that wheel in its "forward" direction. */
#define MOTOR_A_PORT_1        GPIOB
#define MOTOR_A_PIN_1         GPIO_PIN_10   /* A-1A */
#define MOTOR_A_PORT_2        GPIOB
#define MOTOR_A_PIN_2         GPIO_PIN_5    /* A-1B */
#define MOTOR_B_PORT_1        GPIOA
#define MOTOR_B_PIN_1         GPIO_PIN_9    /* B-1A */
#define MOTOR_B_PORT_2        GPIOA
#define MOTOR_B_PIN_2         GPIO_PIN_8    /* B-2A */

/* Physical chassis mapping - which electrical channel is on which side, and
   whether a side's "electrical forward" actually drives it backward once
   mounted. Keeping this here means motors_forward()/backward()/turn_left()/
   turn_right() never need to know about channel labels or chassis
   orientation - if the car drives the wrong way, fix the flags here, never
   the helpers below. */
#define LEFT_MOTOR_IS_MOTOR_A 1      /* 0 if Motor B is physically the left wheel */
#define MOTOR_LEFT_REVERSED   0      /* 1 if only the left wheel spins backward when driven forward */
#define MOTOR_RIGHT_REVERSED  0      /* 1 if only the right wheel spins backward when driven forward */
/* Both wheels' electrical "forward" turned out to be physically backward once
   mounted - not the same fault as one wheel being reversed (that would make
   the car spin instead of drive straight). This one flag flips both
   straight-line motion and turning together, since a chassis-wide inversion
   affects both. */
#define MOTORS_FWD_BWD_SWAPPED 1

#if LEFT_MOTOR_IS_MOTOR_A
#define MOTOR_LEFT_PORT_1     MOTOR_A_PORT_1
#define MOTOR_LEFT_PIN_1      MOTOR_A_PIN_1
#define MOTOR_LEFT_PORT_2     MOTOR_A_PORT_2
#define MOTOR_LEFT_PIN_2      MOTOR_A_PIN_2
#define MOTOR_RIGHT_PORT_1    MOTOR_B_PORT_1
#define MOTOR_RIGHT_PIN_1     MOTOR_B_PIN_1
#define MOTOR_RIGHT_PORT_2    MOTOR_B_PORT_2
#define MOTOR_RIGHT_PIN_2     MOTOR_B_PIN_2
#else
#define MOTOR_LEFT_PORT_1     MOTOR_B_PORT_1
#define MOTOR_LEFT_PIN_1      MOTOR_B_PIN_1
#define MOTOR_LEFT_PORT_2     MOTOR_B_PORT_2
#define MOTOR_LEFT_PIN_2      MOTOR_B_PIN_2
#define MOTOR_RIGHT_PORT_1    MOTOR_A_PORT_1
#define MOTOR_RIGHT_PIN_1     MOTOR_A_PIN_1
#define MOTOR_RIGHT_PORT_2    MOTOR_A_PORT_2
#define MOTOR_RIGHT_PIN_2     MOTOR_A_PIN_2
#endif

/* Status LEDs on the Arduino header.
     BLUE  = D2 = PA10 : a target is currently detected
     GREEN = D3 = PB3  : the target has been detected for enough consecutive
                          frames to count as confidently locked
   The consecutive-frame counting happens in the Python vision program; the
   STM32 is only ever told the two resulting booleans.
   PB3 is JTDO out of reset, so it needs the debug port disabled to be usable
   as a plain GPIO - see the note next to __HAL_AFIO_REMAP_SWJ_DISABLE() in
   MX_GPIO_Init(), which is why green wouldn't light without it. */
#define LED_BLUE_PORT         GPIOA
#define LED_BLUE_PIN          GPIO_PIN_10   /* D2 */
#define LED_GREEN_PORT        GPIOB
#define LED_GREEN_PIN         GPIO_PIN_3    /* D3 */

/* Bits of the optional 5th vision payload byte. */
#define VISION_STATUS_DETECTED 0x01U
#define VISION_STATUS_LOCKED   0x02U

/* Autonomous target following (drivetrain). Off by default: with this at 0,
   vehicle_follow_update() only holds the motors stopped, so the car behaves
   exactly as it did before this feature existed. Flip to 1 only with the car
   up on blocks first.
   The follower is a pure function of the vision state (it never looks at
   servo position) and runs in the normal 20ms control loop, never in the RX
   interrupt.
   Decision order, most important first:
       1. feature disabled, or still homing, or vision stale -> STOP
       2. target well to the left     -> TURN LEFT
       3. target well to the right    -> TURN RIGHT
       4. centred, target still small -> FORWARD
       5. centred, target too close   -> BACKWARD
       6. centred, in the hold band   -> STOP
   Steering always outranks driving - the car never turns and drives forward
   at the same time. */
#define AUTONOMOUS_DRIVE_ENABLED 1

/* Horizontal dead-band in pixels of err_x, inside which the car is
   considered pointed at the target and may drive. Deliberately much wider
   than PAN_DEADBAND_PX (35) - the camera head already steers itself
   continuously, so the chassis only needs to follow it roughly. Too small
   and the car pivots back and forth forever instead of ever driving
   forward. */
#define CAR_X_DEADBAND         80

/* Distance hysteresis, as percent of frame height the target's box fills.
   Bigger box = closer target. Two separate thresholds (rather than one) stop
   the car flip-flopping between forward and backward right at the
   boundary. */
#define TARGET_STOP_PCT        45U
#define TARGET_REVERSE_PCT     60U

/* A size byte of 0 means "not reported" (an older vision program sending
   only 4 or 5 bytes) as well as "nothing detected" - either way it is not a
   distance measurement and must never authorise forward motion. */
#define TARGET_SIZE_PCT_MAX    100U

/* 16x2 status LCD (PCF8574 I2C backpack, HD44780). I2C1 is remapped onto
   PB8 (SCL) / PB9 (SDA) in MX_GPIO_Init(). */
#define LCD_I2C_CLOCK_HZ        100000U

/* Target-name message: a low-rate, separate message sharing USART2 with the
   tracking packet above, sent only when the selected target changes. Its
   header (0xA5 0x5A) can never be mistaken for the tracking packet's
   (0xAA 0x55), so the two framings can never collide even if bytes from both
   arrive interleaved.
     0xA5 0x5A LEN(1..16) name[LEN bytes, ASCII] checksum
   checksum = XOR of LEN and every payload byte, same as the tracking packet.
   LEN is capped at the LCD's column count so a name can never overflow the
   RX buffer or the display. */
#define TARGET_NAME_HDR1        0xA5U
#define TARGET_NAME_HDR2        0x5AU
#define TARGET_NAME_MAX_LEN     LCD_I2C_COLS
#define TARGET_NAME_FALLBACK    "TARGET"
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart2;

I2C_HandleTypeDef hi2c1;

/* USER CODE BEGIN PV */
/* Vision link, written by the USART2 RX interrupt. */
static uint8_t          rx_byte;
static uart_rx_state_t  uart_rx_state = WAIT_AA;
static uint8_t          uart_rx_length;
static uint8_t          uart_rx_index;
static uint8_t          uart_rx_checksum;
static uint8_t          uart_rx_payload[6];   /* 4 legacy bytes + optional status byte + optional size byte */

/* Target-name message parsing, ISR-side. Shares the RX byte stream and ISR
   with the tracking parser above but keeps fully separate state, since the
   two framings can arrive interleaved on the wire. */
static uint8_t           uart_name_length;
static uint8_t           uart_name_index;
static uint8_t           uart_name_checksum;
static uint8_t           uart_name_payload[TARGET_NAME_MAX_LEN];
static volatile char     target_name_buffer[TARGET_NAME_MAX_LEN + 1] = TARGET_NAME_FALLBACK;
static volatile uint32_t bad_name_checksum_count;

volatile int16_t  target_err_x;
volatile int16_t  target_err_y;
volatile uint8_t  new_target_data;
volatile uint8_t  target_detected;         /* did the frame behind target_err_* actually contain a target? */
volatile uint8_t  vision_status_flags;     /* VISION_STATUS_* bits from the last 5-byte packet */
volatile uint8_t  target_size_pct;         /* 100*box_height/frame_height from the 6th payload byte; 0 = not reported */
volatile uint32_t last_status_tick;        /* when that status last arrived - drives the LED timeout */
volatile uint32_t last_valid_packet_tick;
volatile uint32_t valid_frames_received;
volatile uint32_t bad_checksum_count;
volatile uint32_t uart_error_count;      /* overrun/framing/noise events recovered from */

/* Servo state: the single source of truth for both channels. */
static float    pan_pulse_us  = (float)PAN_CENTER_US;
static float    tilt_pulse_us = (float)TILT_CENTER_US;
static float    filtered_err_x;
static float    filtered_err_y;
static const axis_cfg_t pan_cfg =
{
  PAN_SIGN, PAN_GAIN, PAN_DAMP, PAN_DEADBAND_PX,
  PAN_DELTA_MAX_US, PAN_PULSE_MIN_US, PAN_PULSE_MAX_US, PAN_RING_TRIP
};
static const axis_cfg_t tilt_cfg =
{
  TILT_SIGN, TILT_GAIN, TILT_DAMP, TILT_DEADBAND_PX,
  TILT_DELTA_MAX_US, TILT_PULSE_MIN_US, TILT_PULSE_MAX_US, TILT_RING_TRIP
};
static axis_ctl_t pan_ctl;
static axis_ctl_t tilt_ctl;
static uint32_t last_packet_tick;        /* arrival tick of the previously applied sample */
static uint8_t  tracking_ready;          /* 0 until the boot homing sequence has finished */
static uint8_t  comm_lost;               /* 1 while the vision link is stale */

/* Status LCD: what is currently on screen, so lcd_status_update() only
   touches the I2C bus when that would actually change. */
typedef enum
{
  LCD_SHOW_SEARCHING = 0,
  LCD_SHOW_ACQUIRING,
  LCD_SHOW_LOCKED
} lcd_shown_state_t;

static uint8_t           lcd_shown_valid;   /* 0 until the first draw, so it always happens once */
static lcd_shown_state_t lcd_shown_state;
static char               lcd_shown_name[TARGET_NAME_MAX_LEN + 1];

/* Loop scheduling */
static uint32_t last_control_tick;

/* Autonomous follower state: the last command actually issued to the
   drivetrain, so an unchanged decision isn't re-written every pass, and so
   the debug line can show what the car is doing. */
static drive_state_t vehicle_drive_state = DRIVE_STOP;
static uint8_t       vehicle_drive_valid;   /* 0 until the first command is issued */

/* Debug / telemetry (never in the control path). Sized for the
   TRACKING_DEBUG line's worst case (~247 bytes at full field width, not its
   typical ~110) so a long uptime with large counters can't silently truncate
   the newest fields. */
static char     tx_buffer[256];
#if TRACKING_DEBUG
static uint32_t last_report_tick;
static uint32_t pps_window_tick;
static uint32_t pps_window_start;
static uint32_t packets_per_second;
#endif
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM3_Init(void);
static void MX_I2C1_Init(void);
/* USER CODE BEGIN PFP */
static float   clamp_f(float value, float low, float high);
static float   deadzone_f(float err, float band);
static float   abs_f(float value);
static void    axis_ctl_init(axis_ctl_t *axis, uint32_t now);
static void    axis_ctl_forget(axis_ctl_t *axis, uint32_t now);
static void    axis_ring_watch(axis_ctl_t *axis, float err, uint8_t trip, uint32_t now);
static float   axis_apply(const axis_cfg_t *cfg, axis_ctl_t *axis, float filtered_err,
                          float pulse_us, float rate_scale, uint32_t now);
static uint8_t vision_take_sample(int16_t *err_x, int16_t *err_y, uint32_t *packet_tick,
                                  uint8_t *detected);
static void    tracking_update(uint32_t now);
static void    uart_rx_keepalive(void);
static void    status_leds_update(uint32_t now);
static void    target_name_take(char *out, size_t out_size);
static void    lcd_status_update(uint32_t now);
static void    motors_stop(void);
static void    motors_forward(void);
static void    motors_backward(void);
static void    motors_turn_left(void);
static void    motors_turn_right(void);
static void    vehicle_take_state(vehicle_vision_t *out);
static void    vehicle_drive_command(drive_state_t want);
static void    vehicle_follow_update(uint32_t now);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/* Blocking print. Boot-time only - never called from the control path. */
static void debug_send_blocking(int length)
{
  if (length > 0)
  {
    (void)HAL_UART_Transmit(&huart2, (uint8_t *)tx_buffer, (uint16_t)length, 100);
  }
}

#if TRACKING_DEBUG
/* Non-blocking print for the live loop. Drops the line if the previous one is
   still going out, so telemetry can never stall the controller. */
static void debug_send_async(int length)
{
  if ((length > 0) && (huart2.gState == HAL_UART_STATE_READY))
  {
    (void)HAL_UART_Transmit_IT(&huart2, (uint8_t *)tx_buffer, (uint16_t)length);
  }
}
#endif

/* Error with the dead-zone subtracted, not just gated. A hard gate is
   discontinuous - one pixel outside the band jumps straight to a full
   gain*err kick, which turns detector noise into a limit cycle. Subtracting
   instead makes the correction grow smoothly from zero at the edge of the
   band. */
static float deadzone_f(float err, float band)
{
  if (err > band)
  {
    return err - band;
  }
  if (err < -band)
  {
    return err + band;
  }
  return 0.0f;
}

static float clamp_f(float value, float low, float high)
{
  if (value < low)
  {
    return low;
  }
  if (value > high)
  {
    return high;
  }
  return value;
}

/* Local, so the control path pulls in no libm. */
static float abs_f(float value)
{
  return (value < 0.0f) ? -value : value;
}

/* Full reset, boot only: the axis starts at full gain with no history. */
static void axis_ctl_init(axis_ctl_t *axis, uint32_t now)
{
  axis->inflight_us     = 0.0f;
  axis->gain_scale      = 1.0f;
  axis->peak_err        = 0.0f;
  axis->prev_peak_err   = 0.0f;
  axis->last_cross_tick = now;
  axis->ring_count      = 0U;
  axis->err_sign        = 0;
}

/* Drop the history but keep the gain the watchdog settled on. Used after a
   comms dropout: any move commanded before the gap is long since visible, so
   it must not be subtracted again, and the gap itself isn't a half-cycle so
   it shouldn't be judged as one. The gain is kept on purpose - handing full
   gain straight back could restart a swing that was mid-damping when the
   link cut. */
static void axis_ctl_forget(axis_ctl_t *axis, uint32_t now)
{
  float keep_gain = axis->gain_scale;

  axis_ctl_init(axis, now);
  axis->gain_scale = keep_gain;
}

/* Watches one axis for a delay-driven limit cycle and trims its gain if it
   finds one. Called once per fresh sample with the dead-zoned error. A ring
   is: the error changes sides quickly (faster than any real target moves)
   and the new peak is no smaller than the last one. Three of those in a row
   costs the axis 40% of its gain, and the rule repeats, so a ring that
   survives one cut gets cut again. */
static void axis_ring_watch(axis_ctl_t *axis, float err, uint8_t trip, uint32_t now)
{
  int8_t sign = (err > 0.0f) ? 1 : ((err < 0.0f) ? -1 : 0);
  float  mag  = abs_f(err);

  if (sign == 0)
  {
    /* Inside the dead-zone: settled, nothing to ring about. Forget the
       history and start handing the gain back. */
    axis->peak_err      = 0.0f;
    axis->prev_peak_err = 0.0f;
    axis->ring_count    = 0U;
    axis->err_sign      = 0;
    axis->gain_scale    = clamp_f(axis->gain_scale + RING_GAIN_RECOVER, RING_GAIN_MIN, 1.0f);
    return;
  }

  if (mag > axis->peak_err)
  {
    axis->peak_err = mag;
  }

  if ((axis->err_sign != 0) && (sign != axis->err_sign))
  {
    /* The error just crossed centre: one half-cycle is complete, so judge it. */
    uint8_t fast        = ((now - axis->last_cross_tick) <= RING_HALF_PERIOD_MS) ? 1U : 0U;
    uint8_t not_decaying = ((axis->prev_peak_err > 0.0f) &&
                            (axis->peak_err > (RING_PEAK_DECAY_OK * axis->prev_peak_err))) ? 1U : 0U;

    if (fast && not_decaying)
    {
      axis->ring_count++;
    }
    else if (axis->ring_count > 0U)
    {
      axis->ring_count--;
    }

    if (axis->ring_count >= trip)
    {
      axis->gain_scale = clamp_f(axis->gain_scale * RING_GAIN_CUT, RING_GAIN_MIN, 1.0f);
      axis->ring_count = 0U;
    }

    axis->prev_peak_err   = axis->peak_err;
    axis->peak_err        = 0.0f;
    axis->last_cross_tick = now;
  }
  else
  {
    /* Still on the same side of centre: chasing a target, not ringing.
       Recover, but a quarter as fast as when fully settled, so a cut isn't
       undone mid-pursuit. */
    axis->gain_scale = clamp_f(axis->gain_scale + (RING_GAIN_RECOVER * 0.25f),
                               RING_GAIN_MIN, 1.0f);
  }

  axis->err_sign = sign;
}

/* One axis's complete update for one fresh measurement. Returns the new
   pulse:
       step  = sign * gain * gain_scale * rate * deadzone(err)   (the proportional push)
             - damp * inflight                                    (minus what's already on its way)
       pulse = clamp(pulse + clamp(step, +/-delta_max), travel limits) */
static float axis_apply(const axis_cfg_t *cfg, axis_ctl_t *axis, float filtered_err,
                        float pulse_us, float rate_scale, uint32_t now)
{
  float err = deadzone_f(filtered_err, cfg->band_px);
  float step;
  float next;

  axis_ring_watch(axis, err, cfg->ring_trip, now);

  if (err == 0.0f)
  {
    /* Centred: with err == 0 the step would be just -damp * inflight, a pure
       back-drive that used to push a stationary target's servo backward,
       re-trigger the dead-zone, and hunt forever. The in-flight term exists
       to discount a correction, never to create one of its own - with
       nothing to correct there's nothing to discount. */
    return pulse_us;
  }

  step = (cfg->sign * cfg->gain * axis->gain_scale * rate_scale * err)
         - (cfg->damp * axis->inflight_us);
  step = clamp_f(step, -cfg->delta_max_us, cfg->delta_max_us);

  next = clamp_f(pulse_us + step, cfg->pulse_min_us, cfg->pulse_max_us);

  /* Book the motion that was actually applied - after the per-step cap and
     the travel limits - never the motion that was asked for. Against a limit
     nothing moves, so nothing may accumulate; that's this loop's
     anti-windup. */
  axis->inflight_us += (next - pulse_us);

  return next;
}

/* Atomically takes the newest measurement. Returns 1 only if the ISR has
   produced a packet since the previous call. Reading err_x/err_y outside a
   critical section could pair the x of one frame with the y of the next, and
   the same goes for the detection flag. */
static uint8_t vision_take_sample(int16_t *err_x, int16_t *err_y, uint32_t *packet_tick,
                                  uint8_t *detected)
{
  uint32_t primask = __get_PRIMASK();
  uint8_t  fresh;

  __disable_irq();
  fresh        = new_target_data;
  *err_x       = target_err_x;
  *err_y       = target_err_y;
  *packet_tick = last_valid_packet_tick;
  *detected    = target_detected;
  new_target_data = 0U;
  __set_PRIMASK(primask);

  return fresh;
}

/* The live pan/tilt controller - the only code that writes TIM3 CH1/CH2 once
   tracking is running, and the only writer of pan_pulse_us/tilt_pulse_us. */
static void tracking_update(uint32_t now)
{
  int16_t  raw_x = 0;
  int16_t  raw_y = 0;
  uint32_t packet_tick = 0U;
  uint8_t  fresh;
  uint8_t  detected = 0U;
  float    dt_ms;
  float    rate_scale;
  float    decay;

  fresh = vision_take_sample(&raw_x, &raw_y, &packet_tick, &detected);

  if (!tracking_ready)
  {
    return;                       /* still homing - discard anything that arrives */
  }

  /* Vision link gone quiet: hold the current position exactly where it is,
     deliberately not a return-to-centre. */
  if ((now - packet_tick) > FAILSAFE_TIMEOUT_MS)
  {
    comm_lost = 1U;
    return;
  }

  /* No new information this tick. Re-applying the previous error would
     integrate the same measurement multiple times per frame and multiply the
     effective loop gain, which is what used to make the camera overshoot. */
  if (!fresh)
  {
    return;
  }

  /* A packet arrived but its frame held no target. The link is healthy so
     the failsafe above doesn't fire, but steering on stale/zero error is the
     other way tracking used to wander off. Freeze instead, exactly as for a
     dead link, and re-seed on re-acquisition so the first real frame isn't
     blended with a stale error. */
  if (!detected)
  {
    comm_lost = 1U;
    return;
  }

  if (comm_lost)
  {
    /* First sample after a dropout: seed the filter rather than blending in
       an arbitrarily old error, and drop the damping/ring history for the
       same reason. */
    comm_lost      = 0U;
    filtered_err_x = (float)raw_x;
    filtered_err_y = (float)raw_y;
    axis_ctl_forget(&pan_ctl,  now);
    axis_ctl_forget(&tilt_ctl, now);
  }
  else
  {
    filtered_err_x = (ERR_FILTER_OLD * filtered_err_x) + (ERR_FILTER_NEW * (float)raw_x);
    filtered_err_y = (ERR_FILTER_OLD * filtered_err_y) + (ERR_FILTER_NEW * (float)raw_y);
  }

  /* Time this step actually covers, measured at the packet tick rather than
     the 20ms loop tick, so it's correct even when frames and loop passes
     don't line up 1:1. */
  dt_ms            = (float)(packet_tick - last_packet_tick);
  last_packet_tick = packet_tick;
  rate_scale       = clamp_f(dt_ms / VISION_NOMINAL_FRAME_MS, RATE_SCALE_MIN, RATE_SCALE_MAX);

  /* Age out the in-flight move: whatever was commanded VISION_LATENCY_MS ago
     should be visible in the picture by now, so stop subtracting it. */
  decay = clamp_f(1.0f - (dt_ms / VISION_LATENCY_MS), 0.0f, 1.0f);
  pan_ctl.inflight_us  *= decay;
  tilt_ctl.inflight_us *= decay;

  /* Horizontal authority is handed to the drivetrain while the target is
     outside its steering band, so the pan servo and the car don't both
     correct the same error at once. Pan resumes for fine correction once the
     car has brought the target near centre. */
  if (abs_f((float)raw_x) > CAR_X_DEADBAND)
  {
    pan_ctl.inflight_us = 0.0f;
  }
  else
  {
    pan_pulse_us = axis_apply(&pan_cfg, &pan_ctl, filtered_err_x, pan_pulse_us,
                              rate_scale, now);
  }
  tilt_pulse_us = axis_apply(&tilt_cfg, &tilt_ctl, filtered_err_y, tilt_pulse_us,
                             rate_scale, now);

  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, (uint32_t)pan_pulse_us);
  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, (uint32_t)tilt_pulse_us);
}

/* On an overrun the HAL aborts reception and calls the error callback; with
   no callback implemented, RX was never re-armed and the link died
   permanently after a single overrun. HAL_UART_ErrorCallback below handles
   that; this is the belt-and-braces check for any path that escapes it. */
static void uart_rx_keepalive(void)
{
  uint32_t primask = __get_PRIMASK();

  __disable_irq();
  if (huart2.RxState != HAL_UART_STATE_BUSY_RX)
  {
    uart_rx_state = WAIT_AA;
    (void)HAL_UART_Receive_IT(&huart2, &rx_byte, 1);
  }
  __set_PRIMASK(primask);
}

/* Two GPIO writes, no delays, no allocation - safe on every loop pass. Both
   LEDs go dark once the vision side stops reporting status, so a dropped
   link can never leave a stale "locked" indication lit. Green additionally
   requires detected, so a malformed locked-without-detected status can't
   light green alone. */
static void status_leds_update(uint32_t now)
{
  uint8_t flags = vision_status_flags;
  uint8_t blue  = 0U;
  uint8_t green = 0U;

  if ((now - last_status_tick) <= FAILSAFE_TIMEOUT_MS)
  {
    blue  = ((flags & VISION_STATUS_DETECTED) != 0U) ? 1U : 0U;
    green = ((blue != 0U) && ((flags & VISION_STATUS_LOCKED) != 0U)) ? 1U : 0U;
  }

  HAL_GPIO_WritePin(LED_BLUE_PORT,  LED_BLUE_PIN,
                    (blue  != 0U) ? GPIO_PIN_SET : GPIO_PIN_RESET);
  HAL_GPIO_WritePin(LED_GREEN_PORT, LED_GREEN_PIN,
                    (green != 0U) ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

/* Atomically snapshots the most recently received target name. Same pattern
   as vision_take_sample(): the ISR writes the whole string in one pass, but
   the main loop must not read it mid-write if a fresh name lands between two
   of its characters. out_size must be TARGET_NAME_MAX_LEN + 1; the copy is
   always null-terminated and never reads or writes past out_size. */
static void target_name_take(char *out, size_t out_size)
{
  uint32_t primask = __get_PRIMASK();
  size_t   i;

  __disable_irq();
  for (i = 0U; (i < (out_size - 1U)) && (target_name_buffer[i] != '\0'); i++)
  {
    out[i] = target_name_buffer[i];
  }
  __set_PRIMASK(primask);

  out[i] = '\0';
}

/* Decides what the LCD should show, from the same gated detected/locked
   booleans status_leds_update() uses for the LEDs, so the two can never
   disagree - including on a vision timeout, where both fall back to "no
   target" together. Only touches the I2C bus when what it would show has
   actually changed. Safe to call unconditionally: a no-op if no LCD was
   detected at boot. */
static void lcd_status_update(uint32_t now)
{
  uint8_t           flags    = vision_status_flags;
  uint8_t           detected = 0U;
  uint8_t           locked   = 0U;
  lcd_shown_state_t new_state;
  char              name[TARGET_NAME_MAX_LEN + 1];
  char              line1[LCD_I2C_COLS + 1];
  size_t            i;

  if (!lcd_i2c_is_present())
  {
    return;
  }

  if ((now - last_status_tick) <= FAILSAFE_TIMEOUT_MS)
  {
    detected = ((flags & VISION_STATUS_DETECTED) != 0U) ? 1U : 0U;
    locked   = ((detected != 0U) && ((flags & VISION_STATUS_LOCKED) != 0U)) ? 1U : 0U;
  }

  new_state = (!detected) ? LCD_SHOW_SEARCHING : (!locked ? LCD_SHOW_ACQUIRING : LCD_SHOW_LOCKED);

  target_name_take(name, sizeof(name));

  if (lcd_shown_valid && (new_state == lcd_shown_state) &&
      ((new_state != LCD_SHOW_LOCKED) || (strcmp(name, lcd_shown_name) == 0)))
  {
    return;   /* nothing the LCD needs to show has changed */
  }

  switch (new_state)
  {
    case LCD_SHOW_SEARCHING:
      lcd_i2c_write_line(0U, "WATCHDOG CAR");
      lcd_i2c_write_line(1U, "SEARCHING...");
      break;

    case LCD_SHOW_ACQUIRING:
      lcd_i2c_write_line(0U, "TARGET FOUND");
      lcd_i2c_write_line(1U, "ACQUIRING...");
      break;

    case LCD_SHOW_LOCKED:
    default:
      /* Upper-case, truncated to the 16-column width. name[] is already
         bounded to TARGET_NAME_MAX_LEN characters, so this can't overrun
         line1[]. */
      for (i = 0U; (i < LCD_I2C_COLS) && (name[i] != '\0'); i++)
      {
        char c = name[i];
        line1[i] = ((c >= 'a') && (c <= 'z')) ? (char)(c - 'a' + 'A') : c;
      }
      line1[i] = '\0';
      lcd_i2c_write_line(0U, line1);
      lcd_i2c_write_line(1U, "FOUND");
      break;
  }

  lcd_shown_state = new_state;
  lcd_shown_valid = 1U;
  strncpy(lcd_shown_name, name, sizeof(lcd_shown_name) - 1U);
  lcd_shown_name[sizeof(lcd_shown_name) - 1U] = '\0';
}

/* Simple two-wire L9110S channel: HIGH/LOW only, no PWM. One motor per
   channel. */
typedef enum
{
  MOTOR_DIR_STOP = 0,
  MOTOR_DIR_FWD,
  MOTOR_DIR_BWD
} motor_dir_t;

static void motor_drive(GPIO_TypeDef *port1, uint16_t pin1,
                        GPIO_TypeDef *port2, uint16_t pin2,
                        motor_dir_t dir, uint8_t reversed)
{
  if (reversed && (dir != MOTOR_DIR_STOP))
  {
    dir = (dir == MOTOR_DIR_FWD) ? MOTOR_DIR_BWD : MOTOR_DIR_FWD;
  }

  switch (dir)
  {
    case MOTOR_DIR_FWD:
      HAL_GPIO_WritePin(port1, pin1, GPIO_PIN_SET);
      HAL_GPIO_WritePin(port2, pin2, GPIO_PIN_RESET);
      break;
    case MOTOR_DIR_BWD:
      HAL_GPIO_WritePin(port1, pin1, GPIO_PIN_RESET);
      HAL_GPIO_WritePin(port2, pin2, GPIO_PIN_SET);
      break;
    default:
      HAL_GPIO_WritePin(port1, pin1, GPIO_PIN_RESET);
      HAL_GPIO_WritePin(port2, pin2, GPIO_PIN_RESET);
      break;
  }
}

/* These only ever reference the MOTOR_LEFT_../MOTOR_RIGHT_.. mapping above,
   so higher-level code never needs to know which electrical channel is which
   wheel or which way it's mounted. */
static void motors_stop(void)
{
  motor_drive(MOTOR_LEFT_PORT_1,  MOTOR_LEFT_PIN_1,  MOTOR_LEFT_PORT_2,  MOTOR_LEFT_PIN_2,
             MOTOR_DIR_STOP, MOTOR_LEFT_REVERSED);
  motor_drive(MOTOR_RIGHT_PORT_1, MOTOR_RIGHT_PIN_1, MOTOR_RIGHT_PORT_2, MOTOR_RIGHT_PIN_2,
             MOTOR_DIR_STOP, MOTOR_RIGHT_REVERSED);
}

static void motors_forward(void)
{
#if MOTORS_FWD_BWD_SWAPPED
  const motor_dir_t straight_dir = MOTOR_DIR_BWD;
#else
  const motor_dir_t straight_dir = MOTOR_DIR_FWD;
#endif
  motor_drive(MOTOR_LEFT_PORT_1,  MOTOR_LEFT_PIN_1,  MOTOR_LEFT_PORT_2,  MOTOR_LEFT_PIN_2,
             straight_dir, MOTOR_LEFT_REVERSED);
  motor_drive(MOTOR_RIGHT_PORT_1, MOTOR_RIGHT_PIN_1, MOTOR_RIGHT_PORT_2, MOTOR_RIGHT_PIN_2,
             straight_dir, MOTOR_RIGHT_REVERSED);
}

static void motors_backward(void)
{
#if MOTORS_FWD_BWD_SWAPPED
  const motor_dir_t straight_dir = MOTOR_DIR_FWD;
#else
  const motor_dir_t straight_dir = MOTOR_DIR_BWD;
#endif
  motor_drive(MOTOR_LEFT_PORT_1,  MOTOR_LEFT_PIN_1,  MOTOR_LEFT_PORT_2,  MOTOR_LEFT_PIN_2,
             straight_dir, MOTOR_LEFT_REVERSED);
  motor_drive(MOTOR_RIGHT_PORT_1, MOTOR_RIGHT_PIN_1, MOTOR_RIGHT_PORT_2, MOTOR_RIGHT_PIN_2,
             straight_dir, MOTOR_RIGHT_REVERSED);
}

/* Pivot left: left wheel physically backward, right wheel physically forward. */
static void motors_turn_left(void)
{
#if MOTORS_FWD_BWD_SWAPPED
  const motor_dir_t left_dir  = MOTOR_DIR_FWD;
  const motor_dir_t right_dir = MOTOR_DIR_BWD;
#else
  const motor_dir_t left_dir  = MOTOR_DIR_BWD;
  const motor_dir_t right_dir = MOTOR_DIR_FWD;
#endif
  motor_drive(MOTOR_LEFT_PORT_1,  MOTOR_LEFT_PIN_1,  MOTOR_LEFT_PORT_2,  MOTOR_LEFT_PIN_2,
             left_dir, MOTOR_LEFT_REVERSED);
  motor_drive(MOTOR_RIGHT_PORT_1, MOTOR_RIGHT_PIN_1, MOTOR_RIGHT_PORT_2, MOTOR_RIGHT_PIN_2,
             right_dir, MOTOR_RIGHT_REVERSED);
}

/* Pivot right: left wheel physically forward, right wheel physically backward. */
static void motors_turn_right(void)
{
#if MOTORS_FWD_BWD_SWAPPED
  const motor_dir_t left_dir  = MOTOR_DIR_BWD;
  const motor_dir_t right_dir = MOTOR_DIR_FWD;
#else
  const motor_dir_t left_dir  = MOTOR_DIR_FWD;
  const motor_dir_t right_dir = MOTOR_DIR_BWD;
#endif
  motor_drive(MOTOR_LEFT_PORT_1,  MOTOR_LEFT_PIN_1,  MOTOR_LEFT_PORT_2,  MOTOR_LEFT_PIN_2,
             left_dir, MOTOR_LEFT_REVERSED);
  motor_drive(MOTOR_RIGHT_PORT_1, MOTOR_RIGHT_PIN_1, MOTOR_RIGHT_PORT_2, MOTOR_RIGHT_PIN_2,
             right_dir, MOTOR_RIGHT_REVERSED);
}

/* Autonomous target following - the only code that drives the wheels during
   normal operation. Called from the 20ms control loop, never from the RX
   interrupt, so a burst of serial traffic can never turn into a burst of
   motor commands. It shares the pan/tilt controller's vision state but takes
   its own snapshot (vehicle_take_state) rather than calling
   vision_take_sample(), because that consumes new_target_data and the
   tracking loop must stay its only consumer.
   Every path that isn't "locked, fresh and pointed at the target" ends in
   motors_stop() - the default is stop, not "keep doing the last thing". */

/* One coherent read of the vision state - same critical-section reasoning as
   vision_take_sample(), since err_x, the size byte and the status flags are
   written by the ISR at different points in the same packet. */
static void vehicle_take_state(vehicle_vision_t *out)
{
  uint32_t primask = __get_PRIMASK();

  __disable_irq();
  out->err_x       = target_err_x;
  out->size_pct    = target_size_pct;
  out->flags       = vision_status_flags;
  out->packet_tick = last_valid_packet_tick;
  out->status_tick = last_status_tick;
  __set_PRIMASK(primask);
}

/* Issues a drivetrain command through the motors_*() helpers, skipping the
   GPIO writes when the decision hasn't changed. vehicle_drive_valid starts
   at 0 so the very first decision is always written out. */
static void vehicle_drive_command(drive_state_t want)
{
  if (vehicle_drive_valid && (want == vehicle_drive_state))
  {
    return;
  }

  switch (want)
  {
    case DRIVE_LEFT:     motors_turn_right(); break;
    case DRIVE_RIGHT:    motors_turn_left();  break;
    case DRIVE_FORWARD:  motors_forward();    break;
    case DRIVE_BACKWARD: motors_backward();   break;
    case DRIVE_STOP:
    default:             motors_stop();       break;
  }

  vehicle_drive_state = want;
  vehicle_drive_valid = 1U;
}

static void vehicle_follow_update(uint32_t now)
{
  vehicle_vision_t vis;
  uint8_t          detected;

  /* Feature disabled: hold the wheels stopped rather than just returning, so
     the drivetrain is actively parked instead of left holding whatever a
     glitch last put on the pins. A plain if on the macro rather than #if is
     deliberate - the compiler still folds/drops this exactly as an #if would
     once the constant is 0, but the follower stays type-checked either way. */
  if (!AUTONOMOUS_DRIVE_ENABLED)
  {
    vehicle_drive_command(DRIVE_STOP);
    return;
  }

  /* Anything short of a fresh, freshly-detected target is a STOP - the car
     must never drive on a guess. Both timestamps are checked because they
     can go stale independently: the packet tick dying means the link is
     gone, while the status tick dying alone means the vision side has
     fallen back to packets carrying no detect/lock state, which a car can't
     drive on either.
     DETECTED alone (not LOCKED) authorises movement - waiting for LOCKED
     would add a full second of standing still on every sighting at the
     Pi's ~1fps, and DETECTED already means "seen on the most recent frame".
     LOCKED still gates the green LED and the LCD, just not the drivetrain. */
  if (!tracking_ready)
  {
    vehicle_drive_command(DRIVE_STOP);   /* still homing the servos */
    return;
  }

  vehicle_take_state(&vis);

  if (((now - vis.packet_tick) > FAILSAFE_TIMEOUT_MS) ||
      ((now - vis.status_tick) > FAILSAFE_TIMEOUT_MS))
  {
    vehicle_drive_command(DRIVE_STOP);   /* stale vision - link or status timed out */
    return;
  }

  detected = ((vis.flags & VISION_STATUS_DETECTED) != 0U) ? 1U : 0U;

  if (!detected)
  {
    vehicle_drive_command(DRIVE_STOP);
    return;
  }

  /* Steering outranks driving: while the target is outside the horizontal
     dead-band the car only pivots, never turns and drives forward at once. */
  if (vis.err_x < -CAR_X_DEADBAND)
  {
    vehicle_drive_command(DRIVE_LEFT);
    return;
  }

  if (vis.err_x > CAR_X_DEADBAND)
  {
    vehicle_drive_command(DRIVE_RIGHT);
    return;
  }

  /* Pointed at the target: use its apparent size as the distance estimate.
     A size of 0 means the vision side never reported one (an older 4/5-byte
     packet), so there's no distance information and forward motion is
     refused. */
  if ((vis.size_pct > 0U) && (vis.size_pct < TARGET_STOP_PCT))
  {
    vehicle_drive_command(DRIVE_FORWARD);
    return;
  }

  if (vis.size_pct >= TARGET_REVERSE_PCT)
  {
    vehicle_drive_command(DRIVE_BACKWARD);
    return;
  }

  /* In the hold band, or with no trustworthy size, remain stopped. */
  vehicle_drive_command(DRIVE_STOP);
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  uint8_t wiring_test_i;

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART2_UART_Init();
  HAL_UART_Receive_IT(&huart2, &rx_byte, 1);
  MX_TIM3_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */
  /* Report the reset cause so a brownout/watchdog reset (e.g. a servo stalled
     at its end-of-travel limit) shows up in the serial log instead of looking
     like a tracking bug. */
  debug_send_blocking(snprintf(tx_buffer, sizeof(tx_buffer),
                               "BOOT reset POR=%d PIN=%d SFT=%d IWDG=%d WWDG=%d LPWR=%d\r\n",
                               __HAL_RCC_GET_FLAG(RCC_FLAG_PORRST) ? 1 : 0,
                               __HAL_RCC_GET_FLAG(RCC_FLAG_PINRST) ? 1 : 0,
                               __HAL_RCC_GET_FLAG(RCC_FLAG_SFTRST) ? 1 : 0,
                               __HAL_RCC_GET_FLAG(RCC_FLAG_IWDGRST) ? 1 : 0,
                               __HAL_RCC_GET_FLAG(RCC_FLAG_WWDGRST) ? 1 : 0,
                               __HAL_RCC_GET_FLAG(RCC_FLAG_LPWRRST) ? 1 : 0));
  __HAL_RCC_CLEAR_RESET_FLAGS();

  /* Green-LED wiring check: blink it a few times on every boot, before any
     vision data exists. If it never blinks, the problem is PB3 wiring, not
     the "locked" logic in status_leds_update(). */
  wiring_test_i = 0U;
  for (; wiring_test_i < 6U; wiring_test_i++)
  {
    HAL_GPIO_TogglePin(LED_GREEN_PORT, LED_GREEN_PIN);
    HAL_Delay(150U);
  }
  HAL_GPIO_WritePin(LED_GREEN_PORT, LED_GREEN_PIN, GPIO_PIN_RESET);

  /* Homing/trim - trusted, hand-calibrated against the physical rig. Runs
     exactly once, before the control loop starts; the blocking delays here
     are boot-time settling only and never run again. */
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);

  pan_pulse_us = (float)(PAN_CENTER_US + PAN_TRIM_US);
  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, (uint32_t)pan_pulse_us);

  /* Standardise tilt's starting position: drive to the known bottom limit
     regardless of where it powered up, hold there, then move up by
     TILT_TRIM_US to reach visual centre. */
  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, (uint32_t)TILT_BOTTOM_US);
  HAL_Delay(TILT_BOTTOM_HOLD_MS);
  tilt_pulse_us = clamp_f((float)TILT_BOTTOM_US - (float)TILT_TRIM_US,
                          TILT_PULSE_MIN_US, TILT_PULSE_MAX_US);
  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, (uint32_t)tilt_pulse_us);

  /* Hold neutral and let any stray packets received during power-up settle
     out before tracking is armed. */
  HAL_Delay(SERVO_HOME_SETTLE_MS);

  debug_send_blocking(snprintf(tx_buffer, sizeof(tx_buffer),
                               "HOME pan=%u tilt=%u (tiltMin=%u tiltMax=%u upRoom=%d dnRoom=%d)\r\n",
                               (unsigned)pan_pulse_us, (unsigned)tilt_pulse_us,
                               (unsigned)TILT_PULSE_MIN_US, (unsigned)TILT_PULSE_MAX_US,
                               (int)(tilt_pulse_us - TILT_PULSE_MIN_US),
                               (int)(TILT_PULSE_MAX_US - tilt_pulse_us)));

  /* Status LCD: probed after homing so a slow/failed I2C scan can never
     perturb servo homing timing. Graceful on failure - lcd_i2c_is_present()
     stays false and the rest of the robot runs unmodified with no LCD
     attached. */
  (void)lcd_i2c_probe_and_init(&hi2c1);
  debug_send_blocking(snprintf(tx_buffer, sizeof(tx_buffer),
                               "LCD %s addr=0x%02X\r\n",
                               lcd_i2c_is_present() ? "found" : "not found",
                               lcd_i2c_detected_address()));

  /* Arm tracking last, from the calibrated position, with a clean slate. */
  filtered_err_x = 0.0f;
  filtered_err_y = 0.0f;
  axis_ctl_init(&pan_ctl,  HAL_GetTick());
  axis_ctl_init(&tilt_ctl, HAL_GetTick());
  last_packet_tick = HAL_GetTick();
  comm_lost      = 1U;              /* nothing moves until a real packet arrives */
  last_control_tick = HAL_GetTick();
  last_valid_packet_tick = HAL_GetTick() - (FAILSAFE_TIMEOUT_MS + 1U);
  vision_status_flags    = 0U;
  target_detected        = 0U;
  last_status_tick       = HAL_GetTick() - (FAILSAFE_TIMEOUT_MS + 1U);
  new_target_data = 0U;
  target_size_pct = 0U;             /* no distance information until a 6-byte packet lands */
  uart_rx_state   = WAIT_AA;
  tracking_ready  = 1U;
  /* Park the drivetrain explicitly before the loop starts, whatever the pins
     were left holding. vehicle_drive_valid = 0 forces the first
     vehicle_follow_update() call to write the motor GPIOs rather than assume
     they already agree. */
  vehicle_drive_state = DRIVE_STOP;
  vehicle_drive_valid = 0U;
  motors_stop();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */
    uint32_t now = HAL_GetTick();

    /* Keep the vision link alive: a single UART overrun used to kill
       reception forever. */
    uart_rx_keepalive();

    /* Detect/lock indicators. Pure GPIO writes - never gates or delays
       tracking. */
    status_leds_update(now);

    /* Status LCD. Only touches I2C when the displayed text actually changes,
       so this is cheap every other pass and a no-op with no LCD attached. */
    lcd_status_update(now);

    /* The one periodic control loop, fixed 20ms, non-blocking. */
    if ((now - last_control_tick) >= CONTROL_PERIOD_MS)
    {
      last_control_tick = now;
      tracking_update(now);

      /* Autonomous following runs after the camera head, in the same
         control loop, never from the RX interrupt. Compiles to a held stop
         while AUTONOMOUS_DRIVE_ENABLED is 0. */
      vehicle_follow_update(now);
    }

#if TRACKING_DEBUG
    if ((now - pps_window_tick) >= 1000U)
    {
      pps_window_tick    = now;
      packets_per_second = valid_frames_received - pps_window_start;
      pps_window_start   = valid_frames_received;
    }

    if ((now - last_report_tick) >= 1000U)
    {
      last_report_tick = now;
      /* gp/gt: the watchdog's gain scales in percent - below 100 means it
         caught that axis ringing. ifp/ift: in-flight move in us, the
         correction already sent that the frame can't show yet. up=: tilt's
         remaining upward travel in us - if that sits at 0 while ey stays
         negative, tilt isn't ringing, it's pegged against its top stop and
         needs more travel, not tuning. flg=: the raw status byte, sage=: how
         long ago one last arrived. sz=: the size byte (0 = not reported).
         drv=: what the follower last commanded. */
      debug_send_async(snprintf(tx_buffer, sizeof(tx_buffer),
                                "T pps=%lu age=%lu flg=0x%02X sage=%lu ex=%d ey=%d pan=%u tilt=%u gp=%u gt=%u ifp=%d ift=%d up=%d sz=%u drv=%s bad=%lu err=%lu\r\n",
                                (unsigned long)packets_per_second,
                                (unsigned long)(now - last_valid_packet_tick),
                                (unsigned)vision_status_flags,
                                (unsigned long)(now - last_status_tick),
                                (int)filtered_err_x, (int)filtered_err_y,
                                (unsigned)pan_pulse_us, (unsigned)tilt_pulse_us,
                                (unsigned)(pan_ctl.gain_scale * 100.0f),
                                (unsigned)(tilt_ctl.gain_scale * 100.0f),
                                (int)pan_ctl.inflight_us,
                                (int)tilt_ctl.inflight_us,
                                (int)(tilt_pulse_us - TILT_PULSE_MIN_US),
                                (unsigned)target_size_pct,
                                (vehicle_drive_state == DRIVE_FORWARD)  ? "FWD" :
                                  ((vehicle_drive_state == DRIVE_LEFT)     ? "LFT" :
                                  ((vehicle_drive_state == DRIVE_RIGHT)    ? "RGT" :
                                  ((vehicle_drive_state == DRIVE_BACKWARD) ? "BWD" : "STP"))),
                                (unsigned long)bad_checksum_count,
                                (unsigned long)uart_error_count));
    }
#endif

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 71;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 19999;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 1500;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{
  /* Conservative standard-mode speed (~100 kHz) - the PCF8574 LCD backpack
     does not need and should not be pushed to fast mode. */
  hi2c1.Instance             = I2C1;
  hi2c1.Init.ClockSpeed      = LCD_I2C_CLOCK_HZ;
  hi2c1.Init.DutyCycle       = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1     = 0;
  hi2c1.Init.AddressingMode  = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2     = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode   = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5|GPIO_PIN_8|GPIO_PIN_9, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10|GPIO_PIN_5, GPIO_PIN_RESET);

  /*Configure GPIO pins : PA5 PA8 PA9 */
  GPIO_InitStruct.Pin = GPIO_PIN_5|GPIO_PIN_8|GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB10 PB5 */
  GPIO_InitStruct.Pin = GPIO_PIN_10|GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PB8 PB9 */
  GPIO_InitStruct.Pin = GPIO_PIN_8|GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure peripheral I/O remapping */
  __HAL_AFIO_REMAP_I2C1_ENABLE();

  /* USER CODE BEGIN MX_GPIO_Init_2 */
  /* PB3 is JTDO, only usable as a plain GPIO once the debug port is disabled
     (SWJ_CFG = 0b100). HAL_MspInit() disables it, but
     __HAL_AFIO_REMAP_I2C1_ENABLE() above re-enables the whole SWJ_CFG field
     as a side effect, which is why green wouldn't light without this line.
     Keep this call last - after every AFIO_REMAP_*_ENABLE() and before PB3
     is configured. */
  __HAL_AFIO_REMAP_SWJ_DISABLE();

  /* Status LEDs: D2 = PA10 (blue), D3 = PB3 (green). Both GPIO clocks are
     already enabled above, and both LEDs start OFF. */
  HAL_GPIO_WritePin(LED_BLUE_PORT,  LED_BLUE_PIN,  GPIO_PIN_RESET);
  HAL_GPIO_WritePin(LED_GREEN_PORT, LED_GREEN_PIN, GPIO_PIN_RESET);

  GPIO_InitStruct.Pin   = LED_BLUE_PIN;
  GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull  = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_BLUE_PORT, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = LED_GREEN_PIN;
  HAL_GPIO_Init(LED_GREEN_PORT, &GPIO_InitStruct);
  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == USART2)
  {
    switch (uart_rx_state)
    {
      case WAIT_AA:
        if (rx_byte == 0xAA)
        {
          uart_rx_state = WAIT_55;
        }
        else if (rx_byte == TARGET_NAME_HDR1)
        {
          uart_rx_state = WAIT_NAME_A5;
        }
        break;

      case WAIT_55:
        if (rx_byte == 0x55)
        {
          uart_rx_state = WAIT_LEN;
        }
        else if (rx_byte == 0xAA)
        {
          uart_rx_state = WAIT_55;   /* "AA AA 55" - the second AA is the real header */
        }
        else if (rx_byte == TARGET_NAME_HDR1)
        {
          uart_rx_state = WAIT_NAME_A5;   /* a name header can follow directly after a stray AA */
        }
        else
        {
          uart_rx_state = WAIT_AA;
        }
        break;

      case WAIT_LEN:
        /* 4 = legacy err_x/err_y only. 5 = same plus a status byte for the
           LEDs. 6 = same plus a target-size byte for autonomous following.
           All three are accepted so vision and firmware can be updated
           independently of each other - LEN < 6 just leaves target_size_pct
           at 0, and a size of 0 blocks autonomous forward motion. */
        if ((rx_byte >= 4U) && (rx_byte <= 6U))
        {
          uart_rx_length   = rx_byte;
          uart_rx_index    = 0U;
          uart_rx_checksum = rx_byte;       /* checksum covers LEN + payload */
          uart_rx_state    = WAIT_PAYLOAD;
        }
        else
        {
          uart_rx_state = (rx_byte == 0xAA) ? WAIT_55 :
                           ((rx_byte == TARGET_NAME_HDR1) ? WAIT_NAME_A5 : WAIT_AA);
        }
        break;

      case WAIT_PAYLOAD:
        uart_rx_payload[uart_rx_index] = rx_byte;
        uart_rx_checksum ^= rx_byte;
        uart_rx_index++;
        if (uart_rx_index >= uart_rx_length)
        {
          uart_rx_state = WAIT_CHECKSUM;
        }
        break;

      case WAIT_CHECKSUM:
        if (rx_byte == uart_rx_checksum)
        {
          /* struct.pack("<hh", err_x, err_y): signed int16, little-endian, x then y. */
          target_err_x = (int16_t)((uint16_t)uart_rx_payload[0] |
                                   ((uint16_t)uart_rx_payload[1] << 8));
          target_err_y = (int16_t)((uint16_t)uart_rx_payload[2] |
                                   ((uint16_t)uart_rx_payload[3] << 8));
          last_valid_packet_tick = HAL_GetTick();
          new_target_data = 1U;
          valid_frames_received++;
          if (uart_rx_length >= 5U)
          {
            /* Status is only refreshed by packets that actually carry it, so
               a vision program still sending 4-byte frames leaves both LEDs
               dark via the timeout rather than latching a stale state. */
            vision_status_flags = uart_rx_payload[4];
            last_status_tick    = last_valid_packet_tick;
            /* Latched with the errors it belongs to, so the controller can
               never pair one frame's "detected" with another frame's
               pixels. */
            target_detected = ((uart_rx_payload[4] & VISION_STATUS_DETECTED) != 0U) ? 1U : 0U;
          }
          else
          {
            /* Legacy 4-byte frame carries no detection flag. A vision
               program using that protocol only ever sent a packet when it
               had a target, so treating it as detected preserves the old
               behaviour exactly. */
            target_detected = 1U;
          }
          /* 6th byte: 100 * box_height / frame_height, the only distance cue
             the car has. Set on every accepted packet, never left latched -
             a link that drops back to 4/5-byte frames must clear this to 0
             rather than keep a stale value that could authorise driving
             toward a distance that no longer exists. Clamped to 100 so a
             malformed byte can't read as "impossibly far away". */
          if (uart_rx_length >= 6U)
          {
            target_size_pct = (uart_rx_payload[5] <= TARGET_SIZE_PCT_MAX) ?
                                uart_rx_payload[5] : (uint8_t)TARGET_SIZE_PCT_MAX;
          }
          else
          {
            target_size_pct = 0U;
          }
          HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);   /* LD2 flickers = vision link alive */
        }
        else
        {
          bad_checksum_count++;
        }
        uart_rx_state = WAIT_AA;
        break;

      /* Target-name message: 0xA5 0x5A LEN name[LEN] checksum. Low rate,
         distinct header from the tracking packet above, so it can never be
         mistaken for one. See the framing note at TARGET_NAME_HDR1 in USER
         CODE BEGIN PD. */
      case WAIT_NAME_A5:
        if (rx_byte == TARGET_NAME_HDR2)
        {
          uart_rx_state = WAIT_NAME_LEN;
        }
        else if (rx_byte == 0xAA)
        {
          uart_rx_state = WAIT_55;
        }
        else if (rx_byte == TARGET_NAME_HDR1)
        {
          uart_rx_state = WAIT_NAME_A5;   /* "A5 A5 5A" - the second A5 is the real header */
        }
        else
        {
          uart_rx_state = WAIT_AA;
        }
        break;

      case WAIT_NAME_LEN:
        /* Bounded to the LCD width: a LEN outside [1, TARGET_NAME_MAX_LEN]
           can never index uart_name_payload[] out of bounds below. */
        if ((rx_byte >= 1U) && (rx_byte <= TARGET_NAME_MAX_LEN))
        {
          uart_name_length   = rx_byte;
          uart_name_index    = 0U;
          uart_name_checksum = rx_byte;
          uart_rx_state       = WAIT_NAME_PAYLOAD;
        }
        else if (rx_byte == 0xAA)
        {
          uart_rx_state = WAIT_55;
        }
        else if (rx_byte == TARGET_NAME_HDR1)
        {
          uart_rx_state = WAIT_NAME_A5;
        }
        else
        {
          uart_rx_state = WAIT_AA;
        }
        break;

      case WAIT_NAME_PAYLOAD:
        uart_name_payload[uart_name_index] = rx_byte;
        uart_name_checksum ^= rx_byte;
        uart_name_index++;
        if (uart_name_index >= uart_name_length)
        {
          uart_rx_state = WAIT_NAME_CHECKSUM;
        }
        break;

      case WAIT_NAME_CHECKSUM:
        if (rx_byte == uart_name_checksum)
        {
          uint8_t name_i;

          /* Written in one ISR pass, always null-terminated, never more
             than TARGET_NAME_MAX_LEN characters: target_name_take() in the
             main loop copies this out under a critical section so a name
             arriving mid-read can never be observed half old/half new. */
          for (name_i = 0U; name_i < uart_name_length; name_i++)
          {
            target_name_buffer[name_i] = (char)uart_name_payload[name_i];
          }
          target_name_buffer[uart_name_length] = '\0';
        }
        else
        {
          bad_name_checksum_count++;
        }
        uart_rx_state = WAIT_AA;
        break;

      default:
        uart_rx_state = WAIT_AA;
        break;
    }

    (void)HAL_UART_Receive_IT(&huart2, &rx_byte, 1);
  }
}

/* Without this, one overrun (ORE) permanently ended reception: the HAL aborts
   the Rx transfer, clears RXNEIE and calls this weak callback, which used to
   do nothing. The servos then froze on stale data with no way to recover
   short of a reset. */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == USART2)
  {
    uart_error_count++;

    if (__HAL_UART_GET_FLAG(huart, UART_FLAG_ORE))
    {
      __HAL_UART_CLEAR_OREFLAG(huart);     /* SR read then DR read */
    }
    huart->ErrorCode = HAL_UART_ERROR_NONE;

    /* The byte stream is corrupt at this point: resync the parser. */
    uart_rx_state = WAIT_AA;

    if (huart->RxState == HAL_UART_STATE_READY)
    {
      (void)HAL_UART_Receive_IT(huart, &rx_byte, 1);
    }
  }
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
