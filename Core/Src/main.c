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
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef enum
{
  WAIT_AA,
  WAIT_55,
  WAIT_LEN,
  WAIT_PAYLOAD,
  WAIT_CHECKSUM
} uart_rx_state_t;

/* Per-axis controller state for the dead-time damping and the anti-ring watchdog.
   See the notes beside VISION_LATENCY_MS and RING_HALF_PERIOD_MS. */
typedef struct
{
  float    inflight_us;      /* motion already commanded that the vision feed cannot see yet */
  float    gain_scale;       /* 1.0 normally; cut by the watchdog, handed back slowly */
  float    peak_err;         /* |error| peak so far in the current half-cycle */
  float    prev_peak_err;    /* the previous half-cycle's peak, to tell decaying from growing */
  uint32_t last_cross_tick;  /* when the error last changed sides */
  uint8_t  ring_count;       /* consecutive fast, non-decaying half-cycles */
  int8_t   err_sign;         /* which side of centre the error was on last sample */
} axis_ctl_t;

/* Everything that differs between pan and tilt, in ONE place. This exists because of the
   2026-09-15 bug: tilt was running at 1.5x pan's gain with 2/3 of its damping and a
   per-step cap nine times larger relative to its own travel, and none of that was visible
   while the constants were scattered down the file and passed positionally. Put a new
   axis difference here or not at all. */
typedef struct
{
  float   sign;           /* direction calibration, +/-1.0f */
  float   gain;           /* us of pulse per pixel of error */
  float   damp;           /* fraction of the in-flight move discounted per step */
  float   band_px;        /* dead-zone half-width */
  float   delta_max_us;   /* cap on one step - read it against pulse_max - pulse_min */
  float   pulse_min_us;
  float   pulse_max_us;
  uint8_t ring_trip;      /* fast non-decaying half-cycles tolerated before backing off */
} axis_cfg_t;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* ===========================================================================
 *  Pan/tilt visual tracking
 *
 *  Data path:
 *      laptop vision  -> USART2 @115200 -> RX ISR packet parser
 *                     -> ONE control loop (CONTROL_PERIOD_MS)
 *                     -> TIM3 CH1 (pan, PA6) / CH2 (tilt, PA7)
 *
 *  Sign convention produced by the vision side:
 *      err_x > 0  ->  target is RIGHT of frame centre
 *      err_y > 0  ->  target is BELOW  frame centre
 * =========================================================================== */

/* ---- MPU6050: retained for later stabilisation work, unused by tracking ---- */
#define MPU6050_ADDRESS       (0x68 << 1)
#define MPU6050_WHO_AM_I      0x75
#define MPU6050_PWR_MGMT_1    0x6B
#define MPU6050_ACCEL_XOUT_H  0x3B

/* ---- Hard travel limits. Every pulse written is clamped to these. ---------- */
#define PAN_PULSE_MIN_US      700.0f
#define PAN_PULSE_MAX_US      2300.0f
/* MEASURED 2026-09-14: tilt tracks DOWN well but cannot look UP. Home sits at 500us and
   "up" means DECREASING the pulse, so upward travel is whatever gap exists between home
   and this floor - it was only 100us (~9 deg), effectively nothing. Lowered to 350us to
   give ~150us (~13 deg) of real upward authority, enough to re-centre a target sitting in
   the upper third of the frame.
   IMPORTANT: 400.0f was never a measured servo limit - an earlier revision picked it purely
   so that 2200-1800 came out even. The servo's true minimum is unknown, so this steps into
   unverified territory deliberately and conservatively. If tilt BUZZES/HUMS at the top of
   its travel, or the board resets while looking up (check the BOOT reset flags), the servo
   is stalling against its internal stop - put this back to 400.0f. If it looks up cleanly
   and you want more, 300.0f is the next step.
   The honest ceiling: level sits near one end of this servo's travel, so ~1700us of unused
   DOWNWARD range cannot be traded for upward range in software. Getting symmetric up/down
   authority needs the tilt horn/bracket remounted, not a different constant. */
#define TILT_PULSE_MIN_US     350.0f  /* was 400.0f - see note above */
#define TILT_PULSE_MAX_US     2200.0f

/* ---- Calibrated home position / trim.  TRUSTED - do not "tidy" these. ------
   Pan  : neutral = PAN_CENTER_US + PAN_TRIM_US.
   Tilt : driven to the known bottom stop (TILT_BOTTOM_US), held, then moved UP
          by TILT_TRIM_US.  That makes the boot position repeatable regardless
          of where the horn powered up.  Applied ONCE at boot and never again;
          the tracking loop integrates away from it and never pulls back to it.
   MEASURED 2026-09-14 (serial boot log): with TILT_TRIM_US=1800 the home pulse landed at
   exactly 400us = TILT_PULSE_MIN_US, so tilt had ZERO upward room and every "look up"
   command clamped to a no-op. TILT_TRIM_US=1700 moves home to 500us.
   Upward travel is the gap between home and TILT_PULSE_MIN_US, so it is bought from TWO
   places and both have now been used: home moved up 100us (here), and the floor moved
   down 50us (TILT_PULSE_MIN_US, 400->350). Net upward room is ~150us.
   Prefer moving the FLOOR over moving HOME when more is needed: lowering TILT_TRIM_US
   further tilts the boot framing progressively downward, which costs you the ability to
   see a standing person at all. Watch for servo strain either way - see the note on
   TILT_PULSE_MIN_US above.
   Hard limit worth knowing: "level" sits near one end of this servo's travel, leaving
   ~1700us of DOWNWARD range that is useless and cannot be traded for upward range in
   software. Symmetric up/down authority requires remounting the horn/bracket. */
#define PAN_CENTER_US         1500U
#define PAN_TRIM_US           0
#define TILT_CENTER_US        1500U  /* reference only; tilt neutral is derived from the bottom stop */
#define TILT_BOTTOM_US        TILT_PULSE_MAX_US /* Confirmed: increasing pulse tilts down; this is the down limit */
#define TILT_TRIM_US          1700   /* was 1800; home 400->500us. With the 350us floor: ~150us up-room */
#define TILT_BOTTOM_HOLD_MS   300U   /* Hold at the bottom limit before moving to the trimmed centre */
#define SERVO_HOME_SETTLE_MS  200U   /* Hold neutral before any tracking command is honoured */

/* ---- Direction calibration ------------------------------------------------
   These are the ONLY knobs that decide which way the camera turns.
   Flip a sign to +/-1.0f if the camera runs away from the target on that axis.
   Required end behaviour:
       target right (err_x>0) -> camera pans right
       target below (err_y>0) -> camera tilts down                            */
#define PAN_SIGN             -1.0f   /* CONFIRMED on hardware: converges on the target (it rang around it
                                        rather than running away), so this sign is correct. Do not flip it. */
#define TILT_SIGN             1.0f   /* CONFIRMED on hardware: tilt tracks downward correctly. */

/* ---- Incremental controller: proportional push + dead-time damping --------- */
/* MEASURED 2026-09-14: pan rang (right-left-right-left) around a target it had already
   reached, while tilt at 0.18f/35px was stable. That asymmetry is the whole diagnosis -
   a wrong SIGN runs away and never returns, but ringing around the correct position is a
   delay-driven limit cycle. The vision pipeline (capture -> inference -> serial) is
   roughly 150-200 ms behind, so at 15 fps the controller applies ~3 more corrections
   after it has physically arrived, using an error that is already stale. Overshoot per
   swing is approximately (loop delay in samples) x (fraction of the error cancelled per
   sample); cutting the gain cuts that product directly. 0.12f keeps ~0.3 - comfortably
   damped - while still closing a 200 px error in about a second.
   If this feels sluggish, raise toward 0.16f. Do NOT go back above 0.18f.
   UPDATE 2026-09-15: 0.12f alone was still not enough - the ring came back intermittently
   and GREW. The gain is left exactly where it is; the fix is the dead-time damping added
   below, which attacks the cause (corrections issued against a stale error) instead of
   trading tracking speed away. Because that damping also discounts the large-error
   approach, this is now the one case where 0.16f is worth trying if acquisition feels
   slow - but change PAN_DAMP first and only one of the two at a time. */
/* MEASURED 2026-09-15: with pan fixed, TILT became the axis that rings and loses the
   target - and it rings on VERTICAL STEP CHANGES specifically. "Tilt is confirmed
   stable" was true but misleading: it was never actually stress-tested. Home sits near
   the top of this servo's useful range, most targets sit BELOW centre, and with only
   ~150us of up-room every upward correction hit the travel clamp instead of ringing.
   A clamp looks like stability. It is not - it is an axis that never got to overshoot.
   The dead time (150-200 ms of capture -> inference -> serial) belongs to the VISION
   PIPELINE, not to an axis, so both axes face the same stability margin and there was
   never a reason tilt could live at 1.5x pan's gain with 2/3 of pan's damping. It is
   now matched to the values confirmed on pan. */
#define PAN_GAIN              0.12f  /* was 0.20f - see note above */
#define TILT_GAIN             0.12f  /* was 0.18f - matched to pan; same loop, same dead time */
#define PAN_DELTA_MAX_US      60.0f  /* safety cap on one step, so a bad frame cannot lurch */
/* The per-step cap has to be read against the travel the axis actually HAS, and this is
   where tilt was quietly extreme. Pan: 1600us of range at 60us/step = 27 steps to cross
   it. Tilt UPWARD: 150us of room at 50us/step = THREE steps. A cap that large relative to
   the range makes upward correction effectively bang-bang - the camera slams the whole
   way to its stop and only finds out 180 ms later, which is exactly the "changes vertical
   position -> oscillates -> loses it" report. 25us gives ~6 steps of up-room instead.
   This does not slow real tracking: a person standing up shifts the box ~100 px in half a
   second, which at TILT_GAIN is ~12us/step - the cap never binds. It only tames the
   transient. */
#define TILT_DELTA_MAX_US     25.0f  /* was 50.0f - see note above */
/* Dead-zone half-width, per axis. Must be WIDER than the frame-to-frame noise on that
   axis, otherwise detector jitter alone drives the integrator into a random walk and the
   camera visibly hunts on a stationary target. A person's bounding box is markedly noisier
   vertically than horizontally (posture, limbs, box height), so tilt needs a wider band.
   Tune from the TRACKING_DEBUG ex=/ey= fields: pick a little above the resting spread. */
#define PAN_DEADBAND_PX       35.0f  /* was 20.0f; widened to match the stable tilt axis, so the
                                        residual ring settles instead of hunting forever */
#define TILT_DEADBAND_PX      35.0f
#define ERR_FILTER_OLD        0.15f  /* light filter only - heavy filtering is pure added lag */
#define ERR_FILTER_NEW        0.85f

/* ---- Dead-time damping: the cure for the growing left-right swing ----------
   OBSERVED 2026-09-15: pan usually tracks well, but now and then it starts swinging
   left-right and the swings GROW until the target leaves the frame. A growing - not
   merely persistent - amplitude is the signature of a loop whose gain has crossed the
   stability margin set by its own dead time. It is NOT a wrong sign (that runs away and
   never comes back) and NOT a too-narrow dead-zone (that hunts at constant, small
   amplitude). Two things move that margin around at run time, which is exactly why this
   only bites SOMETIMES rather than always:

     1. FRAME RATE. This is an INCREMENTAL controller: every fresh frame adds gain*error
        to the pulse, so the motion commanded during one latency window is
        (frames arriving in that window) x gain x error. If the laptop speeds up from 15
        to 30 fps, the loop gain DOUBLES although not one constant changed. The rate
        scale below makes each step proportional to the time it actually covers, so the
        commanded velocity stops depending on how often frames happen to arrive.

     2. LATENCY. Under load, capture -> inference -> serial stretches, so even more stale
        corrections pile up before the camera's own motion appears in a frame. The
        in-flight term below is the direct fix: we know exactly how much we commanded in
        the last VISION_LATENCY_MS and the camera has already physically made that move,
        so we subtract it from the next push instead of commanding it a second time. It
        is a Smith predictor cut down to the one piece that matters here, and PAN_DAMP
        absorbs the (uncalibrated) pixels-per-microsecond scale, so no camera FOV
        measurement is needed to tune it.

   This is the "D" the loop was missing, and it is deliberately taken from our COMMANDED
   motion rather than from the measured error: differentiating a noisy detection that is
   already 150-200 ms old mostly amplifies jitter, whereas our own command history is
   exact and has no lag at all.
   Tuning order: if it still rings, raise PAN_DAMP toward 0.45f. If it now feels
   sluggish, lower PAN_DAMP toward 0.20f. Either way, change PAN_DAMP before PAN_GAIN -
   the gain is already at a value confirmed on hardware. */
#define VISION_LATENCY_MS       180.0f /* capture -> inference -> serial; measured at 150-200 ms */
#define VISION_NOMINAL_FRAME_MS 66.0f  /* ~15 fps = the rate today's gains were tuned at, so the
                                          rate scale is 1.0 there and behaviour is unchanged */
#define RATE_SCALE_MIN          0.35f  /* faster frames -> proportionally smaller steps */
/* 2026-09-15: pulled 1.50f -> 1.25f. Honest note on a side effect I introduced with the
   rate scale: below the 15 fps nominal it makes every step BIGGER than the old code did
   (at 10 fps, 1.5x). That is the theoretically right thing - it holds the commanded
   velocity constant as the frame rate drops - but it stacked on top of tilt's already
   high gain, so if the laptop has been running under 15 fps this made tilt worse at the
   same moment it made pan better. Check the pps= field in the TRACKING_DEBUG line: if it
   reads well under 15, set VISION_NOMINAL_FRAME_MS to 1000/pps and this ceiling stops
   mattering. */
#define RATE_SCALE_MAX          1.25f  /* was 1.50f; a dropped frame must not become a lurch */
#define PAN_DAMP                0.30f  /* fraction of the in-flight move discounted per step */
#define TILT_DAMP               0.30f  /* was 0.20f; matched to pan - same pipeline, same dead time */

/* ---- Anti-ring watchdog: the backstop that stops the amplitude growing -----
   The damping above should keep the oscillation from starting. This catches the case
   where it starts anyway (slower laptop, bigger/closer target, a servo fighting more
   friction than when the gains were set) and guarantees the swings decay instead of
   building until the target is lost. It looks for the one pattern only a limit cycle
   produces: the error crossing zero FAST, repeatedly, with peaks that are not shrinking.
   The half-period gate is what keeps a genuinely moving target safe. A delay-driven ring
   on this rig swings with a period near 4 x (latency + servo lag), i.e. half-cycles
   around half a second, while a person pacing across the frame takes several seconds per
   pass - so real tracking is never mistaken for ringing and never loses its gain. */
#define RING_HALF_PERIOD_MS   900U   /* crossings farther apart than this are a real target */
#define RING_PEAK_DECAY_OK    0.70f  /* a healthy overshoot decays; >70% of the last peak rings */
/* Per-axis, because the axes cannot afford the same amount of patience. Pan can ring for
   three half-cycles and still have the target in frame - it has 1600us of range to play
   with. Tilt has ~150us upward, so three half-cycles is already off the top of the frame;
   it gets to trip one half-cycle sooner. */
#define PAN_RING_TRIP         3U     /* non-decaying fast half-cycles tolerated before backing off */
#define TILT_RING_TRIP        2U     /* less room to spare, so less patience */
#define RING_GAIN_CUT         0.60f  /* multiply that axis's gain by this on a trip */
#define RING_GAIN_MIN         0.35f  /* never cut below this - it must still be able to track */
#define RING_GAIN_RECOVER     0.010f /* handed back per settled sample: ~4 s floor -> 1.0 */
#define CONTROL_PERIOD_MS     20U    /* maximum control rate; a step needs a FRESH sample too */
#define FAILSAFE_TIMEOUT_MS   250U   /* older than this -> freeze in place (never re-centre) */

/* ---- Build-time switches -------------------------------------------------- */
#define MPU6050_POLLING_ENABLED 0    /* 0 while validating tracking: the blocking I2C read sat
                                        directly in the control path.  Code is preserved intact. */
#define SERVO_SIGN_TEST       0      /* 1 = open-loop direction test at boot, vision ignored */
#define TRACKING_DEBUG        1      /* 1 = one compact non-blocking status line per second - ON to diagnose "nothing moves" */

/* ---- Temporary drivetrain bring-up test mode -------------------------------
   1 = run a one-shot forward/backward/left/right exercise at boot (through
   the real motors_*() helpers below), then halt forever. Set back to 0 to
   return to normal pan/tilt tracking - this must never be left on for real
   operation, and none of its HAL_Delay() calls compile in when it is off.
   The motors_*() drivetrain API itself is NOT gated by this flag: it is the
   permanent abstraction vision will drive later. */
#define MOTOR_HARDWARE_TEST   0

/* L9110S channel A ("A-1A"/"A-1B") drives one wheel, channel B ("B-1A"/"B-2A")
   drives the other. Wiring per the current harness:
     A-1A = PB10 (D6)   A-1B = PB5 (D4)
     B-1A = PA9  (D8)   B-2A = PA8  (D7)
   Both pins already come up as GPIO_MODE_OUTPUT_PP from MX_GPIO_Init(). Both
   channels are individually confirmed working: driving a channel's pin_1
   HIGH / pin_2 LOW spins that channel's motor in its "forward" direction. */
#define MOTOR_A_PORT_1        GPIOB
#define MOTOR_A_PIN_1         GPIO_PIN_10   /* A-1A */
#define MOTOR_A_PORT_2        GPIOB
#define MOTOR_A_PIN_2         GPIO_PIN_5    /* A-1B */
#define MOTOR_B_PORT_1        GPIOA
#define MOTOR_B_PIN_1         GPIO_PIN_9    /* B-1A */
#define MOTOR_B_PORT_2        GPIOA
#define MOTOR_B_PIN_2         GPIO_PIN_8    /* B-2A */

/* -------- Physical chassis mapping: everything wiring-specific lives here --
   Two separate physical facts, fixed here and nowhere else, so
   motors_forward()/backward()/turn_left()/turn_right() never need to know
   about channel labels or chassis orientation:
     1) which electrical channel (A or B) is mounted on which side, and
     2) whether that side's "electrical forward" is actually backward once
        mounted (a wheel remounted facing the other way spins the "wrong"
        way for the same IN1/IN2 pattern).
   Run the MOTOR_HARDWARE_TEST sequence below and watch the car: if it drives
   backward when motors_forward() runs, or spins in place instead of driving
   straight, fix the flags here - never the helper functions. */
#define LEFT_MOTOR_IS_MOTOR_A 1      /* 0 if Motor B is physically the left wheel */
#define MOTOR_LEFT_REVERSED   0      /* 1 if the left wheel spins backward when driven forward */
#define MOTOR_RIGHT_REVERSED  0      /* 1 if the right wheel spins backward when driven forward */

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
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
/* ---- Vision link, written by the USART2 RX ISR ----------------------------- */
static uint8_t          rx_byte;
static uart_rx_state_t  uart_rx_state = WAIT_AA;
static uint8_t          uart_rx_length;
static uint8_t          uart_rx_index;
static uint8_t          uart_rx_checksum;
static uint8_t          uart_rx_payload[4];

volatile int16_t  target_err_x;
volatile int16_t  target_err_y;
volatile uint8_t  new_target_data;
volatile uint32_t last_valid_packet_tick;
volatile uint32_t valid_frames_received;
volatile uint32_t bad_checksum_count;
volatile uint32_t uart_error_count;      /* overrun/framing/noise events recovered from */

/* ---- Servo state: the single source of truth for both channels ------------- */
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
static uint32_t last_packet_tick;        /* arrival tick of the previously APPLIED sample */
static uint8_t  tracking_ready;          /* 0 until the boot homing sequence has finished */
static uint8_t  comm_lost;               /* 1 while the vision link is stale */

/* ---- Loop scheduling ------------------------------------------------------- */
static uint32_t last_control_tick;

/* ---- Debug / telemetry (never in the control path) ------------------------- */
static char     tx_buffer[160];  /* sized for the widest TRACKING_DEBUG line below */
#if TRACKING_DEBUG
static uint32_t last_report_tick;
static uint32_t pps_window_tick;
static uint32_t pps_window_start;
static uint32_t packets_per_second;
#endif

#if MPU6050_POLLING_ENABLED
static uint8_t  mpu6050_ready;
static uint8_t  mpu6050_data[14];
static uint32_t last_imu_tick;
static uint32_t last_mpu_led_tick;
#endif
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM3_Init(void);
/* USER CODE BEGIN PFP */
static float   clamp_f(float value, float low, float high);
static float   deadzone_f(float err, float band);
static float   abs_f(float value);
static void    axis_ctl_init(axis_ctl_t *axis, uint32_t now);
static void    axis_ctl_forget(axis_ctl_t *axis, uint32_t now);
static void    axis_ring_watch(axis_ctl_t *axis, float err, uint8_t trip, uint32_t now);
static float   axis_apply(const axis_cfg_t *cfg, axis_ctl_t *axis, float filtered_err,
                          float pulse_us, float rate_scale, uint32_t now);
static uint8_t vision_take_sample(int16_t *err_x, int16_t *err_y, uint32_t *packet_tick);
static void    tracking_update(uint32_t now);
static void    uart_rx_keepalive(void);
static void    motors_stop(void);
static void    motors_forward(void);
static void    motors_backward(void);
static void    motors_turn_left(void);
static void    motors_turn_right(void);
#if MOTOR_HARDWARE_TEST
static void    motor_hardware_test_run(void);
#endif
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/* Blocking print. Boot-time / test-mode only - never called from the control path. */
static void debug_send_blocking(int length)
{
  if (length > 0)
  {
    (void)HAL_UART_Transmit(&huart2, (uint8_t *)tx_buffer, (uint16_t)length, 100);
  }
}

#if TRACKING_DEBUG
/* Non-blocking print for the live loop. Drops the line if the previous one is still
   going out, so telemetry can never stall the controller. */
static void debug_send_async(int length)
{
  if ((length > 0) && (huart2.gState == HAL_UART_STATE_READY))
  {
    (void)HAL_UART_Transmit_IT(&huart2, (uint8_t *)tx_buffer, (uint16_t)length);
  }
}
#endif

/* Error with the dead-zone SUBTRACTED, not merely gated. A hard gate is discontinuous:
   one pixel outside the band the servo jumps straight to a full gain*err kick, one pixel
   inside it holds. That discontinuity is what converts detector noise into a limit cycle.
   Subtracting instead makes the correction start at zero at the edge and grow smoothly,
   which is also exactly the desired feel: far -> keeps moving, near -> eases off, centred
   -> stops. */
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

/* Drop the history but KEEP the gain the watchdog settled on. Called after a comms
   dropout: any move commanded before the gap is long since visible, so it must not be
   subtracted again, and the gap itself is not a half-cycle so it must not be judged as
   one. The gain is kept deliberately - if the link stuttered mid-oscillation, handing
   full gain straight back would restart exactly the swing we just damped. */
static void axis_ctl_forget(axis_ctl_t *axis, uint32_t now)
{
  float keep_gain = axis->gain_scale;

  axis_ctl_init(axis, now);
  axis->gain_scale = keep_gain;
}

/* Watch one axis for a delay-driven limit cycle and trim its gain if it finds one.
   Called once per FRESH sample with the dead-zoned error. A ring is: the error changes
   sides quickly (faster than any real target moves) and the new peak is no smaller than
   the last one. Three of those in a row and the axis loses 40% of its gain; the rule
   repeats, so a ring that survives one cut gets cut again until it dies. */
static void axis_ring_watch(axis_ctl_t *axis, float err, uint8_t trip, uint32_t now)
{
  int8_t sign = (err > 0.0f) ? 1 : ((err < 0.0f) ? -1 : 0);
  float  mag  = abs_f(err);

  if (sign == 0)
  {
    /* Inside the dead-zone: settled, so there is nothing to ring about. Forget the
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
    /* Still on the same side of centre: chasing a target, not ringing. Recover, but a
       quarter as fast as when fully settled, so a cut is not undone mid-pursuit. */
    axis->gain_scale = clamp_f(axis->gain_scale + (RING_GAIN_RECOVER * 0.25f),
                               RING_GAIN_MIN, 1.0f);
  }

  axis->err_sign = sign;
}

/* One axis's complete update for one fresh measurement. Returns the new pulse:
       step  = sign * gain * gain_scale * rate * deadzone(err)  <- the proportional push
             - damp * inflight                                   <- minus what is already on its way
       pulse = clamp(pulse + clamp(step, +/-delta_max), travel limits)
   The damping term is the fix for the growing swing: without it the loop keeps
   re-commanding a correction the camera has already made but the stale frame cannot show
   yet, and those repeats ARE the overshoot. */
static float axis_apply(const axis_cfg_t *cfg, axis_ctl_t *axis, float filtered_err,
                        float pulse_us, float rate_scale, uint32_t now)
{
  float err = deadzone_f(filtered_err, cfg->band_px);
  float step;
  float next;

  axis_ring_watch(axis, err, cfg->ring_trip, now);

  step = (cfg->sign * cfg->gain * axis->gain_scale * rate_scale * err)
         - (cfg->damp * axis->inflight_us);
  step = clamp_f(step, -cfg->delta_max_us, cfg->delta_max_us);

  next = clamp_f(pulse_us + step, cfg->pulse_min_us, cfg->pulse_max_us);

  /* Book the motion that was ACTUALLY applied - after the per-step cap AND the travel
     limits - never the motion that was asked for. Against a limit nothing moves, so
     nothing may accumulate: that is this loop's anti-windup. */
  axis->inflight_us += (next - pulse_us);

  return next;
}

/* Atomically take the newest measurement. Returns 1 only if the ISR has produced a
   packet since the previous call. Reading err_x/err_y outside a critical section could
   pair the x of one frame with the y of the next. */
static uint8_t vision_take_sample(int16_t *err_x, int16_t *err_y, uint32_t *packet_tick)
{
  uint32_t primask = __get_PRIMASK();
  uint8_t  fresh;

  __disable_irq();
  fresh        = new_target_data;
  *err_x       = target_err_x;
  *err_y       = target_err_y;
  *packet_tick = last_valid_packet_tick;
  new_target_data = 0U;
  __set_PRIMASK(primask);

  return fresh;
}

/* ---------------------------------------------------------------------------
 *  THE live pan/tilt controller. This is the only code that writes CH1/CH2
 *  once tracking is running, and the only writer of pan_pulse_us/tilt_pulse_us.
 *
 *  Incremental law, one bounded step per FRESH measurement, per axis:
 *      step  = SIGN * GAIN * gain_scale * rate_scale * deadzone(filtered_err)
 *            - DAMP * inflight_us
 *      pulse = clamp(pulse + clamp(step, +/-DELTA_MAX), travel limits)
 *
 *  rate_scale keeps the commanded velocity independent of the vision frame rate,
 *  inflight_us stops the loop re-commanding a move the camera has already made but
 *  the stale frame cannot show yet, and gain_scale is the anti-ring watchdog's
 *  automatic back-off. Together they are what keeps the swing from growing; see the
 *  notes at VISION_LATENCY_MS.
 * ------------------------------------------------------------------------- */
static void tracking_update(uint32_t now)
{
  int16_t  raw_x = 0;
  int16_t  raw_y = 0;
  uint32_t packet_tick = 0U;
  uint8_t  fresh;
  float    dt_ms;
  float    rate_scale;
  float    decay;

  fresh = vision_take_sample(&raw_x, &raw_y, &packet_tick);

  if (!tracking_ready)
  {
    return;                       /* still homing - discard anything that arrives */
  }

  /* Fail-safe: the vision link has gone quiet. Hold the current position exactly
     where it is. Deliberately NOT a return-to-centre. */
  if ((now - packet_tick) > FAILSAFE_TIMEOUT_MS)
  {
    comm_lost = 1U;
    return;
  }

  /* No new information this tick. Re-applying the previous error would integrate
     the same measurement several times per frame, multiplying the effective loop
     gain by (control rate / vision frame rate). That is what made the camera
     overshoot and chase past the target. So: no fresh sample, no movement. */
  if (!fresh)
  {
    return;
  }

  if (comm_lost)
  {
    /* First sample after a dropout: seed the filter rather than blending in an
       arbitrarily old error, and drop the damping/ring history for the same reason. */
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

  /* Time this step actually covers. Measured at the PACKET tick, not at the 20 ms loop
     tick, so it is the real frame interval even when a frame lands between two loop
     passes or two frames land inside one. */
  dt_ms            = (float)(packet_tick - last_packet_tick);
  last_packet_tick = packet_tick;
  rate_scale       = clamp_f(dt_ms / VISION_NOMINAL_FRAME_MS, RATE_SCALE_MIN, RATE_SCALE_MAX);

  /* Age out the in-flight move: whatever was commanded VISION_LATENCY_MS ago is in the
     picture by now, so it must stop being subtracted. */
  decay = clamp_f(1.0f - (dt_ms / VISION_LATENCY_MS), 0.0f, 1.0f);
  pan_ctl.inflight_us  *= decay;
  tilt_ctl.inflight_us *= decay;

  pan_pulse_us  = axis_apply(&pan_cfg,  &pan_ctl,  filtered_err_x, pan_pulse_us,
                             rate_scale, now);
  tilt_pulse_us = axis_apply(&tilt_cfg, &tilt_ctl, filtered_err_y, tilt_pulse_us,
                             rate_scale, now);

  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, (uint32_t)pan_pulse_us);
  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, (uint32_t)tilt_pulse_us);
}

/* On an overrun the HAL aborts reception (UART_EndRxTransfer clears RXNEIE) and calls
   the error callback. With no callback implemented, RX was never re-armed and the
   vision link died permanently after a single overrun. HAL_UART_ErrorCallback below
   handles it; this is the belt-and-braces check for any path that escapes it. */
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

/* Simple two-wire L9110S channel: HIGH/LOW only, no PWM. One motor per channel.
   This is the permanent drivetrain abstraction - vision will call the
   motors_*() functions below directly once connected. */
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

/* These only ever reference the MOTOR_LEFT_.. / MOTOR_RIGHT_.. mapping -
   channel labels and chassis orientation are fully hidden behind the
   mapping in USER CODE BEGIN PD, so higher-level (and future vision) code
   never needs to know which electrical channel is which wheel or which way
   it is mounted. */
static void motors_stop(void)
{
  motor_drive(MOTOR_LEFT_PORT_1,  MOTOR_LEFT_PIN_1,  MOTOR_LEFT_PORT_2,  MOTOR_LEFT_PIN_2,
             MOTOR_DIR_STOP, MOTOR_LEFT_REVERSED);
  motor_drive(MOTOR_RIGHT_PORT_1, MOTOR_RIGHT_PIN_1, MOTOR_RIGHT_PORT_2, MOTOR_RIGHT_PIN_2,
             MOTOR_DIR_STOP, MOTOR_RIGHT_REVERSED);
}

static void motors_forward(void)
{
  motor_drive(MOTOR_LEFT_PORT_1,  MOTOR_LEFT_PIN_1,  MOTOR_LEFT_PORT_2,  MOTOR_LEFT_PIN_2,
             MOTOR_DIR_FWD, MOTOR_LEFT_REVERSED);
  motor_drive(MOTOR_RIGHT_PORT_1, MOTOR_RIGHT_PIN_1, MOTOR_RIGHT_PORT_2, MOTOR_RIGHT_PIN_2,
             MOTOR_DIR_FWD, MOTOR_RIGHT_REVERSED);
}

static void motors_backward(void)
{
  motor_drive(MOTOR_LEFT_PORT_1,  MOTOR_LEFT_PIN_1,  MOTOR_LEFT_PORT_2,  MOTOR_LEFT_PIN_2,
             MOTOR_DIR_BWD, MOTOR_LEFT_REVERSED);
  motor_drive(MOTOR_RIGHT_PORT_1, MOTOR_RIGHT_PIN_1, MOTOR_RIGHT_PORT_2, MOTOR_RIGHT_PIN_2,
             MOTOR_DIR_BWD, MOTOR_RIGHT_REVERSED);
}

/* Pivot left: left wheel backward, right wheel forward. */
static void motors_turn_left(void)
{
  motor_drive(MOTOR_LEFT_PORT_1,  MOTOR_LEFT_PIN_1,  MOTOR_LEFT_PORT_2,  MOTOR_LEFT_PIN_2,
             MOTOR_DIR_BWD, MOTOR_LEFT_REVERSED);
  motor_drive(MOTOR_RIGHT_PORT_1, MOTOR_RIGHT_PIN_1, MOTOR_RIGHT_PORT_2, MOTOR_RIGHT_PIN_2,
             MOTOR_DIR_FWD, MOTOR_RIGHT_REVERSED);
}

/* Pivot right: left wheel forward, right wheel backward. */
static void motors_turn_right(void)
{
  motor_drive(MOTOR_LEFT_PORT_1,  MOTOR_LEFT_PIN_1,  MOTOR_LEFT_PORT_2,  MOTOR_LEFT_PIN_2,
             MOTOR_DIR_FWD, MOTOR_LEFT_REVERSED);
  motor_drive(MOTOR_RIGHT_PORT_1, MOTOR_RIGHT_PIN_1, MOTOR_RIGHT_PORT_2, MOTOR_RIGHT_PIN_2,
             MOTOR_DIR_BWD, MOTOR_RIGHT_REVERSED);
}

#if MOTOR_HARDWARE_TEST
/* One-shot drivetrain bring-up test: exercises the motors_*() helpers
   themselves (not raw channels) so the FUNCTION NAMES can be checked against
   the car's actual physical behavior. HAL_Delay() is only ever used here -
   never in the tracking loop - and this function halts forever afterward, so
   it cannot run alongside normal pan/tilt/vision operation. */
static void motor_hardware_test_run(void)
{
  motors_forward();     HAL_Delay(1000);
  motors_stop();        HAL_Delay(1000);

  motors_backward();    HAL_Delay(1000);
  motors_stop();        HAL_Delay(1000);

  motors_turn_left();   HAL_Delay(1000);
  motors_stop();        HAL_Delay(1000);

  motors_turn_right();  HAL_Delay(1000);
  motors_stop();

  while (1)
  {
    /* Stop permanently: this is a one-shot hardware test, not normal operation. */
  }
}
#endif /* MOTOR_HARDWARE_TEST */
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

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
  /* USER CODE BEGIN 2 */
  /* Report the reset cause once so a brownout/watchdog reset (e.g. from a stalled servo at its
     end-of-travel limit) is visible in the serial log instead of looking like a tracking reversal. */
  debug_send_blocking(snprintf(tx_buffer, sizeof(tx_buffer),
                               "BOOT reset POR=%d PIN=%d SFT=%d IWDG=%d WWDG=%d LPWR=%d\r\n",
                               __HAL_RCC_GET_FLAG(RCC_FLAG_PORRST) ? 1 : 0,
                               __HAL_RCC_GET_FLAG(RCC_FLAG_PINRST) ? 1 : 0,
                               __HAL_RCC_GET_FLAG(RCC_FLAG_SFTRST) ? 1 : 0,
                               __HAL_RCC_GET_FLAG(RCC_FLAG_IWDGRST) ? 1 : 0,
                               __HAL_RCC_GET_FLAG(RCC_FLAG_WWDGRST) ? 1 : 0,
                               __HAL_RCC_GET_FLAG(RCC_FLAG_LPWRRST) ? 1 : 0));
  __HAL_RCC_CLEAR_RESET_FLAGS();

#if MOTOR_HARDWARE_TEST
  /* Temporary standalone drive-motor bring-up: runs once, then halts forever.
     Pan/tilt homing and tracking below never run while this is enabled. */
  motor_hardware_test_run();
#endif

  /* ======================= HOMING / TRIM (trusted, unchanged) =======================
     Runs exactly once, before the control loop. The blocking delays here are boot-time
     settling only and are never reached again. */
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);

  pan_pulse_us = (float)(PAN_CENTER_US + PAN_TRIM_US);
  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, (uint32_t)pan_pulse_us);

  /* Standardize tilt's starting position: drive to the known bottom limit regardless of where
     it powered up, hold there, then move up by TILT_TRIM_US to reach visual center. */
  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, (uint32_t)TILT_BOTTOM_US);
  HAL_Delay(TILT_BOTTOM_HOLD_MS);
  tilt_pulse_us = clamp_f((float)TILT_BOTTOM_US - (float)TILT_TRIM_US,
                          TILT_PULSE_MIN_US, TILT_PULSE_MAX_US);
  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, (uint32_t)tilt_pulse_us);

  /* Hold neutral and discard any stray/garbage packets received during power-up settling. */
  HAL_Delay(SERVO_HOME_SETTLE_MS);
  /* ======================= END HOMING / TRIM ======================================= */

  debug_send_blocking(snprintf(tx_buffer, sizeof(tx_buffer),
                               "HOME pan=%u tilt=%u (tiltMin=%u tiltMax=%u upRoom=%d dnRoom=%d)\r\n",
                               (unsigned)pan_pulse_us, (unsigned)tilt_pulse_us,
                               (unsigned)TILT_PULSE_MIN_US, (unsigned)TILT_PULSE_MAX_US,
                               (int)(tilt_pulse_us - TILT_PULSE_MIN_US),
                               (int)(TILT_PULSE_MAX_US - tilt_pulse_us)));

#if SERVO_SIGN_TEST
  /* ---- Open-loop direction test. Vision is ignored; watch the camera. --------------
     Each move is announced first, then held for 1.5 s. Write down which way the camera
     actually turns, then set PAN_SIGN / TILT_SIGN per the report. */
  {
    const float pan_home  = pan_pulse_us;
    const float tilt_home = tilt_pulse_us;
    float       target;

    debug_send_blocking(snprintf(tx_buffer, sizeof(tx_buffer), "SIGNTEST pan +300us\r\n"));
    target = clamp_f(pan_home + 300.0f, PAN_PULSE_MIN_US, PAN_PULSE_MAX_US);
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, (uint32_t)target);
    HAL_Delay(1500);

    debug_send_blocking(snprintf(tx_buffer, sizeof(tx_buffer), "SIGNTEST pan home\r\n"));
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, (uint32_t)pan_home);
    HAL_Delay(1500);

    debug_send_blocking(snprintf(tx_buffer, sizeof(tx_buffer), "SIGNTEST pan -300us\r\n"));
    target = clamp_f(pan_home - 300.0f, PAN_PULSE_MIN_US, PAN_PULSE_MAX_US);
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, (uint32_t)target);
    HAL_Delay(1500);

    debug_send_blocking(snprintf(tx_buffer, sizeof(tx_buffer), "SIGNTEST pan home\r\n"));
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, (uint32_t)pan_home);
    HAL_Delay(1500);

    /* Tilt: only the directions the current neutral actually has room for are driven. */
    target = clamp_f(tilt_home + 300.0f, TILT_PULSE_MIN_US, TILT_PULSE_MAX_US);
    debug_send_blocking(snprintf(tx_buffer, sizeof(tx_buffer),
                                 "SIGNTEST tilt %u -> %u us\r\n",
                                 (unsigned)tilt_home, (unsigned)target));
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, (uint32_t)target);
    HAL_Delay(1500);

    debug_send_blocking(snprintf(tx_buffer, sizeof(tx_buffer), "SIGNTEST tilt home\r\n"));
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, (uint32_t)tilt_home);
    HAL_Delay(1500);

    target = clamp_f(tilt_home - 300.0f, TILT_PULSE_MIN_US, TILT_PULSE_MAX_US);
    debug_send_blocking(snprintf(tx_buffer, sizeof(tx_buffer),
                                 "SIGNTEST tilt %u -> %u us (no travel if equal)\r\n",
                                 (unsigned)tilt_home, (unsigned)target));
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, (uint32_t)target);
    HAL_Delay(1500);

    debug_send_blocking(snprintf(tx_buffer, sizeof(tx_buffer), "SIGNTEST done\r\n"));
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, (uint32_t)tilt_home);
    HAL_Delay(500);
  }
#endif

#if MPU6050_POLLING_ENABLED
  {
    uint8_t who_am_i = 0;
    uint8_t wake_command = 0x00;

    if (HAL_I2C_Mem_Read(&hi2c1, MPU6050_ADDRESS, MPU6050_WHO_AM_I,
                         I2C_MEMADD_SIZE_8BIT, &who_am_i, 1, 100) == HAL_OK &&
        who_am_i == 0x68 &&
        HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDRESS, MPU6050_PWR_MGMT_1,
                          I2C_MEMADD_SIZE_8BIT, &wake_command, 1, 100) == HAL_OK)
    {
      mpu6050_ready = 1;
      HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);
    }
    else
    {
      HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);
    }
  }
#endif

  /* Arm tracking last, from the calibrated position, with a clean slate. */
  filtered_err_x = 0.0f;
  filtered_err_y = 0.0f;
  axis_ctl_init(&pan_ctl,  HAL_GetTick());
  axis_ctl_init(&tilt_ctl, HAL_GetTick());
  last_packet_tick = HAL_GetTick();
  comm_lost      = 1U;              /* nothing moves until a real packet arrives */
  last_control_tick = HAL_GetTick();
  last_valid_packet_tick = HAL_GetTick() - (FAILSAFE_TIMEOUT_MS + 1U);
  new_target_data = 0U;
  uart_rx_state   = WAIT_AA;
  tracking_ready  = 1U;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */
    uint32_t now = HAL_GetTick();

    /* Keep the vision link alive: a single UART overrun used to kill reception forever. */
    uart_rx_keepalive();

    /* -------- The one and only periodic control loop, fixed 20 ms, non-blocking -------- */
    if ((now - last_control_tick) >= CONTROL_PERIOD_MS)
    {
      last_control_tick = now;
      tracking_update(now);
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
      /* gp/gt are the watchdog's gain scales in percent: a run of values below 100 is
         the loop telling you it caught itself ringing on that axis. ifp is the pan
         in-flight move in us - the correction already sent that the frame cannot show
         yet, ift the same for tilt. up= is tilt's remaining UPWARD travel in us: if that
         sits at 0 while ey stays negative, tilt is not ringing at all, it is pegged
         against its top stop and physically cannot centre the target - that needs travel
         (TILT_TRIM_US / the bracket), not tuning. The two look alike on camera and have
         opposite fixes, so check this field before touching a gain. */
      debug_send_async(snprintf(tx_buffer, sizeof(tx_buffer),
                                "T pps=%lu age=%lu ex=%d ey=%d pan=%u tilt=%u gp=%u gt=%u ifp=%d ift=%d up=%d bad=%lu err=%lu\r\n",
                                (unsigned long)packets_per_second,
                                (unsigned long)(now - last_valid_packet_tick),
                                (int)filtered_err_x, (int)filtered_err_y,
                                (unsigned)pan_pulse_us, (unsigned)tilt_pulse_us,
                                (unsigned)(pan_ctl.gain_scale * 100.0f),
                                (unsigned)(tilt_ctl.gain_scale * 100.0f),
                                (int)pan_ctl.inflight_us,
                                (int)tilt_ctl.inflight_us,
                                (int)(tilt_pulse_us - TILT_PULSE_MIN_US),
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
        else
        {
          uart_rx_state = WAIT_AA;
        }
        break;

      case WAIT_LEN:
        if (rx_byte == 4U)
        {
          uart_rx_length   = rx_byte;
          uart_rx_index    = 0U;
          uart_rx_checksum = rx_byte;       /* checksum covers LEN + payload */
          uart_rx_state    = WAIT_PAYLOAD;
        }
        else
        {
          uart_rx_state = (rx_byte == 0xAA) ? WAIT_55 : WAIT_AA;
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
          HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);   /* LD2 flickers = vision link alive */
        }
        else
        {
          bad_checksum_count++;
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

/* Without this, one overrun (ORE) permanently ended reception: the HAL aborts the Rx
   transfer, clears RXNEIE and calls this weak callback, which used to do nothing. The
   servos then froze on stale data with no way to recover short of a reset. */
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
