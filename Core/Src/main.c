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
#define TILT_PULSE_MIN_US     400.0f  /* NOTE: equals the current tilt neutral - see TILT_TRIM_US */
#define TILT_PULSE_MAX_US     2200.0f

/* ---- Calibrated home position / trim.  TRUSTED - do not "tidy" these. ------
   Pan  : neutral = PAN_CENTER_US + PAN_TRIM_US.
   Tilt : driven to the known bottom stop (TILT_BOTTOM_US), held, then moved UP
          by TILT_TRIM_US.  That makes the boot position repeatable regardless
          of where the horn powered up.  Applied ONCE at boot and never again;
          the tracking loop integrates away from it and never pulls back to it.
   WARNING: 2200 - 1800 = 400us, which is exactly TILT_PULSE_MIN_US, so tilt
          currently has ZERO travel left in the decreasing-pulse direction.
          See the tilt range test in the report before trusting tilt tracking. */
#define PAN_CENTER_US         1500U
#define PAN_TRIM_US           0
#define TILT_CENTER_US        1500U  /* reference only; tilt neutral is derived from the bottom stop */
#define TILT_BOTTOM_US        TILT_PULSE_MAX_US /* Confirmed: increasing pulse tilts down; this is the down limit */
#define TILT_TRIM_US          1800   /* Distance to move UP from the bottom limit to reach visual centre */
#define TILT_BOTTOM_HOLD_MS   300U   /* Hold at the bottom limit before moving to the trimmed centre */
#define SERVO_HOME_SETTLE_MS  200U   /* Hold neutral before any tracking command is honoured */

/* ---- Direction calibration ------------------------------------------------
   These are the ONLY knobs that decide which way the camera turns.
   Flip a sign to +/-1.0f if the camera runs away from the target on that axis.
   Required end behaviour:
       target right (err_x>0) -> camera pans right
       target below (err_y>0) -> camera tilts down                            */
#define PAN_SIGN              1.0f   /* +1 assumes increasing pulse pans RIGHT */
#define TILT_SIGN             1.0f   /* +1 assumes increasing pulse tilts DOWN (see TILT_BOTTOM_US) */

/* ---- Incremental proportional controller (no integral, no derivative) ------ */
#define PAN_GAIN              0.20f  /* us of pulse per pixel of error, per accepted sample */
#define TILT_GAIN             0.18f
#define PAN_DELTA_MAX_US      60.0f  /* safety cap on one step, so a bad frame cannot lurch */
#define TILT_DELTA_MAX_US     50.0f
#define DEADBAND_PX           20.0f  /* inside this the servo holds still */
#define ERR_FILTER_OLD        0.15f  /* light filter only - heavy filtering is pure added lag */
#define ERR_FILTER_NEW        0.85f
#define CONTROL_PERIOD_MS     20U    /* maximum control rate; a step needs a FRESH sample too */
#define FAILSAFE_TIMEOUT_MS   250U   /* older than this -> freeze in place (never re-centre) */

/* ---- Build-time switches -------------------------------------------------- */
#define MPU6050_POLLING_ENABLED 0    /* 0 while validating tracking: the blocking I2C read sat
                                        directly in the control path.  Code is preserved intact. */
#define SERVO_SIGN_TEST       0      /* 1 = open-loop direction test at boot, vision ignored */
#define TRACKING_DEBUG        0      /* 1 = one compact non-blocking status line per second */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

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
static uint8_t  tracking_ready;          /* 0 until the boot homing sequence has finished */
static uint8_t  comm_lost;               /* 1 while the vision link is stale */

/* ---- Loop scheduling ------------------------------------------------------- */
static uint32_t last_control_tick;

/* ---- Debug / telemetry (never in the control path) ------------------------- */
static char     tx_buffer[128];
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
static void MX_I2C1_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM3_Init(void);
/* USER CODE BEGIN PFP */
static float   clamp_f(float value, float low, float high);
static uint8_t vision_take_sample(int16_t *err_x, int16_t *err_y, uint32_t *packet_tick);
static void    tracking_update(uint32_t now);
static void    uart_rx_keepalive(void);
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
 *  Incremental proportional law, one bounded step per FRESH measurement:
 *      pan_pulse_us  += clamp(PAN_SIGN  * PAN_GAIN  * filtered_err_x)
 *      tilt_pulse_us += clamp(TILT_SIGN * TILT_GAIN * filtered_err_y)
 * ------------------------------------------------------------------------- */
static void tracking_update(uint32_t now)
{
  int16_t  raw_x = 0;
  int16_t  raw_y = 0;
  uint32_t packet_tick = 0U;
  uint8_t  fresh;
  float    next_pan;
  float    next_tilt;
  float    step;

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
       arbitrarily old error. */
    comm_lost      = 0U;
    filtered_err_x = (float)raw_x;
    filtered_err_y = (float)raw_y;
  }
  else
  {
    filtered_err_x = (ERR_FILTER_OLD * filtered_err_x) + (ERR_FILTER_NEW * (float)raw_x);
    filtered_err_y = (ERR_FILTER_OLD * filtered_err_y) + (ERR_FILTER_NEW * (float)raw_y);
  }

  next_pan  = pan_pulse_us;
  next_tilt = tilt_pulse_us;

  if ((filtered_err_x > DEADBAND_PX) || (filtered_err_x < -DEADBAND_PX))
  {
    step = clamp_f(PAN_SIGN * PAN_GAIN * filtered_err_x,
                   -PAN_DELTA_MAX_US, PAN_DELTA_MAX_US);
    next_pan += step;
  }

  if ((filtered_err_y > DEADBAND_PX) || (filtered_err_y < -DEADBAND_PX))
  {
    step = clamp_f(TILT_SIGN * TILT_GAIN * filtered_err_y,
                   -TILT_DELTA_MAX_US, TILT_DELTA_MAX_US);
    next_tilt += step;
  }

  pan_pulse_us  = clamp_f(next_pan,  PAN_PULSE_MIN_US,  PAN_PULSE_MAX_US);
  tilt_pulse_us = clamp_f(next_tilt, TILT_PULSE_MIN_US, TILT_PULSE_MAX_US);

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
  MX_I2C1_Init();
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

#if MPU6050_POLLING_ENABLED
    /* Preserved for later stabilisation work. NOTE: HAL_I2C_Mem_Read is blocking with a
       100 ms timeout and sits directly in front of the control update, so it is a real
       latency source. Keep MPU6050_POLLING_ENABLED at 0 until tracking is validated. */
    if (mpu6050_ready && ((now - last_imu_tick) >= 100U))
    {
      last_imu_tick = now;
      if (HAL_I2C_Mem_Read(&hi2c1, MPU6050_ADDRESS, MPU6050_ACCEL_XOUT_H,
                           I2C_MEMADD_SIZE_8BIT, mpu6050_data,
                           sizeof(mpu6050_data), 100) != HAL_OK)
      {
        mpu6050_ready = 0;
      }
    }
    else if (!mpu6050_ready)
    {
      /* Non-blocking heartbeat: the previous HAL_Delay(150) here stalled ALL tracking/UART
         processing for 150 ms every pass whenever the IMU wasn't ready. That was a real bug
         and must not come back. */
      if ((now - last_mpu_led_tick) >= 150U)
      {
        last_mpu_led_tick = now;
        HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
      }
    }
#endif

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
      debug_send_async(snprintf(tx_buffer, sizeof(tx_buffer),
                                "T pps=%lu age=%lu ex=%d ey=%d pan=%u tilt=%u bad=%lu err=%lu\r\n",
                                (unsigned long)packets_per_second,
                                (unsigned long)(now - last_valid_packet_tick),
                                (int)filtered_err_x, (int)filtered_err_y,
                                (unsigned)pan_pulse_us, (unsigned)tilt_pulse_us,
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

static void MX_I2C1_Init(void)
{
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;

  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
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
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);

  /*Configure GPIO pin : PA5 */
  GPIO_InitStruct.Pin = GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

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
