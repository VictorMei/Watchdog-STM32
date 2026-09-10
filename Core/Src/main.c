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

/* Disable the temporary human-readable MPU6050 stream during binary testing. */
#define HAL_UART_Transmit(...) do { } while (0)

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
#define MPU6050_ADDRESS       (0x68 << 1)
#define MPU6050_WHO_AM_I      0x75
#define MPU6050_PWR_MGMT_1    0x6B
#define MPU6050_ACCEL_XOUT_H  0x3B
#define PAN_KP                2.0f

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
static uint8_t mpu6050_ready = 0;
static uint8_t mpu6050_data[14];
static char uart_buffer[96];
static uint8_t rx_byte;
static uart_rx_state_t uart_rx_state = WAIT_AA;
static uint8_t uart_rx_length;
static uint8_t uart_rx_index;
static uint8_t uart_rx_checksum;
static uint8_t uart_rx_payload[4];
static uint32_t last_uart_debug_tick;
static uint16_t pan_pulse_us = 1500U;
static float filtered_err_x = 0.0f;
static uint32_t last_pan_update_tick;
static uint32_t last_imu_tick;

volatile int16_t target_err_x;
volatile int16_t target_err_y;
volatile uint8_t new_target_data = 0;
volatile uint32_t last_valid_packet_tick = 0;
volatile uint32_t uart_bytes_received = 0;
volatile uint32_t valid_frames_received = 0;
volatile uint32_t bad_checksum_count = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM3_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

#undef HAL_UART_Transmit
static HAL_StatusTypeDef uart_debug_transmit(UART_HandleTypeDef *huart,
                                              uint8_t *data,
                                              uint16_t size,
                                              uint32_t timeout)
{
  return HAL_UART_Transmit(huart, data, size, timeout);
}
#define HAL_UART_Transmit(...) do { } while (0)

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

  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, pan_pulse_us);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */
    if (mpu6050_ready && ((HAL_GetTick() - last_imu_tick) >= 100U))
    {
      last_imu_tick = HAL_GetTick();
      if (HAL_I2C_Mem_Read(&hi2c1, MPU6050_ADDRESS, MPU6050_ACCEL_XOUT_H,
                           I2C_MEMADD_SIZE_8BIT, mpu6050_data,
                           sizeof(mpu6050_data), 100) == HAL_OK)
      {
        int16_t accel_x = (int16_t)((mpu6050_data[0] << 8) | mpu6050_data[1]);
        int16_t accel_y = (int16_t)((mpu6050_data[2] << 8) | mpu6050_data[3]);
        int16_t accel_z = (int16_t)((mpu6050_data[4] << 8) | mpu6050_data[5]);
        int16_t gyro_x = (int16_t)((mpu6050_data[8] << 8) | mpu6050_data[9]);
        int16_t gyro_y = (int16_t)((mpu6050_data[10] << 8) | mpu6050_data[11]);
        int16_t gyro_z = (int16_t)((mpu6050_data[12] << 8) | mpu6050_data[13]);
        int length = snprintf(uart_buffer, sizeof(uart_buffer),
                              "IMU AX:%d AY:%d AZ:%d GX:%d GY:%d GZ:%d\r\n",
                              accel_x, accel_y, accel_z,
                              gyro_x, gyro_y, gyro_z);

        if (length > 0)
        {
          uart_debug_transmit(&huart2, (uint8_t *)uart_buffer,
                              (uint16_t)length, 100);
        }
      }
      else
      {
        mpu6050_ready = 0;
      }
      HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);
    }
    else if (!mpu6050_ready)
    {
      HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
      HAL_Delay(150);
    }

    if (new_target_data == 1)
    {
      int16_t received_err_x = target_err_x;
      int16_t received_err_y = target_err_y;
      int length;

      new_target_data = 0;
      filtered_err_x = (0.4f * filtered_err_x) + (0.6f * (float)received_err_x);

      length = snprintf(uart_buffer, sizeof(uart_buffer),
                        "RX err_x=%d err_y=%d\r\n",
                        received_err_x, received_err_y);
      if (length > 0)
      {
        uart_debug_transmit(&huart2, (uint8_t *)uart_buffer,
                            (uint16_t)length, 100);
      }
    }

    if ((HAL_GetTick() - last_pan_update_tick) >= 20U &&
        (HAL_GetTick() - last_valid_packet_tick) <= 500U)
    {
      int16_t delta_us = 0;
      last_pan_update_tick = HAL_GetTick();

      if (filtered_err_x > 15.0f || filtered_err_x < -15.0f)
      {
        delta_us = (int16_t)(PAN_KP * filtered_err_x);
        if (delta_us > 80)
        {
          delta_us = 80;
        }
        else if (delta_us < -80)
        {
          delta_us = -80;
        }

        pan_pulse_us += delta_us;
        if (pan_pulse_us < 700U)
        {
          pan_pulse_us = 700U;
        }
        else if (pan_pulse_us > 2300U)
        {
          pan_pulse_us = 2300U;
        }
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, pan_pulse_us);
      }
    }

    if ((HAL_GetTick() - last_uart_debug_tick) >= 1000U)
    {
      int length = snprintf(uart_buffer, sizeof(uart_buffer),
                            "DBG bytes=%lu valid=%lu bad=%lu\r\n",
                            (unsigned long)uart_bytes_received,
                            (unsigned long)valid_frames_received,
                            (unsigned long)bad_checksum_count);
      last_uart_debug_tick = HAL_GetTick();
      if (length > 0)
      {
        uart_debug_transmit(&huart2, (uint8_t *)uart_buffer,
                            (uint16_t)length, 100);
      }
    }

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
    uart_bytes_received++;
    switch (uart_rx_state)
    {
      case WAIT_AA:
        if (rx_byte == 0xAA)
        {
          uart_rx_state = WAIT_55;
        }
        break;

      case WAIT_55:
        uart_rx_state = (rx_byte == 0x55) ? WAIT_LEN : WAIT_AA;
        break;

      case WAIT_LEN:
        uart_rx_length = rx_byte;
        uart_rx_index = 0;
        uart_rx_checksum = rx_byte;
        uart_rx_state = (uart_rx_length == 4) ? WAIT_PAYLOAD : WAIT_AA;
        break;

      case WAIT_PAYLOAD:
        uart_rx_payload[uart_rx_index++] = rx_byte;
        uart_rx_checksum ^= rx_byte;
        if (uart_rx_index == uart_rx_length)
        {
          uart_rx_state = WAIT_CHECKSUM;
        }
        break;

      case WAIT_CHECKSUM:
        if (rx_byte == uart_rx_checksum)
        {
          target_err_x = (int16_t)((uint16_t)uart_rx_payload[0] |
                                   ((uint16_t)uart_rx_payload[1] << 8));
          target_err_y = (int16_t)((uint16_t)uart_rx_payload[2] |
                                   ((uint16_t)uart_rx_payload[3] << 8));
          last_valid_packet_tick = HAL_GetTick();
          new_target_data = 1;
          valid_frames_received++;
          HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
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

    HAL_UART_Receive_IT(&huart2, &rx_byte, 1);
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
