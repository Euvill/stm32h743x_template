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
#include <ctype.h>
#include <stdio.h>
#include <string.h>
#include "icm42688p.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/*
 * Runtime print period once the full IMU startup sequence has completed.
 * At that point we print converted values, not the original raw registers.
 */
#define UART_SEND_PERIOD_MS    200U
#define UART_RX_BUFFER_SIZE    128U
#define UART_RX_IDLE_MS        20U

/* According to the INV self-test API:
 * bit0 = gyro pass
 * bit1 = accel pass
 * therefore 0x03 means both sensors passed.
 */
#define IMU_SELF_TEST_PASS_MASK      0x03

/* Fixed-point helpers used only for UART formatting. */
#define FIXED_POINT_3_SCALE          1000UL
#define FIXED_POINT_2_SCALE          100UL

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

SPI_HandleTypeDef hspi3;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
/*
 * Application-level state used only by the demo in main.c.
 *
 * The IMU driver state itself lives inside icm42688p.c; these variables only
 * store:
 * - the latest sample for printing
 * - the latest reported official self-test bias values
 * - UART receive helper state
 * - temporary text buffers
 */
static uint32_t uart_send_last_tick = 0U;
static uint8_t uart_rx_byte = 0U;
static char uart_rx_buffer[UART_RX_BUFFER_SIZE] = {0};
static uint16_t uart_rx_len = 0U;
static uint32_t uart_rx_last_tick = 0U;
static ICM42688P_Data_t imu_data;
static ICM42688P_OfficialBias_t imu_official_bias;
static char uart_tx_buffer[192] = {0};
static char uart_reply_buffer[192] = {0};

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MPU_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_SPI3_Init(void);
/* USER CODE BEGIN PFP */
static void UART_FlushReceivedMessage(void);
static void UART_ProcessReceivedByte(uint8_t byte);
static void UART_SendString(const char *text);
static long UART_ScaleFloat(float value, long scale);
static void UART_FormatFixedPoint(char *buffer, size_t buffer_size, long scaled_value, unsigned long scale, unsigned int decimals);
static int UART_IsEnterCommand(const char *command);
static void UART_WaitForEnterCommand(void);
static void UART_PrintOfficialBiasReport(const ICM42688P_OfficialBias_t *bias);
static void UART_PrintConvertedImuData(const ICM42688P_Data_t *data);
static int IMU_StartupSequence(void);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/*
 * Flush the small UART receive line buffer.
 *
 * This is unrelated to the IMU logic. It is only a simple echo-style helper
 * kept from the original template so incoming UART text can still be observed.
 */
static void UART_FlushReceivedMessage(void)
{
  int reply_len = 0;

  if (uart_rx_len == 0U)
  {
    return;
  }

  reply_len = snprintf(uart_reply_buffer, sizeof(uart_reply_buffer),
                       "receive the data from pc: %s\r\n", uart_rx_buffer);
  if (reply_len > 0)
  {
    HAL_UART_Transmit(&huart1, (uint8_t *)uart_reply_buffer, (uint16_t)reply_len, HAL_MAX_DELAY);
  }

  uart_rx_len = 0U;
  uart_rx_buffer[0] = '\0';
}

/* Push one received byte into the line buffer and flush on CR/LF. */
static void UART_ProcessReceivedByte(uint8_t byte)
{
  if ((byte == '\r') || (byte == '\n'))
  {
    UART_FlushReceivedMessage();
    return;
  }

  if (uart_rx_len < (UART_RX_BUFFER_SIZE - 1U))
  {
    uart_rx_buffer[uart_rx_len++] = (char)byte;
    uart_rx_buffer[uart_rx_len] = '\0';
    uart_rx_last_tick = HAL_GetTick();
  }
  else
  {
    uart_rx_len = 0U;
    uart_rx_buffer[0] = '\0';
  }
}

/*
 * Minimal "send one C string" helper.
 *
 * The goal is to keep later status-printing code readable. Without this helper,
 * startup code would be cluttered with repeated HAL_UART_Transmit calls.
 */
static void UART_SendString(const char *text)
{
  if (text == NULL)
  {
    return;
  }

  HAL_UART_Transmit(&huart1, (uint8_t *)text, (uint16_t)strlen(text), HAL_MAX_DELAY);
}

/*
 * Convert a floating-point value to an integer scaled by `scale`.
 *
 * Example:
 * value = 1.234, scale = 1000 -> returns about 1234
 *
 * We do this so the later UART formatting step can build a predictable
 * fixed-point decimal string without relying on `%f`, which is often avoided in
 * embedded `printf` configurations for size reasons.
 */
static long UART_ScaleFloat(float value, long scale)
{
  if (value >= 0.0f)
  {
    return (long)(value * (float)scale + 0.5f);
  }

  return (long)(value * (float)scale - 0.5f);
}

/*
 * Format an already-scaled integer as a decimal string.
 *
 * Example:
 * scaled_value = -1234, scale = 1000, decimals = 3
 * output       = "-1.234"
 *
 * This helper is used by both bias-report printing and normal runtime data
 * printing so those higher-level functions can stay easy to scan.
 */
static void UART_FormatFixedPoint(char *buffer, size_t buffer_size, long scaled_value, unsigned long scale, unsigned int decimals)
{
  const char *sign = "";
  unsigned long abs_value;
  unsigned long whole;
  unsigned long fraction;

  if ((buffer == NULL) || (buffer_size == 0U) || (scale == 0UL))
  {
    return;
  }

  if (scaled_value < 0L)
  {
    sign = "-";
  }

  abs_value = (scaled_value < 0L) ? (unsigned long)(-scaled_value) : (unsigned long)scaled_value;
  whole = abs_value / scale;
  fraction = abs_value % scale;

  (void)snprintf(buffer, buffer_size, "%s%lu.%0*lu", sign, whole, (int)decimals, fraction);
}

/*
 * Check whether one UART line should be treated as the "start runtime" command.
 *
 * We intentionally accept two user habits:
 * - just press Enter on an empty line
 * - type "enter" and then press Enter
 *
 * That makes the startup gate easy to trigger from common serial tools.
 */
static int UART_IsEnterCommand(const char *command)
{
  static const char expected_command[] = "enter";
  size_t index = 0U;

  if (command == NULL)
  {
    return 0;
  }

  if (command[0] == '\0')
  {
    return 1;
  }

  while (expected_command[index] != '\0')
  {
    if ((char)tolower((unsigned char)command[index]) != expected_command[index])
    {
      return 0;
    }

    index++;
  }

  return (command[index] == '\0') ? 1 : 0;
}

/*
 * Block startup until the operator explicitly allows runtime output to begin.
 *
 * Why wait here instead of immediately entering the main loop:
 * - the self-test report stays on screen long enough for the user to read it
 * - the first stream of 200 ms runtime prints does not immediately scroll the
 *   bias report away
 * - the user can decide exactly when the application starts its normal phase
 *
 * Accepted UART actions:
 * - press Enter on an empty line
 * - type "enter" then press Enter
 */
static void UART_WaitForEnterCommand(void)
{
  char command_buffer[16] = {0};
  uint8_t rx_char = 0U;
  uint8_t command_length = 0U;

  UART_SendString("ICM42688P initialization complete.\r\n");
  UART_SendString("Press Enter, or type 'enter' then press Enter, to start runtime output.\r\n");

  while (1)
  {
    if (HAL_UART_Receive(&huart1, &rx_char, 1U, HAL_MAX_DELAY) != HAL_OK)
    {
      continue;
    }

    if ((rx_char == '\r') || (rx_char == '\n'))
    {
      command_buffer[command_length] = '\0';

      if (UART_IsEnterCommand(command_buffer) != 0)
      {
        UART_SendString("Runtime output started.\r\n");
        return;
      }

      UART_SendString("Unknown command. Press Enter or send 'enter'.\r\n");
      command_length = 0U;
      command_buffer[0] = '\0';
      continue;
    }

    if (command_length < (sizeof(command_buffer) - 1U))
    {
      command_buffer[command_length++] = (char)rx_char;
      command_buffer[command_length] = '\0';
    }
    else
    {
      UART_SendString("Command too long. Press Enter or send 'enter'.\r\n");
      command_length = 0U;
      command_buffer[0] = '\0';
    }
  }
}

/*
 * Print the bias report reported by the official INV self-test path.
 *
 * We print each axis twice:
 * - exact Q16 value returned by inv_icm426xx_get_st_bias()
 * - converted engineering value for fast human inspection
 *
 * This is useful when comparing:
 * - our project log
 * - official driver behavior
 * - future register-level debugging
 */
static void UART_PrintOfficialBiasReport(const ICM42688P_OfficialBias_t *bias)
{
  char acc_x[16];
  char acc_y[16];
  char acc_z[16];
  char gyr_x[16];
  char gyr_y[16];
  char gyr_z[16];

  if (bias == NULL)
  {
    return;
  }

  UART_FormatFixedPoint(acc_x, sizeof(acc_x), UART_ScaleFloat(bias->accel_g[0], (long)FIXED_POINT_3_SCALE), FIXED_POINT_3_SCALE, 3U);
  UART_FormatFixedPoint(acc_y, sizeof(acc_y), UART_ScaleFloat(bias->accel_g[1], (long)FIXED_POINT_3_SCALE), FIXED_POINT_3_SCALE, 3U);
  UART_FormatFixedPoint(acc_z, sizeof(acc_z), UART_ScaleFloat(bias->accel_g[2], (long)FIXED_POINT_3_SCALE), FIXED_POINT_3_SCALE, 3U);
  UART_FormatFixedPoint(gyr_x, sizeof(gyr_x), UART_ScaleFloat(bias->gyro_dps[0], (long)FIXED_POINT_3_SCALE), FIXED_POINT_3_SCALE, 3U);
  UART_FormatFixedPoint(gyr_y, sizeof(gyr_y), UART_ScaleFloat(bias->gyro_dps[1], (long)FIXED_POINT_3_SCALE), FIXED_POINT_3_SCALE, 3U);
  UART_FormatFixedPoint(gyr_z, sizeof(gyr_z), UART_ScaleFloat(bias->gyro_dps[2], (long)FIXED_POINT_3_SCALE), FIXED_POINT_3_SCALE, 3U);

  (void)snprintf(uart_tx_buffer,
                 sizeof(uart_tx_buffer),
                 "Official GYR bias Q16[dps*65536]: %ld,%ld,%ld\r\n",
                 (long)bias->gyro_q16[0],
                 (long)bias->gyro_q16[1],
                 (long)bias->gyro_q16[2]);
  UART_SendString(uart_tx_buffer);

  (void)snprintf(uart_tx_buffer,
                 sizeof(uart_tx_buffer),
                 "Official GYR bias[dps]: %s,%s,%s\r\n",
                 gyr_x,
                 gyr_y,
                 gyr_z);
  UART_SendString(uart_tx_buffer);

  (void)snprintf(uart_tx_buffer,
                 sizeof(uart_tx_buffer),
                 "Official ACC bias Q16[g*65536]: %ld,%ld,%ld\r\n",
                 (long)bias->accel_q16[0],
                 (long)bias->accel_q16[1],
                 (long)bias->accel_q16[2]);
  UART_SendString(uart_tx_buffer);

  (void)snprintf(uart_tx_buffer,
                 sizeof(uart_tx_buffer),
                 "Official ACC bias[g]: %s,%s,%s\r\n",
                 acc_x,
                 acc_y,
                 acc_z);
  UART_SendString(uart_tx_buffer);
}

/*
 * Print the normal periodic IMU output once initialization is complete.
 *
 * At this stage:
 * - self-test has already passed
 * - official bias has already been reported to the user
 * - the user has explicitly pressed Enter to begin the runtime phase
 *
 * So this function only focuses on presentation.
 */
static void UART_PrintConvertedImuData(const ICM42688P_Data_t *data)
{
  char acc_x[16];
  char acc_y[16];
  char acc_z[16];
  char gyr_x[16];
  char gyr_y[16];
  char gyr_z[16];
  char temp_c[16];

  if (data == NULL)
  {
    return;
  }

  UART_FormatFixedPoint(acc_x, sizeof(acc_x), UART_ScaleFloat(data->accel_g[0], (long)FIXED_POINT_3_SCALE), FIXED_POINT_3_SCALE, 3U);
  UART_FormatFixedPoint(acc_y, sizeof(acc_y), UART_ScaleFloat(data->accel_g[1], (long)FIXED_POINT_3_SCALE), FIXED_POINT_3_SCALE, 3U);
  UART_FormatFixedPoint(acc_z, sizeof(acc_z), UART_ScaleFloat(data->accel_g[2], (long)FIXED_POINT_3_SCALE), FIXED_POINT_3_SCALE, 3U);
  UART_FormatFixedPoint(gyr_x, sizeof(gyr_x), UART_ScaleFloat(data->gyro_dps[0], (long)FIXED_POINT_3_SCALE), FIXED_POINT_3_SCALE, 3U);
  UART_FormatFixedPoint(gyr_y, sizeof(gyr_y), UART_ScaleFloat(data->gyro_dps[1], (long)FIXED_POINT_3_SCALE), FIXED_POINT_3_SCALE, 3U);
  UART_FormatFixedPoint(gyr_z, sizeof(gyr_z), UART_ScaleFloat(data->gyro_dps[2], (long)FIXED_POINT_3_SCALE), FIXED_POINT_3_SCALE, 3U);
  UART_FormatFixedPoint(temp_c, sizeof(temp_c), UART_ScaleFloat(data->temperature_c, (long)FIXED_POINT_2_SCALE), FIXED_POINT_2_SCALE, 2U);

  (void)snprintf(uart_tx_buffer,
                 sizeof(uart_tx_buffer),
                 "ACC[g]: %s,%s,%s GYR[dps]: %s,%s,%s TMP[C]: %s\r\n",
                 acc_x,
                 acc_y,
                 acc_z,
                 gyr_x,
                 gyr_y,
                 gyr_z,
                 temp_c);
  UART_SendString(uart_tx_buffer);
}

/*
 * Full application-level IMU startup sequence.
 *
 * This is intentionally kept in main.c instead of hiding everything inside
 * ICM42688P_Init(), because the user-facing boot messages are part of the
 * product behavior:
 * 1. initialize communication and default runtime config
 * 2. run official self-test
 * 3. read back the official bias that self-test produced
 * 4. print that official bias
 * 5. announce that initialization is complete
 * 6. wait for the user to send Enter before starting runtime output
 */
static int IMU_StartupSequence(void)
{
  int status;
  int self_test_result = 0;

  UART_SendString("ICM42688P init start...\r\n");

  status = ICM42688P_Init(&hspi3);
  if (status != 0)
  {
    (void)snprintf(uart_tx_buffer, sizeof(uart_tx_buffer), "ICM42688P init failed: %d\r\n", status);
    UART_SendString(uart_tx_buffer);
    return status;
  }

  UART_SendString("ICM42688P self-test running...\r\n");
  status = ICM42688P_RunSelfTest(&self_test_result);
  if (status != 0)
  {
    (void)snprintf(uart_tx_buffer, sizeof(uart_tx_buffer), "ICM42688P self-test command failed: %d\r\n", status);
    UART_SendString(uart_tx_buffer);
    return status;
  }

  if (self_test_result != IMU_SELF_TEST_PASS_MASK)
  {
    (void)snprintf(uart_tx_buffer,
                   sizeof(uart_tx_buffer),
                   "ICM42688P self-test failed: ACC=%s GYR=%s\r\n",
                   ((self_test_result & 0x02) != 0) ? "PASS" : "FAIL",
                   ((self_test_result & 0x01) != 0) ? "PASS" : "FAIL");
    UART_SendString(uart_tx_buffer);
    return -1;
  }

  UART_SendString("ICM42688P self-test passed.\r\n");

  status = ICM42688P_GetOfficialBias(&imu_official_bias);
  if (status != 0)
  {
    (void)snprintf(uart_tx_buffer, sizeof(uart_tx_buffer), "ICM42688P official bias readback failed: %d\r\n", status);
    UART_SendString(uart_tx_buffer);
    return status;
  }

  UART_SendString("Official self-test bias report:\r\n");
  UART_PrintOfficialBiasReport(&imu_official_bias);
  UART_WaitForEnterCommand();

  return 0;
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

  /* MPU Configuration--------------------------------------------------------*/
  MPU_Config();

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
  MX_USART1_UART_Init();
  MX_SPI3_Init();
  /* USER CODE BEGIN 2 */

  /*
   * Run the full IMU bring-up flow before entering the normal main loop.
   * If any stage fails, we stop in Error_Handler() because subsequent periodic
   * printing would otherwise be misleading.
   */
  if (IMU_StartupSequence() != 0)
  {
    Error_Handler();
  }

  uart_send_last_tick = HAL_GetTick();
  uart_rx_last_tick = HAL_GetTick();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    uint32_t now = HAL_GetTick();

    if ((now - uart_send_last_tick) >= UART_SEND_PERIOD_MS)
    {
      int tx_len = 0;
      uart_send_last_tick = now;

      /*
       * Normal runtime phase:
       * read one already-compensated sample and print converted values.
       */
      if (ICM42688P_ReadData(&imu_data) == 0)
      {
        UART_PrintConvertedImuData(&imu_data);
        tx_len = 1;
      }
      else
      {
        tx_len = snprintf(uart_tx_buffer, sizeof(uart_tx_buffer), "ICM42688P read failed\r\n");
        if (tx_len > 0)
        {
          HAL_UART_Transmit(&huart1, (uint8_t *)uart_tx_buffer, (uint16_t)tx_len, HAL_MAX_DELAY);
        }
      }

      HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
    }

    while (HAL_UART_Receive(&huart1, &uart_rx_byte, 1U, 0U) == HAL_OK)
    {
      UART_ProcessReceivedByte(uart_rx_byte);
    }

    if ((uart_rx_len > 0U) && ((HAL_GetTick() - uart_rx_last_tick) >= UART_RX_IDLE_MS))
    {
      UART_FlushReceivedMessage();
    }
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

  /** Supply configuration update enable
  */
  HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE0);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 5;
  RCC_OscInitStruct.PLL.PLLN = 192;
  RCC_OscInitStruct.PLL.PLLP = 2;
  RCC_OscInitStruct.PLL.PLLQ = 5;
  RCC_OscInitStruct.PLL.PLLR = 5;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_2;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
  RCC_OscInitStruct.PLL.PLLFRACN = 0;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief SPI3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI3_Init(void)
{

  /* USER CODE BEGIN SPI3_Init 0 */

  /* USER CODE END SPI3_Init 0 */

  /* USER CODE BEGIN SPI3_Init 1 */

  /* USER CODE END SPI3_Init 1 */
  /* SPI3 parameter configuration*/
  hspi3.Instance = SPI3;
  hspi3.Init.Mode = SPI_MODE_MASTER;
  hspi3.Init.Direction = SPI_DIRECTION_2LINES;
  hspi3.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi3.Init.CLKPolarity = SPI_POLARITY_HIGH;
  hspi3.Init.CLKPhase = SPI_PHASE_2EDGE;
  hspi3.Init.NSS = SPI_NSS_SOFT;
  hspi3.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
  hspi3.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi3.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi3.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi3.Init.CRCPolynomial = 0x0;
  hspi3.Init.NSSPMode = SPI_NSS_PULSE_DISABLE;
  hspi3.Init.NSSPolarity = SPI_NSS_POLARITY_LOW;
  hspi3.Init.FifoThreshold = SPI_FIFO_THRESHOLD_01DATA;
  hspi3.Init.TxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
  hspi3.Init.RxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
  hspi3.Init.MasterSSIdleness = SPI_MASTER_SS_IDLENESS_00CYCLE;
  hspi3.Init.MasterInterDataIdleness = SPI_MASTER_INTERDATA_IDLENESS_00CYCLE;
  hspi3.Init.MasterReceiverAutoSusp = SPI_MASTER_RX_AUTOSUSP_DISABLE;
  hspi3.Init.MasterKeepIOState = SPI_MASTER_KEEP_IO_STATE_DISABLE;
  hspi3.Init.IOSwap = SPI_IO_SWAP_DISABLE;
  if (HAL_SPI_Init(&hspi3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI3_Init 2 */
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /*
   * PA4 is used as a software-controlled chip-select for the IMU.
   * We intentionally configure it after HAL_SPI_Init() so the pin is under
   * plain GPIO control rather than SPI peripheral NSS control.
   */
  GPIO_InitStruct.Pin = ICM42688P_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(ICM42688P_CS_GPIO_Port, &GPIO_InitStruct);
  HAL_GPIO_WritePin(ICM42688P_CS_GPIO_Port, ICM42688P_CS_Pin, GPIO_PIN_SET);

  /* USER CODE END SPI3_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart1, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart1, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

 /* MPU Configuration */

void MPU_Config(void)
{
  MPU_Region_InitTypeDef MPU_InitStruct = {0};

  /* Disables the MPU */
  HAL_MPU_Disable();

  /** Initializes and configures the Region and the memory to be protected
  */
  MPU_InitStruct.Enable = MPU_REGION_ENABLE;
  MPU_InitStruct.Number = MPU_REGION_NUMBER0;
  MPU_InitStruct.BaseAddress = 0x0;
  MPU_InitStruct.Size = MPU_REGION_SIZE_4GB;
  MPU_InitStruct.SubRegionDisable = 0x87;
  MPU_InitStruct.TypeExtField = MPU_TEX_LEVEL0;
  MPU_InitStruct.AccessPermission = MPU_REGION_NO_ACCESS;
  MPU_InitStruct.DisableExec = MPU_INSTRUCTION_ACCESS_DISABLE;
  MPU_InitStruct.IsShareable = MPU_ACCESS_SHAREABLE;
  MPU_InitStruct.IsCacheable = MPU_ACCESS_NOT_CACHEABLE;
  MPU_InitStruct.IsBufferable = MPU_ACCESS_NOT_BUFFERABLE;

  HAL_MPU_ConfigRegion(&MPU_InitStruct);
  /* Enables the MPU */
  HAL_MPU_Enable(MPU_PRIVILEGED_DEFAULT);

}

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
