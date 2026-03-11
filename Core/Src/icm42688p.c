#include "icm42688p.h"

#include "main.h"

#include "Icm426xxDriver_HL.h"
#include "Icm426xxSelfTest.h"
#include "InvError.h"

#include <string.h>

/*
 * This file is the "glue layer" between:
 * - STM32 HAL SPI / HAL timing primitives
 * - the official INV ICM426xx high-level driver
 * - the application-facing API declared in icm42688p.h
 *
 * Why this file exists:
 * The vendor driver is portable and does not know anything about STM32 HAL.
 * It expects the application to provide:
 * - register read/write callbacks
 * - microsecond delay
 * - timestamp source
 *
 * So this file translates between both worlds:
 * 1. HAL SPI transactions are exposed as inv_icm426xx_serif callbacks.
 * 2. INV driver APIs are wrapped as easy-to-read project APIs.
 * 3. Physical unit conversion and user-facing bias reporting live here.
 */

#define ICM42688P_SPI_TIMEOUT_MS           100U
#define ICM42688P_SPI_READ_MASK            0x80U
#define ICM42688P_SPI_WRITE_MASK           0x7FU
#define ICM42688P_BURST_READ_SIZE          (TEMP_DATA_SIZE + ACCEL_DATA_SIZE + GYRO_DATA_SIZE)
#define ICM42688P_TRANSFER_BUFFER_SIZE     (ICM426XX_FIFO_MIRRORING_SIZE + 1U)

/* Default runtime configuration used right after power-up. */
#define ICM42688P_DEFAULT_ACCEL_FSR        ICM426XX_ACCEL_CONFIG0_FS_SEL_16g
#define ICM42688P_DEFAULT_GYRO_FSR         ICM426XX_GYRO_CONFIG0_FS_SEL_2000dps
#define ICM42688P_DEFAULT_ACCEL_ODR        ICM426XX_ACCEL_CONFIG0_ODR_1_KHZ
#define ICM42688P_DEFAULT_GYRO_ODR         ICM426XX_GYRO_CONFIG0_ODR_1_KHZ

/* Temperature conversion from datasheet: Temp(degC) = raw / 132.48 + 25. */
#define ICM42688P_TEMPERATURE_OFFSET_C     25.0f
#define ICM42688P_TEMPERATURE_SENSITIVITY  132.48f

/*
 * Module-private state
 *
 * s_icm42688p_device:
 *   Official INV device object. All inv_icm426xx_* APIs operate on this.
 *
 * s_spi_tx_buffer / s_spi_rx_buffer:
 *   Shared transfer buffers used by the serif callbacks. Keeping them static
 *   avoids stack growth during register access.
 *
 * s_accel_fsr / s_gyro_fsr:
 *   Local mirror of the chosen full-scale ranges. These are needed to convert
 *   raw LSB data into g / dps without re-reading configuration registers every
 *   time a sample is processed.
 *
 * s_icm42688p_initialized:
 *   Simple guard so application code cannot read data before initialization.
 *
 * s_dwt_ready:
 *   Tracks whether the Cortex-M DWT cycle counter is available for us-level
 *   delays required by the official driver.
 */
static struct inv_icm426xx s_icm42688p_device;
static uint8_t s_spi_tx_buffer[ICM42688P_TRANSFER_BUFFER_SIZE];
static uint8_t s_spi_rx_buffer[ICM42688P_TRANSFER_BUFFER_SIZE];
static ICM426XX_ACCEL_CONFIG0_FS_SEL_t s_accel_fsr = ICM42688P_DEFAULT_ACCEL_FSR;
static ICM426XX_GYRO_CONFIG0_FS_SEL_t s_gyro_fsr = ICM42688P_DEFAULT_GYRO_FSR;
static uint8_t s_icm42688p_initialized = 0U;
static uint8_t s_dwt_ready = 0U;

/*
 * Pull chip-select low before a SPI transaction.
 *
 * We use software-controlled CS because the IMU register protocol expects the
 * register address byte and the following payload bytes to belong to one
 * uninterrupted transaction.
 */
static void ICM42688P_Select(void)
{
  HAL_GPIO_WritePin(ICM42688P_CS_GPIO_Port, ICM42688P_CS_Pin, GPIO_PIN_RESET);
}

/* Release chip-select after a SPI transaction. */
static void ICM42688P_Deselect(void)
{
  HAL_GPIO_WritePin(ICM42688P_CS_GPIO_Port, ICM42688P_CS_Pin, GPIO_PIN_SET);
}

/*
 * Enable Cortex-M7 DWT cycle counting.
 *
 * The official INV code uses microsecond waits in several places:
 * - initial power-up settling
 * - sensor mode transitions
 * - self-test flow
 *
 * HAL_Delay() only gives millisecond resolution, so we enable CYCCNT and build
 * a coarse+fine delay implementation on top of it.
 */
static void ICM42688P_EnableCycleCounter(void)
{
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
  DWT->CYCCNT = 0U;
  DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
  s_dwt_ready = ((DWT->CTRL & DWT_CTRL_CYCCNTENA_Msk) != 0U) ? 1U : 0U;
}

/*
 * Convert two bytes from the sensor data stream into a signed 16-bit value.
 *
 * Why not just do a fixed big-endian parse?
 * The ICM42688P can expose data in either endianness, and the official driver
 * stores that choice in s_icm42688p_device.endianess_data during init.
 */
static int16_t ICM42688P_ParseInt16(const uint8_t *buffer)
{
  uint16_t value;

  if (s_icm42688p_device.endianess_data == ICM426XX_INTF_CONFIG0_DATA_BIG_ENDIAN)
  {
    value = (uint16_t)(((uint16_t)buffer[0] << 8) | buffer[1]);
  }
  else
  {
    value = (uint16_t)(((uint16_t)buffer[1] << 8) | buffer[0]);
  }

  return (int16_t)value;
}

/*
 * Read one coherent raw sample group directly from the sensor data registers.
 *
 * Register layout used here:
 * - TEMP_DATA0_UI / TEMP_DATA1_UI
 * - ACCEL_DATA_X/Y/Z
 * - GYRO_DATA_X/Y/Z
 *
 * We read them as one burst because:
 * - it is simpler to understand
 * - it minimizes SPI overhead
 * - it gives one "snapshot" of temp + accel + gyro together
 */
static int ICM42688P_ReadRawSample(int16_t *temperature_raw, int16_t accel_raw[3], int16_t gyro_raw[3])
{
  uint8_t raw_data[ICM42688P_BURST_READ_SIZE];
  int status;

  if ((temperature_raw == NULL) || (accel_raw == NULL) || (gyro_raw == NULL))
  {
    return INV_ERROR_BAD_ARG;
  }

  status = inv_icm426xx_read_reg(&s_icm42688p_device,
                                 MPUREG_TEMP_DATA0_UI,
                                 ICM42688P_BURST_READ_SIZE,
                                 raw_data);
  if (status != INV_ERROR_SUCCESS)
  {
    return status;
  }

  *temperature_raw = ICM42688P_ParseInt16(&raw_data[0]);
  accel_raw[0] = ICM42688P_ParseInt16(&raw_data[2]);
  accel_raw[1] = ICM42688P_ParseInt16(&raw_data[4]);
  accel_raw[2] = ICM42688P_ParseInt16(&raw_data[6]);
  gyro_raw[0] = ICM42688P_ParseInt16(&raw_data[8]);
  gyro_raw[1] = ICM42688P_ParseInt16(&raw_data[10]);
  gyro_raw[2] = ICM42688P_ParseInt16(&raw_data[12]);

  return INV_ERROR_SUCCESS;
}

/*
 * Return the acceleration scale factor for the currently configured FSR.
 *
 * Example:
 * - at +-16 g, one full signed range is 32768 LSB
 * - so 1 LSB = 16 / 32768 g
 */
static float ICM42688P_GetAccelScale(void)
{
  switch (s_accel_fsr)
  {
    case ICM426XX_ACCEL_CONFIG0_FS_SEL_2g:
      return 2.0f / 32768.0f;
    case ICM426XX_ACCEL_CONFIG0_FS_SEL_4g:
      return 4.0f / 32768.0f;
    case ICM426XX_ACCEL_CONFIG0_FS_SEL_8g:
      return 8.0f / 32768.0f;
    case ICM426XX_ACCEL_CONFIG0_FS_SEL_16g:
    default:
      return 16.0f / 32768.0f;
  }
}

/*
 * Return the gyroscope scale factor for the currently configured FSR.
 *
 * Example:
 * - at +-2000 dps, one full signed range is 32768 LSB
 * - so 1 LSB = 2000 / 32768 dps
 */
static float ICM42688P_GetGyroScale(void)
{
  switch (s_gyro_fsr)
  {
    case ICM426XX_GYRO_CONFIG0_FS_SEL_16dps:
      return 16.0f / 32768.0f;
    case ICM426XX_GYRO_CONFIG0_FS_SEL_31dps:
      return 31.25f / 32768.0f;
    case ICM426XX_GYRO_CONFIG0_FS_SEL_62dps:
      return 62.5f / 32768.0f;
    case ICM426XX_GYRO_CONFIG0_FS_SEL_125dps:
      return 125.0f / 32768.0f;
    case ICM426XX_GYRO_CONFIG0_FS_SEL_250dps:
      return 250.0f / 32768.0f;
    case ICM426XX_GYRO_CONFIG0_FS_SEL_500dps:
      return 500.0f / 32768.0f;
    case ICM426XX_GYRO_CONFIG0_FS_SEL_1000dps:
      return 1000.0f / 32768.0f;
    case ICM426XX_GYRO_CONFIG0_FS_SEL_2000dps:
    default:
      return 2000.0f / 32768.0f;
  }
}

/* Convert one Q16 engineering-unit value back to float for printing. */
static float ICM42688P_Q16ToFloat(int q16_value)
{
  return (float)q16_value / 65536.0f;
}

/*
 * Optional transport-side configuration callback required by the INV serif
 * structure.
 *
 * For STM32 HAL SPI there is nothing extra to do here because:
 * - SPI mode is already configured by Cube / HAL
 * - CS is handled explicitly in our read/write helpers
 */
static int ICM42688P_ConfigureSerif(struct inv_icm426xx_serif *serif)
{
  (void)serif;
  return INV_ERROR_SUCCESS;
}

/*
 * Official INV transport callback: read one register range over 4-wire SPI.
 *
 * SPI frame structure:
 * - first byte: register address with bit7 set to 1 for read
 * - following bytes: dummy bytes clocking sensor data out
 *
 * We keep CS low across the full transfer so the device treats it as one
 * contiguous read transaction.
 */
static int ICM42688P_ReadRegister(struct inv_icm426xx_serif *serif, uint8_t reg, uint8_t *buf, uint32_t len)
{
  SPI_HandleTypeDef *hspi;
  HAL_StatusTypeDef hal_status;

  if ((serif == NULL) || (buf == NULL))
  {
    return INV_ERROR_BAD_ARG;
  }

  if (len == 0U)
  {
    return INV_ERROR_SUCCESS;
  }

  if ((len + 1U) > ICM42688P_TRANSFER_BUFFER_SIZE)
  {
    return INV_ERROR_SIZE;
  }

  hspi = (SPI_HandleTypeDef *)serif->context;
  if (hspi == NULL)
  {
    return INV_ERROR_BAD_ARG;
  }

  s_spi_tx_buffer[0] = reg | ICM42688P_SPI_READ_MASK;
  memset(&s_spi_tx_buffer[1], 0xFF, len);
  memset(s_spi_rx_buffer, 0, len + 1U);

  ICM42688P_Select();
  hal_status = HAL_SPI_TransmitReceive(hspi,
                                       s_spi_tx_buffer,
                                       s_spi_rx_buffer,
                                       (uint16_t)(len + 1U),
                                       ICM42688P_SPI_TIMEOUT_MS);
  ICM42688P_Deselect();

  if (hal_status != HAL_OK)
  {
    return INV_ERROR_TRANSPORT;
  }

  memcpy(buf, &s_spi_rx_buffer[1], len);

  return INV_ERROR_SUCCESS;
}

/*
 * Official INV transport callback: write one register range over 4-wire SPI.
 *
 * SPI frame structure:
 * - first byte: register address with bit7 cleared to 0 for write
 * - following bytes: payload bytes written into consecutive registers
 */
static int ICM42688P_WriteRegister(struct inv_icm426xx_serif *serif, uint8_t reg, const uint8_t *buf, uint32_t len)
{
  SPI_HandleTypeDef *hspi;
  HAL_StatusTypeDef hal_status;

  if ((serif == NULL) || ((buf == NULL) && (len > 0U)))
  {
    return INV_ERROR_BAD_ARG;
  }

  if ((len + 1U) > ICM42688P_TRANSFER_BUFFER_SIZE)
  {
    return INV_ERROR_SIZE;
  }

  hspi = (SPI_HandleTypeDef *)serif->context;
  if (hspi == NULL)
  {
    return INV_ERROR_BAD_ARG;
  }

  s_spi_tx_buffer[0] = reg & ICM42688P_SPI_WRITE_MASK;
  if (len > 0U)
  {
    memcpy(&s_spi_tx_buffer[1], buf, len);
  }

  ICM42688P_Select();
  hal_status = HAL_SPI_Transmit(hspi,
                                s_spi_tx_buffer,
                                (uint16_t)(len + 1U),
                                ICM42688P_SPI_TIMEOUT_MS);
  ICM42688P_Deselect();

  if (hal_status != HAL_OK)
  {
    return INV_ERROR_TRANSPORT;
  }

  return INV_ERROR_SUCCESS;
}

/*
 * Required by the official INV driver.
 *
 * This function provides a microsecond-level sleep primitive. The
 * implementation is split into:
 * - HAL_Delay() for the millisecond portion
 * - DWT cycle counting for the remaining microseconds
 *
 * That hybrid approach is easy to understand and good enough for the driver's
 * timing requirements.
 */
void inv_icm426xx_sleep_us(uint32_t us)
{
  uint32_t ms_delay;
  uint32_t us_delay;
  uint32_t cycles_per_us;
  uint32_t start_cycle;
  uint32_t wait_cycles;

  if (us == 0U)
  {
    return;
  }

  ms_delay = us / 1000U;
  us_delay = us % 1000U;

  if (ms_delay > 0U)
  {
    HAL_Delay(ms_delay);
  }

  if (us_delay == 0U)
  {
    return;
  }

  cycles_per_us = HAL_RCC_GetHCLKFreq() / 1000000U;
  if ((s_dwt_ready == 0U) || (cycles_per_us == 0U))
  {
    HAL_Delay(1U);
    return;
  }

  start_cycle = DWT->CYCCNT;
  wait_cycles = us_delay * cycles_per_us;
  while ((DWT->CYCCNT - start_cycle) < wait_cycles)
  {
  }
}

/*
 * Required by the official INV driver.
 *
 * We expose a coarse microsecond timestamp based on HAL_GetTick().
 * Even though its true resolution is 1 ms, it is sufficient for the way the
 * vendor driver uses this hook in our current feature set.
 */
uint64_t inv_icm426xx_get_time_us(void)
{
  return (uint64_t)HAL_GetTick() * 1000ULL;
}

/*
 * Public initialization entry.
 *
 * Read this function as the "bring-up script" for the IMU:
 * 1. make sure delay backend is ready
 * 2. deassert chip select
 * 3. populate the official INV transport structure
 * 4. call inv_icm426xx_init()
 * 5. confirm device identity through WHO_AM_I
 * 6. apply project-default runtime configuration
 * 7. clear wrapper-side baseline cache
 */
int ICM42688P_Init(SPI_HandleTypeDef *hspi)
{
  struct inv_icm426xx_serif serif;
  uint8_t who_am_i = 0U;
  int status = INV_ERROR_SUCCESS;

  if (hspi == NULL)
  {
    return INV_ERROR_BAD_ARG;
  }

  ICM42688P_EnableCycleCounter();
  ICM42688P_Deselect();

  /* Bind HAL SPI to the vendor transport abstraction. */
  memset(&serif, 0, sizeof(serif));
  serif.context = hspi;
  serif.read_reg = ICM42688P_ReadRegister;
  serif.write_reg = ICM42688P_WriteRegister;
  serif.configure = ICM42688P_ConfigureSerif;
  serif.max_read = ICM42688P_TRANSFER_BUFFER_SIZE - 1U;
  serif.max_write = ICM42688P_TRANSFER_BUFFER_SIZE - 1U;
  serif.serif_type = ICM426XX_UI_SPI4;

  status = inv_icm426xx_init(&s_icm42688p_device, &serif, NULL);
  if (status != INV_ERROR_SUCCESS)
  {
    return status;
  }

  status = inv_icm426xx_get_who_am_i(&s_icm42688p_device, &who_am_i);
  if (status != INV_ERROR_SUCCESS)
  {
    return status;
  }

  if (who_am_i != ICM_WHOAMI)
  {
    return INV_ERROR_HW;
  }

  /*
   * Mirror the selected FSR locally.
   * The INV driver already knows the configuration, but caching it here keeps
   * the data conversion path straightforward and self-contained.
   */
  s_accel_fsr = ICM42688P_DEFAULT_ACCEL_FSR;
  s_gyro_fsr = ICM42688P_DEFAULT_GYRO_FSR;

  /* Default sensor setup:
   * accel  : low-noise mode, +-16 g, 1 kHz ODR
   * gyro   : low-noise mode, +-2000 dps, 1 kHz ODR
   */
  status |= inv_icm426xx_set_accel_fsr(&s_icm42688p_device, s_accel_fsr);
  status |= inv_icm426xx_set_gyro_fsr(&s_icm42688p_device, s_gyro_fsr);
  status |= inv_icm426xx_set_accel_frequency(&s_icm42688p_device, ICM42688P_DEFAULT_ACCEL_ODR);
  status |= inv_icm426xx_set_gyro_frequency(&s_icm42688p_device, ICM42688P_DEFAULT_GYRO_ODR);
  status |= inv_icm426xx_enable_accel_low_noise_mode(&s_icm42688p_device);
  status |= inv_icm426xx_enable_gyro_low_noise_mode(&s_icm42688p_device);
  if (status != INV_ERROR_SUCCESS)
  {
    s_icm42688p_initialized = 0U;
    return status;
  }

  s_icm42688p_initialized = 1U;
  return INV_ERROR_SUCCESS;
}

/*
 * Thin wrapper around the official self-test API.
 *
 * Keeping this function separate from Init() makes the startup flow easier to
 * read from main.c:
 * - initialize communication first
 * - run self-test next
 * - then explicitly read back and print the official bias
 */
int ICM42688P_RunSelfTest(int *result)
{
  if ((s_icm42688p_initialized == 0U) || (result == NULL))
  {
    return INV_ERROR_BAD_ARG;
  }

  return inv_icm426xx_run_selftest(&s_icm42688p_device, result);
}

/*
 * Read back the bias currently owned by the official INV self-test path.
 *
 * Important separation of responsibilities:
 * - inv_icm426xx_run_selftest() performs the measurement and stores the result
 *   inside s_icm42688p_device.gyro_st_bias[] / accel_st_bias[]
 * - inv_icm426xx_get_st_bias() converts those internal values into official
 *   Q16 engineering units for application use
 * - this wrapper only repackages that information into a clearer project-level
 *   structure so main.c can print it without knowing INV internals
 *
 * Official unit convention returned by INV:
 * - gyro bias: dps * 2^16
 * - accel bias: g * 2^16
 *
 * If self-test has not run yet, the official INV API returns all zeros.
 */
int ICM42688P_GetOfficialBias(ICM42688P_OfficialBias_t *bias)
{
  int st_bias_q16[6] = {0};
  uint8_t axis;
  int status;

  if ((s_icm42688p_initialized == 0U) || (bias == NULL))
  {
    return INV_ERROR_BAD_ARG;
  }

  status = inv_icm426xx_get_st_bias(&s_icm42688p_device, st_bias_q16);
  if (status != INV_ERROR_SUCCESS)
  {
    return status;
  }

  for (axis = 0U; axis < 3U; axis++)
  {
    bias->gyro_q16[axis] = st_bias_q16[axis];
    bias->accel_q16[axis] = st_bias_q16[axis + 3U];
    bias->gyro_dps[axis] = ICM42688P_Q16ToFloat(st_bias_q16[axis]);
    bias->accel_g[axis] = ICM42688P_Q16ToFloat(st_bias_q16[axis + 3U]);
  }

  return INV_ERROR_SUCCESS;
}

/* Diagnostic helper: direct WHO_AM_I access through the official driver. */
int ICM42688P_ReadWhoAmI(uint8_t *who_am_i)
{
  if ((s_icm42688p_initialized == 0U) || (who_am_i == NULL))
  {
    return INV_ERROR_BAD_ARG;
  }

  return inv_icm426xx_get_who_am_i(&s_icm42688p_device, who_am_i);
}

/*
 * Read one sample and present it in a user-friendly structure.
 *
 * Important behavior:
 * - this function never performs application-side bias subtraction
 * - it simply reads the current data registers and converts them
 *
 * Therefore:
 * - if the official self-test already programmed offset registers, the values
 *   you see here already include that official correction
 * - if self-test was not executed, the values are just direct sensor outputs
 *
 * So the role of this function is:
 * 1. fetch current raw registers
 * 2. preserve them as-is
 * 3. convert them into g / dps / degC
 */
int ICM42688P_ReadData(ICM42688P_Data_t *data)
{
  int16_t temperature_raw;
  int16_t accel_raw[3];
  int16_t gyro_raw[3];
  float accel_scale;
  float gyro_scale;
  int status;

  if ((s_icm42688p_initialized == 0U) || (data == NULL))
  {
    return INV_ERROR_BAD_ARG;
  }

  status = ICM42688P_ReadRawSample(&temperature_raw, accel_raw, gyro_raw);
  if (status != INV_ERROR_SUCCESS)
  {
    return status;
  }

  data->temperature_raw = temperature_raw;
  /*
   * Keep the raw register values exactly as they were read.
   *
   * If official self-test bias has already been written into the IMU offset
   * registers, the sensor hardware itself may already be outputting corrected
   * data. We do not apply any second correction layer here.
   */
  data->accel_raw[0] = accel_raw[0];
  data->accel_raw[1] = accel_raw[1];
  data->accel_raw[2] = accel_raw[2];
  data->gyro_raw[0] = gyro_raw[0];
  data->gyro_raw[1] = gyro_raw[1];
  data->gyro_raw[2] = gyro_raw[2];

  accel_scale = ICM42688P_GetAccelScale();
  gyro_scale = ICM42688P_GetGyroScale();

  /*
   * Store both representations:
   * - raw LSB values are useful for debugging and validation
   * - converted values are what the application prints periodically
   */
  data->temperature_c = ((float)data->temperature_raw / ICM42688P_TEMPERATURE_SENSITIVITY) +
                        ICM42688P_TEMPERATURE_OFFSET_C;
  data->accel_g[0] = (float)data->accel_raw[0] * accel_scale;
  data->accel_g[1] = (float)data->accel_raw[1] * accel_scale;
  data->accel_g[2] = (float)data->accel_raw[2] * accel_scale;
  data->gyro_dps[0] = (float)data->gyro_raw[0] * gyro_scale;
  data->gyro_dps[1] = (float)data->gyro_raw[1] * gyro_scale;
  data->gyro_dps[2] = (float)data->gyro_raw[2] * gyro_scale;

  return INV_ERROR_SUCCESS;
}
