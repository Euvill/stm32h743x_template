#ifndef __ICM42688P_H
#define __ICM42688P_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32h7xx_hal.h"

#include <stdint.h>

/*
 * This header exposes a small board-specific wrapper around the official
 * InvenSense/TDK ICM42688P driver.
 *
 * Design goal:
 * 1. Keep all low-level SPI / delay / scaling details inside one module.
 * 2. Let application code call a few simple functions such as Init, SelfTest,
 *    GetOfficialBias and ReadData.
 * 3. Preserve access to both raw register data and converted engineering units.
 *
 * Important note about "raw" data in this wrapper:
 * - temperature_raw is always the original temperature register value
 * - accel_raw / gyro_raw are the samples returned after the currently applied
 *   INV bias compensation has taken effect
 *
 * In other words, accel_raw / gyro_raw still use LSB as their numeric unit,
 * but they are no longer "factory untouched" register values once bias has
 * been applied through the official INV API.
 */

typedef struct
{
  int16_t temperature_raw; /* Temperature register raw value, unit: LSB */
  int16_t accel_raw[3];    /* Bias-compensated accelerometer raw values, unit: LSB */
  int16_t gyro_raw[3];     /* Bias-compensated gyroscope raw values, unit: LSB */
  float temperature_c;     /* Converted temperature, unit: degC */
  float accel_g[3];        /* Converted acceleration, unit: g */
  float gyro_dps[3];       /* Converted angular rate, unit: dps */
} ICM42688P_Data_t;

typedef struct
{
  /*
   * Bias returned by the official INV self-test API.
   *
   * The official function inv_icm426xx_get_st_bias() returns six Q16 values:
   * - gyro X/Y/Z first, unit = dps * 2^16
   * - accel X/Y/Z next, unit = g * 2^16
   *
   * We keep both representations:
   * - q16 values: exact copy of what the official API returned
   * - float values: easier to print and reason about in application code
   */
  int gyro_q16[3];   /* Official gyro self-test bias, unit: dps * 2^16 */
  int accel_q16[3];  /* Official accel self-test bias, unit: g * 2^16 */
  float gyro_dps[3]; /* Same gyro bias converted to dps */
  float accel_g[3];  /* Same accel bias converted to g */
} ICM42688P_OfficialBias_t;

/*
 * Bring the device up over SPI and apply the default runtime configuration.
 *
 * What this function does:
 * - enables the microsecond delay backend used by the INV driver
 * - binds the official inv_icm426xx transport callbacks to STM32 HAL SPI
 * - checks WHO_AM_I
 * - configures default accel / gyro full-scale range and ODR
 * - enables accel and gyro in low-noise mode
 *
 * What this function does not do:
 * - self-test
 * - UART interaction
 * - periodic reading / printing
 */
int ICM42688P_Init(SPI_HandleTypeDef *hspi);

/*
 * Execute the official hardware self-test sequence supplied by INV.
 *
 * Result bit meaning:
 * - bit0: gyro self-test result
 * - bit1: accel self-test result
 * - value 3 means both passed
 */
int ICM42688P_RunSelfTest(int *result);

/*
 * Read back the bias currently reported by the official INV self-test path.
 *
 * Important behavior:
 * - this function does not estimate bias by itself
 * - it simply asks the official driver for the bias it already owns
 * - if self-test has not run yet, the official API returns zeros
 *
 * Returned bias order matches the official documentation:
 * - gyro X/Y/Z
 * - accel X/Y/Z
 */
int ICM42688P_GetOfficialBias(ICM42688P_OfficialBias_t *bias);

/*
 * Read one complete sample group:
 * - temperature
 * - accel X/Y/Z
 * - gyro X/Y/Z
 *
 * Returned structure contains:
 * - raw LSB values
 * - converted physical values
 */
int ICM42688P_ReadData(ICM42688P_Data_t *data);

/* Helper for diagnostics: read the WHO_AM_I register directly. */
int ICM42688P_ReadWhoAmI(uint8_t *who_am_i);

#ifdef __cplusplus
}
#endif

#endif /* __ICM42688P_H */
