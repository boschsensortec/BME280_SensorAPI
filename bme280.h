/**
* Copyright (c) 2020 Bosch Sensortec GmbH. All rights reserved.
*
* BSD-3-Clause
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are met:
*
* 1. Redistributions of source code must retain the above copyright
*    notice, this list of conditions and the following disclaimer.
*
* 2. Redistributions in binary form must reproduce the above copyright
*    notice, this list of conditions and the following disclaimer in the
*    documentation and/or other materials provided with the distribution.
*
* 3. Neither the name of the copyright holder nor the names of its
*    contributors may be used to endorse or promote products derived from
*    this software without specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
* "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
* LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
* FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
* COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
* INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
* (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
* SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
* HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
* STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING
* IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
* POSSIBILITY OF SUCH DAMAGE.
*
* @file       bme280.h
* @date       2020-12-17
* @version    v3.5.1
*
*/

/*! @file bme280.h
 * @brief Sensor driver for BME280 sensor
 */

/*!
 * @defgroup bme280 BME280
 * @brief <a href="https://www.bosch-sensortec.com/bst/products/all_products/bme280">Product Overview</a>
 * and  <a href="https://github.com/BoschSensortec/BME280_driver">Sensor API Source Code</a>
 */

#ifndef _BME280_H
#define _BME280_H

/*! CPP guard */
#ifdef __cplusplus
extern "C" {
#endif

/* Header includes */
#include "bme280_defs.h"

/**
 * \ingroup bme280
 * \defgroup bme280ApiInit Initialization
 * @brief Initialize the sensor and device structure
 */

/*!
 * \ingroup bme280ApiInit
 * \page bme280_api_bme280_init bme280_init
 * \code
 * int8_t bme280_init(struct bme280_dev *dev);
 * \endcode
 * @details This API reads the chip-id of the sensor which is the first step to
 * verify the sensor and also calibrates the sensor
 * As this API is the entry point, call this API before using other APIs.
 *
 * @param[in,out] dev : Structure instance of bme280_dev
 *
 * @return Result of API execution status.
 *
 * @retval   0 -> Success.
 * @retval > 0 -> Warning.
 * @retval < 0 -> Fail.
 *
 */
int8_t bme280_init(struct bme280_dev *dev);

/**
 * \ingroup bme280
 * \defgroup bme280ApiRegister Registers
 * @brief Generic API for accessing sensor registers
 */

/*!
 * \ingroup bme280ApiRegister
 * \page bme280_api_bme280_set_regs bme280_set_regs
 * \code
 * int8_t bme280_set_regs(const uint8_t reg_addr, const uint8_t *reg_data, uint32_t len, struct bme280_dev *dev);
 * \endcode
 * @details This API writes the given data to the register address of the sensor
 *
 * @param[in] reg_addr : Register addresses to where the data is to be written
 * @param[in] reg_data : Pointer to data buffer which is to be written
 *                       in the reg_addr of sensor.
 * @param[in] len      : No of bytes of data to write
 * @param[in,out] dev  : Structure instance of bme280_dev
 *
 * @return Result of API execution status.
 *
 * @retval   0 -> Success.
 * @retval > 0 -> Warning.
 * @retval < 0 -> Fail.
 *
 */
int8_t bme280_set_regs(uint8_t *reg_addr, const uint8_t *reg_data, uint32_t len, struct bme280_dev *dev);

/*!
 * \ingroup bme280ApiRegister
 * \page bme280_api_bme280_get_regs bme280_get_regs
 * \code
 * int8_t bme280_get_regs(uint8_t reg_addr, uint8_t *reg_data, uint32_t len, struct bme280_dev *dev);
 * \endcode
 * @details This API reads the data from the given register address of sensor.
 *
 * @param[in] reg_addr  : Register address from where the data to be read
 * @param[out] reg_data : Pointer to data buffer to store the read data.
 * @param[in] len       : No of bytes of data to be read.
 * @param[in,out] dev   : Structure instance of bme280_dev.
 *
 * @return Result of API execution status.
 *
 * @retval   0 -> Success.
 * @retval > 0 -> Warning.
 * @retval < 0 -> Fail.
 *
 */
int8_t bme280_get_regs(uint8_t reg_addr, uint8_t *reg_data, uint32_t len, struct bme280_dev *dev);

/**
 * \ingroup bme280
 * \defgroup bme280ApiSensorSettings Sensor Settings
 * @brief Generic API for accessing sensor settings
 */

/*!
 * \ingroup bme280ApiSensorSettings
 * \page bme280_api_bme280_set_sensor_settings bme280_set_sensor_settings
 * \code
 * int8_t bme280_set_sensor_settings(uint8_t desired_settings, const struct bme280_settings *settings, struct bme280_dev *dev);
 * \endcode
 * @details This API sets the oversampling, filter and standby duration
 * (normal mode) settings in the sensor.
 *
 * @param[in] desired_settings  : Variable used to select the settings which
 *                                are to be set in the sensor.
 * @param[in] settings          : Structure instance of bme280_settings.
 * @param[in] dev               : Structure instance of bme280_dev.
 *
 * @note : Below are the macros to be used by the user for selecting the
 * desired settings. User can do OR operation of these macros for configuring
 * multiple settings.
 *
 *@verbatim
 * Macros                 |   Functionality
 * -----------------------|----------------------------------------------
 * BME280_SEL_OSR_PRESS   |   To set pressure oversampling.
 * BME280_SEL_OSR_TEMP    |   To set temperature oversampling.
 * BME280_SEL_OSR_HUM     |   To set humidity oversampling.
 * BME280_SEL_FILTER      |   To set filter setting.
 * BME280_SEL_STANDBY     |   To set standby duration setting.
 *@endverbatim
 *
 * @return Result of API execution status
 *
 * @retval   0 -> Success.
 * @retval > 0 -> Warning.
 * @retval < 0 -> Fail.
 *
 */
int8_t bme280_set_sensor_settings(uint8_t desired_settings,
                                  const struct bme280_settings *settings,
                                  struct bme280_dev *dev);

/*!
 * \ingroup bme280ApiSensorSettings
 * \page bme280_api_bme280_get_sensor_settings bme280_get_sensor_settings
 * \code
 * int8_t bme280_get_sensor_settings(struct bme280_settings *settings, struct bme280_dev *dev);
 * \endcode
 * @details This API gets the oversampling, filter and standby duration
 * (normal mode) settings from the sensor.
 *
 * @param[in] settings  : Structure instance of bme280_settings.
 * @param[in,out] dev   : Structure instance of bme280_dev.
 *
 * @return Result of API execution status
 *
 * @retval   0 -> Success.
 * @retval > 0 -> Warning.
 * @retval < 0 -> Fail.
 *
 */
int8_t bme280_get_sensor_settings(struct bme280_settings *settings, struct bme280_dev *dev);

/**
 * \ingroup bme280
 * \defgroup bme280ApiSensorMode Sensor Mode
 * @brief Generic API for configuring sensor power mode
 */

/*!
 * \ingroup bme280ApiSensorMode
 * \page bme280_api_bme280_set_sensor_mode bme280_set_sensor_mode
 * \code
 * int8_t bme280_set_sensor_mode(uint8_t sensor_mode, const struct bme280_dev *dev);
 * \endcode
 * @details This API sets the power mode of the sensor.
 *
 * @param[in] sensor_mode : Variable which contains the power mode to be set.
 * @param[in] dev         : Structure instance of bme280_dev.
 *
 *@verbatim
 *    sensor_mode       |      Macros
 * ---------------------|-------------------------
 *     0                | BME280_POWERMODE_SLEEP
 *     1                | BME280_POWERMODE_FORCED
 *     3                | BME280_POWERMODE_NORMAL
 *@endverbatim
 *
 * @return Result of API execution status
 *
 * @retval   0 -> Success.
 * @retval > 0 -> Warning.
 * @retval < 0 -> Fail.
 *
 */
int8_t bme280_set_sensor_mode(uint8_t sensor_mode, struct bme280_dev *dev);

/*!
 * \ingroup bme280ApiSensorMode
 * \page bme280_api_bme280_get_sensor_mode bme280_get_sensor_mode
 * \code
 * int8_t bme280_get_sensor_mode(uint8_t *sensor_mode, struct bme280_dev *dev);
 * \endcode
 * @details This API gets the power mode of the sensor.
 *
 * @param[out] sensor_mode : Pointer variable to store the power mode.
 * @param[in] dev          : Structure instance of bme280_dev.
 *
 *@verbatim
 *    sensor_mode       |      Macros
 * ---------------------|-------------------------
 *     0                | BME280_POWERMODE_SLEEP
 *     1                | BME280_POWERMODE_FORCED
 *     3                | BME280_POWERMODE_NORMAL
 *@endverbatim
 *
 * @return Result of API execution status
 *
 * @retval   0 -> Success.
 * @retval > 0 -> Warning.
 * @retval < 0 -> Fail.
 *
 */
int8_t bme280_get_sensor_mode(uint8_t *sensor_mode, struct bme280_dev *dev);

/**
 * \ingroup bme280
 * \defgroup bme280ApiSystem System
 * @brief API that performs system-level operations
 */

/*!
 * \ingroup bme280ApiSystem
 * \page bme280_api_bme280_soft_reset bme280_soft_reset
 * \code
 * int8_t bme280_soft_reset(struct bme280_dev *dev);
 * \endcode
 * @details This API soft-resets the sensor.
 *
 * @param[in,out] dev : Structure instance of bme280_dev.
 *
 * @return Result of API execution status.
 *
 * @retval   0 -> Success.
 * @retval > 0 -> Warning.
 * @retval < 0 -> Fail.
 *
 */
int8_t bme280_soft_reset(struct bme280_dev *dev);

/**
 * \ingroup bme280
 * \defgroup bme280ApiSensorData Sensor Data
 * @brief Data processing of sensor
 */

/*!
 * \ingroup bme280ApiSensorData
 * \page bme280_api_bme280_get_sensor_data bme280_get_sensor_data
 * \code
 * int8_t bme280_get_sensor_data(uint8_t sensor_comp, struct bme280_data *comp_data, struct bme280_dev *dev);
 * \endcode
 * @details This API reads the pressure, temperature and humidity data from the
 * sensor, compensates the data and store it in the bme280_data structure
 * instance passed by the user.
 *
 * @param[in] sensor_comp : Variable which selects which data to be read from
 * the sensor.
 *
 *@verbatim
 * sensor_comp |   Macros
 * ------------|-------------------
 *     1       | BME280_PRESS
 *     2       | BME280_TEMP
 *     4       | BME280_HUM
 *     7       | BME280_ALL
 *@endverbatim
 *
 * @param[out] comp_data : Structure instance of bme280_data.
 * @param[in] dev        : Structure instance of bme280_dev.
 *
 * @return Result of API execution status
 *
 * @retval   0 -> Success.
 * @retval > 0 -> Warning.
 * @retval < 0 -> Fail.
 *
 */
int8_t bme280_get_sensor_data(uint8_t sensor_comp, struct bme280_data *comp_data, struct bme280_dev *dev);

/*!
 * \ingroup bme280ApiSensorData
 * \page bme280_api_bme280_compensate_data bme280_compensate_data
 * \code
 * int8_t bme280_compensate_data(uint8_t sensor_comp,
 *                             const struct bme280_uncomp_data *uncomp_data,
 *                             struct bme280_data *comp_data,
 *                             struct bme280_calib_data *calib_data);
 * \endcode
 * @details This API is used to compensate the pressure and/or
 * temperature and/or humidity data according to the component selected by the
 * user.
 *
 * @param[in] sensor_comp : Used to select pressure and/or temperature and/or
 *                          humidity.
 * @param[in] uncomp_data : Contains the uncompensated pressure, temperature and
 *                          humidity data.
 * @param[out] comp_data  : Contains the compensated pressure and/or temperature
 *                          and/or humidity data.
 * @param[in] calib_data  : Pointer to bme280_calib_data
 *
 * @return Result of API execution status.
 *
 * @retval   0 -> Success.
 * @retval > 0 -> Warning.
 * @retval < 0 -> Fail.
 *
 */
int8_t bme280_compensate_data(uint8_t sensor_comp,
                              const struct bme280_uncomp_data *uncomp_data,
                              struct bme280_data *comp_data,
                              struct bme280_calib_data *calib_data);

/**
 * \ingroup bme280
 * \defgroup bme280ApiSensorDelay Sensor Delay
 * @brief Generic API for measuring sensor delay
 */

/*!
 * \ingroup bme280ApiSensorDelay
 * \page bme280_api_bme280_cal_meas_delay bme280_cal_meas_delay
 * \code
 * uint32_t bme280_cal_meas_delay(uint32_t *max_delay, const struct bme280_settings *settings);
 * \endcode
 *
 * @details This API is used to calculate the maximum delay in microseconds required for the
 * temperature/pressure/humidity(whichever are enabled) measurement to complete.
 * The delay depends upon the number of sensors enabled and their oversampling configuration.
 *
 * @param[out] max_delay  : Delay required in microseconds.
 * @param[in] settings    : Contains the oversampling configurations.
 *
 * @return Result of API execution status.
 *
 * @retval   0 -> Success.
 * @retval > 0 -> Warning.
 * @retval < 0 -> Fail.
 *
 */
int8_t bme280_cal_meas_delay(uint32_t *max_delay, const struct bme280_settings *settings);

#ifdef __cplusplus
}
#endif /* End of CPP guard */
#endif /* _BME280_H */
/** @}*/
