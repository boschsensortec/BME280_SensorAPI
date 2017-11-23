/**\mainpage
 * Copyright (C) 2016 - 2017 Bosch Sensortec GmbH
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright
 * notice, this list of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright
 * notice, this list of conditions and the following disclaimer in the
 * documentation and/or other materials provided with the distribution.
 *
 * Neither the name of the copyright holder nor the names of the
 * contributors may be used to endorse or promote products derived from
 * this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND
 * CONTRIBUTORS "AS IS" AND ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL COPYRIGHT HOLDER
 * OR CONTRIBUTORS BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY,
 * OR CONSEQUENTIAL DAMAGES(INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE
 *
 * The information provided is believed to be accurate and reliable.
 * The copyright holder assumes no responsibility
 * for the consequences of use
 * of such information nor for any infringement of patents or
 * other rights of third parties which may result from its use.
 * No license is granted by implication or otherwise under any patent or
 * patent rights of the copyright holder.
 *
 * File		bme280_selftest.c
 * Date		21 Nov 2017
 * Version	1.0.0
 *
 */

#include "bme280_selftest.h"

#define BME280_CRC_DATA_ADDR	UINT8_C(0xE8)
#define BME280_CRC_DATA_LEN	UINT8_C(1)
#define BME280_CRC_CALIB1_ADDR	UINT8_C(0x88)
#define BME280_CRC_CALIB1_LEN	UINT8_C(26)
#define BME280_CRC_CALIB2_ADDR	UINT8_C(0xE1)
#define BME280_CRC_CALIB2_LEN	UINT8_C(7)

/*!
 * @brief This API calculates the CRC
 *
 * @param[in] mem_values : reg_data parameter to calculate CRC
 * @param[in] mem_length : Parameter to calculate CRC
 *
 * @return Result of API execution status
 * @retval zero -> Success / +ve value -> Warning / -ve value -> Error
 */
static uint8_t crc_calculate(uint8_t *mem_values, uint8_t mem_length);

/*!
 * @brief This API reads the stored CRC and then compare with calculated CRC
 *
 * @param[in] dev : Structure instance of bme280_dev.
 *
 * @return Result of API execution status
 * @retval zero -> self test success / +ve value -> warning(self test fail)
 */
int8_t bme280_crc_selftest(const struct bme280_dev *dev)
{
	int8_t rslt;
	uint8_t reg_addr;
	uint8_t reg_data[64];

	uint8_t stored_crc = 0;
	uint8_t calculated_crc = 0;

	/* Read stored crc value from register */
	reg_addr = BME280_CRC_DATA_ADDR;
	rslt = bme280_get_regs(reg_addr, reg_data, BME280_CRC_DATA_LEN, dev);
	if (rslt == BME280_OK) {
		stored_crc = reg_data[0];
		/* Calculated CRC value with calibration register */
		reg_addr = BME280_CRC_CALIB1_ADDR;
		rslt = bme280_get_regs(reg_addr, &reg_data[0], BME280_CRC_CALIB1_LEN, dev);
		if (rslt == BME280_OK) {
			reg_addr = BME280_CRC_CALIB2_ADDR;
			rslt = bme280_get_regs(reg_addr, &reg_data[BME280_CRC_CALIB1_LEN], BME280_CRC_CALIB2_LEN, dev);
			if (rslt == BME280_OK) {
				calculated_crc = crc_calculate(reg_data, BME280_CRC_CALIB1_LEN + BME280_CRC_CALIB2_LEN);
				/* Validate CRC */
				if (stored_crc == calculated_crc)
					rslt = BME280_OK;
				else
					rslt = BME280_W_SELF_TEST_FAIL;
			}
		}
	}

	return rslt;
}

/*!
 * @brief This API calculates the CRC
 *
 * @param[in] mem_values : reg_data parameter to calculate CRC
 * @param[in] mem_length : Parameter to calculate CRC
 *
 * @return Result of API execution status
 * @retval zero -> Success / +ve value -> Warning / -ve value -> Error
 */
static uint8_t crc_calculate(uint8_t *mem_values, uint8_t mem_length)
{
	uint32_t crc_reg = 0xFF;
	uint8_t polynomial = 0x1D;
	uint8_t bitNo, index;
	uint8_t din = 0;

	for (index = 0; index < mem_length; index++) {
		for (bitNo = 0; bitNo < 8; bitNo++) {
			if (((crc_reg & 0x80) > 0) ^ ((mem_values[index] & 0x80) > 0))
				din = 1;
			else
				din = 0;

			/* Truncate 8th bit for crc_reg and mem_values */
			crc_reg = (uint32_t)((crc_reg & 0x7F) << 1);
			mem_values[index] = (uint8_t)((mem_values[index] & 0x7F) << 1);
			crc_reg = (uint32_t)(crc_reg ^ (polynomial * din));
		}
	}

	return (uint8_t)(crc_reg ^ 0xFF);
}
