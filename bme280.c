/*
****************************************************************************
* Copyright (C) 2013 - 2014 Bosch Sensortec GmbH
*
* bme280.c
* Date: 2014/12/12
* Revision: 2.0.3(Pressure and Temperature compensation code revision is 1.1
*               and Humidity compensation code revision is 1.0)
*
* Usage: Sensor Driver file for BME280 sensor
*
****************************************************************************
* License:
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are met:
*
*   Redistributions of source code must retain the above copyright
*   notice, this list of conditions and the following disclaimer.
*
*   Redistributions in binary form must reproduce the above copyright
*   notice, this list of conditions and the following disclaimer in the
*   documentation and/or other materials provided with the distribution.
*
*   Neither the name of the copyright holder nor the names of the
*   contributors may be used to endorse or promote products derived from
*   this software without specific prior written permission.
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
**************************************************************************/

#include "bme280.h"
static struct bme280_t *p_bme280; /**< pointer to BME280 */

/*!
 *	@brief This function is used for initialize
 *	the bus read and bus write functions
 *  and assign the chip id and I2C address of the BME280 sensor
 *	chip id is read in the register 0xD0 bit from 0 to 7
 *
 *	 @param bme280 structure pointer.
 *
 *	@note While changing the parameter of the bme280_t
 *	@note consider the following point:
 *	Changing the reference value of the parameter
 *	will changes the local copy or local reference
 *	make sure your changes will not
 *	affect the reference value of the parameter
 *	(Better case don't change the reference value of the parameter)
 *
 *
 *
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
 *
*/
BME280_RETURN_FUNCTION_TYPE bme280_init(struct bme280_t *bme280)
{
	/* used to return the communication result*/
	BME280_RETURN_FUNCTION_TYPE com_rslt = ERROR;
	u8 v_data_u8 = BME280_ZERO_U8X;
	p_bme280 = bme280;
	/* assign BME280 ptr */
	com_rslt = p_bme280->BME280_BUS_READ_FUNC(p_bme280->dev_addr,
	BME280_CHIP_ID_REG, &v_data_u8, BME280_ONE_U8X);
	/* read Chip Id */
	p_bme280->chip_id = v_data_u8;

	bme280_get_calib_param();
	/* readout bme280 calibparam structure */
	return com_rslt;
}
/*!
 *	@brief This API is used to read uncompensated temperature
 *	in the registers 0xFA, 0xFB and 0xFC
 *	@note 0xFA -> MSB -> bit from 0 to 7
 *	@note 0xFB -> LSB -> bit from 0 to 7
 *	@note 0xFC -> LSB -> bit from 4 to 7
 *
 * @param v_uncomp_temperature_s32 : The value of uncompensated temperature
 *
 *
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
 *
*/
BME280_RETURN_FUNCTION_TYPE bme280_read_uncomp_temperature(
s32 *v_uncomp_temperature_s32)
{
	/* used to return the communication result*/
	BME280_RETURN_FUNCTION_TYPE com_rslt = ERROR;
	/* Array holding the MSB and LSb value
	a_data_u8r[0] - Temperature MSB
	a_data_u8r[1] - Temperature LSB
	a_data_u8r[2] - Temperature LSB
	*/
	u8 a_data_u8r[ARRAY_SIZE_THREE] = {
	BME280_ZERO_U8X, BME280_ZERO_U8X, BME280_ZERO_U8X};
	/* check the p_bme280 structure pointer as NULL*/
	if (p_bme280 == BME280_NULL) {
		return E_BME280_NULL_PTR;
		} else {
			com_rslt = p_bme280->BME280_BUS_READ_FUNC(
			p_bme280->dev_addr,
			BME280_TEMPERATURE_MSB_REG,
			a_data_u8r, BME280_THREE_U8X);
			*v_uncomp_temperature_s32 = (s32)(((
			(u32) (a_data_u8r[INDEX_ZERO]))
			<< SHIFT_LEFT_12_POSITION) |
			(((u32)(a_data_u8r[INDEX_ONE]))
			<< SHIFT_LEFT_4_POSITION)
			| ((u32)a_data_u8r[INDEX_TWO] >>
			SHIFT_RIGHT_4_POSITION));
		}
	return com_rslt;
}
/*!
 * @brief Reads actual temperature from uncompensated temperature
 * @note Returns the value in 0.01 degree Centigrade
 * Output value of "5123" equals 51.23 DegC.
 *
 *
 *
 *  @param  v_uncomp_temperature_s32 : value of uncompensated temperature
 *
 *
 *  @return Returns the actual temperature
 *
*/
s32 bme280_compensate_T_int32(s32 v_uncomp_temperature_s32)
{
	s32 v_x1_u32r = BME280_ZERO_U8X;
	s32 v_x2_u32r = BME280_ZERO_U8X;
	s32 temperature = BME280_ZERO_U8X;

	v_x1_u32r  = ((((v_uncomp_temperature_s32
	>> SHIFT_RIGHT_3_POSITION) - ((s32)
	p_bme280->cal_param.dig_T1 << SHIFT_LEFT_1_POSITION))) *
	((s32)p_bme280->cal_param.dig_T2))
	>> SHIFT_RIGHT_11_POSITION;
	v_x2_u32r  = (((((v_uncomp_temperature_s32
	>> SHIFT_RIGHT_4_POSITION) -
	((s32)p_bme280->cal_param.dig_T1))
	* ((v_uncomp_temperature_s32 >> SHIFT_RIGHT_4_POSITION) -
	((s32)p_bme280->cal_param.dig_T1)))
	>> SHIFT_RIGHT_12_POSITION) *
	((s32)p_bme280->cal_param.dig_T3))
	>> SHIFT_RIGHT_14_POSITION;
	p_bme280->cal_param.t_fine = v_x1_u32r + v_x2_u32r;
	temperature  = (p_bme280->cal_param.t_fine
	* BME280_FIVE_U8X + BME280_ONE_TWENTY_EIGHT_U8X)
	>> SHIFT_RIGHT_8_POSITION;
	return temperature;
}
/*!
 * @brief Reads actual temperature from uncompensated temperature
 * @note Returns the value with 500LSB/DegC centred around 24 DegC
 * output value of "5123" equals(5123/500)+24 = 34.246DegC
 *
 *
 *  @param v_uncomp_temperature_s32: value of uncompensated temperature
 *
 *
 *
 *  @return Return the actual temperature as s16 output
 *
*/
s16 bme280_compensate_T_int32_sixteen_bit_output(
s32 v_uncomp_temperature_s32)
{
	s16 temperature = BME280_ZERO_U8X;
	bme280_compensate_T_int32(v_uncomp_temperature_s32);
	temperature  = (s16)((((
	p_bme280->cal_param.t_fine
	- BME280_TEMP_1_2_2_8_8_0_DATA)
	* BME280_TWENTY_FIVE_U8X)
	+ BME280_ONE_TWENTY_EIGHT_U8X)
	>> SHIFT_RIGHT_8_POSITION);

	return temperature;
}
/*!
 *	@brief This API is used to read uncompensated pressure.
 *	in the registers 0xF7, 0xF8 and 0xF9
 *	@note 0xF7 -> MSB -> bit from 0 to 7
 *	@note 0xF8 -> LSB -> bit from 0 to 7
 *	@note 0xF9 -> LSB -> bit from 4 to 7
 *
 *
 *
 *	@param v_uncomp_pressure_s32 : The value of uncompensated pressure
 *
 *
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
 *
*/
BME280_RETURN_FUNCTION_TYPE bme280_read_uncomp_pressure(
s32 *v_uncomp_pressure_s32)
{
	/* used to return the communication result*/
	BME280_RETURN_FUNCTION_TYPE com_rslt = ERROR;
	/* Array holding the MSB and LSb value
	a_data_u8[0] - Pressure MSB
	a_data_u8[1] - Pressure LSB
	a_data_u8[2] - Pressure LSB
	*/
	u8 a_data_u8[ARRAY_SIZE_THREE] = {
	BME280_ZERO_U8X, BME280_ZERO_U8X, BME280_ZERO_U8X};
	/* check the p_bme280 structure pointer as NULL*/
	if (p_bme280 == BME280_NULL) {
		return E_BME280_NULL_PTR;
		} else {
			com_rslt = p_bme280->BME280_BUS_READ_FUNC(
			p_bme280->dev_addr,
			BME280_PRESSURE_MSB_REG,
			a_data_u8, BME280_THREE_U8X);
			*v_uncomp_pressure_s32 = (s32)((
			((u32)(a_data_u8[INDEX_ZERO]))
			<< SHIFT_LEFT_12_POSITION) |
			(((u32)(a_data_u8[INDEX_ONE]))
			<< SHIFT_LEFT_4_POSITION) |
			((u32)a_data_u8[INDEX_TWO] >>
			SHIFT_RIGHT_4_POSITION));
		}
	return com_rslt;
}
/*!
 * @brief Reads actual pressure from uncompensated pressure
 * @note Returns the value in Pascal(Pa)
 * Output value of "96386" equals 96386 Pa =
 * 963.86 hPa = 963.86 millibar
 *
 *
 *
 *  @param v_uncomp_pressure_s32 : value of uncompensated pressure
 *
 *
 *
 *  @return Return the actual pressure output as u32
 *
*/
u32 bme280_compensate_P_int32(s32 v_uncomp_pressure_s32)
{
	s32 v_x1_u32 = BME280_ZERO_U8X;
	s32 v_x2_u32 = BME280_ZERO_U8X;
	u32 v_pressure_u32 = BME280_ZERO_U8X;

	v_x1_u32 = (((s32)p_bme280->cal_param.t_fine)
	>> SHIFT_RIGHT_1_POSITION) -
	(s32)BME280_PRESSURE_6_4_0_0_0_DATA;
	v_x2_u32 = (((v_x1_u32 >> SHIFT_RIGHT_2_POSITION)
	* (v_x1_u32 >> SHIFT_RIGHT_2_POSITION))
	>> SHIFT_RIGHT_11_POSITION) *
	((s32)p_bme280->cal_param.dig_P6);
	v_x2_u32 = v_x2_u32 + ((v_x1_u32 *
	((s32)p_bme280->cal_param.dig_P5))
	<< SHIFT_LEFT_1_POSITION);
	v_x2_u32 = (v_x2_u32 >> SHIFT_RIGHT_2_POSITION) +
	(((s32)p_bme280->cal_param.dig_P4)
	<< SHIFT_LEFT_16_POSITION);
	v_x1_u32 = (((p_bme280->cal_param.dig_P3
	* (((v_x1_u32 >> SHIFT_RIGHT_2_POSITION) *
	(v_x1_u32 >> SHIFT_RIGHT_2_POSITION))
	>> SHIFT_RIGHT_13_POSITION)) >> SHIFT_RIGHT_3_POSITION) +
	((((s32)p_bme280->cal_param.dig_P2) *
	v_x1_u32) >> SHIFT_RIGHT_1_POSITION))
	>> SHIFT_RIGHT_18_POSITION;
	v_x1_u32 = ((((BME280_PRESSURE_3_2_7_6_8_DATA + v_x1_u32)) *
	((s32)p_bme280->cal_param.dig_P1))
	>> SHIFT_RIGHT_15_POSITION);
	v_pressure_u32 =
	(((u32)(((s32)BME280_PRESSURE_1_0_4_8_5_7_6_DATA)
	- v_uncomp_pressure_s32) -
	(v_x2_u32 >> SHIFT_RIGHT_12_POSITION)))
	* BME280_PRESSURE_3_1_2_5_DATA;
	if (v_pressure_u32
	< BME280_HEX_PRESSURE_8_0_0_0_0_0_0_0_DATA)
		/* Avoid exception caused by division by zero */
		if (v_x1_u32 != BME280_ZERO_U8X)
			v_pressure_u32 =
			(v_pressure_u32 << SHIFT_LEFT_1_POSITION) /
			((u32)v_x1_u32);
		else
			return BME280_ZERO_U8X;
	else
		/* Avoid exception caused by division by zero */
		if (v_x1_u32 != BME280_ZERO_U8X)
			v_pressure_u32 = (v_pressure_u32
			/ (u32)v_x1_u32) * BME280_TWO_U8X;
		else
			return BME280_ZERO_U8X;

		v_x1_u32 = (((s32)p_bme280->cal_param.dig_P9) *
		((s32)(((v_pressure_u32 >> SHIFT_RIGHT_3_POSITION)
		* (v_pressure_u32 >> SHIFT_RIGHT_3_POSITION))
		>> SHIFT_RIGHT_13_POSITION)))
		>> SHIFT_RIGHT_12_POSITION;
		v_x2_u32 = (((s32)(v_pressure_u32
		>> SHIFT_RIGHT_2_POSITION)) *
		((s32)p_bme280->cal_param.dig_P8))
		>> SHIFT_RIGHT_13_POSITION;
		v_pressure_u32 = (u32)((s32)v_pressure_u32 +
		((v_x1_u32 + v_x2_u32 + p_bme280->cal_param.dig_P7)
		>> SHIFT_RIGHT_4_POSITION));

	return v_pressure_u32;
}
/*!
 *	@brief This API is used to read uncompensated humidity.
 *	in the registers 0xF7, 0xF8 and 0xF9
 *	@note 0xFD -> MSB -> bit from 0 to 7
 *	@note 0xFE -> LSB -> bit from 0 to 7
 *
 *
 *
 *	@param v_uncomp_humidity_s32 : The value of uncompensated humidity
 *
 *
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
 *
*/
BME280_RETURN_FUNCTION_TYPE bme280_read_uncomp_humidity(
s32 *v_uncomp_humidity_s32)
{
	/* used to return the communication result*/
	BME280_RETURN_FUNCTION_TYPE com_rslt = ERROR;
	/* Array holding the MSB and LSb value
	a_data_u8[0] - Humidity MSB
	a_data_u8[1] - Humidity LSB
	*/
	u8 a_data_u8[ARRAY_SIZE_TWO] = {
	BME280_ZERO_U8X, BME280_ZERO_U8X};
	/* check the p_bme280 structure pointer as NULL*/
	if (p_bme280 == BME280_NULL) {
		return E_BME280_NULL_PTR;
		} else {
			com_rslt = p_bme280->BME280_BUS_READ_FUNC(
			p_bme280->dev_addr,
			BME280_HUMIDITY_MSB_REG, a_data_u8, BME280_TWO_U8X);
			*v_uncomp_humidity_s32 = (s32)(
			(((u32)(a_data_u8[INDEX_ZERO]))
			<< SHIFT_LEFT_8_POSITION)|
			((u32)(a_data_u8[INDEX_ONE])));
		}
	return com_rslt;
}
/*!
 * @brief Reads actual humidity from uncompensated humidity
 * @note Returns the value in %rH as unsigned 32bit integer
 * in Q22.10 format(22 integer 10 fractional bits).
 * @note An output value of 42313
 * represents 42313 / 1024 = 41.321 %rH
 *
 *
 *
 *  @param  v_uncomp_humidity_s32: value of uncompensated humidity
 *
 *  @return Return the actual relative humidity output as u32
 *
*/
u32 bme280_compensate_H_int32(s32 v_uncomp_humidity_s32)
{
	s32 v_x1_u32;
	v_x1_u32 = (p_bme280->cal_param.t_fine
	- ((s32)BME280_HUMIDITY_7_6_8_0_0_DATA));
	v_x1_u32 = (((((v_uncomp_humidity_s32
	<< SHIFT_LEFT_14_POSITION) -
	(((s32)p_bme280->cal_param.dig_H4)
	<< SHIFT_LEFT_20_POSITION) -
	(((s32)p_bme280->cal_param.dig_H5) * v_x1_u32)) +
	((s32)BME280_HUMIDITY_1_6_3_8_4_DATA))
	>> SHIFT_RIGHT_15_POSITION) *
	(((((((v_x1_u32 *
	((s32)p_bme280->cal_param.dig_H6))
	>> SHIFT_RIGHT_10_POSITION) *
	(((v_x1_u32 * ((s32)p_bme280->cal_param.dig_H3))
	>> SHIFT_RIGHT_11_POSITION) +
	((s32)BME280_HUMIDITY_3_2_7_6_8_DATA)))
	>> SHIFT_RIGHT_10_POSITION) +
	((s32)BME280_HUMIDITY_2_0_9_7_1_5_2_DATA)) *
	((s32)p_bme280->cal_param.dig_H2)
	+ BME280_HUMIDITY_8_1_9_2_DATA)
	>> SHIFT_RIGHT_14_POSITION));
	v_x1_u32 = (v_x1_u32 - (((((v_x1_u32
	>> SHIFT_RIGHT_15_POSITION) *
	(v_x1_u32 >> SHIFT_RIGHT_15_POSITION))
	>> SHIFT_RIGHT_7_POSITION) *
	((s32)p_bme280->cal_param.dig_H1))
	>> SHIFT_RIGHT_4_POSITION));
	v_x1_u32 = (v_x1_u32 < BME280_ZERO_U8X
	? BME280_ZERO_U8X : v_x1_u32);
	v_x1_u32 =
	(v_x1_u32 > BME280_HUMIDITY_4_1_9_4_3_0_4_0_0_DATA ?
	BME280_HUMIDITY_4_1_9_4_3_0_4_0_0_DATA : v_x1_u32);
	return (u32)(v_x1_u32 >> SHIFT_RIGHT_12_POSITION);
}
/*!
 * @brief Reads actual humidity from uncompensated humidity
 * @note Returns the value in %rH as unsigned 16bit integer
 * @note An output value of 42313
 * represents 42313/512 = 82.643 %rH
 *
 *
 *
 *  @param v_uncomp_humidity_s32: value of uncompensated humidity
 *
 *
 *  @return Return the actual relative humidity output as u16
 *
*/
u16 bme280_compensate_H_int32_sixteen_bit_output(s32 v_uncomp_humidity_s32)
{
	u32 v_x1_u32;
	u16 v_x2_u32;
	v_x1_u32 =  bme280_compensate_H_int32(v_uncomp_humidity_s32);
	v_x2_u32 = (u16)(v_x1_u32 >> SHIFT_RIGHT_1_POSITION);
	return v_x2_u32;
}
/*!
 * @brief This API used to read uncompensated
 * pressure,temperature and humidity
 *
 *
 *
 *
 *  @param  v_uncomp_pressure_s32: The value of uncompensated pressure.
 *  @param  v_uncomp_temperature_s32: The value of uncompensated temperature
 *  @param  v_uncomp_humidity_s32: The value of uncompensated humidity.
 *
 *
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
 *
*/
BME280_RETURN_FUNCTION_TYPE bme280_read_uncomp_pressure_temperature_humidity(
s32 *v_uncomp_pressure_s32,
s32 *v_uncomp_temperature_s32, s32 *v_uncomp_humidity_s32)
{
	/* used to return the communication result*/
	BME280_RETURN_FUNCTION_TYPE com_rslt = ERROR;
	/* Array holding the MSB and LSb value of
	a_data_u8[0] - Pressure MSB
	a_data_u8[1] - Pressure LSB
	a_data_u8[1] - Pressure LSB
	a_data_u8[1] - Temperature MSB
	a_data_u8[1] - Temperature LSB
	a_data_u8[1] - Temperature LSB
	a_data_u8[1] - Humidity MSB
	a_data_u8[1] - Humidity LSB
	*/
	u8 a_data_u8[ARRAY_SIZE_EIGHT] = {
	BME280_ZERO_U8X, BME280_ZERO_U8X,
	BME280_ZERO_U8X, BME280_ZERO_U8X,
	BME280_ZERO_U8X, BME280_ZERO_U8X,
	BME280_ZERO_U8X, BME280_ZERO_U8X};
	/* check the p_bme280 structure pointer as NULL*/
	if (p_bme280 == BME280_NULL) {
		return E_BME280_NULL_PTR;
		} else {
			com_rslt = p_bme280->BME280_BUS_READ_FUNC(
			p_bme280->dev_addr,
			BME280_PRESSURE_MSB_REG,
			a_data_u8, BME280_EIGHT_U8X);
			/*Pressure*/
			*v_uncomp_pressure_s32 = (s32)((
			((u32)(a_data_u8[INDEX_ZERO]))
			<< SHIFT_LEFT_12_POSITION) |
			(((u32)(a_data_u8[INDEX_ONE]))
			<< SHIFT_LEFT_4_POSITION) |
			((u32)a_data_u8[INDEX_TWO] >>
			SHIFT_RIGHT_4_POSITION));

			/* Temperature */
			*v_uncomp_temperature_s32 = (s32)(((
			(u32) (a_data_u8[INDEX_THREE]))
			<< SHIFT_LEFT_12_POSITION) |
			(((u32)(a_data_u8[INDEX_FOUR]))
			<< SHIFT_LEFT_4_POSITION)
			| ((u32)a_data_u8[INDEX_FIVE]
			>> SHIFT_RIGHT_4_POSITION));

			/*Humidity*/
			*v_uncomp_humidity_s32 = (s32)((
			((u32)(a_data_u8[INDEX_SIX]))
			<< SHIFT_LEFT_8_POSITION)|
			((u32)(a_data_u8[INDEX_SEVEN])));
		}
	return com_rslt;
}
/*!
 * @brief This API used to read true pressure, temperature and humidity
 *
 *
 *
 *
 *	@param  v_pressure_u32 : The value of compensated pressure.
 *	@param  v_temperature_s32 : The value of compensated temperature.
 *	@param  v_humidity_u32 : The value of compensated humidity.
 *
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
 *
*/
BME280_RETURN_FUNCTION_TYPE bme280_read_pressure_temperature_humidity(
u32 *v_pressure_u32, s32 *v_temperature_s32, u32 *v_humidity_u32)
{
	/* used to return the communication result*/
	BME280_RETURN_FUNCTION_TYPE com_rslt = ERROR;
	s32 v_uncomp_pressure_s32 = BME280_ZERO_U8X;
	s32 v_uncom_temperature_s32 = BME280_ZERO_U8X;
	s32 v_uncom_humidity_s32 = BME280_ZERO_U8X;
	/* check the p_bme280 structure pointer as NULL*/
	if (p_bme280 == BME280_NULL) {
		return E_BME280_NULL_PTR;
		} else {
			/* read the uncompensated pressure,
			temperature and humidity*/
			com_rslt =
			bme280_read_uncomp_pressure_temperature_humidity(
			&v_uncomp_pressure_s32, &v_uncom_temperature_s32,
			&v_uncom_humidity_s32);
			/* read the true pressure, temperature and humidity*/
			*v_temperature_s32 = bme280_compensate_T_int32(
			v_uncom_temperature_s32);
			*v_pressure_u32 = bme280_compensate_P_int32(
			v_uncomp_pressure_s32);
			*v_humidity_u32 = bme280_compensate_H_int32(
			v_uncom_humidity_s32);
		}
	return com_rslt;
}
/*!
 *	@brief This API is used to
 *	calibration parameters used for calculation in the registers
 *
 *  parameter | Register address |   bit
 *------------|------------------|----------------
 *	dig_T1    |  0x88 and 0x89   | from 0 : 7 to 8: 15
 *	dig_T2    |  0x8A and 0x8B   | from 0 : 7 to 8: 15
 *	dig_T3    |  0x8C and 0x8D   | from 0 : 7 to 8: 15
 *	dig_P1    |  0x8E and 0x8F   | from 0 : 7 to 8: 15
 *	dig_P2    |  0x90 and 0x91   | from 0 : 7 to 8: 15
 *	dig_P3    |  0x92 and 0x93   | from 0 : 7 to 8: 15
 *	dig_P4    |  0x94 and 0x95   | from 0 : 7 to 8: 15
 *	dig_P5    |  0x96 and 0x97   | from 0 : 7 to 8: 15
 *	dig_P6    |  0x98 and 0x99   | from 0 : 7 to 8: 15
 *	dig_P7    |  0x9A and 0x9B   | from 0 : 7 to 8: 15
 *	dig_P8    |  0x9C and 0x9D   | from 0 : 7 to 8: 15
 *	dig_P9    |  0x9E and 0x9F   | from 0 : 7 to 8: 15
 *	dig_H1    |         0xA1     | from 0 to 7
 *	dig_H2    |  0xE1 and 0xE2   | from 0 : 7 to 8: 15
 *	dig_H3    |         0xE3     | from 0 to 7
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
 *
*/
BME280_RETURN_FUNCTION_TYPE bme280_get_calib_param()
{
	/* used to return the communication result*/
	BME280_RETURN_FUNCTION_TYPE com_rslt = ERROR;
	u8 a_data_u8[ARRAY_SIZE_TWENTY_SIX] = {
	BME280_ZERO_U8X, BME280_ZERO_U8X,
	BME280_ZERO_U8X, BME280_ZERO_U8X, BME280_ZERO_U8X,
	BME280_ZERO_U8X, BME280_ZERO_U8X, BME280_ZERO_U8X,
	BME280_ZERO_U8X, BME280_ZERO_U8X, BME280_ZERO_U8X,
	BME280_ZERO_U8X, BME280_ZERO_U8X, BME280_ZERO_U8X,
	BME280_ZERO_U8X, BME280_ZERO_U8X, BME280_ZERO_U8X,
	BME280_ZERO_U8X, BME280_ZERO_U8X, BME280_ZERO_U8X,
	BME280_ZERO_U8X, BME280_ZERO_U8X, BME280_ZERO_U8X,
	BME280_ZERO_U8X, BME280_ZERO_U8X, BME280_ZERO_U8X};
	/* check the p_bme280 structure pointer as NULL*/
	if (p_bme280 == BME280_NULL) {
		return E_BME280_NULL_PTR;
		} else {
			com_rslt = p_bme280->BME280_BUS_READ_FUNC(
			p_bme280->dev_addr,
			BME280_DIG_T1_LSB_REG,
			a_data_u8, BME280_TWENTY_SIX_U8X);

			p_bme280->cal_param.dig_T1 = (u16)(((
			(u16)((u8)a_data_u8[INDEX_ONE])) <<
			SHIFT_LEFT_8_POSITION) | a_data_u8[INDEX_ZERO]);
			p_bme280->cal_param.dig_T2 = (s16)(((
			(s16)((s8)a_data_u8[INDEX_THREE])) <<
			SHIFT_LEFT_8_POSITION) | a_data_u8[INDEX_TWO]);
			p_bme280->cal_param.dig_T3 = (s16)(((
			(s16)((s8)a_data_u8[INDEX_FIVE])) <<
			SHIFT_LEFT_8_POSITION) | a_data_u8[INDEX_FOUR]);
			p_bme280->cal_param.dig_P1 = (u16)(((
			(u16)((u8)a_data_u8[INDEX_SEVEN])) <<
			SHIFT_LEFT_8_POSITION) | a_data_u8[INDEX_SIX]);
			p_bme280->cal_param.dig_P2 = (s16)(((
			(s16)((s8)a_data_u8[INDEX_NINE])) <<
			SHIFT_LEFT_8_POSITION) | a_data_u8[INDEX_EIGHT]);
			p_bme280->cal_param.dig_P3 = (s16)(((
			(s16)((s8)a_data_u8[INDEX_ELEVEN])) <<
			SHIFT_LEFT_8_POSITION) | a_data_u8[INDEX_TEN]);
			p_bme280->cal_param.dig_P4 = (s16)(((
			(s16)((s8)a_data_u8[INDEX_THIRTEEN])) <<
			SHIFT_LEFT_8_POSITION) | a_data_u8[INDEX_TWELVE]);
			p_bme280->cal_param.dig_P5 = (s16)(((
			(s16)((s8)a_data_u8[INDEX_FIVETEEN])) <<
			SHIFT_LEFT_8_POSITION) | a_data_u8[INDEX_FOURTEEN]);
			p_bme280->cal_param.dig_P6 = (s16)(((
			(s16)((s8)a_data_u8[INDEX_SEVENTEEN])) <<
			SHIFT_LEFT_8_POSITION) | a_data_u8[INDEX_SIXTEEN]);
			p_bme280->cal_param.dig_P7 = (s16)(((
			(s16)((s8)a_data_u8[INDEX_NINETEEN])) <<
			SHIFT_LEFT_8_POSITION) | a_data_u8[INDEX_EIGHTEEN]);
			p_bme280->cal_param.dig_P8 = (s16)(((
			(s16)((s8)a_data_u8[INDEX_TWENTY_ONE])) <<
			SHIFT_LEFT_8_POSITION) | a_data_u8[INDEX_TWENTY]);
			p_bme280->cal_param.dig_P9 = (s16)(((
			(s16)((s8)a_data_u8[INDEX_TWENTY_THREE])) <<
			SHIFT_LEFT_8_POSITION) | a_data_u8[INDEX_TWENTY_TWO]);
			p_bme280->cal_param.dig_H1 =
			a_data_u8[INDEX_TWENTY_FIVE];
			com_rslt += p_bme280->BME280_BUS_READ_FUNC(
			p_bme280->dev_addr,
			BME280_DIG_H2_LSB_REG, a_data_u8, BME280_SEVEN_U8X);
			p_bme280->cal_param.dig_H2 = (s16)(((
			(s16)((s8)a_data_u8[INDEX_ONE])) <<
			SHIFT_LEFT_8_POSITION) | a_data_u8[INDEX_ZERO]);
			p_bme280->cal_param.dig_H3 = a_data_u8[INDEX_TWO];
			p_bme280->cal_param.dig_H4 = (s16)(((
			(s16)((s8)a_data_u8[INDEX_THREE])) <<
			SHIFT_LEFT_4_POSITION) |
			(((u8)BME280_HEX_CALIB_0_F_DATA)
			& a_data_u8[INDEX_FOUR]));
			p_bme280->cal_param.dig_H5 = (s16)(((
			(s16)((s8)a_data_u8[INDEX_FIVE])) <<
			SHIFT_LEFT_4_POSITION) | (a_data_u8[INDEX_FOUR] >>
			SHIFT_RIGHT_4_POSITION));
			p_bme280->cal_param.dig_H6 = (s8)a_data_u8[INDEX_SIX];
		}
	return com_rslt;
}
/*!
 *	@brief This API is used to get
 *	the temperature oversampling setting in the register 0xF4
 *	bits from 5 to 7
 *
 *	value               |   Temperature oversampling
 * ---------------------|---------------------------------
 *	0x00                | Skipped
 *	0x01                | BME280_OVERSAMP_1X
 *	0x02                | BME280_OVERSAMP_2X
 *	0x03                | BME280_OVERSAMP_4X
 *	0x04                | BME280_OVERSAMP_8X
 *	0x05,0x06 and 0x07  | BME280_OVERSAMP_16X
 *
 *
 *  @param v_value_u8 : The value of temperature over sampling
 *
 *
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
 *
*/
BME280_RETURN_FUNCTION_TYPE bme280_get_oversamp_temperature(
u8 *v_value_u8)
{
	/* used to return the communication result*/
	BME280_RETURN_FUNCTION_TYPE com_rslt = ERROR;
	u8 v_data_u8 = BME280_ZERO_U8X;
	/* check the p_bme280 structure pointer as NULL*/
	if (p_bme280 == BME280_NULL) {
		return E_BME280_NULL_PTR;
		} else {
			com_rslt = p_bme280->BME280_BUS_READ_FUNC(
			p_bme280->dev_addr,
			BME280_CTRL_MEAS_REG_OVERSAMP_TEMPERATURE__REG,
			&v_data_u8, BME280_ONE_U8X);
			*v_value_u8 = BME280_GET_BITSLICE(v_data_u8,
			BME280_CTRL_MEAS_REG_OVERSAMP_TEMPERATURE);

			p_bme280->oversamp_temperature = *v_value_u8;
		}
	return com_rslt;
}
/*!
 *	@brief This API is used to set
 *	the temperature oversampling setting in the register 0xF4
 *	bits from 5 to 7
 *
 *	value               |   Temperature oversampling
 * ---------------------|---------------------------------
 *	0x00                | Skipped
 *	0x01                | BME280_OVERSAMP_1X
 *	0x02                | BME280_OVERSAMP_2X
 *	0x03                | BME280_OVERSAMP_4X
 *	0x04                | BME280_OVERSAMP_8X
 *	0x05,0x06 and 0x07  | BME280_OVERSAMP_16X
 *
 *
 *  @param v_value_u8 : The value of temperature over sampling
 *
 *
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
 *
*/
BME280_RETURN_FUNCTION_TYPE bme280_set_oversamp_temperature(
u8 v_value_u8)
{
	/* used to return the communication result*/
	BME280_RETURN_FUNCTION_TYPE com_rslt = ERROR;
	u8 v_data_u8 = BME280_ZERO_U8X;
	u8 v_prev_pow_mode_u8 = BME280_ZERO_U8X;
	u8 v_pre_ctrl_hum_value_u8 = BME280_ZERO_U8X;
	u8 v_pre_config_value_u8 = BME280_ZERO_U8X;
	/* check the p_bme280 structure pointer as NULL*/
	if (p_bme280 == BME280_NULL) {
		return E_BME280_NULL_PTR;
		} else {
			v_data_u8 = p_bme280->ctrl_meas_reg;
			v_data_u8 =
			BME280_SET_BITSLICE(v_data_u8,
			BME280_CTRL_MEAS_REG_OVERSAMP_TEMPERATURE, v_value_u8);
			com_rslt = bme280_get_power_mode(&v_prev_pow_mode_u8);
			if (v_prev_pow_mode_u8 != BME280_SLEEP_MODE) {
				com_rslt += bme280_set_soft_rst();
				p_bme280->delay_msec(BME280_3MS_DELAY);
				/* write previous value
				of configuration register*/
				v_pre_config_value_u8 = p_bme280->config_reg;
				com_rslt += bme280_write_register(
					BME280_CONFIG_REG,
				&v_pre_config_value_u8, BME280_ONE_U8X);
				/* write previous value
				of humidity oversampling*/
				v_pre_ctrl_hum_value_u8 =
				p_bme280->ctrl_hum_reg;
				com_rslt += bme280_write_register(
					BME280_CTRL_HUMIDITY_REG,
				&v_pre_ctrl_hum_value_u8, BME280_ONE_U8X);
				/* write previous and updated value
				of configuration register*/
				com_rslt += bme280_write_register(
					BME280_CTRL_MEAS_REG,
				&v_data_u8, BME280_ONE_U8X);
			} else {
				com_rslt = p_bme280->BME280_BUS_WRITE_FUNC(
				p_bme280->dev_addr,
				BME280_CTRL_MEAS_REG_OVERSAMP_TEMPERATURE__REG,
				&v_data_u8, BME280_ONE_U8X);
			}
				p_bme280->oversamp_temperature = v_value_u8;
				/* read the control measurement register value*/
				com_rslt = bme280_read_register(
					BME280_CTRL_MEAS_REG,
				&v_data_u8, BME280_ONE_U8X);
				p_bme280->ctrl_meas_reg = v_data_u8;
				/* read the control humidity register value*/
				com_rslt += bme280_read_register(
					BME280_CTRL_HUMIDITY_REG,
				&v_data_u8, BME280_ONE_U8X);
				p_bme280->ctrl_hum_reg = v_data_u8;
				/* read the control
				configuration register value*/
				com_rslt += bme280_read_register(
					BME280_CONFIG_REG,
				&v_data_u8, BME280_ONE_U8X);
				p_bme280->config_reg = v_data_u8;
		}
	return com_rslt;
}
/*!
 *	@brief This API is used to get
 *	the pressure oversampling setting in the register 0xF4
 *	bits from 2 to 4
 *
 *	value              | Pressure oversampling
 * --------------------|--------------------------
 *	0x00               | Skipped
 *	0x01               | BME280_OVERSAMP_1X
 *	0x02               | BME280_OVERSAMP_2X
 *	0x03               | BME280_OVERSAMP_4X
 *	0x04               | BME280_OVERSAMP_8X
 *	0x05,0x06 and 0x07 | BME280_OVERSAMP_16X
 *
 *
 *  @param v_value_u8 : The value of pressure oversampling
 *
 *
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
 *
*/
BME280_RETURN_FUNCTION_TYPE bme280_get_oversamp_pressure(
u8 *v_value_u8)
{
	/* used to return the communication result*/
	BME280_RETURN_FUNCTION_TYPE com_rslt = ERROR;
	u8 v_data_u8 = BME280_ZERO_U8X;
	/* check the p_bme280 structure pointer as NULL*/
	if (p_bme280 == BME280_NULL) {
		return E_BME280_NULL_PTR;
		} else {
			com_rslt = p_bme280->BME280_BUS_READ_FUNC(
			p_bme280->dev_addr,
			BME280_CTRL_MEAS_REG_OVERSAMP_PRESSURE__REG,
			&v_data_u8, BME280_ONE_U8X);
			*v_value_u8 = BME280_GET_BITSLICE(
			v_data_u8,
			BME280_CTRL_MEAS_REG_OVERSAMP_PRESSURE);

			p_bme280->oversamp_pressure = *v_value_u8;
		}
	return com_rslt;
}
/*!
 *	@brief This API is used to set
 *	the pressure oversampling setting in the register 0xF4
 *	bits from 2 to 4
 *
 *	value              | Pressure oversampling
 * --------------------|--------------------------
 *	0x00               | Skipped
 *	0x01               | BME280_OVERSAMP_1X
 *	0x02               | BME280_OVERSAMP_2X
 *	0x03               | BME280_OVERSAMP_4X
 *	0x04               | BME280_OVERSAMP_8X
 *	0x05,0x06 and 0x07 | BME280_OVERSAMP_16X
 *
 *
 *  @param v_value_u8 : The value of pressure oversampling
 *
 *
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
 *
*/
BME280_RETURN_FUNCTION_TYPE bme280_set_oversamp_pressure(
u8 v_value_u8)
{
	/* used to return the communication result*/
	BME280_RETURN_FUNCTION_TYPE com_rslt = ERROR;
	u8 v_data_u8 = BME280_ZERO_U8X;
	u8 v_prev_pow_mode_u8 = BME280_ZERO_U8X;
	u8 v_pre_ctrl_hum_value_u8 = BME280_ZERO_U8X;
	u8 v_pre_config_value_u8 = BME280_ZERO_U8X;
	/* check the p_bme280 structure pointer as NULL*/
	if (p_bme280 == BME280_NULL) {
		return E_BME280_NULL_PTR;
		} else {
			v_data_u8 = p_bme280->ctrl_meas_reg;
			v_data_u8 =
			BME280_SET_BITSLICE(v_data_u8,
			BME280_CTRL_MEAS_REG_OVERSAMP_PRESSURE, v_value_u8);
			com_rslt = bme280_get_power_mode(&v_prev_pow_mode_u8);
			if (v_prev_pow_mode_u8 != BME280_SLEEP_MODE) {
				com_rslt += bme280_set_soft_rst();
				p_bme280->delay_msec(BME280_3MS_DELAY);
				/* write previous value of
				configuration register*/
				v_pre_config_value_u8 = p_bme280->config_reg;
				com_rslt = bme280_write_register(
					BME280_CONFIG_REG,
				&v_pre_config_value_u8, BME280_ONE_U8X);
				/* write previous value of
				humidity oversampling*/
				v_pre_ctrl_hum_value_u8 =
				p_bme280->ctrl_hum_reg;
				com_rslt += bme280_write_register(
					BME280_CTRL_HUMIDITY_REG,
				&v_pre_ctrl_hum_value_u8, BME280_ONE_U8X);
				/* write previous and updated value of
				control measurement register*/
				bme280_write_register(
					BME280_CTRL_MEAS_REG,
				&v_data_u8, BME280_ONE_U8X);
			} else {
				com_rslt = p_bme280->BME280_BUS_WRITE_FUNC(
				p_bme280->dev_addr,
				BME280_CTRL_MEAS_REG_OVERSAMP_PRESSURE__REG,
				&v_data_u8, BME280_ONE_U8X);
			}
				p_bme280->oversamp_pressure = v_value_u8;
				/* read the control measurement register value*/
				com_rslt = bme280_read_register(
					BME280_CTRL_MEAS_REG,
				&v_data_u8, BME280_ONE_U8X);
				p_bme280->ctrl_meas_reg = v_data_u8;
				/* read the control humidity register value*/
				com_rslt += bme280_read_register(
					BME280_CTRL_HUMIDITY_REG,
				&v_data_u8, BME280_ONE_U8X);
				p_bme280->ctrl_hum_reg = v_data_u8;
				/* read the control
				configuration register value*/
				com_rslt += bme280_read_register(
					BME280_CONFIG_REG,
				&v_data_u8, BME280_ONE_U8X);
				p_bme280->config_reg = v_data_u8;
		}
	return com_rslt;
}
/*!
 *	@brief This API is used to get
 *	the humidity oversampling setting in the register 0xF2
 *	bits from 0 to 2
 *
 *	value               | Humidity oversampling
 * ---------------------|-------------------------
 *	0x00                | Skipped
 *	0x01                | BME280_OVERSAMP_1X
 *	0x02                | BME280_OVERSAMP_2X
 *	0x03                | BME280_OVERSAMP_4X
 *	0x04                | BME280_OVERSAMP_8X
 *	0x05,0x06 and 0x07  | BME280_OVERSAMP_16X
 *
 *
 *  @param  v_value_u8 : The value of humidity over sampling
 *
 *
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
 *
*/
BME280_RETURN_FUNCTION_TYPE bme280_get_oversamp_humidity(
u8 *v_value_u8)
{
	/* used to return the communication result*/
	BME280_RETURN_FUNCTION_TYPE com_rslt = ERROR;
	u8 v_data_u8 = BME280_ZERO_U8X;
	/* check the p_bme280 structure pointer as NULL*/
	if (p_bme280 == BME280_NULL) {
		return E_BME280_NULL_PTR;
		} else {
			com_rslt = p_bme280->BME280_BUS_READ_FUNC(
			p_bme280->dev_addr,
			BME280_CTRL_HUMIDITY_REG_OVERSAM_HUMIDITY__REG,
			&v_data_u8, BME280_ONE_U8X);
			*v_value_u8 = BME280_GET_BITSLICE(
			v_data_u8,
			BME280_CTRL_HUMIDITY_REG_OVERSAM_HUMIDITY);

			p_bme280->oversamp_humidity = *v_value_u8;
		}
	return com_rslt;
}
/*!
 *	@brief This API is used to set
 *	the humidity oversampling setting in the register 0xF2
 *	bits from 0 to 2
 *
 *	value               | Humidity oversampling
 * ---------------------|-------------------------
 *	0x00                | Skipped
 *	0x01                | BME280_OVERSAMP_1X
 *	0x02                | BME280_OVERSAMP_2X
 *	0x03                | BME280_OVERSAMP_4X
 *	0x04                | BME280_OVERSAMP_8X
 *	0x05,0x06 and 0x07  | BME280_OVERSAMP_16X
 *
 *
 *  @param  v_value_u8 : The value of humidity over sampling
 *
 *
 *
 * @note The "BME280_CTRL_HUMIDITY_REG_OVERSAM_HUMIDITY"
 * register sets the humidity
 * data acquisition options of the device.
 * @note changes to this registers only become
 * effective after a write operation to
 * "BME280_CTRL_MEAS_REG" register.
 * @note In the code automated reading and writing of
 *	"BME280_CTRL_HUMIDITY_REG_OVERSAM_HUMIDITY"
 * @note register first set the
 * "BME280_CTRL_HUMIDITY_REG_OVERSAM_HUMIDITY"
 *  and then read and write
 *  the "BME280_CTRL_MEAS_REG" register in the function.
 *
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
 *
*/
BME280_RETURN_FUNCTION_TYPE bme280_set_oversamp_humidity(
u8 v_value_u8)
{
	/* used to return the communication result*/
	BME280_RETURN_FUNCTION_TYPE com_rslt = ERROR;
	u8 v_data_u8 = BME280_ZERO_U8X;
	u8 pre_ctrl_meas_value = BME280_ZERO_U8X;
	u8 v_pre_config_value_u8 = BME280_ZERO_U8X;
	u8 v_prev_pow_mode_u8 = BME280_ZERO_U8X;
	/* check the p_bme280 structure pointer as NULL*/
	if (p_bme280 == BME280_NULL) {
		return E_BME280_NULL_PTR;
		} else {
			/* write humidity oversampling*/
			v_data_u8 = p_bme280->ctrl_hum_reg;
			v_data_u8 =
			BME280_SET_BITSLICE(v_data_u8,
			BME280_CTRL_HUMIDITY_REG_OVERSAM_HUMIDITY, v_value_u8);
			com_rslt = bme280_get_power_mode(&v_prev_pow_mode_u8);
			if (v_prev_pow_mode_u8 != BME280_SLEEP_MODE) {
				com_rslt += bme280_set_soft_rst();
				p_bme280->delay_msec(BME280_3MS_DELAY);
				/* write previous value of
				configuration register*/
				v_pre_config_value_u8 = p_bme280->config_reg;
				com_rslt += bme280_write_register(
					BME280_CONFIG_REG,
				&v_pre_config_value_u8, BME280_ONE_U8X);
				/* write the value of control humidity*/
				com_rslt += bme280_write_register(
					BME280_CTRL_HUMIDITY_REG,
				&v_data_u8, BME280_ONE_U8X);
				/* write previous value of
				control measurement register*/
				pre_ctrl_meas_value =
				p_bme280->ctrl_meas_reg;
				com_rslt += bme280_write_register(
					BME280_CTRL_MEAS_REG,
				&pre_ctrl_meas_value, BME280_ONE_U8X);
			} else {
				com_rslt +=
				p_bme280->BME280_BUS_WRITE_FUNC(
				p_bme280->dev_addr,
				BME280_CTRL_HUMIDITY_REG_OVERSAM_HUMIDITY__REG,
				&v_data_u8, BME280_ONE_U8X);
				/* Control humidity write will effective only
				after the control measurement register*/
				pre_ctrl_meas_value =
				p_bme280->ctrl_meas_reg;
				com_rslt += bme280_write_register(
					BME280_CTRL_MEAS_REG,
				&pre_ctrl_meas_value, BME280_ONE_U8X);
			}
			p_bme280->oversamp_humidity = v_value_u8;
			/* read the control measurement register value*/
			com_rslt += bme280_read_register(BME280_CTRL_MEAS_REG,
			&v_data_u8, BME280_ONE_U8X);
			p_bme280->ctrl_meas_reg = v_data_u8;
			/* read the control humidity register value*/
			com_rslt += bme280_read_register(
			BME280_CTRL_HUMIDITY_REG,
			&v_data_u8, BME280_ONE_U8X);
			p_bme280->ctrl_hum_reg = v_data_u8;
			/* read the control configuration register value*/
			com_rslt += bme280_read_register(BME280_CONFIG_REG,
			&v_data_u8, BME280_ONE_U8X);
			p_bme280->config_reg = v_data_u8;
		}
	return com_rslt;
}
/*!
 *	@brief This API used to get the
 *	Operational Mode from the sensor in the register 0xF4 bit 0 and 1
 *
 *
 *
 *	@param v_power_mode_u8 : The value of power mode
 *  value           |    mode
 * -----------------|------------------
 *	0x00            | BME280_SLEEP_MODE
 *	0x01 and 0x02   | BME280_FORCED_MODE
 *	0x03            | BME280_NORMAL_MODE
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
 *
*/
BME280_RETURN_FUNCTION_TYPE bme280_get_power_mode(u8 *v_power_mode_u8)
{
	/* used to return the communication result*/
	BME280_RETURN_FUNCTION_TYPE com_rslt = ERROR;
	u8 v_mode_u8r = BME280_ZERO_U8X;
	/* check the p_bme280 structure pointer as NULL*/
	if (p_bme280 == BME280_NULL) {
		return E_BME280_NULL_PTR;
		} else {
			com_rslt = p_bme280->BME280_BUS_READ_FUNC(
			p_bme280->dev_addr,
			BME280_CTRL_MEAS_REG_POWER_MODE__REG,
			&v_mode_u8r, BME280_ONE_U8X);
			*v_power_mode_u8 = BME280_GET_BITSLICE(v_mode_u8r,
			BME280_CTRL_MEAS_REG_POWER_MODE);
		}
	return com_rslt;
}
/*!
 *	@brief This API used to set the
 *	Operational Mode from the sensor in the register 0xF4 bit 0 and 1
 *
 *
 *
 *	@param v_power_mode_u8 : The value of power mode
 *  value           |    mode
 * -----------------|------------------
 *	0x00            | BME280_SLEEP_MODE
 *	0x01 and 0x02   | BME280_FORCED_MODE
 *	0x03            | BME280_NORMAL_MODE
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
 *
*/
BME280_RETURN_FUNCTION_TYPE bme280_set_power_mode(u8 v_power_mode_u8)
{
	/* used to return the communication result*/
	BME280_RETURN_FUNCTION_TYPE com_rslt = ERROR;
	u8 v_mode_u8r = BME280_ZERO_U8X;
	u8 v_prev_pow_mode_u8 = BME280_ZERO_U8X;
	u8 v_pre_ctrl_hum_value_u8 = BME280_ZERO_U8X;
	u8 v_pre_config_value_u8 = BME280_ZERO_U8X;
	u8 v_data_u8 = BME280_ZERO_U8X;
	/* check the p_bme280 structure pointer as NULL*/
	if (p_bme280 == BME280_NULL) {
		return E_BME280_NULL_PTR;
		} else {
			if (v_power_mode_u8 < BME280_FOUR_U8X) {
				v_mode_u8r = p_bme280->ctrl_meas_reg;
				v_mode_u8r =
				BME280_SET_BITSLICE(v_mode_u8r,
				BME280_CTRL_MEAS_REG_POWER_MODE,
				v_power_mode_u8);
				com_rslt = bme280_get_power_mode(
					&v_prev_pow_mode_u8);
				if (v_prev_pow_mode_u8 != BME280_SLEEP_MODE) {
					com_rslt += bme280_set_soft_rst();
					p_bme280->delay_msec(BME280_3MS_DELAY);
					/* write previous value of
					configuration register*/
					v_pre_config_value_u8 =
					p_bme280->config_reg;
					com_rslt = bme280_write_register(
						BME280_CONFIG_REG,
					&v_pre_config_value_u8, BME280_ONE_U8X);
					/* write previous value of
					humidity oversampling*/
					v_pre_ctrl_hum_value_u8 =
					p_bme280->ctrl_hum_reg;
					com_rslt += bme280_write_register(
					BME280_CTRL_HUMIDITY_REG,
					&v_pre_ctrl_hum_value_u8,
					BME280_ONE_U8X);
					/* write previous and updated value of
					control measurement register*/
					com_rslt += bme280_write_register(
					BME280_CTRL_MEAS_REG,
					&v_mode_u8r, BME280_ONE_U8X);
				} else {
					com_rslt =
					p_bme280->BME280_BUS_WRITE_FUNC(
					p_bme280->dev_addr,
					BME280_CTRL_MEAS_REG_POWER_MODE__REG,
					&v_mode_u8r, BME280_ONE_U8X);
				}
				/* read the control measurement register value*/
				com_rslt = bme280_read_register(
					BME280_CTRL_MEAS_REG,
				&v_data_u8, BME280_ONE_U8X);
				p_bme280->ctrl_meas_reg = v_data_u8;
				/* read the control humidity register value*/
				com_rslt += bme280_read_register(
					BME280_CTRL_HUMIDITY_REG,
				&v_data_u8, BME280_ONE_U8X);
				p_bme280->ctrl_hum_reg = v_data_u8;
				/* read the config register value*/
				com_rslt += bme280_read_register(
					BME280_CONFIG_REG,
				&v_data_u8, BME280_ONE_U8X);
				p_bme280->config_reg = v_data_u8;
			} else {
			com_rslt = E_BME280_OUT_OF_RANGE;
			}
		}
	return com_rslt;
}
/*!
 * @brief Used to reset the sensor
 * The value 0xB6 is written to the 0xE0
 * register the device is reset using the
 * complete power-on-reset procedure.
 * @note Soft reset can be easily set using bme280_set_softreset().
 * @note Usage Hint : bme280_set_softreset()
 *
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
 *
*/
BME280_RETURN_FUNCTION_TYPE bme280_set_soft_rst()
{
	/* used to return the communication result*/
	BME280_RETURN_FUNCTION_TYPE com_rslt = ERROR;
	u8 v_data_u8 = BME280_SOFT_RESET_CODE;
	/* check the p_bme280 structure pointer as NULL*/
	if (p_bme280 == BME280_NULL) {
		return E_BME280_NULL_PTR;
		} else {
			com_rslt = p_bme280->BME280_BUS_WRITE_FUNC(
			p_bme280->dev_addr,
			BME280_RST_REG, &v_data_u8, BME280_ONE_U8X);
		}
	return com_rslt;
}
/*!
 *	@brief This API used to get the sensor
 *	SPI mode(communication type) in the register 0xF5 bit 0
 *
 *
 *
 *	@param v_enable_disable_u8 : The value of SPI enable
 *	value  | Description
 * --------|--------------
 *   0     | Disable
 *   1     | Enable
 *
 *
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
 *
*/
BME280_RETURN_FUNCTION_TYPE bme280_get_spi3(u8 *v_enable_disable_u8)
{
	/* used to return the communication result*/
	BME280_RETURN_FUNCTION_TYPE com_rslt = ERROR;
	u8 v_data_u8 = BME280_ZERO_U8X;
	/* check the p_bme280 structure pointer as NULL*/
	if (p_bme280 == BME280_NULL) {
		return E_BME280_NULL_PTR;
		} else {
			com_rslt = p_bme280->BME280_BUS_READ_FUNC(
			p_bme280->dev_addr,
			BME280_CONFIG_REG_SPI3_ENABLE__REG,
			&v_data_u8, BME280_ONE_U8X);
			*v_enable_disable_u8 = BME280_GET_BITSLICE(
			v_data_u8,
			BME280_CONFIG_REG_SPI3_ENABLE);
		}
	return com_rslt;
}
/*!
 *	@brief This API used to set the sensor
 *	SPI mode(communication type) in the register 0xF5 bit 0
 *
 *
 *
 *	@param v_enable_disable_u8 : The value of SPI enable
 *	value  | Description
 * --------|--------------
 *   0     | Disable
 *   1     | Enable
 *
 *
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
 *
*/
BME280_RETURN_FUNCTION_TYPE bme280_set_spi3(u8 v_enable_disable_u8)
{
	/* used to return the communication result*/
	BME280_RETURN_FUNCTION_TYPE com_rslt = ERROR;
	u8 v_data_u8 = BME280_ZERO_U8X;
	u8 pre_ctrl_meas_value = BME280_ZERO_U8X;
	u8 v_prev_pow_mode_u8 = BME280_ZERO_U8X;
	u8 v_pre_ctrl_hum_value_u8 =  BME280_ZERO_U8X;
	/* check the p_bme280 structure pointer as NULL*/
	if (p_bme280 == BME280_NULL) {
		return E_BME280_NULL_PTR;
		} else {
			v_data_u8 = p_bme280->config_reg;
			v_data_u8 =
			BME280_SET_BITSLICE(v_data_u8,
			BME280_CONFIG_REG_SPI3_ENABLE, v_enable_disable_u8);
			com_rslt = bme280_get_power_mode(&v_prev_pow_mode_u8);
			if (v_prev_pow_mode_u8 != BME280_SLEEP_MODE) {
				com_rslt += bme280_set_soft_rst();
				p_bme280->delay_msec(BME280_3MS_DELAY);
				/* write previous and updated value of
				configuration register*/
				com_rslt += bme280_write_register(
					BME280_CONFIG_REG,
				&v_data_u8, BME280_ONE_U8X);
				/* write previous value of
				humidity oversampling*/
				v_pre_ctrl_hum_value_u8 =
				p_bme280->ctrl_hum_reg;
				com_rslt +=  bme280_write_register(
					BME280_CTRL_HUMIDITY_REG,
				&v_pre_ctrl_hum_value_u8, BME280_ONE_U8X);
				/* write previous value of
				control measurement register*/
				pre_ctrl_meas_value =
				p_bme280->ctrl_meas_reg;
				com_rslt += bme280_write_register(
					BME280_CTRL_MEAS_REG,
				&pre_ctrl_meas_value, BME280_ONE_U8X);
			} else {
				com_rslt =
				p_bme280->BME280_BUS_WRITE_FUNC(
				p_bme280->dev_addr,
				BME280_CONFIG_REG_SPI3_ENABLE__REG,
				&v_data_u8, BME280_ONE_U8X);
			}
			/* read the control measurement register value*/
			com_rslt += bme280_read_register(
				BME280_CTRL_MEAS_REG,
			&v_data_u8, BME280_ONE_U8X);
			p_bme280->ctrl_meas_reg = v_data_u8;
			/* read the control humidity register value*/
			com_rslt += bme280_read_register(
				BME280_CTRL_HUMIDITY_REG,
			&v_data_u8, BME280_ONE_U8X);
			p_bme280->ctrl_hum_reg = v_data_u8;
			/* read the control configuration register value*/
			com_rslt += bme280_read_register(
				BME280_CONFIG_REG,
			&v_data_u8, BME280_ONE_U8X);
			p_bme280->config_reg = v_data_u8;
		}
	return com_rslt;
}
/*!
 *	@brief This API is used to reads filter setting
 *	in the register 0xF5 bit 3 and 4
 *
 *
 *
 *	@param v_value_u8 : The value of IIR filter coefficient
 *
 *	value	    |	Filter coefficient
 * -------------|-------------------------
 *	0x00        | BME280_FILTER_COEFF_OFF
 *	0x01        | BME280_FILTER_COEFF_2
 *	0x02        | BME280_FILTER_COEFF_4
 *	0x03        | BME280_FILTER_COEFF_8
 *	0x04        | BME280_FILTER_COEFF_16
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
 *
*/
BME280_RETURN_FUNCTION_TYPE bme280_get_filter(u8 *v_value_u8)
{
	/* used to return the communication result*/
	BME280_RETURN_FUNCTION_TYPE com_rslt = ERROR;
	u8 v_data_u8 = BME280_ZERO_U8X;
	/* check the p_bme280 structure pointer as NULL*/
	if (p_bme280 == BME280_NULL) {
		return E_BME280_NULL_PTR;
		} else {
			com_rslt = p_bme280->BME280_BUS_READ_FUNC(
			p_bme280->dev_addr,
			BME280_CONFIG_REG_FILTER__REG,
			&v_data_u8, BME280_ONE_U8X);
			*v_value_u8 = BME280_GET_BITSLICE(v_data_u8,
			BME280_CONFIG_REG_FILTER);
		}
	return com_rslt;
}
/*!
 *	@brief This API is used to write filter setting
 *	in the register 0xF5 bit 3 and 4
 *
 *
 *
 *	@param v_value_u8 : The value of IIR filter coefficient
 *
 *	value	    |	Filter coefficient
 * -------------|-------------------------
 *	0x00        | BME280_FILTER_COEFF_OFF
 *	0x01        | BME280_FILTER_COEFF_2
 *	0x02        | BME280_FILTER_COEFF_4
 *	0x03        | BME280_FILTER_COEFF_8
 *	0x04        | BME280_FILTER_COEFF_16
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
 *
*/
BME280_RETURN_FUNCTION_TYPE bme280_set_filter(u8 v_value_u8)
{
	/* used to return the communication result*/
	BME280_RETURN_FUNCTION_TYPE com_rslt = ERROR;
	u8 v_data_u8 = BME280_ZERO_U8X;
	u8 pre_ctrl_meas_value = BME280_ZERO_U8X;
	u8 v_prev_pow_mode_u8 = BME280_ZERO_U8X;
	u8 v_pre_ctrl_hum_value_u8 =  BME280_ZERO_U8X;
	/* check the p_bme280 structure pointer as NULL*/
	if (p_bme280 == BME280_NULL) {
		return E_BME280_NULL_PTR;
		} else {
			v_data_u8 = p_bme280->config_reg;
			v_data_u8 =
			BME280_SET_BITSLICE(v_data_u8,
			BME280_CONFIG_REG_FILTER, v_value_u8);
			com_rslt = bme280_get_power_mode(&v_prev_pow_mode_u8);
			if (v_prev_pow_mode_u8 != BME280_SLEEP_MODE) {
				com_rslt += bme280_set_soft_rst();
				p_bme280->delay_msec(BME280_3MS_DELAY);
				/* write previous and updated value of
				configuration register*/
				com_rslt += bme280_write_register(
					BME280_CONFIG_REG,
				&v_data_u8, BME280_ONE_U8X);
				/* write previous value of
				humidity oversampling*/
				v_pre_ctrl_hum_value_u8 =
				p_bme280->ctrl_hum_reg;
				com_rslt += bme280_write_register(
					BME280_CTRL_HUMIDITY_REG,
				&v_pre_ctrl_hum_value_u8, BME280_ONE_U8X);
				/* write previous value of
				control measurement register*/
				pre_ctrl_meas_value =
				p_bme280->ctrl_meas_reg;
				com_rslt += bme280_write_register(
					BME280_CTRL_MEAS_REG,
				&pre_ctrl_meas_value, BME280_ONE_U8X);
			} else {
				com_rslt =
				p_bme280->BME280_BUS_WRITE_FUNC(
				p_bme280->dev_addr,
				BME280_CONFIG_REG_FILTER__REG,
				&v_data_u8, BME280_ONE_U8X);
			}
			/* read the control measurement register value*/
			com_rslt += bme280_read_register(BME280_CTRL_MEAS_REG,
			&v_data_u8, BME280_ONE_U8X);
			p_bme280->ctrl_meas_reg = v_data_u8;
			/* read the control humidity register value*/
			com_rslt += bme280_read_register(
			BME280_CTRL_HUMIDITY_REG,
			&v_data_u8, BME280_ONE_U8X);
			p_bme280->ctrl_hum_reg = v_data_u8;
			/* read the configuration register value*/
			com_rslt += bme280_read_register(BME280_CONFIG_REG,
			&v_data_u8, BME280_ONE_U8X);
			p_bme280->config_reg = v_data_u8;
		}
	return com_rslt;
}
/*!
 *	@brief This API used to Read the
 *	standby duration time from the sensor in the register 0xF5 bit 5 to 7
 *
 *	@param v_standby_durn_u8 : The value of standby duration time value.
 *  value       | standby duration
 * -------------|-----------------------
 *    0x00      | BME280_STANDBY_TIME_1_MS
 *    0x01      | BME280_STANDBY_TIME_63_MS
 *    0x02      | BME280_STANDBY_TIME_125_MS
 *    0x03      | BME280_STANDBY_TIME_250_MS
 *    0x04      | BME280_STANDBY_TIME_500_MS
 *    0x05      | BME280_STANDBY_TIME_1000_MS
 *    0x06      | BME280_STANDBY_TIME_2000_MS
 *    0x07      | BME280_STANDBY_TIME_4000_MS
 *
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
 *
*/
BME280_RETURN_FUNCTION_TYPE bme280_get_standby_durn(u8 *v_standby_durn_u8)
{
	/* used to return the communication result*/
	BME280_RETURN_FUNCTION_TYPE com_rslt = ERROR;
	u8 v_data_u8 = BME280_ZERO_U8X;
	/* check the p_bme280 structure pointer as NULL*/
	if (p_bme280 == BME280_NULL) {
		return E_BME280_NULL_PTR;
		} else {
			com_rslt = p_bme280->BME280_BUS_READ_FUNC(
			p_bme280->dev_addr,
			BME280_CONFIG_REG_TSB__REG,
			&v_data_u8, BME280_ONE_U8X);
			*v_standby_durn_u8 = BME280_GET_BITSLICE(
			v_data_u8, BME280_CONFIG_REG_TSB);
		}
	return com_rslt;
}
/*!
 *	@brief This API used to write the
 *	standby duration time from the sensor in the register 0xF5 bit 5 to 7
 *
 *	@param v_standby_durn_u8 : The value of standby duration time value.
 *  value       | standby duration
 * -------------|-----------------------
 *    0x00      | BME280_STANDBY_TIME_1_MS
 *    0x01      | BME280_STANDBY_TIME_63_MS
 *    0x02      | BME280_STANDBY_TIME_125_MS
 *    0x03      | BME280_STANDBY_TIME_250_MS
 *    0x04      | BME280_STANDBY_TIME_500_MS
 *    0x05      | BME280_STANDBY_TIME_1000_MS
 *    0x06      | BME280_STANDBY_TIME_2000_MS
 *    0x07      | BME280_STANDBY_TIME_4000_MS
 *
 *	@note Normal mode comprises an automated perpetual
 *	cycling between an (active)
 *	Measurement period and an (inactive) standby period.
 *	@note The standby time is determined by
 *	the contents of the register t_sb.
 *	Standby time can be set using BME280_STANDBY_TIME_125_MS.
 *
 *	@note Usage Hint : bme280_set_standby_durn(BME280_STANDBY_TIME_125_MS)
 *
 *
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
 *
*/
BME280_RETURN_FUNCTION_TYPE bme280_set_standby_durn(u8 v_standby_durn_u8)
{
	/* used to return the communication result*/
	BME280_RETURN_FUNCTION_TYPE com_rslt = ERROR;
	u8 v_data_u8 = BME280_ZERO_U8X;
	u8 pre_ctrl_meas_value = BME280_ZERO_U8X;
	u8 v_prev_pow_mode_u8 = BME280_ZERO_U8X;
	u8 v_pre_ctrl_hum_value_u8 =  BME280_ZERO_U8X;
	/* check the p_bme280 structure pointer as NULL*/
	if (p_bme280 == BME280_NULL) {
		return E_BME280_NULL_PTR;
		} else {
			v_data_u8 = p_bme280->config_reg;
			v_data_u8 =
			BME280_SET_BITSLICE(v_data_u8,
			BME280_CONFIG_REG_TSB, v_standby_durn_u8);
			com_rslt = bme280_get_power_mode(&v_prev_pow_mode_u8);
			if (v_prev_pow_mode_u8 != BME280_SLEEP_MODE) {
				com_rslt += bme280_set_soft_rst();
				p_bme280->delay_msec(BME280_3MS_DELAY);
				/* write previous and updated value of
				configuration register*/
				com_rslt += bme280_write_register(
					BME280_CONFIG_REG,
				&v_data_u8, BME280_ONE_U8X);
				/* write previous value of
				humidity oversampling*/
				v_pre_ctrl_hum_value_u8 =
				p_bme280->ctrl_hum_reg;
				com_rslt += bme280_write_register(
					BME280_CTRL_HUMIDITY_REG,
				&v_pre_ctrl_hum_value_u8, BME280_ONE_U8X);
				/* write previous value of control
				measurement register*/
				pre_ctrl_meas_value =
				p_bme280->ctrl_meas_reg;
				com_rslt += bme280_write_register(
					BME280_CTRL_MEAS_REG,
				&pre_ctrl_meas_value, BME280_ONE_U8X);
			} else {
				com_rslt =
				p_bme280->BME280_BUS_WRITE_FUNC(
				p_bme280->dev_addr,
				BME280_CONFIG_REG_TSB__REG,
				&v_data_u8, BME280_ONE_U8X);
			}
			/* read the control measurement register value*/
			com_rslt += bme280_read_register(BME280_CTRL_MEAS_REG,
			&v_data_u8, BME280_ONE_U8X);
			p_bme280->ctrl_meas_reg = v_data_u8;
			/* read the control humidity register value*/
			com_rslt += bme280_read_register(
			BME280_CTRL_HUMIDITY_REG,
			&v_data_u8, BME280_ONE_U8X);
			p_bme280->ctrl_hum_reg = v_data_u8;
			/* read the configuration register value*/
			com_rslt += bme280_read_register(BME280_CONFIG_REG,
			&v_data_u8, BME280_ONE_U8X);
			p_bme280->config_reg = v_data_u8;
		}
	return com_rslt;
}
/*
 * @brief Writes the working mode to the sensor
 *
 *
 *
 *
 *  @param v_work_mode_u8 : Mode to be set
 *  value    | Working mode
 * ----------|--------------------
 *   0       | BME280_ULTRALOWPOWER_MODE
 *   1       | BME280_LOWPOWER_MODE
 *   2       | BME280_STANDARDRESOLUTION_MODE
 *   3       | BME280_HIGHRESOLUTION_MODE
 *   4       | BME280_ULTRAHIGHRESOLUTION_MODE
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
 *
*/
/*BME280_RETURN_FUNCTION_TYPE bme280_set_work_mode(u8 v_work_mode_u8)
{
BME280_RETURN_FUNCTION_TYPE com_rslt = ERROR;
u8 v_data_u8 = BME280_ZERO_U8X;
if (p_bme280 == BME280_NULL) {
	return E_BME280_NULL_PTR;
} else {
	if (v_work_mode_u8 <= BME280_FOUR_U8X) {
		com_rslt = p_bme280->BME280_BUS_READ_FUNC(
			p_bme280->dev_addr,	BME280_CTRL_MEAS_REG,
			&v_data_u8, BME280_ONE_U8X);
		if (com_rslt == SUCCESS) {
			switch (v_work_mode_u8) {
			case BME280_ULTRALOWPOWER_MODE:
				p_bme280->oversamp_temperature =
				BME280_ULTRALOWPOWER_OSRS_T;
				p_bme280->osrs_p =
				BME280_ULTRALOWPOWER_OSRS_P;
				break;
			case BME280_LOWPOWER_MODE:
				p_bme280->oversamp_temperature =
				BME280_LOWPOWER_OSRS_T;
				p_bme280->osrs_p = BME280_LOWPOWER_OSRS_P;
				break;
			case BME280_STANDARDRESOLUTION_MODE:
				p_bme280->oversamp_temperature =
				BME280_STANDARDRESOLUTION_OSRS_T;
				p_bme280->osrs_p =
				BME280_STANDARDRESOLUTION_OSRS_P;
				break;
			case BME280_HIGHRESOLUTION_MODE:
				p_bme280->oversamp_temperature =
				BME280_HIGHRESOLUTION_OSRS_T;
				p_bme280->osrs_p = BME280_HIGHRESOLUTION_OSRS_P;
				break;
			case BME280_ULTRAHIGHRESOLUTION_MODE:
				p_bme280->oversamp_temperature =
				BME280_ULTRAHIGHRESOLUTION_OSRS_T;
				p_bme280->osrs_p =
				BME280_ULTRAHIGHRESOLUTION_OSRS_P;
				break;
			}
			v_data_u8 = BME280_SET_BITSLICE(v_data_u8,
				BME280_CTRL_MEAS_REG_OVERSAMP_TEMPERATURE,
				p_bme280->oversamp_temperature);
			v_data_u8 = BME280_SET_BITSLICE(v_data_u8,
				BME280_CTRL_MEAS_REG_OVERSAMP_PRESSURE,
				p_bme280->osrs_p);
			com_rslt += p_bme280->BME280_BUS_WRITE_FUNC(
				p_bme280->dev_addr,	BME280_CTRL_MEAS_REG,
				&v_data_u8, BME280_ONE_U8X);
		}
	} else {
		com_rslt = E_BME280_OUT_OF_RANGE;
	}
}
return com_rslt;
}*/
/*!
 * @brief This API used to read uncompensated
 * temperature,pressure and humidity in forced mode
 *
 *
 *	@param v_uncom_pressure_s32: The value of uncompensated pressure
 *	@param v_uncom_temperature_s32: The value of uncompensated temperature
 *	@param v_uncom_humidity_s32: The value of uncompensated humidity
 *
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
 *
*/
BME280_RETURN_FUNCTION_TYPE
bme280_get_forced_uncomp_pressure_temperature_humidity(
s32 *v_uncom_pressure_s32,
s32 *v_uncom_temperature_s32, s32 *v_uncom_humidity_s32)
{
	/* used to return the communication result*/
	BME280_RETURN_FUNCTION_TYPE com_rslt = ERROR;
	u8 v_data_u8 = BME280_ZERO_U8X;
	u8 v_waittime_u8r = BME280_ZERO_U8X;
	u8 v_prev_pow_mode_u8 = BME280_ZERO_U8X;
	u8 v_mode_u8r = BME280_ZERO_U8X;
	u8 pre_ctrl_config_value = BME280_ZERO_U8X;
	u8 v_pre_ctrl_hum_value_u8 = BME280_ZERO_U8X;
	/* check the p_bme280 structure pointer as NULL*/
	if (p_bme280 == BME280_NULL) {
		return E_BME280_NULL_PTR;
		} else {
			v_mode_u8r = p_bme280->ctrl_meas_reg;
			v_mode_u8r =
			BME280_SET_BITSLICE(v_mode_u8r,
			BME280_CTRL_MEAS_REG_POWER_MODE, BME280_FORCED_MODE);
			com_rslt = bme280_get_power_mode(&v_prev_pow_mode_u8);
			if (v_prev_pow_mode_u8 != BME280_SLEEP_MODE) {
				com_rslt += bme280_set_soft_rst();
				p_bme280->delay_msec(BME280_3MS_DELAY);
				/* write previous and updated value of
				configuration register*/
				pre_ctrl_config_value = p_bme280->config_reg;
				com_rslt += bme280_write_register(
					BME280_CONFIG_REG,
				&pre_ctrl_config_value, BME280_ONE_U8X);
				/* write previous value of
				humidity oversampling*/
				v_pre_ctrl_hum_value_u8 =
				p_bme280->ctrl_hum_reg;
				com_rslt += bme280_write_register(
					BME280_CTRL_HUMIDITY_REG,
				&v_pre_ctrl_hum_value_u8, BME280_ONE_U8X);
				/* write the force mode  */
				com_rslt += bme280_write_register(
					BME280_CTRL_MEAS_REG,
				&v_mode_u8r, BME280_ONE_U8X);
			} else {
				/* write previous value of
				humidity oversampling*/
				v_pre_ctrl_hum_value_u8 =
				p_bme280->ctrl_hum_reg;
				com_rslt += bme280_write_register(
					BME280_CTRL_HUMIDITY_REG,
				&v_pre_ctrl_hum_value_u8, BME280_ONE_U8X);
				/* write the force mode  */
				com_rslt += bme280_write_register(
					BME280_CTRL_MEAS_REG,
				&v_mode_u8r, BME280_ONE_U8X);
			}
			bme280_compute_wait_time(&v_waittime_u8r);
			p_bme280->delay_msec(v_waittime_u8r);
			/* read the force-mode value of pressure
			temperature and humidity*/
			com_rslt +=
			bme280_read_uncomp_pressure_temperature_humidity(
			v_uncom_pressure_s32, v_uncom_temperature_s32,
			v_uncom_humidity_s32);

			/* read the control humidity register value*/
			com_rslt += bme280_read_register(
			BME280_CTRL_HUMIDITY_REG,
			&v_data_u8, BME280_ONE_U8X);
			p_bme280->ctrl_hum_reg = v_data_u8;
			/* read the configuration register value*/
			com_rslt += bme280_read_register(BME280_CONFIG_REG,
			&v_data_u8, BME280_ONE_U8X);
			p_bme280->config_reg = v_data_u8;

			/* read the control measurement register value*/
			com_rslt += bme280_read_register(BME280_CTRL_MEAS_REG,
			&v_data_u8, BME280_ONE_U8X);
			p_bme280->ctrl_meas_reg = v_data_u8;
		}
	return com_rslt;
}
/*!
 * @brief
 *	This API write the data to
 *	the given register
 *
 *
 *	@param v_addr_u8 -> Address of the register
 *	@param v_data_u8 -> The data from the register
 *	@param v_len_u8 -> no of bytes to read
 *
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
 *
 */
BME280_RETURN_FUNCTION_TYPE bme280_write_register(u8 v_addr_u8,
u8 *v_data_u8, u8 v_len_u8)
{
	/* used to return the communication result*/
	BME280_RETURN_FUNCTION_TYPE com_rslt = ERROR;
	/* check the p_bme280 structure pointer as NULL*/
	if (p_bme280 == BME280_NULL) {
		return E_BME280_NULL_PTR;
		} else {
			com_rslt = p_bme280->BME280_BUS_WRITE_FUNC(
			p_bme280->dev_addr,
			v_addr_u8, v_data_u8, v_len_u8);
		}
	return com_rslt;
}
/*!
 * @brief
 *	This API reads the data from
 *	the given register
 *
 *
 *	@param v_addr_u8 -> Address of the register
 *	@param v_data_u8 -> The data from the register
 *	@param v_len_u8 -> no of bytes to read
 *
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
 *
 */
BME280_RETURN_FUNCTION_TYPE bme280_read_register(u8 v_addr_u8,
u8 *v_data_u8, u8 v_len_u8)
{
	/* used to return the communication result*/
	BME280_RETURN_FUNCTION_TYPE com_rslt = ERROR;
	/* check the p_bme280 structure pointer as NULL*/
	if (p_bme280 == BME280_NULL) {
		return E_BME280_NULL_PTR;
		} else {
			com_rslt = p_bme280->BME280_BUS_READ_FUNC(
			p_bme280->dev_addr,
			v_addr_u8, v_data_u8, v_len_u8);
		}
	return com_rslt;
}
#ifdef BME280_ENABLE_FLOAT
/*!
 * @brief Reads actual temperature from uncompensated temperature
 * @note returns the value in Degree centigrade
 * @note Output value of "51.23" equals 51.23 DegC.
 *
 *
 *
 *  @param v_uncom_temperature_s32 : value of uncompensated temperature
 *
 *
 *
 *  @return  Return the actual temperature in floating point
 *
*/
double bme280_compensate_T_double(s32 v_uncom_temperature_s32)
{
	double v_x1_u32 = BME280_ZERO_U8X;
	double v_x2_u32 = BME280_ZERO_U8X;
	double temperature = BME280_ZERO_U8X;

	v_x1_u32  = (((double)v_uncom_temperature_s32)
	/ BME280_FLOAT_TRUE_TEMP_1_6_3_8_4_DATA -
	((double)p_bme280->cal_param.dig_T1)
	/ BME280_FLOAT_TRUE_TEMP_1_0_2_4_DATA) *
	((double)p_bme280->cal_param.dig_T2);
	v_x2_u32  = ((((double)v_uncom_temperature_s32)
	/ BME280_FLOAT_TRUE_TEMP_1_3_1_0_7_2_DATA -
	((double)p_bme280->cal_param.dig_T1)
	/ BME280_FLOAT_TRUE_TEMP_8_1_9_2_DATA) *
	(((double)v_uncom_temperature_s32) /
	BME280_FLOAT_TRUE_TEMP_1_3_1_0_7_2_DATA -
	((double)p_bme280->cal_param.dig_T1) /
	BME280_FLOAT_TRUE_TEMP_8_1_9_2_DATA)) *
	((double)p_bme280->cal_param.dig_T3);
	p_bme280->cal_param.t_fine = (s32)(v_x1_u32 + v_x2_u32);
	temperature  = (v_x1_u32 + v_x2_u32) /
	BME280_FLOAT_TRUE_TEMP_5_1_2_0_DATA;


	return temperature;
}
/*!
 * @brief Reads actual pressure from uncompensated pressure
 * @note Returns pressure in Pa as double.
 * @note Output value of "96386.2"
 * equals 96386.2 Pa = 963.862 hPa.
 *
 *
 *  @param v_uncom_pressure_s32 : value of uncompensated pressure
 *
 *
 *  @return  Return the actual pressure in floating point
 *
*/
double bme280_compensate_P_double(s32 v_uncom_pressure_s32)
{
	double v_x1_u32 = BME280_ZERO_U8X;
	double v_x2_u32 = BME280_ZERO_U8X;
	double pressure = BME280_ZERO_U8X;

	v_x1_u32 = ((double)p_bme280->cal_param.t_fine /
	BME280_FLAOT_TRUE_PRESSURE_2_DATA) -
	BME280_FLAOT_TRUE_PRESSURE_6_4_0_0_0_DATA;
	v_x2_u32 = v_x1_u32 * v_x1_u32 *
	((double)p_bme280->cal_param.dig_P6) /
	BME280_FLAOT_TRUE_PRESSURE_3_2_7_6_8_DATA;
	v_x2_u32 = v_x2_u32 + v_x1_u32 *
	((double)p_bme280->cal_param.dig_P5) *
	BME280_FLAOT_TRUE_PRESSURE_2_DATA;
	v_x2_u32 = (v_x2_u32 / BME280_FLAOT_TRUE_PRESSURE_4_DATA) +
	(((double)p_bme280->cal_param.dig_P4) *
	BME280_FLAOT_TRUE_PRESSURE_6_5_5_3_6_DATA);
	v_x1_u32 = (((double)p_bme280->cal_param.dig_P3) *
	v_x1_u32 * v_x1_u32
	/ BME280_FLAOT_TRUE_PRESSURE_5_2_4_2_8_8_DATA +
	((double)p_bme280->cal_param.dig_P2) * v_x1_u32) /
	BME280_FLAOT_TRUE_PRESSURE_5_2_4_2_8_8_DATA;
	v_x1_u32 = (BME280_FLAOT_TRUE_PRESSURE_1_DATA + v_x1_u32
	/ BME280_FLAOT_TRUE_PRESSURE_3_2_7_6_8_DATA) *
	((double)p_bme280->cal_param.dig_P1);
	pressure = BME280_FLAOT_TRUE_PRESSURE_1_0_4_8_5_7_6_DATA
	- (double)v_uncom_pressure_s32;
	/* Avoid exception caused by division by zero */
	if (v_x1_u32 != BME280_ZERO_U8X)
		pressure = (pressure - (v_x2_u32
		/ BME280_FLAOT_TRUE_PRESSURE_4_0_9_6_DATA))
		* BME280_FLAOT_TRUE_PRESSURE_6_2_5_0_DATA / v_x1_u32;
	else
		return BME280_ZERO_U8X;
	v_x1_u32 = ((double)p_bme280->cal_param.dig_P9) *
	pressure * pressure /
	BME280_FLAOT_TRUE_PRESSURE_2_1_4_7_4_8_3_6_4_8_DATA;
	v_x2_u32 = pressure * ((double)p_bme280->cal_param.dig_P8)
	/ BME280_FLAOT_TRUE_PRESSURE_3_2_7_6_8_DATA;
	pressure = pressure + (v_x1_u32 + v_x2_u32 +
	((double)p_bme280->cal_param.dig_P7))
	/ BME280_FLAOT_TRUE_PRESSURE_1_6_DATA;

	return pressure;
}
/*!
 * @brief Reads actual humidity from uncompensated humidity
 * @note returns the value in relative humidity (%rH)
 * @note Output value of "42.12" equals 42.12 %rH
 *
 *  @param v_uncom_humidity_s32 : value of uncompensated humidity
 *
 *
 *
 *  @return Return the actual humidity in floating point
 *
*/
double bme280_compensate_H_double(s32 v_uncom_humidity_s32)
{
	double var_h = BME280_ZERO_U8X;
	var_h = (((double)p_bme280->cal_param.t_fine)
	- BME280_TRUE_HUMIDITY_7_6_8_0_0_DATA);
	if (var_h != BME280_ZERO_U8X)
		var_h = (v_uncom_humidity_s32 -
		(((double)p_bme280->cal_param.dig_H4)
		* BME280_TRUE_HUMIDITY_6_4_DATA +
		((double)p_bme280->cal_param.dig_H5)
		/ BME280_TRUE_HUMIDITY_1_6_3_8_4_DATA * var_h))*
		(((double)p_bme280->cal_param.dig_H2)
		/BME280_TRUE_HUMIDITY_6_5_5_3_6_DATA *
		(BME280_TRUE_HUMIDITY_1_DATA + ((double)
		p_bme280->cal_param.dig_H6)
		/ BME280_TRUE_HUMIDITY_6_7_1_0_8_8_6_4_DATA
		* var_h * (BME280_TRUE_HUMIDITY_1_DATA + ((double)
		p_bme280->cal_param.dig_H3)
		/ BME280_TRUE_HUMIDITY_6_7_1_0_8_8_6_4_DATA * var_h)));
	else
		return BME280_ZERO_U8X;
	var_h = var_h * (BME280_TRUE_HUMIDITY_1_DATA - ((double)
	p_bme280->cal_param.dig_H1)*var_h
	/ BME280_TRUE_HUMIDITY_5_2_4_2_8_8_DATA);
	if (var_h > BME280_TRUE_HUMIDITY_1_0_0_DATA)
		var_h = BME280_TRUE_HUMIDITY_1_0_0_DATA;
	else if (var_h < BME280_TRUE_HUMIDITY_0_DATA)
		var_h = BME280_TRUE_HUMIDITY_0_DATA;
	return var_h;

}
#endif
#if defined(BME280_ENABLE_INT64) && defined(BME280_64BITSUPPORT_PRESENT)
/*!
 * @brief Reads actual pressure from uncompensated pressure
 * @note Returns the value in Pa as unsigned 32 bit
 * integer in Q24.8 format (24 integer bits and
 * 8 fractional bits).
 * @note Output value of "24674867"
 * represents 24674867 / 256 = 96386.2 Pa = 963.862 hPa
 *
 *
 *
 *  @param  v_uncom_pressure_s32 : value of uncompensated temperature
 *
 *
 *  @return Return the actual pressure in u32
 *
*/
u32 bme280_compensate_P_int64(s32 v_uncom_pressure_s32)
{
	s64 v_x1_s64r = BME280_ZERO_U8X;
	s64 v_x2_s64r = BME280_ZERO_U8X;
	s64 pressure = BME280_ZERO_U8X;
	v_x1_s64r = ((s64)p_bme280->cal_param.t_fine)
	- BME280_TRUE_PRESSURE_1_2_8_0_0_0_DATA;
	v_x2_s64r = v_x1_s64r * v_x1_s64r *
	(s64)p_bme280->cal_param.dig_P6;
	v_x2_s64r = v_x2_s64r + ((v_x1_s64r *
	(s64)p_bme280->cal_param.dig_P5)
	<< SHIFT_LEFT_17_POSITION);
	v_x2_s64r = v_x2_s64r +
	(((s64)p_bme280->cal_param.dig_P4)
	<< SHIFT_LEFT_35_POSITION);
	v_x1_s64r = ((v_x1_s64r * v_x1_s64r *
	(s64)p_bme280->cal_param.dig_P3)
	>> SHIFT_RIGHT_8_POSITION) +
	((v_x1_s64r * (s64)p_bme280->cal_param.dig_P2)
	<< SHIFT_LEFT_12_POSITION);
	v_x1_s64r = (((((s64)BME280_ONE_U8X)
	<< SHIFT_LEFT_47_POSITION) + v_x1_s64r)) *
	((s64)p_bme280->cal_param.dig_P1)
	>> SHIFT_RIGHT_33_POSITION;
	pressure = BME280_TRUE_PRESSURE_1_0_4_8_5_7_6_DATA
	- v_uncom_pressure_s32;
	/* Avoid exception caused by division by zero */
	if (v_x1_s64r != BME280_ZERO_U8X)
		#if defined __KERNEL__
			pressure = div64_s64((((pressure
			<< SHIFT_LEFT_31_POSITION) - v_x2_s64r)
			* BME280_TRUE_PRESSURE_3_1_2_5_DATA),
			v_x1_s64r);
		#else
			pressure = (((pressure
			<< SHIFT_LEFT_31_POSITION) - v_x2_s64r)
			* BME280_TRUE_PRESSURE_3_1_2_5_DATA) / v_x1_s64r;
		#endif
	else
		return BME280_ZERO_U8X;
	v_x1_s64r = (((s64)p_bme280->cal_param.dig_P9) *
	(pressure >> SHIFT_RIGHT_13_POSITION) *
	(pressure >> SHIFT_RIGHT_13_POSITION))
	>> SHIFT_RIGHT_25_POSITION;
	v_x2_s64r = (((s64)p_bme280->cal_param.dig_P8) *
	pressure) >> SHIFT_RIGHT_19_POSITION;
	pressure = (((pressure + v_x1_s64r +
	v_x2_s64r) >> SHIFT_RIGHT_8_POSITION) +
	(((s64)p_bme280->cal_param.dig_P7)
	<< SHIFT_LEFT_4_POSITION));

	return (u32)pressure;
}
/*!
 * @brief Reads actual pressure from uncompensated pressure
 * @note Returns the value in Pa.
 * @note Output value of "12337434"
 * @note represents 12337434 / 128 = 96386.2 Pa = 963.862 hPa
 *
 *
 *
 *  @param v_uncom_pressure_s32 : value of uncompensated pressure
 *
 *
 *  @return the actual pressure in u32
 *
*/
u32 bme280_compensate_P_int64_twentyfour_bit_output(s32 v_uncom_pressure_s32)
{
	u32 pressure = BME280_ZERO_U8X;
	pressure = bme280_compensate_P_int64(v_uncom_pressure_s32);
	pressure = (u32)(pressure >> SHIFT_RIGHT_1_POSITION);
	return pressure;
}
#endif
/*!
 * @brief Computing waiting time for sensor data read
 *
 *
 *
 *
 *  @param v_delaytime_u8 : The value of delay time for force mode
 *
 *
 *	@retval 0 -> Success
 *
 *
 */
BME280_RETURN_FUNCTION_TYPE bme280_compute_wait_time(u8
*v_delaytime_u8)
{
	/* used to return the communication result*/
	BME280_RETURN_FUNCTION_TYPE com_rslt = SUCCESS;

	*v_delaytime_u8 = (T_INIT_MAX +
	T_MEASURE_PER_OSRS_MAX *
	(((BME280_ONE_U8X
	<< p_bme280->oversamp_temperature)
	>> SHIFT_RIGHT_1_POSITION) +
	((BME280_ONE_U8X << p_bme280->oversamp_pressure)
	>> SHIFT_RIGHT_1_POSITION) +
	((BME280_ONE_U8X << p_bme280->oversamp_humidity)
	>> SHIFT_RIGHT_1_POSITION))+
	(p_bme280->oversamp_pressure ?
	T_SETUP_PRESSURE_MAX : BME280_ZERO_U8X) +
	(p_bme280->oversamp_humidity ?
	T_SETUP_HUMIDITY_MAX : BME280_ZERO_U8X)
	+ BME280_FIVETEEN_U8X) / BME280_SIXTEEN_U8X;
	return com_rslt;
}
