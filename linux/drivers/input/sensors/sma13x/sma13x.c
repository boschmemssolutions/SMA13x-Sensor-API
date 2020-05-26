// SPDX-License-Identifier: BSD-3-Clause OR GPL-2.0
/**
 * This file is provided under a dual BSD/GPLv2 license.  When using or
 * redistributing this file, you may do so under either license.
 *
 * GPL LICENSE
 * Copyright (c) 2020 Robert Bosch GmbH. All rights reserved.
 * Copyright (c) 2020 Bosch Sensortec GmbH. All rights reserved.
 *
 * This file is free software licensed under the terms of version 2 
 * of the GNU General Public License, available from the file LICENSE-GPL 
 * in the main directory of this source tree.
 *
 * BSD LICENSE
 * Copyright (c) 2020 Bosch Sensortec GmbH. All rights reserved.
 * Copyright (c) 2020 Robert Bosch GmbH. All rights reserved.
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
 **/

#include "sma13x.h"
/*! user defined code to be added here ... */
static struct sma13x_t *p_sma13x;
/*! Based on Bit resolution value_u8 should be modified */
/* SMA13X supports ONLY 14bit resolution */
u8 V_SMA13XRESOLUTION_U8 = SMA13X_14_RESOLUTION;

/****************************************************************************/
/*!	Static Function Declarations
*****************************************************************************/

SMA13X_RETURN_FUNCTION_TYPE sma13x_burst_read(u8 addr_u8,
u8 *data_u8, u32 len_u32)
{
	/* Variable used to return value of
	communication routine*/
	SMA13X_RETURN_FUNCTION_TYPE com_rslt = ERROR;

	if (p_sma13x == SMA13X_NULL) {
		/* Check the struct p_sma13x is empty */
		return E_SMA13X_NULL_PTR;
		} else {
			/* Read the data from the register*/
			com_rslt = p_sma13x->SMA13X_BURST_READ_FUNC
			(p_sma13x->dev_addr, addr_u8, data_u8, len_u32);
		}
	return com_rslt;
}
/*!
 *	@brief
 *	This function is used for initialize
 *	bus read and bus write functions
 *	assign the chip id and device address
 *	chip id is read in the register 0x00 bit from 0 to 7
 *
 *	@param sma13x : structure pointer
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
 *	@note
 *	While changing the parameter of the sma13x_t
 *	consider the following point:
 *	Changing the reference value of the parameter
 *	will changes the local copy or local reference
 *	make sure your changes will not
 *	affect the reference value of the parameter
 *	(Better case don't change the reference value of the parameter)
 *
*/
SMA13X_RETURN_FUNCTION_TYPE sma13x_init(struct sma13x_t *sma13x)
{
	/*  Variable used to return value of
	communication routine*/
	SMA13X_RETURN_FUNCTION_TYPE com_rslt = ERROR;
	u8 data_u8 = SMA13X_INIT_VALUE;
	/* assign sma13x ptr */
	p_sma13x = sma13x;
	if (p_sma13x == SMA13X_NULL) {
		/* Check the struct p_sma13x is empty */
		com_rslt = E_SMA13X_NULL_PTR;
	} else {
		/* read Chip Id */
		com_rslt = p_sma13x->SMA13X_BUS_READ_FUNC
		(p_sma13x->dev_addr,
		SMA13X_CHIP_ID_REG, &data_u8, SMA13X_GEN_READ_WRITE_LENGTH);
		p_sma13x->chip_id = data_u8;    /* get bit slice */
	}
	return com_rslt;
}
/*!
 * @brief
 *	This API gives data to the given register and
 *	the data is written in the corresponding register address
 *
 *
 *	@param adr_u8  -> Address of the register
 *	@param data_u8 -> The data to the register
 *	@param len_u8 -> no of bytes to read
 *
 *
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
 *
*/
SMA13X_RETURN_FUNCTION_TYPE sma13x_write_reg(u8 adr_u8,
u8 *data_u8, u8 len_u8)
{
	/*  Variable used to return value of
	communication routine*/
	SMA13X_RETURN_FUNCTION_TYPE com_rslt = ERROR;

	if (p_sma13x == SMA13X_NULL) {
		/* Check the struct p_sma13x is empty */
		return E_SMA13X_NULL_PTR;
		} else {
		/* Write the data to the register*/
		com_rslt = p_sma13x->SMA13X_BUS_WRITE_FUNC
		(p_sma13x->dev_addr, adr_u8, data_u8, len_u8);

		if (p_sma13x->power_mode_u8 != SMA13X_MODE_NORMAL) {
			/*A minimum interface idle time delay
			of atleast 450us is required as per the data sheet.*/
			p_sma13x->delay_msec(SMA13X_INTERFACE_IDLE_TIME_DELAY);
		}
	}
	return com_rslt;
}
/*!
 * @brief This API reads the data from
 *           the given register address
 *
 *
 *	@param adr_u8 -> Address of the register
 *	@param data_u8 -> The data from the register
 *	@param len_u8 -> no of bytes to read
 *
 *
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
 *
*/
SMA13X_RETURN_FUNCTION_TYPE sma13x_read_reg(u8 adr_u8,
u8 *data_u8, u8 len_u8)
{
	/*  Variable used to return value of
	communication routine*/
	SMA13X_RETURN_FUNCTION_TYPE com_rslt = ERROR;

	if (p_sma13x == SMA13X_NULL) {
		/* Check the struct p_sma13x is empty */
		return E_SMA13X_NULL_PTR;
		} else {
			/*Read the data from the register*/
			com_rslt = p_sma13x->SMA13X_BUS_READ_FUNC
			(p_sma13x->dev_addr, adr_u8, data_u8, len_u8);
		}
	return com_rslt;
}
/*!
 * @brief
 *	This API reads acceleration data X values
 *	from location 02h and 03h
 *
 *
 *  @param   accel_x_s16 : pointer holding the data of accel X
 *		       value       |   resolution
 *       ----------------- | --------------
 *              0          | SMA13X_12_RESOLUTION
 *              1          | SMA13X_10_RESOLUTION
 *              2          | SMA13X_14_RESOLUTION
 *
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
 *
 */
SMA13X_RETURN_FUNCTION_TYPE sma13x_read_accel_x(s16 *accel_x_s16)
{
	/*  Variable used to return value of
	communication routine*/
	SMA13X_RETURN_FUNCTION_TYPE com_rslt = ERROR;
	/* Array holding the accel x value
	data_u8[0] - x->LSB
	data_u8[1] - x->MSB
	*/
	u8	data_u8[SMA13X_ACCEL_DATA_SIZE] = {
	SMA13X_INIT_VALUE, SMA13X_INIT_VALUE};
	if (p_sma13x == SMA13X_NULL) {
		/* Check the struct p_sma13x is empty */
		return E_SMA13X_NULL_PTR;
		} else {
		switch (V_SMA13XRESOLUTION_U8) {
		/* This case used for the resolution bit 12*/
		case SMA13X_12_RESOLUTION:
			com_rslt = p_sma13x->SMA13X_BUS_READ_FUNC
			(p_sma13x->dev_addr,
			SMA13X_ACCEL_X12_LSB_REG, data_u8,
			SMA13X_LSB_MSB_READ_LENGTH);
			*accel_x_s16 = (s16)((((s32)((s8)
			data_u8[SMA13X_SENSOR_DATA_ACCEL_MSB]))
			<< SMA13X_SHIFT_EIGHT_BITS) |
			(data_u8[SMA13X_SENSOR_DATA_ACCEL_LSB] &
			SMA13X_RESOLUTION_12_MASK));
			*accel_x_s16 = *accel_x_s16 >>
			SMA13X_SHIFT_FOUR_BITS;
		break;
		/* This case used for the resolution bit 10*/
		case SMA13X_10_RESOLUTION:
			com_rslt = p_sma13x->SMA13X_BUS_READ_FUNC
			(p_sma13x->dev_addr,
			SMA13X_ACCEL_X10_LSB_REG, data_u8,
			SMA13X_LSB_MSB_READ_LENGTH);
			*accel_x_s16 = (s16)((((s32)((s8)
			data_u8[SMA13X_SENSOR_DATA_ACCEL_MSB]))
			<< SMA13X_SHIFT_EIGHT_BITS) |
			(data_u8[SMA13X_SENSOR_DATA_ACCEL_LSB] &
			SMA13X_RESOLUTION_10_MASK));
			*accel_x_s16 = *accel_x_s16 >>
			SMA13X_SHIFT_SIX_BITS;
		break;
		/* This case used for the resolution bit 14*/
		case SMA13X_14_RESOLUTION:
			com_rslt = p_sma13x->SMA13X_BUS_READ_FUNC
			(p_sma13x->dev_addr,
			SMA13X_ACCEL_X14_LSB_REG, data_u8,
			SMA13X_LSB_MSB_READ_LENGTH);
			*accel_x_s16 = (s16)((((s32)((s8)
			data_u8[SMA13X_SENSOR_DATA_ACCEL_MSB]))
			<< SMA13X_SHIFT_EIGHT_BITS) |
			(data_u8[SMA13X_SENSOR_DATA_ACCEL_LSB] &
			SMA13X_RESOLUTION_14_MASK));
			*accel_x_s16 = *accel_x_s16 >>
			SMA13X_SHIFT_TWO_BITS;
		break;
		default:
		break;
		}
	}
	return com_rslt;
}
/*!
 * @brief
 *	This API reads acceleration data X values
 *	from location 02h and 03h bit resolution support 8bit
 *
 *
 *  @param   accel_x_s8 : pointer holding the data of accel X
 *
 *
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
 *
 */
SMA13X_RETURN_FUNCTION_TYPE sma13x_read_accel_eight_resolution_x(
s8 *accel_x_s8)
{
	/*  Variable used to return value of
	communication routine*/
	SMA13X_RETURN_FUNCTION_TYPE com_rslt = ERROR;
	u8	data = SMA13X_INIT_VALUE;

	if (p_sma13x == SMA13X_NULL) {
		/* Check the struct p_sma13x is empty */
		return E_SMA13X_NULL_PTR;
		} else {
			/* Read the sensor X data*/
			com_rslt = p_sma13x->SMA13X_BUS_READ_FUNC
			(p_sma13x->dev_addr,
			SMA13X_X_AXIS_MSB_ADDR, &data,
			SMA13X_GEN_READ_WRITE_LENGTH);
			*accel_x_s8 = SMA13X_GET_BITSLICE(data,
			SMA13X_ACCEL_X_MSB);
		}
	return com_rslt;
}
/*!
 * @brief
 *	This API reads acceleration data Y values
 *	from location 04h and 05h
 *
 *  @param   accel_y_s16 : pointer holding the data of accel Y
 *		       value       |   resolution
 *       ----------------- | --------------
 *              0          | SMA13X_12_RESOLUTION
 *              1          | SMA13X_10_RESOLUTION
 *              2          | SMA13X_14_RESOLUTION
 *
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
 *
*/
SMA13X_RETURN_FUNCTION_TYPE sma13x_read_accel_y(s16 *accel_y_s16)
{
	/*  Variable used to return value of
	communication routine*/
	SMA13X_RETURN_FUNCTION_TYPE com_rslt = ERROR;
	/* Array holding the accel y value
	data_u8[0] - y->LSB
	data_u8[1] - y->MSB
	*/
	u8 data_u8[SMA13X_ACCEL_DATA_SIZE] = {SMA13X_INIT_VALUE,
	SMA13X_INIT_VALUE};

	if (p_sma13x == SMA13X_NULL) {
		/* Check the struct p_sma13x is empty */
		return E_SMA13X_NULL_PTR;
		} else {
		switch (V_SMA13XRESOLUTION_U8) {
		/* This case used for the resolution bit 12*/
		case SMA13X_12_RESOLUTION:
			com_rslt = p_sma13x->SMA13X_BUS_READ_FUNC
			(p_sma13x->dev_addr,
			SMA13X_ACCEL_Y12_LSB_REG, data_u8,
			SMA13X_LSB_MSB_READ_LENGTH);
			*accel_y_s16 = (s16)((((s32)((s8)
			data_u8[SMA13X_SENSOR_DATA_ACCEL_MSB]))
			<< SMA13X_SHIFT_EIGHT_BITS) |
			(data_u8[SMA13X_SENSOR_DATA_ACCEL_LSB] &
			SMA13X_12_BIT_SHIFT));
			*accel_y_s16 = *accel_y_s16 >>
			SMA13X_SHIFT_FOUR_BITS;
		break;
		/* This case used for the resolution bit 10*/
		case SMA13X_10_RESOLUTION:
			com_rslt = p_sma13x->SMA13X_BUS_READ_FUNC
			(p_sma13x->dev_addr,
			SMA13X_ACCEL_Y10_LSB_REG, data_u8,
			SMA13X_LSB_MSB_READ_LENGTH);
			*accel_y_s16 = (s16)((((s32)((s8)
			data_u8[SMA13X_SENSOR_DATA_ACCEL_MSB]))
			<< SMA13X_SHIFT_EIGHT_BITS) |
			(data_u8[SMA13X_SENSOR_DATA_ACCEL_LSB] &
			SMA13X_10_BIT_SHIFT));
			*accel_y_s16 = *accel_y_s16 >>
			SMA13X_SHIFT_SIX_BITS;
		break;
		/* This case used for the resolution bit 14*/
		case SMA13X_14_RESOLUTION:
			com_rslt = p_sma13x->SMA13X_BUS_READ_FUNC
			(p_sma13x->dev_addr,
			SMA13X_ACCEL_Y14_LSB_REG, data_u8,
			SMA13X_LSB_MSB_READ_LENGTH);
			*accel_y_s16 = (s16)((((s32)((s8)
			data_u8[SMA13X_SENSOR_DATA_ACCEL_MSB]))
			<< SMA13X_SHIFT_EIGHT_BITS) |
			(data_u8[SMA13X_SENSOR_DATA_ACCEL_LSB] &
			SMA13X_14_BIT_SHIFT));
			*accel_y_s16 = *accel_y_s16 >>
			SMA13X_SHIFT_TWO_BITS;
		break;
		default:
		break;
		}
	}
	return com_rslt;
}
/*!
 * @brief This API reads acceleration data Y values of
 * 8bit  resolution  from location 05h
 *
 *
 *
 *
 *  @param accel_y_s8   The data of y
 *
 *
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
 *
*/
SMA13X_RETURN_FUNCTION_TYPE sma13x_read_accel_eight_resolution_y(
s8 *accel_y_s8)
{
		/*  Variable used to return value of
	communication routine*/
	SMA13X_RETURN_FUNCTION_TYPE com_rslt = ERROR;
	u8	data = SMA13X_INIT_VALUE;

	if (p_sma13x == SMA13X_NULL) {
		/* Check the struct p_sma13x is empty */
		return E_SMA13X_NULL_PTR;
		} else {
			com_rslt = p_sma13x->SMA13X_BUS_READ_FUNC
			(p_sma13x->dev_addr,
			SMA13X_Y_AXIS_MSB_ADDR, &data,
			SMA13X_GEN_READ_WRITE_LENGTH);
			*accel_y_s8 = SMA13X_GET_BITSLICE(data,
			SMA13X_ACCEL_Y_MSB);
		}
	return com_rslt;
}
/*!
 * @brief This API reads acceleration data Z values
 *                          from location 06h and 07h
 *
 *
 *  @param   accel_z_s16 : pointer holding the data of accel Z
 *		       value       |   resolution
 *       ----------------- | --------------
 *              0          | SMA13X_12_RESOLUTION
 *              1          | SMA13X_10_RESOLUTION
 *              2          | SMA13X_14_RESOLUTION
 *
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
 *
*/
SMA13X_RETURN_FUNCTION_TYPE sma13x_read_accel_z(s16 *accel_z_s16)
{
	/*  Variable used to return value of
	communication routine*/
	SMA13X_RETURN_FUNCTION_TYPE com_rslt = ERROR;
	/* Array holding the accel z value
	data_u8[0] - z->LSB
	data_u8[1] - z->MSB
	*/
	u8 data_u8[SMA13X_ACCEL_DATA_SIZE] = {SMA13X_INIT_VALUE,
	SMA13X_INIT_VALUE};

	if (p_sma13x == SMA13X_NULL) {
		/* Check the struct p_sma13x is empty */
		return E_SMA13X_NULL_PTR;
		} else {
		switch (V_SMA13XRESOLUTION_U8) {
		case SMA13X_12_RESOLUTION:
			/* This case used for the resolution bit 12*/
			com_rslt = p_sma13x->SMA13X_BUS_READ_FUNC
			(p_sma13x->dev_addr,
			SMA13X_ACCEL_Z12_LSB_REG, data_u8,
			SMA13X_LSB_MSB_READ_LENGTH);
			*accel_z_s16 = (s16)((((s32)((s8)
			data_u8[SMA13X_SENSOR_DATA_ACCEL_MSB]))
			<< SMA13X_SHIFT_EIGHT_BITS) |
			(data_u8[SMA13X_SENSOR_DATA_ACCEL_LSB]
			& SMA13X_12_BIT_SHIFT));
			*accel_z_s16 = *accel_z_s16 >>
			SMA13X_SHIFT_FOUR_BITS;
		break;
		/* This case used for the resolution bit 10*/
		case SMA13X_10_RESOLUTION:
			com_rslt = p_sma13x->SMA13X_BUS_READ_FUNC
			(p_sma13x->dev_addr,
			SMA13X_ACCEL_Z10_LSB_REG, data_u8,
			SMA13X_LSB_MSB_READ_LENGTH);
			*accel_z_s16 = (s16)((((s32)((s8)
			data_u8[SMA13X_SENSOR_DATA_ACCEL_MSB]))
			<< SMA13X_SHIFT_EIGHT_BITS) |
			(data_u8[SMA13X_SENSOR_DATA_ACCEL_LSB]
			& SMA13X_10_BIT_SHIFT));
			*accel_z_s16 = *accel_z_s16 >>
			SMA13X_SHIFT_SIX_BITS;
		break;
		/* This case used for the resolution bit 14*/
		case SMA13X_14_RESOLUTION:
			com_rslt = p_sma13x->SMA13X_BUS_READ_FUNC
			(p_sma13x->dev_addr,
			SMA13X_ACCEL_Z14_LSB_REG, data_u8,
			SMA13X_LSB_MSB_READ_LENGTH);
			*accel_z_s16 = (s16)((((s32)((s8)
			data_u8[SMA13X_SENSOR_DATA_ACCEL_MSB]))
			<< SMA13X_SHIFT_EIGHT_BITS) |
			(data_u8[SMA13X_SENSOR_DATA_ACCEL_LSB]
			& SMA13X_14_BIT_SHIFT));
			*accel_z_s16 = *accel_z_s16 >>
			SMA13X_SHIFT_TWO_BITS;
		break;
		default:
		break;
		}
	}
	return com_rslt;
}
/*!
 * @brief
 *	This API reads acceleration data Z values of
 *	8bit  resolution  from location 07h
 *
 *
 *
 *
 *  \@aram  accel_z_s8 : the data of z
 *
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
 *
*/
SMA13X_RETURN_FUNCTION_TYPE sma13x_read_accel_eight_resolution_z(
s8 *accel_z_s8)
{
		/*  Variable used to return value of
	communication routine*/
	SMA13X_RETURN_FUNCTION_TYPE com_rslt = ERROR;
	u8	data = SMA13X_INIT_VALUE;

	if (p_sma13x == SMA13X_NULL) {
		/* Check the struct p_sma13x is empty */
		return E_SMA13X_NULL_PTR;
		} else {
			com_rslt = p_sma13x->SMA13X_BUS_READ_FUNC
			(p_sma13x->dev_addr,
			SMA13X_Z_AXIS_MSB_ADDR, &data,
			SMA13X_GEN_READ_WRITE_LENGTH);
			*accel_z_s8 = SMA13X_GET_BITSLICE(data,
			SMA13X_ACCEL_Z_MSB);
		}
	return com_rslt;
}
/*!
 *	@brief This API reads acceleration data X,Y,Z values
 *	from location 02h to 07h
 *
 *  @param accel : pointer holding the data of accel
 *		       value       |   resolution
 *       ----------------- | --------------
 *              0          | SMA13X_12_RESOLUTION
 *              1          | SMA13X_10_RESOLUTION
 *              2          | SMA13X_14_RESOLUTION
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
 *
*/
SMA13X_RETURN_FUNCTION_TYPE sma13x_read_accel_xyz(
struct sma13x_accel_data *accel)
{
	/*  Variable used to return value of
	communication routine*/
	SMA13X_RETURN_FUNCTION_TYPE com_rslt = ERROR;
	/* Array holding the accel xyz value
	data_u8[0] - x->LSB
	data_u8[1] - x->MSB
	data_u8[2] - y->MSB
	data_u8[3] - y->MSB
	data_u8[4] - z->MSB
	data_u8[5] - z->MSB
	*/
	u8 data_u8[SMA13X_ACCEL_XYZ_DATA_SIZE] = {
	SMA13X_INIT_VALUE, SMA13X_INIT_VALUE,
	SMA13X_INIT_VALUE, SMA13X_INIT_VALUE,
	SMA13X_INIT_VALUE, SMA13X_INIT_VALUE};

	if (p_sma13x == SMA13X_NULL) {
		/* Check the struct p_sma13x is empty */
		return E_SMA13X_NULL_PTR;
		} else {
		switch (V_SMA13XRESOLUTION_U8) {
		/* This case used for the resolution bit 12*/
		case SMA13X_12_RESOLUTION:
			com_rslt = p_sma13x->SMA13X_BUS_READ_FUNC
			(p_sma13x->dev_addr, SMA13X_ACCEL_X12_LSB_REG,
			data_u8, SMA13X_SHIFT_SIX_BITS);
			/* read the x data_u8*/
			accel->x = (s16)((((s32)((s8)
			data_u8[SMA13X_SENSOR_DATA_XYZ_X_MSB]))
			<< SMA13X_SHIFT_EIGHT_BITS) |
			(data_u8[SMA13X_SENSOR_DATA_XYZ_X_LSB] &
			SMA13X_12_BIT_SHIFT));
			accel->x = accel->x >> SMA13X_SHIFT_FOUR_BITS;

			/* read the y data_u8*/
			accel->y = (s16)((((s32)((s8)
			data_u8[SMA13X_SENSOR_DATA_XYZ_Y_MSB]))
			<< SMA13X_SHIFT_EIGHT_BITS) |
			(data_u8[SMA13X_SENSOR_DATA_XYZ_Y_LSB] &
			SMA13X_12_BIT_SHIFT));
			accel->y = accel->y >> SMA13X_SHIFT_FOUR_BITS;

			/* read the z data_u8*/
			accel->z = (s16)((((s32)((s8)
			data_u8[SMA13X_SENSOR_DATA_XYZ_Z_MSB]))
			<< SMA13X_SHIFT_EIGHT_BITS) |
			(data_u8[SMA13X_SENSOR_DATA_XYZ_Z_LSB] &
			SMA13X_12_BIT_SHIFT));
			accel->z = accel->z >> SMA13X_SHIFT_FOUR_BITS;

		break;
		case SMA13X_10_RESOLUTION:
		/* This case used for the resolution bit 10*/
			com_rslt = p_sma13x->SMA13X_BUS_READ_FUNC
			(p_sma13x->dev_addr, SMA13X_ACCEL_X10_LSB_REG,
			data_u8, SMA13X_SHIFT_SIX_BITS);
			/* read the x data_u8*/
			accel->x = (s16)((((s32)((s8)
			data_u8[SMA13X_SENSOR_DATA_XYZ_X_MSB]))
			<< SMA13X_SHIFT_EIGHT_BITS) |
			(data_u8[SMA13X_SENSOR_DATA_XYZ_X_LSB] &
			SMA13X_10_BIT_SHIFT));
			accel->x = accel->x >> SMA13X_SHIFT_SIX_BITS;

			/* read the y data_u8*/
			accel->y = (s16)((((s32)((s8)
			data_u8[SMA13X_SENSOR_DATA_XYZ_Y_MSB]))
			<< SMA13X_SHIFT_EIGHT_BITS) |
			(data_u8[SMA13X_SENSOR_DATA_XYZ_Y_LSB] &
			SMA13X_10_BIT_SHIFT));
			accel->y = accel->y >> SMA13X_SHIFT_SIX_BITS;

			/* read the z data_u8*/
			accel->z = (s16)((((s32)((s8)
			data_u8[SMA13X_SENSOR_DATA_XYZ_Z_MSB]))
			<< SMA13X_SHIFT_EIGHT_BITS) |
			(data_u8[SMA13X_SENSOR_DATA_XYZ_Z_LSB]
			& SMA13X_10_BIT_SHIFT));
			accel->z = accel->z >> SMA13X_SHIFT_SIX_BITS;
		break;
		/* This case used for the resolution bit 14*/
		case SMA13X_14_RESOLUTION:
			com_rslt = p_sma13x->SMA13X_BUS_READ_FUNC
			(p_sma13x->dev_addr, SMA13X_ACCEL_X14_LSB_REG,
			data_u8, SMA13X_SHIFT_SIX_BITS);

			/* read the x data_u8*/
			accel->x = (s16)((((s32)((s8)
			data_u8[SMA13X_SENSOR_DATA_XYZ_X_MSB]))<<
			SMA13X_SHIFT_EIGHT_BITS) |
			(data_u8[SMA13X_SENSOR_DATA_XYZ_X_LSB]
			& SMA13X_14_BIT_SHIFT));
			accel->x = accel->x >> SMA13X_SHIFT_TWO_BITS;

			/* read the y data_u8*/
			accel->y = (s16)((((s32)((s8)
			data_u8[SMA13X_SENSOR_DATA_XYZ_Y_MSB]))<<
			SMA13X_SHIFT_EIGHT_BITS) |
			(data_u8[SMA13X_SENSOR_DATA_XYZ_Y_LSB]
			& SMA13X_14_BIT_SHIFT));
			accel->y = accel->y >> SMA13X_SHIFT_TWO_BITS;

			/* read the z data_u8*/
			accel->z = (s16)((((s32)((s8)
			data_u8[SMA13X_SENSOR_DATA_XYZ_Z_MSB]))<<
			SMA13X_SHIFT_EIGHT_BITS) |
			(data_u8[SMA13X_SENSOR_DATA_XYZ_Z_LSB]
			& SMA13X_14_BIT_SHIFT));
			accel->z = accel->z >> SMA13X_SHIFT_TWO_BITS;
		break;
		default:
		break;
		}
	}
	return com_rslt;
}
/*!
 * @brief This API reads acceleration of 8 bit resolution
 * data of X,Y,Z values
 * from location 03h , 05h and 07h
 *
 *
 *
 *
 *  @param accel : pointer holding the data of accel
 *
 *
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
 *
*/
SMA13X_RETURN_FUNCTION_TYPE sma13x_read_accel_eight_resolution_xyz(
struct sma13x_accel_eight_resolution *accel)
{
	/*  Variable used to return value of
	communication routine*/
	SMA13X_RETURN_FUNCTION_TYPE com_rslt = ERROR;
	u8	data_u8 = SMA13X_INIT_VALUE;

	if (p_sma13x == SMA13X_NULL) {
		/* Check the struct p_sma13x is empty */
		return E_SMA13X_NULL_PTR;
		} else {
		com_rslt = p_sma13x->SMA13X_BUS_READ_FUNC
		(p_sma13x->dev_addr,
		SMA13X_X_AXIS_MSB_ADDR, &data_u8,
		SMA13X_GEN_READ_WRITE_LENGTH);
		accel->x = SMA13X_GET_BITSLICE(data_u8,
		SMA13X_ACCEL_X_MSB);

		com_rslt = p_sma13x->SMA13X_BUS_READ_FUNC
		(p_sma13x->dev_addr,
		SMA13X_Y_AXIS_MSB_ADDR, &data_u8,
		SMA13X_GEN_READ_WRITE_LENGTH);
		accel->y = SMA13X_GET_BITSLICE(data_u8,
		SMA13X_ACCEL_Y_MSB);

		com_rslt = p_sma13x->SMA13X_BUS_READ_FUNC
		(p_sma13x->dev_addr,
		SMA13X_Z_AXIS_MSB_ADDR, &data_u8,
		SMA13X_GEN_READ_WRITE_LENGTH);
		accel->z = SMA13X_GET_BITSLICE(data_u8,
		SMA13X_ACCEL_Z_MSB);
		}
	return com_rslt;
}
/*!
 *	@brief This API read interrupt status of slow no motion, slope, highg
 *	from location 09h
 *
 *
 *
 *	@param  intr_stat_u8 : The value of interrupt status
 *
 *
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
 *
*/
SMA13X_RETURN_FUNCTION_TYPE sma13x_get_intr_stat(
u8 *intr_stat_u8)
{
	/*  Variable used to return value of
	communication routine*/
	SMA13X_RETURN_FUNCTION_TYPE com_rslt = ERROR;

	if (p_sma13x == SMA13X_NULL) {
		/* Check the struct p_sma13x is empty */
		return E_SMA13X_NULL_PTR;
		} else {
			/* Read the interrupt status register 0x09*/
			com_rslt = p_sma13x->SMA13X_BUS_READ_FUNC(
			p_sma13x->dev_addr,
			SMA13X_STAT1_ADDR, intr_stat_u8,
			SMA13X_SHIFT_FOUR_BITS);
		}
	return com_rslt;
}
/*!
 * @brief This API is used to get the ranges(g values) of the sensor
 *	in the register 0x0F bit from 0 to 3
 *
 *
 *	@param range_u8 : The value of range
 *		  range_u8       |   result
 *       ----------------- | --------------
 *              0x03       | SMA13X_RANGE_2G
 *              0x05       | SMA13X_RANGE_4G
 *              0x08       | SMA13X_RANGE_8G
 *              0x0C       | SMA13X_RANGE_16G
 *
 *
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
 *
 */
SMA13X_RETURN_FUNCTION_TYPE sma13x_get_range(u8 *range_u8)
{
	/*  Variable used to return value of
	communication routine*/
	SMA13X_RETURN_FUNCTION_TYPE com_rslt = ERROR;
	u8 data_u8 = SMA13X_INIT_VALUE;

	if (p_sma13x == SMA13X_NULL) {
		/* Check the struct p_sma13x is empty */
		return E_SMA13X_NULL_PTR;
		} else {
		/* Read the range register 0x0F*/
		com_rslt = p_sma13x->SMA13X_BUS_READ_FUNC(p_sma13x->dev_addr,
		SMA13X_RANGE_SELECT_REG, &data_u8,
		SMA13X_GEN_READ_WRITE_LENGTH);
		data_u8 = SMA13X_GET_BITSLICE(data_u8, SMA13X_RANGE_SELECT);
		*range_u8 = data_u8;
	}
	return com_rslt;
}
/*!
 * @brief This API is used to set the ranges(g values) of the sensor
 *	in the register 0x0F bit from 0 to 3
 *
 *
 *	@param range_u8 : The value of range
 *		  range_u8 |   result
 *       ----------------- | --------------
 *              0x03       | SMA13X_RANGE_2G
 *              0x05       | SMA13X_RANGE_4G
 *              0x08       | SMA13X_RANGE_8G
 *              0x0C       | SMA13X_RANGE_16G
 *
 *
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
 *
 */
SMA13X_RETURN_FUNCTION_TYPE sma13x_set_range(u8 range_u8)
{
	/*  Variable used to return value of
	communication routine*/
	SMA13X_RETURN_FUNCTION_TYPE com_rslt = ERROR;
	u8 data_u8 = SMA13X_INIT_VALUE;

	if (p_sma13x == SMA13X_NULL) {
		/* Check the struct p_sma13x is empty */
		return E_SMA13X_NULL_PTR;
		} else {
		if ((range_u8 == SMA13X_RANGE_2G) ||
		(range_u8 == SMA13X_RANGE_4G) ||
		(range_u8 == SMA13X_RANGE_8G) ||
		(range_u8 == SMA13X_RANGE_16G)) {
			com_rslt = p_sma13x->SMA13X_BUS_READ_FUNC
			(p_sma13x->dev_addr,
			SMA13X_RANGE_SELECT_REG, &data_u8,
			SMA13X_GEN_READ_WRITE_LENGTH);
			switch (range_u8) {
			case SMA13X_RANGE_2G:
				data_u8  = SMA13X_SET_BITSLICE(data_u8,
				SMA13X_RANGE_SELECT,
				SMA13X_RANGE_2G);
			break;
			case SMA13X_RANGE_4G:
				data_u8  = SMA13X_SET_BITSLICE(data_u8,
				SMA13X_RANGE_SELECT,
				SMA13X_RANGE_4G);
			break;
			case SMA13X_RANGE_8G:
				data_u8  = SMA13X_SET_BITSLICE(data_u8,
				SMA13X_RANGE_SELECT,
				SMA13X_RANGE_8G);
			break;
			case SMA13X_RANGE_16G:
				data_u8  = SMA13X_SET_BITSLICE(data_u8,
				SMA13X_RANGE_SELECT,
				SMA13X_RANGE_16G);
			break;
			default:
			break;
			}
			/* Write the range register 0x0F*/
			com_rslt += sma13x_write_reg(SMA13X_RANGE_SELECT_REG,
				&data_u8, SMA13X_GEN_READ_WRITE_LENGTH);
		} else {
		com_rslt = E_OUT_OF_RANGE;
		}
	}
	return com_rslt;
}
/*!
 *  @brief This API is used to get the bandwidth of the sensor in the register
 *  0x10 bit from 0 to 4
 *
 *
 *  @param bw_u8 : The value of bandwidth
 *          bw_u8          |   result
 *       ----------------- | --------------
 *              0x08       | SMA13X_BW_7_81HZ
 *              0x09       | SMA13X_BW_15_63HZ
 *              0x0A       | SMA13X_BW_31_25HZ
 *              0x0B       | SMA13X_BW_62_50HZ
 *              0x0C       | SMA13X_BW_125HZ
 *              0x0D       | SMA13X_BW_250HZ
 *              0x0E       | SMA13X_BW_500HZ
 *              0x0F       | SMA13X_BW_1000HZ
 *
 *
 *
 *  @return results of bus communication function
 *  @retval 0 -> Success
 *  @retval -1 -> Error
 *
 *
*/
SMA13X_RETURN_FUNCTION_TYPE sma13x_get_bw(u8 *bw_u8)
{
	/*  Variable used to return value of
	communication routine*/
	SMA13X_RETURN_FUNCTION_TYPE com_rslt = ERROR;
	u8 data_u8 = SMA13X_INIT_VALUE;

	if (p_sma13x == SMA13X_NULL) {
		/* Check the struct p_sma13x is empty */
		return E_SMA13X_NULL_PTR;
		} else {
			/* Read the bandwidth register 0x10*/
			com_rslt = p_sma13x->SMA13X_BUS_READ_FUNC
			(p_sma13x->dev_addr,
			SMA13X_BW_REG, &data_u8,
			SMA13X_GEN_READ_WRITE_LENGTH);
			data_u8 = SMA13X_GET_BITSLICE(data_u8, SMA13X_BW);
			*bw_u8 = data_u8;
		}
	return com_rslt;
}
/*!
 *	@brief This API is used to set the bandwidth of the sensor
 *      in the register
 *	0x10 bit from 0 to 4
 *
 *
 *  @param bw_u8 : The value of bandwidth
 *		  bw_u8          |   result
 *       ----------------- | --------------
 *              0x08       | SMA13X_BW_7_81HZ
 *              0x09       | SMA13X_BW_15_63HZ
 *              0x0A       | SMA13X_BW_31_25HZ
 *              0x0B       | SMA13X_BW_62_50HZ
 *              0x0C       | SMA13X_BW_125HZ
 *              0x0D       | SMA13X_BW_250HZ
 *              0x0E       | SMA13X_BW_500HZ
 *              0x0F       | SMA13X_BW_1000HZ
 *
 *
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
 *
*/
SMA13X_RETURN_FUNCTION_TYPE sma13x_set_bw(u8 bw_u8)
{
/*  Variable used to return value of
communication routine*/
SMA13X_RETURN_FUNCTION_TYPE com_rslt = ERROR;
u8 data_u8 = SMA13X_INIT_VALUE;
u8 data_bw_u8 = SMA13X_INIT_VALUE;
if (p_sma13x == SMA13X_NULL) {
		/* Check the struct p_sma13x is empty */
		com_rslt = E_SMA13X_NULL_PTR;
	} else {
	/* Check the chip id 0xFB, it support upto 500Hz*/
	if (p_sma13x->chip_id == BANDWIDTH_DEFINE) {
		if (bw_u8 > SMA13X_ACCEL_BW_MIN_RANGE &&
		bw_u8 < SMA13X_ACCEL_BW_1000HZ_RANGE) {
			switch (bw_u8) {
			case SMA13X_BW_7_81HZ:
				data_bw_u8 = SMA13X_BW_7_81HZ;

				/*  7.81 Hz      64000 uS   */
			break;
			case SMA13X_BW_15_63HZ:
				data_bw_u8 = SMA13X_BW_15_63HZ;

			/*  15.63 Hz     32000 uS   */
			break;
			case SMA13X_BW_31_25HZ:
				data_bw_u8 = SMA13X_BW_31_25HZ;

			/*  31.25 Hz     16000 uS   */
			break;
			case SMA13X_BW_62_50HZ:
				data_bw_u8 = SMA13X_BW_62_50HZ;

			/*  62.50 Hz     8000 uS   */
			break;
			case SMA13X_BW_125HZ:
				data_bw_u8 = SMA13X_BW_125HZ;

			/*  125 Hz       4000 uS   */
			break;
			case SMA13X_BW_250HZ:
				data_bw_u8 = SMA13X_BW_250HZ;

			/*  250 Hz       2000 uS   */
			break;
			case SMA13X_BW_500HZ:
				data_bw_u8 = SMA13X_BW_500HZ;

			/*  500 Hz       1000 uS   */
			break;
			default:
			break;
			}
			/* Write the bandwidth register */
			com_rslt = p_sma13x->SMA13X_BUS_READ_FUNC
			(p_sma13x->dev_addr,
			SMA13X_BW_REG, &data_u8,
			SMA13X_GEN_READ_WRITE_LENGTH);
			data_u8 = SMA13X_SET_BITSLICE(data_u8,
			SMA13X_BW, data_bw_u8);
			com_rslt += sma13x_write_reg
			(SMA13X_BW_REG, &data_u8,
			SMA13X_GEN_READ_WRITE_LENGTH);
			} else {
			com_rslt = E_OUT_OF_RANGE;
			}
		} else {
		if (bw_u8 > SMA13X_ACCEL_BW_MIN_RANGE &&
		bw_u8 < SMA13X_ACCEL_BW_MAX_RANGE) {
			switch (bw_u8) {
			case SMA13X_BW_7_81HZ:
				data_bw_u8 = SMA13X_BW_7_81HZ;

			/*  7.81 Hz      64000 uS   */
			break;
			case SMA13X_BW_15_63HZ:
				data_bw_u8 = SMA13X_BW_15_63HZ;

			/*  15.63 Hz     32000 uS   */
			break;
			case SMA13X_BW_31_25HZ:
				data_bw_u8 = SMA13X_BW_31_25HZ;

			/*  31.25 Hz     16000 uS   */
			break;
			case SMA13X_BW_62_50HZ:
				data_bw_u8 = SMA13X_BW_62_50HZ;

			/*  62.50 Hz     8000 uS   */
			break;
			case SMA13X_BW_125HZ:
				data_bw_u8 = SMA13X_BW_125HZ;

			/*  125 Hz       4000 uS   */
			break;
			case SMA13X_BW_250HZ:
				data_bw_u8 = SMA13X_BW_250HZ;

			/*  250 Hz       2000 uS   */
			break;
			case SMA13X_BW_500HZ:
				data_bw_u8 = SMA13X_BW_500HZ;

			/*!  500 Hz       1000 uS   */
			break;
			case SMA13X_BW_1000HZ:
				data_bw_u8 = SMA13X_BW_1000HZ;

			/*  1000 Hz      500 uS   */
			break;
			default:
			break;
			}
			/* Write the bandwidth register */
			com_rslt = p_sma13x->SMA13X_BUS_READ_FUNC
			(p_sma13x->dev_addr,
			SMA13X_BW_REG, &data_u8,
			SMA13X_GEN_READ_WRITE_LENGTH);
			data_u8 = SMA13X_SET_BITSLICE
			(data_u8, SMA13X_BW, data_bw_u8);
			com_rslt += sma13x_write_reg(
			SMA13X_BW_REG, &data_u8,
			SMA13X_GEN_READ_WRITE_LENGTH);
			} else {
			com_rslt = E_OUT_OF_RANGE;
			}
		}
	}
	return com_rslt;
}
/*!
 *	@brief This API is used to get the operating
 *	modes of the sensor in the register 0x11 and 0x12
 *	@note Register 0x11 - bit from 5 to 7
 *	@note Register 0x12 - bit from 5 and 6
 *
 *
 *  @param power_mode_u8 : The value of power mode
 *	power_mode_u8           |value  |   0x11  |   0x12
 *  ------------------------- |-------| --------|--------
 *  SMA13X_MODE_NORMAL        |  0    |  0x00   |  0x00
 *  SMA13X_MODE_LOWPOWER1     |  1    |  0x02   |  0x00
 *  SMA13X_MODE_SUSPEND       |  2    |  0x06   |  0x00
 *  SMA13X_MODE_DEEP_SUSPEND  |  3    |  0x01   |  0x00
 *
 *
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
 *
 */
SMA13X_RETURN_FUNCTION_TYPE sma13x_get_power_mode(
u8 *power_mode_u8)
{
	/*  Variable used to return value of
	communication routine*/
SMA13X_RETURN_FUNCTION_TYPE com_rslt = ERROR;
u8 data_u8 = SMA13X_INIT_VALUE;
u8 data2_u8 = SMA13X_INIT_VALUE;
if (p_sma13x == SMA13X_NULL) {
	/* Check the struct p_sma13x is empty */
		com_rslt = E_SMA13X_NULL_PTR;
	} else {
		com_rslt = p_sma13x->SMA13X_BUS_READ_FUNC
		(p_sma13x->dev_addr, SMA13X_MODE_CTRL_REG,
		&data_u8, SMA13X_GEN_READ_WRITE_LENGTH);
		com_rslt += p_sma13x->SMA13X_BUS_READ_FUNC
		(p_sma13x->dev_addr, SMA13X_LOW_NOISE_CTRL_ADDR,
		&data2_u8, SMA13X_GEN_READ_WRITE_LENGTH);

		data_u8  = (data_u8 &
		SMA13X_POWER_MODE_HEX_E_ZERO_MASK) >>
		SMA13X_SHIFT_FIVE_BITS;
		data2_u8  = (data2_u8 &
		SMA13X_POWER_MODE_HEX_4_ZERO_MASK) >>
		SMA13X_SHIFT_SIX_BITS;

	if ((data_u8 ==
	SMA13X_POWER_MODE_HEX_ZERO_ZERO_MASK) &&
	(data2_u8 ==
	SMA13X_POWER_MODE_HEX_ZERO_ZERO_MASK)) {
		*power_mode_u8  = SMA13X_MODE_NORMAL;
		} else {
		if ((data_u8 ==
		SMA13X_POWER_MODE_HEX_ZERO_TWO_MASK) &&
		(data2_u8 ==
		SMA13X_POWER_MODE_HEX_ZERO_ZERO_MASK)) {
			*power_mode_u8  =
			SMA13X_MODE_LOWPOWER1;
			} else {
			if ((data_u8 ==
			SMA13X_POWER_MODE_HEX_ZERO_FOUR_MASK
			|| data_u8 ==
			SMA13X_POWER_MODE_HEX_ZERO_SIX_MASK) &&
			(data2_u8 ==
			SMA13X_POWER_MODE_HEX_ZERO_ZERO_MASK)) {
				*power_mode_u8  =
				SMA13X_MODE_SUSPEND;
				} else {
					*power_mode_u8  =
					SMA13X_MODE_DEEP_SUSPEND;
				}
			}
		}
	}
	p_sma13x->power_mode_u8 = *power_mode_u8;
return com_rslt;
}
/*!
 *	@brief This API is used to set the operating
 *	modes of the sensor in the register 0x11 and 0x12
 *	@note Register 0x11 - bit from 5 to 7
 *	@note Register 0x12 - bit from 5 and 6
 *
 *
 *  @param power_mode_u8 : The value of power mode
 *	power_mode_u8         |value  |   0x11  |   0x12
 *  ------------------------- |-------| --------|--------
 *  SMA13X_MODE_NORMAL        |  0    |  0x00   |  0x00
 *  SMA13X_MODE_LOWPOWER1     |  1    |  0x02   |  0x00
 *  SMA13X_MODE_SUSPEND       |  2    |  0x06   |  0x00
 *  SMA13X_MODE_DEEP_SUSPEND  |  3    |  0x01   |  0x00
 *
 *
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
 *
 */
SMA13X_RETURN_FUNCTION_TYPE sma13x_set_power_mode(u8 power_mode_u8)
{
		/*  Variable used to return value of
	communication routine*/
	SMA13X_RETURN_FUNCTION_TYPE com_rslt = ERROR;
	u8 mode_ctr_eleven_reg = SMA13X_INIT_VALUE;
	u8 mode_ctr_twel_reg = SMA13X_INIT_VALUE;
	u8 data_u8 = SMA13X_INIT_VALUE;
	u8 data2_u8 = SMA13X_INIT_VALUE;

	if (p_sma13x == SMA13X_NULL) {
		/* Check the struct p_sma13x is empty */
		com_rslt = E_SMA13X_NULL_PTR;
	} else {
		com_rslt = p_sma13x->SMA13X_BUS_READ_FUNC(p_sma13x->dev_addr,
			SMA13X_MODE_CTRL_REG, &data_u8, 1);
		com_rslt += p_sma13x->SMA13X_BUS_READ_FUNC(p_sma13x->dev_addr,
			SMA13X_LOW_POWER_MODE_REG, &data2_u8, 1);

		com_rslt += sma13x_set_mode_value(power_mode_u8);
		mode_ctr_eleven_reg = p_sma13x->ctrl_mode_reg;
		mode_ctr_twel_reg =  p_sma13x->low_mode_reg;

		/* write the power mode to the register 0x12*/
		data2_u8  = SMA13X_SET_BITSLICE(data2_u8, SMA13X_LOW_POWER_MODE,
					mode_ctr_twel_reg);
		com_rslt += sma13x_write_reg(SMA13X_LOW_POWER_MODE_REG,
					&data2_u8, 1);

		/*A minimum delay of atleast 450us is required for
		the low power modes, as per the data sheet.*/
		p_sma13x->delay_msec(SMA13X_INTERFACE_IDLE_TIME_DELAY);

		if ((p_sma13x->power_mode_u8 == SMA13X_MODE_LOWPOWER1) &&
				(power_mode_u8 == SMA13X_MODE_NORMAL)) {
				/* Enter the power mode to suspend*/
				data_u8  = SMA13X_SET_BITSLICE(data_u8,
				SMA13X_MODE_CTRL, SMA13X_SHIFT_FOUR_BITS);
				/* write the power mode to suspend*/
				com_rslt += sma13x_write_reg(
				SMA13X_MODE_CTRL_REG, &data_u8,
				SMA13X_GEN_READ_WRITE_LENGTH);
			}

		/* write the power mode to 0x11 register*/
		data_u8  = SMA13X_SET_BITSLICE(data_u8, SMA13X_MODE_CTRL,
			mode_ctr_eleven_reg);
		com_rslt += sma13x_write_reg(SMA13X_MODE_CTRL_REG, &data_u8, 1);
		/*A minimum delay of atleast 450us is required for
		the low power modes, as per the data sheet.*/
		p_sma13x->delay_msec(SMA13X_INTERFACE_IDLE_TIME_DELAY);

		/*Assigning the power mode to the global variable*/
		p_sma13x->power_mode_u8 = power_mode_u8;
	}
	return com_rslt;
}
/*!
 *	@brief This API is used to assign the power mode values
 *	modes of the sensor in the register 0x11 and 0x12
 *	@note Register 0x11 - bit from 5 to 7
 *	@note Register 0x12 - bit from 5 and 6
 *
 *
 *  @param power_mode_u8 : The value of power mode
 *	power_mode_u8           |value  |   0x11  |   0x12
 *  ------------------------- |-------| --------|--------
 *  SMA13X_MODE_NORMAL        |  0    |  0x00   |  0x00
 *  SMA13X_MODE_LOWPOWER1     |  1    |  0x02   |  0x00
 *  SMA13X_MODE_SUSPEND       |  2    |  0x06   |  0x00
 *  SMA13X_MODE_DEEP_SUSPEND  |  3    |  0x01   |  0x00
 *
 *
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
 *
 */
SMA13X_RETURN_FUNCTION_TYPE sma13x_set_mode_value(u8 power_mode_u8)
{
	SMA13X_RETURN_FUNCTION_TYPE com_rslt = SUCCESS;

	if (p_sma13x == SMA13X_NULL) {
		/* Check the struct p_sma13x is empty */
		com_rslt = E_SMA13X_NULL_PTR;
	} else {
	if (power_mode_u8 < SMA13X_POWER_MODE_RANGE) {
		switch (power_mode_u8)	{
		case SMA13X_MODE_NORMAL:
			p_sma13x->ctrl_mode_reg =
			SMA13X_POWER_MODE_HEX_ZERO_ZERO_MASK;
			p_sma13x->low_mode_reg =
			SMA13X_POWER_MODE_HEX_ZERO_ZERO_MASK;
		break;
		case SMA13X_MODE_LOWPOWER1:
			p_sma13x->ctrl_mode_reg =
			SMA13X_POWER_MODE_HEX_ZERO_TWO_MASK;
			p_sma13x->low_mode_reg =
			SMA13X_POWER_MODE_HEX_ZERO_ZERO_MASK;
		break;
		case SMA13X_MODE_SUSPEND:
			p_sma13x->ctrl_mode_reg =
			SMA13X_POWER_MODE_HEX_ZERO_FOUR_MASK;
			p_sma13x->low_mode_reg =
			SMA13X_POWER_MODE_HEX_ZERO_ZERO_MASK;
		break;
		case SMA13X_MODE_DEEP_SUSPEND:
			p_sma13x->ctrl_mode_reg =
			SMA13X_POWER_MODE_HEX_ZERO_ONE_MASK;
		break;
		}
		} else {
			com_rslt = E_OUT_OF_RANGE;
		}
	}
	return com_rslt;
}
/*!
 *	@brief This API is used to get
 *	the sleep duration of the sensor in the register 0x11
 *	Register 0x11 - bit from 0 to 3
 *
 *
 *  @param  sleep_durn_u8 : The value of sleep duration time
 *         sleep_durn_u8 |   result
 *       ----------------- | ----------------------
 *              0x05       | SMA13X_SLEEP_DURN_0_5MS
 *              0x06       | SMA13X_SLEEP_DURN_1MS
 *              0x07       | SMA13X_SLEEP_DURN_2MS
 *              0x08       | SMA13X_SLEEP_DURN_4MS
 *              0x09       | SMA13X_SLEEP_DURN_6MS
 *              0x0A       | SMA13X_SLEEP_DURN_10MS
 *              0x0B       | SMA13X_SLEEP_DURN_25MS
 *              0x0C       | SMA13X_SLEEP_DURN_50MS
 *              0x0D       | SMA13X_SLEEP_DURN_100MS
 *              0x0E       | SMA13X_SLEEP_DURN_500MS
 *              0x0F       | SMA13X_SLEEP_DURN_1S
 *
 *
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
 *
 */
SMA13X_RETURN_FUNCTION_TYPE sma13x_get_sleep_durn(u8 *sleep_durn_u8)
{
	u8 data_u8 = SMA13X_INIT_VALUE;
	/*  Variable used to return value of
	communication routine*/
	SMA13X_RETURN_FUNCTION_TYPE com_rslt = ERROR;

	if (p_sma13x == SMA13X_NULL) {
		/* Check the struct p_sma13x is empty */
		return E_SMA13X_NULL_PTR;
		} else {
			/* read the sleep duration */
			com_rslt = p_sma13x->SMA13X_BUS_READ_FUNC
			(p_sma13x->dev_addr, SMA13X_SLEEP_DURN_REG,
			&data_u8, SMA13X_GEN_READ_WRITE_LENGTH);
			*sleep_durn_u8 = SMA13X_GET_BITSLICE
			(data_u8, SMA13X_SLEEP_DURN);
		}
	return com_rslt;
}
/*!
 *	@brief This API is used to set
 *	the sleep duration of the sensor in the register 0x11
 *	Register 0x11 - bit from 0 to 3
 *
 *
 *
 *
 *  @param  sleep_durn_u8 : The value of sleep duration time
 *        sleep_durn_u8  |   result
 *       ----------------- | ----------------------
 *              0x05       | SMA13X_SLEEP_DURN_0_5MS
 *              0x06       | SMA13X_SLEEP_DURN_1MS
 *              0x07       | SMA13X_SLEEP_DURN_2MS
 *              0x08       | SMA13X_SLEEP_DURN_4MS
 *              0x09       | SMA13X_SLEEP_DURN_6MS
 *              0x0A       | SMA13X_SLEEP_DURN_10MS
 *              0x0B       | SMA13X_SLEEP_DURN_25MS
 *              0x0C       | SMA13X_SLEEP_DURN_50MS
 *              0x0D       | SMA13X_SLEEP_DURN_100MS
 *              0x0E       | SMA13X_SLEEP_DURN_500MS
 *              0x0F       | SMA13X_SLEEP_DURN_1S
 *
 *
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
 *
 */
SMA13X_RETURN_FUNCTION_TYPE sma13x_set_sleep_durn(u8 sleep_durn_u8)
{
	u8 data_u8 = SMA13X_INIT_VALUE;
	/*  Variable used to return value of
	communication routine*/
	SMA13X_RETURN_FUNCTION_TYPE com_rslt = ERROR;
	u8 data_sleep_durn_u8 = SMA13X_INIT_VALUE;

	if (p_sma13x == SMA13X_NULL) {
		/* Check the struct p_sma13x is empty */
		return E_SMA13X_NULL_PTR;
		} else {
		if (sleep_durn_u8 > SMA13X_SLEEP_DURN_MIN_RANGE &&
		sleep_durn_u8 < SMA13X_SLEEP_DURN_MAX_RANGE) {
			switch (sleep_durn_u8) {
			case SMA13X_SLEEP_DURN_0_5MS:
				data_sleep_durn_u8 = SMA13X_SLEEP_DURN_0_5MS;

				/*  0.5 MS   */
			break;
			case SMA13X_SLEEP_DURN_1MS:
				data_sleep_durn_u8 = SMA13X_SLEEP_DURN_1MS;

				/*  1 MS  */
			break;
			case SMA13X_SLEEP_DURN_2MS:
				data_sleep_durn_u8 = SMA13X_SLEEP_DURN_2MS;

				/*  2 MS  */
			break;
			case SMA13X_SLEEP_DURN_4MS:
				data_sleep_durn_u8 = SMA13X_SLEEP_DURN_4MS;

				/*  4 MS   */
			break;
			case SMA13X_SLEEP_DURN_6MS:
				data_sleep_durn_u8 = SMA13X_SLEEP_DURN_6MS;

				/*  6 MS  */
			break;
			case SMA13X_SLEEP_DURN_10MS:
				data_sleep_durn_u8 = SMA13X_SLEEP_DURN_10MS;

				/*  10 MS  */
			break;
			case SMA13X_SLEEP_DURN_25MS:
				data_sleep_durn_u8 = SMA13X_SLEEP_DURN_25MS;

				/*  25 MS  */
			break;
			case SMA13X_SLEEP_DURN_50MS:
				data_sleep_durn_u8 = SMA13X_SLEEP_DURN_50MS;

				/*  50 MS   */
			break;
			case SMA13X_SLEEP_DURN_100MS:
				data_sleep_durn_u8 = SMA13X_SLEEP_DURN_100MS;

				/*  100 MS  */
			break;
			case SMA13X_SLEEP_DURN_500MS:
				data_sleep_durn_u8 = SMA13X_SLEEP_DURN_500MS;

				/*  500 MS   */
			break;
			case SMA13X_SLEEP_DURN_1S:
				data_sleep_durn_u8 = SMA13X_SLEEP_DURN_1S;

				/*!  1 SECS   */
			break;
			default:
			break;
			}
			/* write the sleep duration */
			com_rslt = p_sma13x->SMA13X_BUS_READ_FUNC
			(p_sma13x->dev_addr, SMA13X_SLEEP_DURN_REG,
			&data_u8, SMA13X_GEN_READ_WRITE_LENGTH);
			data_u8 = SMA13X_SET_BITSLICE
			(data_u8, SMA13X_SLEEP_DURN, data_sleep_durn_u8);
			com_rslt += sma13x_write_reg(SMA13X_SLEEP_DURN_REG,
			&data_u8, SMA13X_GEN_READ_WRITE_LENGTH);
		} else {
		com_rslt = E_OUT_OF_RANGE;
		}
	}
	return com_rslt;
}
/*!
 * @brief This API is used to get the sleep timer mode
 *	in the register 0x12 bit 5
 *
 *
 *
 *
 *  @param  sleep_timer_u8 : The value of sleep timer mode
 *        sleep_timer_u8 |   result
 *       ----------------- | ----------------------
 *              0          | enable EventDrivenSampling(EDT)
 *              1          | enable Equidistant sampling mode(EST)
 *
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
 *
 */
SMA13X_RETURN_FUNCTION_TYPE sma13x_get_sleep_timer_mode(
u8 *sleep_timer_u8)
{
	u8 data_u8 = SMA13X_INIT_VALUE;
	/*  Variable used to return value of
	communication routine*/
	SMA13X_RETURN_FUNCTION_TYPE com_rslt = ERROR;

	if (p_sma13x == SMA13X_NULL) {
		/* Check the struct p_sma13x is empty */
		return E_SMA13X_NULL_PTR;
		} else {
			/*Read the SLEEP TIMER MODE*/
			com_rslt = p_sma13x->SMA13X_BUS_READ_FUNC
			(p_sma13x->dev_addr, SMA13X_SLEEP_TIMER_REG,
			&data_u8, SMA13X_GEN_READ_WRITE_LENGTH);
			*sleep_timer_u8 = SMA13X_GET_BITSLICE
			(data_u8, SMA13X_SLEEP_TIMER);
		}
	return com_rslt;
}
/*!
 * @brief This API is used to set the sleep timer mode
 *	in the register 0x12 bit 5
 *
 *
 *
 *
 *  @param  sleep_timer_u8 : The value of sleep timer mode
 *        sleep_timer_u8 |   result
 *       ----------------- | ----------------------
 *              0          | enable EventDrivenSampling(EDT)
 *              1          | enable Equidistant sampling mode(EST)
 *
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
 *
 */
SMA13X_RETURN_FUNCTION_TYPE sma13x_set_sleep_timer_mode(u8 sleep_timer_u8)
{
	u8 data_u8 = SMA13X_INIT_VALUE;
	/*  Variable used to return value of
	communication routine*/
	SMA13X_RETURN_FUNCTION_TYPE com_rslt = ERROR;

	if (p_sma13x == SMA13X_NULL) {
		/* Check the struct p_sma13x is empty */
		return E_SMA13X_NULL_PTR;
		} else {
		if (sleep_timer_u8 < SMA13X_SLEEP_TIMER_MODE_RANGE) {
			/* write the SLEEP TIMER MODE*/
			com_rslt = p_sma13x->SMA13X_BUS_READ_FUNC
			(p_sma13x->dev_addr, SMA13X_SLEEP_TIMER_REG,
			&data_u8, SMA13X_GEN_READ_WRITE_LENGTH);
			data_u8 = SMA13X_SET_BITSLICE
			(data_u8, SMA13X_SLEEP_TIMER, sleep_timer_u8);
			com_rslt += sma13x_write_reg(SMA13X_SLEEP_TIMER_REG,
			&data_u8, SMA13X_GEN_READ_WRITE_LENGTH);
		} else {
		com_rslt = E_OUT_OF_RANGE;
		}
	}
	return com_rslt;
}
/*!
 * @brief This API is used to get high bandwidth
 *		in the register 0x13 bit 7
 *
 *  @param  high_bw_u8 : The value of high bandwidth
 *         high_bw_u8    |   result
 *       ----------------- | ----------------------
 *              0          | Unfiltered High Bandwidth
 *              1          | Filtered Low Bandwidth
 *
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
 *
 */
SMA13X_RETURN_FUNCTION_TYPE sma13x_get_high_bw(u8 *high_bw_u8)
{
	/*  Variable used to return value of
	communication routine*/
	SMA13X_RETURN_FUNCTION_TYPE com_rslt = ERROR;
	u8 data_u8 = SMA13X_INIT_VALUE;

	if (p_sma13x == SMA13X_NULL) {
		return  E_SMA13X_NULL_PTR;
		} else {
			/* Read the high bandwidth*/
			com_rslt = p_sma13x->SMA13X_BUS_READ_FUNC
			(p_sma13x->dev_addr, SMA13X_ENABLE_DATA_HIGH_BW_REG,
			&data_u8, SMA13X_GEN_READ_WRITE_LENGTH);
			*high_bw_u8 = SMA13X_GET_BITSLICE
			(data_u8, SMA13X_ENABLE_DATA_HIGH_BW);
		}
	return com_rslt;
}
/*!
 * @brief This API is used to write high bandwidth
 *		in the register 0x13 bit 7
 *
 *  @param  high_bw_u8 : The value of high bandwidth
 *         high_bw_u8    |   result
 *       ----------------- | ----------------------
 *              0          | Unfiltered High Bandwidth
 *              1          | Filtered Low Bandwidth
 *
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
 *
 */
SMA13X_RETURN_FUNCTION_TYPE sma13x_set_high_bw(u8 high_bw_u8)
{
	/*  Variable used to return value of
	communication routine*/
	SMA13X_RETURN_FUNCTION_TYPE com_rslt = ERROR;
	u8 data_u8 = SMA13X_INIT_VALUE;

	if (p_sma13x == SMA13X_NULL) {
		return  E_SMA13X_NULL_PTR;
		}  else {
			/* Write the high bandwidth*/
			com_rslt = p_sma13x->SMA13X_BUS_READ_FUNC
			(p_sma13x->dev_addr, SMA13X_ENABLE_DATA_HIGH_BW_REG,
			&data_u8, SMA13X_GEN_READ_WRITE_LENGTH);
			data_u8 = SMA13X_SET_BITSLICE(data_u8,
			SMA13X_ENABLE_DATA_HIGH_BW, high_bw_u8);
			com_rslt += sma13x_write_reg(
			SMA13X_ENABLE_DATA_HIGH_BW_REG,
			&data_u8, SMA13X_GEN_READ_WRITE_LENGTH);
		}
	return com_rslt;
}
/*!
 *	@brief This API is used to get shadow dis
 *	in the register 0x13 bit 6
 *
 *  @param  shadow_dis_u8 : The value of shadow dis
 *        shadow_dis_u8  |   result
 *       ----------------- | ------------------
 *              0          | MSB is Locked
 *              1          | No MSB Locking
 *
 *
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
 *
 */
SMA13X_RETURN_FUNCTION_TYPE sma13x_get_shadow_dis(u8 *shadow_dis_u8)
{
	/*  Variable used to return value of
	communication routine*/
	SMA13X_RETURN_FUNCTION_TYPE com_rslt = ERROR;
	u8 data_u8 = SMA13X_INIT_VALUE;

	if (p_sma13x == SMA13X_NULL) {
		return  E_SMA13X_NULL_PTR;
		} else {
			/*Read the shadow dis*/
			com_rslt = p_sma13x->SMA13X_BUS_READ_FUNC
			(p_sma13x->dev_addr,
			SMA13X_DIS_SHADOW_PROC_REG,
			&data_u8, SMA13X_GEN_READ_WRITE_LENGTH);
			*shadow_dis_u8 = SMA13X_GET_BITSLICE
			(data_u8, SMA13X_DIS_SHADOW_PROC);
		}
	return com_rslt;
}
/*!
 *	@brief This API is used to set shadow dis
 *	in the register 0x13 bit 6
 *
 *  @param  shadow_dis_u8 : The value of shadow dis
 *        shadow_dis_u8  |   result
 *       ----------------- | ------------------
 *              0          | MSB is Locked
 *              1          | No MSB Locking
 *
 *
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
 *
 */
SMA13X_RETURN_FUNCTION_TYPE sma13x_set_shadow_dis(u8 shadow_dis_u8)
{
	/*  Variable used to return value of
	communication routine*/
	SMA13X_RETURN_FUNCTION_TYPE com_rslt = ERROR;
	u8 data_u8 = SMA13X_INIT_VALUE;

	if (p_sma13x == SMA13X_NULL) {
		return  E_SMA13X_NULL_PTR;
		} else {
			/* Write the shadow dis*/
			com_rslt = p_sma13x->SMA13X_BUS_READ_FUNC
			(p_sma13x->dev_addr, SMA13X_DIS_SHADOW_PROC_REG,
			&data_u8, SMA13X_GEN_READ_WRITE_LENGTH);
			data_u8 = SMA13X_SET_BITSLICE
			(data_u8, SMA13X_DIS_SHADOW_PROC, shadow_dis_u8);
			com_rslt += sma13x_write_reg(SMA13X_DIS_SHADOW_PROC_REG,
			&data_u8, SMA13X_GEN_READ_WRITE_LENGTH);
		}
	return com_rslt;
}
/*!
 *	@brief This function is used for the soft reset
 *	The soft reset register will be written
 *	with 0xB6 in the register 0x14.
 *
 *
 *
 *  \param : None
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
 *
 */
SMA13X_RETURN_FUNCTION_TYPE sma13x_soft_rst(void)
{
	/*  Variable used to return value of
	communication routine*/
	SMA13X_RETURN_FUNCTION_TYPE com_rslt = ERROR;
	u8 data_u8 = SMA13X_ENABLE_SOFT_RESET_VALUE;

	if (p_sma13x == SMA13X_NULL) {
		/* Check the struct p_sma13x is empty */
		return E_SMA13X_NULL_PTR;
		}  else {
			/*! To reset the sensor
			0xB6 value_u8 will be written */
			com_rslt = sma13x_write_reg(SMA13X_RST_ADDR,
			&data_u8, SMA13X_GEN_READ_WRITE_LENGTH);
		}
	return com_rslt;
}
/*!
 *	@brief This API is used to get
 *  interrupt enable bits of the sensor in the registers 0x16 and 0x17
 *	@note slope-x enable, slope-y enable, slope-z enable,
 *	@note high-x enable, high-y enable, high-z enable
 *
 *
 *
 *  @param intr_type_u8: The value of interrupts
 *        intr_type_u8   |   result
 *       ----------------- | ------------------
 *              1          | SMA13X_HIGH_G_X_INTR
 *              2          | SMA13X_HIGH_G_Y_INTR
 *              3          | SMA13X_HIGH_G_Z_INTR
 *              4          | SMA13X_DATA_ENABLE
 *              5          | SLOPE_X_INTR
 *              6          | SLOPE_Y_INTR
 *              7          | SLOPE_Z_INTR
 *
 *  @param value_u8 : The value of interrupts enable
 *        value_u8       |   result
 *       ----------------- | ------------------
 *              0x00       | INTR_DISABLE
 *              0x01       | INTR_ENABLE
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
SMA13X_RETURN_FUNCTION_TYPE sma13x_get_intr_enable(u8 intr_type_u8,
u8 *value_u8)
{
	u8 data_u8 = SMA13X_INIT_VALUE;
		/*  Variable used to return value of
	communication routine*/
	SMA13X_RETURN_FUNCTION_TYPE com_rslt = ERROR;

	if (p_sma13x == SMA13X_NULL) {
		/* Check the struct p_sma13x is empty */
		return E_SMA13X_NULL_PTR;
		} else {
		switch (intr_type_u8) {
		case SMA13X_HIGH_G_X_INTR:
			com_rslt = p_sma13x->SMA13X_BUS_READ_FUNC
			(p_sma13x->dev_addr,
			SMA13X_ENABLE_HIGH_G_X_INTR_REG,
			&data_u8, SMA13X_GEN_READ_WRITE_LENGTH);
			*value_u8 = SMA13X_GET_BITSLICE
			(data_u8, SMA13X_ENABLE_HIGH_G_X_INTR);
		break;
		case SMA13X_HIGH_G_Y_INTR:
			com_rslt = p_sma13x->SMA13X_BUS_READ_FUNC
			(p_sma13x->dev_addr,
			SMA13X_ENABLE_HIGH_G_Y_INTR_REG,
			&data_u8, SMA13X_GEN_READ_WRITE_LENGTH);
			*value_u8 = SMA13X_GET_BITSLICE
			(data_u8, SMA13X_ENABLE_HIGH_G_Y_INTR);
		break;
		case SMA13X_HIGH_G_Z_INTR:
			com_rslt = p_sma13x->SMA13X_BUS_READ_FUNC
			(p_sma13x->dev_addr,
			SMA13X_ENABLE_HIGH_G_Z_INTR_REG,
			&data_u8, SMA13X_GEN_READ_WRITE_LENGTH);
			*value_u8 = SMA13X_GET_BITSLICE
			(data_u8, SMA13X_ENABLE_HIGH_G_Z_INTR);
		break;
		case SMA13X_DATA_ENABLE:
			com_rslt = p_sma13x->SMA13X_BUS_READ_FUNC
			(p_sma13x->dev_addr,
			SMA13X_ENABLE_NEW_DATA_INTR_REG,
			&data_u8, SMA13X_GEN_READ_WRITE_LENGTH);
			*value_u8 = SMA13X_GET_BITSLICE
			(data_u8, SMA13X_ENABLE_NEW_DATA_INTR);
		break;
		case SMA13X_SLOPE_X_INTR:
			com_rslt = p_sma13x->SMA13X_BUS_READ_FUNC
			(p_sma13x->dev_addr,
			SMA13X_ENABLE_SLOPE_X_INTR_REG,
			&data_u8, SMA13X_GEN_READ_WRITE_LENGTH);
			*value_u8 = SMA13X_GET_BITSLICE
			(data_u8, SMA13X_ENABLE_SLOPE_X_INTR);
		break;
		case SMA13X_SLOPE_Y_INTR:
			com_rslt = p_sma13x->SMA13X_BUS_READ_FUNC
			(p_sma13x->dev_addr,
			SMA13X_ENABLE_SLOPE_Y_INTR_REG,
			&data_u8, SMA13X_GEN_READ_WRITE_LENGTH);
			*value_u8 = SMA13X_GET_BITSLICE
			(data_u8, SMA13X_ENABLE_SLOPE_Y_INTR);
		break;
		case SMA13X_SLOPE_Z_INTR:
			com_rslt = p_sma13x->SMA13X_BUS_READ_FUNC
			(p_sma13x->dev_addr,
			SMA13X_ENABLE_SLOPE_Z_INTR_REG,
			&data_u8, SMA13X_GEN_READ_WRITE_LENGTH);
			*value_u8 = SMA13X_GET_BITSLICE
			(data_u8, SMA13X_ENABLE_SLOPE_Z_INTR);
		break;
		default:
		com_rslt = E_OUT_OF_RANGE;
		break;
		}
	}
	return com_rslt;
}
/*!
 *	@brief This API is used to set
 *  interrupt enable bits of the sensor in the registers 0x16 and 0x17
 *	@note slope-x enable, slope-y enable, slope-z enable,
 *	@note high-x enable, high-y enable, high-z enable
 *
 *
 *
 *  @param intr_type_u8: The value of interrupts
 *        intr_type_u8   |   result
 *       ----------------- | ------------------
 *              1          | SMA13X_HIGH_G_X_INTR
 *              2          | SMA13X_HIGH_G_Y_INTR
 *              3          | SMA13X_HIGH_G_Z_INTR
 *              4          | SMA13X_DATA_ENABLE
 *              5          | SLOPE_X_INTR
 *              6          | SLOPE_Y_INTR
 *              7          | SLOPE_Z_INTR
 *
 *  @param value_u8 : The value of interrupts enable
 *        value_u8       |   result
 *       ----------------- | ------------------
 *              0x00       | INTR_DISABLE
 *              0x01       | INTR_ENABLE
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
SMA13X_RETURN_FUNCTION_TYPE sma13x_set_intr_enable(u8 intr_type_u8,
u8 value_u8)
{
		/*  Variable used to return value of
	communication routine*/
	SMA13X_RETURN_FUNCTION_TYPE com_rslt = ERROR;
	u8 data_u8 = SMA13X_INIT_VALUE;
	u8 data2_u8 = SMA13X_INIT_VALUE;

	if (p_sma13x == SMA13X_NULL) {
		/* Check the struct p_sma13x is empty */
		return E_SMA13X_NULL_PTR;
		} else {
		com_rslt = p_sma13x->SMA13X_BUS_READ_FUNC
		(p_sma13x->dev_addr, SMA13X_INTR_ENABLE1_ADDR,
		&data_u8, SMA13X_GEN_READ_WRITE_LENGTH);
		com_rslt += p_sma13x->SMA13X_BUS_READ_FUNC
		(p_sma13x->dev_addr, SMA13X_INTR_ENABLE2_ADDR,
		&data2_u8, SMA13X_GEN_READ_WRITE_LENGTH);
		value_u8 = value_u8 & SMA13X_GEN_READ_WRITE_LENGTH;
		switch (intr_type_u8) {
		case SMA13X_HIGH_G_X_INTR:
			/* High G X Interrupt */
			data2_u8 = SMA13X_SET_BITSLICE(data2_u8,
			SMA13X_ENABLE_HIGH_G_X_INTR, value_u8);
		break;
		case SMA13X_HIGH_G_Y_INTR:
			/* High G Y Interrupt */
			data2_u8 = SMA13X_SET_BITSLICE(data2_u8,
			SMA13X_ENABLE_HIGH_G_Y_INTR, value_u8);
		break;
		case SMA13X_HIGH_G_Z_INTR:
			/* High G Z Interrupt */
			data2_u8 = SMA13X_SET_BITSLICE(data2_u8,
			SMA13X_ENABLE_HIGH_G_Z_INTR, value_u8);
		break;
		case SMA13X_DATA_ENABLE:
			/*Data En Interrupt  */
			data2_u8 = SMA13X_SET_BITSLICE(data2_u8,
			SMA13X_ENABLE_NEW_DATA_INTR, value_u8);
		break;
		case SMA13X_SLOPE_X_INTR:
			/* Slope X Interrupt */
			data_u8 = SMA13X_SET_BITSLICE(data_u8,
			SMA13X_ENABLE_SLOPE_X_INTR, value_u8);
		break;
		case SMA13X_SLOPE_Y_INTR:
			/* Slope Y Interrupt */
			data_u8 = SMA13X_SET_BITSLICE(data_u8,
			SMA13X_ENABLE_SLOPE_Y_INTR, value_u8);
		break;
		case SMA13X_SLOPE_Z_INTR:
			/* Slope Z Interrupt */
			data_u8 = SMA13X_SET_BITSLICE(data_u8,
			SMA13X_ENABLE_SLOPE_Z_INTR, value_u8);
		break;
		default:
			com_rslt = E_OUT_OF_RANGE;
		break;
		}
		/* write the interrupt*/
		com_rslt += sma13x_write_reg
		(SMA13X_INTR_ENABLE1_ADDR,
		&data_u8, SMA13X_GEN_READ_WRITE_LENGTH);
		com_rslt += sma13x_write_reg
		(SMA13X_INTR_ENABLE2_ADDR,
		&data2_u8, SMA13X_GEN_READ_WRITE_LENGTH);
	}
	return com_rslt;
}
/*!
 * @brief This API is used to get
 * the interrupt status of slow/no motion select and slow no motion
 * enable xyz interrupt in the register 0x18 bit from 0 to 3
 *
 *
 *  @param  channel_u8 : The value of slow/no motion select
 *        channel_u8     |   result
 *       ----------------- | ------------------
 *              0          | SMA13X_ACCEL_SLOW_NO_MOTION_ENABLE_X
 *              1          | SMA13X_ACCEL_SLOW_NO_MOTION_ENABLE_Y
 *              2          | SMA13X_ACCEL_SLOW_NO_MOTION_ENABLE_Z
 *              3          | SMA13X_ACCEL_SLOW_NO_MOTION_ENABLE_SEL
 *
 *	@param slow_no_motion_u8 : The value of slow no motion interrupt enable
 *        slow_no_motion_u8     |   result
 *       ------------------------ | ------------------
 *              0x00              | INTR_DISABLE
 *              0x01              | INTR_ENABLE
 *
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
 *
 */
SMA13X_RETURN_FUNCTION_TYPE sma13x_get_slow_no_motion(u8 channel_u8,
u8 *slow_no_motion_u8)
{
	u8 data_u8 = SMA13X_INIT_VALUE;
		/*  Variable used to return value of
	communication routine*/
	SMA13X_RETURN_FUNCTION_TYPE com_rslt = ERROR;

	if (p_sma13x == SMA13X_NULL) {
		/* Check the struct p_sma13x is empty */
		return E_SMA13X_NULL_PTR;
		} else {
		/* Read the slow no motion interrupt */
		switch (channel_u8) {
		case SMA13X_SLOW_NO_MOTION_ENABLE_X:
			com_rslt = p_sma13x->SMA13X_BUS_READ_FUNC
			(p_sma13x->dev_addr,
			SMA13X_INTR_SLOW_NO_MOTION_ENABLE_X_INTR_REG,
			&data_u8, SMA13X_GEN_READ_WRITE_LENGTH);
			*slow_no_motion_u8 = SMA13X_GET_BITSLICE
			(data_u8, SMA13X_INTR_SLOW_NO_MOTION_ENABLE_X_INTR);
		break;
		case SMA13X_SLOW_NO_MOTION_ENABLE_Y:
			com_rslt = p_sma13x->SMA13X_BUS_READ_FUNC
			(p_sma13x->dev_addr,
			SMA13X_INTR_SLOW_NO_MOTION_ENABLE_Y_INTR_REG,
			&data_u8, SMA13X_GEN_READ_WRITE_LENGTH);
			*slow_no_motion_u8 = SMA13X_GET_BITSLICE
			(data_u8, SMA13X_INTR_SLOW_NO_MOTION_ENABLE_Y_INTR);
		break;
		case SMA13X_SLOW_NO_MOTION_ENABLE_Z:
			com_rslt = p_sma13x->SMA13X_BUS_READ_FUNC
			(p_sma13x->dev_addr,
			SMA13X_INTR_SLOW_NO_MOTION_ENABLE_Z_INTR_REG,
			&data_u8, SMA13X_GEN_READ_WRITE_LENGTH);
			*slow_no_motion_u8 = SMA13X_GET_BITSLICE
			(data_u8, SMA13X_INTR_SLOW_NO_MOTION_ENABLE_Z_INTR);
		break;
		case SMA13X_SLOW_NO_MOTION_ENABLE_SELECT:
			com_rslt = p_sma13x->SMA13X_BUS_READ_FUNC
			(p_sma13x->dev_addr,
			SMA13X_INTR_SLOW_NO_MOTION_ENABLE_SELECT_INTR_REG,
			&data_u8, SMA13X_GEN_READ_WRITE_LENGTH);
			*slow_no_motion_u8 = SMA13X_GET_BITSLICE
			(data_u8,
			SMA13X_INTR_SLOW_NO_MOTION_ENABLE_SELECT_INTR);
		break;
		default:
			com_rslt = E_OUT_OF_RANGE;
		break;
		}
	}
	return com_rslt;
}
/*!
 * @brief This API is used to set
 * the interrupt status of slow/no motion select and slow no motion
 * enable xyz interrupt in the register 0x18 bit from 0 to 3
 *
 *
 *  @param  channel_u8 : The value of slow/no motion select
 *        channel_u8     |   result
 *       ----------------- | ------------------
 *              0          | SMA13X_ACCEL_SLOW_NO_MOTION_ENABLE_X
 *              1          | SMA13X_ACCEL_SLOW_NO_MOTION_ENABLE_Y
 *              2          | SMA13X_ACCEL_SLOW_NO_MOTION_ENABLE_Z
 *              3          | SMA13X_ACCEL_SLOW_NO_MOTION_ENABLE_SEL
 *
 *	@param slow_no_motion_u8 : The value of slow no motion
 *      interrupt enable
 *        slow_no_motion_u8     |   result
 *       ------------------------ | ------------------
 *              0x00              | INTR_DISABLE
 *              0x01              | INTR_ENABLE
 *
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
 *
 */
SMA13X_RETURN_FUNCTION_TYPE sma13x_set_slow_no_motion(u8 channel_u8,
u8 slow_no_motion_u8)
{
	u8 data_u8 = SMA13X_INIT_VALUE;
		/*  Variable used to return value of
	communication routine*/
	SMA13X_RETURN_FUNCTION_TYPE com_rslt = ERROR;

	if (p_sma13x == SMA13X_NULL) {
		/* Check the struct p_sma13x is empty */
		return E_SMA13X_NULL_PTR;
		} else {
		/* Write the slow no motion interrupt*/
		switch (channel_u8) {
		case SMA13X_SLOW_NO_MOTION_ENABLE_X:
			com_rslt = p_sma13x->SMA13X_BUS_READ_FUNC
			(p_sma13x->dev_addr,
			SMA13X_INTR_SLOW_NO_MOTION_ENABLE_X_INTR_REG,
			&data_u8, SMA13X_GEN_READ_WRITE_LENGTH);
			data_u8 = SMA13X_SET_BITSLICE
			(data_u8,
			SMA13X_INTR_SLOW_NO_MOTION_ENABLE_X_INTR,
			slow_no_motion_u8);
			com_rslt += sma13x_write_reg(
			SMA13X_INTR_SLOW_NO_MOTION_ENABLE_X_INTR_REG,
			&data_u8, SMA13X_GEN_READ_WRITE_LENGTH);
		break;
		case SMA13X_SLOW_NO_MOTION_ENABLE_Y:
			com_rslt = p_sma13x->SMA13X_BUS_READ_FUNC
			(p_sma13x->dev_addr,
			SMA13X_INTR_SLOW_NO_MOTION_ENABLE_Y_INTR_REG,
			&data_u8, SMA13X_GEN_READ_WRITE_LENGTH);
			data_u8 = SMA13X_SET_BITSLICE
			(data_u8,
			SMA13X_INTR_SLOW_NO_MOTION_ENABLE_Y_INTR,
			slow_no_motion_u8);
			com_rslt += sma13x_write_reg(
			SMA13X_INTR_SLOW_NO_MOTION_ENABLE_Y_INTR_REG,
			&data_u8, SMA13X_GEN_READ_WRITE_LENGTH);
		break;
		case SMA13X_SLOW_NO_MOTION_ENABLE_Z:
			com_rslt = p_sma13x->SMA13X_BUS_READ_FUNC
			(p_sma13x->dev_addr,
			SMA13X_INTR_SLOW_NO_MOTION_ENABLE_Z_INTR_REG,
			&data_u8, SMA13X_GEN_READ_WRITE_LENGTH);
			data_u8 = SMA13X_SET_BITSLICE
			(data_u8,
			SMA13X_INTR_SLOW_NO_MOTION_ENABLE_Z_INTR,
			slow_no_motion_u8);
			com_rslt += sma13x_write_reg(
			SMA13X_INTR_SLOW_NO_MOTION_ENABLE_Z_INTR_REG,
			&data_u8, SMA13X_GEN_READ_WRITE_LENGTH);
		break;
		case SMA13X_SLOW_NO_MOTION_ENABLE_SELECT:
			com_rslt = p_sma13x->SMA13X_BUS_READ_FUNC
			(p_sma13x->dev_addr,
			SMA13X_INTR_SLOW_NO_MOTION_ENABLE_SELECT_INTR_REG,
			&data_u8, SMA13X_GEN_READ_WRITE_LENGTH);
			data_u8 = SMA13X_SET_BITSLICE
			(data_u8,
			SMA13X_INTR_SLOW_NO_MOTION_ENABLE_SELECT_INTR,
			slow_no_motion_u8);
			com_rslt += sma13x_write_reg(
			SMA13X_INTR_SLOW_NO_MOTION_ENABLE_SELECT_INTR_REG,
			&data_u8, SMA13X_GEN_READ_WRITE_LENGTH);
		break;
		default:
			com_rslt = E_OUT_OF_RANGE;
		break;
		}
	}
	return com_rslt;
}
/*!
 * @brief This API is used to get
 * the interrupt enable of high_g interrupt in the register 0x19
 * @note INTR1_high_g -> register 0x19 bit 1
 *
 *  @param  channel_u8: The value of high_g interrupt selection
 *        channel_u8     |   result
 *       ----------------- | ------------------
 *              0          | SMA13X_ACCEL_INTR1_HIGH_G
 *
 * @param intr_high_g_u8 : the value of high_g interrupt
 *        intr_high_g_u8        |   result
 *       ------------------------ | ------------------
 *              0x00              | INTR_DISABLE
 *              0x01              | INTR_ENABLE
 *
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
 *
 */
SMA13X_RETURN_FUNCTION_TYPE sma13x_get_intr_high_g(u8 channel_u8,
u8 *intr_high_g_u8)
{
	u8 data_u8 = SMA13X_INIT_VALUE;
	/*  Variable used to return value of
	communication routine*/
	SMA13X_RETURN_FUNCTION_TYPE com_rslt = ERROR;

	if (p_sma13x == SMA13X_NULL) {
		/* Check the struct p_sma13x is empty */
		return E_SMA13X_NULL_PTR;
		} else {
		switch (channel_u8) {
		/* read the high_g interrupt*/
		case SMA13X_INTR1_HIGH_G:
			com_rslt = p_sma13x->SMA13X_BUS_READ_FUNC
			(p_sma13x->dev_addr,
			SMA13X_ENABLE_INTR1_PAD_HIGH_G_REG,
			&data_u8, SMA13X_GEN_READ_WRITE_LENGTH);
			*intr_high_g_u8 = SMA13X_GET_BITSLICE
			(data_u8, SMA13X_ENABLE_INTR1_PAD_HIGH_G);
		break;
		default:
		com_rslt = E_OUT_OF_RANGE;
		break;
		}
	}
	return com_rslt;
}
/*!
 * @brief This API is used to set
 * the interrupt enable of high_g interrupt in the register 0x19
 * @note INTR1_high_g -> register 0x19 bit 1
 *
 *
 *  @param  channel_u8: The value of high_g interrupt selection
 *        channel_u8     |   result
 *       ----------------- | ------------------
 *              0          | SMA13X_ACCEL_INTR1_HIGH_G
 *
 * @param intr_high_g_u8 : the value of high_g interrupt
 *        intr_high_g_u8        |   result
 *       ------------------------ | ------------------
 *              0x00              | INTR_DISABLE
 *              0x01              | INTR_ENABLE
 *
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
 *
 */
SMA13X_RETURN_FUNCTION_TYPE sma13x_set_intr_high_g(u8 channel_u8,
u8 intr_high_g_u8)
{
	u8 data_u8 = SMA13X_INIT_VALUE;
	/*  Variable used to return value of
	communication routine*/
	SMA13X_RETURN_FUNCTION_TYPE com_rslt = ERROR;

	if (p_sma13x == SMA13X_NULL) {
		/* Check the struct p_sma13x is empty */
		return E_SMA13X_NULL_PTR;
		} else {
		/* write the high_g interrupt*/
		switch (channel_u8) {
		case SMA13X_INTR1_HIGH_G:
			com_rslt = p_sma13x->SMA13X_BUS_READ_FUNC
			(p_sma13x->dev_addr,
			SMA13X_ENABLE_INTR1_PAD_HIGH_G_REG,
			&data_u8, SMA13X_GEN_READ_WRITE_LENGTH);
			data_u8 = SMA13X_SET_BITSLICE
			(data_u8, SMA13X_ENABLE_INTR1_PAD_HIGH_G,
			intr_high_g_u8);
			com_rslt += sma13x_write_reg(
			SMA13X_ENABLE_INTR1_PAD_HIGH_G_REG,
			&data_u8, SMA13X_GEN_READ_WRITE_LENGTH);
		break;
		default:
		com_rslt = E_OUT_OF_RANGE;
		break;
		}
	}
	return com_rslt;
}
/*!
 * @brief This API is used to get
 * the interrupt enable of slope interrupt in the register 0x19
 * @note INTR1_slope -> register 0x19 bit 2
 *
 *
 *
 * @param channel_u8: the value of slope channel select
 *        channel_u8     |   result
 *       ----------------- | ------------------
 *              0          | SMA13X_ACCEL_INTR1_SLOPE
 *
 * @param intr_slope_u8 : The slope value enable value
 *        intr_slope_u8         |   result
 *       ------------------------ | ------------------
 *              0x00              | INTR_DISABLE
 *              0x01              | INTR_ENABLE
 *
 *
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
 *
 */
SMA13X_RETURN_FUNCTION_TYPE sma13x_get_intr_slope(u8 channel_u8,
u8 *intr_slope_u8)
{
	u8 data_u8 = SMA13X_INIT_VALUE;
		/*  Variable used to return value of
	communication routine*/
	SMA13X_RETURN_FUNCTION_TYPE com_rslt = ERROR;

	if (p_sma13x == SMA13X_NULL) {
		/* Check the struct p_sma13x is empty */
		return E_SMA13X_NULL_PTR;
		} else {
		/* Read the slope value */
		switch (channel_u8) {
		case SMA13X_INTR1_SLOPE:
			com_rslt = p_sma13x->SMA13X_BUS_READ_FUNC
			(p_sma13x->dev_addr,
			SMA13X_ENABLE_INTR1_PAD_SLOPE_REG,
			&data_u8, SMA13X_GEN_READ_WRITE_LENGTH);
			*intr_slope_u8 = SMA13X_GET_BITSLICE
			(data_u8, SMA13X_ENABLE_INTR1_PAD_SLOPE);
		break;
		default:
		com_rslt = E_OUT_OF_RANGE;
		break;
		}
	}
	return com_rslt;
}
/*!
 * @brief This API is used to set
 * the interrupt enable of slope interrupt in the register 0x19
 * @note INTR1_slope -> register 0x19 bit 2
 *
 *
 *
 * @param channel_u8: the value of slope channel select
 *        channel_u8     |   result
 *       ----------------- | ------------------
 *              0          | SMA13X_ACCEL_INTR1_SLOPE
 *
 * @param intr_slope_u8 : The slope value enable value
 *        intr_slope_u8         |   result
 *       ------------------------ | ------------------
 *              0x00              | INTR_DISABLE
 *              0x01              | INTR_ENABLE
 *
 *
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
 *
 */
SMA13X_RETURN_FUNCTION_TYPE sma13x_set_intr_slope(u8 channel_u8,
u8 intr_slope_u8)
{
	u8 data_u8 = SMA13X_INIT_VALUE;
		/*  Variable used to return value of
	communication routine*/
	SMA13X_RETURN_FUNCTION_TYPE com_rslt = ERROR;

	if (p_sma13x == SMA13X_NULL) {
		/* Check the struct p_sma13x is empty */
		return E_SMA13X_NULL_PTR;
		} else {
		switch (channel_u8) {
		/* Write the slope value */
		case SMA13X_INTR1_SLOPE:
			com_rslt = p_sma13x->SMA13X_BUS_READ_FUNC
			(p_sma13x->dev_addr,
			SMA13X_ENABLE_INTR1_PAD_SLOPE_REG,
			&data_u8, SMA13X_GEN_READ_WRITE_LENGTH);
			data_u8 = SMA13X_SET_BITSLICE
			(data_u8, SMA13X_ENABLE_INTR1_PAD_SLOPE,
			intr_slope_u8);
			com_rslt += sma13x_write_reg(
			SMA13X_ENABLE_INTR1_PAD_SLOPE_REG,
			&data_u8, SMA13X_GEN_READ_WRITE_LENGTH);
		break;
		default:
			com_rslt = E_OUT_OF_RANGE;
		break;
		}
	}
	return com_rslt;
}
/*!
 * @brief This API is used to get
 * the interrupt enable of slow/no motion interrupt in
 * the register 0x19
 * @note INTR1_slow_no_motion -> register 0x19 bit 3
 *
 *
 *
 *
 *  @param channel_u8 : The value of slow/no motion selection
 *        channel_u8     |   result
 *       ----------------- | ------------------
 *              0          | SMA13X_INTR1_SLOW_NO_MOTION
 *
 *  @param intr_slow_no_motion_u8:  the slow_no_motion enable value
 *       intr_slow_no_motion_u8 |   result
 *       ------------------------ | ------------------
 *              0x00              | INTR_DISABLE
 *              0x01              | INTR_ENABLE
 *
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
 *
 */
SMA13X_RETURN_FUNCTION_TYPE sma13x_get_intr_slow_no_motion(u8 channel_u8,
u8 *intr_slow_no_motion_u8)
{
	u8 data_u8 = SMA13X_INIT_VALUE;
		/*  Variable used to return value of
	communication routine*/
	SMA13X_RETURN_FUNCTION_TYPE com_rslt = ERROR;

	if (p_sma13x == SMA13X_NULL) {
		/* Check the struct p_sma13x is empty */
		return E_SMA13X_NULL_PTR;
		} else {
		/* Read the slow no motion interrupt */
		switch (channel_u8) {
		case SMA13X_INTR1_SLOW_NO_MOTION:
			com_rslt = p_sma13x->SMA13X_BUS_READ_FUNC
			(p_sma13x->dev_addr,
			SMA13X_ENABLE_INTR1_PAD_SLOW_NO_MOTION_REG,
			&data_u8, SMA13X_GEN_READ_WRITE_LENGTH);
			*intr_slow_no_motion_u8 = SMA13X_GET_BITSLICE
			(data_u8, SMA13X_ENABLE_INTR1_PAD_SLOW_NO_MOTION);
		break;
		default:
			com_rslt = E_OUT_OF_RANGE;
		break;
		}
	}
	return com_rslt;
}
/*!
 * @brief This API is used to set
 * the interrupt enable of slow/no motion interrupt in
 * the register 0x19
 * @note INTR1_slow_no_motion -> register 0x19 bit 3
 *
 *
 *
 *
 *  @param channel_u8 : The value of slow/no motion selection
 *        channel_u8     |   result
 *       ----------------- | ------------------
 *              0          | SMA13X_INTR1_SLOW_NO_MOTION
 *
 *  @param intr_slow_no_motion_u8:  the slow_no_motion enable value
 *       intr_slow_no_motion_u8 |   result
 *       ------------------------ | ------------------
 *              0x00              | INTR_DISABLE
 *              0x01              | INTR_ENABLE
 *
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
 *
 */
SMA13X_RETURN_FUNCTION_TYPE sma13x_set_intr_slow_no_motion(u8 channel_u8,
u8 intr_slow_no_motion_u8)
{
	u8 data_u8 = SMA13X_INIT_VALUE;
	/*  Variable used to return value of
	communication routine*/
	SMA13X_RETURN_FUNCTION_TYPE com_rslt = ERROR;

	if (p_sma13x == SMA13X_NULL) {
		/* Check the struct p_sma13x is empty */
		return E_SMA13X_NULL_PTR;
		} else {
		switch (channel_u8) {
		/* Write the slow no motion interrupt */
		case SMA13X_INTR1_SLOW_NO_MOTION:
			com_rslt = p_sma13x->SMA13X_BUS_READ_FUNC
			(p_sma13x->dev_addr,
			SMA13X_ENABLE_INTR1_PAD_SLOW_NO_MOTION_REG,
			&data_u8, SMA13X_GEN_READ_WRITE_LENGTH);
			data_u8 = SMA13X_SET_BITSLICE
			(data_u8,
			SMA13X_ENABLE_INTR1_PAD_SLOW_NO_MOTION,
			intr_slow_no_motion_u8);
			com_rslt += sma13x_write_reg(
			SMA13X_ENABLE_INTR1_PAD_SLOW_NO_MOTION_REG,
			&data_u8, SMA13X_GEN_READ_WRITE_LENGTH);
		break;
		default:
			com_rslt = E_OUT_OF_RANGE;
		break;
		}
	}
	return com_rslt;
}
/*!
 * @brief This API is used to get
 * the interrupt status of new data in the register 0x19
 * @note INTR1_data -> register 0x19 bit 0
 *
 *
 *
 *  @param channel_u8: The value of new data interrupt select
 *        channel_u8     |   result
 *       ----------------- | ------------------
 *              0          | SMA13X_ACCEL_INTR1_NEWDATA
 *
 *	@param intr_newdata_u8: The new data interrupt enable value
 *       intr_newdata_u8          |    result
 *       ------------------------ | ------------------
 *              0x00              | INTR_DISABLE
 *              0x01              | INTR_ENABLE
 *
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
 *
 */
SMA13X_RETURN_FUNCTION_TYPE sma13x_get_new_data(u8 channel_u8,
u8 *intr_newdata_u8)
{
	u8 data_u8 = SMA13X_INIT_VALUE;
		/*  Variable used to return value of
	communication routine*/
	SMA13X_RETURN_FUNCTION_TYPE com_rslt = ERROR;

	if (p_sma13x == SMA13X_NULL) {
		/* Check the struct p_sma13x is empty */
		return E_SMA13X_NULL_PTR;
		} else {
		switch (channel_u8) {
		/* Read the data interrupt*/
		case SMA13X_INTR1_NEWDATA:
			com_rslt = p_sma13x->SMA13X_BUS_READ_FUNC
			(p_sma13x->dev_addr,
			SMA13X_ENABLE_INTR1_PAD_NEWDATA_REG,
			&data_u8, SMA13X_GEN_READ_WRITE_LENGTH);
			*intr_newdata_u8 = SMA13X_GET_BITSLICE
			(data_u8, SMA13X_ENABLE_INTR1_PAD_NEWDATA);
		break;
		default:
			com_rslt = E_OUT_OF_RANGE;
		break;
		}
	}
	return com_rslt;
}
/*!
 * @brief This API is used to set
 * the interrupt status of new data in the register 0x19
 * @note INTR1_data -> register 0x19 bit 0
 *
 *
 *
 *  @param channel_u8: The value of new data interrupt select
 *        channel_u8     |   result
 *       ----------------- | ------------------
 *              0          | SMA13X_ACCEL_INTR1_NEWDATA
 *
 *	@param intr_newdata_u8: The new data interrupt enable value
 *       intr_newdata_u8          |    result
 *       ------------------------ | ------------------
 *              0x00              | INTR_DISABLE
 *              0x01              | INTR_ENABLE
 *
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
 *
 */
SMA13X_RETURN_FUNCTION_TYPE sma13x_set_new_data(u8 channel_u8,
u8 intr_newdata_u8)
{
	u8 data_u8 = SMA13X_INIT_VALUE;
	/*  Variable used to return value of
	communication routine*/
	SMA13X_RETURN_FUNCTION_TYPE com_rslt = ERROR;

	if (p_sma13x == SMA13X_NULL) {
		/* Check the struct p_sma13x is empty */
		return E_SMA13X_NULL_PTR;
		} else {
		switch (channel_u8) {
		/* write the new data interrupt */
		case SMA13X_INTR1_NEWDATA:
			com_rslt = p_sma13x->SMA13X_BUS_READ_FUNC
			(p_sma13x->dev_addr,
			SMA13X_ENABLE_INTR1_PAD_NEWDATA_REG,
			&data_u8, SMA13X_GEN_READ_WRITE_LENGTH);
			data_u8 = SMA13X_SET_BITSLICE
			(data_u8,
			SMA13X_ENABLE_INTR1_PAD_NEWDATA, intr_newdata_u8);
			com_rslt += sma13x_write_reg(
			SMA13X_ENABLE_INTR1_PAD_NEWDATA_REG,
			&data_u8, SMA13X_GEN_READ_WRITE_LENGTH);
		break;
		default:
			com_rslt = E_OUT_OF_RANGE;
		break;
		}
	}
	return com_rslt;
}
/*!
 *	@brief This API is used to get
 *	the source data status of source data,
 *	source slow no motion, source slope, source high
 *	and source low in the register 0x1E bit from 0 to 5
 *
 *
 *
 *  @param channel_u8 : The value of source select
 *       channel_u8     |    result
 *       -----------------| ------------------
 *               1        | SMA13X_ACCEL_SOURCE_HIGH_G
 *               2        | SMA13X_ACCEL_SOURCE_SLOPE
 *               3        | SMA13X_ACCEL_SOURCE_SLOW_NO_MOTION
 *               5        | SMA13X_ACCEL_SOURCE_DATA
 *
 *	@param intr_source_u8: The source status enable value
 *       intr_source_u8         |    result
 *       ------------------------ | ------------------
 *              0x00              | INTR_DISABLE
 *              0x01              | INTR_ENABLE
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
 */
SMA13X_RETURN_FUNCTION_TYPE sma13x_get_source(u8 channel_u8,
u8 *intr_source_u8)
{
	u8 data_u8 = SMA13X_INIT_VALUE;
	/*  Variable used to return value of
	communication routine*/
	SMA13X_RETURN_FUNCTION_TYPE com_rslt = ERROR;

	if (p_sma13x == SMA13X_NULL) {
		return  E_SMA13X_NULL_PTR;
		} else {
		/* read the source interrupt register */
		switch (channel_u8) {
		case SMA13X_SOURCE_HIGH_G:
			com_rslt = p_sma13x->SMA13X_BUS_READ_FUNC
			(p_sma13x->dev_addr,
			SMA13X_UNFILT_INTR_SOURCE_HIGH_G_REG,
			&data_u8, SMA13X_GEN_READ_WRITE_LENGTH);
			*intr_source_u8 = SMA13X_GET_BITSLICE
			(data_u8, SMA13X_UNFILT_INTR_SOURCE_HIGH_G);
		break;
		case SMA13X_SOURCE_SLOPE:
			com_rslt = p_sma13x->SMA13X_BUS_READ_FUNC
			(p_sma13x->dev_addr,
			SMA13X_UNFILT_INTR_SOURCE_SLOPE_REG,
			&data_u8, SMA13X_GEN_READ_WRITE_LENGTH);
			*intr_source_u8 = SMA13X_GET_BITSLICE
			(data_u8, SMA13X_UNFILT_INTR_SOURCE_SLOPE);
		break;
		case SMA13X_SOURCE_SLOW_NO_MOTION:
			com_rslt = p_sma13x->SMA13X_BUS_READ_FUNC
			(p_sma13x->dev_addr,
			SMA13X_UNFILT_INTR_SOURCE_SLOW_NO_MOTION_REG,
			&data_u8, SMA13X_GEN_READ_WRITE_LENGTH);
			*intr_source_u8 = SMA13X_GET_BITSLICE
			(data_u8, SMA13X_UNFILT_INTR_SOURCE_SLOW_NO_MOTION);
		break;
		case SMA13X_SOURCE_DATA:
			com_rslt = p_sma13x->SMA13X_BUS_READ_FUNC
			(p_sma13x->dev_addr,
			SMA13X_UNFILT_INTR_SOURCE_DATA_REG,
			&data_u8, SMA13X_GEN_READ_WRITE_LENGTH);
			*intr_source_u8 = SMA13X_GET_BITSLICE
			(data_u8, SMA13X_UNFILT_INTR_SOURCE_DATA);
		break;
		default:
			com_rslt = E_OUT_OF_RANGE;
		break;
			}
		}
	return com_rslt;
}
/*!
 *	@brief This API is used to set
 *	the source data status of source data,
 *	source slow no motion, source slope, source high
 *	and source low in the register 0x1E bit from 0 to 5
 *
 *
 *
 *  @param channel_u8 : The value of source select
 *       channel_u8     |    result
 *       -----------------| ------------------
 *               1        | SMA13X_ACCEL_SOURCE_HIGH_G
 *               2        | SMA13X_ACCEL_SOURCE_SLOPE
 *               3        | SMA13X_ACCEL_SOURCE_SLOW_NO_MOTION
 *               5        | SMA13X_ACCEL_SOURCE_DATA
 *
 *	@param intr_source_u8: The source status enable value
 *       intr_source_u8         |    result
 *       ------------------------ | ------------------
 *              0x00              | INTR_DISABLE
 *              0x01              | INTR_ENABLE
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
 */
SMA13X_RETURN_FUNCTION_TYPE sma13x_set_source(u8 channel_u8,
u8 intr_source_u8)
{
	u8 data_u8 = SMA13X_INIT_VALUE;
	/*  Variable used to return value of
	communication routine*/
	SMA13X_RETURN_FUNCTION_TYPE com_rslt = ERROR;
		if (p_sma13x == SMA13X_NULL) {
			com_rslt = E_SMA13X_NULL_PTR;
		} else {
		switch (channel_u8) {
		/* write the source interrupt register*/
		case SMA13X_SOURCE_HIGH_G:
			com_rslt = p_sma13x->SMA13X_BUS_READ_FUNC
			(p_sma13x->dev_addr,
			SMA13X_UNFILT_INTR_SOURCE_HIGH_G_REG,
			&data_u8, SMA13X_GEN_READ_WRITE_LENGTH);
			data_u8 = SMA13X_SET_BITSLICE
			(data_u8,
			SMA13X_UNFILT_INTR_SOURCE_HIGH_G, intr_source_u8);
			com_rslt += sma13x_write_reg(
			SMA13X_UNFILT_INTR_SOURCE_HIGH_G_REG,
			&data_u8, SMA13X_GEN_READ_WRITE_LENGTH);
		break;
		case SMA13X_SOURCE_SLOPE:
			com_rslt = p_sma13x->SMA13X_BUS_READ_FUNC
			(p_sma13x->dev_addr,
			SMA13X_UNFILT_INTR_SOURCE_SLOPE_REG,
			&data_u8, SMA13X_GEN_READ_WRITE_LENGTH);
			data_u8 = SMA13X_SET_BITSLICE
			(data_u8,
			SMA13X_UNFILT_INTR_SOURCE_SLOPE, intr_source_u8);
			com_rslt += sma13x_write_reg(
			SMA13X_UNFILT_INTR_SOURCE_SLOPE_REG,
			&data_u8, SMA13X_GEN_READ_WRITE_LENGTH);
		break;
		case SMA13X_SOURCE_SLOW_NO_MOTION:
			com_rslt = p_sma13x->SMA13X_BUS_READ_FUNC
			(p_sma13x->dev_addr,
			SMA13X_UNFILT_INTR_SOURCE_SLOW_NO_MOTION_REG,
			&data_u8, SMA13X_GEN_READ_WRITE_LENGTH);
			data_u8 = SMA13X_SET_BITSLICE
			(data_u8,
			SMA13X_UNFILT_INTR_SOURCE_SLOW_NO_MOTION,
			intr_source_u8);
			com_rslt += sma13x_write_reg(
			SMA13X_UNFILT_INTR_SOURCE_SLOW_NO_MOTION_REG,
			&data_u8, SMA13X_GEN_READ_WRITE_LENGTH);
		break;
		case SMA13X_SOURCE_DATA:
			com_rslt = p_sma13x->SMA13X_BUS_READ_FUNC
			(p_sma13x->dev_addr,
			SMA13X_UNFILT_INTR_SOURCE_DATA_REG,
			&data_u8, SMA13X_GEN_READ_WRITE_LENGTH);
			data_u8 = SMA13X_SET_BITSLICE
			(data_u8, SMA13X_UNFILT_INTR_SOURCE_DATA,
			intr_source_u8);
			com_rslt += sma13x_write_reg(
			SMA13X_UNFILT_INTR_SOURCE_DATA_REG,
			&data_u8, SMA13X_GEN_READ_WRITE_LENGTH);
		break;
		default:
			com_rslt = E_OUT_OF_RANGE;
		break;
		}
	}
	return com_rslt;
}
/*!
 *	@brief This API is used to get
 *	the interrupt output type in the register 0x20.
 *	@note INTR1 -> bit 1
 *
 *  @param channel_u8: The value of output type select
 *       channel_u8     |    result
 *       -----------------| ------------------
 *               0        | SMA13X_ACCEL_INTR1_OUTPUT
 *
 *	@param intr_output_type_u8: The value of output type select
 *       intr_source_u8         |    result
 *       ------------------------ | ------------------
 *              0x01              | OPEN_DRAIN
 *              0x00              | PUSS_PULL
 *
 *
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
 *
 */
SMA13X_RETURN_FUNCTION_TYPE sma13x_get_intr_output_type(u8 channel_u8,
u8 *intr_output_type_u8)
{
		u8 data_u8 = SMA13X_INIT_VALUE;
		/*  Variable used to return value of
		communication routine*/
		SMA13X_RETURN_FUNCTION_TYPE com_rslt = ERROR;

		if (p_sma13x == SMA13X_NULL) {
			com_rslt = E_SMA13X_NULL_PTR;
		} else {
		switch (channel_u8) {
		/* read the output type */
		case SMA13X_INTR1_OUTPUT:
			com_rslt = p_sma13x->SMA13X_BUS_READ_FUNC
			(p_sma13x->dev_addr,
			SMA13X_INTR1_PAD_OUTPUT_TYPE_REG,
			&data_u8, SMA13X_GEN_READ_WRITE_LENGTH);
			*intr_output_type_u8 = SMA13X_GET_BITSLICE
			(data_u8, SMA13X_INTR1_PAD_OUTPUT_TYPE);
		break;
		default:
			com_rslt = E_OUT_OF_RANGE;
		break;
		}
	}
	return com_rslt;
}
/*!
 *	@brief This API is used to set
 *	the interrupt output type in the register 0x20.
 *	@note INTR1 -> bit 1
 *
 *  @param channel_u8: The value of output type select
 *         channel_u8   |    result
 *       -----------------| ------------------
 *               0        | SMA13X_ACCEL_INTR1_OUTPUT
 *
 *	@param intr_output_type_u8: The value of output type select
 *       intr_source_u8         |    result
 *       ------------------------ | ------------------
 *              0x01              | OPEN_DRAIN
 *              0x00              | PUSS_PULL
 *
 *
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
 *
 */
SMA13X_RETURN_FUNCTION_TYPE sma13x_set_intr_output_type(u8 channel_u8,
u8 intr_output_type_u8)
{
		u8 data_u8 = SMA13X_INIT_VALUE;
		/*  Variable used to return value of
		communication routine*/
		SMA13X_RETURN_FUNCTION_TYPE com_rslt = ERROR;

		if (p_sma13x == SMA13X_NULL) {
			com_rslt = E_SMA13X_NULL_PTR;
		}  else {
		switch (channel_u8) {
		/* write the output type*/
		case SMA13X_INTR1_OUTPUT:
			com_rslt = p_sma13x->SMA13X_BUS_READ_FUNC
			(p_sma13x->dev_addr,
			SMA13X_INTR1_PAD_OUTPUT_TYPE_REG,
			&data_u8, SMA13X_GEN_READ_WRITE_LENGTH);
			data_u8 = SMA13X_SET_BITSLICE
			(data_u8,
			SMA13X_INTR1_PAD_OUTPUT_TYPE, intr_output_type_u8);
			com_rslt += sma13x_write_reg(
			SMA13X_INTR1_PAD_OUTPUT_TYPE_REG,
			&data_u8, SMA13X_GEN_READ_WRITE_LENGTH);
		break;
		default:
			com_rslt = E_OUT_OF_RANGE;
		break;
		}
	}
	return com_rslt;
}
/*!
 *	@brief This API is used to get
 *	Active Level status in the register 0x20
 *	@note INTR1 -> bit 0
 *
 *  @param channel_u8: The value of Active Level select
 *       channel_u8     |    result
 *       -----------------| ------------------
 *               0        | SMA13X_ACCEL_INTR1_LEVEL
 *
 *  @param intr_level_u8: The Active Level status enable value
 *        intr_level_u8         |    result
 *       ------------------------ | ------------------
 *              0x01              | ACTIVE_HIGH
 *              0x00              | ACTIVE_LOW
 *
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
 *
 */
SMA13X_RETURN_FUNCTION_TYPE sma13x_get_intr_level(u8 channel_u8,
u8 *intr_level_u8)
{
		u8 data_u8 = SMA13X_INIT_VALUE;
		/*  Variable used to return value of
		communication routine*/
		SMA13X_RETURN_FUNCTION_TYPE com_rslt = ERROR;

		if (p_sma13x == SMA13X_NULL) {
			com_rslt = E_SMA13X_NULL_PTR;
		} else {
		switch (channel_u8) {
		/* read the active level*/
		case SMA13X_INTR1_LEVEL:
			com_rslt = p_sma13x->SMA13X_BUS_READ_FUNC
			(p_sma13x->dev_addr,
			SMA13X_INTR1_PAD_ACTIVE_LEVEL_REG,
			&data_u8, SMA13X_GEN_READ_WRITE_LENGTH);
			*intr_level_u8 = SMA13X_GET_BITSLICE
			(data_u8, SMA13X_INTR1_PAD_ACTIVE_LEVEL);
		break;
		default:
			com_rslt = E_OUT_OF_RANGE;
		break;
		}
	}
	return com_rslt;
}
/*!
 *	@brief This API is used to set
 *	Active Level status in the register 0x20
 *	@note INTR1 -> bit 0
 *
 *  @param channel_u8: The value of Active Level select
 *       channel_u8     |    result
 *       -----------------| ------------------
 *               0        | SMA13X_ACCEL_INTR1_LEVEL
 *
 *  @param intr_level_u8: The Active Level status enable value
 *       intr_level_u8          |    result
 *       ------------------------ | ------------------
 *              0x01              | ACTIVE_HIGH
 *              0x00              | ACTIVE_LOW
 *
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
 *
 */
SMA13X_RETURN_FUNCTION_TYPE sma13x_set_intr_level(u8 channel_u8,
u8 intr_level_u8)
{
		u8 data_u8 = SMA13X_INIT_VALUE;
		/*  Variable used to return value of
		communication routine*/
		SMA13X_RETURN_FUNCTION_TYPE com_rslt = ERROR;

		if (p_sma13x == SMA13X_NULL) {
			com_rslt = E_SMA13X_NULL_PTR;
		} else {
		switch (channel_u8) {
		/* write the active level */
		case SMA13X_INTR1_LEVEL:
			com_rslt = p_sma13x->SMA13X_BUS_READ_FUNC
			(p_sma13x->dev_addr,
			SMA13X_INTR1_PAD_ACTIVE_LEVEL_REG,
			&data_u8, SMA13X_GEN_READ_WRITE_LENGTH);
			data_u8 = SMA13X_SET_BITSLICE
			(data_u8,
			SMA13X_INTR1_PAD_ACTIVE_LEVEL, intr_level_u8);
			com_rslt += sma13x_write_reg(
			SMA13X_INTR1_PAD_ACTIVE_LEVEL_REG,
			&data_u8, SMA13X_GEN_READ_WRITE_LENGTH);
		break;
		default:
			com_rslt = E_OUT_OF_RANGE;
		break;
		}
	}
	return com_rslt;
}
/*!
 *	@brief This API is used to set
 *	the reset interrupt in the register 0x21 bit 7
 *
 *
 *
 *  @param  rst_intr_u8: The value of reset interrupt
 *          rst_intr_u8         |  result
 *       ------------------------ | ------------------
 *              0x01              | clear any latch interrupt
 *              0x00              | keep latch interrupt active
 *
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
 *
 */
SMA13X_RETURN_FUNCTION_TYPE sma13x_rst_intr(u8 rst_intr_u8)
{
	u8 data_u8 = SMA13X_INIT_VALUE;
		/*  Variable used to return value of
	communication routine*/
	SMA13X_RETURN_FUNCTION_TYPE com_rslt = ERROR;

	if (p_sma13x == SMA13X_NULL) {
		/* Check the struct p_sma13x is empty */
		return E_SMA13X_NULL_PTR;
		} else {
			com_rslt = p_sma13x->SMA13X_BUS_READ_FUNC
			(p_sma13x->dev_addr, SMA13X_RESET_INTR_REG,
			&data_u8, SMA13X_GEN_READ_WRITE_LENGTH);
			data_u8 = SMA13X_SET_BITSLICE
			(data_u8, SMA13X_RESET_INTR, rst_intr_u8);
			com_rslt += sma13x_write_reg(SMA13X_RESET_INTR_REG,
			&data_u8, SMA13X_GEN_READ_WRITE_LENGTH);
		}
	return com_rslt;
}
/*!
 *	@brief This API is used to get
 *	the latch duration in the register 0x21 bit from 0 to 3
 *
 *	@param latch_intr_u8: The value of latch duration
 *        latch_intr_u8 |  result
 *       -----------------| ------------------
 *               0x00     | SMA13X_LATCH_DURN_NON_LATCH
 *               0x01     | SMA13X_LATCH_DURN_250MS
 *               0x02     | SMA13X_LATCH_DURN_500MS
 *               0x03     | SMA13X_LATCH_DURN_1S
 *               0x04     | SMA13X_LATCH_DURN_2S
 *               0x05     | SMA13X_LATCH_DURN_4S
 *               0x06     | SMA13X_LATCH_DURN_8S
 *               0x07     | SMA13X_LATCH_DURN_LATCH
 *               0x08     | SMA13X_LATCH_DURN_NON_LATCH1
 *               0x09     | SMA13X_LATCH_DURN_250US
 *               0x0A     | SMA13X_LATCH_DURN_500US
 *               0x0B     | SMA13X_LATCH_DURN_1MS
 *               0x0C     | SMA13X_LATCH_DURN_12_5MS
 *               0x0D     | SMA13X_LATCH_DURN_25MS
 *               0x0E     | SMA13X_LATCH_DURN_50MS
 *               0x0F     | SMA13X_LATCH_DURN_LATCH1
 *
 *
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
 *
 */
SMA13X_RETURN_FUNCTION_TYPE sma13x_get_latch_intr(u8 *latch_intr_u8)
{
	u8 data_u8 = SMA13X_INIT_VALUE;
	/*  Variable used to return value of
	communication routine*/
	SMA13X_RETURN_FUNCTION_TYPE com_rslt = ERROR;

	if (p_sma13x == SMA13X_NULL) {
		/* Check the struct p_sma13x is empty */
		return E_SMA13X_NULL_PTR;
		} else {
			/* read the latch duration */
			com_rslt = p_sma13x->SMA13X_BUS_READ_FUNC
			(p_sma13x->dev_addr, SMA13X_LATCH_INTR_REG,
			&data_u8, SMA13X_GEN_READ_WRITE_LENGTH);
			*latch_intr_u8 = SMA13X_GET_BITSLICE
			(data_u8, SMA13X_LATCH_INTR);
		}
	return com_rslt;
}
/*!
 *	@brief This API is used to set
 *	the latch duration in the register 0x21 bit from 0 to 3
 *
 *	@param latch_intr_u8: The value of latch duration
 *        latch_intr_u8 |  result
 *       -----------------| ------------------
 *               0x00     | SMA13X_LATCH_DURN_NON_LATCH
 *               0x01     | SMA13X_LATCH_DURN_250MS
 *               0x02     | SMA13X_LATCH_DURN_500MS
 *               0x03     | SMA13X_LATCH_DURN_1S
 *               0x04     | SMA13X_LATCH_DURN_2S
 *               0x05     | SMA13X_LATCH_DURN_4S
 *               0x06     | SMA13X_LATCH_DURN_8S
 *               0x07     | SMA13X_LATCH_DURN_LATCH
 *               0x08     | SMA13X_LATCH_DURN_NON_LATCH1
 *               0x09     | SMA13X_LATCH_DURN_250US
 *               0x0A     | SMA13X_LATCH_DURN_500US
 *               0x0B     | SMA13X_LATCH_DURN_1MS
 *               0x0C     | SMA13X_LATCH_DURN_12_5MS
 *               0x0D     | SMA13X_LATCH_DURN_25MS
 *               0x0E     | SMA13X_LATCH_DURN_50MS
 *               0x0F     | SMA13X_LATCH_DURN_LATCH1
 *
 *
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
 *
 */
SMA13X_RETURN_FUNCTION_TYPE sma13x_set_latch_intr(u8 latch_intr_u8)
{
u8 data_u8 = SMA13X_INIT_VALUE;
/*  Variable used to return value of
communication routine*/
SMA13X_RETURN_FUNCTION_TYPE com_rslt = ERROR;
u8 latch_durn_u8 = SMA13X_INIT_VALUE;
if (p_sma13x == SMA13X_NULL)  {
		/* Check the struct p_sma13x is empty */
		return E_SMA13X_NULL_PTR;
		} else  {
		if (latch_intr_u8 < SMA13X_ACCEL_BW_MAX_RANGE) {
			switch (latch_intr_u8) {
			case SMA13X_LATCH_DURN_NON_LATCH:
				latch_durn_u8 = SMA13X_LATCH_DURN_NON_LATCH;

				/*  NON LATCH   */
			break;
			case SMA13X_LATCH_DURN_250MS:
				latch_durn_u8 = SMA13X_LATCH_DURN_250MS;

				/*  250 MS  */
			break;
			case SMA13X_LATCH_DURN_500MS:
				latch_durn_u8 = SMA13X_LATCH_DURN_500MS;

				/*  500 MS  */
			break;
			case SMA13X_LATCH_DURN_1S:
				latch_durn_u8 = SMA13X_LATCH_DURN_1S;

				/*  1 S   */
			break;
			case SMA13X_LATCH_DURN_2S:
				latch_durn_u8 = SMA13X_LATCH_DURN_2S;

				/*  2 S  */
			break;
			case SMA13X_LATCH_DURN_4S:
				latch_durn_u8 = SMA13X_LATCH_DURN_4S;

				/*  4 S  */
			break;
			case SMA13X_LATCH_DURN_8S:
				latch_durn_u8 = SMA13X_LATCH_DURN_8S;

				/*  8 S  */
			break;
			case SMA13X_LATCH_DURN_LATCH:
				latch_durn_u8 = SMA13X_LATCH_DURN_LATCH;

				/*  LATCH  */
			break;
			case SMA13X_LATCH_DURN_NON_LATCH1:
				latch_durn_u8 = SMA13X_LATCH_DURN_NON_LATCH1;

				/*  NON LATCH1  */
			break;
			case SMA13X_LATCH_DURN_250US:
				latch_durn_u8 = SMA13X_LATCH_DURN_250US;

				/*  250 US   */
			break;
			case SMA13X_LATCH_DURN_500US:
				latch_durn_u8 = SMA13X_LATCH_DURN_500US;

				/*  500 US   */
			break;
			case SMA13X_LATCH_DURN_1MS:
				latch_durn_u8 = SMA13X_LATCH_DURN_1MS;

				/*  1 MS   */
			break;
			case SMA13X_LATCH_DURN_12_5MS:
				latch_durn_u8 = SMA13X_LATCH_DURN_12_5MS;

				/*  12.5 MS   */
			break;
			case SMA13X_LATCH_DURN_25MS:
				latch_durn_u8 = SMA13X_LATCH_DURN_25MS;

				/*  25 MS   */
			break;
			case SMA13X_LATCH_DURN_50MS:
				latch_durn_u8 = SMA13X_LATCH_DURN_50MS;

				/*  50 MS   */
			break;
			case SMA13X_LATCH_DURN_LATCH1:
				latch_durn_u8 = SMA13X_LATCH_DURN_LATCH1;

				/*  LATCH1   */
			break;
			default:
			break;
			}
			/* write the latch duration */
			com_rslt = p_sma13x->SMA13X_BUS_READ_FUNC
			(p_sma13x->dev_addr, SMA13X_LATCH_INTR_REG,
			&data_u8, SMA13X_GEN_READ_WRITE_LENGTH);
			data_u8 = SMA13X_SET_BITSLICE
			(data_u8, SMA13X_LATCH_INTR, latch_durn_u8);
			com_rslt += sma13x_write_reg(SMA13X_LATCH_INTR_REG,
			&data_u8, SMA13X_GEN_READ_WRITE_LENGTH);
		} else {
		com_rslt = E_OUT_OF_RANGE;
		}
	}
	return com_rslt;
}
/*!
 *	@brief This API is used to get the duration of
 *	Low, High, Slope and slow no motion interrupts in the registers
 *	@note HIGH_DURN		-> register 0x25 bit form 0 to 7
 *	@note SLOPE_DURN		-> register 0x27 bit form 0 to 1
 *	@note SLO_NO_MOT_DURN -> register 0x27 bit form 2 to 7
 *
 *  @param channel_u8: The value of duration select
 *     channel_u8   | result
 *   -----------------| ------------------
 *               1    | SMA13X_ACCEL_HIGH_DURN
 *               2    | SMA13X_ACCEL_SLOPE_DURN
 *               3    | SMA13X_ACCEL_SLOW_NO_MOTION_DURN
 *
 *	@param durn_u8: The value of duration
 *
 *	@note :
 *     Duration           |    result
 * -----------------------| ------------------
 * SMA13X_ACCEL_HIGH_DURN | high-g interrupt trigger
 *         -              | delay according to([durn_u8 +1]*2)ms
 *         -              | range from 2ms to 512ms. default is 32ms
 * SMA13X_ACCEL_SLOPE_DURN| slope interrupt trigger
 *         -              | if[durn_u8<1:0>+1] consecutive data points
 *         -              | are above the slope interrupt threshold
 * SLO_NO_MOT_DURN        | Refer data sheet for clear information
 *
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
 *
 */
SMA13X_RETURN_FUNCTION_TYPE sma13x_get_durn(u8 channel_u8,
u8 *durn_u8)
{
	u8 data_u8 = SMA13X_INIT_VALUE;
	/*  Variable used to return value of
	communication routine*/
	SMA13X_RETURN_FUNCTION_TYPE com_rslt = ERROR;

	if (p_sma13x == SMA13X_NULL) {
		/* Check the struct p_sma13x is empty */
		return E_SMA13X_NULL_PTR;
		} else {
		/* write the duration data */
		switch (channel_u8) {
		case SMA13X_HIGH_DURN:
			/*HIGH DURATION*/
			com_rslt = p_sma13x->SMA13X_BUS_READ_FUNC
			(p_sma13x->dev_addr, SMA13X_HIGH_DURN_ADDR,
			&data_u8, SMA13X_GEN_READ_WRITE_LENGTH);
			*durn_u8 = data_u8;
		break;
		case SMA13X_SLOPE_DURN:
			/*SLOPE DURATION*/
			com_rslt = p_sma13x->SMA13X_BUS_READ_FUNC
			(p_sma13x->dev_addr, SMA13X_SLOPE_DURN_REG,
			&data_u8, SMA13X_GEN_READ_WRITE_LENGTH);
			*durn_u8 = SMA13X_GET_BITSLICE
			(data_u8, SMA13X_SLOPE_DURN);
		break;
		case SMA13X_SLOW_NO_MOTION_DURN:
			/*SLO NO MOT DURATION*/
			com_rslt = p_sma13x->SMA13X_BUS_READ_FUNC
			(p_sma13x->dev_addr,
			SMA13X_SLOW_NO_MOTION_DURN_REG,
			&data_u8, SMA13X_GEN_READ_WRITE_LENGTH);
			*durn_u8 = SMA13X_GET_BITSLICE
			(data_u8, SMA13X_SLOW_NO_MOTION_DURN);
		break;
		default:
			com_rslt = E_OUT_OF_RANGE;
		break;
		}
	}
	return com_rslt;
}
/*!
 *	@brief This API is used to set the duration of
 *	Low, High, Slope and slow no motion interrupts in the registers
 *	@note HIGH_DURN		-> register 0x25 bit form 0 to 7
 *	@note SLOPE_DURN		-> register 0x27 bit form 0 to 1
 *	@note SLO_NO_MOT_DURN -> register 0x27 bit form 2 to 7
 *
 *  @param channel_u8: The value of duration select
 *     channel_u8   | result
 *   -----------------| ------------------
 *               1    | SMA13X_ACCEL_HIGH_DURN
 *               2    | SMA13X_ACCEL_SLOPE_DURN
 *               3    | SMA13X_ACCEL_SLOW_NO_MOTION_DURN
 *
 *	@param durn_u8: The value of duration
 *
 *	@note :
 *     Duration           |    result
 * -----------------------| ------------------
 * SMA13X_ACCEL_HIGH_DURN | high-g interrupt trigger
 *         -              | delay according to([durn_u8 +1]*2)ms
 *         -              | range from 2ms to 512ms. default is 32ms
 * SMA13X_ACCEL_SLOPE_DURN| slope interrupt trigger
 *         -              | if[durn_u8<1:0>+1] consecutive data points
 *         -              | are above the slope interrupt threshold
 * SLO_NO_MOT_DURN        | Refer data sheet for clear information
 *
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
 *
 */
SMA13X_RETURN_FUNCTION_TYPE sma13x_set_durn(u8 channel_u8,
u8 durn_u8)
{
	u8 data_u8 = SMA13X_INIT_VALUE;
		/*  Variable used to return value of
	communication routine*/
	SMA13X_RETURN_FUNCTION_TYPE com_rslt = ERROR;

	if (p_sma13x == SMA13X_NULL)  {
		/* Check the struct p_sma13x is empty */
		return E_SMA13X_NULL_PTR;
		}  else  {
		/* write duration data */
		switch (channel_u8)   {
		case SMA13X_HIGH_DURN:
			/*HIGH DURATION*/
			data_u8 = durn_u8;
			com_rslt = sma13x_write_reg(
			SMA13X_HIGH_DURN_ADDR,
			&data_u8, SMA13X_GEN_READ_WRITE_LENGTH);
		break;
		case SMA13X_SLOPE_DURN:
			/*SLOPE DURATION*/
			com_rslt = p_sma13x->SMA13X_BUS_READ_FUNC
			(p_sma13x->dev_addr,
			SMA13X_SLOPE_DURN_REG,
			&data_u8, SMA13X_GEN_READ_WRITE_LENGTH);
			data_u8 = SMA13X_SET_BITSLICE
			(data_u8, SMA13X_SLOPE_DURN, durn_u8);
			com_rslt += sma13x_write_reg(
			SMA13X_SLOPE_DURN_REG,
			&data_u8, SMA13X_GEN_READ_WRITE_LENGTH);
		break;
		case SMA13X_SLOW_NO_MOTION_DURN:
			/*SLO NO MOT DURATION*/
			com_rslt = p_sma13x->SMA13X_BUS_READ_FUNC
			(p_sma13x->dev_addr,
			SMA13X_SLOW_NO_MOTION_DURN_REG,
			&data_u8, SMA13X_GEN_READ_WRITE_LENGTH);
			data_u8 = SMA13X_SET_BITSLICE
			(data_u8, SMA13X_SLOW_NO_MOTION_DURN, durn_u8);
			com_rslt += sma13x_write_reg(
			SMA13X_SLOW_NO_MOTION_DURN_REG,
			&data_u8, SMA13X_GEN_READ_WRITE_LENGTH);
		break;
		default:
			com_rslt = E_OUT_OF_RANGE;
		break;
		}
	}
	return com_rslt;
}
/*!
 * @brief This API is used to get the threshold of
 *	Low, High, Slope and slow no motion interrupts in the registers
 *	@note HIGH_THRES		-> register 0x26 bit form 0 to 7
 *	@note SLOPE_THRES		-> register 0x28 bit form 0 to 7
 *	@note SLO_NO_MOT_THRES -> register 0x29 bit form 0 to 7
 *
 *  @param channel_u8: The value of threshold selection
 *     channel_u8   | result
 *   -----------------| ------------------
 *               1    | SMA13X_ACCEL_HIGH_THRES
 *               2    | SMA13X_ACCEL_SLOPE_THRES
 *               3    | SMA13X_ACCEL_SLOW_NO_MOTION_THRES
 *
 *  @param thres_u8: The threshold value of selected interrupts
 *
 *	@note : HIGH-G THRESHOLD
 *	@note Threshold of high-g interrupt according to accel g range
 *    g-range           |      High-g threshold
 *  --------------------|----------------------------
 *     2g               |    (thres_u8 * 7.81) mg
 *     4g               |    (thres_u8 * 15.63) mg
 *     8g               |    (thres_u8 * 31.25) mg
 *     16g              |    (thres_u8 * 62.5) mg
 *
 *	@note : SLOPE THRESHOLD
 *	@note Threshold of slope interrupt according to accel g range
 *    g-range           |      Slope threshold
 *  --------------------|----------------------------
 *     2g               |    (thres_u8 * 3.19) mg
 *     4g               |    (thres_u8 * 7.81) mg
 *     8g               |    (thres_u8 * 15.63) mg
 *     16g              |    (thres_u8 * 31.25) mg
 *
 *	@note : SLOW NO MOTION THRESHOLD
 *	@note Threshold of slow no motion interrupt according to accel g range
 *    g-range           |   slow no motion threshold
 *  --------------------|----------------------------
 *     2g               |    (thres_u8 * 3.19) mg
 *     4g               |    (thres_u8 * 7.81) mg
 *     8g               |    (thres_u8 * 15.63) mg
 *     16g              |    (thres_u8 * 31.25) mg
 *
 *
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
 *
 */
SMA13X_RETURN_FUNCTION_TYPE sma13x_get_thres(u8 channel_u8,
u8 *thres_u8)
{
	u8 data_u8 = SMA13X_INIT_VALUE;
	/*  Variable used to return value of
	communication routine*/
	SMA13X_RETURN_FUNCTION_TYPE com_rslt = ERROR;

	if (p_sma13x == SMA13X_NULL) {
		/* Check the struct p_sma13x is empty */
		return E_SMA13X_NULL_PTR;
		} else {
		switch (channel_u8) {
		/* Read the threshold value */
		case SMA13X_HIGH_THRES:
			/*HIGH THRESHOLD*/
			com_rslt = p_sma13x->SMA13X_BUS_READ_FUNC
			(p_sma13x->dev_addr,
			SMA13X_HIGH_THRES_ADDR,
			&data_u8, SMA13X_GEN_READ_WRITE_LENGTH);
			*thres_u8 = data_u8;
		break;
		case SMA13X_SLOPE_THRES:
			/*SLOPE THRESHOLD*/
			com_rslt = p_sma13x->SMA13X_BUS_READ_FUNC
			(p_sma13x->dev_addr,
			SMA13X_SLOPE_THRES_ADDR,
			&data_u8, SMA13X_GEN_READ_WRITE_LENGTH);
			*thres_u8 = data_u8;
		break;
		case SMA13X_SLOW_NO_MOTION_THRES:
			/*SLO NO MOT THRESHOLD*/
			com_rslt = p_sma13x->SMA13X_BUS_READ_FUNC
			(p_sma13x->dev_addr,
			SMA13X_SLOW_NO_MOTION_THRES_ADDR,
			&data_u8, SMA13X_GEN_READ_WRITE_LENGTH);
			*thres_u8 = data_u8;
		break;
		default:
			com_rslt = E_OUT_OF_RANGE;
		break;
		}
	}
	return com_rslt;
}
/*!
 * @brief This API is used to set the threshold of
 *	Low, High, Slope and slow no motion interrupts in the registers
 *	@note HIGH_THRES		-> register 0x26 bit form 0 to 7
 *	@note SLOPE_THRES		-> register 0x28 bit form 0 to 7
 *	@note SLO_NO_MOT_THRES -> register 0x29 bit form 0 to 7
 *
 *  @param channel_u8: The value of threshold selection
 *     channel_u8   | result
 *   -----------------| ------------------
 *               1    | SMA13X_ACCEL_HIGH_THRES
 *               2    | SMA13X_ACCEL_SLOPE_THRES
 *               3    | SMA13X_ACCEL_SLOW_NO_MOTION_THRES
 *
 *  @param thres_u8: The threshold value of selected interrupts
 *
 *	@note : HIGH-G THRESHOLD
 *	@note Threshold of high-g interrupt according to accel g range
 *    g-range           |      High-g threshold
 *  --------------------|----------------------------
 *     2g               |    (thres_u8 * 7.81) mg
 *     4g               |    (thres_u8 * 15.63) mg
 *     8g               |    (thres_u8 * 31.25) mg
 *     16g              |    (thres_u8 * 62.5) mg
 *
 *	@note : SLOPE THRESHOLD
 *	@note Threshold of slope interrupt according to accel g range
 *    g-range           |      Slope threshold
 *  --------------------|----------------------------
 *     2g               |    (thres_u8 * 3.19) mg
 *     4g               |    (thres_u8 * 7.81) mg
 *     8g               |    (thres_u8 * 15.63) mg
 *     16g              |    (thres_u8 * 31.25) mg
 *
 *	@note : SLOW NO MOTION THRESHOLD
 *	@note Threshold of slow no motion interrupt according to accel g range
 *    g-range           |   slow no motion threshold
 *  --------------------|----------------------------
 *     2g               |    (thres_u8 * 3.19) mg
 *     4g               |    (thres_u8 * 7.81) mg
 *     8g               |    (thres_u8 * 15.63) mg
 *     16g              |    (thres_u8 * 31.25) mg
 *
 *
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
 *
 */
SMA13X_RETURN_FUNCTION_TYPE sma13x_set_thres(u8 channel_u8,
u8 thres_u8)
{
	u8 data_u8 = SMA13X_INIT_VALUE;
	/*  Variable used to return value of
	communication routine*/
	SMA13X_RETURN_FUNCTION_TYPE com_rslt = ERROR;

	if (p_sma13x == SMA13X_NULL) {
		/* Check the struct p_sma13x is empty */
		return E_SMA13X_NULL_PTR;
		} else {
		switch (channel_u8) {
		/* write the threshold value*/
		case SMA13X_HIGH_THRES:
			/*HIGH THRESHOLD*/
			data_u8 = thres_u8;
			com_rslt = sma13x_write_reg(
			SMA13X_HIGH_THRES_ADDR, &data_u8,
			SMA13X_GEN_READ_WRITE_LENGTH);
		break;
		case SMA13X_SLOPE_THRES:
			/*SLOPE THRESHOLD*/
			data_u8 = thres_u8;
			com_rslt = sma13x_write_reg(
			SMA13X_SLOPE_THRES_ADDR, &data_u8,
			SMA13X_GEN_READ_WRITE_LENGTH);
		break;
		case SMA13X_SLOW_NO_MOTION_THRES:
			/*SLO NO MOT THRESHOLD*/
			data_u8 = thres_u8;
			com_rslt = sma13x_write_reg(
			SMA13X_SLOW_NO_MOTION_THRES_ADDR,
			&data_u8, SMA13X_GEN_READ_WRITE_LENGTH);
		break;
		default:
			com_rslt = E_OUT_OF_RANGE;
		break;
		}
	}
	return com_rslt;
}
/*!
 *	@brief This API is used to get
 *	the low high hysteresis in the registers 0x24
 *	@note HIGH_G_HYST  -> bit from 6 to 7
 *
 *  @param channel_u8: The value of hysteresis selection
 *     channel_u8   | result
 *   -----------------| ------------------
 *           1        | SMA13X_ACCEL_HIGH_G_HYST
 *
 *  @param hyst_u8: The hysteresis data
 *
 *	@note HIGH HYSTERESIS
 *	@note High hysteresis depends on the accel range selection
 *    g-range           |    High Hysteresis
 *  --------------------|----------------------------
 *     2g               |    (thres_u8 * 125) mg
 *     4g               |    (thres_u8 * 250) mg
 *     8g               |    (thres_u8 * 500) mg
 *     16g              |    (thres_u8 * 1000) mg
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
 *
 */
SMA13X_RETURN_FUNCTION_TYPE sma13x_get_low_high_g_hyst(u8 channel_u8,
u8 *hyst_u8)
{
	u8 data_u8 = SMA13X_INIT_VALUE;
	/*  Variable used to return value of
	communication routine*/
	SMA13X_RETURN_FUNCTION_TYPE com_rslt = ERROR;

	if (p_sma13x == SMA13X_NULL) {
		/* Check the struct p_sma13x is empty */
		return E_SMA13X_NULL_PTR;
		} else {
		switch (channel_u8) {
		/* read the hysteresis data */
		case SMA13X_HIGH_G_HYST:
			com_rslt = p_sma13x->SMA13X_BUS_READ_FUNC
			(p_sma13x->dev_addr,
			SMA13X_HIGH_G_HYST_REG,
			&data_u8, SMA13X_GEN_READ_WRITE_LENGTH);
			*hyst_u8 = SMA13X_GET_BITSLICE
			(data_u8, SMA13X_HIGH_G_HYST);
		break;
		default:
			com_rslt = E_OUT_OF_RANGE;
		break;
		}
	}
	return com_rslt;
}
/*!
 *	@brief This API is used to set
 *	the low high hysteresis in the registers 0x24
 *	@note HIGH_G_HYST  -> bit from 6 to 7
 *
 *  @param channel_u8: The value of hysteresis selection
 *     channel_u8   | result
 *   -----------------| ------------------
 *           1        | SMA13X_ACCEL_HIGH_G_HYST
 *
 *  @param hyst_u8: The hysteresis data
 *
 *	@note HIGH HYSTERESIS
 *	@note High hysteresis depends on the accel range selection
 *    g-range           |    High Hysteresis
 *  --------------------|----------------------------
 *     2g               |    (thres_u8 * 125) mg
 *     4g               |    (thres_u8 * 250) mg
 *     8g               |    (thres_u8 * 500) mg
 *     16g              |    (thres_u8 * 1000) mg
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
 *
 */
SMA13X_RETURN_FUNCTION_TYPE sma13x_set_low_high_g_hyst(u8 channel_u8,
u8 hyst_u8)
{
	u8 data_u8 = SMA13X_INIT_VALUE;
	/*  Variable used to return value of
	communication routine*/
	SMA13X_RETURN_FUNCTION_TYPE com_rslt = ERROR;

	if (p_sma13x == SMA13X_NULL) {
		/* Check the struct p_sma13x is empty */
		return E_SMA13X_NULL_PTR;
		} else {
		switch (channel_u8) {
		/* write the hysteresis data  */
		case SMA13X_HIGH_G_HYST:
			com_rslt = p_sma13x->SMA13X_BUS_READ_FUNC
			(p_sma13x->dev_addr,
			SMA13X_HIGH_G_HYST_REG, &data_u8,
			SMA13X_GEN_READ_WRITE_LENGTH);
			data_u8 = SMA13X_SET_BITSLICE
			(data_u8, SMA13X_HIGH_G_HYST, hyst_u8);
			com_rslt += sma13x_write_reg(
			SMA13X_HIGH_G_HYST_REG,
			&data_u8,  SMA13X_GEN_READ_WRITE_LENGTH);
		break;
		default:
			com_rslt = E_OUT_OF_RANGE;
		break;
		}
	}
	return com_rslt;
}
/*!
 *	@brief This API is for to get
 *	the self test axis(self_test_axis) in the register ox32 bit 0 to 2
 *
 *
 *
 *  @param selftest_axis_u8 : The value of selftest axis
 *     selftest_axis_u8     |    result
 *  ------------------------- |------------------
 *     0x00                   | self test disable
 *     0x01                   | x-axis
 *     0x02                   | y-axis
 *     0x03                   | z-axis
 *
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
 *
 */
SMA13X_RETURN_FUNCTION_TYPE sma13x_get_selftest_axis(
u8 *selftest_axis_u8)
{
	u8 data_u8 = SMA13X_INIT_VALUE;
	/*  Variable used to return value of
	communication routine*/
	SMA13X_RETURN_FUNCTION_TYPE com_rslt = ERROR;

	if (p_sma13x == SMA13X_NULL) {
		/* Check the struct p_sma13x is empty */
		return E_SMA13X_NULL_PTR;
		} else {
			/* read the self test axis*/
			com_rslt = p_sma13x->SMA13X_BUS_READ_FUNC
			(p_sma13x->dev_addr,
			SMA13X_ENABLE_SELFTEST_REG,
			&data_u8, SMA13X_GEN_READ_WRITE_LENGTH);
			*selftest_axis_u8 = SMA13X_GET_BITSLICE
			(data_u8, SMA13X_ENABLE_SELFTEST);
		}
	return com_rslt;
}
/*!
 *	@brief This API is for to set
 *	the self test axis(self_test_axis) in the register ox32 bit 0 to 2
 *
 *
 *
 *  @param selftest_axis_u8 : The value of selftest axis
 *     selftest_axis_u8     |    result
 *  ------------------------- |------------------
 *     0x00                   | self test disable
 *     0x01                   | x-axis
 *     0x02                   | y-axis
 *     0x03                   | z-axis
 *
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
 *
 */
SMA13X_RETURN_FUNCTION_TYPE sma13x_set_selftest_axis(
u8 selftest_axis_u8)
{
	u8 data_u8 = SMA13X_INIT_VALUE;
	/*  Variable used to return value of
	communication routine*/
	SMA13X_RETURN_FUNCTION_TYPE com_rslt = ERROR;

	if (p_sma13x == SMA13X_NULL) {
		/* Check the struct p_sma13x is empty */
		return E_SMA13X_NULL_PTR;
		} else {
		if (selftest_axis_u8 < SMA13X_SELF_TEST_AXIS_RANGE) {
			/* write the self test axis*/
			com_rslt = p_sma13x->SMA13X_BUS_READ_FUNC
			(p_sma13x->dev_addr,
			SMA13X_ENABLE_SELFTEST_REG,
			&data_u8, SMA13X_GEN_READ_WRITE_LENGTH);
			data_u8 = SMA13X_SET_BITSLICE
			(data_u8, SMA13X_ENABLE_SELFTEST, selftest_axis_u8);
			com_rslt += sma13x_write_reg(
			SMA13X_ENABLE_SELFTEST_REG,
			&data_u8, SMA13X_GEN_READ_WRITE_LENGTH);
		 } else {
		com_rslt = E_OUT_OF_RANGE;
		}
	}
	return com_rslt;
}
/*!
 *	@brief This API is for to get
 *	the Self Test sign(selftest_sign) in the register 0x32 bit 2
 *
 *
 *
 *  @param selftest_sign_u8 : The value of self test sign
 *     selftest_sign_u8     |    result
 *  ------------------------- |------------------
 *     0x00                   | negative sign
 *     0x01                   | positive sign
 *
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
 *
 */
SMA13X_RETURN_FUNCTION_TYPE sma13x_get_selftest_sign(
u8 *selftest_sign_u8)
{
	u8 data_u8 = SMA13X_INIT_VALUE;
	/*  Variable used to return value of
	communication routine*/
	SMA13X_RETURN_FUNCTION_TYPE com_rslt = ERROR;

	if (p_sma13x == SMA13X_NULL) {
		/* Check the struct p_sma13x is empty */
		return E_SMA13X_NULL_PTR;
		} else {
			/* read self test sign */
			com_rslt = p_sma13x->SMA13X_BUS_READ_FUNC
			(p_sma13x->dev_addr,
			SMA13X_NEG_SELFTEST_REG,
			&data_u8, SMA13X_GEN_READ_WRITE_LENGTH);
			*selftest_sign_u8 = SMA13X_GET_BITSLICE
			(data_u8, SMA13X_NEG_SELFTEST);
		}
	return com_rslt;
}
/*!
 *	@brief This API is for to set
 *	the Self Test sign(selftest_sign) in the register 0x32 bit 2
 *
 *
 *
 *  @param selftest_sign_u8 : The value of self test sign
 *     selftest_sign_u8     |    result
 *  ------------------------- |------------------
 *     0x00                   | negative sign
 *     0x01                   | positive sign
 *
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
 *
 */
SMA13X_RETURN_FUNCTION_TYPE sma13x_set_selftest_sign(
u8 selftest_sign_u8)
{
	u8 data_u8 = SMA13X_INIT_VALUE;
	/*  Variable used to return value of
	communication routine*/
	SMA13X_RETURN_FUNCTION_TYPE com_rslt = ERROR;

	if (p_sma13x == SMA13X_NULL) {
		/* Check the struct p_sma13x is empty */
		return E_SMA13X_NULL_PTR;
		} else {
		if (selftest_sign_u8 <
		SMA13X_SELF_TEST_SIGN_RANGE) {
			/* write self test sign */
			com_rslt = p_sma13x->SMA13X_BUS_READ_FUNC
			(p_sma13x->dev_addr,
			SMA13X_NEG_SELFTEST_REG,
			&data_u8, SMA13X_GEN_READ_WRITE_LENGTH);
			data_u8 = SMA13X_SET_BITSLICE
			(data_u8, SMA13X_NEG_SELFTEST, selftest_sign_u8);
			com_rslt += sma13x_write_reg(
			SMA13X_NEG_SELFTEST_REG,
			&data_u8, SMA13X_GEN_READ_WRITE_LENGTH);
		} else {
		com_rslt = E_OUT_OF_RANGE;
		}
	}
	return com_rslt;
}
/*!
 *	@brief This API is used to get the enable status of spi3
 *	in the register 0x34 bit 0
 *
 *
 *
 *  @param  spi3_u8 : The value of SPI 3 or 4 wire enable
 *     spi3_u8              |    result
 *  ------------------------- |------------------
 *     0x00                   |     spi4
 *     0x01                   |     spi3
 *
 *
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
 *
 */
SMA13X_RETURN_FUNCTION_TYPE sma13x_get_spi3(u8 *spi3_u8)
{
	u8 data_u8 = SMA13X_INIT_VALUE;
	/*  Variable used to return value of
	communication routine*/
	SMA13X_RETURN_FUNCTION_TYPE com_rslt = ERROR;

	if (p_sma13x == SMA13X_NULL) {
		/* Check the struct p_sma13x is empty */
		return E_SMA13X_NULL_PTR;
		} else {
			/* read the spi status*/
			com_rslt = p_sma13x->SMA13X_BUS_READ_FUNC
			(p_sma13x->dev_addr,
			SMA13X_ENABLE_SPI_MODE_3_REG,
			&data_u8, SMA13X_GEN_READ_WRITE_LENGTH);
			*spi3_u8 = SMA13X_GET_BITSLICE
			(data_u8, SMA13X_ENABLE_SPI_MODE_3);
		}
	return com_rslt;
}
/*!
 *	@brief This API is used to set the enable status of spi3
 *	in the register 0x34 bit 0
 *
 *
 *
 *  @param  spi3_u8 : The value of SPI 3 or 4 wire enable
 *     spi3_u8              |    result
 *  ------------------------- |------------------
 *     0x00                   |     spi4
 *     0x01                   |     spi3
 *
 *
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
 *
 */
SMA13X_RETURN_FUNCTION_TYPE sma13x_set_spi3(u8 spi3_u8)
{
	u8 data_u8 = SMA13X_INIT_VALUE;
	/*  Variable used to return value of
	communication routine*/
	SMA13X_RETURN_FUNCTION_TYPE com_rslt = ERROR;

	if (p_sma13x == SMA13X_NULL) {
		/* Check the struct p_sma13x is empty */
		return E_SMA13X_NULL_PTR;
		} else {
			/* write the spi status*/
			com_rslt = p_sma13x->SMA13X_BUS_READ_FUNC
			(p_sma13x->dev_addr,
			SMA13X_ENABLE_SPI_MODE_3_REG,
			&data_u8, SMA13X_GEN_READ_WRITE_LENGTH);
			data_u8 = SMA13X_SET_BITSLICE
			(data_u8, SMA13X_ENABLE_SPI_MODE_3, spi3_u8);
			com_rslt += sma13x_write_reg(
			SMA13X_ENABLE_SPI_MODE_3_REG,
			&data_u8, SMA13X_GEN_READ_WRITE_LENGTH);
		}
	return com_rslt;
}
/*!
 *	@brief This API is used to get the i2c
 *	watch dog timer period and I2C interface mode is selected
 *	in the register 0x34 bit 1 and 2
 *
 *
 *  @param channel_u8: The i2c option selection
 *     channel_u8           |    result
 *  ------------------------- |------------------
 *        0                   |   SMA13X_ACCEL_I2C_SELECT
 *        1                   |   SMA13X_ACCEL_I2C_ENABLE
 *
 *  @param i2c_wdt_u8: watch dog timer period
 *	and I2C interface mode is selected
 *     SMA13X_ACCEL_I2C_SELECT|    result
 *  ------------------------- |------------------
 *     0x00                   | Disable the watchdog at SDI pin
 *     0x01                   | Enable watchdog
 *
 *     SMA13X_I2C_ENABLE      |    result
 *  ------------------------- |------------------
 *     0x00                   | 1ms
 *     0x01                   | 50ms
 *
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
 *
 */
SMA13X_RETURN_FUNCTION_TYPE sma13x_get_i2c_wdt(u8 channel_u8,
u8 *i2c_wdt_u8)
{
	u8 data_u8 = SMA13X_INIT_VALUE;
		/*  Variable used to return value of
	communication routine*/
	SMA13X_RETURN_FUNCTION_TYPE com_rslt = ERROR;

	if (p_sma13x == SMA13X_NULL) {
		/* Check the struct p_sma13x is empty */
		return E_SMA13X_NULL_PTR;
		} else {
		switch (channel_u8) {
		case SMA13X_I2C_SELECT:
			com_rslt = p_sma13x->SMA13X_BUS_READ_FUNC
			(p_sma13x->dev_addr,
			SMA13X_I2C_WDT_PERIOD_REG,
			&data_u8, SMA13X_GEN_READ_WRITE_LENGTH);
			*i2c_wdt_u8 = SMA13X_GET_BITSLICE(data_u8,
			SMA13X_I2C_WDT_PERIOD);
		break;
		case SMA13X_I2C_ENABLE:
			com_rslt = p_sma13x->SMA13X_BUS_READ_FUNC
			(p_sma13x->dev_addr,
			SMA13X_ENABLE_I2C_WDT_REG,
			&data_u8, SMA13X_GEN_READ_WRITE_LENGTH);
			*i2c_wdt_u8 = SMA13X_GET_BITSLICE
			(data_u8, SMA13X_ENABLE_I2C_WDT);
		break;
		default:
		com_rslt = E_OUT_OF_RANGE;
		break;
		}
	}
	return com_rslt;
}
/*!
 *	@brief This API is used to set the i2c
 *	watch dog timer period and I2C interface mode is selected
 *	in the register 0x34 bit 1 and 2
 *
 *
 *  @param channel_u8: The i2c option selection
 *     channel_u8           |    result
 *  ------------------------- |------------------
 *        0                   |   SMA13X_ACCEL_I2C_SELECT
 *        1                   |   SMA13X_ACCEL_I2C_ENABLE
 *
 *  @param i2c_wdt_u8: watch dog timer period
 *	and I2C interface mode is selected
 *     SMA13X_ACCEL_I2C_SELECT|    result
 *  ------------------------- |------------------
 *     0x00                   | Disable the watchdog at SDI pin
 *     0x01                   | Enable watchdog
 *
 *     SMA13X_I2C_ENABLE      |    result
 *  ------------------------- |------------------
 *     0x00                   | 1ms
 *     0x01                   | 50ms
 *
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
 *
 */
SMA13X_RETURN_FUNCTION_TYPE sma13x_set_i2c_wdt(u8 channel_u8,
u8 i2c_wdt_u8)
{
	u8 data_u8 = SMA13X_INIT_VALUE;
	/*  Variable used to return value of
	communication routine*/
	SMA13X_RETURN_FUNCTION_TYPE com_rslt = ERROR;

	if (p_sma13x == SMA13X_NULL) {
		/* Check the struct p_sma13x is empty */
		return E_SMA13X_NULL_PTR;
		} else {
		switch (channel_u8) {
		case SMA13X_I2C_SELECT:
			com_rslt = p_sma13x->SMA13X_BUS_READ_FUNC
			(p_sma13x->dev_addr,
			SMA13X_I2C_WDT_PERIOD_REG,
			&data_u8, SMA13X_GEN_READ_WRITE_LENGTH);
			data_u8 = SMA13X_SET_BITSLICE
			(data_u8,
			SMA13X_I2C_WDT_PERIOD, i2c_wdt_u8);
			com_rslt += sma13x_write_reg(
			SMA13X_I2C_WDT_PERIOD_REG,
			&data_u8, SMA13X_GEN_READ_WRITE_LENGTH);
		break;
		case SMA13X_I2C_ENABLE:
			com_rslt = p_sma13x->SMA13X_BUS_READ_FUNC
			(p_sma13x->dev_addr,
			SMA13X_ENABLE_I2C_WDT_REG,
			&data_u8, SMA13X_GEN_READ_WRITE_LENGTH);
			data_u8 = SMA13X_SET_BITSLICE
			(data_u8,
			SMA13X_ENABLE_I2C_WDT, i2c_wdt_u8);
			com_rslt += sma13x_write_reg(
			SMA13X_ENABLE_I2C_WDT_REG,
			&data_u8, SMA13X_GEN_READ_WRITE_LENGTH);
		break;
		default:
			com_rslt = E_OUT_OF_RANGE;
		break;
		}
	}
	return com_rslt;
}
/*!
 *	@brief This API is used to get
 *	slow compensation(hp_x_enable, hp_y_enable and hp_z_enable) enable
 *	in the register 0x36 bit 0 to 2
 *	@note SLOW_COMP_X -> bit 0
 *	@note SLOW_COMP_Y -> bit 1
 *	@note SLOW_COMP_Z -> bit 2
 *
 *
 *	@param channel_u8: The value of slow compensation selection
 *     channel_u8           |    result
 *  ------------------------- |------------------
 *        0                   |   SMA13X_ACCEL_SLOW_COMP_X
 *        1                   |   SMA13X_ACCEL_SLOW_COMP_Y
 *        2                   |   SMA13X_ACCEL_SLOW_COMP_Z
 *
 *  @param slow_comp_u8: The value of slow compensation enable
 *     slow_comp_u8         |    result
 *  ------------------------- |------------------
 *         0x00               |    Disable
 *        0x01                |    Enable
 *
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
 *
 */
SMA13X_RETURN_FUNCTION_TYPE sma13x_get_slow_comp(u8 channel_u8,
u8 *slow_comp_u8)
{
	u8 data_u8 = SMA13X_INIT_VALUE;
	/*  Variable used to return value of
	communication routine*/
	SMA13X_RETURN_FUNCTION_TYPE com_rslt = ERROR;

	if (p_sma13x == SMA13X_NULL) {
		/* Check the struct p_sma13x is empty */
		return E_SMA13X_NULL_PTR;
		} else {
		switch (channel_u8) {
		case SMA13X_SLOW_COMP_X:
			/*SLOW COMP X*/
			com_rslt = p_sma13x->SMA13X_BUS_READ_FUNC
			(p_sma13x->dev_addr,
			SMA13X_ENABLE_SLOW_COMP_X_REG,
			&data_u8, SMA13X_GEN_READ_WRITE_LENGTH);
			*slow_comp_u8 = SMA13X_GET_BITSLICE
			(data_u8, SMA13X_ENABLE_SLOW_COMP_X);
		break;
		case SMA13X_SLOW_COMP_Y:
			/*SLOW COMP Y*/
			com_rslt = p_sma13x->SMA13X_BUS_READ_FUNC
			(p_sma13x->dev_addr,
			SMA13X_ENABLE_SLOW_COMP_Y_REG,
			&data_u8, SMA13X_GEN_READ_WRITE_LENGTH);
			*slow_comp_u8 = SMA13X_GET_BITSLICE
			(data_u8, SMA13X_ENABLE_SLOW_COMP_Y);
		break;
		case SMA13X_SLOW_COMP_Z:
			/*SLOW COMP Z*/
			com_rslt = p_sma13x->SMA13X_BUS_READ_FUNC
			(p_sma13x->dev_addr,
			SMA13X_ENABLE_SLOW_COMP_Z_REG,
			&data_u8, SMA13X_GEN_READ_WRITE_LENGTH);
			*slow_comp_u8 = SMA13X_GET_BITSLICE
			(data_u8, SMA13X_ENABLE_SLOW_COMP_Z);
		break;
		default:
			com_rslt = E_OUT_OF_RANGE;
		break;
		}
	}
	return com_rslt;
}
/*!
 *	@brief This API is used to set
 *	slow compensation(hp_x_enable, hp_y_enable and hp_z_enable) enable
 *	in the register 0x36 bit 0 to 2
 *	@note SLOW_COMP_X -> bit 0
 *	@note SLOW_COMP_Y -> bit 1
 *	@note SLOW_COMP_Z -> bit 2
 *
 *
 *	@param channel_u8: The value of slow compensation selection
 *     channel_u8           |    result
 *  ------------------------- |------------------
 *        0                   |   SMA13X_ACCEL_SLOW_COMP_X
 *        1                   |   SMA13X_ACCEL_SLOW_COMP_Y
 *        2                   |   SMA13X_ACCEL_SLOW_COMP_Z
 *
 *  @param slow_comp_u8: The value of slow compensation enable
 *     slow_comp_u8         |    result
 *  ------------------------- |------------------
 *         0x00               |    Disable
 *        0x01                |    Enable
 *
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
 *
 */
SMA13X_RETURN_FUNCTION_TYPE sma13x_set_slow_comp(u8 channel_u8,
u8 slow_comp_u8)
{
	u8 data_u8 = SMA13X_INIT_VALUE;
		/*  Variable used to return value of
	communication routine*/
	SMA13X_RETURN_FUNCTION_TYPE com_rslt = ERROR;

	if (p_sma13x == SMA13X_NULL) {
		/* Check the struct p_sma13x is empty */
		return E_SMA13X_NULL_PTR;
		} else {
		switch (channel_u8) {
		case SMA13X_SLOW_COMP_X:
			/*SLOW COMP X*/
			com_rslt = p_sma13x->SMA13X_BUS_READ_FUNC
			(p_sma13x->dev_addr,
			SMA13X_ENABLE_SLOW_COMP_X_REG,
			&data_u8, SMA13X_GEN_READ_WRITE_LENGTH);
			data_u8 = SMA13X_SET_BITSLICE
			(data_u8,
			SMA13X_ENABLE_SLOW_COMP_X, slow_comp_u8);
			com_rslt += sma13x_write_reg(
			SMA13X_ENABLE_SLOW_COMP_X_REG,
			&data_u8, SMA13X_GEN_READ_WRITE_LENGTH);
		break;
		case SMA13X_SLOW_COMP_Y:
			/*SLOW COMP Y*/
			com_rslt = p_sma13x->SMA13X_BUS_READ_FUNC
			(p_sma13x->dev_addr,
			SMA13X_ENABLE_SLOW_COMP_Y_REG,
			&data_u8, SMA13X_GEN_READ_WRITE_LENGTH);
			data_u8 = SMA13X_SET_BITSLICE
			(data_u8,
			SMA13X_ENABLE_SLOW_COMP_Y, slow_comp_u8);
			com_rslt += sma13x_write_reg(
			SMA13X_ENABLE_SLOW_COMP_Y_REG,
			&data_u8, SMA13X_GEN_READ_WRITE_LENGTH);
		break;
		case SMA13X_SLOW_COMP_Z:
			/*SLOW COMP Z*/
			com_rslt = p_sma13x->SMA13X_BUS_READ_FUNC
			(p_sma13x->dev_addr,
			SMA13X_ENABLE_SLOW_COMP_Z_REG,
			&data_u8, SMA13X_GEN_READ_WRITE_LENGTH);
			data_u8 = SMA13X_SET_BITSLICE
			(data_u8,
			SMA13X_ENABLE_SLOW_COMP_Z, slow_comp_u8);
			com_rslt += sma13x_write_reg(
			SMA13X_ENABLE_SLOW_COMP_Z_REG,
			&data_u8, SMA13X_GEN_READ_WRITE_LENGTH);
		break;
		default:
			com_rslt = E_OUT_OF_RANGE;
		break;
		}
	}
	return com_rslt;
}
/*!
 *	@brief This API is used to get
 *	the status of fast offset compensation(cal_rdy) in the register 0x36
 *	bit 4(Read Only Possible)
 *
 *
 *
 *  @param  cal_rdy_u8: The value of cal_ready
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
SMA13X_RETURN_FUNCTION_TYPE sma13x_get_cal_rdy(u8 *cal_rdy_u8)
{
	u8 data_u8 = SMA13X_INIT_VALUE;
	/*  Variable used to return value of
	communication routine*/
	SMA13X_RETURN_FUNCTION_TYPE com_rslt = ERROR;

	if (p_sma13x == SMA13X_NULL) {
		/* Check the struct p_sma13x is empty */
		return E_SMA13X_NULL_PTR;
		} else {
		com_rslt = p_sma13x->SMA13X_BUS_READ_FUNC
		(p_sma13x->dev_addr,
		SMA13X_FAST_CAL_RDY_STAT_REG,
		&data_u8, SMA13X_GEN_READ_WRITE_LENGTH);
		*cal_rdy_u8 = SMA13X_GET_BITSLICE(data_u8,
		SMA13X_FAST_CAL_RDY_STAT);
	}
	return com_rslt;
}
/*!
 *	@brief This API is used to set
 *	the status of fast offset compensation(cal_rdy) in the register 0x36
 *	bit 4(Read Only Possible)
 *
 *
 *
 *  @param  cal_trigger_u8: The value of cal_ready
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
SMA13X_RETURN_FUNCTION_TYPE sma13x_set_cal_trigger(u8 cal_trigger_u8)
{
	u8 data_u8 = SMA13X_INIT_VALUE;
	/*  Variable used to return value of
	communication routine*/
	SMA13X_RETURN_FUNCTION_TYPE com_rslt = ERROR;

	if (p_sma13x == SMA13X_NULL) {
		/* Check the struct p_sma13x is empty */
		return E_SMA13X_NULL_PTR;
		} else {
			com_rslt = p_sma13x->SMA13X_BUS_READ_FUNC
			(p_sma13x->dev_addr,
			SMA13X_CAL_TRIGGER_REG, &data_u8,
			SMA13X_GEN_READ_WRITE_LENGTH);
			data_u8 = SMA13X_SET_BITSLICE(data_u8,
			SMA13X_CAL_TRIGGER, cal_trigger_u8);
			com_rslt += sma13x_write_reg(
			SMA13X_CAL_TRIGGER_REG,
			&data_u8, SMA13X_GEN_READ_WRITE_LENGTH);
		}
	return com_rslt;
}
/*!
 *	@brief This API is used to set
 *	the offset reset(offset_reset) in the register 0x36
 *	bit 7(Write only possible)
 *
 *
 *
 *  @param  offset_rst_u8: The offset reset value
 *
 *
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
 *
 */
SMA13X_RETURN_FUNCTION_TYPE sma13x_set_offset_rst(u8 offset_rst_u8)
{
	u8 data_u8 = SMA13X_INIT_VALUE;
	/*  Variable used to return value of
	communication routine*/
	SMA13X_RETURN_FUNCTION_TYPE com_rslt = ERROR;

	if (p_sma13x == SMA13X_NULL) {
		/* Check the struct p_sma13x is empty */
		return E_SMA13X_NULL_PTR;
		} else {
			com_rslt = p_sma13x->SMA13X_BUS_READ_FUNC
			(p_sma13x->dev_addr,
			SMA13X_RST_OFFSET_REG,
			&data_u8, SMA13X_GEN_READ_WRITE_LENGTH);
			data_u8 = SMA13X_SET_BITSLICE
			(data_u8, SMA13X_RST_OFFSET,
			offset_rst_u8);
			com_rslt += sma13x_write_reg(
			SMA13X_RST_OFFSET_REG,
			&data_u8, SMA13X_GEN_READ_WRITE_LENGTH);
		}
	return com_rslt;
}
/*!
 *	@brief This API is used to get the status of offset
 *	(offset_x, offset_y and offset_z) in the registers 0x38,0x39 and 0x3A
 *	@note offset_x -> register 0x38 bit 0 to 7
 *	@note offset_y -> register 0x39 bit 0 to 7
 *	@note offset_z -> register 0x3A bit 0 to 7
 *
 *
 *  @param channel_u8: The value of offset selection
 *     channel_u8           |    result
 *  ------------------------- |------------------
 *        0                   |   SMA13X_ACCEL_X_AXIS
 *        1                   |   SMA13X_ACCEL_Y_AXIS
 *        2                   |   SMA13X_ACCEL_Z_AXIS
 *
 *  @param offset_u8: The value of offset
 *
 *
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
 *
 */
SMA13X_RETURN_FUNCTION_TYPE sma13x_get_offset(u8 channel_u8,
s8 *offset_u8)
{
	u8 data_u8 = SMA13X_INIT_VALUE;
	/*  Variable used to return value of
	communication routine*/
	SMA13X_RETURN_FUNCTION_TYPE com_rslt = ERROR;

	if (p_sma13x == SMA13X_NULL) {
		/* Check the struct p_sma13x is empty */
		return E_SMA13X_NULL_PTR;
		} else {
		switch (channel_u8) {
		case SMA13X_X_AXIS:
			com_rslt = p_sma13x->SMA13X_BUS_READ_FUNC
			(p_sma13x->dev_addr,
			SMA13X_OFFSET_X_AXIS_ADDR, &data_u8,
			SMA13X_GEN_READ_WRITE_LENGTH);
			*offset_u8 = (s8)data_u8;
		break;
		case SMA13X_Y_AXIS:
			com_rslt = p_sma13x->SMA13X_BUS_READ_FUNC
			(p_sma13x->dev_addr,
			SMA13X_OFFSET_Y_AXIS_ADDR, &data_u8,
			SMA13X_GEN_READ_WRITE_LENGTH);
			*offset_u8 = (s8)data_u8;
		break;
		case SMA13X_Z_AXIS:
			com_rslt = p_sma13x->SMA13X_BUS_READ_FUNC
			(p_sma13x->dev_addr,
			SMA13X_OFFSET_Z_AXIS_ADDR, &data_u8,
			SMA13X_GEN_READ_WRITE_LENGTH);
			*offset_u8 = (s8)data_u8;
		break;
		default:
			com_rslt = E_OUT_OF_RANGE;
		break;
		}
	}
	return com_rslt;
}
/*!
 *	@brief This API is used to set the status of offset
 *	(offset_x, offset_y and offset_z) in the registers 0x38,0x39 and 0x3A
 *	@note offset_x -> register 0x38 bit 0 to 7
 *	@note offset_y -> register 0x39 bit 0 to 7
 *	@note offset_z -> register 0x3A bit 0 to 7
 *
 *
 *  @param channel_u8: The value of offset selection
 *     channel_u8           |    result
 *  ------------------------- |------------------
 *        0                   |   SMA13X_ACCEL_X_AXIS
 *        1                   |   SMA13X_ACCEL_Y_AXIS
 *        2                   |   SMA13X_ACCEL_Z_AXIS
 *
 *  @param offset_u8: The value of offset
 *
 *
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
 *
 */
SMA13X_RETURN_FUNCTION_TYPE sma13x_set_offset(u8 channel_u8,
s8 offset_u8)
{
	u8 data_u8 = SMA13X_INIT_VALUE;
	/*  Variable used to return value of
	communication routine*/
	SMA13X_RETURN_FUNCTION_TYPE com_rslt = ERROR;

	if (p_sma13x == SMA13X_NULL) {
		/* Check the struct p_sma13x is empty */
		return E_SMA13X_NULL_PTR;
		} else {
		switch (channel_u8) {
		case SMA13X_X_AXIS:
			data_u8 = offset_u8;
			com_rslt = sma13x_write_reg(
			SMA13X_OFFSET_X_AXIS_ADDR, &data_u8,
			SMA13X_GEN_READ_WRITE_LENGTH);
		break;
		case SMA13X_Y_AXIS:
			data_u8 = offset_u8;
			com_rslt = sma13x_write_reg(
			SMA13X_OFFSET_Y_AXIS_ADDR, &data_u8,
			SMA13X_GEN_READ_WRITE_LENGTH);
		break;
		case SMA13X_Z_AXIS:
			data_u8 = offset_u8;
			com_rslt = sma13x_write_reg(
			SMA13X_OFFSET_Z_AXIS_ADDR, &data_u8,
			SMA13X_GEN_READ_WRITE_LENGTH);
		break;
		default:
			com_rslt = E_OUT_OF_RANGE;
		break;
		}
	}
	return com_rslt;
}
