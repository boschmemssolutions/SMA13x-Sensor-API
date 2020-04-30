/* SPDX-License-Identifier: BSD-3-Clause OR GPL-2.0 */
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

#ifndef __SMA13X_H__
#define __SMA13X_H__
/****************************************************************/
/**\name	DATA TYPES INCLUDES		*/
/************************************************************/
/*!
* @brief The following definition uses for define the data types
*
* @note While porting the API please consider the following
* @note Please check the version of C standard
* @note Are you using Linux platform
*/

/*!
* @brief For the Linux platform support
* Please use the types.h for your data types definitions
*/
#ifdef	__KERNEL__

#include <linux/types.h>
/* singed integer type*/
typedef	int8_t s8;/**< used for signed 8bit */
typedef	int16_t s16;/**< used for signed 16bit */
typedef	int32_t s32;/**< used for signed 32bit */
typedef	int64_t s64;/**< used for signed 64bit */

typedef	u_int8_t u8;/**< used for unsigned 8bit */
typedef	u_int16_t u16;/**< used for unsigned 16bit */
typedef	u_int32_t u32;/**< used for unsigned 32bit */
typedef	u_int64_t u64;/**< used for unsigned 64bit */



#else /* ! __KERNEL__ */
/**********************************************************
* These definition uses for define the C
* standard version data types
***********************************************************/
# if !defined(__STDC_VERSION__)

/************************************************
 * compiler is C11 C standard
************************************************/
#if (__STDC_VERSION__ == 201112L)

/************************************************/
#include <stdint.h>
/************************************************/

/*unsigned integer types*/
typedef	uint8_t u8;/**< used for unsigned 8bit */
typedef	uint16_t u16;/**< used for unsigned 16bit */
typedef	uint32_t u32;/**< used for unsigned 32bit */
typedef	uint64_t u64;/**< used for unsigned 64bit */

/*signed integer types*/
typedef	int8_t s8;/**< used for signed 8bit */
typedef	int16_t s16;/**< used for signed 16bit */
typedef	int32_t s32;/**< used for signed 32bit */
typedef	int64_t s64;/**< used for signed 64bit */
/************************************************
 * compiler is C99 C standard
************************************************/

#elif (__STDC_VERSION__ == 199901L)

/* stdint.h is a C99 supported c library.
which is used to fixed the integer size*/
/************************************************/
#include <stdint.h>
/************************************************/

/*unsigned integer types*/
typedef	uint8_t u8;/**< used for unsigned 8bit */
typedef	uint16_t u16;/**< used for unsigned 16bit */
typedef	uint32_t u32;/**< used for unsigned 32bit */
typedef	uint64_t u64;/**< used for unsigned 64bit */

/*signed integer types*/
typedef int8_t s8;/**< used for signed 8bit */
typedef	int16_t s16;/**< used for signed 16bit */
typedef	int32_t s32;/**< used for signed 32bit */
typedef	int64_t s64;/**< used for signed 64bit */
/************************************************
 * compiler is C89 or other C standard
************************************************/

#else /*  !defined(__STDC_VERSION__) */
/*!
* @brief By default it is defined as 32 bit machine configuration
*	define your data types based on your
*	machine/compiler/controller configuration
*/
#define  MACHINE_32_BIT

/*! @brief
 *	If your machine support 16 bit
 *	define the MACHINE_16_BIT
 */
#ifdef MACHINE_16_BIT
#include <limits.h>
/*signed integer types*/
typedef	signed char  s8;/**< used for signed 8bit */
typedef	signed short int s16;/**< used for signed 16bit */
typedef	signed long int s32;/**< used for signed 32bit */

#if defined(LONG_MAX) && LONG_MAX == 0x7fffffffffffffffL
typedef long int s64;/**< used for signed 64bit */
typedef unsigned long int u64;/**< used for unsigned 64bit */
#elif defined(LLONG_MAX) && (LLONG_MAX == 0x7fffffffffffffffLL)
typedef long long int s64;/**< used for signed 64bit */
typedef unsigned long long int u64;/**< used for unsigned 64bit */
#else
#warning Either the correct data type for signed 64 bit integer \
could not be found, or 64 bit integers are not supported in your environment.
#warning If 64 bit integers are supported on your platform, \
please set s64 manually.
#endif

/*unsigned integer types*/
typedef	unsigned char u8;/**< used for unsigned 8bit */
typedef	unsigned short int u16;/**< used for unsigned 16bit */
typedef	unsigned long int u32;/**< used for unsigned 32bit */

/* If your machine support 32 bit
define the MACHINE_32_BIT*/
#elif defined MACHINE_32_BIT
/*signed integer types*/
typedef	signed char  s8;/**< used for signed 8bit */
typedef	signed short int s16;/**< used for signed 16bit */
typedef	signed int s32;/**< used for signed 32bit */
typedef	signed long long int s64;/**< used for signed 64bit */

/*unsigned integer types*/
typedef	unsigned char u8;/**< used for unsigned 8bit */
typedef	unsigned short int u16;/**< used for unsigned 16bit */
typedef	unsigned int u32;/**< used for unsigned 32bit */
typedef	unsigned long long int u64;/**< used for unsigned 64bit */

/* If your machine support 64 bit
define the MACHINE_64_BIT*/
#elif defined MACHINE_64_BIT
/*signed integer types*/
typedef	signed char  s8;/**< used for signed 8bit */
typedef	signed short int s16;/**< used for signed 16bit */
typedef	signed int s32;/**< used for signed 32bit */
typedef	signed long int s64;/**< used for signed 64bit */

/*unsigned integer types*/
typedef	unsigned char u8;/**< used for unsigned 8bit */
typedef	unsigned short int u16;/**< used for unsigned 16bit */
typedef	unsigned int u32;/**< used for unsigned 32bit */
typedef	unsigned long int u64;/**< used for unsigned 64bit */

#else
#warning The data types defined above which not supported \
define the data types manually
#endif
#endif

/*** This else will execute for the compilers
 *	which are not supported the C standards
 *	Like C89/C99/C11***/
#else
/*!
* @brief By default it is defined as 32 bit machine configuration
*	define your data types based on your
*	machine/compiler/controller configuration
*/
#define  MACHINE_32_BIT

/* If your machine support 16 bit
define the MACHINE_16_BIT*/
#ifdef MACHINE_16_BIT
#include <limits.h>
/*signed integer types*/
typedef	signed char  s8;/**< used for signed 8bit */
typedef	signed short int s16;/**< used for signed 16bit */
typedef	signed long int s32;/**< used for signed 32bit */

#if defined(LONG_MAX) && LONG_MAX == 0x7fffffffffffffffL
typedef long int s64;/**< used for signed 64bit */
typedef unsigned long int u64;/**< used for unsigned 64bit */
#elif defined(LLONG_MAX) && (LLONG_MAX == 0x7fffffffffffffffLL)
typedef long long int s64;/**< used for signed 64bit */
typedef unsigned long long int u64;/**< used for unsigned 64bit */
#else
#warning Either the correct data type for signed 64 bit integer \
could not be found, or 64 bit integers are not supported in your environment.
#warning If 64 bit integers are supported on your platform, \
please set s64 manually.
#endif

/*unsigned integer types*/
typedef	unsigned char u8;/**< used for unsigned 8bit */
typedef	unsigned short int u16;/**< used for unsigned 16bit */
typedef	unsigned long int u32;/**< used for unsigned 32bit */

/*! @brief If your machine support 32 bit
define the MACHINE_32_BIT*/
#elif defined MACHINE_32_BIT
/*signed integer types*/
typedef	signed char  s8;/**< used for signed 8bit */
typedef	signed short int s16;/**< used for signed 16bit */
typedef	signed int s32;/**< used for signed 32bit */
typedef	signed long long int s64;/**< used for signed 64bit */

/*unsigned integer types*/
typedef	unsigned char u8;/**< used for unsigned 8bit */
typedef	unsigned short int u16;/**< used for unsigned 16bit */
typedef	unsigned int u32;/**< used for unsigned 32bit */
typedef	unsigned long long int u64;/**< used for unsigned 64bit */

/* If your machine support 64 bit
define the MACHINE_64_BIT*/
#elif defined MACHINE_64_BIT
/*signed integer types*/
typedef	signed char  s8;/**< used for signed 8bit */
typedef	signed short int s16;/**< used for signed 16bit */
typedef	signed int s32;/**< used for signed 32bit */
typedef	signed long int s64;/**< used for signed 64bit */

/*unsigned integer types*/
typedef	unsigned char u8;/**< used for unsigned 8bit */
typedef	unsigned short int u16;/**< used for unsigned 16bit */
typedef	unsigned int u32;/**< used for unsigned 32bit */
typedef	unsigned long int u64;/**< used for unsigned 64bit */

#else
#warning The data types defined above which not supported \
define the data types manually
#endif
#endif
#endif

/***************************************************************/
/**\name	BUS READ AND WRITE FUNCTION POINTERS        */
/***************************************************************/
/*!
* @brief Define the calling convention of YOUR bus communication routine.
* @note This includes types of parameters.
* This example shows the configuration for an SPI bus link.

* If your communication function looks like this:

* write_my_bus_xy(u8 device_addr,
* u8 register_addr, u8 * data, u8 length);

* The SMA13X_WR_FUNC_PTR would equal:

* SMA13X_WR_FUNC_PTR char
* (* bus_write)(u8, u8, u8 *, u8)

* Parameters can be mixed as needed refer to
* the \ref SMA13X_BUS_WRITE_FUNC  macro.
*/
#define SMA13X_WR_FUNC_PTR s8(*bus_write)\
(u8, u8, u8 *, u8)

/*!
* @brief link macro between API function calls and bus write function
* @note The bus write function can change since
* this is a system dependant issue.

* If the bus_write parameter calling order is like:
* reg_addr, reg_data, wr_len it would be as it is here.

* If the parameters are differently ordered or your communication function
* like I2C need to know the device address,
* you can change this macro accordingly.


* define SMA13X_BUS_WRITE_FUNC(dev_addr, reg_addr, reg_data, wr_len)\
* bus_write(dev_addr, reg_addr, reg_data, wr_len)

* This macro lets all API functions call YOUR communication routine in
* a way that equals your definition in the
* ref SMA13X_WR_FUNC_PTR definition.

*/
#define SMA13X_BUS_WRITE_FUNC(dev_addr, reg_addr, reg_data, wr_len)\
bus_write(dev_addr, reg_addr, reg_data, wr_len)


/*!
* @brief Define the calling convention of YOUR bus communication routine.
* @note This includes types of parameters. This example
*shows the configuration for an SPI bus link.

*If your communication function looks like this:

*read_my_bus_xy(u8 device_addr,
*u8 register_addr, u8* data, u8 length);

*The SMA13X_RD_FUNC_PTR would equal:

*SMA13X_RD_FUNC_PTR s8
*(* bus_read)(u8, u8, u8*, u8)

*Parameters can be mixed as needed refer to the
 ref SMA13X_BUS_READ_FUNC  macro.
*/

#define SMA13X_SPI_RD_MASK 0x80
/* for spi read transactions on SPI the MSB has to be set */
#define SMA13X_RD_FUNC_PTR s8(*bus_read)\
(u8, u8, u8 *, u8)
#define SMA13X_BRD_FUNC_PTR s8(*burst_read)\
(u8, u8, u8 *, u32)

/*!
* @brief link macro between API function calls and bus read function
* @note The bus write function can change since
* this is a system dependant issue.

* If the bus_read parameter calling order is like:
* reg_addr, reg_data, wr_len it would be as it is here.

*  If the parameters are differently ordered or your
*  communication function like I2C need to know the device address,
*  you can change this macro accordingly.


*  define SMA13X_BUS_READ_FUNC(dev_addr, reg_addr, reg_data, wr_len)\
* bus_read(dev_addr, reg_addr, reg_data, wr_len)

* This macro lets all API functions call YOUR
* communication routine in a way that equals your definition in the
* ref SMA13X_WR_FUNC_PTR definition.

* @note: this macro also includes the "MSB='1'" for reading SMA13X addresses.
*/



#define SMA13X_BUS_READ_FUNC(dev_addr, reg_addr, reg_data, r_len)\
bus_read(dev_addr, reg_addr, reg_data, r_len)
#define SMA13X_BURST_READ_FUNC(device_addr,\
register_addr, register_data, rd_len)\
burst_read(device_addr, register_addr, register_data, rd_len)
/**************************************************************/
/**\name	I2C ADDRESS DEFINITIONS    */
/**************************************************************/
#define SMA13X_I2C_ADDR1                (0x18)
#define SMA13X_I2C_ADDR2                (0x19)

#define SMA13X_I2C_ADDR3                (0x10)
#define SMA13X_I2C_ADDR4                (0x11)

/**************************************************************/
/**\name	CONSTANTS DEFINITION    */
/**************************************************************/
#define         SMA13X_INIT_VALUE                       ((u8)0)
#define         SMA13X_GEN_READ_WRITE_LENGTH            ((u8)1)
#define         SMA13X_INTERFACE_IDLE_TIME_DELAY	((u8)1)
#define         SMA13X_LSB_MSB_READ_LENGTH		((u8)2)
	/**	BIT SHIFT DEFINITIONS    */
#define         SMA13X_SHIFT_TWO_BITS                   ((u8)2)
#define         SMA13X_SHIFT_FOUR_BITS                  ((u8)4)
#define         SMA13X_SHIFT_FIVE_BITS                  ((u8)5)
#define         SMA13X_SHIFT_SIX_BITS                   ((u8)6)
#define         SMA13X_SHIFT_EIGHT_BITS                 ((u8)8)
	/**	MODE RANGES    */
#define         SMA13X_ACCEL_BW_MIN_RANGE               ((u8)7)
#define         SMA13X_ACCEL_BW_1000HZ_RANGE            ((u8)15)
#define         SMA13X_ACCEL_BW_MAX_RANGE               ((u8)16)
#define         SMA13X_SLEEP_DURN_MIN_RANGE		((u8)4)
#define         SMA13X_SLEEP_TIMER_MODE_RANGE		((u8)2)
#define         SMA13X_SLEEP_DURN_MAX_RANGE		((u8)16)
#define         SMA13X_POWER_MODE_RANGE			((u8)6)
#define         SMA13X_SELF_TEST_AXIS_RANGE		((u8)4)
#define         SMA13X_SELF_TEST_SIGN_RANGE		((u8)2)

/**************************************************************/
/**\name	ERROR CODE DEFINITIONS    */
/**************************************************************/
#define E_OUT_OF_RANGE          ((s8)-2)
#define E_SMA13X_NULL_PTR       ((s8)-127)
#define SMA13X_NULL             ((void *)0)
#define ERROR                   ((s8)-1)
#define SUCCESS                 ((u8)0)
/**************************************************************/
/**\name	RETURN TYPE DEFINITION    */
/**************************************************************/
#define SMA13X_RETURN_FUNCTION_TYPE        s8
/**< This refers SMA13X return type as char */

/**************************************************************/
/**\name	REGISTER ADDRESS DEFINITIONS    */
/**************************************************************/
#define SMA13X_EEP_OFFSET                       (0x16)
#define SMA13X_IMAGE_BASE                       (0x38)
#define SMA13X_IMAGE_LEN                        (22)
#define SMA13X_CHIP_ID_ADDR                     (0x00)
/** DATA ADDRESS DEFINITIONS */
#define SMA13X_X_AXIS_LSB_ADDR                  (0x02)
#define SMA13X_X_AXIS_MSB_ADDR                  (0x03)
#define SMA13X_Y_AXIS_LSB_ADDR                  (0x04)
#define SMA13X_Y_AXIS_MSB_ADDR                  (0x05)
#define SMA13X_Z_AXIS_LSB_ADDR                  (0x06)
#define SMA13X_Z_AXIS_MSB_ADDR                  (0x07)
/**STATUS ADDRESS DEFINITIONS */
#define SMA13X_STAT1_ADDR                       (0x09)
#define SMA13X_STAT2_ADDR                       (0x0A)
#define SMA13X_STAT_SLOPE_ADDR                  (0x0B)
#define SMA13X_STAT_HIGH_G_ADDR                   (0x0C)
/**STATUS ADDRESS DEFINITIONS */
#define SMA13X_RANGE_SELECT_ADDR                (0x0F)
#define SMA13X_BW_SELECT_ADDR                   (0x10)
#define SMA13X_MODE_CTRL_ADDR                   (0x11)
#define SMA13X_LOW_NOISE_CTRL_ADDR              (0x12)
#define SMA13X_DATA_CTRL_ADDR                   (0x13)
#define SMA13X_RST_ADDR                         (0x14)
/**INTERUPT ADDRESS DEFINITIONS */
#define SMA13X_INTR_ENABLE1_ADDR                (0x16)
#define SMA13X_INTR_ENABLE2_ADDR                (0x17)
#define SMA13X_INTR_SLOW_NO_MOTION_ADDR         (0x18)
#define SMA13X_INTR1_PAD_SELECT_ADDR            (0x19)
#define SMA13X_INTR_DATA_SELECT_ADDR            (0x1A)
#define SMA13X_INTR_SOURCE_ADDR                 (0x1E)
#define SMA13X_INTR_SET_ADDR                    (0x20)
#define SMA13X_INTR_CTRL_ADDR                   (0x21)
/** FEATURE ADDRESS DEFINITIONS */
#define SMA13X_LOW_HIGH_HYST_ADDR                (0x24)
#define SMA13X_HIGH_DURN_ADDR                    (0x25)
#define SMA13X_HIGH_THRES_ADDR                   (0x26)
#define SMA13X_SLOPE_DURN_ADDR                   (0x27)
#define SMA13X_SLOPE_THRES_ADDR                  (0x28)
#define SMA13X_SLOW_NO_MOTION_THRES_ADDR         (0x29)
#define SMA13X_SELFTEST_ADDR                     (0x32)
#define SMA13X_SERIAL_CTRL_ADDR                  (0x34)
/**OFFSET ADDRESS DEFINITIONS */
#define SMA13X_OFFSET_CTRL_ADDR                  (0x36)
#define SMA13X_OFFSET_X_AXIS_ADDR                (0x38)
#define SMA13X_OFFSET_Y_AXIS_ADDR                (0x39)
#define SMA13X_OFFSET_Z_AXIS_ADDR                (0x3A)

#define SMA13X_REG_NUM SMA13X_OFFSET_Z_AXIS_ADDR

/**************************************************************/
/**\name	ACCEL RESOLUTION DEFINITION   */
/**************************************************************/
#define SMA13X_12_RESOLUTION                    (0)
#define SMA13X_10_RESOLUTION                    (1)
#define SMA13X_14_RESOLUTION                    (2)

/**************************************************************/
/**\name	ACCEL DELAY DEFINITION   */
/**************************************************************/
/* register write and read delays */
#define SMA13X_MDELAY_DATA_TYPE                 u32
#define SMA13X_EE_W_DELAY                       (28)

/**************************************************************/
/**\name	STRUCTURE DEFINITIONS    */
/**************************************************************/
/*!
*	@brief read accel xyz data for 10,14 and 12 bit resolution
*/
struct sma13x_accel_data {
s16 x,/**< accel x data 10,14 and 12 resolution*/
y,/**< accel y data 10,14 and 12 resolution*/
z;/**< accel z data 10,14 and 12 resolution*/
};

/*!
*	@brief read accel xyz data for 8 bit resolution
*/
struct  sma13x_accel_eight_resolution {
s8 x,/**< accel x data with eight bit resolution*/
y,/**< accel y data with eight bit resolution*/
z;/**< accel z data with eight bit resolution*/
};

/*!
 *	@brief sma13x initialization struct
 *	struct sma13x_t is used for assigning the following parameters.
 *
 *	Bus write function pointer: SMA13X_WR_FUNC_PTR
 *	Bus read function pointer: SMA13X_RD_FUNC_PTR
 *	Burst read function pointer: SMA13X_BRD_FUNC_PTR
 *	Delay function pointer: delay_msec
 *	I2C address: dev_addr
 *	Chip id of the sensor: chip_id
 */
struct sma13x_t {
	/*! save current sma13x operation mode */
	u8 power_mode_u8;
	/*! chip_id of sma13x */
	u8 chip_id;
	/*! the value of power mode register 0x11*/
	u8 ctrl_mode_reg;
	/*! the value of power mode register 0x12*/
	u8 low_mode_reg;
	/*! initializes sma13x's I2C device address*/
	u8 dev_addr;
	/*! function pointer to the SPI/I2C write function */
	SMA13X_WR_FUNC_PTR;
	/*! function pointer to the SPI/I2C read function */
	SMA13X_RD_FUNC_PTR;
	/*! function pointer to the SPI/I2C burst read function */
	SMA13X_BRD_FUNC_PTR;
	/*! delay(in ms) function pointer */
	void (*delay_msec)(SMA13X_MDELAY_DATA_TYPE);
};

/*********************************************************************/
/**\name REGISTER BIT MASK, BIT LENGTH, BIT POSITION DEFINITIONS  */
/********************************************************************/
/******************************/
/**\name CHIP ID  */
/******************************/
#define SMA13X_CHIP_ID_POS             (0)
#define SMA13X_CHIP_ID_MSK             (0xFF)
#define SMA13X_CHIP_ID_LEN             (8)
#define SMA13X_CHIP_ID_REG             SMA13X_CHIP_ID_ADDR

/******************************/
/**\name DATA REGISTER-X  */
/******************************/
#define SMA13X_NEW_DATA_X_POS          (0)
#define SMA13X_NEW_DATA_X_LEN          (1)
#define SMA13X_NEW_DATA_X_MSK          (0x01)
#define SMA13X_NEW_DATA_X_REG          SMA13X_X_AXIS_LSB_ADDR

#define SMA13X_ACCEL_X14_LSB_POS          (2)
#define SMA13X_ACCEL_X14_LSB_LEN          (6)
#define SMA13X_ACCEL_X14_LSB_MSK          (0xFC)
#define SMA13X_ACCEL_X14_LSB_REG           SMA13X_X_AXIS_LSB_ADDR

#define SMA13X_ACCEL_X12_LSB_POS           (4)
#define SMA13X_ACCEL_X12_LSB_LEN           (4)
#define SMA13X_ACCEL_X12_LSB_MSK           (0xF0)
#define SMA13X_ACCEL_X12_LSB_REG           SMA13X_X_AXIS_LSB_ADDR

#define SMA13X_ACCEL_X10_LSB_POS           (6)
#define SMA13X_ACCEL_X10_LSB_LEN           (2)
#define SMA13X_ACCEL_X10_LSB_MSK           (0xC0)
#define SMA13X_ACCEL_X10_LSB_REG           SMA13X_X_AXIS_LSB_ADDR

#define SMA13X_ACCEL_X8_LSB_POS           (0)
#define SMA13X_ACCEL_X8_LSB_LEN           (0)
#define SMA13X_ACCEL_X8_LSB_MSK           (0x00)
#define SMA13X_ACCEL_X8_LSB_REG           SMA13X_X_AXIS_LSB_ADDR

#define SMA13X_ACCEL_X_MSB_POS           (0)
#define SMA13X_ACCEL_X_MSB_LEN           (8)
#define SMA13X_ACCEL_X_MSB_MSK           (0xFF)
#define SMA13X_ACCEL_X_MSB_REG           SMA13X_X_AXIS_MSB_ADDR
/******************************/
/**\name DATA REGISTER-Y  */
/******************************/
#define SMA13X_NEW_DATA_Y_POS          (0)
#define SMA13X_NEW_DATA_Y_LEN          (1)
#define SMA13X_NEW_DATA_Y_MSK          (0x01)
#define SMA13X_NEW_DATA_Y_REG          SMA13X_Y_AXIS_LSB_ADDR

#define SMA13X_ACCEL_Y14_LSB_POS           (2)
#define SMA13X_ACCEL_Y14_LSB_LEN           (6)
#define SMA13X_ACCEL_Y14_LSB_MSK           (0xFC)
#define SMA13X_ACCEL_Y14_LSB_REG           SMA13X_Y_AXIS_LSB_ADDR

#define SMA13X_ACCEL_Y12_LSB_POS           (4)
#define SMA13X_ACCEL_Y12_LSB_LEN           (4)
#define SMA13X_ACCEL_Y12_LSB_MSK           (0xF0)
#define SMA13X_ACCEL_Y12_LSB_REG           SMA13X_Y_AXIS_LSB_ADDR

#define SMA13X_ACCEL_Y10_LSB_POS           (6)
#define SMA13X_ACCEL_Y10_LSB_LEN           (2)
#define SMA13X_ACCEL_Y10_LSB_MSK           (0xC0)
#define SMA13X_ACCEL_Y10_LSB_REG           SMA13X_Y_AXIS_LSB_ADDR

#define SMA13X_ACCEL_Y8_LSB_POS           (0)
#define SMA13X_ACCEL_Y8_LSB_LEN           (0)
#define SMA13X_ACCEL_Y8_LSB_MSK           (0x00)
#define SMA13X_ACCEL_Y8_LSB_REG           SMA13X_Y_AXIS_LSB_ADDR

#define SMA13X_ACCEL_Y_MSB_POS           (0)
#define SMA13X_ACCEL_Y_MSB_LEN           (8)
#define SMA13X_ACCEL_Y_MSB_MSK           (0xFF)
#define SMA13X_ACCEL_Y_MSB_REG           SMA13X_Y_AXIS_MSB_ADDR
/******************************/
/**\name DATA REGISTER-Z  */
/******************************/
#define SMA13X_NEW_DATA_Z_POS          (0)
#define SMA13X_NEW_DATA_Z_LEN          (1)
#define SMA13X_NEW_DATA_Z_MSK          (0x01)
#define SMA13X_NEW_DATA_Z_REG          SMA13X_Z_AXIS_LSB_ADDR

#define SMA13X_ACCEL_Z14_LSB_POS           (2)
#define SMA13X_ACCEL_Z14_LSB_LEN           (6)
#define SMA13X_ACCEL_Z14_LSB_MSK           (0xFC)
#define SMA13X_ACCEL_Z14_LSB_REG           SMA13X_Z_AXIS_LSB_ADDR

#define SMA13X_ACCEL_Z12_LSB_POS           (4)
#define SMA13X_ACCEL_Z12_LSB_LEN           (4)
#define SMA13X_ACCEL_Z12_LSB_MSK           (0xF0)
#define SMA13X_ACCEL_Z12_LSB_REG           SMA13X_Z_AXIS_LSB_ADDR

#define SMA13X_ACCEL_Z10_LSB_POS           (6)
#define SMA13X_ACCEL_Z10_LSB_LEN           (2)
#define SMA13X_ACCEL_Z10_LSB_MSK           (0xC0)
#define SMA13X_ACCEL_Z10_LSB_REG           SMA13X_Z_AXIS_LSB_ADDR

#define SMA13X_ACCEL_Z8_LSB_POS           (0)
#define SMA13X_ACCEL_Z8_LSB_LEN           (0)
#define SMA13X_ACCEL_Z8_LSB_MSK           (0x00)
#define SMA13X_ACCEL_Z8_LSB_REG           SMA13X_Z_AXIS_LSB_ADDR

#define SMA13X_ACCEL_Z_MSB_POS           (0)
#define SMA13X_ACCEL_Z_MSB_LEN           (8)
#define SMA13X_ACCEL_Z_MSB_MSK           (0xFF)
#define SMA13X_ACCEL_Z_MSB_REG           SMA13X_Z_AXIS_MSB_ADDR

/***************************************/
/**\name INTERRUPT STATUS OF HIGH-G */
/**************************************/
#define SMA13X_HIGH_G_INTR_STAT_POS          (1)
#define SMA13X_HIGH_G_INTR_STAT_LEN          (1)
#define SMA13X_HIGH_G_INTR_STAT_MSK          (0x02)
#define SMA13X_HIGH_G_INTR_STAT_REG          SMA13X_STAT1_ADDR
/***************************************/
/**\name INTERRUPT STATUS OF SLOPE */
/**************************************/
#define SMA13X_SLOPE_INTR_STAT_POS          (2)
#define SMA13X_SLOPE_INTR_STAT_LEN          (1)
#define SMA13X_SLOPE_INTR_STAT_MSK          (0x04)
#define SMA13X_SLOPE_INTR_STAT_REG          SMA13X_STAT1_ADDR
/*******************************************/
/**\name INTERRUPT STATUS OF SLOW NO MOTION*/
/*******************************************/
#define SMA13X_SLOW_NO_MOTION_INTR_STAT_POS          (3)
#define SMA13X_SLOW_NO_MOTION_INTR_STAT_LEN          (1)
#define SMA13X_SLOW_NO_MOTION_INTR_STAT_MSK          (0x08)
#define SMA13X_SLOW_NO_MOTION_INTR_STAT_REG          SMA13X_STAT1_ADDR
/***************************************/
/**\name INTERRUPT STATUS OF DATA */
/**************************************/
#define SMA13X_DATA_INTR_STAT_POS           (7)
#define SMA13X_DATA_INTR_STAT_LEN           (1)
#define SMA13X_DATA_INTR_STAT_MSK           (0x80)
#define SMA13X_DATA_INTR_STAT_REG           SMA13X_STAT2_ADDR
/*********************************************/
/**\name INTERRUPT STATUS SLOPE XYZ AND SIGN */
/*********************************************/
#define SMA13X_SLOPE_FIRST_X_POS        (0)
#define SMA13X_SLOPE_FIRST_X_LEN        (1)
#define SMA13X_SLOPE_FIRST_X_MSK        (0x01)
#define SMA13X_SLOPE_FIRST_X_REG        SMA13X_STAT_SLOPE_ADDR

#define SMA13X_SLOPE_FIRST_Y_POS        (1)
#define SMA13X_SLOPE_FIRST_Y_LEN        (1)
#define SMA13X_SLOPE_FIRST_Y_MSK        (0x02)
#define SMA13X_SLOPE_FIRST_Y_REG        SMA13X_STAT_SLOPE_ADDR

#define SMA13X_SLOPE_FIRST_Z_POS        (2)
#define SMA13X_SLOPE_FIRST_Z_LEN        (1)
#define SMA13X_SLOPE_FIRST_Z_MSK        (0x04)
#define SMA13X_SLOPE_FIRST_Z_REG        SMA13X_STAT_SLOPE_ADDR

#define SMA13X_SLOPE_SIGN_STAT_POS         (3)
#define SMA13X_SLOPE_SIGN_STAT_LEN         (1)
#define SMA13X_SLOPE_SIGN_STAT_MSK         (0x08)
#define SMA13X_SLOPE_SIGN_STAT_REG         SMA13X_STAT_SLOPE_ADDR
/*********************************************/
/**\name INTERRUPT STATUS HIGH_G XYZ AND SIGN */
/*********************************************/
#define SMA13X_HIGH_G_FIRST_X_POS        (0)
#define SMA13X_HIGH_G_FIRST_X_LEN        (1)
#define SMA13X_HIGH_G_FIRST_X_MSK        (0x01)
#define SMA13X_HIGH_G_FIRST_X_REG        SMA13X_STAT_HIGH_ADDR

#define SMA13X_HIGH_G_FIRST_Y_POS        (1)
#define SMA13X_HIGH_G_FIRST_Y_LEN        (1)
#define SMA13X_HIGH_G_FIRST_Y_MSK        (0x02)
#define SMA13X_HIGH_G_FIRST_Y_REG        SMA13X_STAT_HIGH_ADDR

#define SMA13X_HIGH_G_FIRST_Z_POS        (2)
#define SMA13X_HIGH_G_FIRST_Z_LEN        (1)
#define SMA13X_HIGH_G_FIRST_Z_MSK        (0x04)
#define SMA13X_HIGH_G_FIRST_Z_REG        SMA13X_STAT_HIGH_ADDR

#define SMA13X_HIGH_G_SIGN_STAT_POS         (3)
#define SMA13X_HIGH_G_SIGN_STAT_LEN         (1)
#define SMA13X_HIGH_G_SIGN_STAT_MSK         (0x08)
#define SMA13X_HIGH_G_SIGN_STAT_REG         SMA13X_STAT_HIGH_ADDR

/****************************/
/**\name RANGE */
/****************************/
#define SMA13X_RANGE_SELECT_POS             (0)
#define SMA13X_RANGE_SELECT_LEN             (4)
#define SMA13X_RANGE_SELECT_MSK             (0x0F)
#define SMA13X_RANGE_SELECT_REG             SMA13X_RANGE_SELECT_ADDR
/****************************/
/**\name BANDWIDTH */
/****************************/
#define SMA13X_BW_POS             (0)
#define SMA13X_BW_LEN             (5)
#define SMA13X_BW_MSK             (0x1F)
#define SMA13X_BW_REG             SMA13X_BW_SELECT_ADDR
/****************************/
/**\name SLEEP DURATION */
/****************************/
#define SMA13X_SLEEP_DURN_POS             (1)
#define SMA13X_SLEEP_DURN_LEN             (4)
#define SMA13X_SLEEP_DURN_MSK             (0x1E)
#define SMA13X_SLEEP_DURN_REG             SMA13X_MODE_CTRL_ADDR
/****************************/
/**\name POWER MODEPOWER MODE */
/****************************/
#define SMA13X_MODE_CTRL_POS             (5)
#define SMA13X_MODE_CTRL_LEN             (3)
#define SMA13X_MODE_CTRL_MSK             (0xE0)
#define SMA13X_MODE_CTRL_REG             SMA13X_MODE_CTRL_ADDR
/****************************/
/**\name SLEEP TIMER */
/****************************/
#define SMA13X_SLEEP_TIMER_POS          (5)
#define SMA13X_SLEEP_TIMER_LEN          (1)
#define SMA13X_SLEEP_TIMER_MSK          (0x20)
#define SMA13X_SLEEP_TIMER_REG          SMA13X_LOW_NOISE_CTRL_ADDR
/****************************/
/**\name LOWPOWER MODE */
/****************************/
#define SMA13X_LOW_POWER_MODE_POS          (6)
#define SMA13X_LOW_POWER_MODE_LEN          (1)
#define SMA13X_LOW_POWER_MODE_MSK          (0x40)
#define SMA13X_LOW_POWER_MODE_REG          SMA13X_LOW_NOISE_CTRL_ADDR
/*******************************************/
/**\name DISABLE MSB SHADOWING PROCEDURE  */
/*******************************************/
#define SMA13X_DIS_SHADOW_PROC_POS       (6)
#define SMA13X_DIS_SHADOW_PROC_LEN       (1)
#define SMA13X_DIS_SHADOW_PROC_MSK       (0x40)
#define SMA13X_DIS_SHADOW_PROC_REG       SMA13X_DATA_CTRL_ADDR
/***************************************************/
/**\name FILTERED OR UNFILTERED ACCELERATION DATA   */
/***************************************************/
#define SMA13X_ENABLE_DATA_HIGH_BW_POS         (7)
#define SMA13X_ENABLE_DATA_HIGH_BW_LEN         (1)
#define SMA13X_ENABLE_DATA_HIGH_BW_MSK         (0x80)
#define SMA13X_ENABLE_DATA_HIGH_BW_REG         SMA13X_DATA_CTRL_ADDR
/***************************************************/
/**\name SOFT RESET VALUE   */
/***************************************************/
#define SMA13X_ENABLE_SOFT_RESET_VALUE        (0xB6)
/**********************************************/
/**\name INTERRUPT ENABLE OF SLOPE-XYZ   */
/**********************************************/
#define SMA13X_ENABLE_SLOPE_X_INTR_POS         (0)
#define SMA13X_ENABLE_SLOPE_X_INTR_LEN         (1)
#define SMA13X_ENABLE_SLOPE_X_INTR_MSK         (0x01)
#define SMA13X_ENABLE_SLOPE_X_INTR_REG         SMA13X_INTR_ENABLE1_ADDR

#define SMA13X_ENABLE_SLOPE_Y_INTR_POS         (1)
#define SMA13X_ENABLE_SLOPE_Y_INTR_LEN         (1)
#define SMA13X_ENABLE_SLOPE_Y_INTR_MSK         (0x02)
#define SMA13X_ENABLE_SLOPE_Y_INTR_REG         SMA13X_INTR_ENABLE1_ADDR

#define SMA13X_ENABLE_SLOPE_Z_INTR_POS         (2)
#define SMA13X_ENABLE_SLOPE_Z_INTR_LEN         (1)
#define SMA13X_ENABLE_SLOPE_Z_INTR_MSK         (0x04)
#define SMA13X_ENABLE_SLOPE_Z_INTR_REG         SMA13X_INTR_ENABLE1_ADDR
/**********************************************/
/**\name INTERRUPT ENABLE OF HIGH_G-XYZ   */
/**********************************************/
#define SMA13X_ENABLE_HIGH_G_X_INTR_POS         (0)
#define SMA13X_ENABLE_HIGH_G_X_INTR_LEN         (1)
#define SMA13X_ENABLE_HIGH_G_X_INTR_MSK         (0x01)
#define SMA13X_ENABLE_HIGH_G_X_INTR_REG         SMA13X_INTR_ENABLE2_ADDR

#define SMA13X_ENABLE_HIGH_G_Y_INTR_POS         (1)
#define SMA13X_ENABLE_HIGH_G_Y_INTR_LEN         (1)
#define SMA13X_ENABLE_HIGH_G_Y_INTR_MSK         (0x02)
#define SMA13X_ENABLE_HIGH_G_Y_INTR_REG         SMA13X_INTR_ENABLE2_ADDR

#define SMA13X_ENABLE_HIGH_G_Z_INTR_POS         (2)
#define SMA13X_ENABLE_HIGH_G_Z_INTR_LEN         (1)
#define SMA13X_ENABLE_HIGH_G_Z_INTR_MSK         (0x04)
#define SMA13X_ENABLE_HIGH_G_Z_INTR_REG         SMA13X_INTR_ENABLE2_ADDR
/**********************************************/
/**\name INTERRUPT ENABLE OF DATA   */
/**********************************************/
#define SMA13X_ENABLE_NEW_DATA_INTR_POS        (4)
#define SMA13X_ENABLE_NEW_DATA_INTR_LEN        (1)
#define SMA13X_ENABLE_NEW_DATA_INTR_MSK        (0x10)
#define SMA13X_ENABLE_NEW_DATA_INTR_REG        SMA13X_INTR_ENABLE2_ADDR
/**********************************************/
/************************************************/
/**\name INTERRUPT ENABLE OF SLOW NO MOTION-XYZ */
/*************************************************/
#define SMA13X_INTR_SLOW_NO_MOTION_ENABLE_X_INTR_POS        (0)
#define SMA13X_INTR_SLOW_NO_MOTION_ENABLE_X_INTR_LEN        (1)
#define SMA13X_INTR_SLOW_NO_MOTION_ENABLE_X_INTR_MSK        (0x01)
#define SMA13X_INTR_SLOW_NO_MOTION_ENABLE_X_INTR_REG        \
SMA13X_INTR_SLOW_NO_MOTION_ADDR

#define SMA13X_INTR_SLOW_NO_MOTION_ENABLE_Y_INTR_POS        (1)
#define SMA13X_INTR_SLOW_NO_MOTION_ENABLE_Y_INTR_LEN        (1)
#define SMA13X_INTR_SLOW_NO_MOTION_ENABLE_Y_INTR_MSK        (0x02)
#define SMA13X_INTR_SLOW_NO_MOTION_ENABLE_Y_INTR_REG        \
SMA13X_INTR_SLOW_NO_MOTION_ADDR

#define SMA13X_INTR_SLOW_NO_MOTION_ENABLE_Z_INTR_POS        (2)
#define SMA13X_INTR_SLOW_NO_MOTION_ENABLE_Z_INTR_LEN        (1)
#define SMA13X_INTR_SLOW_NO_MOTION_ENABLE_Z_INTR_MSK        (0x04)
#define SMA13X_INTR_SLOW_NO_MOTION_ENABLE_Z_INTR_REG        \
SMA13X_INTR_SLOW_NO_MOTION_ADDR
/**********************************************/
/**\name INTERRUPT ENABLE OF SLOW NO MOTION SELECT */
/**********************************************/
#define SMA13X_INTR_SLOW_NO_MOTION_ENABLE_SELECT_INTR_POS        (3)
#define SMA13X_INTR_SLOW_NO_MOTION_ENABLE_SELECT_INTR_LEN        (1)
#define SMA13X_INTR_SLOW_NO_MOTION_ENABLE_SELECT_INTR_MSK        (0x08)
#define SMA13X_INTR_SLOW_NO_MOTION_ENABLE_SELECT_INTR_REG        \
SMA13X_INTR_SLOW_NO_MOTION_ADDR
/**********************************************/
/**\name INTERRUPT1 ENABLE OF PAD HIGH_G */
/**********************************************/
#define SMA13X_ENABLE_INTR1_PAD_HIGH_G_POS       (1)
#define SMA13X_ENABLE_INTR1_PAD_HIGH_G_LEN       (1)
#define SMA13X_ENABLE_INTR1_PAD_HIGH_G_MSK       (0x02)
#define SMA13X_ENABLE_INTR1_PAD_HIGH_G_REG       SMA13X_INTR1_PAD_SELECT_ADDR
/**********************************************/
/**\name INTERRUPT1 ENABLE OF PAD SLOPE */
/**********************************************/
#define SMA13X_ENABLE_INTR1_PAD_SLOPE_POS       (2)
#define SMA13X_ENABLE_INTR1_PAD_SLOPE_LEN       (1)
#define SMA13X_ENABLE_INTR1_PAD_SLOPE_MSK       (0x04)
#define SMA13X_ENABLE_INTR1_PAD_SLOPE_REG       SMA13X_INTR1_PAD_SELECT_ADDR
/**********************************************/
/**\name INTERRUPT1 ENABLE OF SLOW NO MOTION  */
/**********************************************/
#define SMA13X_ENABLE_INTR1_PAD_SLOW_NO_MOTION_POS        (3)
#define SMA13X_ENABLE_INTR1_PAD_SLOW_NO_MOTION_LEN        (1)
#define SMA13X_ENABLE_INTR1_PAD_SLOW_NO_MOTION_MSK        (0x08)
#define SMA13X_ENABLE_INTR1_PAD_SLOW_NO_MOTION_REG        \
SMA13X_INTR1_PAD_SELECT_ADDR
/**********************************************/
/**\name INTERRUPT1 ENABLE OF PAD DATA */
/**********************************************/
#define SMA13X_ENABLE_INTR1_PAD_NEWDATA_POS     (0)
#define SMA13X_ENABLE_INTR1_PAD_NEWDATA_LEN     (1)
#define SMA13X_ENABLE_INTR1_PAD_NEWDATA_MSK     (0x01)
#define SMA13X_ENABLE_INTR1_PAD_NEWDATA_REG     SMA13X_INTR_DATA_SELECT_ADDR
/**********************************************/
/**\name  INTERRUPT SOURCE SELECTION OF HIGH_G*/
/**********************************************/
#define SMA13X_UNFILT_INTR_SOURCE_HIGH_G_POS       (1)
#define SMA13X_UNFILT_INTR_SOURCE_HIGH_G_LEN       (1)
#define SMA13X_UNFILT_INTR_SOURCE_HIGH_G_MSK       (0x02)
#define SMA13X_UNFILT_INTR_SOURCE_HIGH_G_REG       SMA13X_INTR_SOURCE_ADDR
/**********************************************/
/**\name  INTERRUPT SOURCE SELECTION OF SLOPE*/
/**********************************************/
#define SMA13X_UNFILT_INTR_SOURCE_SLOPE_POS       (2)
#define SMA13X_UNFILT_INTR_SOURCE_SLOPE_LEN       (1)
#define SMA13X_UNFILT_INTR_SOURCE_SLOPE_MSK       (0x04)
#define SMA13X_UNFILT_INTR_SOURCE_SLOPE_REG       SMA13X_INTR_SOURCE_ADDR
/**********************************************/
/**\name  INTERRUPT SOURCE SELECTION OF SLOW NO MOTION*/
/**********************************************/
#define SMA13X_UNFILT_INTR_SOURCE_SLOW_NO_MOTION_POS        (3)
#define SMA13X_UNFILT_INTR_SOURCE_SLOW_NO_MOTION_LEN        (1)
#define SMA13X_UNFILT_INTR_SOURCE_SLOW_NO_MOTION_MSK        (0x08)
#define SMA13X_UNFILT_INTR_SOURCE_SLOW_NO_MOTION_REG        \
SMA13X_INTR_SOURCE_ADDR
/**********************************************/
/**\name  INTERRUPT SOURCE SELECTION OF DATA*/
/**********************************************/
#define SMA13X_UNFILT_INTR_SOURCE_DATA_POS        (5)
#define SMA13X_UNFILT_INTR_SOURCE_DATA_LEN        (1)
#define SMA13X_UNFILT_INTR_SOURCE_DATA_MSK        (0x20)
#define SMA13X_UNFILT_INTR_SOURCE_DATA_REG        SMA13X_INTR_SOURCE_ADDR
/****************************************************/
/**\name  INTERRUPT PAD ACTIVE LEVEL AND OUTPUT TYPE*/
/****************************************************/
#define SMA13X_INTR1_PAD_ACTIVE_LEVEL_POS       (0)
#define SMA13X_INTR1_PAD_ACTIVE_LEVEL_LEN       (1)
#define SMA13X_INTR1_PAD_ACTIVE_LEVEL_MSK       (0x01)
#define SMA13X_INTR1_PAD_ACTIVE_LEVEL_REG       SMA13X_INTR_SET_ADDR

#define SMA13X_INTR1_PAD_OUTPUT_TYPE_POS        (1)
#define SMA13X_INTR1_PAD_OUTPUT_TYPE_LEN        (1)
#define SMA13X_INTR1_PAD_OUTPUT_TYPE_MSK        (0x02)
#define SMA13X_INTR1_PAD_OUTPUT_TYPE_REG        SMA13X_INTR_SET_ADDR
/****************************************************/
/**\name   LATCH INTERRUPT */
/****************************************************/
#define SMA13X_LATCH_INTR_POS                (0)
#define SMA13X_LATCH_INTR_LEN                (4)
#define SMA13X_LATCH_INTR_MSK                (0x0F)
#define SMA13X_LATCH_INTR_REG                SMA13X_INTR_CTRL_ADDR
/****************************************************/
/**\name   RESET LATCH INTERRUPT */
/****************************************************/
#define SMA13X_RESET_INTR_POS           (7)
#define SMA13X_RESET_INTR_LEN           (1)
#define SMA13X_RESET_INTR_MSK           (0x80)
#define SMA13X_RESET_INTR_REG           SMA13X_INTR_CTRL_ADDR
/****************************************************/
/**\name   HIGH_G HYSTERESIS */
/****************************************************/
#define SMA13X_HIGH_G_HYST_POS                  (6)
#define SMA13X_HIGH_G_HYST_LEN                  (2)
#define SMA13X_HIGH_G_HYST_MSK                  (0xC0)
#define SMA13X_HIGH_G_HYST_REG                  SMA13X_LOW_HIGH_HYST_ADDR
/****************************************************/
/**\name   SLOPE DURATION */
/****************************************************/
#define SMA13X_SLOPE_DURN_POS                    (0)
#define SMA13X_SLOPE_DURN_LEN                    (2)
#define SMA13X_SLOPE_DURN_MSK                    (0x03)
#define SMA13X_SLOPE_DURN_REG                    SMA13X_SLOPE_DURN_ADDR
/****************************************************/
/**\name   SLOW NO MOTION DURATION */
/****************************************************/
#define SMA13X_SLOW_NO_MOTION_DURN_POS                    (2)
#define SMA13X_SLOW_NO_MOTION_DURN_LEN                    (6)
#define SMA13X_SLOW_NO_MOTION_DURN_MSK                    (0xFC)
#define SMA13X_SLOW_NO_MOTION_DURN_REG                    SMA13X_SLOPE_DURN_ADDR

/****************************************************/
/**\name   ACTIVATE SELF TEST  */
/****************************************************/
#define SMA13X_ENABLE_SELFTEST_POS                (0)
#define SMA13X_ENABLE_SELFTEST_LEN                (2)
#define SMA13X_ENABLE_SELFTEST_MSK                (0x03)
#define SMA13X_ENABLE_SELFTEST_REG                SMA13X_SELFTEST_ADDR
/****************************************************/
/**\name   SELF TEST -- NEGATIVE   */
/****************************************************/
#define SMA13X_NEG_SELFTEST_POS               (2)
#define SMA13X_NEG_SELFTEST_LEN               (1)
#define SMA13X_NEG_SELFTEST_MSK               (0x04)
#define SMA13X_NEG_SELFTEST_REG               SMA13X_SELFTEST_ADDR

/****************************************************/
/**\name   SPI INTERFACE MODE SELECTION   */
/***************************************************/
#define SMA13X_ENABLE_SPI_MODE_3_POS              (0)
#define SMA13X_ENABLE_SPI_MODE_3_LEN              (1)
#define SMA13X_ENABLE_SPI_MODE_3_MSK              (0x01)
#define SMA13X_ENABLE_SPI_MODE_3_REG              SMA13X_SERIAL_CTRL_ADDR
/****************************************************/
/**\name   I2C WATCHDOG PERIOD SELECTION   */
/***************************************************/
#define SMA13X_I2C_WDT_PERIOD_POS        (1)
#define SMA13X_I2C_WDT_PERIOD_LEN        (1)
#define SMA13X_I2C_WDT_PERIOD_MSK        (0x02)
#define SMA13X_I2C_WDT_PERIOD_REG        SMA13X_SERIAL_CTRL_ADDR
/****************************************************/
/**\name   I2C WATCHDOG ENABLE   */
/***************************************************/
#define SMA13X_ENABLE_I2C_WDT_POS            (2)
#define SMA13X_ENABLE_I2C_WDT_LEN            (1)
#define SMA13X_ENABLE_I2C_WDT_MSK            (0x04)
#define SMA13X_ENABLE_I2C_WDT_REG            SMA13X_SERIAL_CTRL_ADDR
/****************************************************/
/**\name   SPI INTERFACE MODE SELECTIONE            */
/***************************************************/
#define SMA13X_UNLOCK_EE_WRITE_TRIM_POS        (4)
#define SMA13X_UNLOCK_EE_WRITE_TRIM_LEN        (4)
#define SMA13X_UNLOCK_EE_WRITE_TRIM_MSK        (0xF0)
#define SMA13X_UNLOCK_EE_WRITE_TRIM_REG        SMA13X_CTRL_UNLOCK_REG
/******************************************************************/
/**\name   OFFSET  COMPENSATION/SLOW COMPENSATION FOR X,Y,Z AXIS */
/*****************************************************************/
#define SMA13X_ENABLE_SLOW_COMP_X_POS              (0)
#define SMA13X_ENABLE_SLOW_COMP_X_LEN              (1)
#define SMA13X_ENABLE_SLOW_COMP_X_MSK              (0x01)
#define SMA13X_ENABLE_SLOW_COMP_X_REG              SMA13X_OFFSET_CTRL_ADDR

#define SMA13X_ENABLE_SLOW_COMP_Y_POS              (1)
#define SMA13X_ENABLE_SLOW_COMP_Y_LEN              (1)
#define SMA13X_ENABLE_SLOW_COMP_Y_MSK              (0x02)
#define SMA13X_ENABLE_SLOW_COMP_Y_REG              SMA13X_OFFSET_CTRL_ADDR

#define SMA13X_ENABLE_SLOW_COMP_Z_POS              (2)
#define SMA13X_ENABLE_SLOW_COMP_Z_LEN              (1)
#define SMA13X_ENABLE_SLOW_COMP_Z_MSK              (0x04)
#define SMA13X_ENABLE_SLOW_COMP_Z_REG              SMA13X_OFFSET_CTRL_ADDR
/****************************************************/
/**\name   FAST COMPENSATION READY FLAG            */
/***************************************************/
#define SMA13X_FAST_CAL_RDY_STAT_POS             (4)
#define SMA13X_FAST_CAL_RDY_STAT_LEN             (1)
#define SMA13X_FAST_CAL_RDY_STAT_MSK             (0x10)
#define SMA13X_FAST_CAL_RDY_STAT_REG             SMA13X_OFFSET_CTRL_ADDR
/****************************************************/
/**\name   FAST COMPENSATION FOR X,Y,Z AXIS         */
/***************************************************/
#define SMA13X_CAL_TRIGGER_POS                (5)
#define SMA13X_CAL_TRIGGER_LEN                (2)
#define SMA13X_CAL_TRIGGER_MSK                (0x60)
#define SMA13X_CAL_TRIGGER_REG                SMA13X_OFFSET_CTRL_ADDR
/****************************************************/
/**\name   RESET OFFSET REGISTERS         */
/***************************************************/
#define SMA13X_RST_OFFSET_POS           (7)
#define SMA13X_RST_OFFSET_LEN           (1)
#define SMA13X_RST_OFFSET_MSK           (0x80)
#define SMA13X_RST_OFFSET_REG           SMA13X_OFFSET_CTRL_ADDR
/****************************************************/
/**\name  BITSLICE FUNCTIONS      */
/***************************************************/
#define SMA13X_GET_BITSLICE(regvar, bitname)\
((regvar & bitname##_MSK) >> bitname##_POS)


#define SMA13X_SET_BITSLICE(regvar, bitname, val)\
((regvar & ~bitname##_MSK) | ((val<<bitname##_POS)&bitname##_MSK))

/****************************************************/
/**\name   CONSTANTS      */
/***************************************************/
/**\name  RESOLUTION SELECTION      */
/***************************************************/
/* Definitions used for accel resolution bit shifting*/
#define SMA13X_14_BIT_SHIFT		(0xFC)
/**< It refers 14bit accel resolution*/
#define SMA13X_10_BIT_SHIFT		(0xC0)
/**< It refers 10bit accel resolution*/
#define SMA13X_12_BIT_SHIFT		(0xF0)
/**< It refers 12bit accel resolution*/
#define BANDWIDTH_DEFINE		(0xFB)
/**< Chip id set for accel bandwidth define*/

/****************************************************/
/**\name  ENABLE DISABLE SELECTION     */
/***************************************************/
#define INTR_ENABLE	(0X01)
/**< Enable selection for bit */
#define INTR_DISABLE	(0x00)
/**< Disable selection for bit */

/****************************************************/
/**\name  OUTPUT TYPE SELECT     */
/***************************************************/
#define OPEN_DRAIN	(0x01)
/**< It refers open drain selection*/
#define PUSS_PULL	(0x01)
/**< It refers push pull selection*/

/****************************************************/
/**\name  LEVEL SELECT     */
/***************************************************/
#define	ACTIVE_LOW	(0x00)
/**< It refers active low selection*/
#define	ACTIVE_HIGH	(0x01)
/**< It refers active high selection*/

/****************************************************/
/**\name  STATUS SELECT     */
/***************************************************/
#define SMA13X_STAT1                             (0)
/**< It refers Status interrupt1 */
#define SMA13X_STAT2                             (1)
/**< It refers Status interrupt2 */
#define SMA13X_STAT3                             (2)
/**< It refers Status interrupt3  */
#define SMA13X_STAT4                             (3)
/**< It refers Status interrupt4  */
#define SMA13X_STAT5                             (4)
/**< It refers Status interrupt5  */

/****************************************************/
/**\name  RANGE AND BANDWIDTH SELECT     */
/***************************************************/
#define SMA13X_RANGE_2G                 (3)
/**< sets range to +/- 2G mode */
#define SMA13X_RANGE_4G                 (5)
/**< sets range to +/- 4G mode */
#define SMA13X_RANGE_8G                 (8)
/**< sets range to +/- 8G mode */
#define SMA13X_RANGE_16G                (12)
/**< sets range to +/- 16G mode */


#define SMA13X_BW_7_81HZ        (0x08)
 /**< sets bandwidth to LowPass 7.81HZ  */
#define SMA13X_BW_15_63HZ       (0x09)
/**< sets bandwidth to LowPass 15.63HZ  */
#define SMA13X_BW_31_25HZ       (0x0A)
/**< sets bandwidth to LowPass 31.25HZ  */
#define SMA13X_BW_62_50HZ       (0x0B)
 /**< sets bandwidth to LowPass 62.50HZ  */
#define SMA13X_BW_125HZ         (0x0C)
 /**< sets bandwidth to LowPass 125HZ  */
#define SMA13X_BW_250HZ         (0x0D)
/**< sets bandwidth to LowPass 250HZ  */
#define SMA13X_BW_500HZ         (0x0E)
/**< sets bandwidth to LowPass 500HZ  */
#define SMA13X_BW_1000HZ        (0x0F)
 /**< sets bandwidth to LowPass 1000HZ  */

/******************************************/
/**\name  SLEEP DURATION SELECT     */
/******************************************/
#define SMA13X_SLEEP_DURN_0_5MS        (0x05)
/* sets sleep duration to 0.5 ms  */
#define SMA13X_SLEEP_DURN_1MS          (0x06)
/* sets sleep duration to 1 ms */
#define SMA13X_SLEEP_DURN_2MS          (0x07)
/* sets sleep duration to 2 ms */
#define SMA13X_SLEEP_DURN_4MS          (0x08)
/* sets sleep duration to 4 ms */
#define SMA13X_SLEEP_DURN_6MS          (0x09)
/* sets sleep duration to 6 ms*/
#define SMA13X_SLEEP_DURN_10MS         (0x0A)
/* sets sleep duration to 10 ms */
#define SMA13X_SLEEP_DURN_25MS         (0x0B)
/* sets sleep duration to 25 ms */
#define SMA13X_SLEEP_DURN_50MS         (0x0C)
/* sets sleep duration to 50 ms */
#define SMA13X_SLEEP_DURN_100MS        (0x0D)
/* sets sleep duration to 100 ms */
#define SMA13X_SLEEP_DURN_500MS        (0x0E)
/* sets sleep duration to 500 ms */
#define SMA13X_SLEEP_DURN_1S           (0x0F)
/* sets sleep duration to 1 s */

/******************************************/
/**\name  LATCH DURATION     */
/******************************************/
#define SMA13X_LATCH_DURN_NON_LATCH    (0x00)
/* sets LATCH duration to NON LATCH  */
#define SMA13X_LATCH_DURN_250MS        (0x01)
/* sets LATCH duration to 250 ms */
#define SMA13X_LATCH_DURN_500MS        (0x02)
/* sets LATCH duration to 500 ms */
#define SMA13X_LATCH_DURN_1S           (0x03)
 /* sets LATCH duration to 1 s */
#define SMA13X_LATCH_DURN_2S           (0x04)
 /* sets LATCH duration to 2 s*/
#define SMA13X_LATCH_DURN_4S           (0x05)
 /* sets LATCH duration to 4 s */
#define SMA13X_LATCH_DURN_8S           (0x06)
 /* sets LATCH duration to 8 s */
#define SMA13X_LATCH_DURN_LATCH        (0x07)
 /* sets LATCH duration to LATCH */
#define SMA13X_LATCH_DURN_NON_LATCH1   (0x08)
 /* sets LATCH duration to NON LATCH1 */
#define SMA13X_LATCH_DURN_250US        (0x09)
 /* sets LATCH duration to 250 Us */
#define SMA13X_LATCH_DURN_500US        (0x0A)
 /* sets LATCH duration to 500 Us */
#define SMA13X_LATCH_DURN_1MS          (0x0B)
 /* sets LATCH duration to 1 Ms */
#define SMA13X_LATCH_DURN_12_5MS       (0x0C)
/* sets LATCH duration to 12.5 Ms */
#define SMA13X_LATCH_DURN_25MS         (0x0D)
/* sets LATCH duration to 25 Ms */
#define SMA13X_LATCH_DURN_50MS         (0x0E)
 /* sets LATCH duration to 50 Ms */
#define SMA13X_LATCH_DURN_LATCH1       (0x0F)
/* sets LATCH duration to LATCH*/

/******************************************/
/**\name  MODE SETTINGS     */
/******************************************/
#define SMA13X_MODE_NORMAL             (0)
#define SMA13X_MODE_LOWPOWER1          (1)
#define SMA13X_MODE_SUSPEND            (2)
#define SMA13X_MODE_DEEP_SUSPEND       (3)

/******************************************/
/**\name  AXIS SELECTION     */
/******************************************/
#define SMA13X_X_AXIS           (0)
/**< It refers SMA13X X-axis */
#define SMA13X_Y_AXIS           (1)
/**< It refers SMA13X Y-axis */
#define SMA13X_Z_AXIS           (2)
/**< It refers SMA13X Z-axis */

/******************************************/
/**\name  INTERRUPT TYPE SELECTION     */
/******************************************/
#define SMA13X_HIGH_G_X_INTR    (1)
/**< enable/disable high_g X interrupt*/
#define SMA13X_HIGH_G_Y_INTR    (2)
/**< enable/disable high_g Y interrupt*/
#define SMA13X_HIGH_G_Z_INTR    (3)
/**< enable/disable high_g Z interrupt*/
#define SMA13X_DATA_ENABLE      (4)
/**< enable/disable data interrupt*/
#define SMA13X_SLOPE_X_INTR     (5)
/**< enable/disable slope X interrupt*/
#define SMA13X_SLOPE_Y_INTR     (6)
/**< enable/disable slope X interrupt*/
#define SMA13X_SLOPE_Z_INTR     (7)
/**< enable/disable slope X interrupt*/

/******************************************/
/**\name  INTERRUPTS PADS     */
/******************************************/
#define SMA13X_INTR1_HIGH_G            (0)
/**< disable high-g interrupt*/
/**< enable high-g interrupt*/
#define SMA13X_INTR1_SLOPE             (0)
/**< disable slope interrupt*/
/**< enable slope interrupt*/
#define SMA13X_INTR1_SLOW_NO_MOTION    (0)
/**< disable slow no motion interrupt*/
/**< enable slow no motion  interrupt*/
#define SMA13X_INTR1_NEWDATA           (0)
/**< disable data  interrupt*/
/**< enable data interrupt*/

/******************************************/
/**\name  SOURCE REGISTER     */
/******************************************/
#define SMA13X_SOURCE_HIGH_G           (1)
#define SMA13X_SOURCE_SLOPE            (2)
#define SMA13X_SOURCE_SLOW_NO_MOTION   (3)
#define SMA13X_SOURCE_DATA             (5)

#define SMA13X_INTR1_OUTPUT      (0)
#define SMA13X_INTR1_LEVEL       (0)

/******************************************/
/**\name  DURATION     */
/******************************************/
#define SMA13X_HIGH_DURN               (1)
#define SMA13X_SLOPE_DURN              (2)
#define SMA13X_SLOW_NO_MOTION_DURN     (3)

/******************************************/
/**\name  THRESHOLD     */
/******************************************/
#define SMA13X_HIGH_THRES               (1)
#define SMA13X_SLOPE_THRES              (2)
#define SMA13X_SLOW_NO_MOTION_THRES     (3)


#define SMA13X_HIGH_G_HYST               (1)

#define SMA13X_I2C_SELECT               (0)
#define SMA13X_I2C_ENABLE               (1)
/******************************************/
/**\name  COMPENSATION     */
/******************************************/
#define SMA13X_SLOW_COMP_X              (0)
#define SMA13X_SLOW_COMP_Y              (1)
#define SMA13X_SLOW_COMP_Z              (2)
/******************************************/
/**\name  OFFSET TRIGGER     */
/******************************************/
#define SMA13X_CUT_OFF                  (0)
#define SMA13X_OFFSET_TRIGGER_X         (1)
#define SMA13X_OFFSET_TRIGGER_Y         (2)
#define SMA13X_OFFSET_TRIGGER_Z         (3)
/******************************************/
/**\name  GP REGISTERS     */
/******************************************/
#define SMA13X_GP0                      (0)
#define SMA13X_GP1                      (1)
/******************************************/
/**\name  SLO NO MOTION REGISTER      */
/******************************************/
#define SMA13X_SLOW_NO_MOTION_ENABLE_X          (0)
#define SMA13X_SLOW_NO_MOTION_ENABLE_Y          (1)
#define SMA13X_SLOW_NO_MOTION_ENABLE_Z          (2)
#define SMA13X_SLOW_NO_MOTION_ENABLE_SELECT     (3)
/******************************************/
/**\name  WAKE UP      */
/******************************************/
#define SMA13X_WAKE_UP_DURN_20MS         (0)
#define SMA13X_WAKE_UP_DURN_80MS         (1)
#define SMA13X_WAKE_UP_DURN_320MS        (2)
#define SMA13X_WAKE_UP_DURN_2560MS       (3)


/* LG/HG thresholds are in LSB and depend on RANGE setting */
/* no range check on threshold calculation */

#define SMA13X_SELFTEST0_ON            (1)
#define SMA13X_SELFTEST1_ON            (2)

#define SMA13X_EE_W_OFF                 (0)
#define SMA13X_EE_W_ON                  (1)
/******************************************/
/**\name  RESOLUTION SETTINGS      */
/******************************************/
#define SMA13X_RESOLUTION_12_BIT        (0)
#define SMA13X_RESOLUTION_10_BIT        (1)
#define SMA13X_RESOLUTION_14_BIT        (3)

/******************************************/
/**\name  CHIP ID SELECTION      */
/******************************************/
#define SMA130_CHIP_ID           (0xFB)
#define SMA131_CHIP_ID           (0xF8)
/****************************************************/
/**\name	ARRAY SIZE DEFINITIONS      */
/***************************************************/
#define SMA13X_ACCEL_DATA_SIZE			(2)
#define SMA13X_ACCEL_XYZ_DATA_SIZE		(6)
/****************************************************/
/**\name	ARRAY PARAMETERS      */
/***************************************************/

#define SMA13X_SENSOR_DATA_ACCEL_LSB	(0)
#define SMA13X_SENSOR_DATA_ACCEL_MSB	(1)

#define SMA13X_SENSOR_DATA_XYZ_X_LSB	(0)
#define SMA13X_SENSOR_DATA_XYZ_X_MSB	(1)
#define SMA13X_SENSOR_DATA_XYZ_Y_LSB	(2)
#define SMA13X_SENSOR_DATA_XYZ_Y_MSB	(3)
#define SMA13X_SENSOR_DATA_XYZ_Z_LSB	(4)
#define SMA13X_SENSOR_DATA_XYZ_Z_MSB	(5)

#define SMA13X_RESOLUTION_12_MASK		(0xF0)
#define SMA13X_RESOLUTION_10_MASK		(0xC0)
#define SMA13X_RESOLUTION_14_MASK		(0xFC)

#define	SMA13X_POWER_MODE_HEX_E_ZERO_MASK	(0xE0)
#define	SMA13X_POWER_MODE_HEX_4_ZERO_MASK	(0x40)
#define	SMA13X_POWER_MODE_HEX_ZERO_ZERO_MASK	(0x00)
#define	SMA13X_POWER_MODE_HEX_ZERO_ONE_MASK	(0x01)
#define	SMA13X_POWER_MODE_HEX_ZERO_TWO_MASK	(0x02)
#define	SMA13X_POWER_MODE_HEX_ZERO_FOUR_MASK	(0x04)
#define	SMA13X_POWER_MODE_HEX_ZERO_SIX_MASK	(0x06)

/** Macro to convert floating point high-g-thresholds
    in G to 8-bit register values.<br>
  * Example: SMA13X_HIGH_TH_IN_G( 1.4, 2.0)
  * generates the register value for 1.4G threshold in 2G mode.
  * \brief convert g-values to 8-bit value
 */
#define SMA13X_HIGH_THRES_IN_G(gthres, range)   ((256 * gthres) / range)

/** Macro to convert floating point high-g-hysteresis
   in G to 8-bit register values.<br>
  * Example: SMA13X_HIGH_HY_IN_G( 0.2, 2.0) generates
  *the register value for 0.2G threshold in 2G mode.
  * \brief convert g-values to 8-bit value
 */
#define SMA13X_HIGH_HYST_IN_G(ghyst, range)    ((32 * ghyst) / range)


/** Macro to convert floating point G-thresholds
    to 8-bit register values<br>
  * Example: SMA13X_SLOPE_TH_IN_G( 1.2, 2.0)
  * generates the register value for 1.2G threshold in 2G mode.
  * \brief convert g-values to 8-bit value
 */

#define SMA13X_SLOPE_THRES_IN_G(gthres, range)    ((128 * gthres) / range)
/******************************************/
/**\name FUNCTION DECLARATION  */
/******************************************/
/**\name FUNCTION FOR COMMON READ AND WRITE   */
/******************************************/
/*!
 * @brief
 *	This API reads the data from
 *	the given register continuously
 *
 *
 *	@param addr_u8 -> Address of the register
 *	@param data_u8 -> The data from the register
 *	@param len_u32 -> no of bytes to read
 *
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
 *
 */
SMA13X_RETURN_FUNCTION_TYPE sma13x_burst_read(u8 addr_u8,
u8 *data_u8, u32 len_u32);
/******************************************/
/**\name FUNCTION FOR INTIALIZE  */
/******************************************/
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
SMA13X_RETURN_FUNCTION_TYPE sma13x_init(struct sma13x_t *sma13x);
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
u8 *data_u8, u8 len_u8);
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
u8 *data_u8, u8 len_u8);
/******************************************/
/**\name FUNCTION FOR   DATA READ*/
/******************************************/
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
SMA13X_RETURN_FUNCTION_TYPE sma13x_read_accel_x(s16 *accel_x_s16);
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
s8 *accel_x_s8);
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
SMA13X_RETURN_FUNCTION_TYPE sma13x_read_accel_y(s16 *accel_y_s16);
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
s8 *accel_y_s8);
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
SMA13X_RETURN_FUNCTION_TYPE sma13x_read_accel_z(s16 *accel_z_s16);
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
s8 *accel_z_s8);
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
struct sma13x_accel_data *accel);
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
struct sma13x_accel_eight_resolution *accel);
/******************************************/
/**\name FUNCTION FOR  RANGE */
/******************************************/
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
u8 *intr_stat_u8);
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
SMA13X_RETURN_FUNCTION_TYPE sma13x_get_range(u8 *range_u8);
/*!
 * @brief This API is used to set the ranges(g values) of the sensor
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
SMA13X_RETURN_FUNCTION_TYPE sma13x_set_range(u8 range_u8);
/******************************************/
/**\name FUNCTION FOR   BANDWIDTH*/
/******************************************/
/*!
 *	@brief This API is used to get the bandwidth of the sensor in the register
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
SMA13X_RETURN_FUNCTION_TYPE sma13x_get_bw(u8 *bw_u8);
/*!
 *	@brief This API is used to set the bandwidth of the sensor in the register
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
SMA13X_RETURN_FUNCTION_TYPE sma13x_set_bw(u8 bw_u8);
/******************************************/
/**\name FUNCTION FOR   POWER MODE*/
/******************************************/
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
u8 *power_mode_u8);
/*!
 *	@brief This API is used to set the operating
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
SMA13X_RETURN_FUNCTION_TYPE sma13x_set_power_mode(u8 power_mode_u8);
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
SMA13X_RETURN_FUNCTION_TYPE sma13x_set_mode_value(u8 power_mode_u8);
/******************************************/
/**\name FUNCTION FOR  SLEEP CONFIGURATION */
/******************************************/
/*!
 *	@brief This API is used to get
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
SMA13X_RETURN_FUNCTION_TYPE sma13x_get_sleep_durn(u8 *sleep_durn_u8);
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
SMA13X_RETURN_FUNCTION_TYPE sma13x_set_sleep_durn(u8 sleep_durn_u8);
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
u8 *sleep_timer_u8);
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
SMA13X_RETURN_FUNCTION_TYPE sma13x_set_sleep_timer_mode(u8 sleep_timer_u8);
/******************************************/
/**\name FUNCTION FOR   HIGH BANDWIDTH*/
/******************************************/
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
SMA13X_RETURN_FUNCTION_TYPE sma13x_get_high_bw(u8 *high_bw_u8);
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
SMA13X_RETURN_FUNCTION_TYPE sma13x_set_high_bw(u8 high_bw_u8);
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
SMA13X_RETURN_FUNCTION_TYPE sma13x_get_shadow_dis(u8 *shadow_dis_u8);
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
SMA13X_RETURN_FUNCTION_TYPE sma13x_set_shadow_dis(u8 shadow_dis_u8);
/******************************************/
/**\name FUNCTION FOR  SOFT RESET */
/******************************************/
/*!
 *	@brief This function is used for the soft reset
 *	The soft reset register will be written
 *	with 0xB6 in the register 0x14.
 *
 *
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
 *
 */
SMA13X_RETURN_FUNCTION_TYPE sma13x_soft_rst(void);
/*!
 * @brief This API is used to update the register values
 *
 *
 *
 *
 *  @param : None
 *
 *
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
 *
 */
SMA13X_RETURN_FUNCTION_TYPE sma13x_update_image(void);
/******************************************/
/**\name FUNCTION FOR  INTERRUPT ENABLE */
/******************************************/
/*!
 *	@brief This API is used to get
 *  interrupt enable bits of the sensor in the registers 0x16 and 0x17
 *	@note slope-x enable, slope-y enable, slope-z enable,
 *	@note high-z enable, high-y enable
 *	@note high-z enable
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
u8 *value_u8);
/*!
 *	@brief This API is used to set
 *  interrupt enable bits of the sensor in the registers 0x16 and 0x17
 *	@note slope-x enable, slope-y enable, slope-z enable,
 *	@note high-z enable, high-y enable
 *	@note high-z enable
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
u8 value_u8);
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
 *	@param slow_no_motion_u8 : The value of slow no motion interrupt
 *      enable
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
u8 *slow_no_motion_u8);
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
SMA13X_RETURN_FUNCTION_TYPE sma13x_set_slow_no_motion(u8 channel_u8,
u8 slow_no_motion_u8);
/*!
 * @brief This API is used to get
 * the interrupt enable of high_g interrupt in the register 0x19 and 0x1B
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
SMA13X_RETURN_FUNCTION_TYPE sma13x_get_intr_high_g(u8 channel_u8,
u8 *intr_high_g_u8);
/*!
 * @brief This API is used to set
 * the interrupt enable of high_g interrupt in the register 0x19 and 0x1B
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
u8 intr_high_g_u8);
/*!
 * @brief This API is used to get
 * the interrupt enable of slope interrupt in the register 0x19 and 0x1B
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
u8 *intr_slope_u8);
/*!
 * @brief This API is used to set
 * the interrupt enable of slope interrupt in the register 0x19 and 0x1B
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
u8 intr_slope_u8);
/*!
 * @brief This API is used to get
 * the interrupt enable of slow/no motion interrupt in
 * the register 0x19 and 0x1B
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
u8 *intr_slow_no_motion_u8);
/*!
 * @brief This API is used to set
 * the interrupt enable of slow/no motion interrupt in
 * the register 0x19 and 0x1B
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
u8 intr_slow_no_motion_u8);
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
u8 *intr_newdata_u8);
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
 *              1          | SMA13X_ACCEL_INTR2_NEWDATA
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
u8 intr_newdata_u8);
/******************************************/
/**\name FUNCTION FOR  SOURCE CONFIGURATION */
/******************************************/
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
u8 *intr_source_u8);
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
u8 intr_source_u8);
/******************************************/
/**\name FUNCTION FOR   OUTPUT TYPE AND LEVEL*/
/******************************************/
/*!
 *	@brief This API is used to get
 *	the interrupt output type in the register 0x20.
 *	@note INTR1 -> bit 1
 *	@note INTR2 -> bit 3
 *
 *  @param channel_u8: The value of output type select
 *       channel_u8     |    result
 *       -----------------| ------------------
 *               0        | SMA13X_ACCEL_INTR1_OUTPUT
 *               1        | SMA13X_ACCEL_INTR2_OUTPUT
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
u8 *intr_output_type_u8);
/*!
 *	@brief This API is used to set
 *	the interrupt output type in the register 0x20.
 *	@note INTR1 -> bit 1
 *	@note INTR2 -> bit 3
 *
 *  @param channel_u8: The value of output type select
 *         channel_u8   |    result
 *       -----------------| ------------------
 *               0        | SMA13X_ACCEL_INTR1_OUTPUT
 *               1        | SMA13X_ACCEL_INTR2_OUTPUT
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
u8 intr_output_type_u8);
/*!
 *	@brief This API is used to get
 *	Active Level status in the register 0x20
 *	@note INTR1 -> bit 0
 *	@note INTR2 -> bit 2
 *
 *  @param channel_u8: The value of Active Level select
 *       channel_u8     |    result
 *       -----------------| ------------------
 *               0        | SMA13X_ACCEL_INTR1_LEVEL
 *               1        | SMA13X_ACCEL_INTR2_LEVEL
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
SMA13X_RETURN_FUNCTION_TYPE sma13x_get_intr_level(u8 channel_u8,
u8 *intr_level_u8);
/*!
 *	@brief This API is used to set
 *	Active Level status in the register 0x20
 *	@note INTR1 -> bit 0
 *	@note INTR2 -> bit 2
 *
 *  @param channel_u8: The value of Active Level select
 *       channel_u8     |    result
 *       -----------------| ------------------
 *               0        | SMA13X_ACCEL_INTR1_LEVEL
 *               1        | SMA13X_ACCEL_INTR2_LEVEL
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
u8 intr_level_u8);
/******************************************/
/**\name FUNCTION FOR  RESET INTERRUPT*/
/******************************************/
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
SMA13X_RETURN_FUNCTION_TYPE sma13x_rst_intr(u8 rst_intr_u8);
/******************************************/
/**\name FUNCTION FOR   LATCH INTERRUPT INTERRUPT*/
/******************************************/
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
SMA13X_RETURN_FUNCTION_TYPE sma13x_get_latch_intr(u8 *latch_intr_u8);
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
SMA13X_RETURN_FUNCTION_TYPE sma13x_set_latch_intr(u8 latch_intr_u8);
/******************************************/
/**\name FUNCTION FOR   INTERRUPT DURATION CONFIGURATION*/
/******************************************/
/*!
 *	@brief This API is used to get the duration of
 *	Low, High, Slope and slow no motion interrupts in the registers
 *	@note HIGH_DURN		-> register 0x25 bit form 0 to 7
 *	@note SLOPE_DURN		-> register 0x27 bit form 0 to 1
 *	@note SLOW_NO_MOT_DURN -> register 0x27 bit form 2 to 7
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
u8 *durn_u8);
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
u8 durn_u8);
/******************************************/
/**\name FUNCTION FOR  INTERRUPT THRESHOLD CONFIGURATION */
/******************************************/
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
u8 *thres_u8);
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
u8 thres_u8);
/******************************************/
/**\name FUNCTION FOR INTERRUPT HYSTERESIS  */
/******************************************/
/*!
 *	@brief This API is used to get
 *	the high hysteresis in the registers 0x24
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
u8 *hyst_u8);
/*!
 *	@brief This API is used to set
 *	the high hysteresis in the registers 0x24
 *	@note HIGH_G_HYST  -> bit from 6 to 7
 *
 *  @param channel_u8: The value of hysteresis selection
 *     channel_u8   | result
 *   -----------------| ------------------
 *           1        | SMA13X_ACCEL_HIGH_G_HYST
 *
 *  @param hyst_u8: The hysteresis data
 *
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
u8 hyst_u8);
/******************************************/
/**\name FUNCTION FOR SELFTEST   */
/******************************************/
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
u8 *selftest_axis_u8);
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
u8 selftest_axis_u8);
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
u8 *selftest_sign_u8);
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
u8 selftest_sign_u8);
/*!
 * @brief This API is used to get
 * the nvm program ready in the register bit 2
 *
 *
 *  @param nvprog_ready_u8: The value of nvm program ready
 *     nvprog_ready_u8      |    result
 *  ------------------------- |------------------
 *     0x00                   | nvm write/update operation is in progress
 *     0x01                   | nvm is ready to accept a new write
 *
 *
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
 *
 */
SMA13X_RETURN_FUNCTION_TYPE sma13x_get_nvmprog_ready(u8 *nvprog_ready_u8);
/*!
 * @brief This API is used to set
 * the nvm program ready in the register bit 2
 *
 *
 *  @param nvprog_remain_u8: The value of nvm program ready
 *     nvprog_remain_u8     |    result
 *  ------------------------- |------------------
 *     0x00                   | nvm write/update operation is in progress
 *     0x01                   | nvm is ready to accept a new write
 *
 *
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
 *
 */
SMA13X_RETURN_FUNCTION_TYPE sma13x_get_nvmprog_remain(u8 *nvprog_remain_u8);
/******************************************/
/**\name FUNCTION FOR  SPI/I2C CONFIGURATION */
/******************************************/
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
SMA13X_RETURN_FUNCTION_TYPE sma13x_get_spi3(u8 *spi3_u8);
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
SMA13X_RETURN_FUNCTION_TYPE sma13x_set_spi3(u8 spi3_u8);
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
u8 *i2c_wdt_u8);
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
u8 i2c_wdt_u8);
/******************************************/
/**\name FUNCTION FOR OFFSET  */
/******************************************/
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
u8 *slow_comp_u8);
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
u8 slow_comp_u8);
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
SMA13X_RETURN_FUNCTION_TYPE sma13x_get_cal_rdy(u8 *cal_rdy_u8);
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
SMA13X_RETURN_FUNCTION_TYPE sma13x_set_cal_trigger(u8 cal_trigger_u8);
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
SMA13X_RETURN_FUNCTION_TYPE sma13x_set_offset_rst(u8 offset_rst_u8);
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
s8 *offset_u8);
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
s8 offset_u8);
#endif
