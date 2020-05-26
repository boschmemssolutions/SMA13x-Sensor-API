// SPDX-License-Identifier: BSD-3-Clause OR GPL-2.0
/**
 * This file is provided under a dual BSD/GPLv2 license.  When using or
 * redistributing this file, you may do so under either license.
 *
 * GPL LICENSE
 * Copyright (c) 2020 Robert Bosch GmbH. All rights reserved.
 *
 * This file is free software licensed under the terms of version 2 
 * of the GNU General Public License, available from the file LICENSE-GPL 
 * in the main directory of this source tree.
 *
 * BSD LICENSE
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
 
#include <linux/types.h>
#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/delay.h>

#include "sma13x_driver.h"
#define MODULE_TAG MODULE_NAME
#include "sma13x_log.h"
#include "sma13x.h"

#define SMA13X_MAX_RETRY_I2C_XFER 10
#define SMA13X_I2C_WRITE_DELAY_TIME 10
#define SMA13X_I2C_ADDR 0x18

static struct i2c_adapter *sma13x_i2c_adapter;
static struct sma13x_t sma13x_dev;

static s8 sma13x_i2c_read(u8 dev_addr,
	u8 reg_addr, u8 *data, u8 len)
{
	int32_t retry;

	struct i2c_msg msg[] = {
		{
		.addr = dev_addr,
		.flags = 0,
		.len = 1,
		.buf = &reg_addr,
		},

		{
		.addr = dev_addr,
		.flags = I2C_M_RD,
		.len = len,
		.buf = data,
		},
	};
	for (retry = 0; retry < SMA13X_MAX_RETRY_I2C_XFER; retry++) {
		if (i2c_transfer(sma13x_i2c_adapter, msg, ARRAY_SIZE(msg)) > 0)
			break;
		else
			usleep_range(SMA13X_I2C_WRITE_DELAY_TIME * 1000,
				SMA13X_I2C_WRITE_DELAY_TIME * 1000);
	}

	/*PDEBUG("read reg addr 0x%x, len %d, *data %x", reg_addr, len, *data);*/
	if (SMA13X_MAX_RETRY_I2C_XFER <= retry) {
		PERR("I2C xfer error");
		return -EIO;
	}

	return 0;
}

static s8 sma13x_i2c_write(u8 dev_addr,
	u8 reg_addr, u8 *data, u8 len)
{
	int32_t retry;

	struct i2c_msg msg = {
		.addr = dev_addr,
		.flags = 0,
		.len = len + 1,
		.buf = NULL,
	};

	/*PDEBUG("write reg addr 0x%x, len %d, *data %x", reg_addr, len, *data);*/

	msg.buf = kmalloc(len + 1, GFP_KERNEL);
	if (!msg.buf) {
		PERR("Allocate mem failed\n");
		return -ENOMEM;
	}
	msg.buf[0] = reg_addr;
	memcpy(&msg.buf[1], data, len);
	for (retry = 0; retry < SMA13X_MAX_RETRY_I2C_XFER; retry++) {
		if (i2c_transfer(sma13x_i2c_adapter, &msg, 1) > 0)
			break;
		else
			usleep_range(SMA13X_I2C_WRITE_DELAY_TIME * 1000,
				SMA13X_I2C_WRITE_DELAY_TIME * 1000);
	}
	kfree(msg.buf);
	if (SMA13X_MAX_RETRY_I2C_XFER <= retry) {
		PERR("I2C xfer error");
		return -EIO;
	}

	return 0;
}


static int sma13x_i2c_probe(struct i2c_client *client,
	const struct i2c_device_id *id)
{
	int err = 0;

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		PERR("i2c_check_functionality error!");
		err = -EIO;
		return err;
	}

	sma13x_i2c_adapter = client->adapter;

	err = sma13x_init(&sma13x_dev);

        if ((err == SUCCESS) && (sma13x_dev.chip_id == SMA130_CHIP_ID)) {
		PINFO("Bosch Sensor Device %s detected, chip: %x", SENSOR_NAME, sma13x_dev.chip_id);
		return sma13x_probe(&client->dev, &sma13x_dev);
	}
	else {
		PERR("Bosch Sensor Device %s detect failed, error %d",
				SENSOR_NAME, err);
		return err;
	}

}

static int sma13x_i2c_remove(struct i2c_client *client)
{
	return sma13x_remove(&client->dev);
}

static const struct i2c_device_id sma13x_id[] = {
	{ SENSOR_NAME, 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, sma13x_id);

static const struct of_device_id sma13x_of_match[] = {
	{ .compatible = SENSOR_NAME, },
	{ }
};
MODULE_DEVICE_TABLE(of, sma13x_of_match);

struct i2c_driver sma13x_i2c_driver = {
	.driver = {
		.owner = THIS_MODULE,
		.name = SENSOR_NAME,
		.of_match_table = sma13x_of_match,
	},
	.class = I2C_CLASS_HWMON,
	.id_table = sma13x_id,
	.probe = sma13x_i2c_probe,
	.remove = sma13x_i2c_remove,
};

static int __init sma13x_module_init(void)
{
	sma13x_dev.bus_write = sma13x_i2c_write;
	sma13x_dev.bus_read = sma13x_i2c_read;
	sma13x_dev.delay_msec = sma13x_delay;
	sma13x_dev.dev_addr = SMA13X_I2C_ADDR;
	
	return i2c_add_driver(&sma13x_i2c_driver);
}

static void __exit sma13x_module_exit(void)
{
	i2c_del_driver(&sma13x_i2c_driver);
}

module_init(sma13x_module_init);
module_exit(sma13x_module_exit);

MODULE_DESCRIPTION("SMA13X SENSOR DRIVER");
MODULE_LICENSE("Dual BSD/GPL");
