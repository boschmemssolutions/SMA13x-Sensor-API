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
#include <linux/spi/spi.h>
#include <linux/module.h>

#include "sma13x_driver.h"
#define MODULE_TAG MODULE_NAME
#include "sma13x_log.h"
#include "sma13x.h"

#define SMA13X_SPI_BUF_SIZE      32

static struct spi_device *sma13x_device;

static struct sma13x_t sma13x_dev;

static s8 sma13x_spi_write(u8 dev_addr,
	u8 reg_addr, u8 *data, u8 len)
{
	struct spi_message msg;
	u8 buffer[SMA13X_SPI_BUF_SIZE + 1];
	struct spi_transfer xfer = {
		.tx_buf = buffer,
		.len = len + 1,
	};

	if (sma13x_device == NULL)
		return E_SMA13X_NULL_PTR;
	if (len > SMA13X_SPI_BUF_SIZE)
		return E_OUT_OF_RANGE;

	buffer[0] = reg_addr;
	memcpy(&buffer[1], data, len);
	spi_message_init(&msg);
	spi_message_add_tail(&xfer, &msg);

	return spi_sync(sma13x_device, &msg);
}

static s8 sma13x_spi_read(u8 dev_addr,
	u8 reg_addr, u8 *data, u8 len)
{
#ifdef SMA13X_DEBUG
	int ret, i;
#endif
	struct spi_message msg;
	struct spi_transfer xfer[2] = {
		[0] = {
			.tx_buf = &reg_addr,
			.len = 1,
		},
		[1] = {
			.rx_buf = data,
			.len = len,
		}
	};

	if (sma13x_device == NULL)
		return E_SMA13X_NULL_PTR;

	reg_addr |= 0x80;
	spi_message_init(&msg);
	spi_message_add_tail(&xfer[0], &msg);
	spi_message_add_tail(&xfer[1], &msg);

#ifdef SMA13X_DEBUG
	PINFO("before spi_sync\n");
        for (i = 0; i < len; i++)
		PINFO("data[%u]:%x\t", i, data[i]);
	PINFO("\n");
	ret = spi_sync(sma13x_device, &msg);
	PINFO("after spi_sync\n");
        for (i = 0; i < len; i++)
		PINFO("data[%u]:%x\t", i, data[i]);
	PINFO("\n");

	return ret;
#else
	return spi_sync(sma13x_device, &msg);
#endif
}

static int sma13x_spi_probe(struct spi_device *device)
{
	int err;

	device->bits_per_word = 8;
	err = spi_setup(device);
	if (err < 0) {
		PERR("spi_setup err!\n");
		return err;
	}

	sma13x_device = device;

	err = sma13x_init(&sma13x_dev);

        if ((err == SUCCESS) && (sma13x_dev.chip_id == SMA130_CHIP_ID)) {
		PINFO("Bosch Sensor Device %s detected, chip: %x", SENSOR_NAME, sma13x_dev.chip_id);
		return sma13x_probe(&device->dev, &sma13x_dev);
	}
	else {
		sma13x_device = NULL;
		PERR("Bosch Sensor Device %s detect failed, error %d",
				SENSOR_NAME, err);
		return err;
	}

}

static int sma13x_spi_remove(struct spi_device *device)
{
	return sma13x_remove(&device->dev);
}

static const struct spi_device_id sma13x_id[] = {
	{ SENSOR_NAME, 0 },
	{ }
};
MODULE_DEVICE_TABLE(spi, sma13x_id);

static const struct of_device_id sma13x_of_match[] = {
	{ .compatible = SENSOR_NAME, },
	{ }
};
MODULE_DEVICE_TABLE(of, sma13x_of_match);

static struct spi_driver sma13x_driver = {
	.driver = {
		.owner = THIS_MODULE,
		.name  = SENSOR_NAME,
		.of_match_table = sma13x_of_match,
	},
	.id_table = sma13x_id,
	.probe    = sma13x_spi_probe,
	.remove	= sma13x_spi_remove,
};

static int __init sma13x_module_init(void)
{
	sma13x_dev.bus_write = sma13x_spi_write;
	sma13x_dev.bus_read = sma13x_spi_read;
	sma13x_dev.delay_msec = sma13x_delay;
	
	return spi_register_driver(&sma13x_driver);
}

static void __exit sma13x_module_exit(void)
{
	spi_unregister_driver(&sma13x_driver);
}

module_init(sma13x_module_init);
module_exit(sma13x_module_exit);

MODULE_DESCRIPTION("SMA13X SENSOR DRIVER");
MODULE_LICENSE("Dual BSD/GPL");
