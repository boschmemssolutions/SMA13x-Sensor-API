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
#include <linux/interrupt.h>
#include <linux/input.h>
#include <linux/workqueue.h>
#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/of_gpio.h>
#include <linux/of_irq.h>

#include "sma13x.h"
#include "sma13x_driver.h"

#define MODULE_TAG MODULE_NAME
#include "sma13x_log.h"
#include "sma13x.h"

/* in 14bit resolution, the max vaule is defined here */
#define SMA13X_MIN_VALUE -8192
#define SMA13X_MAX_VALUE 8192

#define SMA13X_DEBUG 1

struct sma13x_drv_data {
	struct device *dev;
	struct input_dev *input;
	int IRQ;
	u8 gpio_pin;
	struct work_struct irq_work;
};

static inline SMA13X_RETURN_FUNCTION_TYPE sma13x_enable_all_high_g_int(void)
{
	SMA13X_RETURN_FUNCTION_TYPE err = 0;
	err |= sma13x_set_intr_enable(SMA13X_HIGH_G_X_INTR, INTR_ENABLE);
	err |= sma13x_set_intr_enable(SMA13X_HIGH_G_Y_INTR, INTR_ENABLE);
	err |= sma13x_set_intr_enable(SMA13X_HIGH_G_Z_INTR, INTR_ENABLE);
	return err;
}

static inline SMA13X_RETURN_FUNCTION_TYPE sma13x_enable_all_slope_int(void)
{
	SMA13X_RETURN_FUNCTION_TYPE err = 0;
	err |= sma13x_set_intr_enable(SMA13X_SLOPE_X_INTR, INTR_ENABLE);
	err |= sma13x_set_intr_enable(SMA13X_SLOPE_Y_INTR, INTR_ENABLE);
	err |= sma13x_set_intr_enable(SMA13X_SLOPE_Z_INTR, INTR_ENABLE);
	return err;
}

static inline SMA13X_RETURN_FUNCTION_TYPE sma13x_enable_all_no_motion_int(void)
{
	SMA13X_RETURN_FUNCTION_TYPE err = 0;
	err |= sma13x_set_slow_no_motion(SMA13X_SLOW_NO_MOTION_ENABLE_X, INTR_ENABLE);
	err |= sma13x_set_slow_no_motion(SMA13X_SLOW_NO_MOTION_ENABLE_Y, INTR_ENABLE);
	err |= sma13x_set_slow_no_motion(SMA13X_SLOW_NO_MOTION_ENABLE_Z, INTR_ENABLE);
	err |= sma13x_set_slow_no_motion(SMA13X_SLOW_NO_MOTION_ENABLE_SELECT, INTR_ENABLE);
	return err;
}

static ssize_t sma13x_reg_dump(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	u8 data[SMA13X_REG_NUM + 1];
	int err = 0;
	int i;

	err = sma13x_read_reg(0, data, SMA13X_REG_NUM);
	if (err) {
		PERR("reg dump falied");
		return err;
	}
	PINFO("SMA13X reg dump");
	for (i = 0; i <= SMA13X_REG_NUM; i++) {
		PINFO("register 0x%x:0x%hhx", i, data[i]);
	}

	return 0;
}

static ssize_t sma13x_show_chip_id(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	u8 chip_id[2] = {0};
	int err = 0;

	err = sma13x_read_reg(SMA13X_CHIP_ID_REG, chip_id, 2);
	if (err) {
		return scnprintf(buf, PAGE_SIZE, "read chip id failed\n");
	}
	return scnprintf(buf, PAGE_SIZE, "chip_id=0x%x rev_id=0x%x\n",
		chip_id[0], chip_id[1]);
}

static ssize_t sma13x_soft_reset(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	int err = 0;

	err |= sma13x_soft_rst();

	if (err < 0) {
		return scnprintf(buf, PAGE_SIZE, "soft reset failed\n");
	}
	else {
		return scnprintf(buf, PAGE_SIZE, "sensor is soft reseted\n");
	}
}

static ssize_t sma13x_self_test_x(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	int err = 0;
	u8 range;
	s16 data;

	err = sma13x_get_range(&range);
	PINFO("current range value %hd is stored", range);
	if (err < 0)
		goto exit_self_test;

	/* x axis testing */
	PINFO("active x axis test -----------------------");
	err |= sma13x_soft_rst();
	sma13x_delay(50);
	PINFO("soft reset is done and it is recommented");
	err = sma13x_set_range(SMA13X_RANGE_4G);
	PINFO("set range to 4G");
	err |= sma13x_set_selftest_axis(0b01);
	PINFO("apply negative force");
	err |= sma13x_set_selftest_sign(0);
	sma13x_delay(50);
	err |= sma13x_read_accel_x(&data);
	PINFO("x value under negative force: %hd", data);
	PINFO("apply positive force");
	err |= sma13x_set_selftest_sign(1);
	sma13x_delay(50);
	err |= sma13x_read_accel_x(&data);
	PINFO("x value under positive force: %hd", data);
	PINFO("disable selftest");
	err |= sma13x_set_selftest_axis(0b00);
	if (err < 0)
		goto exit_self_test;

	err |= sma13x_soft_rst();
	PINFO("soft reset is done and it is recommented");
	err |= sma13x_set_range(range);
	PINFO("recover range value");
	if (err < 0)
		goto exit_self_test;

	return scnprintf(buf, PAGE_SIZE, "self test is done, use command dmesg to monitor the output\n");

exit_self_test:
	PERR("self test failed");
	return scnprintf(buf, PAGE_SIZE, "self test failed, use command dmesg to monitor the output\n");
}

static ssize_t sma13x_self_test_y(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	int err = 0;
	u8 range;
	s16 data;

	err = sma13x_get_range(&range);
	PINFO("current range value %hd is stored", range);
	if (err < 0)
		goto exit_self_test;

	/* y axis testing */
	PINFO("active y axis test -----------------------");
	err |= sma13x_soft_rst();
	sma13x_delay(50);
	PINFO("soft reset is done and it is recommented");
	err = sma13x_set_range(SMA13X_RANGE_4G);
	PINFO("set range to 4G");
	err |= sma13x_set_selftest_axis(0b10);
	PINFO("apply negative force");
	err |= sma13x_set_selftest_sign(0);
	sma13x_delay(50);
	err |= sma13x_read_accel_y(&data);
	PINFO("y value under negative force: %hd", data);
	PINFO("apply positive force");
	err |= sma13x_set_selftest_sign(1);
	sma13x_delay(50);
	err |= sma13x_read_accel_y(&data);
	PINFO("y value under positive force: %hd", data);
	PINFO("disable selftest");
	err |= sma13x_set_selftest_axis(0b00);
	if (err < 0)
		goto exit_self_test;

	err |= sma13x_soft_rst();
	PINFO("soft reset is done and it is recommented");
	err |= sma13x_set_range(range);
	PINFO("recover range value");
	if (err < 0)
		goto exit_self_test;

	return scnprintf(buf, PAGE_SIZE, "self test is done, use command dmesg to monitor the output\n");

exit_self_test:
	PERR("self test failed");
	return scnprintf(buf, PAGE_SIZE, "self test failed, use command dmesg to monitor the output\n");
}

static ssize_t sma13x_self_test_z(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	int err = 0;
	u8 range;
	s16 data;

	err = sma13x_get_range(&range);
	PINFO("current range value %hd is stored", range);
	if (err < 0)
		goto exit_self_test;

	/* z axis testing */
	PINFO("active z axis test -----------------------");
	err |= sma13x_soft_rst();
	sma13x_delay(50);
	PINFO("soft reset is done and it is recommented");
	err = sma13x_set_range(SMA13X_RANGE_4G);
	PINFO("set range to 4G");
	err |= sma13x_set_selftest_axis(0b11);
	PINFO("apply negative force");
	err |= sma13x_set_selftest_sign(0);
	sma13x_delay(50);
	err |= sma13x_read_accel_z(&data);
	PINFO("z value under negative force: %hd", data);
	PINFO("apply positive force");
	err |= sma13x_set_selftest_sign(1);
	sma13x_delay(50);
	err |= sma13x_read_accel_z(&data);
	PINFO("z value under positive force: %hd", data);
	PINFO("disable selftest");
	err |= sma13x_set_selftest_axis(0b00);
	if (err < 0)
		goto exit_self_test;


	err |= sma13x_soft_rst();
	PINFO("soft reset is done and it is recommented");
	err |= sma13x_set_range(range);
	PINFO("recover range value");
	if (err < 0)
		goto exit_self_test;

	return scnprintf(buf, PAGE_SIZE, "self test is done, use command dmesg to monitor the output\n");

exit_self_test:
	PERR("self test failed");
	return scnprintf(buf, PAGE_SIZE, "self test failed, use command dmesg to monitor the output\n");
}

static ssize_t sma13x_show_pw_cfg(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	int err;
	u8 mode;

	err = sma13x_get_power_mode(&mode);
	if (err) {
		return scnprintf(buf, PAGE_SIZE, "power config cannot be read out\n");
	}
	return scnprintf(buf, PAGE_SIZE, "%hhu (0:normal 1:LP1 2:suspend 3:deep suspend)\n", mode);
}

static ssize_t sma13x_store_pw_cfg(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	int err = 0;
	u8 value;

	err = kstrtou8(buf, 10, &value);
	if (err)
		return err;
	err = sma13x_set_power_mode(value);
	if (err) {
		PERR("pw config failed");
		return err;
	}
	return count;
}

static ssize_t sma13x_show_high_g_thresh(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	int err;
	u8 value;

	err = sma13x_get_thres(SMA13X_HIGH_THRES, &value);
	if (err < 0) {
		return scnprintf(buf, PAGE_SIZE, "set high-g threshhold failed\n");
	}
	return scnprintf(buf, PAGE_SIZE, "high-g threshhold is %hhu\n", value);
}

static ssize_t sma13x_store_high_g_thresh(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	int err = 0;
	u8 value;

	err = kstrtou8(buf, 10, &value);
	if (err)
		return err;

	PINFO("set high-g threshhold %hhu", value);
	err = sma13x_set_thres(SMA13X_HIGH_THRES, value);
	if (err) {
		PERR("set high-g threshhold ailed");
		return err;
	}
	return count;
}

static ssize_t sma13x_show_int_ctl(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	int err;
	u8 value;

	err = sma13x_get_intr_enable(SMA13X_DATA_ENABLE, &value);
	if (err < 0) {
		return scnprintf(buf, PAGE_SIZE, "set interupt register failed\n");
	}
	return scnprintf(buf, PAGE_SIZE, "new data int is %hhu\n", value);
}

static ssize_t sma13x_store_int_ctl(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	int err = 0;
	unsigned long int_cfg;

	err = kstrtoul(buf, 10, &int_cfg);
	if (err)
		return err;

	if (int_cfg == 0)
		err = sma13x_set_intr_enable(SMA13X_DATA_ENABLE, INTR_DISABLE);
	else
		err = sma13x_set_intr_enable(SMA13X_DATA_ENABLE, INTR_ENABLE);
	if (err) {
		PERR("set interupt register failed");
		return err;
	}
	return count;
}

static ssize_t sma13x_show_acc_value(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	int err;
	struct sma13x_accel_data data = {0};

	err = sma13x_read_accel_xyz(&data);
	if (err < 0) {
		return scnprintf(buf, PAGE_SIZE, "acc value cannot be read out\n");
	}
	return scnprintf(buf, PAGE_SIZE, "%hd %hd %hd\n",
			data.x, data.y, data.z);
}

static ssize_t sma13x_show_driver_version(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	return scnprintf(buf, PAGE_SIZE,
		"Driver version: %s\n", DRIVER_VERSION);
}

static DEVICE_ATTR(regs_dump, S_IRUGO,
	sma13x_reg_dump, NULL);
static DEVICE_ATTR(chip_id, S_IRUGO,
	sma13x_show_chip_id, NULL);
static DEVICE_ATTR(soft_rst, S_IRUGO,
	sma13x_soft_reset, NULL);
static DEVICE_ATTR(self_test_x, S_IRUGO,
	sma13x_self_test_x, NULL);
static DEVICE_ATTR(self_test_y, S_IRUGO,
	sma13x_self_test_y, NULL);
static DEVICE_ATTR(self_test_z, S_IRUGO,
	sma13x_self_test_z, NULL);
static DEVICE_ATTR(pw_cfg, S_IRUGO|S_IWUSR|S_IWGRP,
	sma13x_show_pw_cfg, sma13x_store_pw_cfg);
static DEVICE_ATTR(high_g_thresh, S_IRUGO|S_IWUSR|S_IWGRP,
	sma13x_show_high_g_thresh, sma13x_store_high_g_thresh);
static DEVICE_ATTR(int_ctl, S_IRUGO|S_IWUSR|S_IWGRP,
	sma13x_show_int_ctl, sma13x_store_int_ctl);
static DEVICE_ATTR(value, S_IRUGO,
	sma13x_show_acc_value, NULL);
static DEVICE_ATTR(driver_version, S_IRUGO,
	sma13x_show_driver_version, NULL);

/* driver users can take this as a reference adding other functions here*/
static struct attribute *sma13x_attributes[] = {
	&dev_attr_regs_dump.attr,
	&dev_attr_chip_id.attr,
	&dev_attr_soft_rst.attr,
	&dev_attr_self_test_x.attr,
	&dev_attr_self_test_y.attr,
	&dev_attr_self_test_z.attr,
	&dev_attr_pw_cfg.attr,
	&dev_attr_int_ctl.attr,
	&dev_attr_high_g_thresh.attr,
	&dev_attr_value.attr,
	&dev_attr_driver_version.attr,
	NULL
};

static struct attribute_group sma13x_attribute_group = {
	.attrs = sma13x_attributes
};

static int sma13x_input_init(struct sma13x_drv_data *drv_data)
{
	int err = 0;
	struct input_dev *dev = input_allocate_device();

	if (dev == NULL)
		return -ENOMEM;

	dev->name = SENSOR_NAME;
	input_set_drvdata(dev, drv_data);
	drv_data->input = dev;

	input_set_capability(dev, EV_MSC, MSC_GESTURE);
	input_set_abs_params(dev, ABS_X, SMA13X_MIN_VALUE, SMA13X_MAX_VALUE, 0, 0);
	input_set_abs_params(dev, ABS_Y, SMA13X_MIN_VALUE, SMA13X_MAX_VALUE, 0, 0);
	input_set_abs_params(dev, ABS_Z, SMA13X_MIN_VALUE, SMA13X_MAX_VALUE, 0, 0);

	err = input_register_device(dev);
	if (err)
		input_free_device(dev);
	return err;
}

#ifdef CONFIG_SMA13X_ENABLE_INT
static void sma13x_data_ready_handle(
	struct sma13x_drv_data *drv_data)
{
	int err = 0;
	u8 status;

#ifdef CONFIG_SMA13X_NEWDATA_INT
	struct sma13x_accel_data data = {0};

	err = sma13x_read_reg(SMA13X_DATA_INTR_STAT_REG, &status, 1);
	if (err) {
		PERR("read data interupt status falied");
		return;
	}
	if (status & SMA13X_DATA_INTR_STAT_MSK) {
		err = sma13x_read_accel_xyz(&data);
		if (err < 0) {
			PERR("read xyz value falied");
			return;
		}

		input_event(drv_data->input, EV_ABS, ABS_X, (int)data.x);
		input_event(drv_data->input, EV_ABS, ABS_Y, (int)data.y);
		input_event(drv_data->input, EV_ABS, ABS_Z, (int)data.z);
		input_sync(drv_data->input);
	}
#endif
#ifdef CONFIG_SMA13X_HIGH_G
	err = sma13x_read_reg(SMA13X_HIGH_G_INTR_STAT_REG, &status, 1);
	if (err) {
		PERR("read high-g interupt status falied");
		return;
	}
	if (status & SMA13X_HIGH_G_INTR_STAT_MSK) {
		PINFO("high-g interupt happened");
		err = sma13x_read_reg(SMA13X_STAT_HIGH_G_ADDR, &status, 1);
		if (err < 0) {
			PERR("read high-g value falied");
			return;
		}

		/*1000 is a value big enough to draw attention on the image of test result*/
		if (status & SMA13X_HIGH_G_SIGN_STAT_MSK) {
			PINFO("sign positive");
			input_event(drv_data->input, EV_MSC, MSC_GESTURE, 1000);
		}
		else {
			PINFO("sign negative");
			input_event(drv_data->input, EV_MSC, MSC_GESTURE, -1000);
		}
		input_sync(drv_data->input);
	}

#endif
#ifdef CONFIG_SMA13X_SLOPE
	err = sma13x_read_reg(SMA13X_SLOPE_INTR_STAT_REG, &status, 1);
	if (err) {
		PERR("read slope interupt status falied");
		return;
	}
	if (status & SMA13X_SLOPE_INTR_STAT_MSK) {
		PINFO("slope interupt happened");
		err = sma13x_read_reg(SMA13X_STAT_SLOPE_ADDR, &status, 1);
		if (err < 0) {
			PERR("read slope value falied");
			return;
		}

		/*1000 is a value big enough to draw attention on the image of test result*/
		if (status & SMA13X_SLOPE_SIGN_STAT_MSK) {
			PINFO("sign positive");
			input_event(drv_data->input, EV_MSC, MSC_GESTURE, 1000);
		}
		else {
			PINFO("sign negative");
			input_event(drv_data->input, EV_MSC, MSC_GESTURE, -1000);
		}

		/*value 1,2,3 represent x,y,z axis respectively*/
		if (status & SMA13X_SLOPE_FIRST_X_MSK) {
			PINFO("slope triggered by X");
			input_event(drv_data->input, EV_MSC, MSC_GESTURE, 1);
		}
		else if (status & SMA13X_SLOPE_FIRST_Y_MSK) {
			PINFO("slope triggered by Y");
			input_event(drv_data->input, EV_MSC, MSC_GESTURE, 2);
		}
		else if (status & SMA13X_SLOPE_FIRST_Z_MSK) {
			PINFO("slope triggered by Z");
			input_event(drv_data->input, EV_MSC, MSC_GESTURE, 3);
		}
		else
			PINFO("slope triggered by error");

		input_sync(drv_data->input);
	}
#endif
#ifdef CONFIG_SMA13X_NO_MOTION
	err = sma13x_read_reg(SMA13X_SLOW_NO_MOTION_INTR_STAT_REG, &status, 1);
	if (err) {
		PERR("read no-motion interupt status falied");
		return;
	}
	if (status & SMA13X_SLOW_NO_MOTION_INTR_STAT_MSK) {
		PINFO("no-motion is detected");
		input_event(drv_data->input, EV_MSC, MSC_GESTURE, 1000);
		input_sync(drv_data->input);
	}
#endif

}

static void sma13x_irq_work_func(struct work_struct *work)
{
	struct sma13x_drv_data *drv_data =
		container_of(work, struct sma13x_drv_data, irq_work);

	sma13x_data_ready_handle(drv_data);
}

static irqreturn_t sma13x_irq_handle(int irq, void *handle)
{
	struct sma13x_drv_data *drv_data = handle;
	int err = 0;

	err = schedule_work(&drv_data->irq_work);
	if (err < 0)
		PERR("schedule_work failed");

	return IRQ_HANDLED;
}

static void sma13x_free_irq(struct sma13x_drv_data *drv_data)
{
	free_irq(drv_data->IRQ, drv_data);
	gpio_free(drv_data->gpio_pin);
}

static int sma13x_request_irq(struct sma13x_drv_data *drv_data)
{
	int err = 0;
	drv_data->gpio_pin = of_get_named_gpio_flags(
		drv_data->dev->of_node,
		"gpio_irq", 0, NULL);
	PINFO("SMA13X_ACC gpio number:%d", drv_data->gpio_pin);
	err = gpio_request_one(drv_data->gpio_pin,
				GPIOF_IN, "sma13x_interrupt");
	if (err < 0) {
		PDEBUG("gpio request failed");
		return err;
	}
	err = gpio_direction_input(drv_data->gpio_pin);
	if (err < 0) {
		PDEBUG("gpio set direction failed");
		return err;
	}
	drv_data->IRQ = gpio_to_irq(drv_data->gpio_pin);
	err = request_irq(drv_data->IRQ, sma13x_irq_handle,
			IRQF_TRIGGER_RISING,
			SENSOR_NAME, drv_data);
	if (err < 0) {
		PDEBUG("request_irq");
		return err;
	}
	INIT_WORK(&drv_data->irq_work, sma13x_irq_work_func);
	return err;
}
#endif

static void sma13x_input_destroy(struct sma13x_drv_data *drv_data)
{
	struct input_dev *dev = drv_data->input;
	input_unregister_device(dev);
	/* to avoid underflow of refcount, do a checck before call free device*/
	if (dev->devres_managed)
		input_free_device(dev);
}

int sma13x_remove(struct device *dev)
{
	int err = 0;
	struct sma13x_drv_data *drv_data = dev_get_drvdata(dev);

	if (NULL != drv_data) {
		sma13x_free_irq(drv_data);
		sysfs_remove_group(&drv_data->input->dev.kobj,
				&sma13x_attribute_group);
		sma13x_input_destroy(drv_data);
		kfree(drv_data);
	}
	return err;
}

int sma13x_probe(struct device *dev, struct sma13x_t *sma13x_t)
{
	int err = 0;
	struct sma13x_drv_data *drv_data = NULL;

	if (dev == NULL || sma13x_t == NULL)
		return -EINVAL;

	drv_data = kzalloc(sizeof(struct sma13x_drv_data),
						GFP_KERNEL);
	if (NULL == drv_data) {
		PERR("no memory available");
		err = -ENOMEM;
		goto exit_directly;
	}

	drv_data->dev = dev;

	/* soft rst bring the sensor to normal
	 * sma13x_set_power_mode(SMA13X_MODE_NORMAL) also set power mode to
	 * normal
	 */
	err |= sma13x_soft_rst();
	sma13x_delay(5);

	/* default is max ODR 1KHz, if not set here */
	err |= sma13x_set_bw(SMA13X_BW_125HZ);
	/* 2G is the default, if not set */
	err |= sma13x_set_range(SMA13X_RANGE_2G);

	if (err < 0) {
		PERR("init configuration failed");
		goto exit_free_drv_data;
	}

	err = sma13x_input_init(drv_data);
	if (err < 0) {
		PERR("input init failed");
		goto exit_free_drv_data;
	}

	err = sysfs_create_group(&drv_data->input->dev.kobj,
			&sma13x_attribute_group);
	if (err < 0) {
		PERR("sysfs create failed");
		goto exit_cleanup_input;
	}
#ifdef CONFIG_SMA13X_ENABLE_INT
	err = sma13x_request_irq(drv_data);
	if (err < 0) {
		PERR("Request irq failed");
		goto exit_cleanup_sysfs;
	}
	PINFO("set int latch");
	err = sma13x_set_latch_intr(SMA13X_LATCH_DURN_250MS);
#endif

#ifdef CONFIG_SMA13X_NEWDATA_INT
	PINFO("map new data int to pin1");
	err = sma13x_set_new_data(SMA13X_INTR1_NEWDATA, INTR_ENABLE);
#ifdef SMA13X_DEBUG
	PINFO("set new data enable");
	err = sma13x_set_intr_enable(SMA13X_DATA_ENABLE, INTR_ENABLE);
	if (err < 0) {
		PERR("new data int config failed");
		goto exit_free_int;
	}
#endif
#endif

#ifdef CONFIG_SMA13X_HIGH_G
	PINFO("map high-g int to pin1");
	err = sma13x_set_intr_high_g(SMA13X_INTR1_HIGH_G, INTR_ENABLE);
	PINFO("enable high-g interupt");
	err = sma13x_enable_all_high_g_int();
	if (err < 0) {
		PERR("high-g int config failed");
		goto exit_free_int;
	}
	PINFO("high-g settings are all default values:");
	PINFO("high-g threshhold 0xC0, 192*7.81mg");
	PINFO("high-g duration 0x0F * 2ms");
	PINFO("high-g hysteresis 0x2 * 125mg");
#endif

#ifdef CONFIG_SMA13X_SLOPE
	PINFO("map slope int to pin1");
	err = sma13x_set_intr_slope(SMA13X_INTR1_SLOPE, INTR_ENABLE);
	PINFO("enable slope interupt");
	err = sma13x_enable_all_slope_int();
	if (err < 0) {
		PERR("slope int config failed");
		goto exit_free_int;
	}
	PINFO("slope detection settings are all default values:");
	PINFO("slope threshhold 0x14*3,91mg");
	PINFO("slope duration (0x0 + 1) * 2ms");
#endif
#ifdef CONFIG_SMA13X_NO_MOTION
	PINFO("map no-motion int to pin1");
	err = sma13x_set_intr_slow_no_motion(SMA13X_INTR1_SLOW_NO_MOTION, INTR_ENABLE);
	PINFO("enable no-motion interupt");
	err = sma13x_enable_all_no_motion_int();
	if (err < 0) {
		PERR("no-motion int config failed");
		goto exit_free_int;
	}
	PINFO("no-motion detection settings are all default values:");
	PINFO("no-motion threshhold 0x14*3,91mg");
	PINFO("no-motion duration (0x0 + 1)");
#endif

	PINFO("Sensor %s was probed successfully", SENSOR_NAME);

	return 0;
exit_free_int:
	sma13x_free_irq(drv_data);
exit_cleanup_sysfs:
	sysfs_remove_group(&drv_data->input->dev.kobj,
		&sma13x_attribute_group);
exit_cleanup_input:
	sma13x_input_destroy(drv_data);
exit_free_drv_data:
	if (drv_data != NULL)
		kfree(drv_data);
exit_directly:
	return err;
}

void sma13x_delay(uint32_t msec)
{
	unsigned long mseond = msec;
	unsigned long min = mseond * (1000);
	/* if the time less than 20ms */
	if (msec <= 20)
		usleep_range(min, (min + 1000));
	else
		msleep(msec);
}
