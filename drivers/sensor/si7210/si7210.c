/*
 * Copyright (c) 2019 Accelerando
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT silabs_si7210

#include <device.h>
#include <drivers/i2c.h>
#include <drivers/sensor.h>
#include <logging/log.h>

#include "si7210.h"

#define SIGN_BIT_MASK 0x7F

LOG_MODULE_REGISTER(si7210, CONFIG_SENSOR_LOG_LEVEL);

struct si7210_data {
	const struct device *i2c_dev;
	uint16_t temperature;
	uint16_t field;
};

static int si7210_reg_read(struct si7210_data *drv_data, uint8_t reg,
			     uint8_t *val)
{
	//LOG_INF("si7210_reg_read 0x%x\n", (int)reg);
	int err;

	err = i2c_reg_read_byte(drv_data->i2c_dev,
				DT_INST_REG_ADDR(0), reg, val);
	if (err != 0) {
		LOG_INF("si7210_reg_read error %d\n", (int)err);
		return -EIO;
	}

	return 0;
}

static int si7210_reg_write(struct si7210_data *drv_data, uint8_t reg,
			      uint8_t val)
{
	//LOG_INF("si7210_reg_write 0x%x 0x%x\n", (int)reg, (int)val);
	return i2c_reg_write_byte(drv_data->i2c_dev,
		DT_INST_REG_ADDR(0), reg, val);
}

static int si7210_set_oneshot(struct si7210_data *drv_data, bool oneshot) 
{
	uint8_t ctrl;

	if (si7210_reg_read(drv_data, SI7210_REG_CTRL, &ctrl) != 0) {
		return -EIO;
	}

	if (oneshot) {
		ctrl = ctrl | (1<<SI7210_CTRL_BIT_ONEBURST);
	}
	else {
		ctrl = ctrl & ~(1<<SI7210_CTRL_BIT_ONEBURST);
	}
	ctrl = ctrl & ~(1<<SI7210_CTRL_BIT_STOP);

	if (si7210_reg_write(drv_data, SI7210_REG_CTRL, ctrl) != 0) {
		return -EIO;
	}

	return 0;
}

static int si7210_sample_fetch(const struct device *dev,
			       enum sensor_channel chan)
{
	struct si7210_data *drv_data = dev->data;

	int err;
	uint8_t dspsigm;
	uint8_t dspsigl;

	if (si7210_set_oneshot(drv_data, true) != 0) {
		return -EIO;
	}


	if ((chan == SENSOR_CHAN_MAGN_Z) || (chan == SENSOR_CHAN_ALL)) {
		err = si7210_reg_write(drv_data, SI7210_REG_SIG_SEL, 0);
		
		if (!err) {
			err = si7210_reg_read(drv_data, SI7210_REG_SIG_HIGH,
					      &dspsigm);
		}
		if (!err) {
			err = si7210_reg_read(drv_data, SI7210_REG_SIG_LOW,
					    &dspsigl);
		}
		if (err != 0) return err;

		drv_data->field = (((dspsigm&0x7f)<<8)|(dspsigl&0xFF))-16384;
	}
	// not an else-case
	if ((chan == SENSOR_CHAN_MAGN_Z) || (chan == SENSOR_CHAN_ALL)) {
		err = si7210_reg_write(drv_data, SI7210_REG_SIG_SEL, 1);
		if (!err) {
			err = si7210_reg_read(drv_data, SI7210_REG_SIG_HIGH,
					 &dspsigm);
		}
		if (!err) {
			err = si7210_reg_read(drv_data, SI7210_REG_SIG_LOW,
					      &dspsigl);
		}
		if (err != 0) return err;
		drv_data->temperature = (256 * (dspsigm & SIGN_BIT_MASK)) + dspsigl;

	}

	//LOG_INF("fetch success chan=%d", (int)chan);
	return err;
}

static int si7210_channel_get(const struct device *dev,
			      enum sensor_channel chan,
			      struct sensor_value *val)
{
	struct si7210_data *drv_data = dev->data;
	int32_t uval;

	if (chan == SENSOR_CHAN_MAGN_Z) {
		// 1 Gauss (cgs) == 0.0001 Tesla (SI mks)
		// 1 Gauss = 0.1mT (10mT/G)
		// mT = sensorvalue * scale
		uval = drv_data->field * 1000000 / SI7210_SCALE_DIVISOR_COARSE;
		val->val1 = uval / 1000000;
		val->val2 = uval % 1000000;
		//LOG_INF("Field = val1:%d, val2:%d", val->val1, val->val2);
	}
	else if (chan == SENSOR_CHAN_AMBIENT_TEMP) {
		uval = ((55 * 160) + (drv_data->temperature - 16384)) >> 4;
		val->val1 = uval / 10;
		val->val2 = (uval % 10) * 100000;

		//LOG_INF("Temperature = val1:%d, val2:%d", val->val1, val->val2);

	} else {
		LOG_ERR("Unsupported channel %d", (int)chan);
		return -ENOTSUP;
	}

	//LOG_INF("get success chan=%d", (int)chan);
	return 0;
}

static const struct sensor_driver_api si7210_api = {
	.sample_fetch = &si7210_sample_fetch,
	.channel_get = &si7210_channel_get,
};

static int si7210_chip_init(const struct device *dev)
{
	struct si7210_data *drv_data = dev->data;
	uint8_t value;

	drv_data->i2c_dev = device_get_binding(
		DT_INST_BUS_LABEL(0));

	if (!drv_data->i2c_dev) {
		LOG_ERR("Failed to get pointer to %s device!",
			DT_INST_BUS_LABEL(0));
		return -EINVAL;
	}

	if (si7210_reg_read(drv_data, SI7210_REG_CHIP_INFO,
		&value) != 0) {
		LOG_ERR("Chip info read failed");
		return -EIO;
	}

	if ((value >> 4) != SI7210_CHIP_ID_VALUE) {
		LOG_ERR("Bad chip id 0x%x", value);
		return -ENOTSUP;
	}

	//LOG_INF("si7210_chip_init success\n");
	return 0;
}

static int si7210_init(const struct device *dev)
{
	//LOG_INF("si7210_init\n");
	
	if (si7210_chip_init(dev) < 0) {
		return -EINVAL;
	}

	return 0;
}

static struct si7210_data si_data;

DEVICE_AND_API_INIT(si7210, DT_INST_LABEL(0), si7210_init,
	&si_data, NULL, POST_KERNEL, CONFIG_SENSOR_INIT_PRIORITY, &si7210_api);
