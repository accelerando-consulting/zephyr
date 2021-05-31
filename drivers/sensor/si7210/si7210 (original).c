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
	int8_t temp_offset;
	int8_t temp_gain;
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

	
	uint32_t busy = 0;
	uint8_t ctrl;
	while (busy > 10000) {
		if (si7210_reg_read(drv_data, SI7210_REG_CTRL, &ctrl) != 0) {
			return -EIO;
		}
		if (ctrl & (1<<SI7210_CTRL_BIT_STOP)) {
			break;
		}
		++busy;
	}
	if (busy > 0) {
		LOG_INF("Waited for %lu busyloops for sensor data", (unsigned long) busy);
	}

	if ((chan == SENSOR_CHAN_MAGN_Z) || (chan == SENSOR_CHAN_ALL)) {
		err = si7210_reg_write(drv_data, SI7210_REG_SIG_SEL, 0);

		if (!err) {
			err = si7210_set_oneshot(drv_data, true);
			if (err) {
				LOG_ERR("Si7210 read initiation failed");
				return err;
			}
		}
		

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
	if ((chan == SENSOR_CHAN_AMBIENT_TEMP) || (chan == SENSOR_CHAN_ALL)) {
		err = si7210_reg_write(drv_data, SI7210_REG_SIG_SEL, 1);


		if (!err) {
			err = si7210_set_oneshot(drv_data, true);
			if (err) {
				LOG_ERR("Si7210 read initiation failed");
				return err;
			}
		}

		if (!err) {
			err = si7210_reg_read(drv_data, SI7210_REG_SIG_HIGH,
					 &dspsigm);
		}
		if (!err) {
			err = si7210_reg_read(drv_data, SI7210_REG_SIG_LOW,
					      &dspsigl);
		}
		if (!err) {
			err = si7210_reg_read(drv_data, SI7210_REG_TEMP_OFFSET,
					      &(drv_data->temp_offset));
		}
		if (!err) {
			err = si7210_reg_read(drv_data, SI7210_REG_TEMP_GAIN,
					      &(drv_data->temp_gain));
		}
		if (err != 0) {
			LOG_ERR("Temperature read error: %d", err);
			return err;
		}
		drv_data->temperature = ((dspsigm & SIGN_BIT_MASK)<<5) + (dspsigl>>3);
		/*
		LOG_INF("temperature dspsigm:l=%02x:%02x, offset,gain=%d,%d, 12-bit-fold=%04x",
			(unsigned int)dspsigm, (unsigned int)dspsigl,
			(int)drv_data->temp_offset, (int)drv_data->temp_gain,
			(unsigned int)drv_data->temperature
			);
		*/
	}

	//LOG_INF("fetch success chan=%d", (int)chan);
	return err;
}

static int si7210_channel_get(const struct device *dev,
			      enum sensor_channel chan,
			      struct sensor_value *val)
{
	struct si7210_data *drv_data = dev->data;
	int32_t ival;
	uint32_t uval;

	if (chan == SENSOR_CHAN_MAGN_Z) {
		// 1 Gauss (cgs) == 0.0001 Tesla (SI mks)
		// 1 Gauss = 0.1mT (10mT/G)
		// mT = sensorvalue * scale
		ival = drv_data->field * 1000000 / SI7210_SCALE_DIVISOR_COARSE;
		val->val1 = ival / 1000000;
		val->val2 = ival % 1000000;
		//LOG_INF("Field = val1:%d, val2:%d", val->val1, val->val2);
	}
	else if (chan == SENSOR_CHAN_AMBIENT_TEMP) {
		uval = drv_data->temperature;
		float temperature_raw = (-3.83e-6 * (uval * uval)) + (0.16094 * uval) - 279.80;

		/*
		LOG_INF("uval=0x%x", uval);
		LOG_INF("temperature_raw=%f offset_raw=0x%02x gain_raw=0x%02x",
			temperature_raw,
			(unsigned int)drv_data->temp_offset, (unsigned int)drv_data->temp_gain);
		*/
		float offset = drv_data->temp_offset/16;
		float gain = 1+(drv_data->temp_gain/2048);
		float temperature = gain * temperature_raw + offset - (0.222 * 3.3);
		uval = 1000000 * temperature;
		//LOG_INF("calibrated temperature = %f uval=%lu", temperature, (unsigned long)uval);

		val->val1 = uval / 1000000;
		val->val2 = uval % 1000000;
		//LOG_INF("Temp = val1:%ld, val2:%ld", (long int)val->val1, (long int)val->val2);

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
	int8_t svalue;

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

	if (si7210_reg_read(drv_data, SI7210_REG_TEMP_OFFSET, &svalue) != 0) {
		LOG_ERR("Temp offset read failed");
		return -EIO;
	}
	drv_data->temp_offset = svalue;
	LOG_INF("Si7210 calibration offset is 0x%02x", (int)svalue);
	

	if (si7210_reg_read(drv_data, SI7210_REG_TEMP_GAIN, &svalue) != 0) {
		LOG_ERR("Temp gain read failed");
		return -EIO;
	}
	drv_data->temp_gain = svalue;
	LOG_INF("Si7210 calibration gain is  0x%02x", (int)svalue);

	LOG_INF("si7210_chip_init success\n");
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
