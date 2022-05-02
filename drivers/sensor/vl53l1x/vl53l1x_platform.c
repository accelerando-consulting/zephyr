/* vl53l1x_platform.c - Zephyr customization of ST vl53l1x library.
 * (library is located in ext/hal/st/lib/sensor/vl53l1x/)
 */

/*
 * Copyright (c) 2017 STMicroelectronics
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "vl53l1x_platform.h"

#include <drivers/sensor.h>
#include <kernel.h>
#include <device.h>
#include <init.h>
#include <drivers/i2c.h>
#include <logging/log.h>

LOG_MODULE_DECLARE(VL53L1X, CONFIG_SENSOR_LOG_LEVEL);

VL53L1X_Error VL53L1X_WriteMulti(VL53L1X_DEV Dev, uint16_t index, uint8_t *pdata,
				 uint32_t count)
{

	VL53L1X_Error Status = VL53L1X_ERROR_NONE;
	int32_t status_int = 0;
	uint8_t I2CBuffer[count+1];

	I2CBuffer[0] = index >> 8;
	I2CBuffer[1] = index & 0x00FF;
	memcpy(&I2CBuffer[2], pdata, count);

	status_int = i2c_write(Dev->i2c, I2CBuffer, count+2, Dev->I2cDevAddr);

	if (status_int < 0) {
		Status = VL53L1X_ERROR_CONTROL_INTERFACE;
		LOG_ERR("Failed to write");
	}

	return Status;
}

VL53L1X_Error VL53L1X_ReadMulti(VL53L1X_DEV Dev, uint16_t index, uint8_t *pdata,
				uint32_t count)
{
	VL53L1X_Error Status = VL53L1X_ERROR_NONE;
	int32_t status_int;

	uint8_t regspec[2] = { index >>8, index & 0x00FF };

	status_int = i2c_write_read(Dev->i2c, Dev->I2cDevAddr,
				    regspec, sizeof(regspec), pdata, count);
	if (status_int < 0) {
		LOG_ERR("Failed to read");
		return -EIO;
	}

	return Status;
}


VL53L1X_Error VL53L1X_WrByte(VL53L1X_DEV Dev, uint16_t index, uint8_t data)
{
	VL53L1X_Error Status = VL53L1X_ERROR_NONE;
	int32_t status_int;
	uint8_t I2CBuffer[3] = { index >> 8, index & 0x00FF, data };

	status_int = i2c_write(Dev->i2c, I2CBuffer, sizeof(I2CBuffer), Dev->I2cDevAddr);

	if (status_int < 0) {
		Status = VL53L1X_ERROR_CONTROL_INTERFACE;
		LOG_ERR("i2c_reg_write_byte failed (%d)", Status);
	}

	return Status;
}

VL53L1X_Error VL53L1X_WrWord(VL53L1X_DEV Dev, uint16_t index, uint16_t data)
{
	VL53L1X_Error Status = VL53L1X_ERROR_NONE;
	int32_t status_int;
	uint8_t I2CBuffer[4];

	I2CBuffer[0] = index >> 8;
	I2CBuffer[1] = index & 0x00FF;
	I2CBuffer[2] = data >> 8;
	I2CBuffer[3] = data & 0x00FF;

	status_int = i2c_write(Dev->i2c, I2CBuffer, sizeof(I2CBuffer), Dev->I2cDevAddr);
	if (status_int < 0) {
		Status = VL53L1X_ERROR_CONTROL_INTERFACE;
		LOG_ERR("i2c_write failed (%d)", Status);
	}

	return Status;
}

VL53L1X_Error VL53L1X_WrDWord(VL53L1X_DEV Dev, uint16_t index, uint32_t data)
{
	VL53L1X_Error Status = VL53L1X_ERROR_NONE;
	int32_t status_int;
	uint8_t I2CBuffer[6];

	I2CBuffer[0] = index >> 8;
	I2CBuffer[1] = index & 0x00FF;
	I2CBuffer[2] = (data >> 24) & 0xFF;
	I2CBuffer[3] = (data >> 16) & 0xFF;
	I2CBuffer[4] = (data >> 8)  & 0xFF;
	I2CBuffer[5] = (data >> 0) & 0xFF;

	status_int = i2c_write(Dev->i2c, I2CBuffer, sizeof(I2CBuffer), Dev->I2cDevAddr);
	if (status_int < 0) {
		Status = VL53L1X_ERROR_CONTROL_INTERFACE;
		LOG_ERR("i2c_write failed (%d)", Status);
	}

	return Status;
}

VL53L1X_Error VL53L1X_UpdateByte(VL53L1X_DEV Dev, uint16_t index,
				 uint8_t AndData, uint8_t OrData)
{
	VL53L1X_Error Status = VL53L1X_ERROR_NONE;
	int32_t status_int;
	uint8_t deviceAddress;
	uint8_t data;

	deviceAddress = Dev->I2cDevAddr;

	status_int = VL53L1X_RdByte(Dev, index, &data);
	if (status_int < 0) {
		Status = VL53L1X_ERROR_CONTROL_INTERFACE;
		LOG_ERR("VL53L1X_RdByte failed (%d)", Status);
	}

	if (Status == VL53L1X_ERROR_NONE) {
		data = (data & AndData) | OrData;
		status_int = VL53L1X_WrByte(Dev, index, data);
		if (status_int != 0) {
			Status = VL53L1X_ERROR_CONTROL_INTERFACE;
			LOG_DBG("VL53L1X_WrByte failed.(%d)", Status);
		}
	}

	return Status;
}

VL53L1X_Error VL53L1X_RdByte(VL53L1X_DEV Dev, uint16_t index, uint8_t *data)
{
	VL53L1X_Error Status = VL53L1X_ERROR_NONE;
	int32_t status_int;
	uint8_t regspec[2] = { index >>8, index & 0x00FF };

	status_int = i2c_write_read(Dev->i2c, Dev->I2cDevAddr,
				    regspec, sizeof(regspec), data, 1);
	if (status_int < 0) {
		Status = VL53L1X_ERROR_CONTROL_INTERFACE;
		LOG_ERR("i2c_write_read failed (err %d, return %d)", status_int, Status);
	}

	return Status;
}

VL53L1X_Error VL53L1X_RdWord(VL53L1X_DEV Dev, uint16_t index, uint16_t *data)
{
	VL53L1X_Error Status = VL53L1X_ERROR_NONE;
	int32_t status_int;
	uint8_t regspec[2] = { index >>8, index & 0x00FF };
	uint8_t buf[2];

	status_int = i2c_write_read(Dev->i2c, Dev->I2cDevAddr,
				    regspec, sizeof(regspec), buf, sizeof(buf));
	if (status_int < 0) {
		LOG_ERR("i2c_write_read failed (%d)", status_int);
		return -EIO;
	}
	*data = ((uint16_t)buf[0]<<8) + (uint16_t)buf[1];

	return Status;
}

VL53L1X_Error  VL53L1X_RdDWord(VL53L1X_DEV Dev, uint16_t index, uint32_t *data)
{
	VL53L1X_Error Status = VL53L1X_ERROR_NONE;
	int32_t status_int;
	uint8_t regspec[2] = { index >>8, index & 0x00FF };
	uint8_t buf[4];

	status_int = i2c_write_read(Dev->i2c, Dev->I2cDevAddr,
				    regspec, sizeof(regspec), buf, sizeof(buf));
	if (status_int < 0) {
		LOG_ERR("i2c_burst_read failed (err %d)", status_int);
		return -EIO;
	}
	*data = ((uint32_t)buf[0]<<24) + ((uint32_t)buf[1]<<16) +
		((uint32_t)buf[2]<<8) + (uint32_t)buf[3];

	return Status;
}

VL53L1X_Error VL53L1X_PollingDelay(VL53L1X_DEV Dev)
{
	k_sleep(K_MSEC(2));
	return VL53L1X_ERROR_NONE;
}
