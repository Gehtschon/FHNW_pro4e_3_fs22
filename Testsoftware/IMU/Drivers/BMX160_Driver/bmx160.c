/*
 * bmx160.c
 *
 *  Created on: May 9, 2022
 *      Author: Roland Frunz
 */

#include "bmx160.h"

/*********************************************************************/
/* Static function declarations */

/******************************************************************************
 function:	Write one byte of data to BMX160 via I2C
 parameters: dev: struct bmx160_dev
 addr: register address
 value: value to write into the register
 return:     result of API execution status
 retval Zero Success
 retval other than Zero Error
 Info:
 ******************************************************************************/
static uint8_t BMX160_Write_Byte(bmx160_dev *dev, uint8_t addr, uint8_t value) {
	uint8_t buf = value;
	return HAL_I2C_Mem_Write(dev->i2cHandle, (dev->address << 1), addr,
	MEMADDSIZE8, &buf, 1, BMX160_I2C_TIMEOUT_DURATION);
}

/******************************************************************************
 function:	Read one byte of data of BMX160 via I2C
 parameters: dev: struct bmx160_dev
 addr: Register address
 return:     result of API execution status
 retval Zero Success
 retval other than zero Error
 Info:		This API reads the data from the register, stores it in the data pointer passed by the user
 ******************************************************************************/
static uint8_t BMX160_Read_Byte(bmx160_dev *dev, uint8_t addr, uint8_t *data) {
	uint8_t *buf = data;
	return HAL_I2C_Mem_Read(dev->i2cHandle, (dev->address << 1), addr,
	MEMADDSIZE8, buf, 1, BMX160_I2C_TIMEOUT_DURATION);
}

/******************************************************************************
 function:	 Read one word of data of BMX160 via I2C
 parameters: dev: struct bmx160_dev
 addr: register address (the lower of the two)
 data: pointer to data buffer to store the read data
 return:     Result of API execution status
 retval Zero Success
 retval other than zero Error
 Info:		This API reads sensor data, stores it in the data pointer passed by the user
 ******************************************************************************/
static uint8_t BMX160_Read_Word(bmx160_dev *dev, uint8_t addr, uint16_t *data) {
	uint8_t rslt;
	uint8_t data_assembled[2] = { 0, 0 };
	rslt = BMX160_Read_Byte(dev, addr, &data_assembled[0]);
	if (rslt == BMX160_OK) {
		rslt = BMX160_Read_Byte(dev, addr + 1, &data_assembled[1]);
	} else {
		return rslt;
	}
	*data = (data_assembled[1] << 8) | (data_assembled[0]);
	return rslt;
}

/******************************************************************************
 function:	 get status confirmation from indirect read/write operation
 parameters: dev: struct bmx160_dev
 return:     Result of API execution status
 retval Zero Success
 retval other than zero Error
 Info:		This API waits for the bit mag_man_op of register Status to become 0.
 This means the indirect read/write command has been completed.
 ******************************************************************************/
static uint8_t bmx160_get_status_conf(bmx160_dev *dev) {
	uint8_t rslt;
	uint8_t status = 1;
	uint8_t timeout_status = BMX160_STATUS_TIMEOUT;
	while (status && BMX160_STATUS_TIMEOUT) {
		rslt = BMX160_Read_Byte(dev, BMX160_STATUS_ADDR, &status);
		if (rslt == BMX160_OK) {
			status = status & BMX160_STATUS_COMPLETED_MASK;
			timeout_status--;
		} else {
			return rslt;
		}
	}
	if (timeout_status == 0) {
		return rslt = BMX160_TIMEOUT;
	} else {
		return rslt;
	}
}

/*!
 *  @brief This API is the entry point for sensor.It performs
 *  the selection of I2C/SPI read mechanism according to the
 *  selected interface and reads the chip-id of bmi160 sensor.
 */

/*********************** User function definitions ****************************/

/******************************************************************************
 function:	 initialization of the BMX160 interface
 parameters: dev: struct bmx160_dev
 hi2c1: I2C_HandleTypeDef from configurated I2C
 return:     Result of API execution status
 retval Zero Success
 retval other than zero Error
 Info:		-
 ******************************************************************************/
uint8_t bmx160_if_init(bmx160_dev *dev, I2C_HandleTypeDef *hi2c1) {
	uint8_t rslt;
	uint8_t chip_id;
	dev->address = BMX160_ADDR_I2C;
	dev->i2cHandle = hi2c1;
	rslt = BMX160_Read_Byte(dev, BMX160_CHIP_ID_ADDR, &chip_id);
	if ((rslt == BMX160_OK)&&(chip_id==BMX160_CHIP_ID)) {
		dev->chip_id = chip_id;
	}
	return rslt;
}

/******************************************************************************
 function:	 initialization of the BMX160 magnetometer sensor interface
 parameters: dev: struct bmx160_dev
 return:     Result of API execution status
 retval Zero Success
 retval other than zero Error
 Info:		This API initializes the sensor acc. to datasheet p.25
 initialize magnetometer to low power preset at 12.5Hz and enable magnetometer interface data mode
 ******************************************************************************/
uint8_t bmx160_mag_init(bmx160_dev *dev) {
	uint8_t rslt;
	rslt = BMX160_Write_Byte(dev, BMX160_COMMAND_REG_ADDR,
	BMX160_MAG_NORMAL_MODE);
	if (rslt != BMX160_OK) {
		return rslt;
	} else {
		HAL_Delay(BMX160_WAIT_ONE_MS);
	}
	rslt = BMX160_Write_Byte(dev, BMX160_MAG_IF0_ADDR,
	BMX160_MANUAL_MODE_EN_MSK);
	if (rslt != BMX160_OK) {
		return rslt;
	}
	rslt = BMX160_Write_Byte(dev, BMX160_MAG_IF3_WRITE_DATA_ADDR, 0x01);
	if (rslt != BMX160_OK) {
		return rslt;
	}
	rslt = BMX160_Write_Byte(dev, BMX160_MAG_IF2_WRITE_ADDR_ADDR, 0x4B);
	if (rslt != BMX160_OK) {
		return rslt;
	}
	rslt = bmx160_get_status_conf(dev);
	if (rslt != BMX160_OK) {
		return rslt;
	}
	rslt = BMX160_Write_Byte(dev, BMX160_MAG_IF3_WRITE_DATA_ADDR,
	BMX160_MAG_REPXY_LOW_POWER_PRESET);
	if (rslt != BMX160_OK) {
		return rslt;
	}
	rslt = BMX160_Write_Byte(dev, BMX160_MAG_IF2_WRITE_ADDR_ADDR,
	BMX160_INT_ENABLE_1_ADDR);
	if (rslt != BMX160_OK) {
		return rslt;
	}
	rslt = bmx160_get_status_conf(dev);
	if (rslt != BMX160_OK) {
		return rslt;
	}
	rslt = BMX160_Write_Byte(dev, BMX160_MAG_IF3_WRITE_DATA_ADDR,
	BMX160_MAG_REPZ_REGULAR_PRESET);
	if (rslt != BMX160_OK) {
		return rslt;
	}
	rslt = BMX160_Write_Byte(dev, BMX160_MAG_IF2_WRITE_ADDR_ADDR,
	BMX160_INT_ENABLE_2_ADDR);
	if (rslt != BMX160_OK) {
		return rslt;
	}
	rslt = bmx160_get_status_conf(dev);
	if (rslt != BMX160_OK) {
		return rslt;
	}
	rslt = bmx160_mag_setup2datamode(dev);
	if (rslt != BMX160_OK) {
		return rslt;
	}
	rslt = BMX160_Write_Byte(dev, BMX160_MAG_CONF_ADDR,
	BMX160_MAG_ODR_12_5HZ);
	if (rslt != BMX160_OK) {
		return rslt;
	}
	rslt = BMX160_Write_Byte(dev, BMX160_MAG_IF0_ADDR,
	BMX160_DATA_MODE_EN_MSK);
	if (rslt != BMX160_OK) {
		return rslt;
	}
	rslt = BMX160_Write_Byte(dev, BMX160_COMMAND_REG_ADDR,
	BMX160_MAG_LOWPOWER_MODE);
	if (rslt != BMX160_OK) {
		return rslt;
	} else {
		HAL_Delay(BMX160_WAIT_ONE_MS);
	}

	return rslt;
}

/******************************************************************************
 function:	 switch magnetometer interface from setup mode to data mode
 parameters: dev: struct bmx160_dev
 return:     Result of API execution status
 retval Zero Success
 retval other than zero Error
 Info:		 see also datasheet p.24
 ******************************************************************************/
uint8_t bmx160_mag_setup2datamode(bmx160_dev *dev) {
	uint8_t rslt;
	rslt = BMX160_Write_Byte(dev, BMX160_MAG_IF3_WRITE_DATA_ADDR, 0x02);
	if (rslt != BMX160_OK) {
		return rslt;
	}
	rslt = BMX160_Write_Byte(dev, BMX160_MAG_IF2_WRITE_ADDR_ADDR, 0x4C);
	if (rslt != BMX160_OK) {
		return rslt;
	}
	rslt = BMX160_Write_Byte(dev, BMX160_MAG_IF1_READ_ADDR_ADDR, 0x42);
	if (rslt != BMX160_OK) {
		return rslt;
	}
	return rslt;
}

