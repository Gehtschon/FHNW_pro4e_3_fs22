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
 reg: register address
 value: value to write into the register
 return:     result of API execution status
 retval Zero Success
 retval other than Zero Error
 Info:
 ******************************************************************************/
static uint8_t BMX160_write_byte(bmx160_dev *dev, uint8_t reg, uint8_t value) {
	uint8_t buf = value;
	return HAL_I2C_Mem_Write(dev->i2cHandle, (dev->address << 1), reg,
	MEMADDSIZE8, &buf, 1, BMX160_I2C_TIMEOUT_DURATION);
}

/******************************************************************************
 function:	Read one byte of data of BMX160 via I2C
 parameters: dev: struct bmx160_dev
 reg: Register address
 *data: pointer to a uint8_t variable where the read data can be stored
 return:     result of API execution status
 retval Zero Success
 retval other than zero Error
 Info:		This API reads the data from the register, stores it in the data pointer passed by the user
 ******************************************************************************/
static uint8_t BMX160_read_byte(bmx160_dev *dev, uint8_t reg, uint8_t *data) {
	uint8_t *buf = data;
	return HAL_I2C_Mem_Read(dev->i2cHandle, (dev->address << 1), reg,
	MEMADDSIZE8, buf, 1, BMX160_I2C_TIMEOUT_DURATION);
}

/******************************************************************************
 function:	 Read one word of data of BMX160 via I2C
 parameters: dev: struct bmx160_dev
 reg: register address (the lower of the two)
 data: pointer to data buffer to store the read data
 return:     Result of API execution status
 retval Zero Success
 retval other than zero Error
 Info:		This API reads sensor data, stores it in the data pointer passed by the user
 ******************************************************************************/
static uint8_t BMX160_read_word(bmx160_dev *dev, uint8_t reg, uint16_t *data) {
	uint8_t rslt;
	uint8_t data_assembled[2] = { 0, 0 };
	rslt = BMX160_read_byte(dev, reg, &data_assembled[0]);
	if (rslt == BMX160_OK) {
		rslt = BMX160_read_byte(dev, reg + 1, &data_assembled[1]);
	} else {
		return rslt;
	}
	*data = (data_assembled[1] << 8) | (data_assembled[0]);
	return rslt;
}

/******************************************************************************
 function:	 wait until the expected bit of a register falls to 0
 parameters: dev: struct bmx160_dev
 reg: register address
 bit_number: bit number (range 0 to 7) in register to observe and wait to become 0
 try: number of tries until API is quit without result (timeout)
 return:     Result of API execution status
 retval Zero Success
 retval other than zero Error
 Info:		This API waits for the bit number given in bit_number in the register reg to become 0.
 After the number of loops in try has been reached, the API is quit with a timeout.
 ******************************************************************************/
static uint8_t bmx160_wait_conf(bmx160_dev *dev, uint8_t reg,
		uint8_t bit_number, uint8_t try) {
	uint8_t rslt;
	uint8_t read_data;
	while (try--) {
		rslt = BMX160_read_byte(dev, reg, &read_data);
		if ((rslt == BMX160_OK) && (read_data & (0 << bit_number))) {
			return rslt;
		}
	}
	if (try == 0) {
		return BMX160_TIMEOUT;
	} else {
		return rslt;
	}
}

/******************************************************************************
 function:	 perform an indirect write operation via the magnetometer interface registers
 parameters: dev: struct bmx160_dev
 reg: Register address
 value: value to write into the register
 return:     Result of API execution status
 retval Zero Success
 retval other than zero Error
 Info:		This API performs an indirect write operation.
 The data to be written is written into register 0x4F MAG_IF[3]
 ******************************************************************************/
static uint8_t bmx160_indirect_write_byte(bmx160_dev *dev, uint8_t reg,
		uint8_t value) {
	uint8_t rslt;
	rslt = BMX160_write_byte(dev, BMX160_MAG_IF3_WRITE_DATA_ADDR, value);
	if (rslt == BMX160_OK) {
		rslt = BMX160_write_byte(dev, BMX160_MAG_IF2_WRITE_ADDR_ADDR, reg);
	}
	if (rslt == BMX160_OK) {
		rslt = bmx160_wait_conf(dev, BMX160_STATUS_ADDR,
		BMX160_MAG_MAN_OP_BIT_NUMBER, BMX160_MAG_WAIT_STATUS);
		printf("\nrslt: %d\r", rslt);
	}
	return rslt;
}
/******************************************************************************
 function:	 perform an indirect read operation of the magnetometer interface registers
 parameters: dev: struct bmx160_dev
 reg: Register address
 *data: pointer to a uint8_t variable where the read data can be stored
 return:     Result of API execution status
 retval Zero Success
 retval other than zero Error
 Info:		This API performs an indirect read operation.
 The address to be read is written into register 0x4D MAG_IF[1]
 ******************************************************************************/
/*
 static uint8_t bmx160_indirect_read_byte(bmx160_dev *dev, uint8_t reg,
 uint8_t *data) {
 uint8_t rslt;
 rslt = BMX160_write_byte(dev, BMX160_MAG_IF1_READ_ADDR_ADDR, reg);
 rslt = bmx160_wait_conf(dev, BMX160_STATUS_ADDR, BMX160_MAG_INDIRECT_WR_STATUS_FINISHED, BMX160_MAG_WAIT_STATUS);
 rslt = BMX160_read_byte(§, reg, data)
 return rslt;
 }
 function not finished, unclear where to read the data indirectly from
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
uint8_t bmx160_if_init(bmx160_dev *dev, I2C_HandleTypeDef *i2c_handle) {
	uint8_t rslt;
	uint8_t chip_id;
	uint8_t try = 3;
	dev->address = BMX160_ADDR_I2C;
	dev->i2cHandle = i2c_handle;
	while ((try--) && (dev->chip_id != BMX160_CHIP_ID)) {
		rslt = BMX160_read_byte(dev, BMX160_CHIP_ID_ADDR, &chip_id);

		if ((rslt == BMX160_OK) && (chip_id == BMX160_CHIP_ID)) {
			dev->chip_id = chip_id;
			break;
		}
	}
	rslt = bmx160_soft_reset(dev);
	return rslt;
}

/******************************************************************************
 function:	 reset and restart the device
 parameters: dev: struct bmx160_dev
 return:     Result of API execution status
 retval Zero Success
 retval other than zero Error
 Info:		 It triggers a reset including a reboot. Other values are ignored.
 Following a delay, all user configuration settings are overwritten with their
 default state or the setting stored in the NVM, wherever applicable.
 ******************************************************************************/
uint8_t bmx160_soft_reset(bmx160_dev *dev) {
	uint8_t rslt;
	rslt = BMX160_write_byte(dev, BMX160_COMMAND_REG_ADDR,
	BMX160_SOFT_RESET_CMD);
	HAL_Delay(BMX160_SOFT_RESET_DELAY_MS);
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
// mag if to normal mode
	rslt = BMX160_write_byte(dev, BMX160_COMMAND_REG_ADDR,
	BMX160_MAG_NORMAL_MODE);

	if (rslt == BMX160_OK) {
		HAL_Delay(BMX160_WAIT_ONE_MS);
	}
// mag if to setup mode and mag-offset to maximum offset
	if (rslt == BMX160_OK) {
		rslt = BMX160_write_byte(dev, BMX160_MAG_IF0_ADDR,
		BMX160_MAG_SETUP_MODE_EN_MSK);
	}
// indirect write 0x01 to mag register 0x4B, put mag into sleep mode
	if (rslt == BMX160_OK) {
		rslt = bmx160_indirect_write_byte(dev, 0x4B, 0x01);
	}
// indirect set of low power preset for repxy, temp. on high accuracy *WATCH OUT*
	if (rslt == BMX160_OK) {
		rslt = bmx160_indirect_write_byte(dev, BMX160_INT_ENABLE_1_ADDR,
		BMX160_MAG_REPXY_HIGH_ACCURACY_PRESET);
	}
// indirect set of regular preset for repz, temp. on high accuracy *WATCH OUT*
	if (rslt == BMX160_OK) {
		rslt = bmx160_indirect_write_byte(dev, BMX160_INT_ENABLE_2_ADDR,
		BMX160_MAG_REPZ_HIGH_ACCURACY_PRESET);
	}
// prepare MAG_IF[1-3] for mag_if data mode
	if (rslt == BMX160_OK) {
		rslt = bmx160_mag_setup2datamode(dev);
	}
// set output data rate ODR to 12.5 Hz
	if (rslt == BMX160_OK) {
		rslt = BMX160_write_byte(dev, BMX160_MAG_CONF_ADDR,
		BMX160_MAG_ODR_12_5HZ);
	}
// mag_if data mode and maximum offset
	if (rslt == BMX160_OK) {
		rslt = BMX160_write_byte(dev, BMX160_MAG_IF0_ADDR,
		BMX160_MAG_DATA_MODE_EN_MSK);
	}
// put mag_if to low power mode
	if (rslt == BMX160_OK) {
		rslt = BMX160_write_byte(dev, BMX160_COMMAND_REG_ADDR,
		BMX160_MAG_LOWPOWER_MODE);
	}
// delay to make sure all the settings are stored
	if (rslt == BMX160_OK) {
		HAL_Delay(BMX160_WAIT_ONE_MS);
	}
	return rslt;
}

/******************************************************************************
 function:	 initialization of the BMX160 acceleration sensor interface
 parameters: dev: struct bmx160_dev
 return:     Result of API execution status
 retval Zero Success
 retval other than zero Error
 Info:		This API initializes the acceleration sensor acc. to datasheet p.25
 ******************************************************************************/
uint8_t bmx160_acc_init(bmx160_dev *dev) {
	uint8_t rslt;
	uint8_t acc_data_rate_setting = BMX160_ACC_ODR_12_5HZ;
	uint8_t acc_bw_setting = BMX160_ACC_BW_NORMAL_AVG4;
	uint8_t acc_conf_data = (acc_bw_setting << 4) | (acc_data_rate_setting);
	/* Write output data rate and bandwidth */
	rslt = BMX160_write_byte(dev, BMX160_ACC_CONFIG_ADDR, acc_conf_data);

	if (rslt == BMX160_OK) {
		rslt = BMX160_write_byte(dev, BMX160_ACC_RANGE_ADDR,
				BMX160_ACC_RANGE_2G);
	}
	if (rslt == BMX160_OK) {
		rslt = BMX160_write_byte(dev, BMX160_COMMAND_REG_ADDR,
				BMX160_ACC_NORMAL_MODE);
	}

	if (rslt == BMX160_OK) {
		HAL_Delay(BMX160_ACCEL_DELAY_MS);
	}
}

/******************************************************************************
 function:	 perform a normal self-test of the magnetometer acc. to datasheet p.49
 parameters: dev: struct bmx160_dev
 return:     Result of API execution status
 retval Zero Success
 retval other than zero Error
 Info:		During normal self-test, the following verifications are performed:
 - FlipCore signal path is verified by generating signals on-chip. These are processed
 through the signal path and the measurement result is compared to known thresholds.
 - FlipCore (X and Y) connection to ASIC are checked for connectivity and short circuits.
 - Hall sensor connectivity is checked for open and shorted connections.
 - Hall sensor signal path and hall sensor element offset are checked for overflow
 ******************************************************************************/
/*
 uint8_t bmx160_mag_self_test(bmx160_dev *dev) {
 uint8_t rslt;
 // put mag if into normal mode
 BMX160_write_byte(dev, BMX160_COMMAND_REG_ADDR, BMX160_MAG_NORMAL_MODE);
 // enter setup-mode
 if (rslt == BMX160_OK) {
 BMX160_write_byte(dev, BMX160_MAG_IF0_ADDR,
 BMX160_MAG_SETUP_MODE_EN_MSK);
 }
 // indirect write 0x01 to mag register 0x4B, put mag into sleep mode
 if (rslt == BMX160_OK) {
 rslt = BMX160_write_byte(dev, BMX160_MAG_IF3_WRITE_DATA_ADDR, 0x01);
 }
 if (rslt == BMX160_OK) {
 rslt = BMX160_write_byte(dev, BMX160_MAG_IF2_WRITE_ADDR_ADDR, 0x4B);
 }
 // enter self-test: indirect mag register write to 0x4C Bit0 to 1
 if (rslt == BMX160_OK) {
 rslt = bmx160_indirect_write_byte(dev, BMX160_MAG_IF0_ADDR, BMX160_MAG_SELF_TEST_EN_MSK);
 }
 HAL_Delay(BMX160_DELAY_NORMAL_SELF_TEST);
 // check if bit0 of MAG_IF_0 is set back to 0. This means the self-test is finished
 bmx160_wait_conf(dev, reg, bit_number, try)

 while ((try--) && (dev->chip_id != BMX160_CHIP_ID)) {
 rslt = BMX160_read_byte(dev, BMX160_CHIP_ID_ADDR, &chip_id);

 if ((rslt == BMX160_OK) && (chip_id == BMX160_CHIP_ID)) {
 dev->chip_id = chip_id;
 }
 }

 data_write = 0b10000001; // write the write data into Register (0x4F) MAG_IF[3]
 HAL_I2C_Mem_Write(&hi2c1, (bmi160dev.id << 1), 0x4F, memAddSize8,
 &data_write, size, timeout);
 data_write = 0x4C; // write magnetometer register address to write into Register (0x4E) MAG_IF[2]
 HAL_I2C_Mem_Write(&hi2c1, (bmi160dev.id << 1), 0x4E, memAddSize8,
 &data_write, size, timeout);
 // read Register (0x1B) STATUS until the bit mag_man_op is “0” to confirm the write access has been completed
 uint8_t status = 0x00;
 uint8_t timeout_status = 50;

 while ((!status) && (timeout_status)) {	// wait until the write access has been confirmed
 HAL_I2C_Mem_Read(&hi2c1, (bmi160dev.id << 1), 0x1B, memAddSize8,
 &status, size, timeout);
 status = status & (0b00000100);
 timeout_status--;
 };
 // check end of self-test: indirect mag register read to 0x4C Bit0 to 1
 data_write = 0x4C;// write magnetometer register address to read from into Register (0x4D) MAG_IF[1]
 HAL_I2C_Mem_Write(&hi2c1, (bmi160dev.id << 1), 0x4D, memAddSize8,
 &data_write, size, timeout);
 data_read = 0x4C;// read register (0x1B) STATUS until the bit mag_man_op is “0”
 HAL_I2C_Mem_Write(&hi2c1, (bmi160dev.id << 1), 0x4E, memAddSize8,
 &data_write, size, timeout);
 }
 function not finished, unclear where to read from the data in the end
 */

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
	rslt = BMX160_write_byte(dev, BMX160_MAG_IF3_WRITE_DATA_ADDR, 0x02);
	if (rslt == BMX160_OK) {
		rslt = BMX160_write_byte(dev, BMX160_MAG_IF2_WRITE_ADDR_ADDR, 0x4C);
	}
	if (rslt == BMX160_OK) {
		rslt = BMX160_write_byte(dev, BMX160_MAG_IF1_READ_ADDR_ADDR, 0x42);
	}
	return rslt;
}

/******************************************************************************
 function:	 reads the magnetometer data in registers 0x04 to 0x09
 parameters: dev: struct bmx160_dev
 *data: pointer to a uint16_t variable where the read data can be stored
 return:     Result of API execution status
 retval Zero Success
 retval other than zero Error
 Info:		 see also datasheet p.24
 ******************************************************************************/
uint8_t bmx160_mag_read_data(bmx160_dev *dev, uint16_t *data_x,
		uint16_t *data_y, uint16_t *data_z) {
	uint8_t rslt;
	uint8_t reg_status;
	uint8_t mag_x[2] = { 0, 0 };
	uint8_t mag_y[2] = { 0, 0 };
	uint8_t mag_z[2] = { 0, 0 };
// check if data is ready
	rslt = BMX160_read_byte(dev, BMX160_STATUS_ADDR, &reg_status);
	if ((rslt == BMX160_OK) && (reg_status & BMX160_MAG_DATA_RDY_MSK)) {
		// read each magnetometer register (2 per channel)
		rslt = BMX160_read_byte(dev, BMX160_MAG_X_DATA_LSB_ADDR, &mag_x[0]);
		if (rslt == BMX160_OK) {
			rslt = BMX160_read_byte(dev, BMX160_MAG_X_DATA_MSB_ADDR, &mag_x[1]);
		}
		if (rslt == BMX160_OK) {
			rslt = BMX160_read_byte(dev, BMX160_MAG_Y_DATA_LSB_ADDR, &mag_y[0]);
		}
		if (rslt == BMX160_OK) {
			rslt = BMX160_read_byte(dev, BMX160_MAG_Y_DATA_MSB_ADDR, &mag_y[1]);
		}
		if (rslt == BMX160_OK) {
			rslt = BMX160_read_byte(dev, BMX160_MAG_Z_DATA_LSB_ADDR, &mag_z[0]);
		}
		if (rslt == BMX160_OK) {
			rslt = BMX160_read_byte(dev, BMX160_MAG_Z_DATA_MSB_ADDR, &mag_z[1]);
		}
		*data_x = (int16_t) ((mag_x[1] << 8) | (mag_x[0]));
		*data_y = (int16_t) ((mag_y[1] << 8) | (mag_y[0]));
		*data_z = (int16_t) ((mag_z[1] << 8) | (mag_z[0]));
	}

	return rslt;
}

/******************************************************************************
 function:	 reads the accelerometer data in registers 0x12 to 0x17
 parameters: dev: struct bmx160_dev
 *data: pointer to a uint16_t variable where the read data can be stored
 return:     Result of API execution status
 retval Zero Success
 retval other than zero Error
 Info:
 ******************************************************************************/
uint8_t bmx160_acc_read_data(bmx160_dev *dev, uint16_t *data_x,
		uint16_t *data_y, uint16_t *data_z) {
	uint8_t rslt;
	uint8_t reg_status;
	uint8_t acc_x[2] = { 0, 0 };
	uint8_t acc_y[2] = { 0, 0 };
	uint8_t acc_z[2] = { 0, 0 };
// check if data is ready
	rslt = BMX160_read_byte(dev, BMX160_STATUS_ADDR, &reg_status);
	if ((rslt == BMX160_OK) && (reg_status & BMX160_ACC_DATA_RDY_MSK)) {
		// read each magnetometer register (2 per channel)
		rslt = BMX160_read_byte(dev, BMX160_ACC_X_DATA_LSB_ADDR, &acc_x[0]);
		if (rslt == BMX160_OK) {
			rslt = BMX160_read_byte(dev, BMX160_ACC_X_DATA_MSB_ADDR, &acc_x[1]);
		}
		if (rslt == BMX160_OK) {
			rslt = BMX160_read_byte(dev, BMX160_ACC_Y_DATA_LSB_ADDR, &acc_y[0]);
		}
		if (rslt == BMX160_OK) {
			rslt = BMX160_read_byte(dev, BMX160_ACC_Y_DATA_MSB_ADDR, &acc_y[1]);
		}
		if (rslt == BMX160_OK) {
			rslt = BMX160_read_byte(dev, BMX160_ACC_Z_DATA_LSB_ADDR, &acc_z[0]);
		}
		if (rslt == BMX160_OK) {
			rslt = BMX160_read_byte(dev, BMX160_ACC_Z_DATA_MSB_ADDR, &acc_z[1]);
		}
		*data_x = (int16_t) ((acc_x[1] << 8) | (acc_x[0]));
		*data_y = (int16_t) ((acc_y[1] << 8) | (acc_y[0]));
		*data_z = (int16_t) ((acc_z[1] << 8) | (acc_z[0]));
	}

	return rslt;
}
