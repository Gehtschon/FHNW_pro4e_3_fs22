/*
 * bmx160.h
 *
 *  Created on: May 9, 2022
 *      Author: Roland Frunz
 */

#ifndef INC_BMX160_H_
#define INC_BMX160_H_

#include <math.h>
#include <string.h>
#include <stdlib.h>
#include "stdio.h"
#include "stm32l0xx_hal.h" /* Needed for I2C */

/*********************** DEFINES ************************/

/** BMX160 Register map, see also datasheet p.53/54*/
#define BMX160_CHIP_ID_ADDR                       UINT8_C(0x00)
#define BMX160_ERROR_REG_ADDR                     UINT8_C(0x02)
#define BMX160_PMU_STATUS_ADDR                    UINT8_C(0x03)
#define BMX160_MAG_X_DATA_LSB_ADDR                UINT8_C(0x04)
#define BMX160_MAG_X_DATA_MSB_ADDR                UINT8_C(0x05)
#define BMX160_MAG_Y_DATA_LSB_ADDR                UINT8_C(0x06)
#define BMX160_MAG_Y_DATA_MSB_ADDR                UINT8_C(0x07)
#define BMX160_MAG_Z_DATA_LSB_ADDR                UINT8_C(0x08)
#define BMX160_MAG_Z_DATA_MSB_ADDR                UINT8_C(0x09)
#define BMX160_ACC_X_DATA_LSB_ADDR                UINT8_C(0x12)
#define BMX160_ACC_X_DATA_MSB_ADDR                UINT8_C(0x13)
#define BMX160_ACC_Y_DATA_LSB_ADDR                UINT8_C(0x14)
#define BMX160_ACC_Y_DATA_MSB_ADDR                UINT8_C(0x15)
#define BMX160_ACC_Z_DATA_LSB_ADDR                UINT8_C(0x16)
#define BMX160_ACC_Z_DATA_MSB_ADDR                UINT8_C(0x17)
#define BMX160_SENSOR_TIME_LSB_ADDR               UINT8_C(0x18)
#define BMX160_SENSOR_TIME_XLSB_ADDR          	  UINT8_C(0x19)
#define BMX160_SENSOR_TIME_MSB_ADDR               UINT8_C(0x1A)
#define BMX160_STATUS_ADDR						  UINT8_C(0x1B)
#define BMX160_TEMPERATURE_LSB_ADDR  	          UINT8_C(0x20)
#define BMX160_TEMPERATURE_MSB_ADDR  	          UINT8_C(0x21)
#define BMX160_ACC_CONFIG_ADDR                    UINT8_C(0x40)
#define BMX160_ACC_RANGE_ADDR          		      UINT8_C(0x41)
#define BMX160_MAG_CONF_ADDR   	                  UINT8_C(0x44)
#define BMX160_MAG_IF0_ADDR 		              UINT8_C(0x4C)
#define BMX160_MAG_IF1_READ_ADDR_ADDR             UINT8_C(0x4D)
#define BMX160_MAG_IF2_WRITE_ADDR_ADDR            UINT8_C(0x4E)
#define BMX160_MAG_IF3_WRITE_DATA_ADDR            UINT8_C(0x4F)
#define BMX160_INT_ENABLE_1_ADDR                  UINT8_C(0x51)
#define BMX160_INT_ENABLE_2_ADDR                  UINT8_C(0x52)
#define BMX160_SELF_TEST_ADDR  			          UINT8_C(0x6D)
#define BMX160_COMMAND_REG_ADDR                   UINT8_C(0x7E)

/** Error code definitions */
#define BMX160_OK                                 UINT8_C(0)
#define BMX160_ERROR                              UINT8_C(1)
#define BMX160_BUSY								  UINT8_C(2)
#define BMX160_TIMEOUT						      UINT8_C(3)

/** Mask definitions*/
#define BMX160_STATUS_COMPLETED_MSK				  UINT8_C(0x04)
#define BMX160_MAG_SETUP_MODE_EN_MSK  			  UINT8_C(0x80)
#define BMX160_MAG_DATA_MODE_EN_MSK				  UINT8_C(0x00)
#define BMX160_MAG_SELF_TEST_EN_MSK				  UINT8_C(0x81)
#define BMX160_MAG_INDIRECT_WR_STATUS_FINISHED	  UINT8_C(0x04)

/* Accel power mode */
#define BMX160_ACC_NORMAL_MODE                    UINT8_C(0x11)
#define BMX160_ACC_LOWPOWER_MODE                  UINT8_C(0x12)
#define BMX160_ACC_SUSPEND_MODE                   UINT8_C(0x10)

/** Range settings */
/* Accel Range */
#define BMX160_ACC_RANGE_2G                       UINT8_C(0x03)
#define BMX160_ACC_RANGE_4G                       UINT8_C(0x05)
#define BMX160_ACC_RANGE_8G                       UINT8_C(0x08)
#define BMX160_ACC_RANGE_16G                      UINT8_C(0x0C)

/* Mag power mode */
#define BMX160_MAG_SUSPEND_MODE				      UINT8_C(0x18)
#define BMX160_MAG_NORMAL_MODE                    UINT8_C(0x19)
#define BMX160_MAG_LOWPOWER_MODE                  UINT8_C(0x1A)

#define BMX160_MAG_REPXY_LOW_POWER_PRESET         UINT8_C(0x01)
#define BMX160_MAG_REPXY_REGULAR_PRESET		      UINT8_C(0x04)
#define BMX160_MAG_REPXY_ENHANCED_REGULAR_PRESET  UINT8_C(0x07)
#define BMX160_MAG_REPXY_HIGH_ACCURACY_PRESET     UINT8_C(0x17)

#define BMX160_MAG_REPZ_LOW_POWER_PRESET          UINT8_C(0x02)
#define BMX160_MAG_REPZ_REGULAR_PRESET		      UINT8_C(0x0E)
#define BMX160_MAG_REPZ_ENHANCED_REGULAR_PRESET   UINT8_C(0x1A)
#define BMX160_MAG_REPZ_HIGH_ACCURACY_PRESET      UINT8_C(0x52)

/** Bandwidth settings */
/* Accel Bandwidth */
#define BMX160_ACC_BW_OSR4_AVG1					  UINT8_C(0x00)
#define BMX160_ACC_BW_OSR2_AVG2                   UINT8_C(0x01)
#define BMX160_ACC_BW_NORMAL_AVG4                 UINT8_C(0x02)
#define BMX160_ACC_BW_RES_AVG8                    UINT8_C(0x03)
#define BMX160_ACC_BW_RES_AVG16                   UINT8_C(0x04)
#define BMX160_ACC_BW_RES_AVG32                   UINT8_C(0x05)
#define BMX160_ACC_BW_RES_AVG64                   UINT8_C(0x06)
#define BMX160_ACC_BW_RES_AVG128                  UINT8_C(0x07)

/* Output Data Rate settings */
/* Accel Output data rate */
#define BMX160_ACC_ODR_RESERVED                   UINT8_C(0x00)
#define BMX160_ACC_ODR_0_78HZ                     UINT8_C(0x01)
#define BMX160_ACC_ODR_1_56HZ                     UINT8_C(0x02)
#define BMX160_ACC_ODR_3_12HZ                     UINT8_C(0x03)
#define BMX160_ACC_ODR_6_25HZ                     UINT8_C(0x04)
#define BMX160_ACC_ODR_12_5HZ                     UINT8_C(0x05)
#define BMX160_ACC_ODR_25HZ                       UINT8_C(0x06)
#define BMX160_ACC_ODR_50HZ                       UINT8_C(0x07)
#define BMX160_ACC_ODR_100HZ                      UINT8_C(0x08)
#define BMX160_ACC_ODR_200HZ                      UINT8_C(0x09)
#define BMX160_ACC_ODR_400HZ                      UINT8_C(0x0A)
#define BMX160_ACC_ODR_800HZ                      UINT8_C(0x0B)
#define BMX160_ACC_ODR_1600HZ                     UINT8_C(0x0C)
#define BMX160_ACC_ODR_RESERVED0                  UINT8_C(0x0D)
#define BMX160_ACC_ODR_RESERVED1                  UINT8_C(0x0E)
#define BMX160_ACC_ODR_RESERVED2                  UINT8_C(0x0F)

/* Magnetometer sensor Output data rate */
#define BMX160_MAG_ODR_RESERVED         		  UINT8_C(0x00)
#define BMX160_MAG_ODR_0_78HZ           		  UINT8_C(0x01)
#define BMX160_MAG_ODR_1_56HZ           		  UINT8_C(0x02)
#define BMX160_MAG_ODR_3_12HZ           		  UINT8_C(0x03)
#define BMX160_MAG_ODR_6_25HZ           		  UINT8_C(0x04)
#define BMX160_MAG_ODR_12_5HZ           	      UINT8_C(0x05)
#define BMX160_MAG_ODR_25HZ            		 	  UINT8_C(0x06)
#define BMX160_MAG_ODR_50HZ             		  UINT8_C(0x07)
#define BMX160_MAG_ODR_100HZ            	      UINT8_C(0x08)
#define BMX160_MAG_ODR_200HZ            		  UINT8_C(0x09)
#define BMX160_MAG_ODR_400HZ            		  UINT8_C(0x0A)
#define BMX160_MAG_ODR_800HZ            		  UINT8_C(0x0B)

/* BMX160 read data definitions */
#define BMX160_MAG_DATA_RDY_MSK           		  UINT8_C(0x20)
#define BMX160_ACC_DATA_RDY_MSK           		  UINT8_C(0x80)

/** BMX160 other define */
#define BMX160_CHIP_ID                            UINT8_C(0xD8)
#define BMX160_ADDR_I2C							  UINT8_C(0x68)
#define BMX160_STATUS_TIMEOUT					  UINT8_C(50)
#define BMX160_I2C_TIMEOUT_DURATION				  UINT8_C(32)
#define MEMADDSIZE8 						      I2C_MEMADD_SIZE_8BIT
#define BMX160_MANUAL_MODE_EN_POS  				  UINT8_C(7)
#define BMX160_MAG_WAIT_STATUS  				  UINT8_C(100)
#define BMX160_MAG_MAN_OP_BIT_NUMBER			  UINT8_C(2)

/* Delay in ms settings */
#define BMX160_WAIT_ONE_MS                        UINT8_C(1)
#define BMX160_DELAY_NORMAL_SELF_TEST             UINT8_C(2)
#define BMX160_ACCEL_DELAY_MS					  UINT8_C(5)
#define BMX160_GYRO_DELAY_MS                      UINT8_C(81)
#define BMX160_MAG_COM_DELAY                      UINT8_C(10)
#define BMX160_GYRO_SELF_TEST_DELAY            	  UINT8_C(20)
#define BMX160_ACCEL_SELF_TEST_DELAY              UINT8_C(50)

/** Soft reset command */
#define BMX160_SOFT_RESET_CMD                     UINT8_C(0xB6)
#define BMX160_SOFT_RESET_DELAY_MS                UINT8_C(5)

/** Interface settings */
#define BMX160_SPI_INTF                           UINT8_C(0)
#define BMX160_I2C_INTF                           UINT8_C(1)

/** Utility macros */
#define BMX160_SET_LOW_BYTE     				  UINT16_C(0x00FF)
#define BMX160_SET_HIGH_BYTE					  UINT16_C(0xFF00)

typedef struct {
	/* I2CHandle */
	I2C_HandleTypeDef *i2cHandle;

	/* Chip ID */
	uint8_t chip_id;

	/* Device address */
	uint8_t address;

// activate if used
/*! Structure to configure Accel sensor */
//struct bmi160_cfg accel_cfg;
// activate if used
/*! Structure to hold previous/old accel config parameters.
 * This is used at driver level to prevent overwriting of same
 * data, hence user does not change it in the code */
//	struct bmi160_cfg prev_accel_cfg;
} bmx160_dev;

/*!
 * @brief bmx160 sensor data structure which comprises of accel data
 */
struct bmx160_sensor_data {
	/* X-axis sensor data */
	int16_t x;

	/* Y-axis sensor data */
	int16_t y;

	/* Z-axis sensor data */
	int16_t z;

	/* sensor time */
	uint32_t sensortime;
};

/*********************** User function prototypes ************************/

/**
 * initialization of the BMX160 interface
 */
uint8_t bmx160_if_init(bmx160_dev *dev, I2C_HandleTypeDef *i2c_handle);

/**
 * Reset and restart the device.
 * All register values are overwritten with default parameters.
 */
uint8_t bmx160_soft_reset(bmx160_dev *dev);

/**
 * initialization of the BMX160 magnetometer sensor interface
 */
uint8_t bmx160_mag_init(bmx160_dev *dev);

/**
 * initialization of the BMX160 acceleration sensor interface
 */
uint8_t bmx160_acc_init(bmx160_dev *dev);

/**
 * perform a normal self-test of the magnetometer acc. to datasheet p.49
 */
uint8_t bmx160_mag_self_test(bmx160_dev *dev);

/**
 * switch magnetometer interface from setup mode to data mode
 */
uint8_t bmx160_mag_setup2datamode(bmx160_dev *dev);

/**
 * reads the magnetometer data in registers 0x04 to 0x09
 */
uint8_t bmx160_mag_read_data(bmx160_dev *dev, uint16_t *data_x,
		uint16_t *data_y, uint16_t *data_z);

/**
 * reads the accelerometer data in registers 0x12 to 0x17
 */
uint8_t bmx160_acc_read_data(bmx160_dev *dev, uint16_t *data_x,
		uint16_t *data_y, uint16_t *data_z);

#endif /* INC_BMX160_H_ */
