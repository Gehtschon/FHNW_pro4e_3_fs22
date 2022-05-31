/*
 * LSM303AGR 3D accelerometer and magnetometer 12C Driver
 *
 * Author: Gian-Luca Portmann
 * Created: 19.04.22
 *
 */

#ifndef LSM303AGR_I2C_DRIVER_H
#define LSM303AGR_I2C_DRIVER_H

#include "stm32l0xx_hal.h" /* Needed for I2C */
#include <math.h>
#include <stdio.h>

/*
 * DEFINES
 */

#define LSM303AGR_MAG_I2C_ADDR 	(0x1E << 1) /* Adresslenght is 7 bit (1 bit for read/write) */
#define LSM303AGR_ACC_I2C_ADDR 	(0x19 << 1)

#define LSM303AGR_MAG_WHO_AM_I 	(0x40) /* Who am I. For communication test at startup.*/
#define LSM303AGR_ACC_WHO_AM_I 	(0x33) /* Who am I. For communication test at startup.*/

/*
 * REGISTERS
 */

/* Temperature sensor Registers (New vs lsm303dlhc.h) */
#define LSM303AGR_STATUS_REG_AUX_A          0x07  /* status register */
#define LSM303AGR_OUT_TEMP_L_A              0x0C  /* output temperature register */
#define LSM303AGR_OUT_TEMP_H_A              0x0D  /* output temperature register */
#define LSM303AGR_IN_COUNTER_REG_A          0x0E  /* register */
/* Acceleration Registers */
#define LSM303AGR_WHO_AM_I_ADDR             0x0F  /* device identification register (0x33) */
#define LSM303AGR_WHO_AM_I_A                LSM303AGR_WHO_AM_I_ADDR
/* Temperature sensor Registers */
#define LSM303AGR_TEMP_CFG_REG_A            0x1F  /* temperature configuration register */
/* Acceleration Registers */
#define LSM303AGR_CTRL_REG1_A               0x20  /* Control register 1 acceleration */
#define LSM303AGR_CTRL_REG2_A               0x21  /* Control register 2 acceleration */
#define LSM303AGR_CTRL_REG3_A               0x22  /* Control register 3 acceleration */
#define LSM303AGR_CTRL_REG4_A               0x23  /* Control register 4 acceleration */
#define LSM303AGR_CTRL_REG5_A               0x24  /* Control register 5 acceleration */
#define LSM303AGR_CTRL_REG6_A               0x25  /* Control register 6 acceleration */
#define LSM303AGR_REFERENCE_A               0x26  /* Reference register acceleration */
#define LSM303AGR_STATUS_REG_A              0x27  /* Status register acceleration */
#define LSM303AGR_OUT_X_L_A                 0x28  /* Output Register X acceleration */
#define LSM303AGR_OUT_X_H_A                 0x29  /* Output Register X acceleration */
#define LSM303AGR_OUT_Y_L_A                 0x2A  /* Output Register Y acceleration */
#define LSM303AGR_OUT_Y_H_A                 0x2B  /* Output Register Y acceleration */
#define LSM303AGR_OUT_Z_L_A                 0x2C  /* Output Register Z acceleration */
#define LSM303AGR_OUT_Z_H_A                 0x2D  /* Output Register Z acceleration */
#define LSM303AGR_FIFO_CTRL_REG_A           0x2E  /* Fifo control Register acceleration */
#define LSM303AGR_FIFO_SRC_REG_A            0x2F  /* Fifo src Register acceleration */

#define LSM303AGR_INT1_CFG_A                0x30  /* Interrupt 1 configuration Register acceleration */
#define LSM303AGR_INT1_SOURCE_A             0x31  /* Interrupt 1 source Register acceleration */
#define LSM303AGR_INT1_THS_A                0x32  /* Interrupt 1 Threshold register acceleration */
#define LSM303AGR_INT1_DURATION_A           0x33  /* Interrupt 1 DURATION register acceleration */

#define LSM303AGR_INT2_CFG_A                0x34  /* Interrupt 2 configuration Register acceleration */
#define LSM303AGR_INT2_SOURCE_A             0x35  /* Interrupt 2 source Register acceleration */
#define LSM303AGR_INT2_THS_A                0x36  /* Interrupt 2 Threshold register acceleration */
#define LSM303AGR_INT2_DURATION_A           0x37  /* Interrupt 2 DURATION register acceleration */

#define LSM303AGR_CLICK_CFG_A               0x38  /* Click configuration Register acceleration */
#define LSM303AGR_CLICK_SOURCE_A            0x39  /* Click 2 source Register acceleration */
#define LSM303AGR_CLICK_THS_A               0x3A  /* Click 2 Threshold register acceleration */

#define LSM303AGR_TIME_LIMIT_A              0x3B  /* Time Limit Register acceleration */
#define LSM303AGR_TIME_LATENCY_A            0x3C  /* Time Latency Register acceleration */
#define LSM303AGR_TIME_WINDOW_A             0x3D  /* Time window register acceleration */

/* System Registers(New vs lsm303dlhc.h) */
#define LSM303AGR_Act_THS_A                 0x3E  /* return to sleep activation threshold register */
#define LSM303AGR_Act_DUR_A                 0x3F  /* return to sleep duration register */
/* Magnetometer */
#define LSM303AGR_X_REG_L_M                 0x45  /* Hard-iron X magnetic field */
#define LSM303AGR_X_REG_H_M                 0x46  /* Hard-iron X magnetic field */
#define LSM303AGR_Y_REG_L_M                 0x47  /* Hard-iron Y magnetic field */
#define LSM303AGR_Y_REG_H_M                 0x48  /* Hard-iron Y magnetic field */
#define LSM303AGR_Z_REG_L_M                 0x49  /* Hard-iron Z magnetic field */
#define LSM303AGR_Z_REH_H_M                 0x4A  /* Hard-iron Z magnetic field */
#define LSM303AGR_WHO_AM_I_M                0x4F  /* Who am i register magnetic field (0x40) */
#define LSM303AGR_CFG_REG_A_M               0x60  /* Configuration register A magnetic field */
#define LSM303AGR_CFG_REG_B_M               0x61  /* Configuration register B magnetic field */
#define LSM303AGR_CFG_REG_C_M               0x62  /* Configuration register C magnetic field */
#define LSM303AGR_INT_CTRL_REG_M            0x63  /* interrupt control register magnetic field */
#define LSM303AGR_INT_SOURCE_REG_M          0x64  /* interrupt source register magnetic field */
#define LSM303AGR_INT_THS_L_REG_M           0x65  /* interrupt threshold register magnetic field */
#define LSM303AGR_INT_THS_H_REG_M           0x66  /* interrupt threshold register magnetic field*/
#define LSM303AGR_STATUS_REG_M              0x67  /* Status Register magnetic field */
#define LSM303AGR_OUTX_L_REG_M              0x68  /* Output Register X magnetic field */
#define LSM303AGR_OUTX_H_REG_M              0x69  /* Output Register X magnetic field */
#define LSM303AGR_OUTY_L_REG_M              0x6A  /* Output Register X magnetic field */
#define LSM303AGR_OUTY_H_REG_M              0x6B  /* Output Register X magnetic field */
#define LSM303AGR_OUTZ_L_REG_M              0x6C  /* Output Register X magnetic field */
#define LSM303AGR_OUTZ_H_REG_M              0x6D  /* Output Register X magnetic field */

/*
 * SENSOR STRUCT
 */

typedef struct {
	/* I2C handle */
	I2C_HandleTypeDef *i2cHandle;

	/* Acceleration data (X, Y, Z) */
	float acc[3];
	int16_t acc_raw[3];

	/* Acceleration pitch/roll */
	float pitch;
	float roll;


	/* Magnetometer data (X, Y, Z) */
	float mag[3];
	int16_t mag_raw[3];

	/* Magnetometer alligement */
	float alignment;

	/* Temperature */
	float temp_C;
} LSM303AGR;

/*
* INITIALISATION
*/

uint8_t LSM303AGR_Initialise(LSM303AGR *dev, I2C_HandleTypeDef *i2cHandle); /* the return is for error reasons */

/*
 * DATA ACQUISITION
 */

HAL_StatusTypeDef LSM303AGR_ReadTemperature(LSM303AGR *dev); /* The return type only indicates if the I2C communication was successfull */
HAL_StatusTypeDef LSM303AGR_ReadAcceleration(LSM303AGR *dev);
HAL_StatusTypeDef LSM303AGR_ReadMagnetometer(LSM303AGR *dev);

/*
 * DEFAULT FUNCTIONS
 */

HAL_StatusTypeDef LSM303AGR_MAG_ReadRegister(LSM303AGR *dev, uint8_t reg, uint8_t *data);
HAL_StatusTypeDef LSM303AGR_MAG_ReadRegisters(LSM303AGR *dev, uint8_t reg, uint8_t *data, uint8_t length);
HAL_StatusTypeDef LSM303AGR_MAG_WriteRegister(LSM303AGR *dev, uint8_t reg, uint8_t *data);

HAL_StatusTypeDef LSM303AGR_ACC_ReadRegister(LSM303AGR *dev, uint8_t reg, uint8_t *data);
HAL_StatusTypeDef LSM303AGR_ACC_ReadRegisters(LSM303AGR *dev, uint8_t reg, uint8_t *data, uint8_t length);
HAL_StatusTypeDef LSM303AGR_ACC_WriteRegister(LSM303AGR *dev, uint8_t reg, uint8_t *data);

#endif







