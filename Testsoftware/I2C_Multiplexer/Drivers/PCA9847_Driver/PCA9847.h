/*
 * PCA9847 MUX Driver
 *
 * Author: Fabian Glutz
 * Created: 09.05.22
 * V1
 *
 */

#ifndef PCA9847_I2C_DRIVER_H
#define PCA9847_I2C_DRIVER_H

#include "stm32l4xx_hal.h" /* Needed for I2C */
#include <stdio.h>

/*
 * Example Code
 *
 * PCA9847 sensor;
 * uint8_t channelNumber;
 * uint8_t errIni = 0;
 *
 * errIni = PCA9847_Initialise(&sensor, &hi2c1);
 * printf("Error-Initialsierung: %d \n",errIni);
 *
 * while (1){
 * 		PCA9847_SetChannel(&sensor, 1);
 * 		channelNumber = PCA9847_CheckChannel(&sensor);
 *
 * 		[Communicate here]
 *
 * 		PCA9847_SetResetState(&sensor);
 * }
 */

/*
 * REGISTERS/DEFINES
 */

/*
 * Adresslenght is 7 bit (1 bit for read/write)
 * A0, A1, A2 = 0
 * Binary = 1110000 / HEX = 70
 */
#define PCA9847_I2C_ADDR 	(0x71 << 1)
#define ENABLE_CHANNEL_0 		(0x08)
#define ENABLE_CHANNEL_1 		(0x09)
#define ENABLE_CHANNEL_2 		(0x0A)
#define ENABLE_CHANNEL_3 		(0x0B)
#define ENABLE_CHANNEL_4 		(0x0C)
#define ENABLE_CHANNEL_5 		(0x0D)
#define ENABLE_CHANNEL_6 		(0x0E)
#define ENABLE_CHANNEL_7 		(0x0F)
#define DISABLE_CHANNELS		(0x00)

/*
 * Struct
 */

typedef struct {
	/* I2C handle */
	I2C_HandleTypeDef *i2cHandle;
} PCA9847;

/*
 * FUNCTIONS
 */

uint8_t PCA9847_Initialise(PCA9847 *dev, I2C_HandleTypeDef *i2cHandle);
void PCA9847_SetChannel(PCA9847 *dev, uint8_t channelNumber); // (0 - 7)
void PCA9847_SetNextChannel(PCA9847 *dev);
void PCA9847_SetResetState(PCA9847 *dev);
uint8_t PCA9847_CheckChannel(PCA9847 *dev);

/*
 * DEFAULT FUNCTIONS
 */

HAL_StatusTypeDef PCA9847_Write(PCA9847 *dev, uint8_t *data);
HAL_StatusTypeDef PCA9847_Read(PCA9847 *dev, uint8_t *data);


#endif
