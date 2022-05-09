/*
 * PCA9546A MUX Driver
 *
 * Author: Gian-Luca Portmann
 * Created: 07.05.22
 * V1
 *
 */

#ifndef PCA9546A_I2C_DRIVER_H
#define PCA9546A_I2C_DRIVER_H

#include "stm32g4xx_hal.h" /* Needed for I2C */
#include <stdio.h>

/*
 * Example Code
 *
 * PCA9546A sensor;
 * uint8_t channelNumber;
 * uint8_t errIni = 0;
 *
 * errIni = PCA9546A_Initialise(&sensor, &hi2c1);
 * printf("Error-Initialsierung: %d \n",errIni);
 *
 * while (1){
 * 		PCA9546A_SetChannel(&sensor, 1);
 * 		channelNumber = PCA9546A_CheckChannel(&sensor);
 *
 * 		[Communicate here]
 *
 * 		PCA9546A_SetResetState(&sensor);
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
#define PCA9546A_I2C_ADDR 	(0x70 << 1)
#define ENABLE_CHANNEL_0 		(0x01)
#define ENABLE_CHANNEL_1 		(0x02)
#define ENABLE_CHANNEL_2 		(0x04)
#define ENABLE_CHANNEL_3 		(0x08)
#define DISABLE_CHANNELS		(0x00)

/*
 * Struct
 */

typedef struct {
	/* I2C handle */
	I2C_HandleTypeDef *i2cHandle;
} PCA9546A;

/*
 * FUNCTIONS
 */

uint8_t PCA9546A_Initialise(PCA9546A *dev, I2C_HandleTypeDef *i2cHandle);
void PCA9546A_SetChannel(PCA9546A *dev, uint8_t channelNumber); // (0 - 3)
void PCA9546A_SetResetState(PCA9546A *dev);
uint8_t PCA9546A_CheckChannel(PCA9546A *dev);

/*
 * DEFAULT FUNCTIONS
 */

HAL_StatusTypeDef PCA9546A_Write(PCA9546A *dev, uint8_t *data);
HAL_StatusTypeDef PCA9546A_Read(PCA9546A *dev, uint8_t *data);


#endif
