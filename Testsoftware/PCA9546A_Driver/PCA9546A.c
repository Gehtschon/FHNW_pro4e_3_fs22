#include "PCA9546A.h"

uint8_t PCA9546A_Initialise(PCA9546A *dev, I2C_HandleTypeDef *i2cHandle){
	/* Set struct parameters */
	dev->i2cHandle 	= i2cHandle;

	/* Store number of transaction errors */
	uint8_t regData;
	HAL_StatusTypeDef status;
	uint8_t errNum = 0;

	/* Set to reset default state (all channels disabled) */
	regData = DISABLE_CHANNELS;
	status = PCA9546A_Write(dev, &regData);
	errNum += (status != HAL_OK);

	return errNum;
}

void PCA9546A_SetChannel(PCA9546A *dev, uint8_t channelNumber){
	uint8_t regData;
	switch (channelNumber) {
	case 0:
		regData = ENABLE_CHANNEL_0;
		break;
	case 1:
		regData = ENABLE_CHANNEL_1;
		break;
	case 2:
		regData = ENABLE_CHANNEL_2;
		break;
	case 3:
		regData = ENABLE_CHANNEL_3;
		break;
	default:
		printf("Channelnumber should be 0-3 but is %d \r\n", channelNumber);
		return;
	}
	PCA9546A_Write(dev, &regData);
}

void PCA9546A_SetResetState(PCA9546A *dev){
	uint8_t regData;
	regData = DISABLE_CHANNELS;
	PCA9546A_Write(dev, &regData);
}

uint8_t PCA9546A_CheckChannel(PCA9546A *dev) {
	uint8_t regData;
	PCA9546A_Read(dev, &regData);
	switch (regData) {
	case ENABLE_CHANNEL_0:
		return 0;
	case ENABLE_CHANNEL_1:
		return 1;
	case ENABLE_CHANNEL_2:
		return 2;
	case ENABLE_CHANNEL_3:
		return 3;
	case DISABLE_CHANNELS:
		return 4;
	default:
		printf("Couldn't check Channel. Received Data: %d \r\n", regData);
		return 5;
	}
}


/*
 * DEFAULT FUNCTIONS
 */

HAL_StatusTypeDef PCA9546A_Write(PCA9546A *dev, uint8_t *data){
	return HAL_I2C_Master_Transmit(dev->i2cHandle, PCA9546A_I2C_ADDR, data, 1, 10);
}

HAL_StatusTypeDef PCA9546A_Read(PCA9546A *dev, uint8_t *data){
	return HAL_I2C_Master_Receive(dev->i2cHandle, PCA9546A_I2C_ADDR, data, 1, 10);
}

