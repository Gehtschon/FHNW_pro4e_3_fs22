#include "PCA9847.h"

uint8_t PCA9847_Initialise(PCA9847 *dev, I2C_HandleTypeDef *i2cHandle) {
	/* Set struct parameters */
	dev->i2cHandle = i2cHandle;

	/* Store number of transaction errors */
	uint8_t regData;
	HAL_StatusTypeDef status;
	uint8_t errNum = 0;

	/* Set to reset default state (all channels disabled) */
	regData = DISABLE_CHANNELS;
	status = PCA9847_Write(dev, &regData);
	errNum += (status != HAL_OK);

	return errNum;
}

void PCA9847_SetChannel(PCA9847 *dev, uint8_t channelNumber) {
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
	case 4:
		regData = ENABLE_CHANNEL_4;
		break;
	case 5:
		regData = ENABLE_CHANNEL_5;
		break;
	case 6:
		regData = ENABLE_CHANNEL_6;
		break;
	case 7:
		regData = ENABLE_CHANNEL_7;
		break;
	default:
		printf("Channelnumber should be 0-7 but is %d \r\n", channelNumber);
		return;
	}
	PCA9847_Write(dev, &regData);
}

void PCA9847_SetNextChannel(PCA9847 *dev) {
	uint8_t currentChannel;
	uint8_t nextChannel;
	currentChannel = PCA9847_CheckChannel(dev);
	nextChannel = currentChannel + 1;
	if (nextChannel >= 8) {
		nextChannel = 0;
	}
	PCA9847_SetChannel(dev, nextChannel);

}

void PCA9847_SetResetState(PCA9847 *dev) {
	uint8_t regData;
	regData = DISABLE_CHANNELS;
	PCA9847_Write(dev, &regData);
}

uint8_t PCA9847_CheckChannel(PCA9847 *dev) {
	uint8_t regData;
	PCA9847_Read(dev, &regData);
	switch (regData) {
	case ENABLE_CHANNEL_0:
		return 0;
	case ENABLE_CHANNEL_1:
		return 1;
	case ENABLE_CHANNEL_2:
		return 2;
	case ENABLE_CHANNEL_3:
		return 3;
	case ENABLE_CHANNEL_4:
		return 4;
	case ENABLE_CHANNEL_5:
		return 5;
	case ENABLE_CHANNEL_6:
		return 6;
	case ENABLE_CHANNEL_7:
		return 7;
	case DISABLE_CHANNELS:
		return 8;
	default:
		printf("Couldn't check Channel. Received Data: %d \r\n", regData);
		return 9;
	}
}

/*
 * DEFAULT FUNCTIONS
 */

HAL_StatusTypeDef PCA9847_Write(PCA9847 *dev, uint8_t *data) {
	return HAL_I2C_Master_Transmit(dev->i2cHandle, PCA9847_I2C_ADDR, data, 1,
			10);
}

HAL_StatusTypeDef PCA9847_Read(PCA9847 *dev, uint8_t *data) {
	return HAL_I2C_Master_Receive(dev->i2cHandle, PCA9847_I2C_ADDR, data, 1, 10);
}

