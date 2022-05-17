/*
 * MS5607 i2c Driver
 *
 * Author: Fabian Glutz
 * Created: 14.05.22
 * V1
 *
 */

#include "MS5607.h"

//Default constructor
MS5607::MS5607() {

}

MS5607::MS5607(uint8_t address) {
	this->MS5607_ADDR = address;
}

bool MS5607::setup(I2C_HandleTypeDef *i2cHandle){
	this->i2cHandle = i2cHandle;
	return(readCalibration());
}

bool MS5607::setOSR(uint16_t OSR_SET){
	// check if OSR is one of the defined values
	if (OSR_SET == OSR_4096 || OSR_SET == OSR_1024 || OSR_SET == OSR_512 || OSR_SET == OSR_256) {

		this->OSR = OSR_SET;
		switch (OSR) {
			case OSR_4096:
				CONV_D1 = OSR_4096_CONV_D1;
				CONV_D2 = OSR_4096_CONV_D2;
				CONV_DELAY = OSR_4096_CONV_DELAY;
				break;

			case OSR_2048:
				CONV_D1 = OSR_2048_CONV_D1;
				CONV_D2 = OSR_2048_CONV_D2;
				CONV_DELAY = OSR_2048_CONV_DELAY;
				break;

			case OSR_1024:
				CONV_D1 = OSR_1024_CONV_D1;
				CONV_D2 = OSR_1024_CONV_D2;
				CONV_DELAY = OSR_1024_CONV_DELAY;
				break;

			case OSR_512:
				CONV_D1 = OSR_512_CONV_D1;
				CONV_D2 = OSR_512_CONV_D2;
				CONV_DELAY = OSR_512_CONV_DELAY;
				break;

			case OSR_256:
				CONV_D1 = OSR_256_CONV_D1;
				CONV_D2 = OSR_256_CONV_D2;
				CONV_DELAY = OSR_256_CONV_DELAY;
				break;
				// default is OSR 256
			default:
				CONV_D1 = OSR_256_CONV_D1;
				CONV_D2 = OSR_256_CONV_D2;
				CONV_DELAY = OSR_256_CONV_DELAY;
				break;
		}
		return true;

	}else {
		return false;
	}
}

