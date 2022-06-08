#include "LSM303AGR.h"

uint8_t LSM303AGR_Initialise(LSM303AGR *dev, I2C_HandleTypeDef *i2cHandle){
	/* Set struct parameters */
	dev->i2cHandle 	= i2cHandle;
	dev->mag[0] 	= 0.0f;
	dev->mag[1] 	= 0.0f;
	dev->mag[2] 	= 0.0f;
	dev->acc[0] 	= 0.0f;
	dev->acc[1] 	= 0.0f;
	dev->acc[2] 	= 0.0f;
	dev->temp_C		= 0.0f;

	/* Store number of transaction errors (gets returned at the end of the function) */
	uint8_t errNum = 0;
	HAL_StatusTypeDef status;

	/* Check who am i of both devices */
	uint8_t regData;

	status = LSM303AGR_MAG_ReadRegister(dev, LSM303AGR_WHO_AM_I_M ,&regData);
	errNum += (status != HAL_OK);
	if (regData != LSM303AGR_MAG_WHO_AM_I){
		printf("Error MAG_WHO_AM_I: regdata is %x but should be %x \n",regData,LSM303AGR_MAG_WHO_AM_I);
		//return 255;
	}

	status = LSM303AGR_ACC_ReadRegister(dev, LSM303AGR_WHO_AM_I_A ,&regData);
	errNum += (status != HAL_OK);
	if (regData != LSM303AGR_ACC_WHO_AM_I){
		printf("Error ACC_WHO_AM_I: regdata is %x but should be %x \n",regData,LSM303AGR_ACC_WHO_AM_I);
		//return 255;
	}

	/* MAG: Output data rate and measurement configuration */

	/*
	 * temperature compensated: yes
	 * hight resolution: yes
	 * data rate: 10Hz
	 * continuous mode: yes
	 */

	regData = 0x80;
	status = LSM303AGR_MAG_WriteRegister(dev, LSM303AGR_CFG_REG_A_M, &regData);
	errNum += (status != HAL_OK);



	/*
	 * offset cancellation single measurement disabled
	 * no interrupt recognition checks
	 * offset cancellation enabled
	 * lowpass enabled
	 */
	regData = 0x03;
	status = LSM303AGR_MAG_WriteRegister(dev, LSM303AGR_CFG_REG_B_M, &regData);
	errNum += (status != HAL_OK);

	/*
	 * Block Data Update (BDU) enable
	 * avoiding of reading incorrect data when data is read asynchronously
	 * output registers not updated until MSB and LSB have been read
	 */
	regData = 0x10; // old 0x00
	status = LSM303AGR_MAG_WriteRegister(dev, LSM303AGR_CFG_REG_C_M, &regData);
	errNum += (status != HAL_OK);

	/* ACC: Output data rate and measurement configuration */

	/*
	 * 10Hz sampling
	 * normal mode (Power)
	 * X, Y and Z activated
	 */
	regData = 0x27;
	status = LSM303AGR_ACC_WriteRegister(dev, LSM303AGR_CTRL_REG1_A, &regData);
	errNum += (status != HAL_OK);

	/*
	 * Block Data Update (BDU) enable
	 * data LSb at lower address
	 * +- 4g
	 * High resolution
	 * spi/selftest disabled
	 */
	// regData = 0x38; // Continuous Update
	// regData = 0xB8; // +- 16g
	// regData = 0x88; // +- 2g
	regData = 0x98;
	status = LSM303AGR_ACC_WriteRegister(dev, LSM303AGR_CTRL_REG4_A, &regData);
	errNum += (status != HAL_OK);


	/*
	 * Temperature Sensor enabled
	 */
	regData = 0xC0;
	status = LSM303AGR_ACC_WriteRegister(dev, LSM303AGR_TEMP_CFG_REG_A, &regData);

	/* Return number of errors (0 if successful initialisation) */
	return errNum;
}

HAL_StatusTypeDef LSM303AGR_ReadTemperature(LSM303AGR *dev){

	uint8_t errNum = 0;

	/* Read raw values from Resgister (x, y, z -> 16 bits each) */
	//uint8_t regData[2];
	//HAL_StatusTypeDef status = LSM303AGR_ACC_ReadRegisters(dev, LSM303AGR_OUT_TEMP_L_A, regData, 2);

	uint8_t regDataTL, regDataTH;
	// errNum += (LSM303AGR_ACC_ReadRegister(dev, LSM303AGR_OUT_TEMP_L_A, &regDataTH) != HAL_OK); old version
	errNum += (LSM303AGR_ACC_ReadRegister(dev, LSM303AGR_OUT_TEMP_L_A, &regDataTL) != HAL_OK);
	errNum += (LSM303AGR_ACC_ReadRegister(dev, LSM303AGR_OUT_TEMP_H_A, &regDataTH) != HAL_OK);

	//printf("TH: %d, Tl: %d \n",regDataTH,regDataTL);

	/* Converting to signed */
	int8_t tmpRawSigned = (int8_t)regDataTH;

	/* Convertig to grad celsius */
	dev->temp_C = (tmpRawSigned + 25);

	return errNum;
}

HAL_StatusTypeDef LSM303AGR_ReadAcceleration(LSM303AGR *dev){

	/* Store number of transaction errors (gets returned at the end of the function) */
	uint8_t errNum = 0;

	/* Read raw values from Resgister (x, y, z -> 16 bits each) */
	uint8_t regDataXL, regDataXH, regDataYL, regDataYH, regDataZL, regDataZH;
	errNum += (LSM303AGR_ACC_ReadRegister(dev, LSM303AGR_OUT_X_L_A, &regDataXL) != HAL_OK);
	errNum += (LSM303AGR_ACC_ReadRegister(dev, LSM303AGR_OUT_X_H_A, &regDataXH) != HAL_OK);
	errNum += (LSM303AGR_ACC_ReadRegister(dev, LSM303AGR_OUT_Y_L_A, &regDataYL) != HAL_OK);
	errNum += (LSM303AGR_ACC_ReadRegister(dev, LSM303AGR_OUT_Y_H_A, &regDataYH) != HAL_OK);
	errNum += (LSM303AGR_ACC_ReadRegister(dev, LSM303AGR_OUT_Z_L_A, &regDataZL) != HAL_OK);
	errNum += (LSM303AGR_ACC_ReadRegister(dev, LSM303AGR_OUT_Z_H_A, &regDataZH) != HAL_OK);


	/* Combining 2x8bit unsigned to 12bit signed */
	int16_t accRawSigned[3];
	uint8_t bitmask = 0xF0;
	accRawSigned[0] = (((int16_t)((((uint16_t)regDataXH) << 8 ) | ((uint16_t)(regDataXL & bitmask)))) >> 4);
	accRawSigned[1] = (((int16_t)((((uint16_t)regDataYH) << 8 ) | ((uint16_t)(regDataYL & bitmask)))) >> 4);
	accRawSigned[2] = (((int16_t)((((uint16_t)regDataZH) << 8 ) | ((uint16_t)(regDataZL & bitmask)))) >> 4);

	/* Converting to g */
	dev->acc_raw[0] = accRawSigned[0];
	dev->acc_raw[1] = accRawSigned[1];
	dev->acc_raw[2] = accRawSigned[2];

	// measuring range set to +-4g
	 dev->acc[0] = accRawSigned[0] * 0.00195;
	 dev->acc[1] = accRawSigned[1] * 0.00195;
	 dev->acc[2] = accRawSigned[2] * 0.00195;

	/*
	// measuring range set to +-2g
	 dev->acc[0] = accRawSigned[0] * 0.00098;
	 dev->acc[1] = accRawSigned[1] * 0.00098;
	 dev->acc[2] = accRawSigned[2] * 0.00098;
	*/
	 /*
	// measuring range set to +-16g
	dev->acc[0] = accRawSigned[0] * 0.01172;
	dev->acc[1] = accRawSigned[1] * 0.01172;
	dev->acc[2] = accRawSigned[2] * 0.01172;
	 */

	/* Calculating Pitch/Roll */
	dev->pitch = 180 * atan (dev->acc[0]/sqrt(dev->acc[1]*dev->acc[1]  + dev->acc[2]*dev->acc[2]))/M_PI;
	dev->roll = 180 * atan (dev->acc[1]/sqrt(dev->acc[0]*dev->acc[0] + dev->acc[2]*dev->acc[2]))/M_PI;
	return errNum;

}

HAL_StatusTypeDef LSM303AGR_ReadMagnetometer(LSM303AGR *dev){

	/* Read raw values from Register (x, y, z -> 16 bits each) */
		uint8_t regData[6];
		HAL_StatusTypeDef status = LSM303AGR_MAG_ReadRegisters(dev, LSM303AGR_OUTX_L_REG_M, regData, 6);

		/* Combining 2x8bit to 16bit */
		int32_t magRawSigned[3];
		magRawSigned[0] = (int16_t)(((uint16_t)regData[0]) | ((uint16_t)regData[1] << 8)); /* X-Value */
		magRawSigned[1] = (int16_t)(((uint16_t)regData[2]) | ((uint16_t)regData[3] << 8)); /* Y-Value */
		magRawSigned[2] = (int16_t)(((uint16_t)regData[4]) | ((uint16_t)regData[5] << 8)); /* Z-Value */

		/* Converting to Gauss */
		dev->mag_raw[0] = magRawSigned[0];
		dev->mag_raw[1] = magRawSigned[1];
		dev->mag_raw[2] = magRawSigned[2];

		dev->mag[0] = magRawSigned[0] * 0.0015;
		dev->mag[1] = magRawSigned[1] * 0.0015;
		dev->mag[2] = magRawSigned[2] * 0.0015;

		/* Calculate alignment*/
		float alignment;
		alignment = (atan2(dev->mag[1], dev->mag[0]) * 180) / M_PI;
		if (alignment < 0) {
			alignment = 360 + alignment;
		}
		dev->alignment = alignment;

		return status;
	}



/*
 * DEFAULT FUNCTIONS
 */

HAL_StatusTypeDef LSM303AGR_MAG_ReadRegister(LSM303AGR *dev, uint8_t reg, uint8_t *data){
	return HAL_I2C_Mem_Read(dev->i2cHandle, LSM303AGR_MAG_I2C_ADDR, reg, I2C_MEMADD_SIZE_8BIT, data, 1, 10);
}

HAL_StatusTypeDef LSM303AGR_MAG_ReadRegisters(LSM303AGR *dev, uint8_t reg, uint8_t *data, uint8_t length){
	return HAL_I2C_Mem_Read(dev->i2cHandle, LSM303AGR_MAG_I2C_ADDR, reg, I2C_MEMADD_SIZE_8BIT, data, length, 10);
}

HAL_StatusTypeDef LSM303AGR_MAG_WriteRegister(LSM303AGR *dev, uint8_t reg, uint8_t *data){
	return HAL_I2C_Mem_Write(dev->i2cHandle, LSM303AGR_MAG_I2C_ADDR, reg, I2C_MEMADD_SIZE_8BIT, data, 1, 10);
}

HAL_StatusTypeDef LSM303AGR_ACC_ReadRegister(LSM303AGR *dev, uint8_t reg, uint8_t *data){
	return HAL_I2C_Mem_Read(dev->i2cHandle, LSM303AGR_ACC_I2C_ADDR, reg, I2C_MEMADD_SIZE_8BIT, data, 1, 10);
}

HAL_StatusTypeDef LSM303AGR_ACC_ReadRegisters(LSM303AGR *dev, uint8_t reg, uint8_t *data, uint8_t length){
	return HAL_I2C_Mem_Read(dev->i2cHandle, LSM303AGR_ACC_I2C_ADDR, reg, I2C_MEMADD_SIZE_8BIT, data, length, 10);
}

HAL_StatusTypeDef LSM303AGR_ACC_WriteRegister(LSM303AGR *dev, uint8_t reg, uint8_t *data){
	return HAL_I2C_Mem_Write(dev->i2cHandle, LSM303AGR_ACC_I2C_ADDR, reg, I2C_MEMADD_SIZE_8BIT, data, 1, 10);
}
