/*
 * MS5607 i2c Driver
 *
 * Author: Fabian Glutz
 * Created: 14.05.22
 * V1
 *
 */

#ifndef MS5607_H
#define MS5607_H

#include "stm32l4xx_hal.h" /* Needed for I2C */
#include <stdio.h>

/*
 * DEFINES
 */

/*
 * Adresslenght is 7 bit (1 bit for read/write)
 * A0, A1, A2 = 0
 * Binary = 1110000 / HEX = 70
 */
#define MS5607_I2C_ADDR 	(0x76 << 1)
#define Reset				(0x1E)
#define ADC_READ			(0x00)
#define PROM_READ			(0xA0) // From 0xA0 to 0xAE


// OSR values

// OSR 4096
#define OSR_4096			(4096)
#define OSR_4096_CONV_D1	(0x48)
#define OSR_4096_CONV_D2	(0x58)
#define OSR_4096_CONV_DELAY	(10)


// OSR 2048
#define OSR_2048			(2048)
#define OSR_2048_CONV_D1	(0x46)
#define OSR_2048_CONV_D2	(0x56)
#define OSR_2048_CONV_DELAY	(5)

// OSR 1024
#define OSR_1024			(1024)
#define OSR_1024_CONV_D1	(0x44)
#define OSR_1024_CONV_D2	(0x54)
#define OSR_1024_CONV_DELAY	(3)

// OSR 512
#define OSR_512				(512)
#define OSR_512_CONV_D1		(0x42)
#define OSR_512_CONV_D2		(0x52)
#define OSR_512_CONV_DELAY	(2)

// OSR 256
#define OSR_256				(256)
#define OSR_256_CONV_D1		(0x40)
#define OSR_256_CONV_D2		(0x50)
#define OSR_256_CONV_DELAY	(1)


class MS5607 {
public:
	 MS5607();
	 MS5607(uint8_t address);
	bool setup(I2C_HandleTypeDef *i2cHandle);
	bool setOSR(uint16_t OSR_SET);
	float getTemperature(void);
	float getPressure(void);
	bool readRawData(void);
	float getAltitude(void);

private:

uint8_t MS5607_ADDR = MS5607_I2C_ADDR;
uint16_t OSR = OSR_4096; // default OSR
uint16_t CONV_D1 = OSR_4096_CONV_D1; //temp cov. for OSR
uint16_t CONV_D2 = OSR_4096_CONV_D2; //press cov. for OSR
uint8_t CONV_DELAY = OSR_4096_CONV_DELAY; // Conversion time 	from adc for OSR

uint16_t C1,C2,C3,C4,C5,C6; // see datasheet page 8
uint32_t Digital_Pressure,Digital_Temperature; // see datasheet page 8
int32_t dt, TEMP, P; // see datasheet page 8
int64_t OFF, SENS; // see datasheet page 8
I2C_HandleTypeDef *i2cHandle;



bool resetDevice(void);
bool readCalibration();
bool readUInt_16(char address, unsigned int &value);
bool readBytes(unsigned char *values, char length);
bool startConversion(char CMD);
bool startMeasurment(void);
bool getDigitalValue(unsigned long &value);

};

#endif
