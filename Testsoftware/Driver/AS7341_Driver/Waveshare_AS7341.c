/*****************************************************************************
* | File      	:   AS7341.c
* | Author      :   Waveshare team
* | Function    :   AS7341 driver
* | Info        :
*----------------
* |	This version:   V1.0
* | Date        :   2021-1-13
* | Info        :   Basic version
*
******************************************************************************/
//#include "DEV_Config.h"
#include "../AS7341_Driver/Waveshare_AS7341.h"

#include <stm32g4xx_hal_i2c.h>
#include <stdio.h>
#include <sys/_stdint.h>

 eMode_t measureMode;
 uint8_t I2C_ADDR;
/******************************************************************************
function:	Read one byte of data to AS7341 via I2C
parameter:  
            Addr: Register address
Info:
******************************************************************************/
static UBYTE AS7341_Read_Byte(AS7341 *dev, UBYTE Addr)
{
    return I2C_Read_Byte(dev, Addr);
}

/******************************************************************************
function:	Read one word of data to AS7341 via I2C
parameter:
            Addr: Register address
Info:
******************************************************************************/
/*
static UWORD AS7341_Read_Word(UBYTE Addr)
{
    return I2C_Read_Word(Addr);
}
*/
/******************************************************************************
function:	Send one byte of data to AS7341 via I2C
parameter:
            Addr: Register address
           Value: Write to the value of the register
Info:
******************************************************************************/
static void AS7341_Write_Byte(AS7341 *dev, UBYTE Addr, UBYTE Value)
{
    I2C_Write_Byte(dev, Addr, Value);
}


/******************************************************************************
function:	AS7341 Initialization
parameter:
Info:
******************************************************************************/
UBYTE AS7341_Init(AS7341 *dev, I2C_HandleTypeDef *i2cHandle, eMode_t mode){
	dev->i2cHandle 	= i2cHandle;
	DEV_I2C_Init(AS7341_ADDRESS<<1);//8-bit address
	AS7341_Enable(dev, true);
	measureMode=mode;
	return 0;
}

/******************************************************************************
function： enable PON
info：	power on
******************************************************************************/
void AS7341_Enable(AS7341 *dev, int flag)
{
	UBYTE data;
	data=AS7341_Read_Byte(dev, AS7341_ENABLE);
    if(flag == true){
    data = data | (1<<0);
    } else {
    data = data & (~1);
    }
	AS7341_Write_Byte(dev, AS7341_ENABLE,data);
	printf("Initialization is complete !\r\n");
    AS7341_Write_Byte(dev, 0x00, 0x30);
}

/******************************************************************************
function： enable Spectral measure
info：	
******************************************************************************/
void AS7341_EnableSpectralMeasure(AS7341 *dev, int flag)//Enable spectral measurement
{
    UBYTE data;
    data=AS7341_Read_Byte(dev, AS7341_ENABLE);
    if(flag == true){
      data = data | (1<<1);
    } else {
      data = data & (~(1<<1));
    }
    AS7341_Write_Byte(dev, AS7341_ENABLE,data);
}

/******************************************************************************
function： enable SMUX
info：	
******************************************************************************/
void AS7341_EnableSMUX(AS7341 *dev, int flag)//Enable multiplexer
/*The other available channels can be accessed by amultiplexer (SMUX) connecting them to one of the internal ADCs.*/
{
  UBYTE data;
    data=AS7341_Read_Byte(dev, AS7341_ENABLE);
  if(flag == true){
    data = data | (1<<4);
  } else {
    data = data & (~(1<<4));
  }
    AS7341_Write_Byte(dev, AS7341_ENABLE,data);
}

/******************************************************************************
function:	enable flicker detection
info：	
******************************************************************************/
void AS7341_EnableFlickerDetection(AS7341 *dev, int flag)
{

  UBYTE data;
  data=AS7341_Read_Byte(dev, AS7341_ENABLE);
  if(flag == true){
    data = data | (1<<6);
  } else {
    data = data & (~(1<<6));
  }
  AS7341_Write_Byte(dev, AS7341_ENABLE,data);
}

/******************************************************************************
function:	choose model for spectral measurement
info：	
******************************************************************************/
void AS7341_Config(AS7341 *dev, eMode_t mode)
{
  UBYTE data;
  AS7341_SetBank(dev, 1);
  data=AS7341_Read_Byte(dev, AS7341_CONFIG);
  switch(mode){
    case eSpm : {
      data = (data & (~3)) | eSpm;
    };
	break;
    case eSyns : {
      data = (data & (~3)) | eSyns;
    };
	break;
    case eSynd : {
      data = (data & (~3)) | eSynd;
    };
	break;
    default : break;
  }
  AS7341_Write_Byte(dev, AS7341_CONFIG,data);
  AS7341_SetBank(dev, 0);
}

/******************************************************************************
function:	Configure SMUX for sensors F1-4, Clear and NIR
info：	
******************************************************************************/
void F1F4_Clear_NIR(AS7341 *dev)
{
  AS7341_Write_Byte(dev, 0x00, 0x30);
  AS7341_Write_Byte(dev, 0x01, 0x01);
  AS7341_Write_Byte(dev, 0x02, 0x00);
  AS7341_Write_Byte(dev, 0x03, 0x00);
  AS7341_Write_Byte(dev, 0x04, 0x00);
  AS7341_Write_Byte(dev, 0x05, 0x42);
  AS7341_Write_Byte(dev, 0x06, 0x00);
  AS7341_Write_Byte(dev, 0x07, 0x00);
  AS7341_Write_Byte(dev, 0x08, 0x50);
  AS7341_Write_Byte(dev, 0x09, 0x00);
  AS7341_Write_Byte(dev, 0x0A, 0x00);
  AS7341_Write_Byte(dev, 0x0B, 0x00);
  AS7341_Write_Byte(dev, 0x0C, 0x20);
  AS7341_Write_Byte(dev, 0x0D, 0x04);
  AS7341_Write_Byte(dev, 0x0E, 0x00);
  AS7341_Write_Byte(dev, 0x0F, 0x30);
  AS7341_Write_Byte(dev, 0x10, 0x01);
  AS7341_Write_Byte(dev, 0x11, 0x50);
  AS7341_Write_Byte(dev, 0x12, 0x00);
  AS7341_Write_Byte(dev, 0x13, 0x06);
}
/******************************************************************************
function:	Configure SMUX for sensors F5-8, Clear and NIR
info：	
******************************************************************************/
void F5F8_Clear_NIR(AS7341 *dev)
{
  AS7341_Write_Byte(dev, 0x00, 0x00);
  AS7341_Write_Byte(dev, 0x01, 0x00);
  AS7341_Write_Byte(dev, 0x02, 0x00);
  AS7341_Write_Byte(dev, 0x03, 0x40);
  AS7341_Write_Byte(dev, 0x04, 0x02);
  AS7341_Write_Byte(dev, 0x05, 0x00);
  AS7341_Write_Byte(dev, 0x06, 0x10);
  AS7341_Write_Byte(dev, 0x07, 0x03);
  AS7341_Write_Byte(dev, 0x08, 0x50);
  AS7341_Write_Byte(dev, 0x09, 0x10);
  AS7341_Write_Byte(dev, 0x0A, 0x03);
  AS7341_Write_Byte(dev, 0x0B, 0x00);
  AS7341_Write_Byte(dev, 0x0C, 0x00);
  AS7341_Write_Byte(dev, 0x0D, 0x00);
  AS7341_Write_Byte(dev, 0x0E, 0x24);
  AS7341_Write_Byte(dev, 0x0F, 0x00);
  AS7341_Write_Byte(dev, 0x10, 0x00);
  AS7341_Write_Byte(dev, 0x11, 0x50);
  AS7341_Write_Byte(dev, 0x12, 0x00);
  AS7341_Write_Byte(dev, 0x13, 0x06);
}
/******************************************************************************
function:	Configure SMUX for flicker detection
info：	
******************************************************************************/
void FDConfig(AS7341 *dev)
{

  AS7341_Write_Byte(dev, 0x00, 0x00);
  AS7341_Write_Byte(dev, 0x01, 0x00);
  AS7341_Write_Byte(dev, 0x02, 0x00);
  AS7341_Write_Byte(dev, 0x03, 0x00);
  AS7341_Write_Byte(dev, 0x04, 0x00);
  AS7341_Write_Byte(dev, 0x05, 0x00);
  AS7341_Write_Byte(dev, 0x06, 0x00);
  AS7341_Write_Byte(dev, 0x07, 0x00);
  AS7341_Write_Byte(dev, 0x08, 0x00);
  AS7341_Write_Byte(dev, 0x09, 0x00);
  AS7341_Write_Byte(dev, 0x0A, 0x00);
  AS7341_Write_Byte(dev, 0x0B, 0x00);
  AS7341_Write_Byte(dev, 0x0C, 0x00);
  AS7341_Write_Byte(dev, 0x0D, 0x00);
  AS7341_Write_Byte(dev, 0x0E, 0x00);
  AS7341_Write_Byte(dev, 0x0F, 0x00);
  AS7341_Write_Byte(dev, 0x10, 0x00);
  AS7341_Write_Byte(dev, 0x11, 0x00);
  AS7341_Write_Byte(dev, 0x12, 0x00);
  AS7341_Write_Byte(dev, 0x13, 0x60);
}

/******************************************************************************
function:	Start the measurement
info：		This function only handles SPM and SYNS modes.
******************************************************************************/
void AS7341_startMeasure(AS7341 *dev, eChChoose_t mode)
{
	  UBYTE data=0;
	  data = AS7341_Read_Byte(dev, AS7341_CFG_0);
	  data = data & (~(1<<4));

	  AS7341_Write_Byte(dev, AS7341_CFG_0,data);
	  
	  AS7341_EnableSpectralMeasure(dev, false);
	  AS7341_Write_Byte(dev, 0xAF,0x10);//SMUX Command config
	  
	  if(mode  == eF1F4ClearNIR)
      F1F4_Clear_NIR(dev);
	  else if(mode  == eF5F8ClearNIR)
	  F5F8_Clear_NIR(dev);
	  AS7341_EnableSMUX(dev, true);
	  if(measureMode == eSyns){
	  AS7341_SetGpioMode(dev, INPUT);
      AS7341_Config(dev, eSyns);
      }
	  else if(measureMode == eSpm){
      AS7341_Config(dev, eSpm);
	  }
	  AS7341_EnableSpectralMeasure(dev, true);
      if(measureMode == eSpm){
        while(!AS7341_MeasureComplete(dev)){
        DEV_Delay_ms(1);
        }
      }
}
/******************************************************************************
function:  read flicker data
info：		
******************************************************************************/
UBYTE AS7341_ReadFlickerData(AS7341 *dev)
{
	  UBYTE flicker;
	  UBYTE data=0;
	  data = AS7341_Read_Byte(dev, AS7341_CFG_0);
	  data = data & (~(1<<4));
	  AS7341_Write_Byte(dev, AS7341_CFG_0,data);
	  AS7341_EnableSpectralMeasure(dev, false);
	  AS7341_Write_Byte(dev, 0xAF,0x10);
	  FDConfig(dev);
	  AS7341_EnableSMUX(dev, true);
	  AS7341_EnableSpectralMeasure(dev, true);
	  //AS7341_Write_Byte(0xDA,0x00);	
	  UBYTE retry = 100;
	  if(retry == 0) printf(" data access error");
	  AS7341_EnableFlickerDetection(dev, true);
	  DEV_Delay_ms(600);
	  flicker = AS7341_Read_Byte(dev, AS7341_STATUS);
	  printf("flicker: %d \r\n",flicker);
	  AS7341_EnableFlickerDetection(dev, false);
	  switch(flicker){
		case 37:
		  flicker = 100;
		  break;
		case 40:
		  flicker = 0;
		  break;
		case 42:
		  flicker = 120;
		  break;		  
		case 44:
		  flicker = 1;
		  break;		  
		case 45:
		  flicker = 2;
		  break;		  
		default:
		  flicker = 2;
	  }
	  return flicker;
}

/******************************************************************************
function:  Determine whether the measurement is complete
info：		
******************************************************************************/

int AS7341_MeasureComplete(AS7341 *dev){
	UBYTE status;
	status = AS7341_Read_Byte(dev, AS7341_STATUS_2);
	if((status & (1<<6))){
		return true;
	}
	else{
		return false;
	}
}

/******************************************************************************
function:  Gets data for all channels
info：		
******************************************************************************/
UWORD AS7341_GetChannelData(AS7341 *dev, UBYTE channel)
{
  UWORD data[2];
  UWORD channelData = 0x0000;
  data[0] = AS7341_Read_Byte(dev, AS7341_CH0_DATA_L + channel*2);
  data[1] = AS7341_Read_Byte(dev, AS7341_CH0_DATA_H + channel*2);
  channelData = data[1];
  channelData = (channelData<<8) | data[0];
  DEV_Delay_ms(50);
  return channelData;
}

/******************************************************************************
function:  Use SMUX to read data from the low channel
info：		
******************************************************************************/

void AS7341_ReadSpectralDataOne(AS7341 *dev)
{
  dev->dataLowChannels.channel1 = AS7341_GetChannelData(dev, 0);
  dev->dataLowChannels.channel2 = AS7341_GetChannelData(dev, 1);
  dev->dataLowChannels.channel3 = AS7341_GetChannelData(dev, 2);
  dev->dataLowChannels.channel4 = AS7341_GetChannelData(dev, 3);
  dev->dataLowChannels.CLEAR = AS7341_GetChannelData(dev, 4);
  dev->dataLowChannels.NIR = AS7341_GetChannelData(dev,5);
}

/******************************************************************************
function:  Use SMUX to read data from the high channel
info：		
******************************************************************************/

void AS7341_ReadSpectralDataTwo(AS7341 *dev)
{
  dev->dataHighChannels.channel5 = AS7341_GetChannelData(dev, 0);
  dev->dataHighChannels.channel6 = AS7341_GetChannelData(dev, 1);
  dev->dataHighChannels.channel7 = AS7341_GetChannelData(dev, 2);
  dev->dataHighChannels.channel8 = AS7341_GetChannelData(dev, 3);
  dev->dataHighChannels.CLEAR = AS7341_GetChannelData(dev, 4);
  dev->dataHighChannels.NIR = AS7341_GetChannelData(dev, 5);
}

/******************************************************************************
function:	Set GPIO to input or output mode
info：
******************************************************************************/
void AS7341_SetGpioMode(AS7341 *dev, UBYTE mode)
{
  UBYTE data;

  data = AS7341_Read_Byte(dev, AS7341_GPIO_2);
  if(mode == INPUT){
     data = data | (1<<2);
  }
  
  if(mode == OUTPUT){
     data = data & (~(1<<2));
  }
  AS7341_Write_Byte(dev, AS7341_GPIO_2,data);
}

/******************************************************************************
function:	Configure the ATIME register
info：
******************************************************************************/
void AS7341_ATIME_config(AS7341 *dev, UBYTE value)
{
  AS7341_Write_Byte(dev, AS7341_ATIME,value);
}

/******************************************************************************
function:	Configure the ASTEP register
info：
******************************************************************************/
void AS7341_ASTEP_config(AS7341 *dev, UWORD value)
{
  UBYTE highValue,lowValue;
  lowValue = value & 0x00ff;
  highValue = value >> 8 ;
  AS7341_Write_Byte(dev, AS7341_ASTEP_L,lowValue);
  AS7341_Write_Byte(dev, AS7341_ASTEP_H,highValue);
}

/******************************************************************************
function:	Configure the AGAIN register
value:    0    1    2    3    4    5      6     7     8     9      10
gain:   X0.5 | X1 | X2 | X4 | X8 | X16 | X32 | X64 | X128 | X256 | X512
******************************************************************************/
void AS7341_AGAIN_config(AS7341 *dev, UBYTE value)
{
  if(value > 10) value = 10;
  AS7341_Write_Byte(dev, AS7341_CFG_1,value);
}

/******************************************************************************
function:	enable led
info：
******************************************************************************/
void AS7341_EnableLED(AS7341 *dev, int flag)
{
  UBYTE data=0;
  UBYTE data1=0;
  AS7341_SetBank(dev, 1);
  data = AS7341_Read_Byte(dev, AS7341_CONFIG);
  data1 = AS7341_Read_Byte(dev, AS7341_LED);
  if(flag== true){
    data = data | (1<<3);
    data1 = data1 | (1<<7);
  } else {
    data = data & (~(1<<3));
    data1 = data1 & (~(1<<7));
  }
  AS7341_Write_Byte(dev, AS7341_CONFIG,data);
  AS7341_Write_Byte(dev, AS7341_LED,data);
  AS7341_SetBank(dev, 0);
}
/******************************************************************************
function:	set REG_BANK
info：	0: Register access to register 0x80 and above
		1: Register access to register 0x60 to 0x74
******************************************************************************/
void AS7341_SetBank(AS7341 *dev, UBYTE addr)
{
  UBYTE data=0;
  data = AS7341_Read_Byte(dev, AS7341_CFG_0);
  if(addr == 1){
  
    data = data | (1<<4);
  }
  
  if(addr == 0){
  
    data = data & (~(1<<4));
  }
  AS7341_Write_Byte(dev, AS7341_CFG_0,data);
}
/******************************************************************************
function:	Control the brightness of the LED
info：
******************************************************************************/
void AS7341_ControlLed(AS7341 *dev, UBYTE current)
{
  UBYTE data=0;
  if(current < 1) current = 1;
    current--;
  if(current > 19) current = 19;
  AS7341_SetBank(dev, 1);
  data = 0;
  data = data | (1<<7);
  data = data | (current & 0x7f);
  AS7341_Write_Byte(dev, AS7341_LED, data);
  DEV_Delay_ms(100);
  AS7341_SetBank(dev, 0);
}

/******************************************************************************
function:	Determine whether the threshold setting is exceeded
info：Spectral interruptions occur when the set threshold is exceeded
******************************************************************************/
void AS7341_INTerrupt(AS7341 *dev)
{
  UBYTE data = AS7341_Read_Byte(dev, AS7341_STATUS_1);
  if(data & 0x80){
     printf("Spectral interrupt generation ！\r\n");
  } else {
    return ;
  }
}

/******************************************************************************
function:	clear interrupt
info：		This register is self-clearing, meaning that writing a "1" to any bit in the
	register clears that status bit. 
******************************************************************************/
void AS7341_ClearInterrupt(AS7341 *dev)
{
	
  AS7341_Write_Byte(dev, AS7341_STATUS_1,0xff);

}
/******************************************************************************
function:	enable spectral interrupt
info：
******************************************************************************/
void AS7341_EnableSpectralInterrupt(AS7341 *dev, int flag)
{
  UBYTE data;
  data = AS7341_Read_Byte(dev, AS7341_INTENAB);
  if(flag == true)
  {
    data = data | (1<<3);
    AS7341_Write_Byte(dev, AS7341_INTENAB,data);
  }
  else{
    data = data & (~(1<<3));
    AS7341_Write_Byte(dev, AS7341_INTENAB,data);
  }
  
}
/******************************************************************************
function:Spectral Interrupt Persistence
	value:      CHANNEL:
	0			Every spectral cycle generates aninterrupt
	1			1
	2			2
	3			3
	4			5
	5			10
	...			5*(value-3)
	14			55
	15			60
******************************************************************************/
void AS7341_SetInterruptPersistence(AS7341 *dev, UBYTE value)
{
	UBYTE data;
	data= value;
	AS7341_Write_Byte(dev, AS7341_PERS,data);
	data = AS7341_Read_Byte(dev, AS7341_PERS);
}
/******************************************************************************
function:	Set the interrupt threshold up and 
info：
******************************************************************************/
void AS7341_SetThreshold(AS7341 *dev, UWORD lowThre,UWORD highThre)
{
  if(lowThre >= highThre)return ;
  else
  
  AS7341_Write_Byte(dev, AS7341_SP_TH_L_LSB,lowThre);
  AS7341_Write_Byte(dev, AS7341_SP_TH_L_MSB,lowThre>>8);
  
  AS7341_Write_Byte(dev, AS7341_SP_TH_H_LSB,highThre);
  AS7341_Write_Byte(dev, AS7341_SP_TH_H_MSB,highThre>>8);
  
  DEV_Delay_ms(20);
}
/******************************************************************************
function:	Set the Spectral Threshold Channel
		VALUE 			CHANNEL
		0 				CH0
		1 				CH1
		2 				CH2
		3 				CH3
		4 				CH4
******************************************************************************/
void AS7341_SetSpectralThresholdChannel(AS7341 *dev, UBYTE value)
{
	AS7341_Write_Byte(dev, AS7341_CFG_12,value);
}


/******************************************************************************
function:	get low threshold
info：
******************************************************************************/

UWORD AS7341_GetLowThreshold(AS7341 *dev)
{
  uint16_t data; 
  data = AS7341_Read_Byte(dev, AS7341_SP_TH_L_LSB);
  data = (AS7341_Read_Byte(dev, AS7341_SP_TH_L_MSB)<<8) | data;
  return data ;
}

/******************************************************************************
function:	get high threshold
info：
******************************************************************************/
UWORD AS7341_GetHighThreshold(AS7341 *dev)
{
  uint16_t data;
  data = AS7341_Read_Byte(dev, AS7341_SP_TH_H_LSB);
  data = (AS7341_Read_Byte(dev, AS7341_SP_TH_H_MSB)<<8) | data;
  return data ;
}

/******************************************************************************
function:	syns interrupt set
info：
******************************************************************************/
void AS7341_SynsINT_sel(AS7341 *dev)
{
	AS7341_Write_Byte(dev, AS7341_CONFIG,0x05);
}


/******************************************************************************
function:	disable power,spectral reading, flicker detection  
info：
******************************************************************************/
void AS7341_disableALL(AS7341 *dev)
{
	AS7341_Write_Byte(dev, AS7341_ENABLE ,0x02);
}

/******************************************************************************
function:	I2C Function initialization and transfer
parameter:
Info:
******************************************************************************/
void DEV_I2C_Init(uint8_t Add)
{
	I2C_ADDR =  Add;
}

void I2C_Write_Byte(AS7341 *dev, uint8_t Cmd, uint8_t value)
{
	UBYTE Buf[1] = {0};
	Buf[0] = value;
	HAL_I2C_Mem_Write(dev->i2cHandle, I2C_ADDR, Cmd, I2C_MEMADD_SIZE_8BIT, Buf, 1, 0x20);
}

int I2C_Read_Byte(AS7341 *dev, uint8_t Cmd)
{
	UBYTE Buf[1]={0};
	HAL_I2C_Mem_Read(dev->i2cHandle, I2C_ADDR+1, Cmd, I2C_MEMADD_SIZE_8BIT, Buf, 1, 0x20);
	return Buf[0];
}

int I2C_Read_Word(AS7341 *dev, uint8_t Cmd)
{
	UBYTE Buf[2]={0, 0};
	HAL_I2C_Mem_Read(dev->i2cHandle, I2C_ADDR+1, Cmd, I2C_MEMADD_SIZE_8BIT, Buf, 2, 0x20);
	return ((Buf[1] << 8) | (Buf[0] & 0xff));
}



/**
 * delay x ms
**/
void DEV_Delay_ms(UDOUBLE xms)
{
	HAL_Delay(xms);
}
