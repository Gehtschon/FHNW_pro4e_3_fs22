/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32l4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define ADC_3V3_Pin GPIO_PIN_0
#define ADC_3V3_GPIO_Port GPIOC
#define ADC_BATTERIE_Pin GPIO_PIN_1
#define ADC_BATTERIE_GPIO_Port GPIOC
#define INT_GPS_Pin GPIO_PIN_2
#define INT_GPS_GPIO_Port GPIOA
#define RESET_GPS_Pin GPIO_PIN_3
#define RESET_GPS_GPIO_Port GPIOA
#define SD_NSS_Pin GPIO_PIN_4
#define SD_NSS_GPIO_Port GPIOA
#define SD_LC_Pin GPIO_PIN_4
#define SD_LC_GPIO_Port GPIOC
#define POWER_SW_Pin GPIO_PIN_5
#define POWER_SW_GPIO_Port GPIOC
#define INT1_AS_Pin GPIO_PIN_0
#define INT1_AS_GPIO_Port GPIOB
#define INT2_AS_Pin GPIO_PIN_1
#define INT2_AS_GPIO_Port GPIOB
#define LoRa_LC2_Pin GPIO_PIN_2
#define LoRa_LC2_GPIO_Port GPIOB
#define READY_LED_Pin GPIO_PIN_10
#define READY_LED_GPIO_Port GPIOB
#define STATUS_LED_Pin GPIO_PIN_11
#define STATUS_LED_GPIO_Port GPIOB
#define LORA_NSS_Pin GPIO_PIN_12
#define LORA_NSS_GPIO_Port GPIOB
#define LORA_SCK_Pin GPIO_PIN_13
#define LORA_SCK_GPIO_Port GPIOB
#define LORA_MISO_Pin GPIO_PIN_14
#define LORA_MISO_GPIO_Port GPIOB
#define LORA_MOSI_Pin GPIO_PIN_15
#define LORA_MOSI_GPIO_Port GPIOB
#define LORA_DIO0_Pin GPIO_PIN_6
#define LORA_DIO0_GPIO_Port GPIOC
#define LORA_DIO1_Pin GPIO_PIN_7
#define LORA_DIO1_GPIO_Port GPIOC
#define LORA_DIO2_Pin GPIO_PIN_8
#define LORA_DIO2_GPIO_Port GPIOC
#define LORA_DIO3_Pin GPIO_PIN_9
#define LORA_DIO3_GPIO_Port GPIOC
#define LORA_DIO4_Pin GPIO_PIN_8
#define LORA_DIO4_GPIO_Port GPIOA
#define LORA_DIO5_Pin GPIO_PIN_9
#define LORA_DIO5_GPIO_Port GPIOA
#define Reset_LoRa_Pin GPIO_PIN_10
#define Reset_LoRa_GPIO_Port GPIOA
#define LORA_LC3_Pin GPIO_PIN_11
#define LORA_LC3_GPIO_Port GPIOA
#define LORA_LC1_Pin GPIO_PIN_12
#define LORA_LC1_GPIO_Port GPIOA
#define SWDIO_Pin GPIO_PIN_13
#define SWDIO_GPIO_Port GPIOA
#define SWCLK_Pin GPIO_PIN_14
#define SWCLK_GPIO_Port GPIOA
#define RESET_n_MUX_Pin GPIO_PIN_15
#define RESET_n_MUX_GPIO_Port GPIOA
#define SS1_Pin GPIO_PIN_10
#define SS1_GPIO_Port GPIOC
#define INT_SS1_Pin GPIO_PIN_11
#define INT_SS1_GPIO_Port GPIOC
#define SS2_Pin GPIO_PIN_12
#define SS2_GPIO_Port GPIOC
#define INT_SS2_Pin GPIO_PIN_2
#define INT_SS2_GPIO_Port GPIOD
#define SWO_Pin GPIO_PIN_3
#define SWO_GPIO_Port GPIOB
#define SS3_Pin GPIO_PIN_4
#define SS3_GPIO_Port GPIOB
#define INT_SS3_Pin GPIO_PIN_5
#define INT_SS3_GPIO_Port GPIOB
#define SS4_Pin GPIO_PIN_6
#define SS4_GPIO_Port GPIOB
#define INT_SS4_Pin GPIO_PIN_7
#define INT_SS4_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */
#define SD_SPI_HANDLE hspi1
#define ANZAHL_SPECTRALSENSOR 4
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
