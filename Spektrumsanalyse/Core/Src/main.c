/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
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
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "fatfs.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "flow.h"
#include "../../Core/Inc/stm32-hal-rfm95/rfm95.h"
#include "../../Drivers/PCA9847_Driver/PCA9847.h"
#include "../../Drivers/MS5607_Driver_C/MS5607.h"
#include "../../Drivers/AS7341_Driver/Waveshare_AS7341.h"
#include "../../Drivers/SHT41/SHT41.h"
#include <stdio.h>
#include <string.h>
#include <stdarg.h> //for va_list var arg functions
#include <stdlib.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
 I2C_HandleTypeDef hi2c1;

SPI_HandleTypeDef hspi1;
SPI_HandleTypeDef hspi2;

TIM_HandleTypeDef htim16;

UART_HandleTypeDef huart4;

/* USER CODE BEGIN PV */
PCA9847 multiplexer;
uint8_t errIni = 0;
MS5607 barometer;

SHT41 temp_innen = { .i2cHandle = &hi2c1, .Multiplexer_Channel = 5 };
SHT41 temp_aussen = { .i2cHandle = &hi2c1, .Multiplexer_Channel = 4 };

rfm95_handle_t rfm95_handle = { .spi_handle = &hspi2, .nss_port =
LORA_NSS_GPIO_Port, .nss_pin = LORA_NSS_Pin, .nrst_port =
Reset_LoRa_GPIO_Port, .nrst_pin =
Reset_LoRa_Pin, .irq_port = LORA_DIO0_GPIO_Port, .irq_pin =
LORA_DIO0_Pin, .dio5_port = LORA_DIO5_GPIO_Port, .dio5_pin =
LORA_DIO5_Pin, .device_address = { 0x26, 0x0B, 0x9B, 0xB7 },
		.application_session_key = { 0x36, 0x99, 0xBA, 0xEE, 0x6F, 0x2C, 0xFC,
				0xB1, 0x20, 0xB6, 0xF8, 0xB0, 0x99, 0xFC, 0xED, 0xE3 },
		.network_session_key = { 0x0C, 0x78, 0xCE, 0x6F, 0xC4, 0x15, 0x29, 0x37,
				0x2A, 0xE6, 0x52, 0x8B, 0x7E, 0x6F, 0xC6, 0x45 },
		.reload_frame_counter = NULL, .save_frame_counter = NULL, .longitude =
				82202500, .longitude_or = { 'E' }, .latitude = 474785200,
		.latitude_or[0] = { 'N' }, .altitude = 2461, .indent = 0 };

bool read_and_safe = false;
bool errorDetected = false;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI2_Init(void);
static void MX_TIM16_Init(void);
static void MX_I2C1_Init(void);
static void MX_SPI1_Init(void);
static void MX_UART4_Init(void);
/* USER CODE BEGIN PFP */
void myprintf(const char *fmt, ...);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void myprintf(const char *fmt, ...) {
	static char buffer[256];
	va_list args;
	va_start(args, fmt);
	vsnprintf(buffer, sizeof(buffer), fmt, args);
	va_end(args);

	int len = strlen(buffer);
	//HAL_UART_Transmit(&huart2, (uint8_t*)buffer, len, -1);

}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_SPI2_Init();
  MX_TIM16_Init();
  MX_I2C1_Init();
  MX_SPI1_Init();
  MX_FATFS_Init();
  MX_UART4_Init();
  /* USER CODE BEGIN 2 */
	FlowInit(&rfm95_handle);

	errIni = PCA9847_Initialise(&multiplexer, &hi2c1);
	Setup_Spectralsensor();

	//NVIC_SetPriority(USART1_IRQn, 0xC);
//	 priority = NVIC_EncodePriority(priorityGroup, 1, 6);
//	NVIC_SetPriority(TIM6_IRQn, 0xC);

//	PCA9847_SetChannel(&multiplexer, 3);
//	AS7341_Init(eSpm);
//	AS7341_ATIME_config(100);
//	AS7341_ASTEP_config(999);
//	AS7341_AGAIN_config(6);

//	PCA9847_SetChannel(&multiplexer, 6);
//	if(MS5607_Init(&barometer, &hi2c1)!=true){
//
//	}

// set RTC interrupt priority to 15 (lowest)
//	NVIC_SetPriority(TIM1_UP_TIM16_IRQn, 15);
//	// set interrupt priority of SysTick to 0 (highest)
//	NVIC_SetPriority(SysTick_IRQn, 0);

//	PCA9847_SetChannel(&multiplexer, 5);
//	SHT41_read(&temp_innen);

	HAL_TIM_Base_Start_IT(&htim16);

//	Write_SD_Card(&rfm95_handle);
//	Write_SD_Card(&rfm95_handle);

	char bfr[1024];
	char msgSep = '$';
	char msgSep1 = 'G';
	char msgSep2 = 'N';
	char msgSep3 = 'G';
	char msgSep4 = 'G';
	char msgSep5 = 'A';

	char part_string[13];
	uint16_t who[] = { 9, 12, 1, 13, 1, 1, 2, 4, 5 };

	bool foundString;
	uint16_t string_Start;

	uint16_t datapoint = 0;

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	while (1) {

		//HAL_GPIO_TogglePin(SS1_GPIO_Port, SS1_Pin);
		//HAL_GPIO_TogglePin(STATUS_LED_GPIO_Port, STATUS_LED_Pin);
		// HAL_GPIO_TogglePin(READY_LED_GPIO_Port, READY_LED_Pin);
		// HAL_Delay(5000);

		foundString = false;
		string_Start = 0;

		for (int o = 0; o < 1024; ++o) {
			bfr[o] = 0;
		}
		HAL_StatusTypeDef status;
		if (UART4->ISR & USART_ISR_ORE)
			UART4->ICR = USART_ICR_ORECF;

		status = HAL_UART_Receive(&huart4, (uint8_t*) bfr, sizeof(bfr), 800);
		for (uint16_t i = 0; i < 1024; i++) {
			if (bfr[i] == msgSep && bfr[i + 1] == msgSep1
					&& bfr[i + 2] == msgSep2 && bfr[i + 3] == msgSep3
					&& bfr[i + 4] == msgSep4 && bfr[i + 5] == msgSep5) {
//	             if (EvalFrame(&bfr[i], sample)){
//	                 return;
//	             }
				printf(bfr[i]);
				foundString = true;
				string_Start = i + 7;

			}
		}

		if (foundString == true) {
			datapoint = 0;
			while (datapoint < 9) {

				for (int var = 0; var <= who[datapoint]; ++var) {
					if (bfr[string_Start + var] != ',') {
						part_string[var] = bfr[string_Start + var];
					} else {
						if (var == 0) {
							part_string[var] = ',';
						}
						string_Start = string_Start + var + 1;
						for (uint16_t p = var + 1; p < 13; ++p) {
							part_string[p] = '?';
						}
						var = 12 + 1;
					}

				}
				if (datapoint == 1) {
					unsigned long answer;
					char *remaining;
					for (uint16_t ll = 5; ll < 10; ++ll) {
						part_string[ll - 1] = part_string[ll];
					}
					for (int z = 0; z < 13; ++z) {
						if (part_string[z] == '?') {
							part_string[z - 1] = '?';

						}
					}
					answer = strtol(part_string, &remaining, 10);
					rfm95_handle.latitude = answer;

				} else if (datapoint == 2) {
					rfm95_handle.latitude_or[0] = part_string[0];

				} else if (datapoint == 3) {
					unsigned long answer;
					char *remaining;
					for (uint16_t ll = 6; ll < 11; ++ll) {
						part_string[ll - 1] = part_string[ll];
					}
					for (int z = 0; z < 13; ++z) {
						if (part_string[z] == '?') {
							part_string[z - 1] = '?';

						}
					}
					answer = strtol(part_string, &remaining, 10);
					rfm95_handle.longitude = answer;
				} else if (datapoint == 4) {
					rfm95_handle.longitude_or[0] = part_string[0];

				} else if (datapoint == 8) {
					unsigned int answer;
					answer = atoi(part_string);
					rfm95_handle.altitude = answer;
				}
				datapoint++;
			}

		}

		if (read_and_safe == true) {
			read_and_safe = false;
			Read_Spectralsensor(&rfm95_handle);
			SHT41_read(&temp_innen);
			SHT41_read(&temp_aussen);
			//read_temp_aussen(&rfm95_handle);
			read_temp_innen(&rfm95_handle);
			Write_SD_Card_Spectral(&rfm95_handle);
		}

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	}
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_MSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x00000E14;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 7;
  hspi1.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief SPI2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI2_Init(void)
{

  /* USER CODE BEGIN SPI2_Init 0 */

  /* USER CODE END SPI2_Init 0 */

  /* USER CODE BEGIN SPI2_Init 1 */

  /* USER CODE END SPI2_Init 1 */
  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_256;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 7;
  hspi2.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi2.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */

}

/**
  * @brief TIM16 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM16_Init(void)
{

  /* USER CODE BEGIN TIM16_Init 0 */

  /* USER CODE END TIM16_Init 0 */

  /* USER CODE BEGIN TIM16_Init 1 */

  /* USER CODE END TIM16_Init 1 */
  htim16.Instance = TIM16;
  htim16.Init.Prescaler = 6000;
  htim16.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim16.Init.Period = 4000;
  htim16.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim16.Init.RepetitionCounter = 0;
  htim16.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim16) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM16_Init 2 */

  /* USER CODE END TIM16_Init 2 */

}

/**
  * @brief UART4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_UART4_Init(void)
{

  /* USER CODE BEGIN UART4_Init 0 */

  /* USER CODE END UART4_Init 0 */

  /* USER CODE BEGIN UART4_Init 1 */

  /* USER CODE END UART4_Init 1 */
  huart4.Instance = UART4;
  huart4.Init.BaudRate = 9600;
  huart4.Init.WordLength = UART_WORDLENGTH_8B;
  huart4.Init.StopBits = UART_STOPBITS_1;
  huart4.Init.Parity = UART_PARITY_NONE;
  huart4.Init.Mode = UART_MODE_TX_RX;
  huart4.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart4.Init.OverSampling = UART_OVERSAMPLING_16;
  huart4.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart4.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART4_Init 2 */

  /* USER CODE END UART4_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_3|SD_LC_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, INT_GPS_Pin|RESET_GPS_Pin|LORA_LC3_Pin|LORA_LC1_Pin
                          |RESET_n_MUX_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(SD_NSS_GPIO_Port, SD_NSS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, POWER_SW_Pin|SS1_Pin|SS2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LoRa_LC2_GPIO_Port, LoRa_LC2_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, READY_LED_Pin|STATUS_LED_Pin|LORA_NSS_Pin|SS4_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : ADC_3V3_Pin ADC_BATTERIE_Pin */
  GPIO_InitStruct.Pin = ADC_3V3_Pin|ADC_BATTERIE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG_ADC_CONTROL;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PC3 SD_LC_Pin POWER_SW_Pin SS1_Pin
                           SS2_Pin */
  GPIO_InitStruct.Pin = GPIO_PIN_3|SD_LC_Pin|POWER_SW_Pin|SS1_Pin
                          |SS2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : INT_GPS_Pin RESET_GPS_Pin SD_NSS_Pin RESET_n_MUX_Pin */
  GPIO_InitStruct.Pin = INT_GPS_Pin|RESET_GPS_Pin|SD_NSS_Pin|RESET_n_MUX_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : INT1_AS_Pin INT2_AS_Pin INT_SS3_Pin INT_SS4_Pin */
  GPIO_InitStruct.Pin = INT1_AS_Pin|INT2_AS_Pin|INT_SS3_Pin|INT_SS4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : LoRa_LC2_Pin */
  GPIO_InitStruct.Pin = LoRa_LC2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LoRa_LC2_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : READY_LED_Pin STATUS_LED_Pin LORA_NSS_Pin SS4_Pin */
  GPIO_InitStruct.Pin = READY_LED_Pin|STATUS_LED_Pin|LORA_NSS_Pin|SS4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : LORA_DIO0_Pin LORA_DIO1_Pin LORA_DIO2_Pin LORA_DIO3_Pin
                           INT_SS1_Pin */
  GPIO_InitStruct.Pin = LORA_DIO0_Pin|LORA_DIO1_Pin|LORA_DIO2_Pin|LORA_DIO3_Pin
                          |INT_SS1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : LORA_DIO4_Pin LORA_DIO5_Pin Reset_LoRa_Pin */
  GPIO_InitStruct.Pin = LORA_DIO4_Pin|LORA_DIO5_Pin|Reset_LoRa_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : LORA_LC3_Pin LORA_LC1_Pin */
  GPIO_InitStruct.Pin = LORA_LC3_Pin|LORA_LC1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : INT_SS2_Pin */
  GPIO_InitStruct.Pin = INT_SS2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(INT_SS2_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : SS3_Pin */
  GPIO_InitStruct.Pin = SS3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(SS3_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
	static int sendintervall = 6;
	if (htim == &htim16) {
		read_and_safe = true;

		if (sendintervall == 6) {
			HAL_GPIO_TogglePin(STATUS_LED_GPIO_Port, STATUS_LED_Pin);
			Flow(&rfm95_handle);
			sendintervall = 0;
		} else {
			sendintervall++;
		}

//		PCA9847_SetChannel(&multiplexer, 3);
//		sModeOneData_t data1;
//		sModeTwoData_t data2;
//		AS7341_startMeasure(eF1F4ClearNIR);
//		data1 = AS7341_ReadSpectralDataOne();
//		AS7341_startMeasure(eF5F8ClearNIR);
//		data2 = AS7341_ReadSpectralDataTwo();
//		Read_Spectralsensor(&rfm95_handle);
//		SHT41_read(&temp_innen);
//		SHT41_read(&temp_aussen);
		//	read_temp_aussen(&rfm95_handle, 4);
		//	read_temp_innen(&rfm95_handle, 5);
//		Write_SD_Card_Spectral(&rfm95_handle);

	}

}

void Setup_Spectralsensor(void) {

	for (int i = 0; i < ANZAHL_SPECTRALSENSOR; ++i) {
		PCA9847_SetChannel(&multiplexer, i);
		AS7341_Init(eSpm);
		AS7341_ATIME_config(100);
		AS7341_ASTEP_config(999);
		AS7341_AGAIN_config(6);
	}

}

void Read_Spectralsensor(rfm95_handle_t *handle) {
	sModeOneData_t data1;
	sModeTwoData_t data2;
	for (int i = 0; i < ANZAHL_SPECTRALSENSOR; ++i) {
		PCA9847_SetChannel(&multiplexer, i);
		AS7341_startMeasure(eF1F4ClearNIR);
		handle->spectraldata_1[i] = AS7341_ReadSpectralDataOne();
		AS7341_startMeasure(eF5F8ClearNIR);
		handle->spectraldata_2[i] = AS7341_ReadSpectralDataTwo();
	}

}
void Write_SD_Card_Spectral(rfm95_handle_t *handle) {
	//some variables for FatFs
	FATFS FatFs; 	//Fatfs handle
	FIL fil; 		//File handle
	FRESULT fres; //Result after operations

	//Open the file system
	fres = f_mount(&FatFs, "", 1); //1=mount now
	if (fres != FR_OK) {
		myprintf("f_mount error (%i)\r\n", fres);
		return;
	}

	//Now let's try and write a file "write.txt"

	//BYTE readBuf[30];
	UINT bytesWrote;
	fres = f_open(&fil, "write.txt",
	FA_WRITE | FA_OPEN_APPEND | FA_OPEN_EXISTING);
	//strncpy((char*) readBuf, "Test", 30);
	char data_packet[51];
	int cx;

	for (int i = 0; i < ANZAHL_SPECTRALSENSOR; ++i) {
		cx = snprintf(data_packet, 51, "%d;%d;%d;%d;%d;%d;%d;%d;%d;%d\r\n",
				handle->spectraldata_1[i].CLEAR, handle->spectraldata_1[i].NIR,
				handle->spectraldata_1[i].channel1,
				handle->spectraldata_1[i].channel2,
				handle->spectraldata_1[i].channel3,
				handle->spectraldata_1[i].channel4,
				handle->spectraldata_2[i].channel5,
				handle->spectraldata_2[i].channel6,
				handle->spectraldata_2[i].channel7,
				handle->spectraldata_2[i].channel8);
		fres = f_write(&fil, data_packet, cx, &bytesWrote);
	}
	cx = snprintf(data_packet, 51, "%f,%f,%f,%f\r\n", handle->tem_innen,
			handle->tem_aussen, handle->rph_innen, handle->rph_aussen);
	fres = f_write(&fil, data_packet, cx, &bytesWrote);
	cx = snprintf(data_packet, 51, "\r\n");
	fres = f_write(&fil, data_packet, cx, &bytesWrote);
	f_close(&fil);

}

void Write_SD_Card(rfm95_handle_t *handle) {

	//some variables for FatFs
	FATFS FatFs; 	//Fatfs handle
	FIL fil; 		//File handle
	FRESULT fres; //Result after operations
	//FSIZE_t pointer; // pointer of the last write action
//
//		//Open the file system
//		fres = f_mount(&FatFs, "", 1); //1=mount now
//		if (fres != FR_OK) {
//			myprintf("f_mount error (%i)\r\n", fres);
//			while (1)
//				;
//		}
//
//		//Let's get some statistics from the SD card
//		DWORD free_clusters, free_sectors, total_sectors;
//
//		FATFS *getFreeFs;
//
//		fres = f_getfree("", &free_clusters, &getFreeFs);
//		if (fres != FR_OK) {
//			myprintf("f_getfree error (%i)\r\n", fres);
//			while (1)
//				;
//		}
//
//		//Formula comes from ChaN's documentation
//		total_sectors = (getFreeFs->n_fatent - 2) * getFreeFs->csize;
//		free_sectors = free_clusters * getFreeFs->csize;
//
//		myprintf(
//				"SD card stats:\r\n%10lu KiB total drive space.\r\n%10lu KiB available.\r\n",
//				total_sectors / 2, free_sectors / 2);
//
//		//Now let's try to open file "test.txt"
//		fres = f_open(&fil, "test.txt", FA_READ);
//		if (fres != FR_OK) {
//			myprintf("f_open error (%i)\r\n");
//			while (1)
//				;
//		}
//		myprintf("I was able to open 'test.txt' for reading!\r\n");
//
//		//Read 30 bytes from "test.txt" on the SD card
//		BYTE readBuf[30];
//
//		//We can either use f_read OR f_gets to get data out of files
//		//f_gets is a wrapper on f_read that does some string formatting for us
//		TCHAR *rres = f_gets((TCHAR*) readBuf, 30, &fil);
//		if (rres != 0) {
//			myprintf("Read string from 'test.txt' contents: %s\r\n", readBuf);
//		} else {
//			myprintf("f_gets error (%i)\r\n", fres);
//		}
//
//		//Be a tidy kiwi - don't forget to close your file!
//		f_close(&fil);
//
//		//Now let's try and write a file "write.txt"
//		fres = f_open(&fil, "write.txt",
//		FA_WRITE | FA_OPEN_ALWAYS | FA_CREATE_ALWAYS);
//		if (fres == FR_OK) {
//			myprintf("I was able to open 'write.txt' for writing\r\n");
//		} else {
//			myprintf("f_open error (%i)\r\n", fres);
//		}
//
//		//Copy in a string
//		strncpy((char*) readBuf, "a new file is made!", 19);
//		UINT bytesWrote;
//		fres = f_write(&fil, readBuf, 19, &bytesWrote);
//		if (fres == FR_OK) {
//			myprintf("Wrote %i bytes to 'write.txt'!\r\n", bytesWrote);
//		} else {
//			myprintf("f_write error (%i)\r\n");
//		}
//
//		//Be a tidy kiwi - don't forget to close your file!
//		f_close(&fil);
//
//		//We're done, so de-mount the drive
//		f_mount(NULL, "", 0);

	//Open the file system
	static uint8_t counter = 0;
	if (counter == 0) {
		fres = f_mount(&FatFs, "", 1); //1=mount now
		if (fres != FR_OK) {
			myprintf("f_mount error (%i)\r\n", fres);
			while (1)
				;
		}
	}

	//Now let's try and write a file "write.txt"

	//BYTE readBuf[30];
	UINT bytesWrote;
	fres = f_open(&fil, "write.txt",
	FA_WRITE | FA_OPEN_APPEND | FA_OPEN_EXISTING);
	//strncpy((char*) readBuf, "Test", 30);
	char data_packet[51];
	int cx;
	cx = snprintf(data_packet, 51, "{%d,%c,%d,%c,%d,%d}\r\n", handle->longitude,
			handle->longitude_or[0], handle->latitude, handle->latitude_or[0],
			handle->altitude, counter);

	fres = f_write(&fil, data_packet, cx, &bytesWrote);
	cx = snprintf(data_packet, 51, "{Test,%d}\r\n", counter);
	fres = f_write(&fil, data_packet, cx, &bytesWrote);

	f_close(&fil);
	counter++;
}

read_temp_innen(rfm95_handle_t *handle) {
	PCA9847_SetChannel(&multiplexer, 5);
	HAL_StatusTypeDef status;
	uint8_t value[6] = { 0 };
	uint8_t cmd = 0xFD;
	status = HAL_I2C_Master_Transmit(&hi2c1, 0x44 << 1, &cmd, 1, 10);
	HAL_Delay(10);
	status = HAL_I2C_Master_Receive(&hi2c1, 0x44 << 1, value, 6, 10);

	float t_ticks = value[0] * 256 + value[1];
	float checksum_t = value[2];
	float rh_ticks = value[3] * 256 + value[4];
	float checksum_rh = value[5];
	float t_degC = -45 + 175 * t_ticks / 65535;
	float rh_pRH = -6 + 125 * rh_ticks / 65535;
	if (rh_pRH > 100) {
		rh_pRH = 100;
	}
	if (rh_pRH < 0) {
		rh_pRH = 0;
	}
	handle->tem_innen = t_degC;
	handle->rph_innen = rh_pRH;
}

read_temp_aussen(rfm95_handle_t *handle) {
	PCA9847_SetChannel(&multiplexer, 4);
	HAL_StatusTypeDef status;
	uint8_t value[6] = { 0 };
	uint8_t cmd = 0xFD;
	status = HAL_I2C_Master_Transmit(&hi2c1, 0x44 << 1, &cmd, 1, 10);
	HAL_Delay(10);
	status = HAL_I2C_Master_Receive(&hi2c1, 0x44 << 1, value, 6, 10);

	float t_ticks = value[0] * 256 + value[1];
	float checksum_t = value[2];
	float rh_ticks = value[3] * 256 + value[4];
	float checksum_rh = value[5];
	float t_degC = -45 + 175 * t_ticks / 65535;
	float rh_pRH = -6 + 125 * rh_ticks / 65535;
	if (rh_pRH > 100) {
		rh_pRH = 100;
	}
	if (rh_pRH < 0) {
		rh_pRH = 0;
	}
	handle->tem_aussen = t_degC;
	handle->rph_aussen = rh_pRH;
}

void read_akkuspannung() {
	uint16_t raw;
//	HAL_ADC_Start(&hadc2);
//	HAL_ADC_PollForConversion(&hadc2, HAL_MAX_DELAY);
//	raw = HAL_ADC_GetValue(&hadc2);
//	rfm95_handle.battery_voltage = raw;
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
	__disable_irq();
	while (1) {
	}
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
