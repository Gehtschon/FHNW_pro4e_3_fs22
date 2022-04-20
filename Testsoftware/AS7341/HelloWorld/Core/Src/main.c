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

#include <cmsis_gcc.h>
#include <main.h>
#include <stm32l452xx.h>
#include <stm32l4xx_hal_def.h>
#include <stm32l4xx_hal_flash.h>
#include <stm32l4xx_hal_gpio.h>
#include <stm32l4xx_hal_i2c.h>
#include <stm32l4xx_hal_i2c_ex.h>
#include <stm32l4xx_hal_pwr_ex.h>
#include <stm32l4xx_hal_rcc.h>
#include <stm32l4xx_hal_uart.h>
#include <stm32l4xx_hal_uart_ex.h>
#include <string.h>
#include <sys/_stdint.h>

#include "../../Drivers/AS7341_Driver/Waveshare_AS7341.h"

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
I2C_HandleTypeDef hi2c3;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
static const uint8_t AS7341_ADDR = 0x39 << 1; //Use 8 bit adress
static const uint8_t CH0_Data_L = 0x61;
static const uint8_t CH0_Data_H = 0x62;

static const uint8_t INTEGRATION_TIME_COUNT[1] = { 100 };
static const uint16_t INTEGRATION_TIME_SIZE[1] = { 999 };
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_I2C3_Init(void);
/* USER CODE BEGIN PFP */
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void) {
	/* USER CODE BEGIN 1 */
	HAL_StatusTypeDef ret;
	uint8_t buf[12]; // For UART
	uint16_t val_channel0;

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
	MX_USART2_UART_Init();
	MX_I2C3_Init();
	/* USER CODE BEGIN 2 */
	AS7341_Init(eSpm);
	AS7341_ATIME_config(100);
	AS7341_ASTEP_config(999);
	AS7341_AGAIN_config(6);

	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1) {
		//	strcpy((char*) buf, "Startup\r\n");
		//HAL_UART_Transmit(&huart2, buf, strlen((char*) buf), HAL_MAX_DELAY);

		sModeOneData_t data1;
		sModeTwoData_t data2;
		AS7341_startMeasure(eF1F4ClearNIR);
		data1 = AS7341_ReadSpectralDataOne();
		printf("channel1(405-425nm):\r\n");
		printf("%d\r\n", data1.channel1);
		printf("channel2(435-455nm):\r\n");
		printf("%d\r\n", data1.channel2);
		printf("channel3(470-490nm):\r\n");
		printf("%d\r\n", data1.channel3);
		printf("channel4(505-525nm):\r\n");
		printf("%d\r\n", data1.channel4);
		AS7341_startMeasure(eF5F8ClearNIR);
		data2 = AS7341_ReadSpectralDataTwo();
		printf("channel5(545-565nm):\r\n");
		printf("%d\r\n", data2.channel5);
		printf("channel6(580-600nm):\r\n");
		printf("%d\r\n", data2.channel6);
		printf("channel7(620-640nm):\r\n");
		printf("%d\r\n", data2.channel7);
		printf("channel8(670-690nm):\r\n");
		printf("%d\r\n", data2.channel8);
		printf("Clear:\r\n");
		printf("%d\r\n", data2.CLEAR);
		printf("NIR:\r\n");
		printf("%d\r\n", data2.NIR);
		printf("--------------------------\r\n");
		DEV_Delay_ms(500);

//		// Start Powerup
//		buf[0] = 1;
//		ret = HAL_I2C_Mem_Write(&hi2c3, AS7341_ADDR, AS7341_ENABLE,
//		I2C_MEMADD_SIZE_8BIT, buf, 1, 200);
//		if (ret != HAL_OK) {
//			strcpy((char*) buf, "Error PON\r\n");
//		}
//		// End Powerup
//
//		// Start Set Integration Time step count
//		ret = HAL_I2C_Mem_Write(&hi2c3, AS7341_ADDR, AS7341_ATIME,
//		I2C_MEMADD_SIZE_8BIT, (uint8_t *)INTEGRATION_TIME_COUNT, 1, 200);
//		if (ret != HAL_OK) {
//			strcpy((char*) buf, "Error Set Int.Time count\r\n");
//		}
//		// End Set Integration Time step count
//
//		//	Start Set the Integration Time step size
//		ret = HAL_I2C_Mem_Write(&hi2c3, AS7341_ADDR, AS7341_ASTEP_L,
//		I2C_MEMADD_SIZE_8BIT, INTEGRATION_TIME_SIZE, 2, 200);
//		if (ret != HAL_OK) {
//			strcpy((char*) buf, "Error Set Int.Time size\r\n");
//		}

		// Stop Set the Integration Time step size

//		buf[0] = CH0_Data_L;
//		ret = HAL_I2C_Master_Transmit(&hi2c3, AS7341_ADDR, buf, 1,
//		1000);
//		if (ret != HAL_OK) {
//			strcpy((char*) buf, "Error Tx\r\n");
//		} else {
//			ret = HAL_I2C_Master_Receive(&hi2c3, AS7341_ADDR, buf, 2,
//			1000);
//			if (ret != HAL_OK) {
//				strcpy((char*) buf, "Error Rx\r\n");
//			} else {
//				// Combine the bytes
//				val_channel0 = ((uint16_t) buf[0] << 4) | buf[1] >> 4;
//
//			}
//
//			// Convert value to string
//			sprintf((char*)buf,"%d",val_channel0);
//
//		}

		//strcpy((char*) buf, "Hello! \r\n");
		//HAL_UART_Transmit(&huart2, buf, strlen((char*) buf), HAL_MAX_DELAY);
		//HAL_Delay(500);

		HAL_GPIO_TogglePin(LD4_GPIO_Port, LD4_Pin);
		//HAL_Delay(1000);
		/* USER CODE END WHILE */

		/* USER CODE BEGIN 3 */
	}
	/* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void) {
	RCC_OscInitTypeDef RCC_OscInitStruct = { 0 };
	RCC_ClkInitTypeDef RCC_ClkInitStruct = { 0 };

	/** Configure the main internal regulator output voltage
	 */
	if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1)
			!= HAL_OK) {
		Error_Handler();
	}

	/** Initializes the RCC Oscillators according to the specified parameters
	 * in the RCC_OscInitTypeDef structure.
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
	RCC_OscInitStruct.HSIState = RCC_HSI_ON;
	RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
	RCC_OscInitStruct.PLL.PLLM = 1;
	RCC_OscInitStruct.PLL.PLLN = 10;
	RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
	RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
	RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
		Error_Handler();
	}

	/** Initializes the CPU, AHB and APB buses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
			| RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK) {
		Error_Handler();
	}
}

/**
 * @brief I2C3 Initialization Function
 * @param None
 * @retval None
 */
static void MX_I2C3_Init(void) {

	/* USER CODE BEGIN I2C3_Init 0 */

	/* USER CODE END I2C3_Init 0 */

	/* USER CODE BEGIN I2C3_Init 1 */

	/* USER CODE END I2C3_Init 1 */
	hi2c3.Instance = I2C3;
	hi2c3.Init.Timing = 0x10909CEC;
	hi2c3.Init.OwnAddress1 = 0;
	hi2c3.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
	hi2c3.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
	hi2c3.Init.OwnAddress2 = 0;
	hi2c3.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
	hi2c3.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
	hi2c3.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
	if (HAL_I2C_Init(&hi2c3) != HAL_OK) {
		Error_Handler();
	}

	/** Configure Analogue filter
	 */
	if (HAL_I2CEx_ConfigAnalogFilter(&hi2c3, I2C_ANALOGFILTER_ENABLE)
			!= HAL_OK) {
		Error_Handler();
	}

	/** Configure Digital filter
	 */
	if (HAL_I2CEx_ConfigDigitalFilter(&hi2c3, 0) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN I2C3_Init 2 */

	/* USER CODE END I2C3_Init 2 */

}

/**
 * @brief USART2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_USART2_UART_Init(void) {

	/* USER CODE BEGIN USART2_Init 0 */

	/* USER CODE END USART2_Init 0 */

	/* USER CODE BEGIN USART2_Init 1 */

	/* USER CODE END USART2_Init 1 */
	huart2.Instance = USART2;
	huart2.Init.BaudRate = 115200;
	huart2.Init.WordLength = UART_WORDLENGTH_8B;
	huart2.Init.StopBits = UART_STOPBITS_1;
	huart2.Init.Parity = UART_PARITY_NONE;
	huart2.Init.Mode = UART_MODE_TX_RX;
	huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart2.Init.OverSampling = UART_OVERSAMPLING_16;
	huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
	huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
	if (HAL_UART_Init(&huart2) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN USART2_Init 2 */

	/* USER CODE END USART2_Init 2 */

}

/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void) {
	GPIO_InitTypeDef GPIO_InitStruct = { 0 };

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOC_CLK_ENABLE();
	__HAL_RCC_GPIOH_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(LD4_GPIO_Port, LD4_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin : B1_Pin */
	GPIO_InitStruct.Pin = B1_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pin : LD4_Pin */
	GPIO_InitStruct.Pin = LD4_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(LD4_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
/**
 * @brief  Retargets the C library printf function to the USART.
 * @param  None
 * @retval None
 */
PUTCHAR_PROTOTYPE {
	/* Place your implementation of fputc here */
	/* e.g. write a character to the USART1 and Loop until the end of transmission */
	HAL_UART_Transmit(&huart2, (uint8_t*) &ch, 1, 0xFFFF);

	return ch;
}

/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void) {
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
