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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdio.h"
#include "bmi160.h"
#include "bmi160_defs.h"
#include "stdio.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
//#define BMI160_DEV_ADDR (BMI160_I2C_ADDR)
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
/*! i2c interface communication, 1 - Enable; 0- Disable */
#define BMI160_INTERFACE_I2C  1

/*! spi interface communication, 1 - Enable; 0- Disable */
#define BMI160_INTERFACE_SPI  0

/*! bmi160 Device address */
#define BMI160_DEV_ADDR       BMI160_I2C_ADDR
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

UART_HandleTypeDef huart2;

struct bmi160_dev bmi160dev;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART2_UART_Init(void);
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
	MX_I2C1_Init();
	MX_USART2_UART_Init();
	/* USER CODE BEGIN 2 */
	bmi160dev.id = BMI160_I2C_ADDR;
//	bmi160dev.read = HAL_I2C_Mem_Read;
	//bmi160dev.write = HAL_I2C_Mem_Write;

	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1) {
		uint8_t data;
		uint16_t acc_z;
		uint8_t dev_ID;
		uint16_t status;
		uint32_t trials = 50;
		uint32_t timeout = 50;
		uint16_t size = 0x1;
		uint16_t memAddSize8 = I2C_MEMADD_SIZE_8BIT;
		uint16_t memAddSize16 = I2C_MEMADD_SIZE_16BIT;

		if (HAL_I2C_IsDeviceReady(&hi2c1, (bmi160dev.id << 1), trials, timeout)
				!= HAL_OK) {
			printf("\nDevice not ready\r");
		};
		if (HAL_I2C_IsDeviceReady(&hi2c1, (bmi160dev.id << 1), trials, timeout)
				== HAL_OK) {
			printf("\nDevice ready\r");
		};

		status = HAL_I2C_Mem_Read(&hi2c1, (bmi160dev.id << 1),
		BMI160_CHIP_ID_ADDR, memAddSize8, &data, size, timeout);

		if (status == HAL_OK) {
			printf("\nI2C ok\r");
		};
		if (status != HAL_OK) {
			printf("\nI2C read error\r");
			if (hi2c1.ErrorCode == HAL_I2C_ERROR_TIMEOUT) {
				printf(" - HAL_I2C_ERROR_TIMEOUT\r"); // HAL_I2C_ERROR_TIMEOUT
			}

		} else {
			dev_ID = data;
			printf("\nDevice-ID=%X\r", dev_ID);
		};
		HAL_Delay(1);

		// Process to initialize magnetometer to low power preset at 12.5Hz and enable magnetometer interface data mode
		// see also DS of BMX160 p.25
		// put MAG_IF into normal mode
		HAL_I2C_Mem_Write(&hi2c1, (bmi160dev.id << 1), 0x7E, memAddSize8, 0x19,
				size, timeout);
		HAL_Delay(1);

		// mag_manual_en= 0b1, mag_if setup mode mag_offset<3:0>= 0b0000, maximum offset, recommend for
		// BSX library
		// enable magnetometer register access on MAG_IF[1] (read operations) or MAG_IF[2] (write access)
		// setup mode ON, data mode OFF
		HAL_I2C_Mem_Write(&hi2c1, (bmi160dev.id << 1), 0x4C, memAddSize8, 0x80,
				size, timeout);

		// indirect write 0x01 to MAG register 0x4B
		HAL_I2C_Mem_Write(&hi2c1, (bmi160dev.id << 1), 0x4F, memAddSize8, 0x01,
				size, timeout);
		HAL_I2C_Mem_Write(&hi2c1, (bmi160dev.id << 1), 0x4E, memAddSize8, 0x4B,
				size, timeout);

		/* Indirect write REPXY=
		 * 			0x01 for low power preset
		 * 			0x04 for regular preset
		 * 			0x07 for enhanced regular preset
		 * 			0x17 for high accuracy preset
		 to MAG register 0x51
		 */
		HAL_I2C_Mem_Write(&hi2c1, (bmi160dev.id << 1), 0x4F, memAddSize8, 0x01,
				size, timeout);
		HAL_I2C_Mem_Write(&hi2c1, (bmi160dev.id << 1), 0x4E, memAddSize8, 0x51,
				size, timeout);

		/* Indirect write REPZ=
		 * 			0x02 for low power preset
		 * 			0x0E for regular preset
		 * 			0x1A for enhanced regular preset
		 * 			0x52 for high accuracy preset
		 to MAG register 0x52
		 */
		HAL_I2C_Mem_Write(&hi2c1, (bmi160dev.id << 1), 0x4F, memAddSize8, 0x0E,
				size, timeout);
		HAL_I2C_Mem_Write(&hi2c1, (bmi160dev.id << 1), 0x4E, memAddSize8, 0x52,
				size, timeout);

		// Prepare MAG_IF[1-3] for mag_if data mode
		HAL_I2C_Mem_Write(&hi2c1, (bmi160dev.id << 1), 0x4F, memAddSize8, 0x02,
				size, timeout);
		HAL_I2C_Mem_Write(&hi2c1, (bmi160dev.id << 1), 0x4E, memAddSize8, 0x4C,
				size, timeout);
		HAL_I2C_Mem_Write(&hi2c1, (bmi160dev.id << 1), 0x4D, memAddSize8, 0x42,
				size, timeout);

		// mag_odr<3:0>= 0b0101, set ODR to 12.5Hz (50/(2^7-val(mag_odr<3:0>))Hz
		HAL_I2C_Mem_Write(&hi2c1, (bmi160dev.id << 1), 0x44, memAddSize8, 0x05,
				size, timeout);

		// mag_manual_en= 0b0, mag_if data mode mag_offset<3:0>= 0b0000, maximum offset, recommend for BSX library
		HAL_I2C_Mem_Write(&hi2c1, (bmi160dev.id << 1), 0x4C, memAddSize8, 0x00,
				size, timeout);

		// put MAG_IF into low power mode
		HAL_I2C_Mem_Write(&hi2c1, (bmi160dev.id << 1), 0x7E, memAddSize8, 0x1A,
				size, timeout);
		// end of magnetometer initialization

		// rf 07.05.2022 14:16


		HAL_I2C_Mem_Read(&hi2c1, (bmi160dev.id << 1), 0x16, memAddSize16,
				&acc_z, size, timeout);
		printf("\nacc. z = %g\r", acc_z);
		printf("\nacc. z = %u16\r", acc_z);
		HAL_Delay(500);

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
	RCC_PeriphCLKInitTypeDef PeriphClkInit = { 0 };

	/** Configure the main internal regulator output voltage
	 */
	__HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

	/** Initializes the RCC Oscillators according to the specified parameters
	 * in the RCC_OscInitTypeDef structure.
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;
	RCC_OscInitStruct.MSIState = RCC_MSI_ON;
	RCC_OscInitStruct.MSICalibrationValue = 0;
	RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_5;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
		Error_Handler();
	}

	/** Initializes the CPU, AHB and APB buses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
			| RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_MSI;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK) {
		Error_Handler();
	}
	PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART2
			| RCC_PERIPHCLK_I2C1;
	PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
	PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_PCLK1;
	if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK) {
		Error_Handler();
	}
}

/**
 * @brief I2C1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_I2C1_Init(void) {

	/* USER CODE BEGIN I2C1_Init 0 */

	/* USER CODE END I2C1_Init 0 */

	/* USER CODE BEGIN I2C1_Init 1 */

	/* USER CODE END I2C1_Init 1 */
	hi2c1.Instance = I2C1;
	hi2c1.Init.Timing = 0x00000708;
	hi2c1.Init.OwnAddress1 = 0;
	hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
	hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
	hi2c1.Init.OwnAddress2 = 0;
	hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
	hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
	hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
	if (HAL_I2C_Init(&hi2c1) != HAL_OK) {
		Error_Handler();
	}

	/** Configure Analogue filter
	 */
	if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE)
			!= HAL_OK) {
		Error_Handler();
	}

	/** Configure Digital filter
	 */
	if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN I2C1_Init 2 */

	/* USER CODE END I2C1_Init 2 */

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

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();

}

/* USER CODE BEGIN 4 */
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
