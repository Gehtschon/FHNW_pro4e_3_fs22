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
#include "../../Core/Inc/stm32-hal-rfm95-master/rfm95.h"
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

LPTIM_HandleTypeDef hlptim1;

SPI_HandleTypeDef hspi1;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_I2C3_Init(void);
static void MX_SPI1_Init(void);
static void MX_LPTIM1_Init(void);
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
int main(void)
{
  /* USER CODE BEGIN 1 */
	uint8_t buf[12]; // For UART
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
  MX_SPI1_Init();
  MX_LPTIM1_Init();
  /* USER CODE BEGIN 2 */
	rfm95_handle_t rfm95_handle =
			{ .spi_handle = &hspi1, .nss_port = GPIOB, .nss_pin = GPIO_PIN_6,
					.nrst_port = RFM95_NRST_GPIO_Port, .nrst_pin =
							RFM95_NRST_Pin, .device_address = { 0x82, 0x0D, 0x0B, 0x26 },
							.application_session_key = { 0x28, 0xA8, 0x53, 0x35, 0xD6, 0x72, 0xA9, 0x7B, 0xC5, 0x94, 0xC8, 0x59, 0x51, 0x43, 0x84, 0xC9 },
					.network_session_key = { 0x24, 0xDA, 0x81, 0x15, 0xAE, 0x41, 0x7D, 0x54, 0xD7, 0x90, 0x85, 0x1F, 0x9A, 0xD7, 0x8B, 0xBA },
							.precision_tick_frequency = 32768,
							    .precision_tick_drift_ns_per_s = 5000,
							    .receive_mode = RFM95_RECEIVE_MODE_RX12,
							    .get_precision_tick = get_precision_tick,
							    .precision_sleep_until = precision_sleep_until,
							    .random_int = random_int,
							    .get_battery_level = get_battery_level};



	// Initialise RFM95 module.
	if (!rfm95_init(&rfm95_handle)) {
		printf("RFM95 init failed\n\r");
	}else {
		printf("RFM95 init sucess\n\r");
	}

	uint8_t data_packet[] = { 0x01, 0x02, 0x03, 0x4 };

	if (!rfm95_send_receive_cycle(&rfm95_handle, data_packet,
			sizeof(data_packet))) {
		printf("RFM95 send failed\n\r");
	} else {
		printf("RFM95 send success\n\r");
	}


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
while (1) {

	//strcpy((char*) buf, "Startup\r\n");
//	HAL_UART_Transmit(&huart2, buf, strlen((char*) buf), HAL_MAX_DELAY);
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
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C3_Init(void)
{

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
  if (HAL_I2C_Init(&hi2c3) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c3, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c3, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C3_Init 2 */

  /* USER CODE END I2C3_Init 2 */

}

/**
  * @brief LPTIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_LPTIM1_Init(void)
{

  /* USER CODE BEGIN LPTIM1_Init 0 */

  /* USER CODE END LPTIM1_Init 0 */

  /* USER CODE BEGIN LPTIM1_Init 1 */

  /* USER CODE END LPTIM1_Init 1 */
  hlptim1.Instance = LPTIM1;
  hlptim1.Init.Clock.Source = LPTIM_CLOCKSOURCE_APBCLOCK_LPOSC;
  hlptim1.Init.Clock.Prescaler = LPTIM_PRESCALER_DIV1;
  hlptim1.Init.Trigger.Source = LPTIM_TRIGSOURCE_SOFTWARE;
  hlptim1.Init.OutputPolarity = LPTIM_OUTPUTPOLARITY_HIGH;
  hlptim1.Init.UpdateMode = LPTIM_UPDATE_IMMEDIATE;
  hlptim1.Init.CounterSource = LPTIM_COUNTERSOURCE_INTERNAL;
  hlptim1.Init.Input1Source = LPTIM_INPUT1SOURCE_GPIO;
  hlptim1.Init.Input2Source = LPTIM_INPUT2SOURCE_GPIO;
  if (HAL_LPTIM_Init(&hlptim1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN LPTIM1_Init 2 */

  /* USER CODE END LPTIM1_Init 2 */

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
  hspi1.Init.NSS = SPI_NSS_HARD_INPUT;
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
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

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
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
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
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, RFM95_NRST_Pin|LD4_Pin|NSS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : RFM95_NRST_Pin LD4_Pin NSS_Pin */
  GPIO_InitStruct.Pin = RFM95_NRST_Pin|LD4_Pin|NSS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

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

volatile uint32_t lptim_tick_msb = 0;

static uint32_t get_precision_tick()
{
    __disable_irq();
    uint32_t precision_tick = lptim_tick_msb | HAL_LPTIM_ReadCounter(&hlptim1);
    __enable_irq();
    return precision_tick;
}

void HAL_LPTIM_AutoReloadMatchCallback(LPTIM_HandleTypeDef *hlptim)
{
    lptim_tick_msb += 0x10000;
}

static void precision_sleep_until(uint32_t target_ticks)
{
    while (true) {

    uint32_t start_ticks = get_precision_tick();
    if (start_ticks > target_ticks) {
        break;
    }

    uint32_t ticks_to_sleep = target_ticks - start_ticks;

    // Only use sleep for at least 10 ticks.
    if (ticks_to_sleep >= 10) {

        // Calculate required value of compare register for the sleep minus a small buffer time to compensate
        // for any ticks that occur while we perform this calculation.
        uint32_t compare = (start_ticks & 0xffff) + ticks_to_sleep - 2;

        // If the counter auto-reloads we will be woken up anyway.
        if (compare > 0xffff) {
            HAL_SuspendTick();
            HAL_PWREx_EnterSTOP2Mode(PWR_STOPENTRY_WFI);
            HAL_ResumeTick();

        // Otherwise, set compare register and use the compare match interrupt to wake up in time.
        } else {
            __HAL_LPTIM_COMPARE_SET(&hlptim1, compare);
            while (!__HAL_LPTIM_GET_FLAG(&hlptim1, LPTIM_FLAG_CMPOK));
            __HAL_LPTIM_CLEAR_FLAG(&hlptim1, LPTIM_FLAG_CMPM);
            __HAL_LPTIM_ENABLE_IT(&hlptim1, LPTIM_IT_CMPM);
            HAL_SuspendTick();
            HAL_PWREx_EnterSTOP2Mode(PWR_STOPENTRY_WFI);
            HAL_ResumeTick();
            __HAL_LPTIM_DISABLE_IT(&hlptim1, LPTIM_IT_CMPM);
        }
    } else {
        break;
    }

    // Busy wait until we have reached the target.
    while (get_precision_tick() < target_ticks);
}
}

static uint8_t random_int(uint8_t max)
{
    return 0; // Use ADC other means of obtaining a random number.
}

static uint8_t get_battery_level()
{
    return 0xff; // 0xff = Unknown battery level.
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
