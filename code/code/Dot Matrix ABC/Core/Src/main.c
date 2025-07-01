/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
#define NUM_matrix 4
void send_data(uint8_t addr, uint8_t data);
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */
	uint8_t val;
	uint8_t font[26][8] = {
	    {0x18, 0x24, 0x42, 0x42, 0x7E, 0x42, 0x42, 0x00}, // A → {0x18, 0x24, 0x42, 0x42, 0x7E, 0x42, 0x42, 0x00}
	    {0x3E, 0x42, 0x42, 0x3E, 0x42, 0x42, 0x3E, 0x00}, // B
	    {0x3C, 0x42, 0x01, 0x01, 0x01, 0x42, 0x3C, 0x00}, // C
	    {0x1E, 0x22, 0x42, 0x42, 0x42, 0x22, 0x1E, 0x00}, // D
	    {0x7E, 0x02, 0x02, 0x3E, 0x02, 0x02, 0x7E, 0x00}, // E
	    {0x7E, 0x02, 0x02, 0x3E, 0x02, 0x02, 0x02, 0x00}, // F
	    {0x3C, 0x42, 0x01, 0x71, 0x41, 0x42, 0x3C, 0x00}, // G
	    {0x42, 0x42, 0x42, 0x7E, 0x42, 0x42, 0x42, 0x00}, // H
	    {0x7C, 0x10, 0x10, 0x10, 0x10, 0x10, 0x7C, 0x00}, // I
	    {0x78, 0x20, 0x20, 0x20, 0x21, 0x21, 0x1E, 0x00}, // J
	    {0x42, 0x22, 0x12, 0x0E, 0x12, 0x22, 0x42, 0x00}, // K
	    {0x02, 0x02, 0x02, 0x02, 0x02, 0x02, 0x7E, 0x00}, // L
	    {0x42, 0x66, 0x5A, 0x5A, 0x42, 0x42, 0x42, 0x00}, // M
	    {0x42, 0x46, 0x4A, 0x52, 0x62, 0x42, 0x42, 0x00}, // N
	    {0x3C, 0x42, 0x42, 0x42, 0x42, 0x42, 0x3C, 0x00}, // O
	    {0x3E, 0x42, 0x42, 0x3E, 0x02, 0x02, 0x02, 0x00}, // P
	    {0x3C, 0x42, 0x42, 0x42, 0x4A, 0x32, 0x5C, 0x00}, // Q
	    {0x3E, 0x42, 0x42, 0x3E, 0x12, 0x22, 0x42, 0x00}, // R
	    {0x3C, 0x42, 0x02, 0x3C, 0x40, 0x42, 0x3C, 0x00}, // S
	    {0xFE, 0x10, 0x10, 0x10, 0x10, 0x10, 0x10, 0x00}, // T
	    {0x42, 0x42, 0x42, 0x42, 0x42, 0x42, 0x3C, 0x00}, // U
	    {0x42, 0x42, 0x42, 0x42, 0x42, 0x24, 0x18, 0x00}, // V
	    {0x42, 0x42, 0x42, 0x5A, 0x5A, 0x66, 0x42, 0x00}, // W
	    {0x42, 0x42, 0x24, 0x18, 0x24, 0x42, 0x42, 0x00}, // X
	    {0x42, 0x42, 0x24, 0x18, 0x10, 0x10, 0x10, 0x00}, // Y
	    {0x7E, 0x40, 0x20, 0x10, 0x08, 0x04, 0x7E, 0x00}  // Z
	};

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
  /* USER CODE BEGIN 2 */
  RCC->APB2ENR |= RCC_APB2ENR_SPI1EN;
  RCC->AHB2ENR |= RCC_AHB2ENR_GPIOBEN;
  RCC->AHB2ENR |= RCC_AHB2ENR_GPIOAEN;

  //PA4 pin ss

    GPIOA->MODER &= ~GPIO_MODER_MODE4_1;
    GPIOA->MODER |= GPIO_MODER_MODE4_0;
    GPIOA->ODR |= GPIO_ODR_OD4;
//  GPIOA->OSPEEDR |= (GPIO_OSPEEDER_OSPEEDR4_0 | GPIO_OSPEEDER_OSPEEDR4_1);
//  GPIOA->AFR[0] |= (GPIO_AFRL_AFSEL4_0 | GPIO_AFRL_AFSEL4_2);

  //PA5 CLK
  GPIOA->MODER &= ~GPIO_MODER_MODE5_0;
  GPIOA->OSPEEDR |= (GPIO_OSPEEDER_OSPEEDR5_0 | GPIO_OSPEEDER_OSPEEDR5_1);
  GPIOA->AFR[0] |= (GPIO_AFRL_AFSEL5_0 | GPIO_AFRL_AFSEL5_2);

  //PA6 MISO pin
  GPIOA->MODER &= ~GPIO_MODER_MODE6_0;
  GPIOA->OSPEEDR |= (GPIO_OSPEEDER_OSPEEDR6_0 | GPIO_OSPEEDER_OSPEEDR6_1);
  GPIOA->AFR[0] |= (GPIO_AFRL_AFSEL6_0 | GPIO_AFRL_AFSEL6_2);

  //PA7 MOSI pin
  GPIOA->MODER &= ~GPIO_MODER_MODE7_0;
  GPIOA->OSPEEDR |= (GPIO_OSPEEDER_OSPEEDR7_0 | GPIO_OSPEEDER_OSPEEDR7_1);
  GPIOA->AFR[0] |= (GPIO_AFRL_AFSEL7_0 | GPIO_AFRL_AFSEL7_2);
//
//  GPIOB->MODER &= ~GPIO_MODER_MODE5_1;
//  GPIOB->MODER |= GPIO_MODER_MODE5_0;
//  GPIOB->OSPEEDR |=(GPIO_OSPEEDER_OSPEEDR5_0 | GPIO_OSPEEDER_OSPEEDR5_1);
//  GPIOB->ODR |= GPIO_ODR_OD5;

  SPI1->CR1 |= SPI_CR1_MSTR | SPI_CR1_CPOL | SPI_CR1_BR_0 | SPI_CR1_BR_1 | SPI_CR1_BR_2;
  SPI1->CR2 |= SPI_CR2_DS_3 ;
  SPI1->CR2 |= SPI_CR2_SSOE ;
  SPI1->CR1 |= SPI_CR1_SPE;

 // data
// 16 data formate (8 + 8)
  send_data(0x09,0x00);
  send_data(0x0a,0x01);
  send_data(0x0b,0x07);
  send_data(0x0c,0x01);
  send_data(0x0f,0x00);
  uint8_t i,j;
  for(j=0;j<26;j++)
  {
	  for(i=1;i<=8;i++)
	  {
		  send_data(i,font[j][8-i]);

	  }
  	  HAL_Delay(1000);
  }

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  val++;
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

/* USER CODE BEGIN 4 */
void send_data(uint8_t addr, uint8_t data)
{

//	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, 0);
	  uint16_t write_data =((addr<<8)|data);
	for(int i=0;i<4;i++)
	{
		GPIOA->ODR &= ~GPIO_ODR_OD4;
		SPI1->DR = write_data; // data formate for dot matrix
			while((SPI1->SR & SPI_SR_TXE) != SPI_SR_TXE);
			HAL_Delay(1);
		GPIOA->ODR |= GPIO_ODR_OD4;// wait for set TXE pin
	}
//	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, 1);

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
  while (1)
  {
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
