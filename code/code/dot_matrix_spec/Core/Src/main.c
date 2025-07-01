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

#define NUM_DEV 4
uint8_t bufferCol[NUM_DEV*8];

void send_data(uint8_t addr, uint8_t data);
void max_write (int row, uint8_t data);
void flushBuffer (void);
void clearDisplay (void);
const uint8_t data[] =
{
		0x00, 0x00, 0x22, 0x22, 0x3e, 0x08, 0x08, 0x00
};

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */
	int val;
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

    GPIOB->MODER &= ~GPIO_MODER_MODE5_1;
    GPIOB->MODER |= GPIO_MODER_MODE5_0;
    GPIOB->ODR |= GPIO_ODR_OD5;
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

  SPI1->CR1 |= SPI_CR1_MSTR | SPI_CR1_BR_0 | SPI_CR1_BR_1 | SPI_CR1_BR_2;
  SPI1->CR2 |= SPI_CR2_DS_3 ;
  SPI1->CR2 |= SPI_CR2_SSOE ;
  SPI1->CR1 |= SPI_CR1_SPE;

  send_data(0x09,0x00);
  send_data(0x0a,0x01);
  send_data(0x0b,0x07);
  send_data(0x0c,0x01);
  send_data(0x0f,0x00);

  clearDisplay();
  flushBuffer();

  for (int i=0; i<8; i++)
  {
	  bufferCol[i] = data[7-i];
  }
  flushBuffer();


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
		GPIOB->ODR &= ~GPIO_ODR_OD5;
		SPI1->DR = write_data; // data formate for dot matrix
			while((SPI1->SR & SPI_SR_TXE) != SPI_SR_TXE);
			HAL_Delay(10);
		GPIOB->ODR |= GPIO_ODR_OD5;// wait for set TXE pin
	}
//	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, 1);

}
void max_write (int row, uint8_t data)
{
	int devTarget = (row - 1) / 8;  // find out which is the actual max, where we need to write the data
	int offset = devTarget * 8;  // The offset of the start byte for the devTarget in the buffer
	uint16_t writeData = 0;

  // Select the slave
	for (int dev = 0; dev < NUM_DEV; dev++)   // for loop for all the max connected
	{
		if (dev == devTarget)  // if this the target
		{
			writeData = ((row - offset)<<8)|data;  // send the column number and the data byte
			GPIOB->ODR &= ~GPIO_ODR_OD5;
			SPI1->DR = writeData; // data formate for dot matrix
				while((SPI1->SR & SPI_SR_TXE) != SPI_SR_TXE);
				HAL_Delay(10);
			GPIOB->ODR |= GPIO_ODR_OD5;
		}
		else
		{
			writeData = 0;  // else send NOOP
			GPIOB->ODR &= ~GPIO_ODR_OD5;
			SPI1->DR = writeData; // data formate for dot matrix
				while((SPI1->SR & SPI_SR_TXE) != SPI_SR_TXE);
				HAL_Delay(10);
			GPIOB->ODR |= GPIO_ODR_OD5;
		}
	}
}
void flushBuffer (void)
{
	uint8_t bufferRow[NUM_DEV*8] = {0};  // buffer to store data column wise

	/* Convert Cols to Rows */
	for (int i=0; i<NUM_DEV*8; i++)  // for loop for all the bytes
	{
		int dev = i/8;  // 0,1,2,3..  // keep track of which max is being written
		for (int j=0; j<8; j++)  // for loop to extract bits
		{
			if ((bufferCol[i])&(1<<(j)))  // if the bit is 1 // start extracting from the 0th bit of C0
			{
				bufferRow[j+(8*dev)] |= (1<<(7-(i-(8*dev))));  // start writing it from the 7th bit of R0
			}
		}
	}


	for (int row=1; row<=(NUM_DEV*8); row++)  // write the column data into the columns
	{
		max_write(row, bufferRow[row-1]);
	}
}
void clearDisplay (void)
{
	for (int i=0; i<NUM_DEV*8-1; i++)
	{
		bufferCol[i] = 0;
	}

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
