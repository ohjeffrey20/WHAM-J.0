/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32g4xx_hal_i2c.h"

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
I2C_HandleTypeDef hi2c2;
UART_HandleTypeDef huart2;
volatile unsigned int TIM2cnt;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void TIM2_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_GPIO_Init(void);
static void MX_I2C2_Init(void);
void TIM2_IRQHandler(void);
void delayms(int ms);
void LCD_Init(void);
void LCD_DC(char c, char DC_Flag);
void I2C_Init(void);
void L2D_Init(void);
void I2C_Write(char w, char DC_Flag);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void){
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
  MX_I2C2_Init();
  MX_USART2_UART_Init();
  TIM2_Init();
  
  I2C_Init();
  L2D_Init();

  /* USER CODE BEGIN 2 */
  // LCD_Init();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while(1){
    HAL_GPIO_TogglePin(GPIOB, LD2_Pin);
    HAL_Delay(100);
  }
}

void LCD_test(void){
  char C = 'A';
  char c = 'a';
  int i = 1;
  LCD_DC(0x01,1); //clear display
	LCD_DC(0x02,1); //return home
  while(i < 26){
    LCD_DC(C,0);
    LCD_DC(c,0);
    C++;
    c++;
    i++;
  }
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Configure the main internal regulator output voltage
  */
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV4;
  RCC_OscInitStruct.PLL.PLLN = 85;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
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
  /** Initializes the peripherals clocks
  */
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART2|RCC_PERIPHCLK_I2C2;
  PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
  PeriphClkInit.I2c2ClockSelection = RCC_I2C2CLKSOURCE_PCLK1; // TODO CHECK THIS (edit from MX)
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
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
  huart2.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart2, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart2, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart2) != HAL_OK)
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
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0|GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7|LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : PB4 PB5 PB6 PB7 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PA9/SCL */
  GPIO_InitStruct.Pin = GPIO_PIN_8 | GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF4_I2C2;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PF0/SDA */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);
}
//static void GPIO_Init(void){
//	uint32_t temp;
//	/* GPIO Ports Clock Enable */
//	RCC->AHB2ENR |= RCC_AHB2ENR_GPIOAEN;
//	RCC->AHB2ENR |= RCC_AHB2ENR_GPIOBEN;
//
//	/*Configure GPIO pin Output Level */
//	LD2_GPIO_Port->BRR |= LD2_Pin;
//
//	/*Configure GPIO pin : LD2_Pin */
//	temp = GPIOB->MODER;
//	temp &= ~(GPIO_PUPDR_PUPD0 << (8U * 2U));
//	temp |= GPIO_MODE_OUTPUT_PP << (8U * 2U);
//	GPIOB->MODER = temp;
//
//	GPIOB->OTYPER &= ~LD2_Pin;	// Push Pull
//
//	temp = GPIOB->OSPEEDR;
//	temp &= ~(GPIO_OSPEEDR_OSPEED0 << (8U * 2U));
//	temp |= GPIO_SPEED_FREQ_LOW << (8U * 2U);
//	GPIOB->OSPEEDR = temp;
//
//	temp = GPIOB->PUPDR;
//	temp &= ~(GPIO_PUPDR_PUPD0 << (8U * 2U));
//	temp |= GPIO_NOPULL << (8U * 2U);
//	GPIOB->PUPDR = temp;
//
//	/*Configure GPIO pin : GPIO_PIN_7 */
//	temp = GPIOB->MODER;
//	temp &= ~(GPIO_PUPDR_PUPD0 << (7U * 2U));
//	temp |= GPIO_MODE_OUTPUT_OD << (7U * 2U);
//	GPIOB->MODER = temp;
//
//	GPIOB->OTYPER &= ~GPIO_PIN_7;	// Push Pull
//
//	temp = GPIOB->OSPEEDR;
//	temp &= ~(GPIO_OSPEEDR_OSPEED0 << (7U * 2U));
//	temp |= GPIO_SPEED_FREQ_LOW << (7U * 2U);
//	GPIOB->OSPEEDR = temp;
//
//	temp = GPIOB->PUPDR;
//	temp &= ~(GPIO_PUPDR_PUPD0 << (7U * 2U));
//	temp |= GPIO_NOPULL << (7U * 2U);
//	GPIOB->PUPDR = temp;
//}

/**
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.Timing = 0x30A0A7FB;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  __I2C2_CLK_ENABLE();
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c2, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c2, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

} 

static void TIM2_Init(void){
  RCC->APB1ENR1 |= RCC_APB1ENR1_TIM2EN;
	TIM2->PSC = 0;
	TIM2->ARR = 48;	// Check freq
	TIM2->CR1 |= TIM_CR1_URS;
	TIM2->DIER |= TIM_DIER_UIE;
	TIM2->EGR |= TIM_EGR_UG;
	NVIC_EnableIRQ(TIM2_IRQn);	// Double check this is clearing flags
}

/* USER CODE BEGIN 4 */
void TIM2_IRQHandler(void){
	if(TIM2->SR && TIM_SR_UIF){	// Check rw,r,w priv
		TIM2cnt++;
		TIM2->SR |= TIM_SR_UIF;
	}
}

void delayms(int ms){
	// Check access priv
	TIM2cnt = 0;
	TIM2->CR1 |= TIM_CR1_CEN;
	while(TIM2cnt < ms){
		asm("nop");
	}
	TIM2->CR1 &= ~TIM_CR1_CEN;
}

void LCD_Init(void){
	/* For 4-bit Parallel:
	 * 1. Vss (Ground)
	 * 2. Vdd (5V)
	 * 3. RegVdd (5V)
	 * 4. D/C (PB0)
	 * 5. R/W (Ground)
	 * 6. E (PB3)
	 * 7-10. N/A (Ground)
	 * 11-14. Data (PB4-PB7)
	 * 15. CS (Ground)
	 * 16. /RES (5V)
	 * 17-19. BS (101)
	 * 20. Vss (Ground)
	 */
//  RES = 1; //reset HIGH – inactive
  HAL_Delay(1);
  LCD_DC(0x2A,1); //function set (extended command set)
  LCD_DC(0x71,1); //function selection A
  LCD_DC(0x00,0); // disable internal VDD regulator (2.8V I/O). data(0x5C) = enable regulator (5V I/O)
  LCD_DC(0x28,1); //function set (fundamental command set)
  LCD_DC(0x08,1); //display off, cursor off, blink off
  LCD_DC(0x2A,1); //function set (extended command set)
  LCD_DC(0x79,1); //OLED command set enabled
  LCD_DC(0xD5,1); //set display clock divide ratio/oscillator frequency
  LCD_DC(0x70,1); //set display clock divide ratio/oscillator frequency
  LCD_DC(0x78,1); //OLED command set disabled
  LCD_DC(0x09,1); //extended function set (4-lines)
  LCD_DC(0x06,1); //COM SEG direction
  LCD_DC(0x72,1); //function selection B
  LCD_DC(0x00,0); //ROM CGRAM selection
  LCD_DC(0x2A,1); //function set (extended command set)
  LCD_DC(0x79,1); //OLED command set enabled
  LCD_DC(0xDA,1); //set SEG pins hardware configuration
  LCD_DC(0x10,1); //set SEG pins hardware configuration
  LCD_DC(0xDC,1); //function selection C
  LCD_DC(0x00,1); //function selection C
  LCD_DC(0x81,1); //set contrast control
  LCD_DC(0x7F,1); //set contrast control
  LCD_DC(0xD9,1); //set phase length
  LCD_DC(0xF1,1); //set phase length
  LCD_DC(0xDB,1); //set VCOMH deselect level
  LCD_DC(0x40,1); //set VCOMH deselect level
  LCD_DC(0x78,1); //OLED command set disabled
  LCD_DC(0x28,1); //function set (fundamental command set)
  LCD_DC(0x01,1); //clear display
  LCD_DC(0x80,1); //set DDRAM address to 0x00
  LCD_DC(0x0C,1); //display ON
  HAL_Delay(100);
}

void LCD_DC(char c, char DC_Flag){
	uint32_t temp;
	if (DC_Flag == 0){
		GPIOB->ODR |= DC;	//Data
	}
	else {
		GPIOB->ODR &= ~DC;	//Command
	}
 	temp = GPIOB->ODR;
	temp &= ~GPIOB_DISPLAY_DATA_Msk;
	temp |= (c & 0xF0);
	GPIOB->ODR = temp;
	GPIOB->ODR |= E;
	HAL_Delay(1);
	GPIOB->ODR &= ~E;

	temp = GPIOB->ODR;
	temp &= ~GPIOB_DISPLAY_DATA_Msk;
	temp |= (c & 0x0F) << 4;
	GPIOB->ODR = temp;
	GPIOB->ODR |= E;
	HAL_Delay(1);
	GPIOB->ODR &= ~E;
}

void I2C_Init(void){
	uint32_t temp;
	// __I2C2_CLK_ENABLE();
	temp = I2C2->CR2;
	temp &= ~(I2C_CR2_SADD & I2C_CR1_PE);	// Write transfer ensured
	temp |= 0x3C << 1;	// Display Address
	I2C2->CR2 = temp;
}

void L2D_Init(void){
  uint32_t ret;
	uint32_t temp;
  uint8_t buf[12];
	I2C2->CR2 |= I2C_CR1_PE;
	// I2C2->CR2 |= I2C_CR2_START;
	// while((I2C2->ISR & I2C_ISR_TXIS) != 1);
  buf[0] = 0x2A;
  ret = HAL_I2C_Master_Transmit(&hi2c2, 0x3C << 1, (uint8_t *)"1234", 4, 10000);
  if (ret != HAL_OK)
    {
        asm("bkpt 255");
    }
}

void I2C_Write(char w, char DC_Flag){
	uint32_t temp;
	if (DC_Flag == 0){	// Data
		temp = I2C2->TXDR;
		temp &= ~I2C_TXDR_TXDATA;
		temp |= 0x40;
		I2C2->TXDR = temp;
	}
	else{	// Command
		temp = I2C2->TXDR;
		temp &= ~I2C_TXDR_TXDATA;
		temp |= 0x00;
		I2C2->TXDR = temp;
	}
	while((I2C2->ISR & I2C_ISR_TXIS) != 1);
	temp = I2C2->TXDR;
	temp &= ~I2C_TXDR_TXDATA;
	temp |= w;
	I2C2->TXDR = temp;
	while((I2C2->ISR & I2C_ISR_TXIS) != 1);
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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
