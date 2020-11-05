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

#include "main.h"

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void){
  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();
  /* Configure the system clock */
  SystemClock_Config();
  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_I2C2_Init();
  MX_USART2_UART_Init();
  /* Initialize extras */
  TIM2_Init();
  LCD_Init();

  while(1){
    LCD_test();
    HAL_GPIO_TogglePin(GPIOB, LD2_Pin);
    delayms(500);
  }
}

/**
  * @brief TIM2 Interrupt Handler
  * @param None
  * @retval None
  */
void TIM2_IRQHandler(void){
	if(TIM2->SR && TIM_SR_UIF){	// Check rw,r,w priv
		TIM2cnt++;
		TIM2->SR &= ~TIM_SR_UIF;
	}
}

/**
  * @brief Delay in ms Function
  * @param ms Delay in ms
  * @retval None
  */
void delayms(int ms){
	TIM2cnt = 0;
	TIM2->CR1 |= TIM_CR1_CEN;
	while(TIM2cnt <= ms){
		asm("nop");
	}
	TIM2->CR1 &= ~TIM_CR1_CEN;
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void){
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
void assert_failed(uint8_t *file, uint32_t line){
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/