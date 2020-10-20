// void I2C_Write(char w, char DC_Flag){
// 	uint32_t temp;
// 	if (DC_Flag == 0){	// Data
// 		temp = I2C2->TXDR;
// 		temp &= ~I2C_TXDR_TXDATA;
// 		temp |= 0x40;
// 		I2C2->TXDR = temp;
// 	}
// 	else{	// Command
// 		temp = I2C2->TXDR;
// 		temp &= ~I2C_TXDR_TXDATA;
// 		temp |= 0x00;
// 		I2C2->TXDR = temp;
// 	}
// 	while((I2C2->ISR & I2C_ISR_TXIS) != 1);
// 	temp = I2C2->TXDR;
// 	temp &= ~I2C_TXDR_TXDATA;
// 	temp |= w;
// 	I2C2->TXDR = temp;
// 	while((I2C2->ISR & I2C_ISR_TXIS) != 1);
// }


// void LCD_Init(void){
//   HAL_Delay(1);
//   LCD_DC(0x2A,1); //function set (extended command set)
//   LCD_DC(0x71,1); //function selection A
//   LCD_DC(0x00,0); // disable internal VDD regulator (2.8V I/O). data(0x5C) = enable regulator (5V I/O)
//   LCD_DC(0x28,1); //function set (fundamental command set)
//   LCD_DC(0x08,1); //display off, cursor off, blink off
//   LCD_DC(0x2A,1); //function set (extended command set)
//   LCD_DC(0x79,1); //OLED command set enabled
//   LCD_DC(0xD5,1); //set display clock divide ratio/oscillator frequency
//   LCD_DC(0x70,1); //set display clock divide ratio/oscillator frequency
//   LCD_DC(0x78,1); //OLED command set disabled
//   LCD_DC(0x09,1); //extended function set (4-lines)
//   LCD_DC(0x06,1); //COM SEG direction
//   LCD_DC(0x72,1); //function selection B
//   LCD_DC(0x00,0); //ROM CGRAM selection
//   LCD_DC(0x2A,1); //function set (extended command set)
//   LCD_DC(0x79,1); //OLED command set enabled
//   LCD_DC(0xDA,1); //set SEG pins hardware configuration
//   LCD_DC(0x10,1); //set SEG pins hardware configuration
//   LCD_DC(0xDC,1); //function selection C
//   LCD_DC(0x00,1); //function selection C
//   LCD_DC(0x81,1); //set contrast control
//   LCD_DC(0x7F,1); //set contrast control
//   LCD_DC(0xD9,1); //set phase length
//   LCD_DC(0xF1,1); //set phase length
//   LCD_DC(0xDB,1); //set VCOMH deselect level
//   LCD_DC(0x40,1); //set VCOMH deselect level
//   LCD_DC(0x78,1); //OLED command set disabled
//   LCD_DC(0x28,1); //function set (fundamental command set)
//   LCD_DC(0x01,1); //clear display
//   LCD_DC(0x80,1); //set DDRAM address to 0x00
//   LCD_DC(0x0C,1); //display ON
//   HAL_Delay(100);
// }

// void LCD_DC(char c, char DC_Flag){
// 	uint32_t temp;
// 	if (DC_Flag == 0){
// 		GPIOB->ODR |= DC;	//Data
// 	}
// 	else {
// 		GPIOB->ODR &= ~DC;	//Command
// 	}
//  	temp = GPIOB->ODR;
// 	temp &= ~GPIOB_DISPLAY_DATA_Msk;
// 	temp |= (c & 0xF0);
// 	GPIOB->ODR = temp;
// 	GPIOB->ODR |= E;
// 	HAL_Delay(1);
// 	GPIOB->ODR &= ~E;

// 	temp = GPIOB->ODR;
// 	temp &= ~GPIOB_DISPLAY_DATA_Msk;
// 	temp |= (c & 0x0F) << 4;
// 	GPIOB->ODR = temp;
// 	GPIOB->ODR |= E;
// 	HAL_Delay(1);
// 	GPIOB->ODR &= ~E;
// }

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