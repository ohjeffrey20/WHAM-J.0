#include "main.h"

/*
   * How I2C Master Transmit Works
   ** Master mode
   ** Reload mode
   ** Size of data
   ** Slave address
   ** Autoend
   ** Send start
   ** Check for I2C_ISR_TXIS
   ** Clear stop flag as autoend triggered
   */

/**
  * @brief  LCD Command Send
  * @param c Command to send
  * @retval None
  */
void command(uint8_t c){
   uint8_t ret;
   tx_packet[0] = Com_add;
   tx_packet[1] = c;
   ret = HAL_I2C_Master_Transmit(&hi2c2, LCD_add << 1, tx_packet, 2, 100);
   if (ret != HAL_OK){
      Error_Handler();
   }
}

/**
  * @brief  LCD Command Send
  * @param d Data to send
  * @param n Number of data to send
  * @retval None
  */
void data(uint8_t d){
   uint8_t ret;
   tx_packet[0] = Data_add;
   tx_packet[1] = d;
   ret = HAL_I2C_Master_Transmit(&hi2c2, LCD_add << 1, tx_packet, 2, 100);
   if (ret != HAL_OK){
      Error_Handler();
   }
}

/**
  * @brief  LCD Initialization Function
  * @param None
  * @retval None
  */
void LCD_Init(void){
   command(0x2A); //function set (extended command set)
   command(0x71); //function selection A
   data(0x00); // disable internal VDD regulator (2.8V I/O). data(0x5C) = enable regulator (5V I/O)
   command(0x28); //function set (fundamental command set)
   command(0x08); //display off, cursor off, blink off
   command(0x2A); //function set (extended command set)
   command(0x79); //OLED command set enabled
   command(0xD5); //set display clock divide ratio/oscillator frequency
   command(0x70); //set display clock divide ratio/oscillator frequency
   command(0x78); //OLED command set disabled
   command(0x09); //extended function set (4-lines)
   command(0x06); //COM SEG direction
   command(0x72); //function selection B
   data(0x00); //ROM CGRAM selection
   command(0x2A); //function set (extended command set)
   command(0x79); //OLED command set enabled
   command(0xDA); //set SEG pins hardware configuration
   command(0x10); //set SEG pins hardware configuration
   command(0xDC); //function selection C
   command(0x00); //function selection C
   command(0x81); //set contrast control
   command(0x7F); //set contrast control
   command(0xD9); //set phase length
   command(0xF1); //set phase length
   command(0xDB); //set VCOMH deselect level
   command(0x40); //set VCOMH deselect level
   command(0x78); //OLED command set disabled
   command(0x28); //function set (fundamental command set)
   command(0x01); //clear display
   command(0x80); //set DDRAM address to 0x00
   command(0x0C); //display ON
   command(0x2A); // Function set: extended command set (RE=1), lines #
}

/**
  * @brief LCD Testing Function
  * @param None
  * @retval None
  */
void LCD_test(void){
  char C = 'A';
  char c = 'a';
  int i = 1;
  command(0x01); //clear display
	command(0x02); //return home
  // while(i < 26){
  //   data(C);
  //   data(c);
  //   C++;
  //   c++;
  //   i++;
  // }
  data('J');
  data('e');
  data('f');
  data('f');
  data('r');
  data('e');
  data('y');
}