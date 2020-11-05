#ifndef __init_H
#define __init_H

#define USART2_TX_Pin GPIO_PIN_2
#define USART2_TX_GPIO_Port GPIOA
#define USART2_RX_Pin GPIO_PIN_3
#define USART2_RX_GPIO_Port GPIOA
#define LD2_Pin GPIO_PIN_8
#define LD2_GPIO_Port GPIOB

void SystemClock_Config(void);
void MX_USART2_UART_Init(void);
void MX_GPIO_Init(void);
void MX_I2C2_Init(void);
void TIM2_Init(void);

#endif