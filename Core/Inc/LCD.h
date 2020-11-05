#ifndef __LCD_H
#define __LCD_H

#define rows 4
#define cols 20
#define line[4] {0x80, 0xA0, 0xC0, 0xE0}
#define LCD_add 0x3C
#define Data_add 0x40
#define Com_add 0x00

volatile uint8_t tx_packet[20];

void command(uint8_t c);
void data(uint8_t d);
void LCD_Init(void);
void LCD_test(void);

#endif