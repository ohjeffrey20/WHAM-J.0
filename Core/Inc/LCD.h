#ifndef __LCD_H
#define __LCD_H

#define rows 4
#define cols 20
#define line[4] {0x80, 0xA0, 0xC0, 0xE0}
#define LCD_add 0x3c
#define Data_add 0x40
#define Com_add 0x00

char tx_packet[20] = {0x00};

void command(uint8_t c);
void data(uint8_t d);
void LCD_Init(void);

#endif