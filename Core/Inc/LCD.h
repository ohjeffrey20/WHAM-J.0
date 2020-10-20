#ifndef __LCD_H
#define __LCD_H

#define rows 4
#define cols 20
#define line[4] {0x80, 0xA0, 0xC0, 0xE0}
#define LCD_add 0x3c
char tx_packet[20] = {0x00};

void command(char c);
void data(char d);
void LCD_Init(void);

#endif