#ifndef __LCD_H
#define __LCD_H

#define ROW_N = 4;                 // Number of display rows
#define COLUMN_N = 20;             // Number of display columns
#define line[4] = {0x80, 0xA0, 0xC0, 0xE0};
#define rows = 0x08;
#define add = 0x3c;
char tx_packet[] = {0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00};

void send_packet(char x);              // SUBROUTINE: SEND TO THE DISPLAY THE x BYTES STORED IN tx_packet

void command(char c);                  // SUBROUTINE: PREPARES THE TRANSMISSION OF A COMMAND

void data(char d);                     // SUBROUTINE: PREPARES THE TRANSMISSION OF A BYTE OF DATA

void setup(void);                    // INITIAL SETUP

#endif