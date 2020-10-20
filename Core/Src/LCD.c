#include "main.h"
#include "LCD.h"

// void send_packet(char x)              // SUBROUTINE: SEND TO THE DISPLAY THE x BYTES STORED IN tx_packet
// {
//     char ix = 0;                       // Bytes index
 
//    Wire.beginTransmission(SLAVE2W);   // Begin the transmission via I2C to the display with the given address
//    for(ix=0; ix<x; ix++)              // One byte at a time,
//    {
//       Wire.write(tx_packet[ix]);      //  queue bytes for transmission
//    }
//    Wire.endTransmission();            // Transmits the bytes that were queued
// }

void command(char c)                  // SUBROUTINE: PREPARES THE TRANSMISSION OF A COMMAND
{
   tx_packet[0] = 0x00;               // Control Byte; C0_bit=0, D/C_bit=0 -> following Data Byte contains command
   tx_packet[1] = c;                  // Data Byte: the command to be executed by the display
   send_packet(2);                    // Transmits the two bytes
}

void data(char d)                     // SUBROUTINE: PREPARES THE TRANSMISSION OF A BYTE OF DATA
{
   tx_packet[0] = 0x40;               // Control Byte; C0_bit=0, D/C_bit=1 -> following Data Byte contains data
   tx_packet[1] = d;                  // Data Byte: the character to be displayed
   send_packet(2);                    // Transmits the two bytes
}

void setup(void)                      // INITIAL SETUP
{
   HAL_Delay(200);            // Waits 200 us for stabilization purpose
   HAL_Delay(10);         // Waits 10 ms for stabilization purpose
   
//    if (ROW_N == 2 || ROW_N == 4)
//       rows = 0x08;                    // Display mode: 2/4 lines
//    else
//       rows = 0x00;                    // Display mode: 1/3 lines
   
   command(0x22 | rows); // Function set: extended command set (RE=1), lines #
   command(0x71);        // Function selection A:
   data(0x5C);           //  enable internal Vdd regulator at 5V I/O mode (def. value) (0x00 for disable, 2.8V I/O)
   command(0x20 | rows); // Function set: fundamental command set (RE=0) (exit from extended command set), lines #
   command(0x08);        // Display ON/OFF control: display off, cursor off, blink off (default values)
   command(0x22 | rows); // Function set: extended command set (RE=1), lines #
   command(0x79);        // OLED characterization: OLED command set enabled (SD=1)
   command(0xD5);        // Set display clock divide ratio/oscillator frequency:
   command(0x70);        //  divide ratio=1, frequency=7 (default values)
   command(0x78);        // OLED characterization: OLED command set disabled (SD=0) (exit from OLED command set)
   
//    if (ROW_N > 2)
//       command(0x09);  // Extended function set (RE=1): 5-dot font, B/W inverting disabled (def. val.), 3/4 lines
//    else
//       command(0x08);  // Extended function set (RE=1): 5-dot font, B/W inverting disabled (def. val.), 1/2 lines
    command(0x09);   
   command(0x06);        // Entry Mode set - COM/SEG direction: COM0->COM31, SEG99->SEG0 (BDC=1, BDS=0)
   command(0x72);        // Function selection B:
   data(0x0A);           //  ROM/CGRAM selection: ROM C, CGROM=250, CGRAM=6 (ROM=10, OPR=10)
   command(0x79);        // OLED characterization: OLED command set enabled (SD=1)
   command(0xDA);        // Set SEG pins hardware configuration:
   command(0x10);        //  alternative odd/even SEG pin, disable SEG left/right remap (default values)
   command(0xDC);        // Function selection C:
   command(0x00);        //  internal VSL, GPIO input disable
   command(0x81);        // Set contrast control:
   command(0x7F);        //  contrast=127 (default value)
   command(0xD9);        // Set phase length:
   command(0xF1);        //  phase2=15, phase1=1 (default: 0x78)
   command(0xDB);        // Set VCOMH deselect level:
   command(0x40);        //  VCOMH deselect level=1 x Vcc (default: 0x20=0,77 x Vcc)
   command(0x78);        // OLED characterization: OLED command set disabled (SD=0) (exit from OLED command set)
   command(0x20 | rows); // Function set: fundamental command set (RE=0) (exit from extended command set), lines #
   command(0x01);        // Clear display
   HAL_Delay(2);              // After a clear display, a minimum pause of 1-2 ms is required
   command(0x80);        // Set DDRAM address 0x00 in address counter (cursor home) (default value)
   command(0x0C);        // Display ON/OFF control: display ON, cursor off, blink off
   HAL_Delay(250);           // Waits 250 ms for stabilization purpose after display on
   
//    if (ROW_N == 2)
//       new_line[1] = 0xC0;             // DDRAM address for each line of the display (only for 2-line mode)
}
