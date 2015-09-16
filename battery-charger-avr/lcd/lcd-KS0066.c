/**
@mainpage Atmel AVR LCD Library for KS0066 driver chip
@version 0.1
@author Ken Sarkies (www.jiggerjuice.net) adapted from code by Avinash Gupta
@date 1 November 2009
@brief Library of functions for control of an LCD 16x2 module that uses the
HD44780 driver chip and adapted to the ATMega48/88/168 board.

The 16x2 LCD panel is based on the KS0066 chip. The initialization process
would be called before any OS is started, so delays are built-in. Other delays
occurring in normal operation are short enough that they can also be built-in.

The library doesn't rely on monitoring the busy signal as some microcontroller
boards make use of a write-only interface.

The LCD board uses similar pinouts as the Atmel ETT development board except that
ports PC0 and PC1 are replaced by PD2 and PD3. The actual pins can be specified
in the defines below.

The communication mode is 4 bit with bytes transferred in two operations.

Ports used:
PD4-PD7       Data
PD2           Enable
PD3           Data/Command

@note Based on library by Avinash Gupta
@note Software: AVR-GCC 1:4.2.2-1
@note Target:   ATMega AVR
@note Tested:   ATMega88 at 8MHz.
*/
/***************************************************************************
 *   Copyright (C) 2009 by Ken Sarkies                                     *
 *   ksarkies@trinity.asn.au                                               *
 *                                                                         *
 *   This file is part of SolarRegulator                                   *
 *                                                                         *
 *   SolarRegulator is free software; you can redistribute it and/or modify*
 *   it under the terms of the GNU General Public License as published by  *
 *   the Free Software Foundation; either version 2 of the License, or     *
 *   (at your option) any later version.                                   *
 *                                                                         *
 *   SolarRegulator is distributed in the hope that it will be useful,     *
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of        *
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         *
 *   GNU General Public License for more details.                          *
 *                                                                         *
 *   You should have received a copy of the GNU General Public License     *
 *   along with SolarRegulator if not, write to the                        *
 *   Free Software Foundation, Inc.,                                       *
 *   51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA.             *
 ***************************************************************************/
#define TRUE  1
#define FALSE 0
#define command 0
#define data 1
// Sets the LCD port used (could generalise if control pins on a different port)
#define LCD_PORT PORTD
// Masks for strobe (Pin 3) and command/data (Pin 2)
#define STROBEPIN 3
#define CMDDATAPIN 2
#define DATAMASKSHIFT 4
#define STROBE   (1<<STROBEPIN)
#define CMD      (1<<CMDDATAPIN)
#define DATAMASK (0x0F<<DATAMASKSHIFT)
#define PORTMASK (DATAMASK | STROBE | CMD)
// Needed for delay computations
#ifndef F_CPU
	#define F_CPU 8000000UL
#endif
// Delay period normally 37us by spec.
#define delaytime 1000

#include <avr/io.h>
#include <util/delay.h>
#include "lcd-KS0066.h"

/*--------------------------------------------------------------------------*/
/** Initialise the LCD to 4-bit mode, 2-line and 5x8 bit font

	style = LS_BLINK, LS_SHOW (can be "OR"ed for combination)
	LS_BLINK = 1 The cursor is blinking type
	LS_SHOW  = 2 Cursor is shown

KS0066 datasheet requires 0x3x to be sent three times before startup sequence
with 40ms after power on needed. Apparently it should do this normally but may not
happen if the power conditions are not right. As this is a once off call, specified
delays are built in. Small delays are needed for data to settle because of the
demands of the interface and the speed of the microcontroller.
See p18 of datasheet for instruction set, p27 for the required startup sequence.
Commands:
	Clear Display	0x01
	Return Home	    0x02
	Entry Mode Set	0x04 +2=increment cursor, +1=shift display
	Display On/Off	0x08 +4=display on, +2=Cursor on +1=Blink Cursor
	Move Cursor	    0x10 +8=shift display, +4=right
	Function Set	0x20 +10=data length is 8-bit, +8=2 line, +4=5x10 font
	CGRAM address	0x40 + 6-bit address
	DDRAM address	0x80 + 7-bit address

This is called once in the initialization phase.
 */
void InitLCD(uint8_t style)
{
    _delay_us(40000);		// Wait 40ms after power on as required
// Keep the unused pins in case they carry important signals. Data/Command pin set low.
    uint8_t keepaside=(LCD_PORT & ~PORTMASK);
    uint8_t portvalue=(0x20 | keepaside);  // Command to set 4-bit mode
    uint8_t portvaluestrobe=(portvalue | STROBE);
    LCD_PORT=portvaluestrobe;	// Ensure enable strobe starts off high when data written
// send command 20 in upper nybble, 8-bit mode, with command register selected (0) and saved unused pins
    _delay_us(1);           // Ensure data has settled
    LCD_PORT=portvalue;		// Drop enable strobe (activate write)
    LCD_PORT=portvaluestrobe;   // Raise enable strobe for next command
// The remainder are now done in 4-bit mode
    LCDByte(command,0x28);  // Repeat function set command to set 2-line mode, low font
    _delay_us(40);          // Wait for 40us as required
    LCDByte(command,0x0C|style);// Display on and style set
    _delay_us(40);          // Wait for 40us as required
    LCDByte(command,0x01);  // Display clear
    _delay_us(1530);        // Wait for 1.5ms apparently needed for clear command to settle
    LCDByte(command,0x06);  // Set I/D=1 to increment, S=0 for no display shift
}

/*--------------------------------------------------------------------------*/
/** Send an ASCII message to the LCD

    msg = a null terminated string to be written
 */
void LCDWriteString(const char *msg)
{
	 while(*msg!='\0')
	 {
		LCDByte(data,*msg);
		msg++;
	 }
}

/*--------------------------------------------------------------------------*/
/** This function writes an integer type value to LCD module

    val  = Value to display
    length = total length of field in which the value is printed
    (must be between 1-5. If -1 the field length is determined
    from the value)
 */
void LCDWriteInt(int val,unsigned int length)
{

}

/*--------------------------------------------------------------------------*/
/** Move cursor to the x,y position

x = 0 to 15
y = 0 or 1
(note the HD44780 chip has 64 characters of memory for each line so exceeding
these parameters could result in a blank display).
The address is set with the lower 6 bits as x offset and bit 6 to select line
(see datasheet p29 and 31)
 */
void LCDGotoXY(uint8_t x,uint8_t y)
{
    LCDByte(command,0x80 | (x+(y<<6)));
}

/*--------------------------------------------------------------------------*/
/** Send a byte to the LCD

The upper nybble is sent first followed by the lower.
When new data is written the enable strobe must be set high to allow time for data to settle.
Following that, the strobe can be applied, and after a delay, raised again.

Note that the necessary delays are long enough that they must use a delay loop.
The second nybble can be written immediately after the first as no internal processing
occurs in the LCD chip.

    cmd  = 0 (command) or 1 (data) register select
    data = byte to be sent
 */
void LCDByte(uint8_t cmd,uint8_t databyte)
{
// Keep the unused pins in case they carry important signals, also add cmd bit back in
    uint8_t keepaside=((LCD_PORT & ~PORTMASK) | (cmd<<CMDDATAPIN));
// send upper nybble with command/data register selected and saved unused pins
    uint8_t portvalue=((databyte & DATAMASK) | keepaside);
    uint8_t portvaluestrobe=(portvalue | STROBE);
    LCD_PORT=portvaluestrobe;	// Ensure enable strobe is high when data written
    _delay_us(1);               // Ensure data has settled
    LCD_PORT=portvalue;     	// Drop enable strobe (activate write)
    LCD_PORT=portvaluestrobe;	// Raise enable strobe
// send lower nybble with command/data register selected and saved unused pins
    portvalue=(((databyte<<4) & DATAMASK) | keepaside);
    portvaluestrobe=(portvalue | STROBE);
    LCD_PORT=portvaluestrobe;   // Ensure enable strobe is high when data written
    _delay_us(1);               // Ensure data has settled
    LCD_PORT=portvalue;     	// Drop enable strobe (activate write)
    _delay_us(delaytime);       // Wait until command is processed
    if (cmd == 0)
        for (uint16_t i=0; i<60000; i++)
            _delay_us(delaytime);
    LCD_PORT=portvaluestrobe;   // Raise enable strobe
}
/*--------------------------------------------------------------------------*/

