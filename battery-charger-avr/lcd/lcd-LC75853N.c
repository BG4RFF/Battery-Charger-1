/**
@mainpage Atmel AVR LCD Library for LC75853N driver chip
@version 0.1
@author Ken Sarkies (www.jiggerjuice.net)
@date 4 February 2010
@brief Library of functions for control of an LCD module that uses the
LC75853N driver chip and adapted to the ATMega48/88/168 board.

The LCD panel is based on the LC75853N chip and is segment oriented as it has no
inbuilt character generator. Delays occurring in normal operation are short 
enough that they can also be built-in.

The LCD board uses similar pinouts as the Atmel ETT development board except that
ports PC0 and PC1 are replaced by PD2 and PD3. The actual pins can be specified
in the defines below.

The communication mode is bitwise.

Ports used:
PD4         Data In
PD5         Data Out
PD2         Enable
PD3         Data/Command

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
// Masks
#define CE 2
#define CK 3
#define DI 4
#define CLOCK    (1<<CK)
#define ENABLE   (1<<CE)
#define DATAMASK (1<<DI)
#define PORTMASK (DATAMASK | CLOCK)
// Needed for delay computations
#ifndef F_CPU
	#define F_CPU 8000000UL
#endif

#include <avr/io.h>
#include <util/delay.h>
#include "lcd-LC75853N.h"

void sendByte(const unsigned char d);

/*--------------------------------------------------------------------------*/
/** Initialise the LCD.

There appears to be no initialization needed as such, but a delay of 4ms is
inserted to ensure that the display has reached the reset state.
 */
void InitLCD(void)
{
    _delay_us(4000);		 // Wait 4ms after power on as required
}

/*--------------------------------------------------------------------------*/
/** Write a block of segment data and controls

The data block is 16 bytes with segment data from seg1 to seg126 in LSB order.
The control byte is:
S0, S1 are 00 for normal operation, others are sleep states.
K0, K1 control whether the segments 41 and 42 are used for Key Scan (to
increase the number of keys) or for segments (11).
P0, P1 control whether the segment outputs S1 to S4 are for the segments (00),
or for general purpose port outputs.
SC is the segment on/off control: 0 for on, 1 for off.
DR controls the LCD drive (1/2 or 1/3)

@param[in] pointer to 16 element array of unsigned character data
@param[in] unsigned character control as follows:
*/
void displayLCD(const unsigned char* d, const unsigned char c)
{
// Keep the unused pins in case they carry important signals
    uint8_t keepaside=(LCD_PORT & ~(PORTMASK | ENABLE));
    LCD_PORT = keepaside;           // clear CE and CLK
    sendByte(0x42);                 // This is the "CCB bus" address, sent with CE low.
    LCD_PORT = (keepaside | ENABLE);// set CE
// Send the first 5 bytes for the first packet
    for (uint8_t  i = 0; i < 5; i++)
    {
        sendByte(d[i]);
    }
// Strip out two bits from the next and start building a byte to send with two bits
// of the controls (the middle four bits are zero)
    sendByte((d[5] & 0x03) | (c << 6));
// Remainder of controls and two following 0's to complete the first packet
    sendByte(c >> 2);
// Now our data is offset by 2 bits, so we have to extract part of it and combine with the next
    uint8_t  temp = (d[5] >> 2);
    for (uint8_t  i = 6; i < 11; i++)
    {
        sendByte((d[i] << 6) | temp);
        temp = (d[i] >> 2);
    }
    sendByte(temp & 0x03);       // pull out the lower two bits and send zeros for the rest
// Send all zeros except for the 01 at the MSB indicating packet 1
    sendByte(0x80);
// Now our data is offset by 4 bits, so we have to extract part of it and combine with the next
    temp = (d[10] >> 4);
    for (uint8_t  i = 11; i < 16; i++)
    {
        sendByte((d[i] << 4) | temp);
        temp = (d[i] >> 4);
    }
    sendByte(temp & 0x03);       // pull out the last two bits and send zeros for the rest
// Send all zeros except for the 10 at the MSB indicating packet 2
    sendByte(0x40);
}

/*--------------------------------------------------------------------------*/
void sendByte(const unsigned char d)
{
// Time delays less than 160ns so rely on program activities to provide sufficient delay
    uint8_t keepaside=(LCD_PORT & ~PORTMASK);
    uint8_t value = d;
    for (uint8_t i=8; i>0; i--)
    {
        uint8_t portOut = (keepaside | ((value & 0x01) << DI)); // LSB first
        LCD_PORT = (portOut & ~CLOCK);      // Put out data with clock low
        _delay_us(1);
        LCD_PORT = (portOut | CLOCK);       // set clock high to latch data
        _delay_us(1);
        value >>= 1;                        // Shift for next bit
        LCD_PORT = (portOut & ~CLOCK);      // set clock low again
        _delay_us(1);
    }
}

