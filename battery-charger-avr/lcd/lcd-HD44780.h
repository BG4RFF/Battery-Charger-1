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
#ifndef LCD_H_
#define LCD_H_

/** Initialize the LCD to 4-bit mode, 2-line and 5x7 bit font

	style = LS_BLINK,LS_ULINE(can be "OR"ed for combination)
	LS_BLINK :The cursor is blinking type
	LS_ULINE :Cursor is "underline" type else "block" type
 */
void InitLCD(const uint8_t style);

/** Send an ASCII message to the LCD

    msg: a null terminated string to
 */
void LCDWriteString(const char *msg);

/** This function writes a integer type value to LCD module

	val  = Value to print
	length = total length of field in which the value is printed
	(must be between 1-5. If -1 the field length is determined
	from the value)
 */
void LCDWriteInt(const int value, const unsigned int length);

void LCDWriteHexWord(const unsigned int value, const unsigned int length);

void LCDClear(void);
/** Move cursor to the x,y position
 */
void LCDGotoXY(uint8_t x,uint8_t y);

/** Send a byte to the LCD

    cmd  = 1 (command) or 0 (data) register select
    data = byte to be sent
 */
void LCDByte(uint8_t cmd,uint8_t databyte);

#endif
