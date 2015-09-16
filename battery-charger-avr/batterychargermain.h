/*****************************************************************************/
/*
Title:    Solar Regulator Project Embedded Atmel AVR module
Author:   Ken Sarkies
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


#ifndef SOLAREGULATORMAIN_H
#define SOLAREGULATORMAIN_H

/*****************************************************************************/
/* System Configuration Information */

#define LOAD    0
#define LED     1
#define PANEL   2

/* These must be defined here to set baudrate otherwise we can't communicate
*/
#ifndef F_CPU               /* CPU speed in Hz */
#define F_CPU               8000000
#endif
#define UART_BAUD_RATE      38400

/* This sets a size for the input and output command buffers, limiting the
length of packets sent and received */
#define DATA_BUFFER_SIZE    16

/* ADC_CAPABLE defines the existence of 1 and 2 Analogue channels */
#define ADC_CAPABLE_1       1
#define ADC_CAPABLE_2       1

/* ADC capability - use interrupts but not sleep or freerun mode */
#define ADC_MODE            2
#define ADC_SPEED           200
#define TWI_SPEED           100

/* Most of the following are defaults */
/* 0.128ms clock for 8MHz clock and value 5 (gives scale 1024, see timer.c)
This gives a 32ms overflow interrupt.*/
#define RTC_SCALE           5

/* External EEPROM information */
#define LOCAL_EEPROM_DEVICE 0xA0
#define EEPROM_END          0xFFFF

/* Internal EEPROM start address for Information Block */
#define INF_BLOCK_START     0

/* Start of data buffer in external EEPROM following channel information.*/
#define BUFFER_START        0x10

/* Size of a data block written to the external EEPROM. Keep as a power of two. */
#define DATA_BLOCK_SIZE    16

/* Location of the bootloader. We don't use this. A watchdog reset is used to
force the MCU to reset and hence jump to the bootloader (if the fuses are
set correctly). */
#define BOOTLOADER          0x1C00

/*****************************************************************************/
/* States of the packet I/O state machines */
#define IDLE 0x0
#define SYNC 0x1
#define ADDRESS 0x2
#define NUMBER_BYTES 0x3
#define COMMAND 0x4
#define DATA_PHASE 0x5
#define CHECKSUM 0x6
#define EOM 0x7

/* Control Characters */
#define IDLE_CHAR 0xDD
#define SYNC_CHAR 0x67
#define EOM_CHAR 0x03

/* Commands */
#define NO_COMMAND 0x0
#define RESET 0x0
#define UNIT_INF_ACCESS 0x01
#define READ_MEMORY 0x02
#define WRITE_MEMORY 0x03
#define READ_EEPROM 0x04
#define WRITE_EEPROM 0x05
#define RESET_BUFFER_POINTERS 0x06
#define RESET_CLOCK 0x07
#define ERROR_COMMAND 0x10
#define SUCCESS_COMMAND 0x11
#define OFFSET_CALIBRATE 0x20
#define ADC_ONCE 0x21
#define ADC_MULTIPLE 0x22
#define SET_CHANNEL_CONTROL 0x23
#define ADC_ABORT 0x24
#define STATE_VARIABLES 0x25
#define BOOTLOADER_JUMP 0x40
#define READ_SRAM 0x42
#define WRITE_SRAM 0x43
#define SET_PORT_BIT 0x44
#define CLEAR_PORT_BIT 0x45
#define READ_PORT 0x46

/* Error Codes */
#define NO_ERROR 0x0
#define COMMAND_SYNTAX 0x1
#define UNKNOWN_COMMAND 0x2
#define PACKET_SEQUENCE_ERROR 0x3
#define READ_MEMORY_SYNTAX 0x10
#define WRITE_MEMORY_SYNTAX 0x11
#define READ_EEPROM_SYNTAX 0x12
#define WRITE_EEPROM_SYNTAX 0x13
#define READ_EEPROM_FAIL 0x14
#define WRITE_EEPROM_FAIL 0x15
#define ADC_SYNTAX 0x16
#define ADC_FAILURE 0x17
#define RESET_BUFFER_POINTERS_SYNTAX 0x18
#define RESET_CLOCK_SYNTAX 0x19
#define UNIT_INF_ACCESS_SYNTAX 0x1A
#define SET_CHANNEL_CONTROL_SYNTAX 0x1B
#define BOOT_SYNTAX 0x1C
#define ADC_RESTART_ATTEMPT 0x1D
#define ADC_PARAMETERS 0x1E
#define STATE_VARIABLE_SYNTAX 0x1F
#define READ_SRAM_SYNTAX 0x40
#define SET_PORT_SYNTAX 0x41
#define CLEAR_PORT_SYNTAX 0x42
#define READ_PORT_SYNTAX 0x43
#define NO_PORT 0x44

/* Baudrate settings */
#define BAUDRATE UART_BAUD_SELECT(UART_BAUD_RATE,F_CPU)

/*****************************************************************************/

/* Prototypes for tasks */

/* This task sets up the main tasks by initializing the EEPROM storage. When ready
it initiates the tasks to run regularly and check the battery voltage.*/
void initiateControlTask(void)  __attribute__ ((naked));
/* This task measures the voltages and currents from the A/D converters, and
scales them according to the calibrated values in the internal EEPROM block.*/
void measurementTask(void)      __attribute__ ((naked));
/* This task runs the main control of the Regulator. Sample data is stored to EEPROM
at intervals.*/
void controlTask(void)          __attribute__ ((naked));
/* This task tests for user input via a hardware push button and manages the display
of system variables and configuration of certain items.*/
void displayTask(void)          __attribute__ ((naked));
/* Communications Tasks */
void processError(void)         __attribute__ ((naked));
void processCommand(void)       __attribute__ ((naked));
void inputPacketReceive(void)   __attribute__ ((naked));
void outputPacketSend(void)     __attribute__ ((naked));

/* Prototypes for local functions */
int  rescale(const int voltage,const int multiplier,const int offset);
void display(const long value);
void setErrorCode(const unsigned int code);

/* Prototypes for local hardware functions */
void configurePorts(void);
void openPanel(void);
void shortPanel(void);
void connectLoad(void);
void isolateLoad(void);
void onLED(void);
void offLED(void);

/* General Functions */

inline int abs(int value)
{
  if (value < 0) return -value;
  else return value;
}

/*****************************************************************************/

#endif
