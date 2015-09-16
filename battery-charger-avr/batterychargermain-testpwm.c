/**
@mainpage Battery Charger Project: Embedded Atmel AVR module
@version 1.0.0
@author Ken Sarkies (www.jiggerjuice.net)
@date 31 March 2010
@brief Code for the embedded portion of the Battery Charger Project,
for an 8MHz Atmel MCU ATMega168 or better.

@details The Battery Charger is a spin-off of the Solar Regulator Project and
has many aspects in common with it and the Acquisition Project. The
distinguishing feature of all these projects is the use of a comprehensive
serial (or USB or wireless) link communication protocol to enable control,
configuration and stored data extraction from the embedded microcontroller.

The battery charger uses commonly specified algorithms for charging different
battery types under cyclic, float and trickle charging. Unlike the more
complex Solar Regulator project, the battery charger does not attempt to
estimate and track battery State of Charge (SoC) or State of Health (SoH).

For Lead-Acid batteries, the charger uses simultaneous voltage and current
limiting to rapidly charge the battery, then reverts to trickle charging
after a termination point is reached at which the current falls to a preset
low value. A number of parameters can be specified either through the
serial interface or via the local push button and LCD interface if it is
present.

@note
Compiler: avr-gcc (GCC) 3.4.5 AVR-Ada-0.4.1 (CDK4AVR 3.0 3.4.5-20060708)
@note
Target:   ATMega AVR with at least 4 A/D ports 10-bit, UART, TWI with I2C
EEPROM storage chip, 16K FLASH, 256 Bytes SRAM, 256 Bytes EEPROM, 2 spare
I/O pins, optional LCD panel, push button(s) and LED.
@note
Tested:   ATMega168 at 8MHz
@note
Uses:     UART library of Peter Fleury, NARTOS scheduling operating system
          library by Ken Sarkies, and TWI, ADC, RTC libraries based on code
          written by Chris Efstathiou.

@todo Code for pushbutton user configuration/display.
@todo Complete programming of the algorithm.
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

#include <inttypes.h>
#include <avr/sfr_defs.h>
#include <avr/eeprom.h>
#include <avr/io.h>
#include <avr/wdt.h>
#include <avr/interrupt.h>
#include "batterychargermain.h"
#include "lcd/lcd-HD44780.h"
#include "uart/uart.h"
#include "timer/timer.h"
#include "twi/i2c.h"
#include "nartos/nartos.h"
#if (ADC_CAPABLE_1 == 1) || (ADC_CAPABLE_2 == 1)
#include "adc/adc.h"
#endif

#define TRUE 1
#define FALSE 0
/* Convenience macros (we don't use them all) */
#define  _BV(bit) (1 << (bit))
#define  inb(sfr) _SFR_BYTE(sfr)
#define  inw(sfr) _SFR_WORD(sfr)
#define  outb(sfr, val) (_SFR_BYTE(sfr) = (val))
#define  outw(sfr, val) (_SFR_WORD(sfr) = (val))
#define  cbi(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit))
#define  sbi(sfr, bit) (_SFR_BYTE(sfr) |= _BV(bit))
#define  high(x) ((uint8_t) (x >> 8) & 0xFF)
#define  low(x) ((uint8_t) (x & 0xFF))

/*****************************************************************************/
/** Real Time Clock Global Variable

The 32 bit time can also be accessed as four bytes. Time scale is defined in
the information block.
*/
volatile union timeUnion
{
    volatile uint32_t timeValue;
    volatile uint8_t  timeByte[4];
} time;

/*****************************************************************************/
/** @name Timer Counters */
//@{
/** Counters available for timer functions. Timing is disabled if the value
is initially set to zero. Enable it by setting it to the timeout value
required.*/
    volatile uint16_t controlDelayCounter = 0;
    volatile uint16_t displayDelayCounter = 0;
    volatile uint16_t displayTaskCounter = 0;
    volatile uint16_t displayInterval = 0;
//@}

/*****************************************************************************/
/* Global Variables */
/** @name UART variables */
/*@{*/
    volatile uint16_t uartInput;   /**< Character and errorcode read from uart */
    volatile uint8_t lastError;    /**< Error code for transmission back */
    volatile uint8_t checkSum;     /**< Checksum on message contents */
/*@}*/
/** @name Input state machine variables */
/*@{*/
    volatile uint8_t packetInState;   /**< State variable for input machine */
    volatile uint8_t numberInBytes;   /**< Number of parameter bytes in packet */
    volatile uint8_t dataInByteCount; /**< Substate variable for data phase */
    volatile uint8_t inputCommand;    /**< Command given by PC */
    volatile uint8_t inputBuffer[DATA_BUFFER_SIZE]; /**< Incoming data buffer*/
/*@}*/
/** @name Output state machine variables */
/*@{*/
    volatile uint8_t outputBusy;      /**< Boolean to lock output */
    volatile uint8_t packetOutState;  /**< State variable for output machine */
    volatile uint8_t numberOutBytes;  /**< Number of parameter bytes in packet */
    volatile uint8_t dataOutByteCount;/**< Substate variable for data phase */
    volatile uint8_t outputCommand;   /**< Command sent to PC */
    volatile uint8_t outputBuffer[DATA_BUFFER_SIZE]; /**< Outgoing data Buffer */
/*@}*/
/** @name Task IDs */
/*@{*/
    volatile uint8_t inputPacketReceiveID;
    volatile uint8_t outputPacketSendID;
    volatile uint8_t processCommandID;
    volatile uint8_t processErrorID;
    volatile uint8_t initiateControlTaskID;
    volatile uint8_t controlTaskID;
    volatile uint8_t measurementTaskID;
    volatile uint8_t displayTaskID;
/*@}*/
/** @name Command specific variables */
/*@{*/
    volatile uint8_t  eepromCount;          /**< Databytes to read/write to EEPROM */
    volatile uint8_t  *address;             /**< Address for EEPROM access */
    volatile uint8_t  adcControlByte;       /**< Mask for channels to be converted */
    volatile uint8_t  adcChannel;           /**< Channel to be converted */
    volatile uint16_t bufferStartPointer=BUFFER_START;/**< Data buffer pointer*/
    volatile uint16_t bufferEndPointer=BUFFER_START;  /**< Data buffer pointer*/
    volatile uint16_t newBufferEndPointer;
    volatile uint8_t  shift;
    volatile uint16_t numberConversions;
/*@}*/
/** @name Task global variables that can be changed by user command */
/*@{*/
    volatile uint8_t  rtcScale;
    volatile uint8_t  localEEPROMDevice;
    volatile uint16_t eepromEnd;
/*@}*/
/** @name Control Task global variables that can be changed by user command */
/*@{*/
    volatile uint8_t  adcShift;             /**< Power of two of averaging count for A/D*/
    volatile uint8_t  storeInterval;        /**< Time between storage of state information*/
    volatile uint8_t  measureInterval;      /**< Delay inserted in control algorithm */
    volatile uint8_t  controlDisable;       /**< This stops the controls from working*/
    volatile uint16_t floatChargeVoltageThreshold;/**< Scaled voltage limit in float charge mode */
    volatile uint16_t cyclicChargeCurrentThreshold;/**< Scaled current limit in cyclic charge mode */
    volatile uint16_t cyclicChargeVoltageThreshold;/**< Scaled voltage limit in cyclic charge mode */
    volatile uint16_t cyclicChargeCurrentCutoff;/**< Scaled current at which cyclic changes to float mode */
    volatile uint8_t  filterShift;          /**< IIR Filter multiplier power of two */
    volatile uint8_t  floatOnly;            /**< Only perform a safe float charge */

/*@}*/
/****************************************************************************/
/** @brief Main Program */

int main(void)
{
/** This sets up the interfaces for EEPROM and RTC */
    wdt_disable();          	    /* Stop watchdog timer */
    setErrorCode(0);                /* Turn off error messages */
/** Initialise some global variables from the UIB if it exists, else use defaults */
    if (eeprom_read_byte((uint8_t *)0) == 0xFF)     /* If unprogrammed, use defaults */
    {
        rtcScale = RTC_SCALE;		/* Tick period for RTC */
        localEEPROMDevice = LOCAL_EEPROM_DEVICE;	/* Address of EEPROM on I2C bus */
        eepromEnd = EEPROM_END;	    /* Last address of EEPROM */
    }
    else
    {
        rtcScale = eeprom_read_byte((uint8_t *)15);
        localEEPROMDevice = eeprom_read_byte((uint8_t *)14);
        eepromEnd = eeprom_read_word((uint16_t *)12);
    }

/** Setup the I/O ports, ADC, TWI, timer and LCD. */
    configurePorts();
    openPanel();
    connectLoad();
    adcInit(ADC_MODE,ADC_SPEED);
    twiInit(TWI_SPEED);
    InitLCD(0x00);
    time.timeValue = 0;
    timer0Init(0,rtcScale);

/** Initialize the UART library, pass the baudrate and avr cpu clock
(uses the macro UART_BAUD_SELECT()). Set the baudrate to a predefined value. */
    uart_init(BAUDRATE);

/** Enable interrupts, since the UART library is interrupt controlled. */
    sei();

/** Set input and output state variables to their idle state. */
    lastError = NO_ERROR;
    packetInState = IDLE;       /* state for packet input state machine */
    packetOutState = IDLE;      /* state for packet transmission state machine */
    outputCommand = NO_COMMAND; /* command for transmission */
    outputBusy = FALSE;

/** Initialise the OS stuff, setup the control and receive tasks, and jump
to the OS. The control task must be initiated first. */
    initNartos();
    inputPacketReceiveID = taskStart((uint16_t)*inputPacketReceive,FALSE);
    initiateControlTaskID = taskStart((uint16_t)*initiateControlTask,FALSE);
    LCDClear();
    LCDWriteString("Solar Power");
    LCDGotoXY(0,1);			    /* Next Line */
    LCDWriteString("Regulator V0.0");
    nartos();
}

/****************************************************************************/
/** @defgroup CT Control Tasks

These functions provide the control and display functionality of the Battery
Charger application.
@{*/

volatile uint16_t channelValue[4];
volatile int      scaledChannelValue[4];

/****************************************************************************/
/** @brief Launch the Control task.

This task sets up the main control tasks by initializing the EEPROM storage.
When ready it initiates the tasks to run regularly and check the battery voltage.
This task then terminates. We do this in a task to allow for time delays
using relinquishing to the OS.
*/

void initiateControlTask(void)
{
/** Setup the external EEPROM for storing the samples.
The first 16 bytes are reserved for general control information which
includes the buffer pointers needed to recover data when power fails.
These pointers are held in locations 8 to 11.*/
    while (twiErrorStatus() == 0xFF) taskRelinquish();
    twiSetupTransaction(0);                 /* Write only */
    twiWriteByte(0);                        /* High EEPROM address */
    twiWriteByte(0);                        /* Low EEPROM address */
    bufferStartPointer = BUFFER_START;
    bufferEndPointer = bufferStartPointer;
    twiWriteByte(low(bufferStartPointer));  /* Start Pointer */
    twiWriteByte(high(bufferStartPointer));
    twiWriteByte(low(bufferEndPointer));    /* End Pointer */
    twiWriteByte(high(bufferEndPointer));
    if (twiLaunch(localEEPROMDevice))       /* device number */
    {
        while (twiErrorStatus() == 0xFF) taskRelinquish();
        uint8_t lastTwiError = twiErrorStatus();
        if (lastTwiError != 0) setErrorCode(WRITE_EEPROM_FAIL);
    }
    else
        setErrorCode(WRITE_EEPROM_FAIL);
/* Set some defaults that can be changed in the configuration channel */
    adcShift = 3;
    storeInterval = 180;
    controlDisable = TRUE;
/** Start the control task that runs forever in a loop.*/
    controlDelayCounter = 0;
    controlTaskID = taskStart((uint16_t)*controlTask,FALSE);
/** Start the display task runs at intervals restarted by the timer ISR.
Set the timer going once per second */
    displayInterval = 30;
    displayTaskCounter = displayInterval;
    displayTaskID = taskStart((uint16_t)*displayTask,FALSE);
    taskExit();
}

/****************************************************************************/
/** @brief Measurement task.

This task measures the voltages and currents from the A/D converters, and
scales them according to the calibrated values in the internal EEPROM block.
The results are stored in two global arrays: one for the unscaled and one for
the scaled values respectively. The order of entries follows the channel number
order which must be set correctly by the configuration program to match the
A/D converter input numbers.

Since we are checking the channel blocks for the Simple A/D converter information
blocks, we also update any other configuration parameters if we come across a
general configuration block. All other information blocks are ignored.

Channel 0 - Panel Voltage
Channel 1 - Battery Voltage
Channel 2 - Load Current
Channel 3 - Panel Current
*/
volatile uint8_t  numberChannels;
volatile uint8_t  channelStart;
volatile uint8_t  channel;
volatile uint8_t  channelNumber;
volatile uint32_t channelSum;
volatile uint16_t adcErrorCount;
volatile uint16_t sampleCount;

void measurementTask(void)
{
/** Check that all analogue channels are defined and get the necessary scale factors.
This needs to happen regularly in case the EEPROM data is changed, so we do
this every time.*/
    numberChannels = eeprom_read_byte((uint8_t *)16);
    channelStart = 2+eeprom_read_byte((uint8_t *)1);    /* First Channel location (length)*/
    for (channel = 0; channel < numberChannels; channel++)
    {
/** The channel Identifier (second channel IB location) has the channel type in the upper
four bits, and the channel number in the lower.*/
        uint8_t channelId = eeprom_read_byte((uint8_t *)(channelStart+1));
        uint8_t channelType = (channelId >> 4);
        channelNumber = (channelId & 0x0F);
/** If the channel type is a configure channel, update the configuration variables */
        if (channelId == 0x30)          /* The advanced configure channel*/
        {
            measureInterval = eeprom_read_byte((uint8_t *)(channelStart+2));
            adcShift        = eeprom_read_byte((uint8_t *)(channelStart+3));
            controlDisable  = eeprom_read_byte((uint8_t *)(channelStart+4));
            storeInterval   = eeprom_read_byte((uint8_t *)(channelStart+5));
        }
        if (channelId == 0x31)          /* The control algorithm configure channel*/
        {
            floatChargeVoltageThreshold  = eeprom_read_word((uint16_t *)(channelStart+4));
            cyclicChargeCurrentThreshold = eeprom_read_word((uint16_t *)(channelStart+6));
            cyclicChargeVoltageThreshold = eeprom_read_word((uint16_t *)(channelStart+8));
            cyclicChargeCurrentCutoff    = eeprom_read_word((uint16_t *)(channelStart+10));
            filterShift                  = eeprom_read_byte((uint8_t *)(channelStart+12));
            floatOnly                    = eeprom_read_byte((uint8_t *)(channelStart+13));
        }
/** Check that the channel type is Simple A/D */
        else if (channelType == 0x02)
        {
/** Compute the number of averaging iterations. Note that the channel sums
are maintained as 32 bit values so we can average quite aggressively
(but more averaging takes up more time). */
            channelSum = 0;                     /* cumulative sums */
            adcErrorCount = 0;                  /* Keep track of errored values */
            for (sampleCount = 0; sampleCount < (1 << adcShift); ++sampleCount)
            {
/** In the event of an error, we do not add in the measured value, but we
must be able to account for the missing elements. We do this by keeping a count
of errored values, and when we get a good value, fill in the previous missing
values with multiple copies of it. A better way would be to use the average value
as a substitute, but this complicates the arithmetic significantly.
Such errors are never signalled back.*/
                if(adcStart(_BV(channelNumber)))
                {
                    int adcValue = 0;
                    do
                    {
                        taskRelinquish();
                        adcValue = adcRead(channelNumber);    /* Check if finished and grab result */
                    }
                    while (adcValue < 0);
                    uint16_t channelLastValue = adcValue;
                    channelSum += channelLastValue;
                    if (adcErrorCount > 0)
                    {
                        for (uint8_t i = 0; i < adcErrorCount; ++i)
                            channelSum += channelLastValue;
                        adcErrorCount = 0;
                    }
                }
                else adcErrorCount++;
            }
/** The summed results need to be scaled back to 10 bits to match the A/D precision.*/
            channelValue[channelNumber] = (channelSum >> adcShift);
/** The scaled result according to calibration is also stored for display.*/
            int channelMulScale = eeprom_read_word((uint16_t *)(channelStart+6));
            int channelOffScale = eeprom_read_word((uint16_t *)(channelStart+8));
            scaledChannelValue[channelNumber] = rescale(channelValue[channelNumber],
                                                    channelMulScale,channelOffScale);
/* Find the next channel information block. */
        }
        channelStart = channelStart+eeprom_read_byte((uint8_t *)(channelStart+0))+1;
    }

/* Reschedule the control task and quit */
    taskSchedule(controlTaskID,FALSE);
    taskExit();
}

/****************************************************************************/
/** @brief Control task.

This task runs the main control of the Battery Charger and relinquishes when possible
to allow serial input and output to occur. Sample data is stored to EEPROM at intervals.
This task has absolute priority.

The task runs forever but at the end of the code it enters a timed wait state to allow
other tasks to run, otherwise the serial communications and user interface can be
disrupted.
*/

volatile uint8_t storeCount;
volatile long voltageSum;
volatile int averageBatteryVoltage;
volatile long currentSum;
volatile long averagePanelCurrent;

void controlTask(void)
{
    for(;;)
    {
/** Start the measurement task and wait until it reschedules us. It will do
this immediately it finishes.*/
        measurementTaskID = taskStart((uint16_t)*measurementTask,FALSE);
        taskWait();
/*---------------------------------------------------------------------------*/
/** Compute the running average voltages and currents using a simple IIR filter.
The filterShift is the power of two of the multiplying time constant parameter.
The larger this value, the longer the time constant of the filter.*/
        long currentBatteryVoltage = scaledChannelValue[1];
        voltageSum = voltageSum + currentBatteryVoltage - averageBatteryVoltage;
        averageBatteryVoltage = (voltageSum >> filterShift);
        long currentPanelCurrent = scaledChannelValue[3];
        currentSum = currentSum + currentPanelCurrent - averagePanelCurrent;
        averagePanelCurrent = (currentSum >> filterShift);
/*---------------------------------------------------------------------------*/
/* The core Control actions */
        if (! controlDisable)
        {
/*---------------------------------------------------------------------------*/
/** Float charging occurs if we have prescribed safe charging, or we are finished
with cyclic charging.*/
            if (floatOnly || (averagePanelCurrent < cyclicChargeCurrentCutoff))
            {
/** Float charging - if the measured instantaneous voltage is above the float
voltage threshold, or the averaged current is above the cyclic charge threshold,
short the panel. Otherwise open it to charge. The average voltage and
current will adapt to be below the threshold values.*/
                if ((currentBatteryVoltage > floatChargeVoltageThreshold) 
                    || (averagePanelCurrent > cyclicChargeCurrentThreshold))
                    shortPanel();
                else
                    openPanel();
            }
            else
/** Cyclic charging - if the measured instantaneous voltage is above the cyclic
voltage threshold, or the averaged current is above the cyclic charge threshold,
short the panel. Otherwise open it to charge. The average voltage and
current will adapt to be below the threshold values.*/
            {
                if ((currentBatteryVoltage > cyclicChargeVoltageThreshold) 
                    || (averagePanelCurrent > cyclicChargeCurrentThreshold))
                    shortPanel();
                else
                    openPanel();
            }
/*---------------------------------------------------------------------------*/
/** Store some state variable data in external EEPROM every storeInterval (>0) ticks. */
            if (storeCount < storeInterval) storeCount++;
            else
            {
                storeCount = 0;

/** Write the new pointers to their location in EEPROM. */
/* Compute a new end pointer that would hold after a data block is written */
                if (eepromEnd - bufferEndPointer > DATA_BLOCK_SIZE)
                    newBufferEndPointer = bufferEndPointer + DATA_BLOCK_SIZE;
                else
                    newBufferEndPointer = BUFFER_START + DATA_BLOCK_SIZE;
/* If full, move the buffer start pointer along one to allow the new value to
overwrite the oldest value */
                if (newBufferEndPointer == bufferStartPointer)
                {
                    if (eepromEnd - bufferStartPointer > DATA_BLOCK_SIZE)
                        bufferStartPointer += DATA_BLOCK_SIZE;
                    else
                        bufferStartPointer = BUFFER_START;
                }
/* Now write the new pointers so that data can be recovered from the EEPROM
in the event of a power loss.*/
                while (twiErrorStatus() == 0xFF) taskRelinquish();
                controlDelayCounter = 2;
                taskWait();                         /* Wait for timer ISR */
                twiSetupTransaction(0);             /* Write only */
                twiWriteByte(0);                    /* Address */
                twiWriteByte(0x08);                 /* of pointers */
                twiWriteByte(low(bufferStartPointer));
                twiWriteByte(high(bufferStartPointer));
                twiWriteByte(low(newBufferEndPointer));
                twiWriteByte(high(newBufferEndPointer));
                twiLaunch(localEEPROMDevice);

/* Write the conversion time and all relevant data to EEPROM */
                while (twiErrorStatus() == 0xFF) taskRelinquish();
                controlDelayCounter = 2;
                taskWait();                             /* Wait for timer ISR */
                twiSetupTransaction(0);                 /* Write only */
                twiWriteByte(high(bufferEndPointer));   /* Address */
                twiWriteByte(low(bufferEndPointer));
                for (uint8_t n = 0; n < 4; ++n)
                    twiWriteByte(time.timeByte[n]);
                twiWriteByte(low(averageBatteryVoltage));
                twiWriteByte(high(averageBatteryVoltage));
                twiWriteByte(low(averagePanelCurrent));
                twiWriteByte(high(averagePanelCurrent));
                twiLaunch(localEEPROMDevice);
                bufferEndPointer = newBufferEndPointer;
            }
        }

/*---------------------------------------------------------------------------*/
/* Wait a bit to pad out cycle */
        controlDelayCounter = measureInterval;
        taskWait();
    }
}

/****************************************************************************/
/** @brief Display task.

This task tests for user input via a hardware push button and manages the LCD
display of system variables and configuration of certain items.
*/

void displayTask(void)
{
/* Display on LCD Panel */
    LCDClear();
    LCDWriteString("Voltage = ");
    display(averageBatteryVoltage);
    LCDWriteString("V");
    LCDGotoXY(0,1);			    /* Next Line */
    LCDWriteString("Current = ");
    display(averagePanelCurrent);
    LCDWriteString("V");
    taskExit();
}
/**@}*/
/****************************************************************************/
/** @defgroup SCIT Serial Communication and Interpretation Tasks

These functions provide the serial communication interface including packet
level interpretation and buffering, and command processing and execution.

These functions are common to this and the Acquisition and Solar Regulator
projects. Some command actions may be different.
@{*/
/****************************************************************************/
/** @brief Error Processing task.

If an error has occurred, reset the input buffer and build an error message to
be output. Wait for any output processing to finish. Input processing continues
but not packet interpretation.

The program can place any number of explanatory bytes in the output buffer
from location 1 onwards, setting numberBytes to the number of additional bytes.
This is incremented again to cover the additional error code at location 0.

The task completes processing of the command and exits. It is activated by the
input task when a message has been received.
*/

void processError(void)
{
    while (outputBusy) taskRelinquish();
    outputCommand = ERROR_COMMAND;
    outputBuffer[0] = lastError;
    ++numberOutBytes;
    lastError = NO_ERROR;
    outputPacketSendID = taskStart((uint16_t)*outputPacketSend,FALSE);
    taskSchedule(inputPacketReceiveID,FALSE);
    taskExit();
}

/****************************************************************************/
/** @brief Input Packet Interpretation Processing task.

This is where the command interpretation is done, causing a number of tasks to
be initiated. The task is scheduled when the input data buffer has something in it.
The input buffer has the parameters and the data following. Use a state machine
approach as it gives simpler code.

Wait until all pending Input and Output processes are complete.

For each command, the number of expected parameters is checked in some way. */

void processCommand(void)
{
    while (outputBusy) taskRelinquish();
    numberOutBytes = 0;
    switch (inputCommand)
    {
/*---------------------------------------------------------------------------*/
/** - <em>Unit Information Block Access data.</em>
No parameters. This command returns the address of the start of the information
block, its length and the size of the largest block that can be transferred.
Following this is the unit ID read from the on-board switches.
*/
        case UNIT_INF_ACCESS:
        {
            if (numberInBytes != 0) lastError = UNIT_INF_ACCESS_SYNTAX;
            else
            {
                numberOutBytes = 7;
                outputBuffer[0] = low(INF_BLOCK_START);
                outputBuffer[1] = high(INF_BLOCK_START);
                outputBuffer[2] = eeprom_read_byte(INF_BLOCK_START);
                outputBuffer[3] = DATA_BUFFER_SIZE;
                outputBuffer[4] = 0;	/* This would be the Board ID */
                outputBuffer[5] = low(F_CPU/1000);
                outputBuffer[6] = high(F_CPU/1000);
                outputCommand = SUCCESS_COMMAND;
            }
        }
        break;
/*---------------------------------------------------------------------------*/
/** - <em> Read Internal EEPROM Memory.</em>
Parameters: two byte address to access data, and number of data bytes to be
read, accounting for limits. This command returns the contents of a block of
internal EEPROM memory.
*/
        case READ_MEMORY:
        {
            if (numberInBytes != 3) lastError = READ_MEMORY_SYNTAX;
            else
            {
                address = (uint8_t *) (inputBuffer[0] + (inputBuffer[1] << 8));
                eepromCount = inputBuffer[2];
                if (eepromCount > DATA_BUFFER_SIZE)
                    eepromCount = DATA_BUFFER_SIZE;
                while (numberOutBytes < eepromCount)
                {
                    if (eeprom_is_ready())
                    {
                    	outputBuffer[numberOutBytes] = eeprom_read_byte(address);
                        ++numberOutBytes;
                        ++address;
                    }
                    else taskRelinquish();
                }
                outputCommand = SUCCESS_COMMAND;
              }
        }
        break;
/*---------------------------------------------------------------------------*/
/** - <em> Write Internal EEPROM Memory.</em>
Parameters: two byte address to access data, followed by all data bytes to
be written. Expect at least 1 and no more than DATA_BUFFER_SIZE-3 data bytes to
follow. This command writes the contents of a block of internal EEPROM memory.
*/
        case WRITE_MEMORY:
        {
            if ((numberInBytes < 3) || (numberInBytes > DATA_BUFFER_SIZE))
                lastError = WRITE_MEMORY_SYNTAX;
            else
            {
                address = (uint8_t *) (inputBuffer[0] + (inputBuffer[1] << 8));
                eepromCount = 2;
                while (eepromCount < numberInBytes)
                {
                    if (eeprom_is_ready())
                    {
                        eeprom_write_byte(address,inputBuffer[eepromCount]);
                        ++eepromCount;
                        ++address;
                    }
                    else taskRelinquish();
                }
                outputCommand = SUCCESS_COMMAND; /* command to send back */
            }
        }
        break;
/*---------------------------------------------------------------------------*/
/** - <em> Read External EEPROM.</em>
Parameters: two byte address to access data followed by device address of
external EEPROM, and the number of data bytes to be read, accounting for
limits. This command returns the contents of a block of external EEPROM memory.
The variable eepromCount contains the number of bytes to be read from
the EEPROM. The twiSetupTransaction, twiWriteByte functions are nonblocking. */
        case READ_EEPROM:
        {
            if (numberInBytes != 4) lastError = READ_EEPROM_SYNTAX;
            else
            {                                   /* wait till not busy */
                while (twiErrorStatus() == 0xFF) taskRelinquish();
                eepromCount = inputBuffer[3];     /* number of bytes */
                twiSetupTransaction(eepromCount); /* Write/Read */
                twiWriteByte(inputBuffer[2]);     /* High address */
                twiWriteByte(inputBuffer[1]);     /* Low address */
                if (eepromCount > DATA_BUFFER_SIZE)
                    eepromCount = DATA_BUFFER_SIZE;
/* Try to launch the read. If there is a problem, drop out */
                if(twiLaunch(inputBuffer[0]))    /* device ID */
                {
/* The twiCompletionStatus shows if the transaction is busy and also how many
bytes were read if an aborted transaction occurred. First wait until not busy.*/
                    while (twiCompletionStatus() == 0xFF) taskRelinquish();
                    uint8_t completionStatus = twiCompletionStatus();
                    {
                        uint8_t numberRead = eepromCount;
                        if (completionStatus != 0) numberRead = (completionStatus & 0x3F);
                        for (;numberRead > 0; --numberRead)
                        {
                            outputBuffer[numberOutBytes] = twiReadByte();
                            ++numberOutBytes;
                        }
                        if ((twiErrorStatus() == 0) && (completionStatus == 0))
                            outputCommand = SUCCESS_COMMAND; /* command to send back */
                        else
                            lastError = READ_EEPROM_FAIL;
                    }
                }
                else
                    lastError = READ_EEPROM_FAIL;
                if (lastError == READ_EEPROM_FAIL)
                {
                    outputBuffer[1] = twiErrorStatus();
                    outputBuffer[2] = twiCompletionStatus();
                    numberOutBytes = 2;
                }
            }
        }
        break;
/*---------------------------------------------------------------------------*/
/** - <em> Write External EEPROM command.</em>
Parameters: address to access data, followed by device ID and number of bytes
and all data bytes to be written. This command writes the contents of a block
of external EEPROM memory. Expect at least 1 and no more than
DATA_BUFFER_SIZE-4 data bytes to follow. */
        case WRITE_EEPROM:
        {
          if ((numberInBytes < 4) || (numberInBytes > DATA_BUFFER_SIZE))
            lastError = WRITE_EEPROM_SYNTAX;
          else
          {
            while (twiErrorStatus() == 0xFF) taskRelinquish();
            eepromCount = 3;                /* count of data bytes */
            twiSetupTransaction(0);         /* Write only */
            twiWriteByte(inputBuffer[2]);   /* High address */
            twiWriteByte(inputBuffer[1]);   /* Low address */
            while(eepromCount < numberInBytes)
            {
              if (! twiWriteByte(inputBuffer[eepromCount])) break;
              ++eepromCount;
            }
/* Try to launch the write. If there is a problem, drop out */
            if (twiLaunch(inputBuffer[0]))  /* device number */
            {
              while (twiErrorStatus() == 0xFF) taskRelinquish();
              uint8_t lastTwiError = twiErrorStatus();
              if (lastTwiError == 0)
                outputCommand = SUCCESS_COMMAND; /* command to send back */
              else
                lastError = WRITE_EEPROM_FAIL;
            }
            else
              lastError = WRITE_EEPROM_FAIL;
            if (lastError == WRITE_EEPROM_FAIL)
            {
              outputBuffer[1] = twiErrorStatus();
              outputBuffer[2] = twiCompletionStatus();
              numberOutBytes = 2;
            }
          }
        }
        break;
/*---------------------------------------------------------------------------*/
/** - <em> Initiate a single A/D conversion.</em>
One single A/D conversion is done.
Parameters: channel number, Gain (not used).
Result:  Timer 4 bytes, A/D result 2 bytes.
*/
        case ADC_ONCE:
        {
            numberOutBytes = 1;
            if (numberInBytes != 2) lastError = ADC_SYNTAX;
            else
            {
                adcChannel = inputBuffer[0];
                uint8_t adcGain = inputBuffer[1];/* For this application, this is a dummy */
                uint8_t adcStatus = adcStart(1 << adcChannel); /* Start Conversion */
                if (! adcStatus)		/* Check if the start was successful*/
                {
                    lastError = ADC_FAILURE;
                    outputBuffer[1] = adcStatus;
                }
                else
                {
                    while (adcIsBusy()) taskRelinquish(); /* Wait for conversion to end */
                    outputBuffer[0] = timer0Read(); /* Timer current value */
                    for (uint8_t n=0;n<3;++n)
                    {
                        outputBuffer[numberOutBytes++] = time.timeByte[n];
                    }
                    int adcValue = adcRead(adcChannel);
                    outputBuffer[numberOutBytes++] = low(adcValue);
                    outputBuffer[numberOutBytes++] = high(adcValue);
                    outputCommand = SUCCESS_COMMAND; /* command to send back */
                }
            }
        }
        break;
/*---------------------------------------------------------------------------*/
/** - <em> Return Data Block of Current State.</em>

Parameters: none.
Result:  Length in bytes of data block
         A block of internal state variables selected according to application.
         In this case, four averaged A/D measurements, 2 bytes each, unscaled.
*/
        case STATE_VARIABLES:
        {
            if ((numberInBytes != 1) || (inputBuffer[0] != 0)) lastError = STATE_VARIABLE_SYNTAX;
            else
            {
                for (uint8_t channel = 0; channel < 4; channel++)
                {
                    outputBuffer[numberOutBytes++] = low(channelValue[channel]);
                    outputBuffer[numberOutBytes++] = high(channelValue[channel]);
                }
                outputBuffer[numberOutBytes++] = low(averageBatteryVoltage);
                outputBuffer[numberOutBytes++] = high(averageBatteryVoltage);
                outputBuffer[numberOutBytes++] = low(averagePanelCurrent);
                outputBuffer[numberOutBytes++] = high(averagePanelCurrent);
                outputBuffer[numberOutBytes++] = low(scaledChannelValue[3]);
                outputBuffer[numberOutBytes++] = high(scaledChannelValue[3]);
                outputCommand = SUCCESS_COMMAND;
            }
        }
        break;
/*---------------------------------------------------------------------------*/
/** - <em> Reset the circular buffer pointers.</em>

This command returns the buffer start and end pointers, then sets the start pointer
to the end pointer. This effectively sets the buffer to empty. The calling program
is then expected to immediately read in all data between the old start and end
pointers to ensure that nothing is overwritten.
Parameters: none.
Result:  none.
*/
        case RESET_BUFFER_POINTERS:
        {
            if (numberInBytes != 0) lastError = RESET_BUFFER_POINTERS_SYNTAX;
            else
            {
                numberOutBytes = 4;
                outputBuffer[0] = low(bufferStartPointer);
                outputBuffer[1] = high(bufferStartPointer);
                outputBuffer[2] = low(bufferEndPointer);
                outputBuffer[3] = high(bufferEndPointer);
                bufferStartPointer = bufferEndPointer;
                outputCommand = SUCCESS_COMMAND; /* command to send back */
            }
        }
        break;
/*---------------------------------------------------------------------------*/
/** - <em> Reset the Real Time Clock.</em>
No parameters.
*/
        case RESET_CLOCK:
        {
            if (numberInBytes != 0) lastError = RESET_CLOCK_SYNTAX;
            else
            {
                time.timeValue = 0;
                timer0Init(0,rtcScale);
                outputCommand = SUCCESS_COMMAND; /* command to send back */
            }
        }
        break;
/*---------------------------------------------------------------------------*/
/** - <em> Jump to the bootloader.</em>
This command causes the program to transfer control to the bootloader. Its
purpose is to enable firmware uploads at any time. For the AVR MCU this is done
by setting the MCU watchdog timer for the shortest period, and continuing its
business. A watchdog timer reset will occur pushing it into the bootloader
where the wdt will be disabled. This is unsatisfactory but necessary as the
compiler could not handle a long jump.
*/
        case BOOTLOADER_JUMP:
        {
          if (numberInBytes != 0) lastError = BOOT_SYNTAX;
          else
          {
            MCUSR = 0;
            WDTCSR = (1 << WDCE) | (1 << WDE);
            WDTCSR = (1 << WDE);
            outputCommand = SUCCESS_COMMAND; /* command to send back */
          }
        }
        break;
#ifdef CORE_DEBUG
/*---------------------------------------------------------------------------*/
/** - <em> Debug Read SRAM.</em>
Parameters: two byte address to access data, and number of data bytes to be
read, accounting for limits. This command returns the contents of a block of
internal SRAM memory.
*/
        case READ_SRAM:
        {
          if (numberInBytes != 3) lastError = READ_SRAM_SYNTAX;
          else
          {
            address =(uint8_t *) (inputBuffer[0] + (inputBuffer[1] << 8));
            uint8_t count = inputBuffer[2];
            if (count > DATA_BUFFER_SIZE)
              count = DATA_BUFFER_SIZE;
            while (numberOutBytes < count)
            {
              outputBuffer[numberOutBytes] = *address;
              ++numberOutBytes;
              ++address;
            }
            outputCommand = SUCCESS_COMMAND; /* command to send back */
          }
        }
        break;
/* End of CORE_DEBUG section */
#endif
#ifdef DEBUG
/*---------------------------------------------------------------------------*/
/** - <em> Debug Set Port Bit.</em>
Parameters: port specification. This has the port address in the lower 4 bits,
and the bit number in the upper 4 bits. The port address is 0x0a to 0x0f, and
the bit specification has the value 0 to 7. This command sets a bit in the
specified port. It is dangerous as it could cause loss of communication to the
PC.
*/
        case SET_PORT_BIT:
        {
          if (numberInBytes != 1) lastError = READ_PORT_SYNTAX;
          else
          {
            uint8_t portBit = inputBuffer[0];
            uint8_t port = portBit & 0x0F;
            uint8_t bit = portBit >> 4;
            if (bit > 7) port = portBit;  /* fudge - drop through to default */
            switch (port)
            {
#if defined PORTA
            case 0x0a: sbi(PORTA,bit); break;
#endif
#if defined PORTB
            case 0x0b: sbi(PORTB,bit); break;
#endif
#if defined PORTC
            case 0x0c: sbi(PORTC,bit); break;
#endif
#if defined PORTD
            case 0x0d: sbi(PORTD,bit); break;
#endif
#if defined PORTE
            case 0x0e: sbi(PORTE,bit); break;
#endif
#if defined PORTF
            case 0x0f: sbi(PORTF,bit); break;
#endif
            default:
              lastError = NO_PORT;
              numberOutBytes = 1;
              outputBuffer[1] = portBit;
              break;
            }
            if (lastError == NO_ERROR)
              outputCommand = SUCCESS_COMMAND; /* command to send back */
          }
        }
        break;
/*---------------------------------------------------------------------------*/
/** - <em> Debug Clear Port Bit.</em>
Parameters: port specification. This has the port address in the lower 4 bits,
and the bit number in the upper 4 bits. The port address is 0x0a to 0x0f, and
the bit specification has the value 0 to 7. This command clears a bit in the
specified port. It is dangerous as it could cause loss of communication to the
PC.
*/
    case CLEAR_PORT_BIT:
    {
      if (numberInBytes != 1) lastError = READ_PORT_SYNTAX;
      else
      {
        uint8_t portBit = inputBuffer[0];
        uint8_t port = portBit & 0x0F;
        uint8_t bit = portBit >> 4;
        if (bit > 7) port = portBit;  /* fudge - drop through to default */
        switch (port)
        {
#if defined PORTA
        case 0x0a: cbi(PORTA,bit); break;
#endif
#if defined PORTB
        case 0x0b: cbi(PORTB,bit); break;
#endif
#if defined PORTC
        case 0x0c: cbi(PORTC,bit); break;
#endif
#if defined PORTD
        case 0x0d: cbi(PORTD,bit); break;
#endif
#if defined PORTE
        case 0x0e: cbi(PORTE,bit); break;
#endif
#if defined PORTF
        case 0x0f: cbi(PORTF,bit); break;
#endif
        default:
          lastError = NO_PORT;
          numberOutBytes = 1;
          outputBuffer[1] = portBit;
          break;
        }
        if (lastError == NO_ERROR)
          outputCommand = SUCCESS_COMMAND; /* command to send back */
      }
    }
    break;
/*---------------------------------------------------------------------------*/
/** - <em> Debug Read Port Pins.</em>
Parameters: port specification. This has the port address in the lower 4 bits,
having values 0x0a to 0x0f. This command reads the specified port.
*/
    case READ_PORT:
    {
      if (numberInBytes != 1) lastError = READ_PORT_SYNTAX;
      else
      {
        uint8_t port = inputBuffer[0];
        uint8_t portContents = 0;
        numberOutBytes = 1;
        switch (port)
        {
#if defined PINA
        case 0x0a: portContents = inb(PINA); break;
#endif
#if defined PINB
        case 0x0b: portContents = inb(PINB); break;
#endif
#if defined PINC
        case 0x0c: portContents = inb(PINC); break;
#endif
#if defined PIND
        case 0x0d: portContents = inb(PIND); break;
#endif
#if defined PINE
        case 0x0e: portContents = inb(PINE); break;
#endif
#if defined PINF
        case 0x0f: portContents = inb(PINF); break;
#endif
        default:
          lastError = NO_PORT;
          outputBuffer[1] = port;
          break;
        }
        if (lastError == NO_ERROR)
        {
          outputBuffer[0] = portContents;
          outputCommand = SUCCESS_COMMAND; /* command to send back */
        }
      }
    }
    break;
/*---------------------------------------------------------------------------*/
/* End of DEBUG section */
#endif
/** - <em> Command not recognized.</em>
Returns an error code.
*/
    default:
      lastError = UNKNOWN_COMMAND;
    break;
  }
/** If an error occurred, launch the error processing task, else launch the
output transmission task. Follow by rescheduling the suspended receive task. */
  if (lastError == NO_ERROR)
    outputPacketSendID = taskStart((uint16_t)*outputPacketSend,FALSE);
  else
    outputPacketSendID = taskStart((uint16_t)*processError,FALSE);
  taskSchedule(inputPacketReceiveID,FALSE);
  taskExit();
}

/****************************************************************************/
/** @brief PC Communication Input task.

This task waits for an incoming character on the UART, and packages it up
into a message.

Low level input interface errors are ignored and such characters are lost.
The input state machine works through the packet stages as characters arrive.
Parameters are read in and the syntax of the packet is checked. The packet data
is stored in a buffer for later interpretation. Input is blocked when the
packet has been fully read in. The buffer holds one single packet in full. This
allows us to discard the packet when an error occurs. A circular buffer would
make such error recovery very difficult.

If an error occurs, the packetInState must be returned to IDLE but the input
task must not be blocked.

The task never exits, but acts as a monitoring task that activates other tasks
as appropriate. This task is not entered if the error or command processing
tasks are active. If there are no incoming characters waiting, the task
relinquishes.
*/

void inputPacketReceive(void)
{
  for(;;)
  {
    uartInput = uart_getc();  /* upper byte has an error code, if any */
    if (high(uartInput) != 0) /* nonzero if error or no character received */
      taskRelinquish();
    else
    {
      uint8_t newCharacter = (uint8_t) uartInput & 0xFF;  /* lower byte */
      switch (packetInState)
      {
/** - The idle state occurs on startup, after a command has been received
fully, or when an error has occurred. When an IDLE character is received, it is
echoed back if the current output is not processing anything. The sync state is
terminated. The next state, SYNC, is entered. In any other case, usually
flushing after an error, the character is ignored.*/
      case IDLE:                  /* loop until an idle char is received */
        if (newCharacter == IDLE_CHAR)
        {                         /* get here with a valid IDLE received */
          if (! outputBusy) uart_putc(IDLE_CHAR);  /* ping PC */
          packetInState = SYNC;   /* look for a sync character */
        }
        break;
/** - If a SYNC character is received, go to the COMMAND state.
We can accept IDLE characters and ignore them, otherwise return to the IDLE
state and flush all following characters (no error is generated)*/
      case SYNC:
        if (newCharacter == SYNC_CHAR) packetInState = ADDRESS;
        else if (newCharacter != IDLE_CHAR) packetInState =IDLE;
        else if (! outputBusy) uart_putc(IDLE_CHAR);  /* ping PC */
        break;
/** - The address+packet type tells us if the packet is addressed to us.
Just accept any address for the moment until we get a means to find out who we
are. The packet type is also ignored until we define more types.*/
      case ADDRESS:
        packetInState = NUMBER_BYTES;
        break;
/** - If the byte count is zero (no parameters), terminate the packet,
otherwise go on to the data phase. */
      case NUMBER_BYTES:
        numberInBytes = newCharacter;
        dataInByteCount = 0;      /* counter for data phase */
        if (numberInBytes == 0) packetInState = CHECKSUM;
        else packetInState = COMMAND;
        break;
/** - Put the command aside and go to the data phase if more characters follow
*/
      case COMMAND:
        inputCommand = newCharacter;
        if (--numberInBytes == 0) packetInState = CHECKSUM;
        else packetInState = DATA_PHASE;
        break;
/** - In the DATA phase just dump everything to the buffer if it will fit.
dataInByteCount points to the next location to add a byte.*/
      case DATA_PHASE:
        if (dataInByteCount < DATA_BUFFER_SIZE)
        {
          inputBuffer[dataInByteCount] = newCharacter;
        }
        if (++dataInByteCount == numberInBytes) packetInState = CHECKSUM;
        break;
/** @todo We ignore the checksum for the moment */
      case CHECKSUM:
        packetInState = EOM;
        break;
/** - EOM terminates the packet, and we should have an EOM character here. We
can now start up the packet interpretation task, blocking any further input
processing. If an error has occurred,the error task will unblock this.*/
      case EOM:
        packetInState = IDLE;
        if (newCharacter != EOM_CHAR)
        {
          lastError = COMMAND_SYNTAX;
          outputBuffer[1] = newCharacter;
          numberOutBytes = 1;
          processErrorID = taskStart((uint16_t)*processError,FALSE);
        }
        else
          processCommandID = taskStart((uint16_t)*processCommand,FALSE);
        taskWait();
        break;
      }
    }
  }
}

/****************************************************************************/
/** @brief PC Communication Output task.

This task transmits any characters found to be in the output buffer.
Normally it is not activated until an entire message is ready to go.

Check for an active output command. If none, but an error is signalled, send
error message. Each command requires a response so a packet is generated.
Use a state machine to work through the stages when output buffer is free.

The task exits after the message has been sent.*/

void outputPacketSend(void)
{
  outputBusy = TRUE;
  for(;;)         // Loop "indefinitely" but will exit when finished
  {
    if (! uart_output_free()) taskRelinquish();
    else
    {
      switch (packetOutState)
      {
      case IDLE:
        uart_putc(IDLE_CHAR);
        packetOutState = SYNC;
        break;
      case SYNC:
        uart_putc(SYNC_CHAR);
        packetOutState = ADDRESS;
        checkSum = 0;
        break;
      case ADDRESS:
        uart_putc(0x0);             /* Address is PC, packet type 0 */
        packetOutState = NUMBER_BYTES;
        break;
      case NUMBER_BYTES:
        uart_putc(numberOutBytes+1);
        checkSum += numberOutBytes+1;
        packetOutState = COMMAND;
        break;
      case COMMAND:
        uart_putc(outputCommand);
        checkSum += outputCommand;
        dataOutByteCount = 0;
        packetOutState = DATA_PHASE;
        break;
      case DATA_PHASE:
        if (numberOutBytes-- > 0)
        {
          uart_putc(outputBuffer[dataOutByteCount]);
          checkSum += outputBuffer[dataOutByteCount];
          ++dataOutByteCount;
        }
        else packetOutState = CHECKSUM;
        break;
      case CHECKSUM:
        uart_putc(checkSum);
        packetOutState = EOM;
        break;
      case EOM:
        uart_putc(EOM_CHAR);
        packetOutState = IDLE;
        outputCommand = NO_COMMAND;
        numberOutBytes = 0;
        outputBusy = FALSE;
        taskExit();
        break;
      }
    }
  }
}
/**@}*/
/****************************************************************************/
/** @brief Timer 0 ISR.

This ISR updates the RTC time, and reschedules any tasks that have been
suspended pending a timeout interval. A task may request a timed suspension by
setting a specifically allocated time variable to a number of timer ticks. The
ISR will reschedule the task after the variable counts down to zero.

The ISR also manages time variables that can be used to start tasks at regular
intervals. The variables are decremented and the taska are started when they
reach zero. If the operation is repetitive the time variable is reset. Each
task must complete its work and terminate before the next start time is
reached.
*/

ISR(SIG_OVERFLOW0)
{
    time.timeValue++;
/* This counter is used to restart a task at precisely timed intervals.
Set the counter just before starting the task for the first time, then
issue a taskExit at the end of the task. The task should complete well
before the time interval is finished */
    if ((displayTaskCounter > 0) && (--displayTaskCounter == 0))
    {
        displayTaskCounter = displayInterval;   /* Reset counter for next cycle*/
        displayTaskID = taskStart((uint16_t)*displayTask,TRUE);
    }
/* This counter is used within a task to enter a timed wait state.
Set the counter just before the taskWait command. */
    if ((controlDelayCounter > 0) && (--controlDelayCounter == 0))
    {
        taskSchedule(controlTaskID,TRUE);
    }
}

/****************************************************************************/
/** @brief Scale a measured A/D quantity

The version of gcc used doesn't correctly multiply two 16 bit numbers to give a
32 bit result, but truncates it. This performs the necessary scaling by working
out the 16 bit products of the 8 two 8 bit parts, then combining the necessary
shift of 10 bits into the shifts needed to renormalize the products. The result
is a 16 bit scaled result with the offset added. This takes into account sign.
*/
int rescale(const int quantity,const int multiplier,const int offset)
{
    uint16_t q = quantity;
    if (quantity < 0) q = -quantity;
    uint16_t m = multiplier;
    if (multiplier < 0) m = -multiplier;
    uint8_t lowVoltage = low(q);
    uint8_t highVoltage = high(q);
    uint8_t lowMultiplier = low(m);
    uint8_t highMultiplier = high(m);
    uint16_t first = lowVoltage*lowMultiplier;
    uint16_t second = lowVoltage*highMultiplier+ highVoltage*lowMultiplier;
    uint16_t third = highVoltage*highMultiplier;
    int product = ((high(first) + second) >> 2) + (third << 6);
    if (((quantity>0) && (multiplier<0)) || ((quantity<0) && (multiplier>0))) product = -product;
    return product+offset;
}
/****************************************************************************/
/** @defgroup UIT User Interface Functions

These provide some local display and input functionsmainly for the LCD panel
and push buttons.
@{*/
/****************************************************************************/
/** @brief Display a 32 bit value as decimal on the LCD

The value is an A/D conversion result. 16 bits represents a value out of 100
and 3 decimal digits are displayed as appropriate for a 10 bit A/D converter.

@param value 32 bit integer value to be converted
*/

void display(const long value)
{
    uint32_t v = value;
    if (value < 0)
    {
        v = -value;
        LCDByte(1,'-');
    }
    uint8_t precision = 4;
    for (uint8_t n = 0; n<precision; n++)
    {
        uint32_t x = v*10;
        uint8_t digit = (x >> 16);
        v = x - ((uint32_t)digit << 16);
        if ((n > 0) || (digit > 0)) LCDByte(1,digit+48);
        if (n == 1) LCDByte(1,'.');
    }
}

/****************************************************************************/
/** @brief Set the Error Code

An error code is set in the internal EEPROM for access by a monitor.
An LED state is modified to indicate presence of an error.
A message is displayed on the LCD screen with the code.
*/

void setErrorCode(const unsigned int code)
{
    if (code == 0)
        offLED();
    else
    {
        onLED();
        LCDClear();
        LCDWriteString("Error Code: ");
        LCDWriteHexWord((unsigned int)code,0);
    }
}
/**@}*/
/****************************************************************************/
/** @defgroup BSHF Board Specific Hardware Functions

These functions pull together all code that relates to hardware and port
manipulations specific to the particular microntroller board and the specific
microcontroller used. These may change if a different board or microcontroller
is used.

The board is a custom designed board with external EEPROM storage,
UART and a buffer or jumpers to read a 4-bit board ID. The SPI programming
port may also be buffered. In this example an ATMega168 is used.
@{*/
/****************************************************************************/
/** @brief Configure the ports to match hardware.

Set PORTD as outputs on bits 2-7 to set the LCD.
Note that some outputs on this port overlap with ID read inputs. On a
controller board that has no hardware buffer to protect these pins, disconnect
any jumpers connected to those pins. Leave the UART lines PD0,PD1 alone.

Also set the Power Control register PRR to turn off timer2, SPI and also ADC
(which will be re-enabled later) and the Analogue Comparator.

Setup the timer 1 to use fast PWM with the OC2B output and OCR2A used as a
compare register.
TCCR1B = 0x18 will stop timer, add 1-5 to divide by 1,8,64,256,1024 resp.
TCCR1A =0x03 sets both ports normal operation.
*/

void configurePorts(void)
{
/* Setup Timer 1 for use with the source disconnect */
    OCR1A = 0x400;
    OCR1B = 0x100;
    TCCR1A = 0x23;              /* Enable OC1B as output, OC1A disconnected */
    TCCR1B = 0x19;              /* Fast PWM, no clock prescale */
/* Set PortD DDR bits 2-7 to outputs for LCD. Bits 0,1 are the USART ports */
    outb(DDRD,(inb(DDRD) | 0xFC));
/* Set PORTB as outputs on bits 0-2 to set controls */
    outb(DDRB,(inb(DDRB) | 0x07));
/* Set all four ADC port pins (analogue inputs) as inputs */
    outb(DDRC,(inb(DDRC) & 0xF0));
/* power down unused peripherals - TC2, SPI, ADC */
    outb(PRR,0x45);
/* Turn off Analogue Comparator */
    sbi(ACSR,7);
}
/****************************************************************************/
/** @brief Port B output 0 when high will open the panel for charging.
*/

void openPanel(void)
{
    sbi(PORTB,PANEL);                   // Panel is not shorted
}
/****************************************************************************/
/** @brief Port B output 0 when low will short the panel.
*/

void shortPanel(void)
{
    cbi(PORTB,PANEL);                   // Panel is shorted
}
/****************************************************************************/
/** @brief Port B output 1 when low will connect the battery to the load.
*/

void connectLoad(void)
{
    cbi(PORTB,LOAD);                    // Load is connected
}
/****************************************************************************/
/** @brief Port B output 1 when high will isolate the load.
*/

void isolateLoad(void)
{
    sbi(PORTB,LOAD);                    // Load is not connected
}
/****************************************************************************/
/** @brief Port B output 2 when low will turn off a LED
*/

void onLED(void)
{
    cbi(PORTB,LED);                     // LED is off
}
/****************************************************************************/
/** @brief Port B output 2 when high will turn on a LED.
*/

void offLED(void)
{
    sbi(PORTB,LED);                     // LED is on
}
/**@}*/
/****************************************************************************/

