#include <inttypes.h>
#include <avr/sfr_defs.h>
#include <avr/io.h>
#include "lcd/lcd-KS0066.h"

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

void configurePorts(void)
{
    outb(DDRD, inb(DDRD) | 0xFC);	    /* Set PortD DDR to outputs for LCD */
    outb(PRR,0x4D);                     /* power down unused peripherals */
    sbi(ACSR,7);                        /* Turn off Analogue Comparator */
}

void testLCD(void)
{
}

int main(void)
{
//  wdt_disable();          	/* Stop watchdog timer */
    configurePorts();
    InitLCD(0x03);
/** Initialise the OS stuff, setup the control and receive tasks, and jump
to the OS. */
    LCDGotoXY(3,1);
    LCDWriteString("Hello");
//    taskExit();
//  for (;;)
//  {
//  }
    for (;;)
    {
        uint8_t x=0;
    }
}

