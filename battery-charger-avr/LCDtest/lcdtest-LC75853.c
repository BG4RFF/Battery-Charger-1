#include <inttypes.h>
#include <avr/sfr_defs.h>
#include <avr/io.h>
#include "lcd/lcd-LC75853N.h"
#include "nartos/nartos.h"

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

volatile uint8_t testLCDID;

void configurePorts(void)
{
  outb(DDRD, inb(DDRD) | 0x1C);	      /* Set PortD DDR to outputs for LCD */
  outb(PRR,0x4D);                     /* power down unused peripherals */
  sbi(ACSR,7);                        /* Turn off Analogue Comparator */
}

int main(void)
{
//  wdt_disable();          	/* Stop watchdog timer */
  configurePorts();
  InitLCD();
  uint8_t data[16];
  uint8_t control = 0;
  for (uint8_t i=8; i>0; i--)
  {
    data[i] = 35;
  }
  displayLCD(data,control);
  for (;;)
  {
    displayLCD(data,control);
  }
}

