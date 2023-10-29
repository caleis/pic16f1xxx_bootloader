/******************************************************************************
 Copyright (C) 2023 Zoltan Fekete (github: caleis)

 Bootloader test program 4
 Blinking 2 LEDs connected to RA2 & RC0, timing based on interrupts (written in C)
 This program is for testing the Bootloader for Enhanced Midrange
 PIC16F1xxx series

 28/10/2023    First implementation committed to github

 This program is free software: you can redistribute it and/or modify
 it under the terms of the GNU General Public License version 3
 as published by the Free Software Foundation.

 This program is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details.

 You should have received a copy of the GNU General Public License
 along with this program.  If not, see <http://www.gnu.org/licenses/>.

 *****************************************************************************/

/* This program uses 8-bit TMR2 timer. This is the only timer which does not need reloading, so would be accurate */

/* PIC12F1574 Configuration Bit Settings */

/* 'C' source line config statements */

/* CONFIG1 */
#pragma config FOSC = INTOSC    /* Oscillator internal, i/o on CLK pin */
#pragma config WDTE = OFF       /* Watchdog Timer Enable (WDT disabled) */
#pragma config PWRTE = OFF      /* Power-up Timer Enable (PWRT disabled) */
#pragma config MCLRE = OFF      /* MCLR Pin Function Select (MCLR is a digital input) */
#pragma config CP = OFF         /* Flash Program Memory Code Protection (Program memory code protection is disabled) */
#pragma config BOREN = ON       /* Brown-out Reset Enable (Brown-out Reset enabled) */
#pragma config CLKOUTEN = OFF   /* Clock Out Enable (CLKOUT function is disabled. I/O or oscillator function on the CLKOUT pin) */

/* CONFIG2 */
#pragma config WRT = OFF        /* Flash Memory Self-Write Protection (Write protection off) */
#pragma config PLLEN = OFF      /* Phase Lock Loop enable (4x PLL is enabled when software sets the SPLLEN bit) */
#pragma config STVREN = ON      /* Stack Overflow/Underflow Reset Enable (Stack Overflow or Underflow will cause a Reset) */
#pragma config BORV = LO        /* Brown-out Reset Voltage Selection (Brown-out Reset Voltage (Vbor), low trip point selected.) */
#pragma config LPBOREN = OFF    /* Low-Power Brown Out Reset (Low-Power BOR is disabled) */
#pragma config LVP = OFF        /* Low-Voltage Programming Enable (High-voltage on MCLR/VPP must be used for programming) */

/* #pragma config statements should precede project file includes.
   Use project enums instead of #define for ON and OFF. */

#include <xc.h>
#include <stdint.h>
#include <stdbool.h>

static _Bool IntDelay = false;
/* Interrupt function
   provides pulse on RC3 to see actual length of interrupt routine */

void __interrupt() tcInt(void){
   LATCbits.LATC3 = 1;
   /* only process Timer2-triggered interrupts */
   if (INTCONbits.PEIE && PIR1bits.TMR2IF) {
      /* set flag for main program */
      IntDelay = true;
      /* clear this interrupt condition */
      PIR1bits.TMR2IF = 0;
   }
   LATCbits.LATC3 = 0;
}

void main(void) {

   uint8_t portValue;
   uint8_t i;

   /* Set oscillator mode: MF osc, 500 kHz - suitable performance, small power */
   OSCCONbits.SPLLEN = 0;
   OSCCONbits.IRCF = 0b0111;
   OSCCONbits.SCS = 0;

    /* Port A access */
   ANSELA = 0x00;                /* set to digital I/O (not analog) */
   ANSELC = 0x00;
   TRISA = 0x3b;                 /* set RA2 as output */
   TRISC = 0xf6;                 /* set RC0, RC3 as output */

    /* Timer2 setup */
    T2CONbits.T2OUTPS = 0b0100;  /* output postscaler is set to 1:5 */
    T2CONbits.TMR2ON = 1;        /* timer 2 on */
    T2CONbits.T2CKPS = 0b11;     /* prescaler set to 1:64 (this will be fosc/4) */
    PR2 = 124;                   /* period register setting */
    PIE1bits.TMR2IE = 1;         /* enable TMR2 interrupt */
    INTCONbits.PEIE = 1;         /* enable periperal interrupts */
    INTCONbits.GIE = 1;          /* global interrupt enabled */

    while(1) {
      LATAbits.LATA2 = 1;        /* write to port latches */
      LATCbits.LATC0 = 0;
      while (!IntDelay);         /* wait for timer expiry */
      IntDelay = 0;
      LATAbits.LATA2 = 0;        /* write to port latches */
      LATCbits.LATC0 = 1;
      while (!IntDelay);         /* wait for timer expiry */
      IntDelay = 0;
    }

    return;
}

