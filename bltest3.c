/******************************************************************************
 Copyright (C) 2023 Zoltan Fekete (github: caleis)

 Bootloader test program 3
 Blinking LEDs with delay loop - cannot be simpler, now in C:
 blinking 2 LED on RA2 and RC5 alternating
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
 along with this program.  If not, see <http://www.gnu.org/licenses/>. ;

 *****************************************************************************/

/* PIC12F1574 Configuration Bit Settings */

/* 'C' source line config statements */

/* CONFIG1 */
#pragma config FOSC = INTOSC     /* Oscillator internal, i/o on CLK pin */
#pragma config WDTE = OFF        /* Watchdog Timer Enable (WDT disabled) */
#pragma config PWRTE = OFF       /* Power-up Timer Enable (PWRT disabled) */
#pragma config MCLRE = OFF       /* MCLR Pin Function Select (MCLR is a digital input) */
#pragma config CP = OFF          /* Flash Program Memory Code Protection (Program memory code protection is disabled) */
#pragma config BOREN = ON        /* Brown-out Reset Enable (Brown-out Reset enabled) */
#pragma config CLKOUTEN = OFF    /* Clock Out Enable (CLKOUT function is disabled. I/O or oscillator function on the CLKOUT pin) */

/* CONFIG2 */
#pragma config WRT = OFF         /* Flash Memory Self-Write Protection (Write protection off) */
#pragma config PLLEN = OFF       /* Phase Lock Loop enable (4x PLL is enabled when software sets the SPLLEN bit) */
#pragma config STVREN = ON       /* Stack Overflow/Underflow Reset Enable (Stack Overflow or Underflow will cause a Reset) */
#pragma config BORV = LO         /* Brown-out Reset Voltage Selection (Brown-out Reset Voltage (Vbor), low trip point selected.) */
#pragma config LPBOREN = OFF     /* Low-Power Brown Out Reset (Low-Power BOR is disabled) */
#pragma config LVP = OFF         /* Low-Voltage Programming Enable (High-voltage on MCLR/VPP must be used for programming) */

/* #pragma config statements should precede project file includes.
   Use project enums instead of #define for ON and OFF. */

#include <xc.h>
#include <stdint.h>

void main(void) {

   uint8_t portValue;

   /* Port A access */
   ANSELA = 0x00;                /* set to digital I/O (not analog) */
   ANSELC = 0x00;
   TRISA = 0x3b;                 /* set RA2 as input only */
   TRISC = 0xfe;
    
   uint8_t i;

   while(1) {
      LATA = 0x04;               /* write to port latches */
      LATC = 0x00;
      _delay(30000);             /* delay value change - instruction cycles */
      LATA = 0x00;               /* write to port latches */
      LATC = 0x01;
      _delay(30000);             /* delay value change - instruction cycles */
   }
 
   return;
}

