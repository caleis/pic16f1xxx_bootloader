; *****************************************************************************
; Copyright (C) 2023 Zoltan Fekete (github: caleis)
;
; Bootloader test program 1
; Blinking LEDs with delay loop - cannot be simpler
; This program is for testing the Bootloader for Enhanced Midrange
; PIC16F1xxx series
;
; 28/10/2023    First implementation committed to github
;
; This program is free software: you can redistribute it and/or modify
; it under the terms of the GNU General Public License version 3
; as published by the Free Software Foundation.

; This program is distributed in the hope that it will be useful,
; but WITHOUT ANY WARRANTY; without even the implied warranty of
; MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
; GNU General Public License for more details.

; You should have received a copy of the GNU General Public License
; along with this program.  If not, see <http://www.gnu.org/licenses/>. ;
;
;
; *****************************************************************************

	PROCESSOR	16F1574

; *****************************************************************************
	#include <xc.inc>
;
; *****************************************************************************

; Device specific parameters
; LED pins and ports
PIN_LEDGREEN	EQU	2			; RA2
PORT_LEDGREEN	EQU	0x0c			; ANSELA/PORTA/TRISA
PIN_LEDRED	EQU	0			; RC0
PORT_LEDRED	EQU	0x0e			; ANSELC/PORTC/TRISC

; Delay values
OUTERDLY	EQU	0xc0			; init value for outer delay counter

; *****************************************************************************
	PSECT	Scratchpad, space=1, class=COMMON, delta=1
	; space=1 represents data area (default is space=0, program memory

InnerLoopCtr:	DS	1			; inner loop counter
OuterLoopCtr:	DS	1			; outer loop counter

    ; *****************************************************************************

; Define configuration word
; Configuration Word 1:
; FOSC:	    Oscillator internal, CLK pin available as generic I/O
; WDTE:	    Watchdog Timer disabled
; -PWRTE:   Power-up Timer enabled - this is a fixed 64 msec power up delay
; MCLRE:    MCLR Pin Function Select - as we have LVP =1, this bit is ignored, pin is -MCLR with weak pull-up
; -CP:	    Flash Program Memory Code Protection off
; BOREN:    Brown-out Reset Enable set to software selectable (SBODEN)
; -CLKOUTEN:Clock Out disabled. CLKOUT is available as generic I/O pin

; Configuration Word 2:
; WRT:	    Flash Memory Self-Write Protection off (whole flash writeable)
; PPS1WAY:  Peripheral Pin Select (PPS) not locked
; PLLEN:    Phase Lock Loop disabled (4x PLL can be enabled when software sets the SPLLEN bit)
; STVREN:   Stack Overflow/Underflow Reset Enabled (Stack Overflow or Underflow will cause a Reset)
; BORV:	    Brown-out Reset Voltage (Vbor), low trip point selected
; -LPBOREN: Low-Power Brown Out Reset disabled
; LVP:	    Low-Voltage Programming enable (a must for our bootloader) */


	CONFIG	CLKOUTEN = OFF, BOREN = SBODEN, CP = OFF, MCLRE = OFF, PWRTE = ON, WDTE = OFF, FOSC = INTOSC
	CONFIG	LVP = ON, LPBOREN = OFF, BORV = LO, STVREN = ON, PLLEN = OFF, PPS1WAY = OFF, WRT = OFF

; *****************************************************************************
	PSECT   ResVect, class=CODE, delta=2
ResVect:
	nop					; for debugger support
	movlw	high(Application)		; Load the start of our test application
	movwf	PCLATH
	goto	Application			; and jump there

	PSECT   IntVect, class=CODE, delta=2
IntVect:
	goto	ApplicationInt			; same for interrupt vector (if any)

; *****************************************************************************
	PSECT   Main, class=CODE, delta=2
Application:
	banksel	OSCCON				; B1
	movlw	01101010B			; B1 Set to SPLLEN=0, 4 MHz and clock
	movwf	OSCCON				; B1
	bcf	INTCON, INTCON_GIE_POSN		; Bx disable interrupts, just in case
	banksel	ANSELA				; B3 Green LED setup, select digital I/O
	bcf	PORT_LEDGREEN, PIN_LEDGREEN	; B3
	bcf	PORT_LEDRED, PIN_LEDRED		; B3
	banksel TRISA				; B1 set it to output
	bcf	PORT_LEDGREEN, PIN_LEDGREEN	; B1
	bcf	PORT_LEDRED, PIN_LEDRED		; B1
	banksel LATA				; B2 set output (active 0)
	bcf	PORT_LEDGREEN, PIN_LEDGREEN	; B2 turn on green
	bsf	PORT_LEDRED, PIN_LEDRED		; B2 turn off red
	clrf	InnerLoopCtr			; initialise loop counters
	movlw	OUTERDLY
	movwf	OuterLoopCtr
ApplicationLoop:
	decfsz	InnerLoopCtr, f			; decrement inner loop counter
	bra	ApplicationLoop			; if not
	decfsz	OuterLoopCtr, f			; decrement outer loop counter
	bra	ApplicationLoop			; if not
	movlw	OUTERDLY			; initialise outer delay
	movwf	OuterLoopCtr
	banksel LATA				; B2
	btfsc	LATA, LATA_LATA2_POSN		; is green LED on?
	bra	GreenOn				; if not
	bsf	PORT_LEDGREEN, PIN_LEDGREEN	; B2 turn off green
	bcf	PORT_LEDRED, PIN_LEDRED		; B2 turn on red
	bra	ApplicationLoop			; go back to delay loop
GreenOn:
	bcf	PORT_LEDGREEN, PIN_LEDGREEN	; B2 turn on green
	bsf	PORT_LEDRED, PIN_LEDRED		; B2 turn off red
	bra	ApplicationLoop			; go back to delay loop


; *****************************************************************************
; Interrupt code - we do not yet have interrupts here
ApplicationInt:
	retfie

; *****************************************************************************


	END



