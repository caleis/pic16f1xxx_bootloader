; *****************************************************************************
; Copyright (C) 2023 Zoltan Fekete (github: caleis)
;
; Bootloader test program 2
; Blinking LEDs with timing provided by Timer interrupt
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

        PROCESSOR       16F1574

; *****************************************************************************
        #include <xc.inc>
;
; *****************************************************************************

; Device specific parameters
; LED pins and ports
PIN_LEDGREEN    EQU     2                       ; RA2
PORT_LEDGREEN   EQU     0x0c                    ; ANSELA/PORTA/TRISA
PIN_LEDRED      EQU     0                       ; RC0
PORT_LEDRED     EQU     0x0e                    ; ANSELC/PORTC/TRISC
PIN_INTPULSE    EQU     3                       ; RC3
PORT_INTPULSE   EQU     0x0e                    ; ANSELC/PORTC/TRISC

; *****************************************************************************
        PSECT   Scratchpad, space=1, class=COMMON, delta=1
        ; space=1 represents data area (default is space=0, program memory

IntDlyFlag:     DS      1                       ; Interrupt delay flag

; *****************************************************************************

; Define configuration word
; Configuration Word 1:
; FOSC:     Oscillator internal, CLK pin available as generic I/O
; WDTE:     Watchdog Timer disabled
; -PWRTE:   Power-up Timer enabled - this is a fixed 64 msec power up delay
; MCLRE:    MCLR Pin Function Select - as we have LVP =1, this bit is ignored, pin is -MCLR with weak pull-up
; -CP:      Flash Program Memory Code Protection off
; BOREN:    Brown-out Reset Enable set to software selectable (SBODEN)
; -CLKOUTEN:Clock Out disabled. CLKOUT is available as generic I/O pin

; Configuration Word 2:
; WRT:      Flash Memory Self-Write Protection off (whole flash writeable)
; PPS1WAY:  Peripheral Pin Select (PPS) not locked
; PLLEN:    Phase Lock Loop disabled (4x PLL can be enabled when software sets the SPLLEN bit)
; STVREN:   Stack Overflow/Underflow Reset Enabled (Stack Overflow or Underflow will cause a Reset)
; BORV:     Brown-out Reset Voltage (Vbor), low trip point selected
; -LPBOREN: Low-Power Brown Out Reset disabled
; LVP:      Low-Voltage Programming enable (a must for our bootloader) */


        CONFIG  CLKOUTEN = OFF, BOREN = SBODEN, CP = OFF, MCLRE = OFF, PWRTE = ON, WDTE = OFF, FOSC = INTOSC
        CONFIG  LVP = ON, LPBOREN = OFF, BORV = LO, STVREN = ON, PLLEN = OFF, PPS1WAY = OFF, WRT = OFF

; *****************************************************************************
        PSECT   ResVect, class=CODE, delta=2
ResVect:
        nop                                     ; for debugger support
        movlw   high(Application)               ; Load the start of our test application
        movwf   PCLATH
        goto    Application                     ; and jump there

; *****************************************************************************
       PSECT   IntVect, class=CODE, delta=2
; Interrupt function
; It provides a pulse on RC3 to see the period and actual length of interrupt routine
; With Fosc/4 (1MHz), 64 as prescaler, 15 as postscaler and 250 as timer counter
; it hits in every 240 msec.
TmrInt:
	banksel	LATC				; B2
        bsf     LATC, LATC_LATC3_POSN           ; B2 set RC3 to high (start of interrupt routine)
        btfss   INTCON, INTCON_PEIE_POSN        ; B2 check if peripheral interrupt enabled
        bra     TmrIntE                         ; B2 if not
	banksel	PIR1				; B0
        btfss   PIR1, PIR1_TMR2IF_POSN          ; B0 check if Timer 2 interrupt active
        bra     TmrIntE                         ; B0 if not
        movlw   1                               ; B0 Set the interrupt delay flag for main program
        movwf   IntDlyFlag			; B0
        bcf     PIR1, PIR1_TMR2IF_POSN          ; B0 clear Timer 2 interrupt flag
TmrIntE:
	banksel LATC				; B2
        bcf     LATC, LATC_LATC3_POSN           ; B2 set RC3 to low (end of interrupt routine)
        retfie

; *****************************************************************************
        PSECT   Main, class=CODE, delta=2
Application:
        banksel OSCCON                          ; B1
        movlw   01101010B                       ; B1 Set to SPLLEN=0, 4 MHz and clock
        movwf   OSCCON                          ; B1
        bcf     INTCON, INTCON_GIE_POSN         ; Bx disable interrupts, just in case
        banksel ANSELA                          ; B3 Green LED setup, select digital I/O
        bcf     PORT_LEDGREEN, PIN_LEDGREEN     ; B3
        bcf     PORT_LEDRED, PIN_LEDRED         ; B3
        bcf     PORT_INTPULSE, PIN_INTPULSE     ; B3
        banksel TRISA                           ; B1 set it to output
        bcf     PORT_LEDGREEN, PIN_LEDGREEN     ; B1
        bcf     PORT_LEDRED, PIN_LEDRED         ; B1
        bcf     PORT_INTPULSE, PIN_INTPULSE     ; B1
        banksel LATA                            ; B2 set output (active 0)
        bcf     PORT_LEDGREEN, PIN_LEDGREEN     ; B2 turn on green
        bsf     PORT_LEDRED, PIN_LEDRED         ; B2 turn off red
        bcf     PORT_INTPULSE, PIN_INTPULSE     ; B2 turn off interrupt pulse (low)
        clrf    IntDlyFlag                      ; B2 Initialise Interrupt delay flag
        banksel T2CON                           ; B0
        movlw   01110111B                       ; B0 TMR2 on, prescaler 64, postscaler 15
        movwf   T2CON
        movlw   249                             ; B0 period register setting
        movwf   PR2
        banksel PIE1                            ; B1
        bsf     PIE1, PIE1_TMR2IE_POSN          ; B1 enabling Timer 2 interrupt
        bsf     INTCON, INTCON_PEIE_POSN        ; B1 enable peripheral interrupt
        bsf     INTCON, INTCON_GIE_POSN         ; B1 enable global interrupt
ApplicationLoop:
        btfss   IntDlyFlag,0                    ; Has interrupt delay expired?
        bra     ApplicationLoop                 ; if not
        clrf    IntDlyFlag
        banksel LATA                            ; B2
        btfsc   PORT_LEDGREEN, PIN_LEDGREEN     ; is green LED on?
        bra     GreenOn                         ; if not
        bsf     PORT_LEDGREEN, PIN_LEDGREEN     ; B2 turn off green
        bcf     PORT_LEDRED, PIN_LEDRED         ; B2 turn on red
        bra     ApplicationLoop                 ; go back to delay loop
GreenOn:
        bcf     PORT_LEDGREEN, PIN_LEDGREEN     ; B2 turn on green
        bsf     PORT_LEDRED, PIN_LEDRED         ; B2 turn off red
        bra     ApplicationLoop                 ; go back to delay loop


; *****************************************************************************


        END



