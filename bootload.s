; *****************************************************************************
; Copyright (C) 2023 Zoltan Fekete (github: caleis)
;
; Bootloader for Enhanced Midrange PIC16F1xxx series
; Adopted from PIC16F bootloader by Rodger Richey, which was adopted from PIC18F
; bootloader by Ross Fosler (from 2002)
; Command compatibility with implementation described in Microchips AN1310 application
; note.
; 28/10/2023    First implementation committed to github

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
; Memory Map
;       -----------------
;       |    0x0000     |       Bootloader Reset vector (4 words)
;       |               |
;       |    0x0004     |       Interrupt vector
;       |       |       |
;       |       |       |
;       |  Application  |       User Application Code Space
;       |       |       |
;       |       |       |
;       |    0x0e1f     |
;       |               |
;       |    0x0e20     |       Unused area (block reserved for reset vector)
;       |       |       |
;       |    0x0e3d     |
;       |               |
;       |    0x0e3e     |       Re-mapped Application Reset Vector (4 words)
;       |               |
;       |    0x0e40     |
;       |       |       |
;       |  Bootloader   |       Bootloader (this program)
;       |       |       |
;       |    0x0fff     |
;       -----------------
;
;
; Incoming data format:
;
;       <STX><STX><DATA><CRC16><ETX>
;                 /    \
;        ________/      \____________________________
;       /                                            \
;       <COMMAND><ADDRL><ADDRH><ADDRU>0<DSIZE><DATA>...
;
; Definitions:
;
;       STX     -       Start of packet indicator
;       ETX     -       End of packet indicator
;       DATA    -       General data up to 255 bytes
;       CRC16   -       16-bit Cyclic Redundancy Check
;       COMMAND -       Base command
;       DSIZE   -       Length of data associated to the command (8/16 bits)
;       ADDR    -       Address up to 24 bits
;       DATA    -       Data (if any)
;
;
; Commands:
;
;       BootloaderInfo  0x00    Read Bootloader Information
;       ReadFlash       0x01    Read Flash Memory
;       VerifyFlash     0x02    Read CRC of Flash Memory
;       EraseFlash      0x03    Erase Flash Memory
;       WriteFlash      0x04    Write Flash Memory
;       ReadEEPROM      0x05    Read EEDATA Memory (not supported in PIC16F1xxx)
;       WriteEEPROM     0x06    Write EEDATA Memory (not supported in PIC16F1xxx)
;       WriteConfig     0x07    Write Config Fuses (not supported in PIC16F1xxx)
;       RunApplication  0x08    Run Application Program
;
; *****************************************************************************

        PROCESSOR       16F1574

; *****************************************************************************
        #include <xc.inc>
; *****************************************************************************

; Hardware specific parameters
; Tx and Rx pins are on PORT A
RXPIN           EQU     5
TXPIN           EQU     4

; Programming parameters
BOOTBYTES       EQU     0x01a0                  ; Bootblock size (size of this bootloader)
COMMANDMASKH    EQU     0x01                    ; Command mask: flash erase available
COMMANDMASKL    EQU     0x02                    ; Command mask: family id 2 (pic16)
STARTBOOT       EQU     0x0e40                  ; Boot block start address

; Bootloader program version (x.y.zz minor zz, major xy)
MINOR_VERSION   EQU     0x06                    ;zz
MAJOR_VERSION   EQU     0x02                    ;xy

; Device ID addresses
DEVICE_ID_ADDR  EQU     0x8006                  ; Device ID config memory location
REV_ID_ADDR     EQU     0x8005                  ; Revision ID config memory location

; Device specific parameters
; PIC 16F1574/5/8/9 specific constants
DEV_ERASEBLOCK  EQU     0x20                    ; Erase block size for device
DEV_WRITEBLOCK  EQU     0x20                    ; Write block size for device

; Host programming protocol related constants
STX             EQU     0x0F
ETX             EQU     0x04
DLE             EQU     0x05

; *****************************************************************************



; *****************************************************************************
        PSECT   Scratchpad, space=1, class=COMMON, delta=1
        ; space=1 represents data area (default is space=0, program memory
TxData:         DS      1                       ; scratchpad register to hold data to be transmitted
CrcTemp:        DS      1                       ; scratchpad register to hold data for CRC calc
CrcTempRx:      DS      1                       ; scratchpad register to hold data for received CRC check
CrcL:           DS      1                       ; temp memory to hold calculated CRC lo byte
CrcH:           DS      1                       ; temp memory to hold calculated CRC hi byte
ByteCtrL:       DS      1                       ; counter register for message byte counting (low byte)
ByteCtrH:       DS      1                       ; counter register for message byte counting (high byte)
ByteCtrIB:      DS      1                       ; counter register for intrablock byte counting

; *****************************************************************************
; Data buffer definition
; Placed into non-banked (class COMMON) memory, starting at address 0x2000
; IMPORTANT: We assume in this program, that receive data buffer (MsgBuf) starts on 0x100 page boundary
;            (i.e. buffer entirely located within the 0x100 long page)
; Frame Format
;  <STX><STX>[<COMMAND><DATALEN><ADDRL><ADDRH><ADDRU><...DATA...>]<CHKSUM><ETX>
; Multiple labels for the same buffer location are used as buffer usage is not consistent among commands

DATA_BUF_LEN    equ     (MsgBufEnd - MsgBuf)    ; maximum data buffer length

        PSECT   DataBuf, class=BIGRAM, delta=1
        ; space=1 represents data area (default is space=0, program memory
MsgBuf:                                         ; Start of receive data buffer
MsgCmd:         DS      1                       ; received command code
MsgDataBuf0:                                    ; start of data buffer for Run Appl cmd (CRC only)
MsgAddress:                                     ; optional received address (following 3 bytes)
MsgAddrL:       DS      1                       ; address low byte
MsgAddrH:       DS      1                       ; address hi byte
MsgAddrU:       DS      1                       ; flash address only upper byte
MsgZero:        DS      1                       ; if present this byte should be zero
MsgDataBuf:                                     ;
MsgSizeL:       DS      1                       ; optional block/byte length 8-bit or low byte
MsgDataBuf1:
MsgSizeH:       DS      1                       ; optional block/byte length 16 but high byte
MsgDatabuf2:    DS      247                     ; received data bytes (including CRC)
MsgDataLen:     DS      1                       ; data length
MsgBufEnd:

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
; Program starts here after reset
; Bootloader is always the one to start up. It will decide if it needs to enter to bootloader
; mode or go to the application.
; Applications need to be compiled normally with reset vector at 0x000, but the host program
; would automatically relocate it to the Application Reset Vector area.
; The reset vector area of the application should contain relocatable instructions only and a
; goto instruction to the entry point of the application.
; XC8 compiler in certain cases seems to make an extra jump within that 4 byte Reset vector area
; which causes issues as it seems to be using absolute jump (goto)
; The host programming software is expected to take care of the modification of the reset vector
; of the application program.
; Microchip should fix their XC8 compiler...

        PSECT   ResVect, space=0, class=CODE, delta=2
ResVect:
        nop                                     ; required to allow debug executive startup when running under ICD
        movlp   high(BootloaderBreakCheck)
        goto    PAGEMASK(BootloaderBreakCheck)


;       PSECT   IntVect, class=CODE, delta=2
; Bootloader does not use interrupts - application interrupt vector will come here
;
; *****************************************************************************
; Location reserved for relocated Application Reset Vector
; We only reserve the space here, will be written during application programming
; Note: because Flash Erase goes in blocks, this must be located one block below
; the start of Bootloader code
;
        PSECT   AppResVect, space=0, class=CODE, delta=2
AppResVect:
        DS      4                               ; reserve the space

; Application Interrupt Vector is not relocated, no need to reserve space for it
;       PSECT   AppIntVect, class=CODE, delta=2

; *****************************************************************************
; Main bootloader program
; The bootloader should be located to the very top of the Flash memory
; For the PIC 16F1574 it can be as high as 0x0e60
;
        PSECT   Bootloader, space=0, class=CODE, delta=2

BootloaderBreakCheck:
        banksel ANSELA                          ; B3
        bcf     ANSELA, RXPIN                   ; B3 set RX pin to digital input
        banksel PORTA                           ; B0
        btfss   PORTA, RXPIN                    ; B0 Check for BREAK on RxD pin B0
        bra     WaitForRxIdle                   ; B0 BREAK detected, go to bootloader

        ; Attempt to startup in Application mode.
        ; Read instruction at the application reset vector location.
        ; If we read 0x3FFF, assume that the application firmware has
        ; not been programmed yet, so don't try going into application mode.
        banksel PMADRL                          ; B0 -> B3
        movlw   low(AppResVect)                 ; Bx load address of application reset vector
        movwf   PMADRL                          ; B3
        movlw   high(AppResVect)
        movwf   PMADRH                          ; B3
        call    PAGEMASK(ReadFlashWord)         ; Bx -> B0

        addlw   1
        btfss   STATUS,STATUS_Z_POSN            ; Bx if the lower byte != 0xFF,
        goto    AppResVect                      ; Bx run application.

        movlw   0x3F
        xorwf   FSR1H, w                        ; Bx if the lower byte == 0xFF but upper byte != 0x3F,
        btfss   STATUS,STATUS_Z_POSN            ; Bx run application
        goto    AppResVect

ProgrammerMode:
        banksel PORTA                           ; B0
WaitForRxIdle:
        btfss   PORTA, RXPIN                    ; B0 BREAK found, wait for RXD to go IDLE
        bra     WaitForRxIdle

        ; Setting up oscillator and UART modes to start
BootLoadMode:
        movlw   00110000B                       ; Fosc/4, 1:8 prescaler - no division required later (but no rounding possible)
        movwf   T1CON                           ; B0
        banksel RCSTA                           ; B3
        movlw   00100100B                       ; B3 Setup UART
        movwf   TXSTA                           ; B3 Async mode, Tx enable, BRGH = 1
        movlw   10010000B                       ; B3 Setup UART
        movwf   RCSTA                           ; B3 SPEN: Serial port enable, CREN: Continuous receive mode
        bsf     BAUDCON, BAUDCON_BRG16_POSN     ; B3 16-bit baud rate generator used
        banksel RXPPS                           ; B28
        movlw   00000101B                       ; B28 Rx pin to RA5
        movwf   RXPPS                           ; B28
        banksel RA4PPS                          ; B29
        movlw   00001001B                       ; B29 Tx to pin RA4
        movwf   RA4PPS                          ; B29
        banksel OSCCON                          ; B1
        movlw   11110000B                       ; B1 Set to PLLEN=1, 8 MHz and clock select according to FOSC bits (clock 32 MHz)
        movwf   OSCCON                          ; B1
        ; from here falls through to MainLoop

; *****************************************************************************
; Main loop after initialisation
; baud rate detection/setting and processing of incoming commands
;
MainLoop:
        call    PAGEMASK(SetBaudRate)
MainLoopInner:
        call    PAGEMASK(ProcessHostCommand)
        btfss   WREG, 0                         ; Bx check if error occured
        bra     MainLoopInner                   ; Bx no error, continue with next command
        bra     MainLoop                        ; Bx error occurred, reinit baudrate

; *****************************************************************************
; Autobaud detection function
; Input:   None
; Returns: None
; ___    __________            ________
;    \__/          \__________/
;       |                     |
;       |-------- p ----------|
;
;   p = The number of instructions between the first and last
;           rising edge of the RS232 control sequence 0x0F. Other
;       possible control sequences are 0x01, 0x03, 0x07, 0x1F,
;       0x3F, 0x7F.
;
;   SPBRG = (p / 32) - 1    BRGH = 1, BRG16 = 0
;   SPBRG = (p / 8) - 1     BRGH = 1, BRG16 = 1
;
SetBaudRate:
        banksel RCSTA                           ; B3
        bcf     RCSTA, RCSTA_CREN_POSN          ; B3 Stop UART RX, we're going to do autobaud pulse width timing instead
        movf    RCREG, W                        ; B3 Empty the UART receive buffer
        movf    RCREG, W                        ; B3
        banksel TMR1H                           ; B0
RetryAutoBaud:
        clrf    TMR1H                           ; B0 reset timer count value
        clrf    TMR1L                           ; B0
        bcf     PIR1, PIR1_TMR1IF_POSN          ; B0
        call    PAGEMASK(WaitForRise)           ; B0 wait for a start bit to pass by
        bsf     T1CON, T1CON_TMR1ON_POSN        ; B0 start timer counting for entire D7..D0 data bit period.
        call    PAGEMASK(WaitForRise)           ; B0 wait for stop bit
        bcf     T1CON, T1CON_TMR1ON_POSN        ; B0 stop the timer from counting further.
        btfsc   PIR1, PIR1_TMR1IF_POSN          ; B0 if TMR1 overflowed, we did not get a good baud capture
        bra     RetryAutoBaud                   ; B0 try again

        ; save new baud rate generator value
        movf    TMR1L, W                        ; B0 warning: must read TMR0L before TMR0H holds real data
        banksel SPBRGL                          ; B3
        movwf   SPBRGL                          ; B3 load value to SPBRG low byte
        banksel TMR1H                           ; B0
        movf    TMR1H, W                        ; B0
        banksel SPBRGH                          ; B3
        movwf   SPBRGH                          ; B3 load value to SPBRG low byte
        return

; *****************************************************************************
; Process host command
; Input:   None
; Returns: W = 0, if ok, then buffer has data decoded from input
;          W = 1, error occurred
;
ProcessHostCommand:                             ; Bx
        banksel RCSTA                           ; B3
        bsf     RCSTA, RCSTA_CREN_POSN          ; B3 start receiving
        movlw   high(MsgBuf)                    ; B3 load address of receive data buffer
        movwf   FSR0H                           ; B3 hi byte is assumed to be static
        movlw   low(MsgCmd)                     ; B3 load lo byte of address of command buffer
        movwf   FSR0L                           ; B3
        call    PAGEMASK(ReadHostByte)          ; Bx get start of transmission <STX>
        xorlw   STX                             ; Bx
        btfss   STATUS, STATUS_Z_POSN           ; Bx if received char STX continue to CMD receive
        retlw   1                               ; Bx got something unexpected, return with error

        ; Read and parse packet data
StartOfLine:
        movlw   STX                             ; send back start of response
        call    PAGEMASK(SendHostByte)          ; Bx

ReceiveDataLoop:
        call    PAGEMASK(ReadHostByte)          ; Bx Get the data
        xorlw   STX                             ; Bx Check for an unexpected STX
        btfsc   STATUS, STATUS_Z_POSN           ; Bx
        bra     StartOfLine                     ; Bx unexpected STX: abort packet and start over.

NoSTX:
        movf    INDF0, W                        ; Bx get latest read byte
        xorlw   ETX                             ; Bx Check for a ETX
        btfsc   STATUS, STATUS_Z_POSN
        bra     VerifyPacketCRC                 ; Bx if ETX, verify CRC

NoETX:
        movf    INDF0, W                        ; Bx
        xorlw   DLE                             ; Bx Check for a DLE
        btfss   STATUS, STATUS_Z_POSN
        bra     AppendDataBuffer                ; Bx if not DLE, append data to buffer
        call    PAGEMASK(ReadHostByte)          ; Bx DLE received, get the next byte and store it

AppendDataBuffer:
        incf    FSR0L, f                        ; Bx move to next empty location
        btfss   STATUS, STATUS_Z_POSN           ; Bx has the pointer overflown the buffer length?
        bra     ReceiveDataLoop                 ; nope, continue receiving data
        retlw   1                               ; overflow, probably wrong baudrate, return with error

VerifyPacketCRC:
        decf    FSR0L, W                        ; Bx pointer to point to CRCL location
        movwf   CrcTempRx                       ; Bx save end of packet pointer
        decf    CrcTempRx, f                    ; Bx pointer to point to last data byte location

        movlw   low(MsgCmd)                     ; Bx load address command buffer
        movwf   FSR0L                           ; Bx
        clrf    CrcL                            ; Bx reset CRC accumulator
        clrf    CrcH                            ; Bx

VerifyPacketCrcLoop:
        movf    INDF0, W                        ; Bx
        call    PAGEMASK(AddByteToCrc)          ; Bx add new data to the CRC
        incf    FSR0L, f                        ; Bx
        movf    FSR0L, W                        ; Bx
        subwf   CrcTempRx, W                    ; Bx
        btfss   STATUS, STATUS_Z_POSN           ; Bx check if end of the buffer
        bra     VerifyPacketCrcLoop             ; Bx if not, loop back for next byte

        movf    CrcL, W                         ; Bx
        subwf   INDF0, W                        ; Bx
        btfss   STATUS, STATUS_Z_POSN           ; Bx check if CRC low byte matches with received
        retlw   1                               ; Bx return with error if not
        incf    FSR0L, f                        ; Bx
        movf    CrcH, W                         ; Bx
        subwf   INDF0, W                        ; Bx
        btfss   STATUS, STATUS_Z_POSN           ; Bx check if CRC hi byte matches with received
        retlw   1                               ; Bx return with error if not

        ; Pre-setup, common to all commands.
        clrf    CrcL                            ; Bx
        clrf    CrcH                            ; Bx
        banksel PMADRH                          ; B3
        movlw   high(MsgBuf)                    ; B3 load hi byte of receive buffer address to FSR0H (does not change)
        movwf   FSR0H                           ; B3
        movlw   low(MsgAddrL)                   ; B3 set up PMADR with starting flash address
        movwf   FSR0L                           ; B3 PMADRL from MsgAddrL
        movf    INDF0, W                        ; B3
        movwf   PMADRL                          ; B3
        movlw   low(MsgAddrH)                   ; B3 PMADRH from MsgAddrH
        movwf   FSR0L                           ; B3
        movf    INDF0, W                        ; B3
        movwf   PMADRH                          ; B3
        movlw   low(MsgCmd)                     ; B3 load command address
        movwf   FSR0L                           ; B3

        ; Selecting and processing commands
CheckCommand:
        movlw  (CmdTableEnd - CmdTableStart)
        subwf   INDF0, W                        ; B3 test for valid command number
        btfsc   STATUS, STATUS_C_POSN           ; B3 is it a valid commmand?
        retlw   1                               ; B3 not valid
        moviw   FSR0++                          ; B3 get the MsgCmd back again, FSR to point to MsgDataBuf0

        ; This jump table must exist entirely within one 256 byte block of program memory.
        brw                                     ; B3 jump to the command routine (return with retlw)
CmdTableStart:
        goto    BootloaderInfo                  ; B0 0
        goto    ReadFlash                       ; Bx 1
        goto    VerifyFlash                     ; Bx 2
        goto    EraseFlash                      ; Bx 3
        goto    WriteFlash                      ; Bx 4
        goto    SendAcknowledge                 ; B0 5 - ReadEEPROM not supported on PIC16F1xxx devices
        goto    SendAcknowledge                 ; Bx 6 - WriteEEPROM not supported on PIC16F1xxx devices
        goto    SendAcknowledge                 ; Bx 7 - WriteConfig not supported on PIC16F1xxx devices
        goto    RunApplication                  ; Bx 8
CmdTableEnd:
#if (JUMPTABLE_BEGIN & 0xFF) > (JUMPTABLE_END & 0xFF)
    error "Jump table is not aligned to fit within a single 256 byte address range."
#endif

        retlw   0                               ; Bx no error occurred return

; *****************************************************************************
; Command - Provides information about Bootloder to host PC.
; Input:   FSR0: pointer to MsgDataBuf0 (just after command code)
; Returns: 0 if ok, 1 if error occurred
;
; First the constant part of Bootloaderinfo message response (the device info will be appended to it)
;
BootInfoBlock:
        db      low(BOOTBYTES), high(BOOTBYTES)
        db      MINOR_VERSION, MAJOR_VERSION
        db      COMMANDMASKH, COMMANDMASKL
        db      low(STARTBOOT), high(STARTBOOT)
        db      0, 0
BootInfoBlockEnd:

BootloaderInfo:                                 ; Bx
        movlw   (BootInfoBlockEnd - BootInfoBlock)
        movwf   ByteCtrL                        ; Bx length of block to counter
        movlw   low(BootInfoBlock)              ; Bx load fixed part address to PMADR
        movwf   FSR1L                           ; Bx
        movlw   high(BootInfoBlock) | 0x80      ; Bx
        movwf   FSR1H                           ; Bx
BootloaderInfo1:
        moviw   FSR1++                          ; Bx get low byte of flash to W and increment ptr
        movwi   FSR0++                          ; Bx save it to data buffer and inc ptr
        movlw   1                               ; Bx get counter and test it
        subwf   ByteCtrL, 1                     ; Bx
        btfss   STATUS, STATUS_Z_POSN           ; Bx finish if counter 0
        bra     BootloaderInfo1                 ; Bx otherwise loop back for more bytes
        ; Get the Device ID / Revision ID
        banksel PMADRL                          ; B3
        movlw   low(DEVICE_ID_ADDR)             ; B3 address to PMADR (hi byte set automatically)
        movwf   PMADRL                          ; B3
        call    PAGEMASK(ReadFlashConfig)       ; B3 get device id
        movwi   FSR0++                          ; B3 save lo byte
        movf    FSR1H, W                        ; B3 save hi byte
        movwi   FSR0++                          ; B3
        ; Send response to host PC
        movlw   low(MsgDataBuf0)                ; B3 load response buffer start, FSR0H did not change
        movwf   FSR0L                           ; B3
        movlw   (BootInfoBlockEnd - BootInfoBlock + 2)
        movwf   ByteCtrL                        ; Bx length of block to counter
BootloaderInfoSend1:
        moviw   FSR0++                          ; Bx next byte to W
        call    PAGEMASK(SendEscapeByte)        ; Bx Send only the command byte (acknowledge packet)
        call    PAGEMASK(AddByteToCrc)          ; Bx
        movlw   1                               ; Bx get counter and test it
        subwf   ByteCtrL, 1                     ; Bx
        btfss   STATUS, STATUS_Z_POSN           ; Bx finish if counter 0
        bra     BootloaderInfoSend1             ; Bx otherwise loop back for more bytes
        call    PAGEMASK(SendChecksum)          ; Bx finish packet and return
        goto    SendETX

; *****************************************************************************
; Command - Read block of bytes from flash and send it out on serial port
;          with STX, escaping, CRC and ETX
; Inputs:  PMADR: pointer to flash word to read
;          MsgSizeL, H: number of bytes to be read
; Returns: 0 if ok, 1 if error occured
;
ReadFlash:
        banksel PMADRH                          ; B3
        moviw   [MsgSizeL - MsgDataBuf0]FSR0    ; B3 setup byte counter in easy to reach place
        movwf   ByteCtrL
        moviw   [MsgSizeH - MsgDataBuf0]FSR0
        movwf   ByteCtrH
ReadFlash1:
        call    PAGEMASK(ReadFlashWord)         ; B3 read word from flash
        call    PAGEMASK(SendEscapeByte)        ; B3 send lo byte with escape
        call    AddByteToCrc                    ; B3 update CRC
        movf    FSR1H, W                        ; B3 read hi bits
        call    PAGEMASK(SendEscapeByte)        ; B3 send with escape
        call    PAGEMASK(AddByteToCrc)          ; B3 update CRC

        incf    PMADRL, f                       ; B3 increment PMADR
        btfsc   STATUS, STATUS_Z_POSN           ; B3
        incf    PMADRH, f                       ; B3
        movlw   1                               ; B3 decrement counter
        subwf   ByteCtrL, f                     ; B3
        btfss   STATUS, STATUS_C_POSN           ; B3
        decf    ByteCtrH, f                     ; B3

        movf    ByteCtrH, W                     ; B3 is ByteCtr == 0?
        iorwf   ByteCtrL, W                     ; B3
        btfss   STATUS, STATUS_Z_POSN           ; B3 non-zero, keep reading more data
        bra     ReadFlash1                      ; B3
        call    PAGEMASK(SendChecksum)          ; B3 zero, exit read loop and send end of packet
        goto    SendETX

; *****************************************************************************
; Command - Verify (i.e. calculate and transmit CRC for) a number of blocks of
;           Flash memory.
;           Response is different from others: 16-bit CRC transmitted for each block
;           with ETX at the end
; Inputs:  PMADR: pointer to first flash word to be verfied
;          MsgSizeL, H: number of blocks to be verified (a bit oversized block counter :)
; Returns: 0 if ok, 1 if error occured
;
VerifyFlash:
        banksel PMADRH                          ; B3
        moviw   [MsgSizeL - MsgDataBuf0]FSR0    ; B3 setup block counter in easy to reach place
        movwf   ByteCtrL
        moviw   [MsgSizeH - MsgDataBuf0]FSR0
        movwf   ByteCtrH
VerifyFlashNextBlock:
        movlw   DEV_ERASEBLOCK                  ; B3 setup byte counter within the block
        movwf   ByteCtrIB
        ; The host program does not seem to reset the CRC generator for each block,
        ; so let's keep it that way for compatibility
        ; If you want to reset the CRC for each block (which is logical), remove the
        ; comments from the following two lines
;       clrf    CrcL                            ; initialise CRC (for each block)
;       clrf    CrcH
VerifyFlashInBlockLoop:
        call    PAGEMASK(ReadFlashWord)         ; B3 read word from flash
        call    PAGEMASK(AddByteToCrc)          ; B3 update CRC
        movf    FSR1H, W                        ; B3 read hi bits
        call    PAGEMASK(AddByteToCrc)          ; B3 update CRC
        incf    PMADRL, f                       ; B3 increment PMADR
        btfsc   STATUS, STATUS_Z_POSN           ; B3
        incf    PMADRH, f                       ; B3
        decf    ByteCtrIB, f                    ; B3 decrement byte in block counter
        btfss   STATUS, STATUS_Z_POSN           ; B3
        bra     VerifyFlashInBlockLoop          ; B3 loop around for all bytes in block
        call    PAGEMASK(SendChecksum)          ; B3 send checksum of block
        decf    ByteCtrL, f                     ; B3 decrement and check block counter
        btfss   STATUS, STATUS_Z_POSN           ; B3
        bra     VerifyFlashNextBlock            ; B3 if lo byte not zero, go to next block
        movf    ByteCtrH, W                     ; B3 is ByteCtrH == 0?
        btfsc   STATUS, STATUS_Z_POSN           ; B3
        bra     VerifyFlashEnd                  ; B3 if ByteCtrH zero jump to finish
        decf    ByteCtrH, f                     ; B3 otherwise decrement hi byte counter
        bra     VerifyFlashNextBlock            ; B3 and go to next block
VerifyFlashEnd:
        goto    SendETX                         ; B3 finish packet with ETX and return

; *****************************************************************************
; Command - Erase a number of block of bytes from flash
; Input:   FSR0: pointer to MsgDataBuf0 (just after command code)
;          PMADR: pointer to an address withing the *last* block to be erase
;          Bank selected: B3
; Returns: 0 if ok, 1 if error occured
;
EraseFlash:
        moviw   [MsgSizeL - MsgDataBuf0]FSR0    ; B3 get erase block counter
        btfsc   STATUS, STATUS_Z_POSN           ; B3 non-zero, keep erasing more data
        bra     EraseFlashSendResp              ; B3 all erase done, send response and end
        decf    WREG, W                         ; B3 decrement erase block counter
        movwi   [MsgSizeL - MsgDataBuf0]FSR0    ; B3 store back block counter
        bcf     PMCON1, PMCON1_CFGS_POSN        ; B3 not configuration space
        bsf     PMCON1, PMCON1_FREE_POSN        ; B3 specify erase operation
        bsf     PMCON1, PMCON1_WREN_POSN        ; B3 enable writes
        movlw   0x55                            ; B3 start unlock sequence to initiate write
        movwf   PMCON2                          ; B3 write 55H
        movlw   0xaa
        movwf   PMCON2                          ; B3 write AAH
        bsf     PMCON1, PMCON1_WR_POSN          ; B3 set WR bit to erase
        nop                                     ; B3 two NOPs are mandatory
        nop                                     ; B3 processor stalls until erase completed
        bcf     PMCON1, PMCON1_WREN_POSN        ; B3 disable writes (block erased)
        movlw   DEV_ERASEBLOCK                  ; B3 adjusting PMADR downwards with blocksize for next block
        subwf   PMADRL, f                       ; B3 decrease low byte
        btfss   STATUS, STATUS_C_POSN           ; B3 skip high byte does not change
        decf    PMADRH, f                       ; B3 otherwise decrease high byte
        bra     EraseFlash                      ; B3 loop for all blocks

EraseFlashSendResp:
        moviw   [MsgCmd - MsgDataBuf0]FSR0      ; Bx load command byte
        call    PAGEMASK(SendEscapeByte)        ; Bx Send only the command byte (acknowledge packet)
        call    PAGEMASK(AddByteToCrc)          ; Bx
        call    PAGEMASK(SendChecksum)          ; Bx finish packet and return
        goto    SendETX

; *****************************************************************************
; Command - Write N blocks of bytes to Flash and send acknowledgement of
;           command code, CRC and ETX to serial port
; Inputs:  PMADR: pointer to first flash word to write
;          MsgSizeL: number of blocks to be read
;          MsgDataBuf1: pointer to start of buffer, containing MsgSizeL blocks of word
; Returns: 0 if ok, 1 if error occured
;
WriteFlash:
        banksel PMADRH                          ; B3
        movlw   low(MsgSizeL)                   ; B3 setup block counter in easy to reach place
        movwf   FSR0L                           ; B3 FSR0 points to number of blocks (FSR0H was set up already)
        moviw   FSR0++                          ; B3 get number of blocks to write
        movwf   ByteCtrH                        ; B3 store it to block counter
WriteFlashLoop:
        movf    ByteCtrH,W                      ; B3 check if we are done
        btfsc   STATUS, STATUS_Z_POSN           ; B3 skip if not
        bra     WriteFlashSendResp              ; B3 otherwise send response and finish
        decf    ByteCtrH,f                      ; B3 and decrement outer block counter
WriteFlashExecute:
        bcf     PMCON1, PMCON1_CFGS_POSN        ; B3 not configuration space
        bsf     PMCON1, PMCON1_WREN_POSN        ; B3 enable writes
        bsf     PMCON1, PMCON1_LWLO_POSN        ; B3 only load write latches (do not write yet)

WriteFlashLatchLoop:
        moviw   FSR0++                          ; B3 load first byte to lower nibble of data word
        movwf   PMDATL
        moviw   FSR0++                          ; B3 and load next to higher nibble
        movwf   PMDATH
        movf    PMADRL,W                        ; B3 check if we are on the last address in block
        xorlw   DEV_WRITEBLOCK-1
        andlw   DEV_WRITEBLOCK-1
        btfsc   STATUS, STATUS_Z_POSN
        bra     WriteFlashStartWrite            ; B3 exit from latch loop if last address in block
        movlw   0x55                            ; B3 start unlock sequence to initiate write
        movwf   PMCON2                          ; B3 write 55H
        movlw   0xaa
        movwf   PMCON2                          ; B3 write AAH
        bsf     PMCON1, PMCON1_WR_POSN          ; B3 set WR bit to write
        nop                                     ; B3 two NOPs are mandatory
        nop                                     ; B3 to write the latches
        incf    PMADRL,f                        ; B3 inrement program memory address
        bra     WriteFlashLatchLoop

WriteFlashStartWrite:
        bcf     PMCON1, PMCON1_LWLO_POSN        ; B3 no more loading of latches, start write
        movlw   0x55                            ; B3 start unlock sequence to initiate write
        movwf   PMCON2                          ; B3 write 55H
        movlw   0xaa
        movwf   PMCON2                          ; B3 write AAH
        bsf     PMCON1, PMCON1_WR_POSN          ; B3 set WR bit to write
        nop                                     ; B3 two NOPs are mandatory
        nop                                     ; B3 to write the latches
        bcf     PMCON1, PMCON1_WREN_POSN        ; B3 disable writes
        bra     WriteFlashLoop                  ; B3 go for next block

WriteFlashSendResp:
        goto    SendAcknowledge                 ; B3 zero, exit read loop and send end of packet


; *****************************************************************************
; Command: Run application (after programming)
; This routine does not return, but passes control to application following a
; stack pointer reset (to avoid possible stack overflow in application)
;
RunApplication:
        ; restore used registers to power-on reset defaults
        banksel SPBRGL                          ; B3
        clrf    SPBRGL                          ; B3
        clrf    SPBRGH                          ; B3
        clrf    BAUDCON                         ; B3
        clrf    TXSTA                           ; B3
        clrf    RCSTA                           ; B3
        banksel RXPPS                           ; B28
        movlw   0x15                            ; B28
        movwf   RXPPS                           ; B28
        banksel RA4PPS                          ; B29
        clrf    RA4PPS                          ; B29
        banksel ANSELA                          ; B3
        movlw   0x17                            ; B3
        movwf   ANSELA                          ; B3
        banksel T1CON                           ; B0
        clrf    T1CON                           ; B0
        clrf    PIR1                            ; B0
        banksel STKPTR                          ; B31 clear the stack first
        movlw   0x1f                            ; B31 default value for stack pointer after reset
        movwf   STKPTR                          ; B31
        movlw   0                               ; B31 clear W reg
        movwf   FSR0L                           ; B31 clear FSR registers
        movwf   FSR0H
        movwf   FSR1L
        movwf   FSR1H
        movwf   BSR                             ; B0 set bank to 0
        movwf   INTCON                          ; B0 and also to INTCON
        movlp   high(ResVect)                   ; B0
        goto    ResVect                         ; B0

; *****************************************************************************
; 16-bit CCITT CRC routine. Adds WREG byte to existing CRC checksum (CrcH:CrcL)
; Input:   W: byte to be added
; Returns: CrcH:CrcL memory registers updated, WREG destroyed
;
AddByteToCrc:
        xorwf   CrcH, w                         ; Bx Pre:  HHHH hhhh     WREG =      IIII iiii
        movwf   CrcTemp                         ; Bx
        movf    CrcL, w                         ; Bx Pre:  LLLL llll     CrcH =      LLLL llll
        movwf   CrcH                            ; Bx
        movf    CrcTemp, w                      ; Bx
        movwf   CrcL                            ; Bx Pre:  IIII iiii     CrcL =      IIII iiii
        swapf   CrcL, w                         ; Bx Pre:  IIII iiii     WREG =      iiii IIII
        andlw   0x0F                            ; Bx Pre:  iiii IIII     WREG =      0000 IIII
        xorwf   CrcL, f                         ; Bx Pre:  IIII iiii     CrcL =      IIII jjjj
        swapf   CrcL, w                         ; Bx Pre:  IIII jjjj     WREG =      jjjj IIII
        andlw   0xF0                            ; Bx Pre:  jjjj IIII     WREG =      jjjj 0000
        xorwf   CrcH, f                         ; Bx Pre:  LLLL llll     CrcH =      MMMM llll
        swapf   CrcL, f                         ; Bx Pre:  IIII jjjj     WREG =      jjjj IIII
        bcf     STATUS, STATUS_C_POSN           ; Bx
        rlf     CrcL, w                         ; Bx Pre:  jjjj IIII     WREG =      jjjI IIIj
        btfsc   STATUS, STATUS_C_POSN           ; Bx
        addlw   1
        xorwf   CrcH, f                         ; Bx Pre:  MMMM llll     CrcH =      XXXN mmmm
        andlw   11100000B                       ; Bx Pre:  jjjI IIIj     WREG =      jjj0 0000
        xorwf   CrcH, f                         ; Bx Pre:  jjj0 0000     CrcH =      MMMN mmmm
        swapf   CrcL, f                         ; Bx Pre:  IIII jjjj     WREG =      jjjj IIII
        xorwf   CrcL, f                         ; Bx Pre:  MMMN mmmm     CrcL =      JJJI jjjj
        return

; *****************************************************************************
; Waiting for a rising edge in baud detection routine
;
WaitForRise:                                    ; B0
        clrwdt
WaitForRiseLoop:                                ; B0
        btfsc   PIR1, PIR1_TMR1IF_POSN          ; B0 if TMR1 overflowed, we did not get a good baud capture
        return                                  ; B0 abort
        btfsc   PORTA, RXPIN                    ; B0 Wait for a falling edge
        bra    WaitForRiseLoop                  ; B0
WtSR:
        btfss   PORTA, RXPIN                    ; B0 Wait for rising edge
        bra    WtSR                             ; B0
        return

; *****************************************************************************
; Waits for byte to arrive on serial line and writes it to location (FSR0)
;
ReadHostByte:
        banksel PIR1                            ; B0
        clrwdt
ReadHostByteWfChar:
        btfss   PIR1, PIR1_RCIF_POSN            ; B0 Wait for data from RS232
        bra    ReadHostByteWfChar               ; B0
        banksel RCREG                           ; B3
        movf    RCREG, W                        ; B3 Save the data to (FSR0)
        movwi   0[FSR0]                         ; B3
        return

; *****************************************************************************
; Send an acknowledgement packet back
;
; <STX><COMMAND><CRCL><CRCH><ETX>

; Some devices only have config words as FLASH memory. Some devices don't have EEPROM.
; For these devices, we can save code by jumping directly to sending back an
; acknowledgement packet if the PC application erroneously requests them.

SendAcknowledge:                                ;
        movlw   high(MsgBuf)                    ; Bx hi address byte of receive data buf to FSR0H (does not change)
        movwf   FSR0H                           ; Bx
        movlw   low(MsgCmd)                     ; Bx load address command buffer
        movwf   FSR0L                           ; Bx
        moviw   0[FSR0]                         ; Bx get command byte to W
        call    PAGEMASK(SendEscapeByte)        ; Bx Send only the command byte (acknowledge packet)
        call    PAGEMASK(AddByteToCrc)          ; Bx falls through to send checksum
        call    PAGEMASK(SendChecksum)          ; Bx send CRC
        goto    SendETX                         ; Bx send closing ETX

; *****************************************************************************
; Send pre-calculated checksum (16-bit CRC) over serial line
;
SendChecksum:                                   ; Bx
        movf    CrcL, W                         ; Bx Send the 16-but CRC
        call    PAGEMASK(SendEscapeByte)        ; Bx
        movf    CrcH, W                         ; Bx
        goto    SendEscapeByte                  ; Bx

; *****************************************************************************
; Send a closing ETX character over serial line
;
SendETX:                                        ; Bx
        movlw   ETX                             ; Bx send end of text condition
        goto    SendHostByte                    ; B3

; *****************************************************************************
; Write a byte to the serial port while escaping control characters with a DLE
; Input: data to be written in W
;
SendEscapeByte:                                 ; Bx
        movwf   TxData                          ; Bx Save the data to TxData

        xorlw   STX                             ; Bx Check for a STX
        btfsc   STATUS, STATUS_Z_POSN           ; Bx Continue test for other ctrl chars if no
        bra    WrDLE                            ; Bx it was STX, continue to write it with escape

        movf    TxData, W                       ; Bx restore data byte
        xorlw   ETX                             ; Bx Check for a ETX
        btfsc   STATUS, STATUS_Z_POSN           ; Bx continue test for other ctrl chars if no
        bra    WrDLE                            ; Bx it was ETX, continue to write it with escape

        movf    TxData, W                       ; Bx restore data byte
        xorlw   DLE                             ; Bx Check for a DLE
        btfss   STATUS, STATUS_Z_POSN           ; Bx if yes, continue to write DLE (escape)
        bra    WrNext                           ; Bx otherwise write it as next ordinary character

WrDLE:
        movlw   DLE                             ; Bx
        call    PAGEMASK(SendHostByte)          ; B3 send DLE escape byte

WrNext:
        movf    TxData, W                       ; B3 Then send original byte to write
;
; *****************************************************************************
; Sending one byte to serial interface
; Input: W: byte to be sent
; Returns: B3
;
SendHostByte:
        banksel PIR1                            ; B0
        clrwdt
SendHostByte1:
        btfss   PIR1, PIR1_TXIF_POSN            ; B0 Write only if TXREG is ready
        bra    SendHostByte1

        banksel TXREG                           ; B3
        movwf   TXREG                           ; B3 Start sending
        return                                  ; B3

; *****************************************************************************
; Read word from flash
; Inputs:  PMADR: pointer to flash word to read
; Bank:    B3 required at entry
; Returns: Read word from given location: W: low byte, FSR1H high byte
;
ReadFlashWord:
        bcf     PMCON1,PMCON1_CFGS_POSN         ; B3 do not select configuration space
ReadFlashInt:
        bsf     PMCON1,PMCON1_RD_POSN           ; initiate read
        nop                                     ; 2 x nops for timing
        nop
        movf    PMDATH, W                       ; read the word
        movwf   FSR1H                           ; and store HI to FSR0H
        movf    PMDATL, W                       ; and store LO to W
        return

; *****************************************************************************
; Read Configuration, Device ID and User ID from flash
; Inputs:  PMADRL: low byte of pointer to flash configuration data to read (PMADRH is set by the routine)
; Bank:    B3 required at entry
; Returns: Read config data from given location: W: low byte, FSR1H high byte
;
ReadFlashConfig:
        movlw   0x80                            ; B3 set up PMADRH
        movwf   PMADRH
        bsf     PMCON1,PMCON1_CFGS_POSN         ; B3 select configuration space
        goto    ReadFlashInt                    ; B3 Continue at read flash word

; *****************************************************************************
; This is the end of bootloader
; Some safety code...

        movlp   high(ResVect)                   ; safety code
        goto    PAGEMASK(ResVect)               ; this code -should- never be executed, but in case of errant
        goto    PAGEMASK(ResVect)               ; execution or firmware bugs, may protect against accidental writes.

; *****************************************************************************

        END     ResVect
