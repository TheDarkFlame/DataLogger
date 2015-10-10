;*******************************************************************************
;                                                                              *
;    Filename:                                                                 *
;    Date:                                                                     *
;    File Version:                                                             *
;    Author:                                                                   *
;    Company:                                                                  *
;    Description:                                                              *
;    Pin config: RC<0:7>    =LCD D<0:7>	(datapins)                             *          
;		 RB4	    =LCD RS	(reg select)			       *	           
;		 RB5	    =LCD R/W	(write/read)			       *		           
;		 RB6	    =LCD E	(enable)			       *
;*******************************************************************************
#include "p16F690.inc"

; CONFIG
; __config 0xFFF7
 __CONFIG _FOSC_EXTRCCLK & _WDTE_OFF & _PWRTE_OFF & _MCLRE_ON & _CP_OFF & _CPD_OFF & _BOREN_ON & _IESO_ON & _FCMEN_ON

variables udata
CUR_EEADR   res 1	    ;stores value of last written EEADR
CUR_TEMPERATURE res 1	    ;stores temperature we are currently working with
CUR_HUMIDITY	res 1	    ;stores humidity we are currently working with
CUR_TIME_L  res 1	    ;stores minutes we are currently working with
CUR_TIME_H  res	1	    ;stores hours we are currently working with
LOG_COUNT   res 1	    ;number of logs we have in memory (63 max)
STATE	res 1		    ;state register
LCD_BUFFER res 1	    ;stores whatever we want to put on the LCD right now
;-------x		    ;mode bit 1=playback 0=display
;------x-		    ;button_up state 0=released 1=pressed
;-----x--		    ;button_up state 0=released 1=pressed
D1  res 1		    ;delay register 1
D2  res 1		    ;delay resgiter 2
RES_VECT  CODE    0x0000            ; processor reset vector
    GOTO    SETUP                   ; go to beginning of program

;*******************************************************************************
; MACROS
;*******************************************************************************

;-------------------------------------------------------------------------------    
delay_10_ms macro
    ;goto=2 cyles, decfsz=1 cycle, if zero=2 cycles
    MOVLW .40
    MOVWF D2

;if innerloopnumber=82, total cycles=10082, which is close enough to 10ms
    MOVLW .83	;approx 1/4 ms
    MOVWF D1	
    
    DECFSZ D1,F	;count until D1=0
    GOTO $-1	;keep counting down until
    DECFSZ D2,F	;D1=0, so count down D2, and reset D1 until D2=0
    GOTO $-4	;reset D1
    ENDM
;-------------------------------------------------------------------------------
test_busy_flag macro
    ;waits until busy flag is cleared
    BANKSEL TRISC
    BSF TRISC,7			    ;set RC7 as input to test busy flag
    BANKSEL PORTC
    BTFSC PORTC,7		    ;test busy flag until cleared
    GOTO $-1
    BANKSEL TRISC
    BCF TRISC,7			    ;set RC7 as output
    BANKSEL PORTB
    ENDM
;-------------------------------------------------------------------------------

;###END#OF#MACROS###
    
;*******************************************************************************
; MAIN PROGRAM
;*******************************************************************************

MAIN_PROG CODE                      ; let linker place main program
SETUP
 
;-------------------------------------------------------------------------------
;LCD setup
;-------------------------------------------------------------------------------
;    Pin config: RC<0:8>    =LCD D<0:7>	(dataout)                                        
;		 RB4	    =LCD RS	(reg select)					           
;		 RB5	    =LCD R/W	(write/read)					           
;		 RB6	    =LCD E	(enable)					       
    
    BANKSEL TRISC
    CLRF TRISC			    ;set PORTC as output
    BANKSEL TRISB
    CLRF TRISB			    ;set PORTB as output
    
    test_busy_flag    
    
    BANKSEL PORTB
    BCF PORTB,5			    ;set LCD to write mode
    BCF PORTB,4			    ;set LCD to command mode
    MOVLW b'0000100'		    ;00001DBC Display=on Blinking=off Cursor=off
    BSF PORTB,6			    ;toggle the enable bit with a delay
    test_busy_flag
    BCF PORTB,6
    MOVLW b'00111000'		    ;001DNFxx D(8bit)=1 N(2line)=1 F(5x11 or 5x8)=0
    BSF PORTB,6			    ;toggle the enable bit with a delay
    test_busy_flag
    BCF PORTB,6
;-------------------------------------------------------------------------------
 
 ;=============ADC config================
    ;most code here is based on the datasheet's example code
    BANKSEL ADCON1
    MOVLW b'0111000'			;configure conversion speed
    MOVWF ADCON1
    
;==================END=ADC=config======== 
 
    ;delay by 50ms for the LCD to initialize

TESTCODE
    MOVLW .97	    ;a
    MOVWF LCD_BUFFER
    CALL lcd.write
;endtestcode
START

   
;    CALL SampleData
;    BTFSS STATE,0		    ; are we in playback mode
    ;delay by 10ms
    GOTO START


;*******************************************************************************
; EEPROM.write 
;*******************************************************************************
eeprom.write
    BCF INTCON, GIE ;disable interrupts
    ;EEDATA structure:
    ;0->251 =	log
    ;		252=CUR_EEADR  address accessed
    ;		253=LOG_COUNT  number of entries in storage (max=63)
    
;store temperature
    BANKSEL 0x00    ;move to bank0
    INCF CUR_EEADR,F
    
    ;check here if the value is 252, if 252, reset to 0
    MOVLW .252
    SUBWF CUR_EEADR,W
    BTFSC STATUS,Z  ;if Z=set (CUR_EEADR=252) then set CUR_EEADR to 0
    CLRF CUR_EEADR  ;set CUR_EEADR=0
    
    MOVFW CUR_EEADR ;move CUR_EEADR into W
    BANKSEL EEADR
    MOVWF EEADR	    ;move W into EEADR
    BANKSEL PORTC
    MOVFW CUR_TEMPERATURE
    BANKSEL EEDAT
    MOVWF EEDAT	    ;move our data into EEDAT
    BANKSEL EECON1
    BCF EECON1, EEPGD;data memory
    BSF EECON1, WREN;enable writes
    
    MOVLW 0x55
    MOVWF EECON2    ;write 0x55 to EECON2
    MOVLW 0xAA
    MOVWF EECON2    ;write 0xAA to EECON2
    BSF EECON1,WR   ;begin write
    
    BTFSC EECON1,WR ;check for end of write
    GOTO $-1	    ;loop until clear(Write complete)
    BCF EECON1, WREN;disable writes
    BANKSEL 0x00    ;move back to bank0
    
    
    
;store humidity
    INCF CUR_EEADR,F
    MOVFW CUR_EEADR ;get next location to store value in
    BANKSEL EEADR
    MOVWF EEADR	    ;move W into EEADR
    BANKSEL PORTC
    MOVFW CUR_HUMIDITY
    BANKSEL EEDAT
    MOVWF EEDAT	    ;move our data into EEDAT
    BANKSEL EECON1
    BCF EECON1, EEPGD;data memory
    BSF EECON1, WREN;enable writes
    
    MOVLW 0x55
    MOVWF EECON2    ;write 0x55 to EECON2
    MOVLW 0xAA
    MOVWF EECON2    ;write 0xAA to EECON2
    BSF EECON1,WR   ;begin write
    
    BTFSC EECON1,WR ;check for end of write
    GOTO $-1	    ;loop until clear(Write complete)
    BCF EECON1, WREN;disable writes
    BANKSEL 0x00    ;move back to bank0
    
    
    
;store minutes
    INCF CUR_EEADR,F
    MOVFW CUR_EEADR ;get next location to store value in
    BANKSEL EEADR
    MOVWF EEADR	    ;move W into EEADR
    BANKSEL PORTC
    MOVFW CUR_TIME_L
    BANKSEL EEDAT
    MOVWF EEDAT	    ;move our data into EEDAT
    BANKSEL EECON1
    BCF EECON1, EEPGD;data memory
    BSF EECON1, WREN;enable writes
    
    MOVLW 0x55
    MOVWF EECON2    ;write 0x55 to EECON2
    MOVLW 0xAA
    MOVWF EECON2    ;write 0xAA to EECON2
    BSF EECON1,WR   ;begin write
    
    BTFSC EECON1,WR ;check for end of write
    GOTO $-1	    ;loop until clear(Write complete)
    BCF EECON1, WREN;disable writes
    BANKSEL 0x00    ;move back to bank0


    
;store hours
    INCF CUR_EEADR,F
    MOVFW CUR_EEADR ;get next location to store value in
    BANKSEL EEADR
    MOVWF EEADR	    ;move W into EEADR
    BANKSEL PORTC
    MOVFW CUR_TIME_H
    BANKSEL EEDAT
    MOVWF EEDAT	    ;move our data into EEDAT
    BANKSEL EECON1
    BCF EECON1, EEPGD;data memory
    BSF EECON1, WREN;enable writes
    
    MOVLW 0x55
    MOVWF EECON2    ;write 0x55 to EECON2
    MOVLW 0xAA
    MOVWF EECON2    ;write 0xAA to EECON2
    BSF EECON1,WR   ;begin write
    
    BTFSC EECON1,WR ;check for end of write
    GOTO $-1	    ;loop until clear(Write complete)
    BCF EECON1, WREN;disable writes
    
    
    
;store current address
    BANKSEL EEADR
    MOVLW .252
    MOVWF EEADR	    ;move W into EEADR
    BANKSEL PORTC
    MOVFW CUR_EEADR
    BANKSEL EEDAT
    MOVWF EEDAT	    ;move our data into EEDAT
    BANKSEL EECON1
    BCF EECON1, EEPGD;data memory
    BSF EECON1, WREN;enable writes
    
    MOVLW 0x55
    MOVWF EECON2    ;write 0x55 to EECON2
    MOVLW 0xAA
    MOVWF EECON2    ;write 0xAA to EECON2
    BSF EECON1,WR   ;begin write
    
    BTFSC EECON1,WR ;check for end of write
    GOTO $-1	    ;loop until clear(Write complete)
    BCF EECON1, WREN;disable writes
 

    
;store number of entries
    BANKSEL EEADR
    MOVLW .253
    MOVWF EEADR	    ;move W into EEADR
    BANKSEL PORTC
    MOVFW LOG_COUNT
    BANKSEL EEDAT
    MOVWF EEDAT	    ;move our data into EEDAT
    BANKSEL EECON1
    BCF EECON1, EEPGD;data memory
    BSF EECON1, WREN;enable writes
    
    MOVLW 0x55
    MOVWF EECON2    ;write 0x55 to EECON2
    MOVLW 0xAA
    MOVWF EECON2    ;write 0xAA to EECON2
    BSF EECON1,WR   ;begin write
    
    BTFSC EECON1,WR ;check for end of write
    GOTO $-1	    ;loop until clear(Write complete)
    BCF EECON1, WREN;disable writes
    BANKSEL 0x00    ;move back to bank0
    
    BSF INTCON, GIE ;disable interrupts
    RETURN
;###END#OF#CALL###
    
;*******************************************************************************
; sample data - covers data sampling, and display updating if required 
;*******************************************************************************
sampledata
    
;--------------------------------
;sample temperature
;--------------------------------
    BANKSEL ADCON0
    MOVLW b'10010001'			;right justify
    MOVWF ADCON0			;Vdd,AN4, on

    ;6uS delay
    nop
    nop
    nop
    nop
    nop
    nop
    
    BSF ADCON0,GO	;start conversion
    BTFSC ADCON0,GO	;are we done the conversion?
    GOTO $-1		;try again if not

    BCF STATUS,C
    BANKSEL ADRESL
    INCF ADRESL,F	;divide by 2, rounding up
    RRF ADRESL,W	;conversion based on Vcc being about 5V
    BANKSEL PORTC
    MOVWF CUR_TEMPERATURE   ;stored for saving to eeprom later (and for displaying)

;--------------------------------
;sample humidity
;--------------------------------
    ;humidity is a vaguely linear value, ranging from 1V(30%) to 3V(90%)
    ;for every 10% there is 0.33V so 1% is 33mV.
    ;thus we require ADC conversions to have minimum step of at most 33mV (preferably half this)
    ;desirable step size = 16mv per step.
    
;--------------------------------
;obtain time
;--------------------------------    
    ;get time via i2c from the RTC (and go nuts with glee if it works)
    
    
    RETURN
;###END#OF#CALL###
    
;*******************************************************************************
; LCD write function - writes data in LCD_BUFFER to the LCD 
;*******************************************************************************
lcd.write

    test_busy_flag
    
    BSF PORTB,4	    ;select transfer display data
    MOVFW LCD_BUFFER
    MOVWF PORTC	    ;output data to LCD
    BSF PORTB,6	    ;toggle enable on
    test_busy_flag
    BCF PORTB,7	    ;toggle enable off
    
    RETURN
;###END#OF#CALL###
    END