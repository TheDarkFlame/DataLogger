;*******************************************************************************
;                                                                              *
;    Filename:                                                                 *
;    Date:                                                                     *
;    File Version:                                                             *
;    Author:                                                                   *
;    Company:                                                                  *
;    Description:                                                              *
;    Pin config: RC<0:7>    =LCD D<0:7>	(datapins)                             *          
;		 RB4	    =LCD RS	(reg select data=1 command=0)	       *	           
;		 RB5	    =LCD R/W	(write=1/read=0)		       *		           
;		 RB6	    =LCD E	(enable=1)			       *
;                                                                              *    
;                                                                              *    
;*******************************************************************************
#include "p16F690.inc"

; CONFIG
; __config 0xFFF7
 __CONFIG _FOSC_EXTRCCLK & _WDTE_OFF & _PWRTE_OFF & _MCLRE_ON & _CP_OFF & _CPD_OFF & _BOREN_ON & _IESO_ON & _FCMEN_ON

variables udata
CUR_EEADR   res 1	    ;stores value of current EEADR + 1 (for cycling through EEDAT)
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
BCD_H res 1		    ;register used for bin->BCD (high byte)
BCD_L res 1		    ;register used for bin->BCD (low byte)
TEMP_REG res 1		    ;temporary register
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
    ;outputs: MCU = bank0; LCD= write mode
    BANKSEL TRISC
    BSF TRISC,7			    ;set RC7 as input to test busy flag
    BANKSEL PORTC
    BSF PORTB,5			    ;set to read mode
    BCF PORTB,4			    ;set to command mode
    
    nop
    BSF PORTB,6			    ;pulse enable bit
    nop
    nop
    BCF PORTB,6
    
    BTFSC PORTC,7		    ;test busy flag until cleared
    GOTO $-6
    
    BCF	PORTB,5			    ;set back to write mode
    BANKSEL TRISC
    BCF TRISC,7			    ;set RC7 as output
    BANKSEL PORTB
    
    ENDM
;-------------------------------------------------------------------------------
lcd.write macro
;writes to LCD, moves WREG->LCD DB<0:7>
;does not set RS (RB4) or R/W (RB5)
    BANKSEL PORTC
    MOVWF PORTC	    ;output data to LCD
    BSF PORTB,6	    ;toggle enable on
    nop
    nop
    BCF PORTB,6	    ;toggle enable off
    
    ENDM
    
;-------------------------------------------------------------------------------
eeprom.store macro edata, eadr
 ;does the eeprom store command, simple saves "edata" into "eadr" in eeprom
    BANKSEL eadr
    MOVFW eadr ;move CUR_EEADR into W
    BANKSEL EEADR
    MOVWF EEADR	    ;move W into EEADR
    BANKSEL PORTC
    MOVFW edata	    ;move our data into W
    BANKSEL EEDAT
    MOVWF EEDAT	    ;move W into EEDAT
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
    
    ENDM
;-------------------------------------------------------------------------------
;###END#OF#MACROS###
    
;*******************************************************************************
; MAIN PROGRAM
;*******************************************************************************

MAIN_PROG CODE                      ; let linker place main program
SETUP

    BANKSEL ANSEL
    CLRF ANSEL	    ;set pins to digital
    CLRF ANSELH
;-------------------------------------------------------------------------------
;LCD setup
;-------------------------------------------------------------------------------
;    Pin config: RC<0:8>    =LCD D<0:7>	(dataout)                                        
;		 RB4	    =LCD RS	(reg select)					           
;		 RB5	    =LCD R/W	(write/read)					           
;		 RB6	    =LCD E	(enable)					       
    
    MOVLW 0xff
    BANKSEL TRISC
    MOVWF TRISC			    ;set PORTC as input so we don't mess with initialization
    BANKSEL TRISB
    MOVWF TRISB			    ;set PORTB as input so we don't mess with initialization

    delay_10_ms
    delay_10_ms
    delay_10_ms
    delay_10_ms
    delay_10_ms
    
 
    BANKSEL TRISC
    CLRF TRISC			    ;set PORTC as output
    BANKSEL TRISB
    CLRF TRISB			    ;set PORTB as output
    
    BANKSEL PORTC
    CLRF PORTC			    ;set default state
    CLRF PORTB			    ;set default state
    
    
    MOVLW b'00111000'		    ;001DNFxx D(8bit)=1 N(2line)=1 F(5x11 or 5x8)=0
    MOVWF LCD_BUFFER		    ;move command into buffer
    CALL lcd.command		    ;send command
    
    
    MOVLW b'00001111'		    ;00001DCB Display=on Blinking=off Cursor=off
    MOVWF LCD_BUFFER		    ;move command into buffer
    CALL lcd.command		    ;send command
;-------------------------------------------------------------------------------
 
 ;=============ADC config================
    ;most code here is based on the datasheet's example code
    BANKSEL ADCON1
    MOVLW b'0111000'			;configure conversion speed
    MOVWF ADCON1
    
;==================END=ADC=config======== 
 
    ;delay by 50ms for the LCD to initialize

;TESTCODE
;    MOVLW 'a'	    ;a
;    MOVWF LCD_BUFFER
;    CALL lcd.write
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
    BANKSEL INTCON
    BCF INTCON, GIE ;disable interrupts
    ;EEDATA structure:
    ;0->251 =	log
    ;		252=CUR_EEADR  address accessed
    ;		253=LOG_COUNT  number of entries in storage (max=63)
    
    BANKSEL 0x00    ;move to bank0
    
;check here if the value is 252, if 252, reset to 0
    MOVLW .252
    SUBWF CUR_EEADR,W
    BTFSC STATUS,Z  ;if Z=set (CUR_EEADR=252) then set CUR_EEADR to 0
    CLRF CUR_EEADR  ;set CUR_EEADR=0

    
;store temperature
    eeprom.store CUR_TEMPERATURE, CUR_EEADR    ;writes CUR_TEMPERATURE to CUR_EEADR
    INCF CUR_EEADR,F
    
;store humidity
    eeprom.store CUR_HUMIDITY, CUR_EEADR    ;writes CUR_HUMIDITY to CUR_EEADR
    INCF CUR_EEADR,F
    
;store minutes
    eeprom.store CUR_TIME_L, CUR_EEADR    ;writes CUR_TIME_L to CUR_EEADR
    INCF CUR_EEADR,F
    
;store hours
    eeprom.store CUR_TIME_H, CUR_EEADR    ;writes CUR_TIME_H to CUR_EEADR
    INCF CUR_EEADR,F  
    
;store current address
    MOVLW .252
    MOVWF TEMP_REG
    eeprom.store CUR_EEADR,TEMP_REG    ;writes CUR_EEADR to .252
    
;store number of entries
    MOVLW .253
    MOVWF TEMP_REG
    eeprom.store LOG_COUNT,TEMP_REG    ;writes CUR_EEADR to .252
    
    BSF INTCON, GIE ;enable interrupts
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
    ;for every 10% there is 0.33V	so 1% is 33mV.
    ;the ADC has a minimum step size of 40mv, as such we will use a OP-amp to amplify
    ;40/33=1.21. thus we use a positive amplifier with resistor values of the ratio 5*Rf=R1 
    ; thus 1bit=1%
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
    MOVFW ADRESL
    BANKSEL PORTC
    MOVWF CUR_HUMIDITY   ;stored for saving to eeprom later (and for displaying)

;--------------------------------
;obtain time
;--------------------------------    
    ;get time via i2c from the RTC (and go nuts with glee if it works)
    
    
    RETURN
;###END#OF#CALL###
    
;*******************************************************************************
; LCD write data - writes data in LCD_BUFFER to the LCD 
;*******************************************************************************
lcd.data
    test_busy_flag		    ;outputs in bank0, write mode
    
    BSF PORTB,4			    ;set to data mode
    MOVFW LCD_BUFFER		    ;move LCD buffer into W
    lcd.write			    ;write data to LCD
    
    RETURN
;###END#OF#CALL###
    
;*******************************************************************************
; LCD write command - writes commands to in LCD_BUFFER to the LCD 
;*******************************************************************************
lcd.command

    test_busy_flag		    ;outputs in bank0, write mode
    
    BCF PORTB,4			    ;set to command mode
    MOVFW LCD_BUFFER		    ;move LCD buffer into W
    lcd.write			    ;write data to LCD
    
    RETURN
;###END#OF#CALL###

    
;*******************************************************************************
;	binary -> BCD outputs W as BCD_L and BCD_H
;*******************************************************************************
BINtoBCD
							
    ;DISP_BUFF_H = quotient
    ;DISP_BUFF_L = remainder
    CLRF   BCD_H		;clear our quotient
    MOVWF   BCD_L		;store the number to be divided
    
    MOVLW   0x0A			;load 10 into W

Div10Loop
    SUBWF   BCD_L,1	;subtract 10 from number
    BTFSC   BCD_L,7	;test for -ve
    GOTO    Div10Finished	;if number is -ve, goto finshed
    
    INCF    BCD_H,F
    GOTO    Div10Loop

Div10Finished
    ADDWF   BCD_L,1	;undo last subtraction to get remainder
    
    RETURN
    
;###END#OF#CALL###

    END