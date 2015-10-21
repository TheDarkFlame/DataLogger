;*******************************************************************************
;                                                                              *
;    Filename:                                                                 *
;    Date:                                                                     *
;    File Version:                                                             *
;    Author:                                                                   *
;    Company:                                                                  *
;    Description:                                                              *
;    Pin config: RC<0:3>    =LCD D<4:7>	(datapins)                             *          
;		 RC4	    =LCD RS	(reg select data=1 command=0)	       *	           
;		 RC6	    =LCD R/W	(write=0/read=1)		       *		           
;		 RC7	    =LCD E	(enable=1)			       *
;                RB4	    =Temperature sensor                                *    
;                RB5        =Humidity sensor                                   *    
;                RA0        =ButtonDown	                                       *    
;                RA1        =ButtonUp	                                       *    
;                RA2        =RTC CE pin	                                       *    
;                RC1        =RTC IO pin	                                       *    
;                RC0        =RTC SCLK pin	                               *    
;   note that RC0 and RC1 are dual bound, but the bindings will never conflict *    
;*******************************************************************************
#include "p16F690.inc"

; CONFIG
; __config 0xFFF7
 __CONFIG _FOSC_EXTRCCLK & _WDTE_OFF & _PWRTE_OFF & _MCLRE_ON & _CP_OFF & _CPD_OFF & _BOREN_ON & _IESO_ON & _FCMEN_ON

 
 
;DEFINITIONS
;PORTC: eg BSF PORTC,LCD_E
#define	LCD_RS	4		; 0 = Command, 1 = Data
#define LCD_RW	6		; 0 = Write, 1 = Read
#define LCD_E	7		
#define LCD_BF	3		; LCD_D7 = BF in read mode
#define Button_Down PORTA,0
#define Button_Up PORTA,1
#define RTC_CE PORTA,2
#define RTC_IO PORTC,1
#define RTC_SCLK PORTC,0
 
;REGISTERS
variables udata
READ_EEADR   res 1	    ;stores value of current EEADR + 1 (for cycling through EEDAT)
READ_TEMPERATURE res 1	    ;stores last temperature we read from the sensor
READ_HUMIDITY	res 1	    ;stores last humidity we read from the sensor
READ_TIME_L  res 1	    ;stores minutes we last read from the RTC
READ_TIME_H  res 1	    ;stores hours we last read from the RTC

CUR_EEADR  res 1	    ;working eeadr (eg for displaying)
CUR_TEMPERATURE res 1	    ;working temperature
CUR_HUMIDITY	res 1	    ;working humidity
CUR_TIME_L  res 1	    ;working minutes
CUR_TIME_H  res 1	    ;working hours
  
LOG_COUNT   res 1	    ;number of logs we have in memory (63 max)
STATE	res 1		    ;state register
RTC_BUFFER res 1	    ;working register to aid getting serial IO  
LCD_BUFFER res 1	    ;stores whatever we want to put on the LCD right now
;-------x		    ;mode bit 1=playback 0=display
;------x-		    ;button_up state 0=released 1=pressed
;-----x--		    ;button_up state 0=released 1=pressed
D1  res 1		    ;delay register 1
D2  res 1		    ;delay resgiter 2
TD1 res 1		    ;timer delay register 1
TD2 res 1		    ;timer delay register 2
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

    
;-------------------------------------------------------------------------------
eeprom.store macro edata, eadr
 ;does the eeprom store command, simple saves "edata" into "eadr" in eeprom
    BANKSEL eadr
    MOVFW eadr ;move READ_EEADR into W
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
; ISRs
;*******************************************************************************
ISR       CODE    0x0004    ;interrupts start at 0x0004
    BTFSC INTCON,T0IF	 				 
    CALL TMR0_ISR	    ; check if ISR is for TMR0	 
    RETFIE
;end ISR
    
    
TMR0_ISR
    
    
    
    BCF INTCON,T0IF	    ;clear interrupt flag
    RETURN
;###END#OF#ISRs###
    
;*******************************************************************************
; MAIN PROGRAM
;*******************************************************************************
MAIN_PROG CODE		    ; let linker place main program
SETUP

    BANKSEL ANSEL
    CLRF ANSEL		    ;set pins to digital
    CLRF ANSELH
    ;set RB4 and RB5 to Analogue
    BSF ANSELH,2	    ;AN10 note ANSELH<0:3> = AN<8:11>
    BSF ANSELH,3	    ;AN11
;-------------------------------------------------------------------------------
;LCD setup
;-------------------------------------------------------------------------------
;    Pin config: RC<0:3>    =LCD D<4:7>	(datapins)                             *          
;		 RC4	    =LCD RS	(reg select data=1 command=0)	       *	           
;		 RC6	    =LCD R/W	(write=0/read=1)		       *		           
;		 RC7	    =LCD E	(enable=1)			       *
    
    MOVLW 0xff
    BANKSEL TRISC
    MOVWF TRISC			    ;set PORTC as input so we don't mess with initialization
    
    delay_10_ms
    delay_10_ms
    delay_10_ms
    delay_10_ms
    delay_10_ms
    
 
    BANKSEL TRISC
    CLRF TRISC			    ;set PORTC as output
    
    BANKSEL PORTC
    CLRF PORTC			    ;set default state
    
    ;currently in 8-bit, move into 4-bit
    
    CALL test_busy_flag		    ;outputs in bank0, write mode
    MOVLW b'00000010'		    ;4-bit mode
    MOVWF PORTC
    BSF PORTC,LCD_E			    ;toggle enable pin
    nop
    nop
    BCF PORTC,LCD_E

    MOVLW b'00001111'		    ;00001DCB Display=on Blinking=on Cursor=on
    MOVWF LCD_BUFFER		    ;move command into buffer
    CALL lcd.command		    ;send command
;-------------------------------------------------------------------------------

;Configure ports for other peripherals
    BANKSEL TRISA
    BSF TRISA,0	    ;button1=input
    BSF TRISA,1	    ;button2=input
    
    BANKSEL TRISB
    BSF TRISB,4	    ;temperature sensor=input
    BSF TRISB,5	    ;humidity sensor=input
    
    
;=============ADC config================
    ;most code here is based on the datasheet's example code
    BANKSEL ADCON1
    MOVLW b'0111000'			;configure conversion speed
    MOVWF ADCON1
    
;==================END=ADC=config======== 
 
    ;delay by 50ms for the LCD to initialize

    ;set the timer up for counting
    BANKSEL OPTION_REG						     
    MOVLW b'00000111'
    MOVWF OPTION_REG 
    ;xxx0xxxx	internal oscilator/4 = clock base
    ;xxxx0xxx	use prescaler
    ;xxxxx111	set prescalter=256
    
    BANKSEL INTCON
    BSF INTCON, T0IE			;enable TMR0 interrupts
    MOVLW .217				; configured for 10ms delay (256=65ms according to stopwatch)
    MOVWF TMR0
    
    
    BSF INTCON, GIE			;enable global interrupts
    
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
    ;		252=READ_EEADR  address accessed
    ;		253=LOG_COUNT  number of entries in storage (max=63)
    
    BANKSEL 0x00    ;move to bank0
    
;check here if the value is 252, if 252, reset to 0
    MOVLW .252
    SUBWF READ_EEADR,W
    BTFSC STATUS,Z  ;if Z=set (READ_EEADR=252) then set READ_EEADR to 0
    CLRF READ_EEADR  ;set READ_EEADR=0

    
;store temperature
    eeprom.store READ_TEMPERATURE, READ_EEADR    ;writes READ_TEMPERATURE to READ_EEADR
    INCF READ_EEADR,F
    
;store humidity
    eeprom.store READ_HUMIDITY, READ_EEADR    ;writes READ_HUMIDITY to READ_EEADR
    INCF READ_EEADR,F
    
;store minutes
    eeprom.store READ_TIME_L, READ_EEADR    ;writes READ_TIME_L to READ_EEADR
    INCF READ_EEADR,F
    
;store hours
    eeprom.store READ_TIME_H, READ_EEADR    ;writes READ_TIME_H to READ_EEADR
    INCF READ_EEADR,F  
    
;store current address
    MOVLW .252
    MOVWF TEMP_REG
    eeprom.store READ_EEADR,TEMP_REG    ;writes READ_EEADR to .252
    
;store number of entries
    MOVLW .253
    MOVWF TEMP_REG
    eeprom.store LOG_COUNT,TEMP_REG    ;writes READ_EEADR to .252
    
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
    MOVLW b'10101101'			;right justify
    MOVWF ADCON0			;Vdd,AN11, on

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
    MOVWF READ_TEMPERATURE   ;stored for saving to eeprom later (and for displaying)

;--------------------------------
;sample humidity
;--------------------------------
    ;humidity is a vaguely linear value, ranging from 1V(30%) to 3V(90%)
    ;for every 10% there is 0.33V	 so 1% is 33mV.
    ;the ADC has a minimum step size of 40mv, as such we will use a OP-amp to amplify
    ;40/33=1.21. thus we use a positive amplifier with resistor values of the ratio 5*Rf=R1 
    ;1bit=1%
    BANKSEL ADCON0
    MOVLW b'10101001'			;right justify
    MOVWF ADCON0			;Vdd,AN10, on

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
    MOVFW ADRESL	;get value from humidity sensor
    BANKSEL PORTC
    MOVWF READ_HUMIDITY   ;stored for saving to eeprom later (and for displaying)
    
;--------------------------------
;obtain time
;--------------------------------    
    ;get time via i2c from the RTC (and go nuts with glee if it works)
    
    
    RETURN
;###END#OF#CALL###
    
;*******************************************************************************
; LCD write data - writes data in LCD_BUFFER to the LCD 
; LCD write command - writes commands to in LCD_BUFFER to the LCD
;
; note only difference is: data mode sets RC4, command mode clears RC4
;*******************************************************************************

;this controls whether it is data or command
lcd.data
    CALL test_busy_flag		    ;outputs in bank0, write mode
    BSF PORTC,LCD_RS			    ;set to data mode
    GOTO $+3
lcd.command
    CALL test_busy_flag		    ;outputs in bank0, write mode
    BCF PORTC,LCD_RS			    ;set to command mode
;part below this is the write part of lcd.data and lcd.command
    
    SWAPF LCD_BUFFER,W		    ;move LCD_BUFFER<4:7> to W<0:3>
    CALL lcd.write			    ;write W<0:3> to LCD<4:7>
    
    MOVFW LCD_BUFFER		    ;move LCD_BUFFER<0:3> to W<0:3>
    CALL lcd.write			    ;write W<0:3> to LCD<4:7>
    
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

    
;*******************************************************************************
;	tests the busy flag and exits when LCD not busy
;*******************************************************************************
test_busy_flag
;    Pin config: RC<0:3>    =LCD D<4:7>	(datapins)                             *          
;		 RC4	    =LCD RS	(reg select data=1 command=0)	       *	           
;		 RC6	    =LCD R/W	(write=0/read=1)		       *		           
;		 RC7	    =LCD E	(enable=1)			       *
    ;waits until busy flag is cleared
    ;outputs: MCU = bank0; LCD= write mode
    BANKSEL TRISC
    BSF TRISC,LCD_BF			    ;set RC3 as input to test busy flag
    BANKSEL PORTC

    
    BSF PORTC,LCD_RW			    ;set to read mode
    BCF PORTC,LCD_RS			    ;set to command mode
    
    nop
    BSF PORTC,LCD_E			    ;pulse enable bit
    nop
    nop
    BCF PORTC,LCD_E
    
    BTFSC PORTC,LCD_BF		    ;test busy flag until cleared
    GOTO $-6
    
    BANKSEL TRISC
    BCF TRISC,LCD_BF			    ;set RC3 as output
    BANKSEL PORTC
    
    BCF PORTC,LCD_RW			    ;back into write mode
    RETURN
    
;###END#OF#CALL###

;*******************************************************************************
;	performs the write actions for the LCD
;*******************************************************************************

lcd.write
;writes to LCD, moves WREG<0:3> -> PORTC<0:3> == LCD DB<4:7>
;does not set RS (RB4) or R/W (RB5)
    BANKSEL TRISC
    CLRF TRISC	    ;set PORTC as output
    BANKSEL PORTC
    MOVWF TEMP_REG  ;save W to be loaded back later
    
    ;bitmask lower nibble of W onto PORTC
    ANDLW 0x0F	    ;clear MSB of W
    IORWF PORTC,F   ;output 1's from W_L to PORTC
    XORLW 0xF0	    ;set MSB of W
    ANDWF PORTC,F   ;output 0's from W_L to PORTC
    
    ;toggle enable
    BSF PORTC,LCD_E	    ;toggle enable on
    nop
    nop
    BCF PORTC,LCD_E	    ;toggle enable off
    
    MOVFW TEMP_REG  ;restore W
    RETURN
    
;###END#OF#CALL###

    
;*******************************************************************************
;	writes values to the RTC
;writes CUR_TIME_H, CUR_TIME_L to RTC
;*******************************************************************************
;needs a re-write to improve efficiency (only write to registers used)
RTC.write
    ;we do a burst write to the RTC
    ;NOTE::: we send in order of LSB->MSB

    BCF RTC_SCLK
    BCF RTC_CE
    
    BANKSEL TRISA
    BCF TRISA,2	;set RTC CE output
    BCF TRISC,0	;set RTC SCLK output
    BCF TRISC,1	;set RTC IO output
    BANKSEL PORTC
    
    BSF RTC_CE			;enable TX
    ;first we must clear the write protect bit
    ;---------------------------------------------------------------------------
    MOVLW b'10010000'		;1,calendar/clock,adr=8,write
    MOVWF RTC_BUFFER
    CALL RTC.write.TX
    ;---------------------------------------------------------------------------
    MOVLW b'00000000'		;write protect=0
    MOVWF RTC_BUFFER
    CALL RTC.write.TX
    ;---------------------------------------------------------------------------
    BCF RTC_CE			;end TX
    
    ;we now can write the the device.
    BSF RTC_CE			;enable TX
    ;---------------------------------------------------------------------------
    MOVLW b'10111110'		;1,calendar/clock,burst mode,write
    MOVWF RTC_BUFFER
    CALL RTC.write.TX
    ;---------------------------------------------------------------------------
    MOVLW b'00000000'		;Clock Halt=0,10seconds=0,seconds=0
    MOVWF RTC_BUFFER
    CALL RTC.write.TX
    ;---------------------------------------------------------------------------
    ;function to get Minutes as BCD xHHHLLLL H=10's L=1's
    MOVFW CUR_TIME_L	    ;move value to write into W
    CALL BINtoBCD	    ;outputs W into BCD_L and BCD_H
    SWAPF BCD_H,F	    ;get BCD_H as HHHH0000
    
    MOVFW BCD_L		    ;start with W=0000LLLL
    IORWF BCD_H,W	    ;now get	W=HHHHLLLL
    ;note that HHHH will always be of the form 0HHH since H<=6
    ;so W= 0HHHLLLL which is what our goal is
    
    MOVWF RTC_BUFFER	    ;0,10minutes,minutes
    CALL RTC.write.TX
    ;---------------------------------------------------------------------------
    ;function to get Hours as BCD 10HHLLLL H=10's L=1's (24hr format)
    MOVFW CUR_TIME_H	    ;move value to write into W
    CALL BINtoBCD	    ;outputs W into BCD_L and BCD_H
    SWAPF BCD_H,F	    ;get BCD_H as HHHH0000
    
    MOVFW BCD_L		    ;start with W=0000LLLL
    IORWF BCD_H,W	    ;now get	W=HHHHLLLL
    ;note that HHHH will always be of the form 00HH since H<=2
    ;so W= 00HHLLLL which is what our goal is
    
    MOVWF RTC_BUFFER	    ;0,0,10hours,hours as desired
    CALL RTC.write.TX
    ;---------------------------------------------------------------------------
    MOVLW b'00000001'		;date=1
    MOVWF RTC_BUFFER
    CALL RTC.write.TX
    ;---------------------------------------------------------------------------
    MOVLW b'00000001'		;month=1
    MOVWF RTC_BUFFER
    CALL RTC.write.TX
    ;---------------------------------------------------------------------------
    MOVLW b'00000001'		;day=1
    MOVWF RTC_BUFFER
    CALL RTC.write.TX
    ;---------------------------------------------------------------------------
    MOVLW b'00000000'		;year=0
    MOVWF RTC_BUFFER
    CALL RTC.write.TX
    ;---------------------------------------------------------------------------
    MOVLW b'10000000'		;write protect=1
    MOVWF RTC_BUFFER
    CALL RTC.write.TX
    ;---------------------------------------------------------------------------
    BCF RTC_CE			;end TX
    
    BANKSEL TRISA
    BSF TRISC,0	;set RTC SCLK input
    BSF TRISC,1	;set RTC IO input
    BANKSEL 0x00
    
    RETURN
    
;###END#OF#CALL###    

;*******************************************************************************
;	serial transmit for RTC read&write
;*******************************************************************************
RTC.write.TX    
;this function is a serial transmit
;it sends a byte 1 by 1 out onto the RTC IO pin
;also controls SCLK behaviour required for the serial TX
    CLRF TEMP_REG
    BSF TEMP_REG,3		;TEMP_REG=8
    
RTC.write.TX.loop    
    BTFSS RTC_BUFFER,0		;test RTC_BUFFER LSB
    GOTO $+3
    BSF RTC_IO			;set RTC_IO if RTC_BUFFER LSB=set
    GOTO $+2
    BCF RTC_IO			;clear RTC_IO if RTC_BUFFER LSB=clear
    
    BSF RTC_SCLK		;toggle RTC serial clock
    nop
    BCF RTC_SCLK
    
    RRF RTC_BUFFER,F		;RTC_BUFFER,1 -> RTC_BUFFER,0
    DECFSZ TEMP_REG,F		;repeat 8 times
    GOTO RTC.write.TX.loop
    RRF RTC_BUFFER,F		;set values as before
    RETURN
;###END#OF#CALL###    
    
;*******************************************************************************
;	read current values on the RTC
;*******************************************************************************
RTC.read    
    
    BANKSEL TRISA
    BCF TRISA,2	;set RTC CE output
    BCF TRISC,0	;set RTC SCLK output
    BCF TRISC,1	;set RTC IO output
    BANKSEL PORTC
    
    ;---------------------------------------------------------------------------
    BSF RTC_CE			;enable TX/RX
    MOVLW b'10000101'		;1,calendar/clock,adr=2,read
    MOVWF RTC_BUFFER
    CALL RTC.write.TX		;send address and specify read mode
    
    BANKSEL TRISA
    BSF TRISC,0	;set RTC SCLK input
    BSF TRISC,1	;set RTC IO input
    BANKSEL PORTC
    
    CALL RTC.read.RX		;receive data in RTC minutes reg into RTC_BUFFER
    MOVFW RTC_BUFFER
    MOVWF READ_TIME_L
    BCF RTC_CE			;end TX/RX
    ;---------------------------------------------------------------------------
    BSF RTC_CE			;enable TX/RX

    BANKSEL TRISA
    BCF TRISC,0	;set RTC SCLK output
    BCF TRISC,1	;set RTC IO output
    BANKSEL PORTC

    MOVLW b'10000111'		;1,calendar/clock,adr=3,read
    MOVWF RTC_BUFFER
    CALL RTC.write.TX		;send address and specify read mode
    
    BANKSEL TRISA
    BSF TRISC,0	;set RTC SCLK input
    BSF TRISC,1	;set RTC IO input
    BANKSEL PORTC
    
    CALL RTC.read.RX		;receive data in RTC minutes reg into RTC_BUFFER
    MOVFW RTC_BUFFER
    MOVWF READ_TIME_H
    BCF RTC_CE			;end TX/RX
    ;---------------------------------------------------------------------------
    
    
    
    
    RETURN
;###END#OF#CALL###    
;*******************************************************************************
;	serial receive for RTC read
;*******************************************************************************
RTC.read.RX    
;this function is a serial receive
;it gets in a byte 1 by 1 on the the RTC IO pin
;also controls SCLK behaviour required for the serial RX
    ;RTC_IO = PORTC,1
    ;RTC_SCLK = PORTC,0
    
    CLRF TEMP_REG
    BSF TEMP_REG,3		;TEMP_REG=8
    
RTC.write.RX.loop    
    BSF RTC_SCLK		;toggle RTC serial clock high
    
    BTFSS RTC_IO		;test RTC_BUFFER LSB
    GOTO $+3
    BSF RTC_BUFFER,0			;set RTC_IO if RTC_BUFFER LSB=set
    GOTO $+2
    BCF RTC_BUFFER,0			;clear RTC_IO if RTC_BUFFER LSB=clear
    
    BCF RTC_SCLK		;toggle RTC serial clock low
    
    
    RLF RTC_BUFFER,F		;RTC_BUFFER,0 -> RTC_BUFFER,1
    DECFSZ TEMP_REG,F		;repeat 8 times
    GOTO RTC.write.RX.loop
    RLF RTC_BUFFER,F
    RETURN
;###END#OF#CALL###      
    
    
    
    
    END