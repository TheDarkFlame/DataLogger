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
;		 RC6	    =					       *		           
;		 RC7	    =LCD E	(enable=1)			       *
;                RB4	    =Temperature sensor                                *    
;                RB5        =Humidity sensor                                   *    
;                RA0        =ButtonDown	                                       *    
;                RA1        =ButtonUp	                                       *    
;                RB7        =ButtonMode		                               *    
;                RA2        =RTC CE pin	                                       *    
;                RC5        =RTC IO pin	                                       *    
;                RC0        =RTC SCLK pin	                               *    
;   note that RC0 and are dual bound, but the bindings will never conflict *    
;*******************************************************************************
#include "p16F690.inc"
    errorlevel -302
; CONFIG
; __config 0xFFF7
 __CONFIG _FOSC_EXTRCCLK & _WDTE_OFF & _PWRTE_OFF & _MCLRE_ON & _CP_OFF & _CPD_OFF & _BOREN_ON & _IESO_ON & _FCMEN_ON

 
 
;DEFINITIONS
;PORTC: eg BSF PORTC,LCD_E
#define	LCD_RS	4		; 0 = Command, 1 = Data
#define LCD_RW	6		; 0 = Write, 1 = Read
#define LCD_E	7		
#define Button_Down PORTA,0
#define Button_Up PORTA,1
#define Button_Mode PORTA,2
#define Button_Down.State STATE,0
#define Button_Up.State STATE,1
#define Button_Mode.State STATE, 2
#define RTC_CE PORTA,2
#define RTC_IO PORTC,5
#define RTC_SCLK PORTC,0
#define StateBit1 STATE,3
#define StateBit2 STATE,4
#define TimeoutDelay .3
#define DownButtonPending STATE,5
#define UpButtonPending STATE,6
#define ModeButtonPending STATE,7
 
;REGISTERS
variables udata
READ_EEADR   res 1	    ;stores value of latest written EEADR + 1 (for cycling through EEDAT)
READ_TEMPERATURE res 1	    ;stores last temperature we read from the sensor
READ_HUMIDITY	res 1	    ;stores last humidity we read from the sensor
READ_TIME_L  res 1	    ;stores minutes we last read from the RTC
READ_TIME_H  res 1	    ;stores hours we last read from the RTC
READ_POWERSUPPLY res 1	    ;last read power level of the device
  
CUR_EEADR  res 1	    ;working eeadr (eg for displaying)
CUR_TEMPERATURE res 1	    ;working temperature
CUR_HUMIDITY	res 1	    ;working humidity
CUR_TIME_L  res 1	    ;working minutes
CUR_TIME_H  res 1	    ;working hours
CUR_LOG_ENTRY res 1	    ;stores which log entry we are at (of the 63 max)
  
LOG_COUNT   res 1	    ;number of logs we have in memory (63 max)
RTC_BUFFER res 1	    ;working register to aid getting serial IO  
LCD_BUFFER res 1	    ;stores whatever we want to put on the LCD right now
LCD_POSITION res 1	    ;for writing to a specific LCD position defined in code
STATE res 1		    ;state register
;-------x		    ;button_up state 0=released 1=pressed
;------x-		    ;button_down state 0=released 1=pressed
;-----x--		    ;button_mode state 0=released 1=pressed
;----x---		    ;FSM current state bit 1
;---x----		    ;FSM current state bit 2
;--x-----		    ;button_up press read
;-x------		    ;button_down press read
;x-------		    ;button_mode state read
;press read indicates a pending read on the button, cleared if no press or if read 
STATE2 res 1		    ;a second state register
;-------x		    ;queue LCD update -> if set, LCD needs update 
 
D1  res 1		    ;delay register 1
D2  res 1		    ;delay resgiter 2
TD1 res 1		    ;timer delay register 1(for 1 sec)
TD2 res 1		    ;timer delay register 2(for 1 min)
TD3 res 1		    ;timer delay register 3(for 5 min)
BCD_H res 1		    ;register used for bin->BCD (high byte)
BCD_L res 1		    ;register used for bin->BCD (low byte)
TEMP_REG res 1		    ;temporary register
TEMP_REG2 res 1		    ;temporary register
CHAR_L res 1		    ;stores lowbyte ascii characters for write functions
CHAR_H res 1		    ;stores highbyte ascii characters for write functions
Button_Up_Counter res 1	    ;used for tracking up button state
Button_Down_Counter res 1	    ;used for tracking down button state
Button_Mode_Counter res 1	    ;used for tracking mode button state
Input1 res 1		    ;used as a generic input for multiple input functions
Input2 res 1		    ;used as a generic input for multiple input functions
W_TEMP res 1		    ;context saving 
State_Timeout res 1	    ;a timeout for state transitions
 
RES_VECT  CODE    0x0000            ; processor reset vector
    GOTO    SETUP                   ; go to beginning of program

;*******************************************************************************
; MACROS
;*******************************************************************************
  
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
    MOVWF W_TEMP	    ;context saving
    
    BTFSC INTCON,T0IF	 				 
    CALL TMR0_ISR	    ; check if ISR is for TMR0	 
    
    MOVFW W_TEMP	    ;context saving
    RETFIE
;end ISR
    
    
TMR0_ISR	;happens once every 10ms
;    GOTO TMR0_ISR.end
    ;------10MS ISR content start
    CALL Poll_Button_Up
    CALL Poll_Button_Down
    CALL Poll_Button_Mode
    CALL UpdateState
    ;------10MS ISR content end
    DECFSZ TD1,F	    ;count down from 100 every 10ms
    GOTO $+2
    GOTO TMR0_ISR.1sec	    ;check for 1second
    GOTO TMR0_ISR.end
    
TMR0_ISR.1sec		    ;happens once every 1sec
    MOVLW .100		    ;reset count for 1 sec
    MOVWF TD1
    ;------1SEC ISR content start
    DECF State_Timeout,F    ;for state transitioning
    CALL SampleData	    ;get data
    CALL lcd.print.lastsample	    ;display data
    ;------1SEC ISR content end    
    DECFSZ TD2,F	    ;count down from 60 every 1 sec
    GOTO $+2
    GOTO TMR0_ISR.1min	    ;if zero, goto ISR.1min
    GOTO TMR0_ISR.end	    ;else goto ISR.end
    
TMR0_ISR.1min		    ;happens every 1 min
    MOVLW .60		    ;reset count for 1 min
    MOVWF TD2
    ;------1MIN ISR content start
    
    ;------1MIN ISR content end
    DECFSZ TD3,F	    ;count down from 5 every 1 min
    GOTO $+2
    GOTO TMR0_ISR.5min	    ;if zero, goto ISR.5min
    GOTO TMR0_ISR.end	    ;else goto ISR.end
    
TMR0_ISR.5min		    ;happens every 5 min
    MOVLW .5		    ;reset count for 5 min
    MOVWF TD3
    ;------1MIN ISR content start
    CALL SampleData	    ;get data
    CALL lcd.print.lastsample	    ;display data
;    CALL eeprom.write	    ;store data
    ;------1MIN ISR content end
    GOTO  TMR0_ISR.end	    ;goto ISR.end
    
TMR0_ISR.end    
    BCF INTCON,T0IF	    ;clear interrupt flag
    MOVLW .217				; configured for 10ms delay (256=65ms according to stopwatch)
    MOVWF TMR0
    RETFIE
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
;		 RC6	    =		       *		           
;		 RC7	    =LCD E	(enable=1)			       *
    
    MOVLW 0xff
    BANKSEL TRISC
    MOVWF TRISC			    ;set PORTC as input so we don't mess with initialization
    
   CALL delay_10_ms
   CALL delay_10_ms
   CALL delay_10_ms
   CALL delay_10_ms
   CALL delay_10_ms
    
 
    BANKSEL TRISC
    CLRF TRISC			    ;set PORTC as output
    
    BANKSEL PORTC
    CLRF PORTC			    ;set default state
    
    ;currently in 8-bit, move into 4-bit
    
    MOVLW b'00000010'		    ;4-bit mode
    MOVWF PORTC
    BSF PORTC,LCD_E			    ;toggle enable pin
    nop
    nop
    BCF PORTC,LCD_E

    CALL delay_10_ms
    MOVLW b'00101111'		    ;4bit, 2 lines,5x11dots
    MOVWF LCD_BUFFER		    ;move command into buffer
    CALL lcd.command		    ;send command
    
    CALL delay_10_ms
    MOVLW b'00001111'		    ;00001DCB Display=on Blinking=on Cursor=on
    MOVWF LCD_BUFFER		    ;move command into buffer
    CALL lcd.command		    ;send command
;-------------------------------------------------------------------------------

;Configure ports for other peripherals
    BANKSEL TRISA
    BSF TRISA,0	    ;button1=input
    BSF TRISA,1	    ;button2=input
    BSF TRISA,2	    ;button3=input
    
    BANKSEL TRISB
    BSF TRISB,4	    ;temperature sensor=input
    BSF TRISB,5	    ;humidity sensor=input
    
    
;=============ADC config================
    ;most code here is based on the datasheet's example code
    BANKSEL ADCON1
    MOVLW b'01110000'			;configure conversion speed
    MOVWF ADCON1
    
;==================END=ADC=config======== 
 
    BANKSEL STATE
    CLRF STATE
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
    
    BANKSEL LOG_COUNT
    CLRF LOG_COUNT
    
    BANKSEL TD1
    ;set all ISR values to 1 so we enter the highest ISR on first interrupt
    MOVLW .1
    MOVWF TD1	;set delay1=100x10ms	(so 1 second intervals)
    MOVLW .1
    MOVWF TD2	;set delay2=60x1s	(so 1 minute intervals)
    MOVLW .1
    MOVWF TD3	;set delay3=5x1min	(so 5 minute intervals)
    
    CALL lcd.clear
    BSF INTCON, GIE			;enable global interrupts

    
    
    
    
START
    GOTO START



;*******************************************************************************
; Update State 
;*******************************************************************************
UpdateState
;    STATE	res 1	    ;state register
;-------x		    ;button_up state 0=released 1=pressed
;------x-		    ;button_down state 0=released 1=pressed
;-----x--		    ;button_mode state 0=released 1=pressed
;----x---		    ;FSM current state bit 1
;---x----		    ;FSM current state bit 2
;--x-----		    ;button_up press read
;-x------		    ;button_down press read
;x-------		    ;button_mode state read
    
    ;States:
    ;s1	display mode	-	current			    SB1=0 SB2=0
    ;s2	display mode	-	log			    SB1=0 SB2=1
    ;s3	reset mode	-	hours			    SB1=1 SB2=0
    ;s4	reset mode	-	minutes			    SB1=1 Sb2=1
    
    ;if (s1<00>){
    ;	up/down press to move to s2
    ;	mode select press to move to s3
    ;	}
    ;if (s2<01>){
    ;	up/down press to scroll through log
    ;	timeout for transition back to current
    ;	}
    ;if (s3<10>){
    ;	up/down press for changing hours
    ;	mode select press to go to s4
    ;	timeout to cancel reset and go back to s1
    ;	}
    ;if (s4<11>){
    ;	up/down press for changing minutes
    ;	mode select press to reset and go to s1
    ;	timeout to cancel reset and go back to s1
    ;	}
    
    
    ;reset timeout if a button is depressed(state=set)
    MOVLW TimeoutDelay	    
    BTFSC Button_Down.State ;do next if set
	MOVWF State_Timeout
    BTFSC Button_Up.State   ;do next if set
	MOVWF State_Timeout
    BTFSC Button_Mode.State ;do next if set
	MOVWF State_Timeout
    
    
    BTFSS StateBit1
    GOTO UpdateState.SB1clear
    GOTo UpdateState.SB1set
    
UpdateState.SB1clear
    BTFSS StateBit2
    GOTO UpdateState.SB1clear.SB2clear
    GOTO UpdateState.SB1clear.SB2set
    
UpdateState.SB1set
    BTFSS StateBit2
    GOTO UpdateState.SB1set.SB2clear
    GOTO UpdateState.SB1set.SB2set

;-------------------------------------------------------------------------------    
UpdateState.SB1clear.SB2clear	;aka s1
    ; s1 tasks:
    
    ;...........................................................................
    ;up/down press to move to s2
    ;...........................................................................
    ;DOWN BUTTON TEST

    BTFSS DownButtonPending		    ;if pressed transition to s2 
    GOTO $+8	; not pressed, don't transition
    
    ;button pressed
    BCF DownButtonPending		    ;register that button press was read
    
    
    MOVFW READ_EEADR			    ;the address we will store next sample to
    MOVWF CUR_EEADR			    ;set our new display value as sample value
    
    MOVFW LOG_COUNT			    ;set our mem read value = latest log,
    MOVWF CUR_LOG_ENTRY			    ; as from this state we always start at latest log
    
    CALL CountAdrDown

    BSF StateBit2			    ;move to s2
    
    
    ;UP BUTTON TEST
    BTFSS UpButtonPending		    ;if pressed transition to s2 
    GOTO $+4	; not pressed, don't transition
    
    ;button pressed
    BCF UpButtonPending			    ;register that button press was read
    CALL CountAdrUp			    ;count up 1 number
    
    
    BSF StateBit2			    ;move to s2
    
    
    ;...........................................................................
    ;	mode select press to move to s3
    ;...........................................................................
    ;MODE BUTTON PRESSED
    BTFSS ModeButtonPending		    ;if pressed do next
    GOTO $+3   ;not pressed, don't transition
    
    ;button pressed
    BCF ModeButtonPending		    ;register that button is pressed
    BSF StateBit1			    ;goto s3
    
    RETURN

    
    
    
;-------------------------------------------------------------------------------    
UpdateState.SB1clear.SB2set	;aka s2
    ;s2 tasks:
    
    ;...........................................................................
    ;	timeout for transition back to current
    ;...........................................................................
    
    ;TIMEOUT TEST
    MOVFW State_Timeout
    BTFSS STATUS,Z
    GOTO $+3			;if not zero skip over
    
    BCF StateBit2		;if zero, goto s1
    RETURN			;no need to check anything more
    
    ;...........................................................................
    ;	up/down press to scroll through log
    ;...........................................................................
    ;TEST UP BUTTON
    BTFSS UpButtonPending		    ;if pressed change log entry viewed
    GOTO $+3	; not pressed, do nothing
    
    ;button pressed
    BCF UpButtonPending
    CALL CountAdrUp
    ;handle changing of read address
    ;
    ;
    
    ;TEST DOWN BUTTON
    BTFSS DownButtonPending		    ;if pressed change log entry viewed 
    GOTO $+3	; not pressed, do nothing
    
    ;button pressed
    BCF DownButtonPending
    CALL CountAdrDown
    ;handle changing of read address
    ;
    ;
    
    RETURN	
    
;-------------------------------------------------------------------------------    
UpdateState.SB1set.SB2clear	;aka s3
    ;s3 tasks
    ;...........................................................................
    ;	up/down press to change hours log
    ;...........................................................................
    BTFSS UpButtonPending		    ;if pressed inc hours 
    GOTO $+3	; not pressed, do nothing
    
    BCF UpButtonPending
    CALL CountHourUp
    ;handle changing of cur hours
    ;
    ;
    
    
    
    BTFSS DownButtonPending		    ;if pressed dec hours 
    GOTO $+3	; not pressed, do nothing
    
    BCF DownButtonPending
    CALL CountHourDown
    ;handle changing of cur hours
    ;
    ;
    
    ;...........................................................................
    ;	mode select press to go to s4
    ;...........................................................................
    BTFSS ModeButtonPending		    ;if pressed dec hours 
    GOTO $+3	; not pressed, do nothing
    
    BCF ModeButtonPending
    ;move to state s4
    BSF StateBit2
    
    
    ;...........................................................................
    ;	timeout to cancel reset and go back to s1
    ;...........................................................................
    ;TIMEOUT TEST
    MOVFW State_Timeout
    BTFSC STATUS,Z
    GOTO $+2			;if not zero skip over
    
    BCF StateBit1
    
    
    RETURN
    
;-------------------------------------------------------------------------------    
UpdateState.SB1set.SB2set	;aka s4
    ;s4 tasks:
    ;...........................................................................
    ;	up/down press for changing minutes
    ;...........................................................................
    
    BTFSS UpButtonPending		    ;if pressed inc minutes 
    GOTO $+3	; not pressed, do nothing
    
    BCF UpButtonPending
    CALL CountMinuteUp
    ;handle changing of cur minutes
    ;
    ;
    
    
    
    BTFSS DownButtonPending		    ;if pressed dec minutes 
    GOTO $+3	; not pressed, do nothing
    
    BCF DownButtonPending
    CALL CountMinuteDown
    ;handle changing of cur minutes
    ;
    ;
    
    
    
    
    
    ;...........................................................................
    ;	mode select press to reset and go to s1
    ;...........................................................................
    BTFSS ModeButtonPending
    GOTO $+4
    
    BCF ModeButtonPending
    ;do a rtc write
    BCF StateBit1		;move to s1
    BCF StateBit2
    
    
    ;...........................................................................
    ;	timeout to cancel reset and go back to s1
    ;...........................................................................
    ;TIMEOUT TEST
    MOVFW State_Timeout
    BTFSC STATUS,Z
    GOTO $+3			;if not zero skip over
    
    BCF StateBit1		;move to s1
    BCF StateBit2
    
    RETURN

;###END#OF#CALL###
    
    ;functions for each of the above states' actions
;-------------------------------------------------------------------------------    
CountAdrDown			;decrease the CUR_EEADR memory block, wrapping as needed
    DECFSZ CUR_LOG_ENTRY,F	;move one entry backward
    GOTO $+8			;if not zero, skip the next 7 lines
    
;if CUR_LOG_ENTRY=0
    MOVFW LOG_COUNT
    MOVWF CUR_LOG_ENTRY		;if equal zero(invalid) then wrap around to max (LOG_COUNT)
    
    ;now we must ***increase*** eeadr by 4*LOG_COUNT, as this correlates with an equal movement of CUR_LOG_ENTRY
    ;eg, if CUR_LOG_ENTRY was 1, and goes to 4, and CUR_EEADR was 52, it should now move up by (4-1)*4=12 entries to 64.
    ;this is a semi-dynamic array type structure.
    RLF LOG_COUNT,W
    MOVWF TEMP_REG
    RLF TEMP_REG,W		;multiply LOG_COUNT by 4
    ADDWF CUR_EEADR,F		;add LOG_COUNT*4 to CUR_EEADR
    GOTO CountAdrDown.Validate
    
;if CUR_LOG_ENTRY!=0
    DECF CUR_LOG_ENTRY,F
    DECF CUR_LOG_ENTRY,F
    DECF CUR_LOG_ENTRY,F
    DECF CUR_LOG_ENTRY,F
    
    MOVLW .252
    SUBWF CUR_EEADR,W
    BTFSC STATUS,Z  ;if Z=set (READ_EEADR=252) then set READ_EEADR to 248 (skip invalid region)
    MOVLW .248
    MOVWF CUR_EEADR
    ;note here we move down past the invalid region to .248, treating it as if it were not there
    RETURN
    
CountAdrDown.Validate		;checks that the CUR_EEADR is valid, and moves it to a valid range if invalid 
    MOVLW .252
    SUBWF CUR_EEADR,W
    BTFSC STATUS,Z  ;if Z=set (READ_EEADR=252) then set READ_EEADR to 0, (skip the invalid region)
    CLRF CUR_EEADR  ;set READ_EEADR=0
    ;note here, since we are moving up back to the top of our list, we move up over the invalid region, to 0
    RETURN
    
;-------------------------------------------------------------------------------    

CountAdrUp			;increase the CUR_EEADR memory block, wrapping as needed
    INCF CUR_LOG_ENTRY,F	;move one entry forward
    INCF LOG_COUNT,W		;max value=LOG_COUNT+1
    SUBWF CUR_LOG_ENTRY,W	; if W>f C=0
    BTFSS STATUS,C		;if f>=LOG_COUNT+1 C=1 so skip the goto if CUR_LOG_ENTRY==LOG_COUNT+1
    GOTO $+8			;if not LOG_COUNT (C=0), skip the next 7 lines
    
;if CUR_LOG_ENTRY=LOG_COUNT+1
    CLRF CUR_LOG_ENTRY		;if equal max (LOG_COUNT+1), wrap around to 1
    INCF CUR_LOG_ENTRY		;set current entry to 1
    
    ;now we must ***decrease*** eeadr by 4*LOG_COUNT, as this correlates with an equal movement of CUR_LOG_ENTRY
    ;eg, if CUR_LOG_ENTRY was 4, and goes to 1, and CUR_EEADR was 52, it should now move up by (4-1)*4 entries to 40.
    ;this is a semi-dynamic array type structure.
    RLF LOG_COUNT,W
    MOVWF TEMP_REG
    RLF TEMP_REG,W		;multiply LOG_COUNT by 4
    SUBWF CUR_EEADR,F		;subtract LOG_COUNT*4 from CUR_EEADR
    GOTO CountAdrUp.Validate
    
;if CUR_LOG_ENTRY!=LOG_COUNT
    INCF CUR_LOG_ENTRY,F
    INCF CUR_LOG_ENTRY,F
    INCF CUR_LOG_ENTRY,F
    INCF CUR_LOG_ENTRY,F
        
    MOVLW .252
    SUBWF CUR_EEADR,W
    BTFSC STATUS,Z  ;if Z=set (READ_EEADR=252) then set READ_EEADR to 0, (skip the invalid region)
    CLRF CUR_EEADR  ;set READ_EEADR=0
    ;note here, since we are moving up back to the top of our list, we move up over the invalid region, to 0
    RETURN
    
CountAdrUp.Validate		;checks that the CUR_EEADR is valid, and moves it to a valid range if invalid 
    MOVLW .252
    SUBWF CUR_EEADR,W
    BTFSC STATUS,Z  ;if Z=set (READ_EEADR=252) then set READ_EEADR to 248 (skip invalid region)
    MOVLW .248
    MOVWF CUR_EEADR
    ;note here we move down past the invalid region to .248, treating it as if it were not there
    RETURN

;-------------------------------------------------------------------------------    
        
CountHourDown			;count CUR_TIME_H down
    MOVLW .23			;handle the case where hours<0
    DECFSZ CUR_TIME_H,F		;set value to 23
    GOTO $+2
    MOVWF CUR_TIME_H
    
    RETURN
    
;-------------------------------------------------------------------------------    

CountHourUp			;count CUR_TIME_H up
    INCF CUR_TIME_H,F		;inc hours
    
    MOVLW .24			;if hours =24, set hours =0
    SUBLW CUR_TIME_H
    BTFSC STATUS,Z
    CLRF CUR_TIME_H
    
    RETURN
    
;-------------------------------------------------------------------------------    

CountMinuteDown			;count CUR_TIME_L down
    MOVLW .59			;handle the case where hours<0
    DECFSZ CUR_TIME_L,F		;set value to 23
    GOTO $+2
    MOVWF CUR_TIME_L
    
    RETURN
    
;-------------------------------------------------------------------------------    

CountMinuteUp			;count CUR_TIME_L up
    INCF CUR_TIME_L,F		;inc minutes
    
    MOVLW .60			;if minutes =60, set minutes =0
    SUBLW CUR_TIME_L
    BTFSC STATUS,Z
    CLRF CUR_TIME_L
    
    RETURN
    
;-------------------------------------------------------------------------------
    
;###END#OF#CALL###
 
    
;*******************************************************************************
; EEPROM.write 
;*******************************************************************************
eeprom.write
    ;increment log count, if f=63, do not increase
    INCF LOG_COUNT,F
    MOVLW .64		;if W>f, C=0
    SUBWF LOG_COUNT,W
    BTFSC STATUS,C
    DECF LOG_COUNT,F	;if not (0.64>f), then decrement f (so keep f=.63)
    
    
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
SampleData
;   http://www.embeddedrelated.com/showarticle/110.php
;   useful advice about how to set up multiple channel ADCs
;--------------------------------
;sample temperature
;--------------------------------

    BANKSEL ADCON0
    MOVLW b'10101001'			;right justify
    MOVWF ADCON0			;Vdd,AN10, on
    CALL delay_5_ms
    
    BSF ADCON0,GO	;start conversion
    BTFSC ADCON0,GO	;are we done the conversion?
    GOTO $-1		;try again if not

    ;get value in from ADC
    BANKSEL STATUS
    BCF STATUS,C
    BANKSEL ADRESL
    INCF ADRESL,F
    RRF ADRESL,W	;divide by 2 with rounding	
    BANKSEL PORTC
    ;parse value read
    ;MOVWF TEMP_REG
    ;BCF STATUS,C
    ;RRF TEMP_REG,W
    ;MOVLW .25
    MOVWF TEMP_REG
    MOVLW .10		    ;calibration offset (ADC read is 10 too high)
    SUBWF TEMP_REG,W
    MOVWF READ_TEMPERATURE   ;stored for saving to eeprom later (and for displaying)

;--------------------------------
;sample humidity
;--------------------------------
    ;humidity is a vaguely linear value, ranging from 1V(30%) to 3V(90%)
    ;thus 0V=0% 3.3V=100%
    ;note that our step number is 1024 (10bit) we will ignore the lowest 4 bits
    ;to improve accuracy, thus effectively we are dealing with a 256 step (8bit)
    ;for such an ADC 1 step=19.53mV, 100 steps=1.953V, this is very small
    ;so instead of scaling 3.3V to 1.953V, we scale 3.3V to 2*1.953V=3.90625V
    ;when parsing our values, we divide the entire number by 2 and get our result.
    ;thus we scale 3.3V to 3.91 with a gain of 3.91/3.3 = 1.188
    ;thus Rf=0.188*R1 for a positive gain op-amp
    ; in the end however, the exact value is hardware calibrated
    BANKSEL ADCON0
    MOVLW b'00101101'			;left justify
    MOVWF ADCON0			;Vdd,AN11, on
    CALL delay_5_ms
    
    BSF ADCON0,GO	;start conversion
    BTFSC ADCON0,GO	;are we done the conversion?
    GOTO $-1		;try again if not
    
    ;get values
    BANKSEL STATUS
    BCF STATUS,C
    BANKSEL ADRESH
    INCF ADRESH,F	;scale is 0->200, so we must half the value
    RRF ADRESH,W	;divide by 2 with rounding and move into W
    BANKSEL PORTC
    MOVWF READ_HUMIDITY   



;--------------------------------
;sample supply level
;--------------------------------
    ;we get in the 0.6V constant level from the ADC.
    ;this will appear as a proportion of Vcc eg, if Vcc=1.2V 
    ;the read will be 512 (of 1024)
    ;as a result we can estimate what
    ;percentage of Vcc 0.6V is
    
    MOVLW b'00110101'			;left justify
    MOVWF ADCON0			;Vdd,0.6V constant, on

    CALL delay_5_ms
    
    BSF ADCON0,GO	;start conversion
    BTFSC ADCON0,GO	;are we done the conversion?
    GOTO $-1		;try again if not
    
    ;x=read-in
    ;y=5x/6=0.5V =10%
    ;how many y's are in 255 (how many 10%'s of 5V are in our current Vcc)
    ;ans: 255/y = 255*6/5x=306/x
    ;this is hard to do as it involves 16bit calculations
    ;z=x/2 -> rrf(x) = z
    ;find 153/z so 153=z*c round down

    ;get value in from ADC
    BCF STATUS,C
    BANKSEL ADRESH
    RRF ADRESH,W	;divide by 2 (get z)	
    BANKSEL PORTC
    ;parse value read
    MOVWF TEMP_REG
    MOVLW .153
    ;swap TEMP_REG and W
    XORWF TEMP_REG,F
    XORWF TEMP_REG,W
    XORWF TEMP_REG,F
    
    CLRF READ_POWERSUPPLY

    INCF READ_POWERSUPPLY,F ;inc counter
    SUBWF TEMP_REG,F
    ;if W<=f carry clear -> skip if carry set
    BTFSS STATUS,C
    GOTO $-3
    DECF READ_POWERSUPPLY,F
    ;this is the 10's digit, the 1's digit will always be 0
    
    ;BCF ADCON0,0    ;disable ADC until next sample
;--------------------------------
;obtain time
;--------------------------------    
    ;get time from the RTC (and go nuts with glee if it works)
    
    
    RETURN
;###END#OF#CALL###
    
    

;*******************************************************************************
;   The following 3 functions are called as the state of the
;   appropriate button changes
;*******************************************************************************
ButtonUp.OnPress
    BSF UpButtonPending

    MOVLW 'U'		;debugging, we are in state 1
    MOVWF LCD_BUFFER
    MOVLW 0x0D
    MOVWF LCD_POSITION
    CALL lcd.writepos
    
    RETURN
    
ButtonDown.OnPress
    BSF DownButtonPending
   
    MOVLW 'D'		;debugging, we are in state 1
    MOVWF LCD_BUFFER
    MOVLW 0x0E
    MOVWF LCD_POSITION
    CALL lcd.writepos
    
    RETURN
    
ButtonMode.OnPress
    BSF ModeButtonPending
    
    MOVLW 'M'		;debugging, we are in state 1
    MOVWF LCD_BUFFER
    MOVLW 0x0F
    MOVWF LCD_POSITION
    CALL lcd.writepos
    
    RETURN

ButtonUp.OnRelease

    MOVLW ' '		;debugging, we are in state 1
    MOVWF LCD_BUFFER
    MOVLW 0x0D
    MOVWF LCD_POSITION
    CALL lcd.writepos
    
    RETURN
    
ButtonDown.OnRelease
   
    MOVLW ' '		;debugging, we are in state 1
    MOVWF LCD_BUFFER
    MOVLW 0x0E
    MOVWF LCD_POSITION
    CALL lcd.writepos
    
    RETURN
    
    MOVLW ' '		;debugging, we are in state 1
    MOVWF LCD_BUFFER
    MOVLW 0x0F
    MOVWF LCD_POSITION
    CALL lcd.writepos
    
ButtonMode.OnRelease
    
    RETURN
;###END#OF#CALL###
    
    
    
    
;*******************************************************************************
;Poll_Button_Up	    ;checks for a logical low
;		     primarily behaves like a set of nested if statements
;*******************************************************************************
Poll_Button_Up    
    
    BTFSS   Button_Up.State		;LSB=0 button released| LSB=1, button pressed
    GOTO    Poll_Button_Up.ReleasedState
    GOTO    Poll_Button_Up.PressedState
    
;============
;State Released SUBCALL
;============
    ;check if button is pressed or released currently
Poll_Button_Up.ReleasedState    
    BTFSS Button_Up	    		;check RA0, if RA0=0 button=pressed, count
								;if RA0=1 button=released, clear count
    GOTO Poll_Button_Up.ReleasedState.Pressed
    GOTO Poll_Button_Up.ReleasedState.Released
    
Poll_Button_Up.ReleasedState.Pressed
    INCF Button_Up_Counter,F
;<check for counter=4, if 4, then inc cur_num and reset counter as needed>
    BTFSS Button_Up_Counter,2	;.4 = b'00000100'
    GOTO Poll_Button_Up.End 	;if !=4, exit the poll
    
    CLRF Button_Up_Counter	    ;clear the cur_num
    BSF Button_Up.State	    	;change status to Pressed (1)
    
    ;---------------
    ;conditional stuff for when our button is pressed
    CALL ButtonUp.OnPress
    ;---------------
    
    GOTO Poll_Button_Up.End
    
Poll_Button_Up.ReleasedState.Released
    CLRF Button_Up_Counter
    GOTO Poll_Button_Up.End
    
;%%%%END%OF%SUBCALL%%%%
    

;============
;State Pressed SUBCALL
;============
    ;check if button is pressed or released currently    
Poll_Button_Up.PressedState    
    BTFSS Button_Up	    		;check RA0, if RA0=0 button=pressed, count
								;if RA0=1 button=released, clear count
    GOTO Poll_Button_Up.PressedState.Pressed
    GOTO Poll_Button_Up.PressedState.Released
    
    
Poll_Button_Up.PressedState.Released
    INCF Button_Up_Counter,F
;<check for counter=4, if 4, then inc cur_num and reset counter as needed>
    BTFSS Button_Up_Counter,2  	;.4 = b'00000100'
    GOTO Poll_Button_Up.End 	;if !=4, exit the poll
    
    CLRF Button_Up_Counter	    ;clear the cur_num
    BCF Button_Up.State	    	;change status to Released (0)
    CALL ButtonUp.OnRelease
    GOTO Poll_Button_Up.End
    
    
Poll_Button_Up.PressedState.Pressed
    CLRF Button_Up_Counter
    GOTO Poll_Button_Up.End
    
;%%%%END%OF%SUBCALL%%%%
    
Poll_Button_Up.End
    RETURN
    
;###END#OF#CALL###								

    
;*******************************************************************************
;Poll_Button_Down	    ;checks port for a logical low
;		     primarily behaves like a set of nested if statements
;*******************************************************************************
Poll_Button_Down    
    
    BTFSS   Button_Down.State		; 0-> button released| 1 -> button pressed
    GOTO    Poll_Button_Down.ReleasedState
    GOTO    Poll_Button_Down.PressedState
    
;============
;State Released SUBCALL
;============
    ;check if button is pressed or released currently
Poll_Button_Down.ReleasedState    
    BTFSS Button_Down	    		;check RA1, if RA1=0 button=pressed, count
								;if RA1=1 button=released, clear count
    GOTO Poll_Button_Down.ReleasedState.Pressed
    GOTO Poll_Button_Down.ReleasedState.Released
    
Poll_Button_Down.ReleasedState.Pressed
    INCF Button_Down_Counter,F
;<check for counter=4, if 4, then inc cur_num and reset counter as needed>
    BTFSS Button_Down_Counter,2	; .4 = b'00000100'
    GOTO Poll_Button_Down.End 	;if !=4, exit the poll
    
    CLRF Button_Down_Counter	;clear the cur_num
    BSF Button_Down.State	    	;change status to Pressed (1)

    ;---------------
    ;conditional stuff for when our button is pressed
    CALL ButtonDown.OnPress
    ;---------------
    GOTO Poll_Button_Down.End
    
Poll_Button_Down.ReleasedState.Released
    CLRF Button_Down_Counter
    GOTO Poll_Button_Down.End
    
;%%%%END%OF%SUBCALL%%%%
    

;============
;State Pressed SUBCALL
;============
    ;check if button is pressed or released currently    
Poll_Button_Down.PressedState    
    BTFSS Button_Down	   			;check RA1, if RA1=0 button=pressed, count
								;if RA1=1 button=released, clear count
    GOTO Poll_Button_Down.PressedState.Pressed
    GOTO Poll_Button_Down.PressedState.Released
    
    
Poll_Button_Down.PressedState.Released
    INCF Button_Down_Counter,F
;<check for counter=4, if 4, then inc cur_num and reset counter as needed>
    BTFSS Button_Down_Counter,2	;.4 = b'00000100'
    GOTO Poll_Button_Down.End 	;if !=4, exit the poll
    
    CLRF Button_Down_Counter	;clear the cur_num
    BCF Button_Down.State	    	;change status to Released (0)
    CALL ButtonDown.OnRelease
    GOTO Poll_Button_Down.End
    
    
Poll_Button_Down.PressedState.Pressed
    CLRF Button_Down_Counter
    GOTO Poll_Button_Down.End
    
;%%%%END%OF%SUBCALL%%%%
    
Poll_Button_Down.End
    RETURN
    
;###END#OF#CALL###								

;*******************************************************************************
;Poll_Button_Mode	    ;checks port for a logical low
;		     primarily behaves like a set of nested if statements
;*******************************************************************************
Poll_Button_Mode    
    
    BTFSS   Button_Mode.State		;LSB=0 button released| LSB=1, button pressed
    GOTO    Poll_Button_Mode.ReleasedState
    GOTO    Poll_Button_Mode.PressedState
    
;============
;State Released SUBCALL
;============
    ;check if button is pressed or released currently
Poll_Button_Mode.ReleasedState    
    BTFSS Button_Mode	    		;check RB7, if RB7=0 button=pressed, count
								;if RB7=1 button=released, clear count
    GOTO Poll_Button_Mode.ReleasedState.Pressed
    GOTO Poll_Button_Mode.ReleasedState.Released
    
Poll_Button_Mode.ReleasedState.Pressed
    INCF Button_Mode_Counter,F
;<check for counter=4, if 4, then inc cur_num and reset counter as needed>
    BTFSS Button_Mode_Counter,2	;.4 = b'00000100'
    GOTO Poll_Button_Mode.End 	;if !=4, exit the poll
    
    CLRF Button_Mode_Counter	    ;clear the cur_num
    BSF Button_Mode.State	    	;change status to Pressed (1)
    
    ;---------------
    ;conditional stuff for when our button is pressed
    CALL ButtonMode.OnPress 
    ;---------------
    
    GOTO Poll_Button_Mode.End
    
Poll_Button_Mode.ReleasedState.Released
    CLRF Button_Mode_Counter
    GOTO Poll_Button_Mode.End
    
;%%%%END%OF%SUBCALL%%%%
    

;============
;State Pressed SUBCALL
;============
    ;check if button is pressed or released currently    
Poll_Button_Mode.PressedState    
    BTFSS PORTA,0	    		;check RA0, if RA0=0 button=pressed, count
								;if RA0=1 button=released, clear count
    GOTO Poll_Button_Mode.PressedState.Pressed
    GOTO Poll_Button_Mode.PressedState.Released
    
    
Poll_Button_Mode.PressedState.Released
    INCF Button_Mode_Counter,F
;<check for counter=4, if 4, then inc cur_num and reset counter as needed>
    BTFSS Button_Mode_Counter,2  	;.4 = b'00000100'
    GOTO Poll_Button_Mode.End 	;if !=4, exit the poll
    
    CLRF Button_Mode_Counter	    ;clear the cur_num
    BCF Button_Mode.State	    	;change status to Released (0)
    CALL ButtonMode.OnRelease
    GOTO Poll_Button_Mode.End
    
    
Poll_Button_Mode.PressedState.Pressed
    CLRF Button_Mode_Counter
    GOTO Poll_Button_Mode.End
    
;%%%%END%OF%SUBCALL%%%%
    
Poll_Button_Mode.End
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
    BCF TRISC,5	;set RTC IO output
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
    BSF TRISC,5	;set RTC IO input
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
    BCF TRISC,5	;set RTC IO output
    BANKSEL PORTC
    
    ;---------------------------------------------------------------------------
    BSF RTC_CE			;enable TX/RX
    MOVLW b'10000101'		;1,calendar/clock,adr=2,read
    MOVWF RTC_BUFFER
    CALL RTC.write.TX		;send address and specify read mode
    
    BANKSEL TRISA
    BSF TRISC,0	;set RTC SCLK input
    BSF TRISC,5	;set RTC IO input
    BANKSEL PORTC
    
    CALL RTC.read.RX		;receive data in RTC minutes reg into RTC_BUFFER
    MOVFW RTC_BUFFER
    MOVWF READ_TIME_L
    BCF RTC_CE			;end TX/RX
    ;---------------------------------------------------------------------------
    BSF RTC_CE			;enable TX/RX

    BANKSEL TRISA
    BCF TRISC,0	;set RTC SCLK output
    BCF TRISC,5	;set RTC IO output
    BANKSEL PORTC

    MOVLW b'10000111'		;1,calendar/clock,adr=3,read
    MOVWF RTC_BUFFER
    CALL RTC.write.TX		;send address and specify read mode
    
    BANKSEL TRISA
    BSF TRISC,0	;set RTC SCLK input
    BSF TRISC,5	;set RTC IO input
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
    
    
    
    
    
;-------------------------------------------------------------------------------    
;===============================================================================    
;-------------------------------------------------------------------------------
;			    LCD CODE BELOW HERE    
;-------------------------------------------------------------------------------    
;===============================================================================    
;-------------------------------------------------------------------------------    
    
;*******************************************************************************
; Delays 
;*******************************************************************************
delay_5_ms
    MOVLW .20
    MOVWF D2
    GOTO $+3
    
delay_10_ms
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
    RETURN
    
;###END#OF#CALL###    
    
    
BINtoLCD
;*******************************************************************************
;	binary -> BCD -> LCD outputs W as BCD_L and BCD_H in ascii
;*******************************************************************************
    CALL BINtoBCD
    MOVLW b'00110000'
    ADDWF BCD_L,F
    ADDWF BCD_H,F
    
    RETURN    
;###END#OF#CALL###
    
    
;*******************************************************************************
;	Single digit binary -> ascii
;*******************************************************************************
BintoCHAR    
    MOVLW b'00110000'
    ADDWF LCD_BUFFER,F
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
    CALL delay_10_ms
    BSF PORTC,LCD_RS			    ;set to data mode
    GOTO $+3
lcd.command
    CALL delay_10_ms
    BCF PORTC,LCD_RS			    ;set to command mode
;part below this is the write part of lcd.data and lcd.command
    
    SWAPF LCD_BUFFER,W		    ;move LCD_BUFFER<4:7> to W<0:3>
    CALL lcd.write			    ;write W<0:3> to LCD<4:7>
    
    MOVFW LCD_BUFFER		    ;move LCD_BUFFER<0:3> to W<0:3>
    CALL lcd.write			    ;write W<0:3> to LCD<4:7>
    
    
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
;	combines all LCD write functions into a formatted print
;*******************************************************************************
lcd.print.lastsample
    CALL lcd.home	;sends clear command
    
    MOVFW READ_TEMPERATURE
    CALL BINtoLCD	;outputs=BCD_H and BCD_L
	MOVFW BCD_H
	MOVWF LCD_BUFFER
	CALL lcd.data	;sends temp_10s to display
    
	MOVFW BCD_L
	MOVWF LCD_BUFFER
	CALL lcd.data	;sends temp_1s to display
	MOVLW b'11011111'	;degrees symbol
	MOVWF LCD_BUFFER
	
	CALL lcd.data	;text to display
	MOVLW 'C'
	MOVWF LCD_BUFFER
	CALL lcd.data	;text to display
    
    CALL lcd.newline	;sends newline command (set address=40h)
	
    MOVFW READ_HUMIDITY
    CALL BINtoLCD	;outputs=BCD_H and BCD_L
	MOVFW BCD_H
	MOVWF LCD_BUFFER
	CALL lcd.data	;sends humidity_10s to display
    
	MOVFW BCD_L
	MOVWF LCD_BUFFER
	CALL lcd.data	;sends humidity_1s to display
	
	MOVLW '%'	;degrees symbol
	MOVWF LCD_BUFFER
	CALL lcd.data	;text to display
    
    RETURN
;###END#OF#CALL###
    
    
    
    
;*******************************************************************************
;	a generic LCD print function that will print any ascii array
;*******************************************************************************
;inspired by this code http://www.microchip.com/forums/m90152.aspx
lcd.printString	;prints a line of text     
    BCF INTCON,GIE	;disable interrupts
    BANKSEL Input1
    MOVFW Input1
    BANKSEL EEADR
    MOVWF EEADR
    BANKSEL Input2
    MOVFW Input2
    BANKSEL EEADR
    MOVWF EEADRH
    
lcd.printString.loop
    CALL eeprom.ReadProgMem
    MOVFW CHAR_H	;test for NullChar
    BTFSC STATUS,Z	;if zero, end routine, else print
	GOTO lcd.printString.end		;end
    MOVWF LCD_BUFFER	;print
    CALL lcd.data
    
    MOVFW CHAR_L	;test for NullChar
    BTFSC STATUS,Z	;if zero, end routine, else print
	GOTO lcd.printString.end		;end
    MOVWF LCD_BUFFER	;print
    CALL lcd.data
	
    BANKSEL EEADR
	INCF EEADR,F	;move to next memory address
    BTFSC STATUS,C	;if carry set, we have a rollover, so inc EEADRH
	INCF EEADRH,F	
    GOTO lcd.printString.loop 
    
lcd.printString.end
    BSF INTCON,GIE	;re-enable interrupts
    RETURN
    
;*******************************************************************************
;	uses eeprom to read ascii data stored in program memory
;*******************************************************************************
eeprom.ReadProgMem
    ;mostly used example 10-3 from the datasheet as a reference
    BANKSEL EECON1
    BSF EECON1, EEPGD	;program memory
    BSF EECON1, RD	;EE read
    nop
    nop
    BANKSEL EEDAT
    MOVFW EEDAT
    BANKSEL 0x00
    MOVWF CHAR_L
    BANKSEL EEDAT
    MOVFW EEDATH
    BANKSEL 0x00
    MOVWF CHAR_H
    ;input was as follows (7H 7L)   : HHHHHHHLLLLLLL
    ;output is as follows	    : 00HHHHHH  HLLLLLLL
    ;as such we must shift the MSB of CHAR_L into CHAR_H
    RLF CHAR_L,W		    ;MSB CHAR_L -> Carry, does not affect CHAR_L
    RLF CHAR_H,F		    ;Carry -> LSB CHAR_H
    BCF CHAR_H,7		    ;clear the MSB of CHAR_H
    BCF CHAR_L,7		    ;clear the MSB of CHAR_L
    
    RETURN
;###END#OF#CALL###

;*******************************************************************************
;   commonly used LCD commands
;*******************************************************************************
    
lcd.clear
    MOVLW 0x01
    MOVWF LCD_BUFFER
    CALL lcd.command	;sends clear command
    RETURN
    
lcd.newline
    MOVLW b'11000000'	;1,addr=100 0000 (40h)
    MOVWF LCD_BUFFER
    CALL lcd.command	;sends newline command (set address=40h)
    RETURN
    
lcd.home
    MOVLW b'10000000'	;1,addr=000 0000 (00h)
    MOVWF LCD_BUFFER
    CALL lcd.command	;sends home command (set address=00h)
    RETURN

lcd.writepos	;writes LCD_BUFFER to LCD_POSITION

    MOVFW LCD_BUFFER	;store character for later
    MOVWF TEMP_REG2
    
    MOVFW LCD_POSITION
    MOVWF LCD_BUFFER
    BSF LCD_BUFFER,7	;1,LCD_POSITION
    CALL lcd.command	;sends location command (set address=LCD_POSITION)
    
    MOVFW TEMP_REG2	;restore character from earlier
    MOVWF LCD_BUFFER
    CALL lcd.data	;send data contained in LCD_BUFFER
    RETURN    
;###END#OF#CALL###

    
;LCD ascii arrays:    
StudentName	DA "  David Parker  ",0
StudentNumber	DA "  213562404     ",0
    
    END