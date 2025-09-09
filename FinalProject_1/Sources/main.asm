;*****************************************************************
;*  COE538 Project                                               *
;*                                                               *
;*  Authors: Mariam Ahmad, Kaden Yu                              *
;*                                                               *
;*****************************************************************

; export symbols
            XDEF Entry, _Startup            ; export 'Entry' symbol
            ABSENTRY Entry        ; for absolute assembly: mark this as application entry point



; Include derivative-specific definitions 
		INCLUDE 'derivative.inc' 

;*********************************************************************
;*
;*********************************************************************

;Liquid Crystal Display Equates (Read-guider Code)

CLEAR_HOME    EQU $01 ; Clear the display and home the cursor
INTERFACE     EQU $38 ; 8 bit interface, two line display
CURSOR_OFF    EQU $0C ; Display on, cursor off
SHIFT_OFF     EQU $06 ; Address increments, no character shift
LCD_SEC_LINE  EQU 64 ; Starting addr. of 2nd line of LCD (note decimal value!)

; LCD Addresses
LCD_CNTR      EQU PTJ ; LCD Control Register: E = PJ7, RS = PJ6
LCD_DAT       EQU PORTB ; LCD Data Register: D7 = PB7, ... , D0 = PB0
LCD_E         EQU $80 ; LCD E-signal pin
LCD_RS        EQU $40 ; LCD RS-signal pin

; Other codes
NULL          EQU 00 ; The string ’null terminator’
CR            EQU $0D ; ’Carriage Return’ character
SPACE         EQU ' ' ; The ’space’ character
                



START         EQU 0
FWD           EQU 1
ALL_STP       EQU 2
L_TRN      EQU 3 ;left turn
R_TRN     EQU 4  ;right turn
REV_TRN       EQU 5   ;reverse turn
L_ALIGN    EQU 6  ;left align
R_ALIGN   EQU 7   ;right align
TIME_LEFT     EQU 8  ;turn timing (left)
TIME_RIGHT    EQU 8  ;turn timing (right)


; variable/data section
              ORG $3800
;---------------------------------------------------------------------------
; Storage Registers (9S12C32 RAM space: $3800 ... $3FFF)

;baseline sensor collaboration
BASE_LINE       FCB $9D
BASE_BOW        FCB $CE ;baseline for front sensor
BASE_MID        FCB $CE ;baseline for mid sensor
BASE_PORT       FCB $CE ;baseline for port sensor
BASE_STARBOARD  FCB $CE ;baselien for starboard sensor


;variance for sensors
LINE_VAR       FCB $20   ;variance thresholds (cannnot exceed these values)
BOW_VAR        FCB $20
PORT_VAR       FCB $20
MID_VAR        FCB $20
STARBOARD_VAR  FCB $20



TOP_LINE      RMB 20 ; Top line of display
              FCB NULL ; terminated by null
              
BOT_LINE      RMB 20 ; Bottom line of display
              FCB NULL ; terminated by null
              
CLEAR_LINE    FCC '                 '
              FCB NULL ; terminated by null
              
TEMP          RMB 1 ; Temporary location



SENSOR_LINE   FCB $01 ; Storage for guider sensor readings
SENSOR_BOW    FCB $23 ; Initialized to test values
SENSOR_PORT   FCB $45
SENSOR_MID    FCB $67
SENSOR_STBD   FCB $89
SENSOR_NUM    RMB 1 ; The currently selected sensor



              ORG $3850 ; Where our TOF counter register lives
TOF_COUNTER   dc.b 0 ; The timer, incremented at 23Hz
CRNT_STATE    dc.b 3 ; Current state register
TIME_TURN     ds.b 1 ; Duration of time it takes to turn
TEN_THOUS     ds.b 1 ; 10,000 digit
THOUSANDS     ds.b 1 ; 1,000 digit
HUNDREDS      ds.b 1 ; 100 digit
TENS          ds.b 1 ; 10 digit
UNITS         ds.b 1 ; 1 digit
NO_BLANK      ds.b 1 ; Used in ’leading zero’ blanking by BCD2ASC
BCD_SPARE     rmb 2

; code section
              ORG $4000 ; Start of program text (FLASH memory)
;---------------------------------------------------------------------------
; Initialization

Entry:
_Startup:
              LDS #$4000 ; Initialize the stack pointer
              CLI ; Enable interrupts
              JSR INIT ; Initialize ports
              JSR openADC ; Initialize the ATD
              JSR initLCD ; Initialize the LCD
              JSR CLR_LCD_BUF ; Write ’space’ characters to the LCD buffer
                                                           ;I
              BSET DDRA,%00000011 ; STAR_DIR, PORT_DIR      N
              BSET DDRT,%00110000 ; STAR_SPEED, PORT_SPEED  I
                                                          ; T
              JSR initAD ; Initialize ATD converter         I
                                                          ; A
              JSR initLCD ; Initialize the LCD              L
              JSR clrLCD ; Clear LCD & home cursor          I
                                                          ; Z
              LDX #msg1 ; Display msg1                      A
              JSR putsLCD                               ; " T
                                                          ; I
              LDAA #$C0 ; Move LCD cursor to the 2nd row    O
              JSR cmd2LCD                                 ; N
              LDX #msg2 ; Display msg2                      |
              JSR putsLCD ;                               " |
                                                          ;
              JSR ENABLE_TOF ; Jump to TOF initialization ---------------
           
;---------------------------------------------------------------------------
; Display Sensors

MAIN          
             
              JSR G_LEDS_ON ; Enable the guider LEDs
              JSR READ_SENSORS ; Read the 5 guider sensors
              JSR G_LEDS_OFF ; Disable the guider LEDs
              JSR UPDT_DISPL ; and write them to the LCD
              LDAA CRNT_STATE ;load the current state ->accumulator A                                           
              JSR DISPATCHER ;
              BRA MAIN ; Loop forever

; data section
*******************************************************************
msg1  dc.b "Battery volt ",0
msg2  dc.b "State ",0
tab   dc.b "START ",0
      dc.b "FWD  ",0                ;display labels and states
      dc.b "ALL_STP",0
      dc.b "L_TRN ",0
      dc.b "R_TRN ",0
      dc.b "REV  ", 0
      dc.b "L_ALIGN",0
      dc.b "R_ALIGN",0


; subroutine section
*******************************************************************
DISPATCHER 
           CMPA #START ; If it’s the START state -----------------
           BNE NOT_START ; |
           JSR START_ST ; then call START_ST routine D
           BRA DISP_EXIT ; and exit I
          
          
NOT_START CMPA #FWD  ;If in the fwd state
          BNE NOT_FWD 
          JSR FWD_ST    ;then call FWD_ST routine
          BRA DISP_EXIT ;and exit
         
          
NOT_FWD CMPA #REV_TRN    ;if in the rev_trn state
        BNE NOT_REV_TRN
        JSR REV_TRN_ST   ;call the REV_TRN_ST
        BRA DISP_EXIT    ;exit
       
        
NOT_REV_TRN CMPA #ALL_STP    ;if in the all_stop state
        BNE NOT_ALL_STOP
        JSR ALL_STP_ST       ;call the ALL_STOP_ST routine
        BRA DISP_EXIT        ;exit
        
        
NOT_ALL_STOP CMPA #L_TRN       ;if in the l_trn (left turn) state
             BNE NOT_L_TRN
             JSR L_TRN_ST      ;call the L_TRN_ST routine
             BRA DISP_EXIT     ;exit


NOT_L_TRN CMPA #L_ALIGN       ;if in the l_align (left align) state 
            BNE NOT_L_ALIGN    
            JSR L_ALIGN_ST    ;call the L_ALIGN_ST routine 
            BRA DISP_EXIT     ;exit 
            
            
NOT_L_ALIGN CMPA #R_TRN      ;if in the r_trn (right turn) state 
            BNE NOT_R_TRN
            JSR R_TRN_ST     ;call the R_TRN_ST routine
            BRA DISP_EXIT    ;exit
            
            
NOT_R_TRN   CMPA #R_ALIGN    ;if in the r_align (right align) state
            JSR R_ALIGN_ST   ;call the R_ALIGN routine
            BRA DISP_EXIT    ;exit       

DISP_EXIT RTS ; Exit from the state dispatcher ----------
*******************************************************************
START_ST BRCLR PORTAD0,$04,NO_FWD  ;if /FWD_BUMP
         JSR INIT_FWD             ; Initialize the FWD state
         MOVB #FWD, CRNT_STATE    ; Go into the FWD state
         BRA START_EXIT

NO_FWD NOP ; Else
START_EXIT RTS ; return to the MAIN routine
*******************************************************************
FWD_ST BRSET PORTAD0,$04,NO_FWD_BUMP    ;will check if front bumper has been hit
       MOVB #REV_TRN, CRNT_STATE        ;if it has been hit then it will go to rev_trn_st
       JSR UPDT_DISPL
       JSR INIT_REV
       LDY #6000
       JSR del_50us
       JSR INIT_RIGHT
       LDY #6000
       JSR del_50us
       JMP FWD_EXIT
       
NO_FWD_BUMP     BRSET PORTAD0,$08,NO_REAR_BUMP  ; If REAR_BUMP, then we should stop
                JSR   INIT_ALL_STP              ; so initialize the ALL_STOP state
                MOVB  #ALL_STP,CRNT_STATE       ; and change state to ALL_STOP
                JMP   FWD_EXIT                  ; and return
                
NO_REAR_BUMP    LDAA SENSOR_BOW  ;check if bot is on the line
                ADDA BOW_VAR
                CMPA BASE_BOW
                BPL NOT_ALIGN   ;if bow is not on the line then it will go to NOT_ALIGN routine
                LDAA SENSOR_MID
                ADDA MID_VAR
                CMPA BASE_MID
                BPL NOT_ALIGN    ;if mid is not on the line then NOT_ALIGN routine
                LDAA SENSOR_LINE
                ADDA LINE_VAR
                CMPA BASE_LINE
                BPL CHECK_R_ALIGN   ;check if sensors are within the defined thresholds (right)
                LDAA SENSOR_LINE
                SUBA LINE_VAR
                CMPA BASE_LINE
                BMI CHECK_L_ALIGN     ;check if sensors are within defined thresholds (left)
                JMP FWD_EXIT
                
FWD_EXIT RTS

*******************************************************************
NOT_ALIGN       LDAA    SENSOR_PORT        ;movement of the eebot depending on the sensors                                                    
                  ADDA    PORT_VAR                                                              
                  CMPA    BASE_PORT                                                              
                  BPL     PARTIAL_L_TRN  ;will go to the PARTIAL_L_TURN routine (result greater than/ equal to BASE_PORT)                                                  
                  BMI     NO_PORT        ;will go to NO_PORT routine (result is less than BASE_PORT)
                  ;partial left turn                                                   
                                              
NO_PORT           LDAA    SENSOR_BOW                                                            
                  ADDA    BOW_VAR              ;no turn excecuted                                                  
                  CMPA    BASE_BOW                                                                
                  BPL     ALIGN_EXIT                                                                    
                  BMI     NO_BOW                                                              

NO_BOW            LDAA    SENSOR_STBD                                                            
                  ADDA    STARBOARD_VAR                                                              
                  CMPA    BASE_STARBOARD                                                              
                  BPL     PARTIAL_R_TRN                                                        
                  BMI     ALIGN_EXIT       ;If certain conditions are met, certain turns will be excecuted 
                  ;partial right turn                 

ALIGN_EXIT      RTS

*******************************************************************  ;for partial left turns and left alignment
PARTIAL_L_TRN     LDY     #6000                                                             
                  JSR     del_50us                                                                
                  JSR     INIT_LEFT                                                              
                  MOVB    #L_TRN, CRNT_STATE                                                  
                  LDY     #6000                                                                    
                  JSR     del_50us                                                                
                  JMP     PARTIAL_L_EXIT

CHECK_L_ALIGN     JSR     INIT_LEFT                                                              
                  MOVB    #L_ALIGN, CRNT_STATE                                                
                  JMP     PARTIAL_L_EXIT
                 
PARTIAL_L_EXIT  RTS                  

*********************************************************************  ;for partial right turns and right alignment
PARTIAL_R_TRN     LDY     #6000                                                                  
                  JSR     del_50us                                                               
                  JSR     INIT_RIGHT                                                              
                  MOVB    #R_TRN, CRNT_STATE                                            
                  LDY     #6000                                                                  
                  JSR     del_50us                                                                
                  BRA     PARTIAL_R_EXIT                                                                  

CHECK_R_ALIGN     JSR     INIT_RIGHT                                                              
                  MOVB    #R_ALIGN, CRNT_STATE                                                
                  BRA     PARTIAL_R_EXIT

PARTIAL_R_EXIT RTS

********************************************************************
L_TRN_ST          LDAA    SENSOR_BOW                                                         
                  ADDA    BOW_VAR                                                                
                  CMPA    BASE_BOW                                                              
                  BPL     L_ALIGN_ST                                                        
                  JMP     LR_EXIT

L_ALIGN_ST        MOVB    #FWD, CRNT_STATE          ;routines for left and right turns                                              
                  JSR     INIT_FWD                  ;and routines for left and right alignment so that                                               
                  JMP     LR_EXIT                   ;the eebot remains on track                                    

R_TRN_ST          LDAA    SENSOR_BOW                                                              
                  ADDA    BOW_VAR                                                               
                  CMPA    BASE_BOW                                                                
                  BPL     R_ALIGN_ST                                                        
                  JMP     LR_EXIT

R_ALIGN_ST  MOVB    #FWD, CRNT_STATE                                                        
                  JSR     INIT_FWD                                                                
                  JMP     LR_EXIT
                 
LR_EXIT   RTS

*******************************************************************
ALL_STP_ST BRSET PORTAD0,$04,NO_START ; If FWD_BUMP
           BCLR PTT,%00110000 ; initialize the START state (both motors off)
           MOVB #START,CRNT_STATE ; set the state to START
           BRA ALL_STP_EXIT ; and return
           
NO_START NOP ; Else
ALL_STP_EXIT RTS ; return to the MAIN routine
*******************************************************************
REV_TRN_ST LDAA SENSOR_BOW
           ADDA BOW_VAR
           CMPA BASE_BOW
           BMI REV_TRN_EXIT
           JSR INIT_LEFT
           MOVB #FWD, CRNT_STATE
           JSR INIT_FWD
           BRA REV_TRN_EXIT
REV_TRN_EXIT RTS
*******************************************************************
INIT_FWD BCLR PORTA,%00000011 ; Set FWD direction for both motors
         BSET PTT,%00110000 ; Turn on the drive motors
  RTS
*******************************************************************
INIT_REV BSET PORTA,%00000011 ; Set REV direction for both motors
         BSET PTT,%00110000 ; Turn on the drive motors
  RTS
*******************************************************************
INIT_ALL_STP BCLR PTT,%00110000 ; Turn off the drive motors
  RTS
*******************************************************************
INIT_LEFT    BSET PORTA,%00000001 
             BCLR PORTA,%00000010
             LDAA TOF_COUNTER ;this marks the TOF time 
             ADDA #TIME_LEFT   ;will be added with the time it takes to make a left turn
             STAA TIME_TURN   ;time will be stored in accumulator A
  RTS
*******************************************************************
INIT_RIGHT   BSET PORTA,%00000010 
             BCLR PORTA,%00000001
             LDAA TOF_COUNTER ;this marks the TOF time 
             ADDA #TIME_RIGHT ;will be added with the time it takes to make a right turn
             STAA TIME_TURN   ;time will be stored in accumulator A
  RTS             

*****************************************************************
; Initialize ports
INIT          BCLR DDRAD,$FF ; Make PORTAD an input (DDRAD @ $0272)
              BSET DDRA,$FF ; Make PORTA an output (DDRA @ $0002)
              BSET DDRB,$FF ; Make PORTB an output (DDRB @ $0003)
              BSET DDRJ,$C0 ; Make pins 7,6 of PTJ outputs (DDRJ @ $026A)
              RTS
;---------------------------------------------------------------------------
; Initialize the ADC
openADC       MOVB #$80,ATDCTL2 ; Turn on ADC (ATDCTL2 @ $0082)
              LDY #1 ; Wait for 50 us for ADC to be ready
              JSR del_50us ; - " -
              MOVB #$20,ATDCTL3 ; 4 conversions on channel AN1 (ATDCTL3 @ $0083)
              MOVB #$97,ATDCTL4 ; 8-bit resolution, prescaler=48 (ATDCTL4 @ $0084)
              RTS
;---------------------------------------------------------------------------
; Clear LCD Buffer
; This routine writes ’space’ characters (ascii 20) into the LCD display
; buffer in order to prepare it for the building of a new display buffer.
; This needs only to be done once at the start of the program. Thereafter the
; display routine should maintain the buffer properly.
CLR_LCD_BUF   LDX #CLEAR_LINE
              LDY #TOP_LINE
              JSR STRCPY
              
CLB_SECOND    LDX #CLEAR_LINE
              LDY #BOT_LINE
              JSR STRCPY
              
CLB_EXIT      RTS

;---------------------------------------------------------------------------
; String Copy
; Copies a null-terminated string (including the null) from one location to
; another
; Passed: X contains starting address of null-terminated string
; Y contains first address of destination
STRCPY      PSHX ; Protect the registers used
            PSHY
            PSHA
            
STRCPY_LOOP LDAA 0,X ; Get a source character
            STAA 0,Y ; Copy it to the destination
            BEQ STRCPY_EXIT ; If it was the null, then exit
            INX ; Else increment the pointers
            INY
            BRA STRCPY_LOOP ; and do it again
            
STRCPY_EXIT PULA ; Restore the registers
            PULY
            PULX
            RTS
;---------------------------------------------------------------------------
; Guider LEDs ON
; This routine enables the guider LEDs so that readings of the sensor
; correspond to the ’illuminated’ situation.
; Passed: Nothing
; Returns: Nothing
; Side: PORTA bit 5 is changed
G_LEDS_ON   BSET PORTA,%00100000 ; Set bit 5
            RTS
;
; Guider LEDs OFF
; This routine disables the guider LEDs. Readings of the sensor
; correspond to the ’ambient lighting’ situation.
; Passed: Nothing
; Returns: Nothing
; Side: PORTA bit 5 is changed
G_LEDS_OFF  BCLR PORTA,%00100000 ; Clear bit 5
            RTS

;---------------------------------------------------------------------------
; Read Sensors
;
; This routine reads the eebot guider sensors and puts the results in RAM
; registers.
; Note: Do not confuse the analog multiplexer on the Guider board with the
; multiplexer in the HCS12. The guider board mux must be set to the
; appropriate channel using the SELECT_SENSOR routine. The HCS12 always
; reads the selected sensor on the HCS12 A/D channel AN1.
; The A/D conversion mode used in this routine is to read the A/D channel
; AN1 four times into HCS12 data registers ATDDR0,1,2,3. The only result
; used in this routine is the value from AN1, read from ATDDR0. However,
; other routines may wish to use the results in ATDDR1, 2 and 3.
; Consequently, Scan=0, Mult=0 and Channel=001 for the ATDCTL5 control word.
; Passed: None
; Returns: Sensor readings in:
; SENSOR_LINE (0) (Sensor E/F)
; SENSOR_BOW (1) (Sensor A)
; SENSOR_PORT (2) (Sensor B)
; SENSOR_MID (3) (Sensor C)
; SENSOR_STBD (4) (Sensor D)
; Note:
; The sensor number is shown in brackets
;
; Algorithm:
; Initialize the sensor number to 0
; Initialize a pointer into the RAM at the start of the Sensor Array storage
; Loop Store %10000001 to the ATDCTL5 (to select AN1 and start a conversion)
; Repeat
; Read ATDSTAT0
; Until Bit SCF of ATDSTAT0 == 1 (at which time the conversion is complete)
; Store the contents of ATDDR0L at the pointer
; If the pointer is at the last entry in Sensor Array, then
; Exit
; Else
; Increment the sensor number
; Increment the pointer
; Loop again.

READ_SENSORS  CLR SENSOR_NUM ; Select sensor number 0
              LDX #SENSOR_LINE ; Point at the start of the sensor array
             
RS_MAIN_LOOP  LDAA SENSOR_NUM ; Select the correct sensor input
              JSR SELECT_SENSOR ; on the hardware
              LDY #400 ; 20 ms delay to allow the
              JSR del_50us ; sensor to stabilize
              
              LDAA #%10000001 ; Start A/D conversion on AN1
              STAA ATDCTL5
              BRCLR ATDSTAT0,$80,* ; Repeat until A/D signals done
              
              LDAA ATDDR0L ; A/D conversion is complete in ATDDR0L
              STAA 0,X ; so copy it to the sensor register
              CPX #SENSOR_STBD ; If this is the last reading
              BEQ RS_EXIT ; Then exit
              
              INC SENSOR_NUM ; Else, increment the sensor number
              INX ; and the pointer into the sensor array
              BRA RS_MAIN_LOOP ; and do it again
              
RS_EXIT       RTS


;---------------------------------------------------------------------------
; Select Sensor
; This routine selects the sensor number passed in ACCA. The motor direction
; bits 0, 1, the guider sensor select bit 5 and the unused bits 6,7 in the
; same machine register PORTA are not affected.
; Bits PA2,PA3,PA4 are connected to a 74HC4051 analog mux on the guider board,
; which selects the guider sensor to be connected to AN1.
; Passed: Sensor Number in ACCA
; Returns: Nothing
; Side Effects: ACCA is changed
; Algorithm:
; First, copy the contents of PORTA into a temporary location TEMP and clear
; the sensor bits 2,3,4 in the TEMP to zeros by ANDing it with the mask
; 11100011. The zeros in the mask clear the corresponding bits in the
; TEMP. The 1’s have no effect.
; Next, move the sensor selection number left two positions to align it
; with the correct bit positions for sensor selection.
; Clear all the bits around the (shifted) sensor number by ANDing it with
; the mask 00011100. The zeros in the mask clear everything except
; the sensor number.
; Now we can combine the sensor number with the TEMP using logical OR.
; The effect is that only bits 2,3,4 are changed in the TEMP, and these
; bits now correspond to the sensor number.
; Finally, save the TEMP to the hardware.

SELECT_SENSOR   PSHA ; Save the sensor number for the moment

                LDAA PORTA ; Clear the sensor selection bits to zeros
                ANDA #%11100011 ;
                STAA TEMP ; and save it into TEMP
                
                PULA ; Get the sensor number
                ASLA ; Shift the selection number left, twice
                ASLA ;
                ANDA #%00011100 ; Clear irrelevant bit positions

                ORAA TEMP ; OR it into the sensor bit positions
                STAA PORTA ; Update the hardware
                RTS

;---------------------------------------------------------------------------
; Display Sensor Readings
; Passed: Sensor values in RAM locations SENSOR_LINE through SENSOR_STBD.
; Returns: Nothing
; Side: Everything
; This routine writes the sensor values to the LCD. It uses the ’shadow buffer’ approach.
; The display buffer is built by the display controller routine and then copied in its
; entirety to the actual LCD display. Although simpler approaches will work in this
; application, we take that approach to make the code more re-useable.
; It’s important that the display controller not write over other information on the
; LCD, so writing the LCD has to be centralized with a controller routine like this one.
; In a more complex program with additional things to display on the LCD, this routine
; would be extended to read other variables and place them on the LCD. It might even
; read some ’display select’ variable to determine what should be on the LCD.
; For the purposes of this routine, we’ll put the sensor values on the LCD
; in such a way that they (sort of) mimic the position of the sensors, so
; the display looks like this:
; 01234567890123456789
; ___FF_______________
; PP_MM_SS_LL_________
; Where FF is the front sensor, PP is port, MM is mid, SS is starboard and
; LL is the line sensor.
; The corresponding addresses in the LCD buffer are defined in the following
; equates (In all cases, the display position is the MSDigit).
DP_FRONT_SENSOR   EQU TOP_LINE+3
DP_PORT_SENSOR    EQU BOT_LINE+0
DP_MID_SENSOR     EQU BOT_LINE+3
DP_STBD_SENSOR    EQU BOT_LINE+6
DP_LINE_SENSOR    EQU BOT_LINE+9

DISPLAY_SENSORS   LDAA SENSOR_BOW ; Get the FRONT sensor value
                  JSR BIN2ASC ; Convert to ascii string in D
                  LDX #DP_FRONT_SENSOR ; Point to the LCD buffer position
                  STD 0,X ; and write the 2 ascii digits there
                  LDAA SENSOR_PORT ; Repeat for the PORT value
                  JSR BIN2ASC
                  LDX #DP_PORT_SENSOR
                  STD 0,X
                  LDAA SENSOR_MID ; Repeat for the MID value
                  JSR BIN2ASC
                  LDX #DP_MID_SENSOR
                  STD 0,X
                  LDAA SENSOR_STBD ; Repeat for the STARBOARD value
                  JSR BIN2ASC
                  LDX #DP_STBD_SENSOR
                  STD 0,X
                  LDAA SENSOR_LINE ; Repeat for the LINE value
                  JSR BIN2ASC
                  LDX #DP_LINE_SENSOR
                  STD 0,X
                  LDAA #CLEAR_HOME ; Clear the display and home the cursor
                  JSR cmd2LCD ; "
                  LDY #40 ; Wait 2 ms until "clear display" command is complete
                  JSR del_50us
                  LDX #TOP_LINE ; Now copy the buffer top line to the LCD
                  JSR putsLCD
                  LDAA #LCD_SEC_LINE ; Position the LCD cursor on the second line
                  JSR LCD_POS_CRSR
                  LDX #BOT_LINE ; Copy the buffer bottom line to the LCD
                  JSR putsLCD
                  RTS

;---------------------------------------------------------------------------
; Binary to ASCII
; Converts an 8 bit binary value in ACCA to the equivalent ASCII character 2
; character string in accumulator D
; Uses a table-driven method rather than various tricks.
; Passed: Binary value in ACCA
; Returns: ASCII Character string in D
; Side Fx: ACCB is destroyed

HEX_TABLE         FCC '0123456789ABCDEF' ; Table for converting values

BIN2ASC           PSHA ; Save a copy of the input number on the stack
                  TAB ; and copy it into ACCB
                  ANDB #%00001111 ; Strip off the upper nibble of ACCB
                  CLRA ; D now contains 000n where n is the LSnibble
                  ADDD #HEX_TABLE ; Set up for indexed load
                  XGDX
                  LDAA 0,X ; Get the LSnibble character
                  
                  PULB ; Retrieve the input number into ACCB
                  PSHA ; and push the LSnibble character in its place
                  RORB ; Move the upper nibble of the input number
                  RORB ; into the lower nibble position.
                  RORB
                  RORB
                  ANDB #%00001111 ; Strip off the upper nibble
                  CLRA ; D now contains 000n where n is the MSnibble
                  ADDD #HEX_TABLE ; Set up for indexed load
                  XGDX
                  LDAA 0,X ; Get the MSnibble character into ACCA
                  PULB ; Retrieve the LSnibble character into ACCB
                  RTS
                  
                  
;---------------------------------------------------------------------------
; Routines to control the Liquid Crystal Display
;---------------------------------------------------------------------------
; Initialize the LCD

initLCD           BSET DDRB,%11111111 ; configure pins PB7...PB0 for output
                  BSET DDRJ,%11000000 ; configure pins PJ7,PJ6 for output
                  LDY #2000 ; wait for LCD to be ready
                  JSR del_50us ; -"-
                  LDAA #$28 ; set 4-bit data, 2-line display
                  JSR cmd2LCD ; -"-
                  LDAA #$0C ; display on, cursor off, blinking off
                  JSR cmd2LCD ; -"-
                  LDAA #$06 ; move cursor right after entering a character
                  JSR cmd2LCD ; -"-
                  RTS

openLCD           LDY #2000 ; Wait 100 ms for LCD to be ready
                  JSR del_50us ; "
                  LDAA #INTERFACE ; Set 8-bit data, 2-line display, 5x8 font
                  JSR cmd2LCD ; "
                  LDAA #CURSOR_OFF ; Display on, cursor off, blinking off
                  JSR cmd2LCD ; "
                  LDAA #SHIFT_OFF ; Move cursor right (address increments, no char. shift)
                  JSR cmd2LCD ; "
                  LDAA #CLEAR_HOME ; Clear the display and home the cursor
                  JSR cmd2LCD ; "
                  LDY #40 ; Wait 2 ms until "clear display" command is complete
                  JSR del_50us ; "
                  RTS
                  
*******************************************************************
clrLCD LDAA #$01 ; clear cursor and return to home position
       JSR cmd2LCD ; -"-
       LDY #40 ; wait until "clear cursor" command is complete
       JSR del_50us ; -"-
  RTS
;---------------------------------------------------------------------------
; Send a command in accumulator A to the LCD

cmd2LCD           BCLR LCD_CNTR,LCD_RS ; Select the LCD Instruction register
                  JSR dataMov ; Send data to IR or DR of the LCD
                  RTS
;---------------------------------------------------------------------------
; Send a character in accumulator in A to LCD

putcLCD           BSET LCD_CNTR,LCD_RS ; select the LCD Data register
                  JSR dataMov ; send data to IR or DR of the LCD
                  RTS
;---------------------------------------------------------------------------
; Send a NULL-terminated string pointed to by X
putsLCD           LDAA 1,X+ ; get one character from the string
                  BEQ donePS ; reach NULL character?
                  JSR putcLCD
                  BRA putsLCD
donePS            RTS

;---------------------------------------------------------------------------
; Send data to the LCD IR or DR depending on the RS signal

dataMov           BSET LCD_CNTR,LCD_E ; pull the LCD E-sigal high
                  STAA LCD_DAT ; send the 8 bits of data to LCD
                  NOP
                  NOP
                  NOP
                  BCLR LCD_CNTR,LCD_E ; pull the E signal low to complete the write operation
                  LDY #1 ; adding this delay will complete the internal
                  JSR del_50us ; operation for most instructions
                  RTS
                  
;---------------------------------------------------------------------------
; Position the Cursor
; This routine positions the display cursor in preparation for the writing
; of a character or string.
; For a 20x2 display:
; The first line of the display runs from 0 .. 19.
; The second line runs from 64 .. 83.
; The control instruction to position the cursor has the format
; 1aaaaaaa
; where aaaaaaa is a 7 bit address.
; Passed: 7 bit cursor Address in ACCA
; Returns: Nothing
; Side Effects: None

LCD_POS_CRSR      ORAA #%10000000 ; Set the high bit of the control word
                  JSR cmd2LCD ; and set the cursor address
                  RTS
;---------------------------------------------------------------------------
; 50 Microsecond Delay

del_50us          PSHX ; (2 E-clk) Protect the X register
eloop             LDX #300 ; (2 E-clk) Initialize the inner loop counter
iloop             NOP ; (1 E-clk) No operation
                  DBNE X,iloop ; (3 E-clk) If the inner cntr not 0, loop again
                  DBNE Y,eloop ; (3 E-clk) If the outer cntr not 0, loop again
                  PULX ; (3 E-clk) Restore the X register
                  RTS ; (5 E-clk) Else return

*******************************************************************
* Update Display (bump switches,Battery Voltage, Current State, sensor readings) *
*******************************************************************
UPDT_DISPL MOVB #$90,ATDCTL5 ; R-just., uns., sing. conv., mult., ch=0, start
           BRCLR ATDSTAT0,$80,* ; Wait until the conver. seq. is complete
           LDAA ATDDR0L ; Load the ch0 result - battery volt - into A
 LDAB #39 ;AccB = 39
 MUL ;AccD = 1st result x 39
 ADDD #600 ;AccD = 1st result x 39 + 600

 JSR int2BCD
 JSR BCD2ASC
 LDAA #$8F ;move LCD cursor to the 1st row, end of msg1
 JSR cmd2LCD ;"

 LDAA TEN_THOUS ;output the TEN_THOUS ASCII character
 JSR putcLCD 

 LDAA THOUSANDS ;output the TEN_THOUS ASCII character
 JSR putcLCD 
 
 LDAA #'.'
 JSR putcLCD


 LDAA HUNDREDS ;output the TEN_THOUS ASCII character
 JSR putcLCD ;

;-------------------------
           LDAA #$C6 ; Move LCD cursor to the 2nd row, end of msg2
           JSR cmd2LCD ;
           LDAB CRNT_STATE ; Display current state
           LSLB ; "
           LSLB ; "
           LSLB ; "
           LDX #tab ; "
           ABX ; "
           JSR putsLCD ; "
  RTS
  
  
  *******************************************************************
initAD MOVB #$C0,ATDCTL2 ;power up AD, select fast flag clear
       JSR del_50us ;wait for 50 us
       MOVB #$00,ATDCTL3 ;8 conversions in a sequence
       MOVB #$85,ATDCTL4 ;res=8, conv-clks=2, prescal=12
       BSET ATDDIEN,$0C ;configure pins AN03,AN02 as digital inputs
  RTS

*******************************************************************
int2BCD XGDX ;Save the binary number into .X
        LDAA #0 ;Clear the BCD_BUFFER
        STAA TEN_THOUS
        STAA THOUSANDS
        STAA HUNDREDS
        STAA TENS
        STAA UNITS
        STAA BCD_SPARE
        STAA BCD_SPARE+1

        CPX #0 ;Check for a zero input
        BEQ CON_EXIT ;and if so, exit

        XGDX ;Not zero, get the binary number back to .D as dividend
        LDX #10 ;Setup 10 (Decimal!) as the divisor
        IDIV ;Divide: Quotient is now in .X, remainder in .D
        STAB UNITS ;Store remainder
        CPX #0 ;If quotient is zero,
        BEQ CON_EXIT ;then exit

        XGDX ;else swap first quotient back into .D
        LDX #10 ;and setup for another divide by 10
        IDIV
        STAB TENS
        CPX #0
        BEQ CON_EXIT

        XGDX ;Swap quotient back into .D
        LDX #10 ;and setup for another divide by 10

        IDIV
        STAB HUNDREDS
        CPX #0
        BEQ CON_EXIT
        *
        XGDX ;Swap quotient back into .D
        LDX #10 ;and setup for another divide by 10
        IDIV
        STAB THOUSANDS
        CPX #0
        BEQ CON_EXIT
        *
        XGDX ;Swap quotient back into .D
        LDX #10 ;and setup for another divide by 10
        IDIV
        STAB TEN_THOUS
        *
CON_EXIT RTS ;We’re done the conversion

*******************************************************************
BCD2ASC LDAA #0 ;Initialize the blanking flag
        STAA NO_BLANK
        *
C_TTHOU LDAA TEN_THOUS ;Check the ’ten_thousands’ digit
        ORAA NO_BLANK
        BNE NOT_BLANK1
        *
ISBLANK1 LDAA #' ' ;It’s blank
         STAA TEN_THOUS ;so store a space
         BRA C_THOU ;and check the ’thousands’ digit
        *
NOT_BLANK1 LDAA TEN_THOUS ;Get the ’ten_thousands’ digit
           ORAA #$30 ;Convert to ascii
           STAA TEN_THOUS
           LDAA #$1 ;Signal that we have seen a ’non-blank’ digit
           STAA NO_BLANK
           *
C_THOU LDAA THOUSANDS ;Check the thousands digit for blankness
       ORAA NO_BLANK ;If it’s blank and ’no-blank’ is still zero
       BNE NOT_BLANK2
       *
ISBLANK2 LDAA #' ' ;Thousands digit is blank
         STAA THOUSANDS ;so store a space
         BRA C_HUNS ;and check the hundreds digit
         *
NOT_BLANK2 LDAA THOUSANDS ;(similar to ’ten_thousands’ case)
         ORAA #$30
         STAA THOUSANDS
         LDAA #$1

         STAA NO_BLANK
         *
C_HUNS LDAA HUNDREDS ;Check the hundreds digit for blankness
       ORAA NO_BLANK ;If it’s blank and ’no-blank’ is still zero
       BNE NOT_BLANK3
       *
ISBLANK3 LDAA #' ' ;Hundreds digit is blank
         STAA HUNDREDS ;so store a space
         BRA C_TENS ;and check the tens digit
         *
NOT_BLANK3 LDAA HUNDREDS ;(similar to ’ten_thousands’ case)
           ORAA #$30
           STAA HUNDREDS
           LDAA #$1
           STAA NO_BLANK
           *
C_TENS LDAA TENS ;Check the tens digit for blankness
       ORAA NO_BLANK ;If it’s blank and ’no-blank’ is still zero
       BNE NOT_BLANK4
       *
ISBLANK4 LDAA #' ' ;Tens digit is blank
         STAA TENS ;so store a space
         BRA C_UNITS ;and check the units digit
         *
NOT_BLANK4 LDAA TENS ;(similar to ’ten_thousands’ case)
           ORAA #$30
           STAA TENS
           *
C_UNITS LDAA UNITS ;No blank check necessary, convert to ascii.
        ORAA #$30
        STAA UNITS
        *
  RTS ;We’re done

*******************************************************************
ENABLE_TOF LDAA #%10000000
           STAA TSCR1 ;enable TCNT
           STAA TFLG2 ;clear tof
           LDAA #%10000100 ;enable toi, select prescale to 16
           STAA TSCR2
  RTS
*******************************************************************
TOF_ISR INC TOF_COUNTER
        LDAA #%10000000
        STAA TFLG2
        RTI


;**************************************************************
;*                 Interrupt Vectors                          *
;**************************************************************
            ORG   $FFFE
            DC.W  Entry           ; Reset Vector
