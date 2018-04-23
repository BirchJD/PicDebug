                  LIST     P = P12F629

                  INCLUDE  "../PIC_INCLUDE/P12F629.INC"

                  __CONFIG _FOSC_INTRCIO & _WDT_ON & _PWRTE_ON & _MCLRE_OFF & _BOREN_OFF & _CP_ON & _CPD_OFF


;/***************************************************************************/
;/* PIC DEBUGGING    2018-04-22 V1.00 (C) Jason Birch                       */
;/*                                                                         */
;/* YouTube Video demonstrating this program can be found here:             */
;/* https://youtu.be/yuuGP2ic2B4                                            */
;/*                                                                         */
;/* Examples of debugging methods for PIC micro-controllers. This code can  */
;/* be loaded onto a PIC12F683, 8 pin device, and run. The code can be      */
;/* migrated to most other PIC micro-contollers, the main consideration     */
;/* when migrating being the banks used for RAM registers and names of the  */
;/* register bits.                                                          */
;/*                                                                         */
;/* 1. LED Flash Count: Periodically flash an LED on a single GPIO pin, to  */
;/*    indicate a status.                                                   */
;/*                                                                         */
;/* 2. Log data values into the EEPROM, so that later the PIC               */
;/*    micro-controller can be placed back into the PIC programmer and the  */
;/*    logged data values read back out to be analysed.                     */
;/*                                                                         */
;/* Code protection can be on, but for logging to EEPROM data protection    */
;/* needs to be off, so the data can be read back by the PIC programmer.    */
;/***************************************************************************/


;/*************/
;/* Constants */
;/*************/

; PORT A
GPIO_LED          EQU      GP5                  ; On/Off LED.


TIMER_0_SCALE     EQU      0x20


; LED_INFO
LED_INFO_INDICATE EQU      1                    ; Flash at lease once so LED flash reporting shows it is working.
LED_INFO_EVENT_1  EQU      2                    ; Each additional event can be a power of two so multi-events can be reported.
LED_INFO_EVENT_2  EQU      4                    ; Or just a number of flashes can be used to report a more codes.
LED_INFO_EVENT_3  EQU      8                    ; Up to 16 flashes is probably useable, more would probably be too many to count.



;/******************/
;/* RAM Registers. */
;/******************/
CBLOCK            0x20
                  INT_W                         ; Temporary store for W during interupt.
                  INT_STATUS                    ; Temporary store for STATUS during interupt.

                  TMR0_SCALE_L                  ; Scale Timer 0 long engough for testing purposes.
                  TMR0_SCALE_H

                  LED_INFO                      ; Flag events to report as an LED flash code.
                  LED_FLASH_COUNT               ; Number of flashes remaining for current LED flash code.

                  LOG_DATA_POS                  ; Current position in the EEPROM log to save data.

                  TEST_DATA                     ; Test data for this example program.
ENDC



                  CODE

;/**********************************/
;/* Reset program location vector. */
;/**********************************/
                  ORG      0x0000

                  MOVLW    TIMER_0_SCALE
                  MOVWF    TMR0_SCALE_H
                  CLRF     TMR0_SCALE_L
                  GOTO     INIT



;/*************************************/
;/* Interupt program location vector. */
;/*************************************/
                  ORG      0x0004

INT_HANDLE        MOVWF    INT_W                ; Store registers from application during interupt.
                  MOVFW    STATUS
                  MOVWF    INT_STATUS

                  BCF      STATUS, RP0          ; Select Register bank 0


                  BTFSS    INTCON, T0IF         ; Did a TIMER0 interupt trigger?
                  GOTO     INT_TIMER0_END       ; TIMER0: Test harness code.

                  DECFSZ   TMR0_SCALE_L         ; Scale timer 0.
                  GOTO     TIMER_0_END
                  DECFSZ   TMR0_SCALE_H
                  GOTO     TIMER_0_END
                  MOVLW    TIMER_0_SCALE
                  MOVWF    TMR0_SCALE_H

                  MOVLW    LED_INFO_INDICATE    ; Flash LED at least once, to show LED flash indicator is working.
                  IORWF    LED_INFO, F

                  BTFSS    TEST_DATA, 0         ; Every other flash code shows event 1 example.
                  GOTO     FLASH_CODE

                  MOVLW    LED_INFO_EVENT_1     ; Add event 1 example to LED flash count.
                  IORWF    LED_INFO, F

FLASH_CODE        CALL     LED_FLASH_CODE       ; Start flash indicator.

                  MOVFW    TEST_DATA            ; Write test data to data log.
                  CALL     LOG_DATA
                  INCF     TEST_DATA            ; On the next cycle write a data value one higher as a demonstration.

TIMER_0_END       BCF      INTCON, T0IF         ; End of timer 0 interupt.


INT_TIMER0_END    BTFSS    PIR1, TMR1IF         ; Did a TIMER1 interupt trigger?
                  GOTO     INT_TIMER1_END       ; TIMER1: Flash LED indicator the required indicator count.

                  MOVF     LED_FLASH_COUNT, F   ; Are any more flashes required?
                  BTFSC    STATUS, Z
                  GOTO     TMR1_FLASH_END
                  BTFSS    GPIO, GPIO_LED       ; Toggle the LED state between On and Off.
                  GOTO     FLASH_LED_ON
                  BCF      GPIO, GPIO_LED       ; Switch LED Off.
                  DECF     LED_FLASH_COUNT      ; Reduce the remaining flash count when LED is switched Off.
                  GOTO     TMR1_FLASH_END
FLASH_LED_ON      BSF      GPIO, GPIO_LED       ; Switch LED On.

TMR1_FLASH_END    BCF      PIR1, TMR1IF         ; End of timer 1 interupt.


INT_TIMER1_END    MOVFW    INT_STATUS           ; Restore registers for application to continue.
                  MOVWF    STATUS
                  MOVFW    INT_W
                  BSF      STATUS, Z
                  BTFSS    INT_STATUS, Z
                  BCF      STATUS, Z
                  RETFIE




INIT              CLRF     GPIO                 ; Clear GPIO Port.

                  BSF      STATUS, RP0          ; Select Register bank 1

                  MOVLW    ~(1 << GPIO_LED)     ; Set all pins to input except LED.
                  MOVWF    TRISIO

                  MOVLW    (1 << PSA)|(1 << PS0)|(1 << PS1)|(1 << PS2)
                  MOVWF    OPTION_REG           ; Prescale watchdog timer.

                  MOVLW    (1 << T1IE)          ; Interupt on Timer 1 for LED indicator.
                  MOVWF    PIE1

                  BCF      STATUS, RP0          ; Select Register bank 0

                  CLRF     TMR0                 ; Start with full period for Timer 0.

                  MOVLW    (1 << GIE)|(1 << PEIE)|(1 << T0IE) ; Enable interupts.
                  MOVWF    INTCON               ; Configure timer 0 interupts and enable interupts.
                  CLRF     PIR1                 ; Clear interupt flags.

                  MOVLW    (1 << TMR1ON)|(1 << NOT_T1SYNC)|(1 << T1CKPS1)
                  MOVWF    T1CON                ; Configure timer 1.

                  CLRF     LED_INFO             ; Ensure no LED info to display at start up.
                  CLRF     LED_FLASH_COUNT
                  CLRF     LOG_DATA_POS         ; Start logging data from start of data log at start up.

                  CLRF     TEST_DATA            ; Reset test data, used for testing EEPROM log.

LOOP              CLRWDT                        ; Tell CPU application still running.
                  GOTO     LOOP                 ; Infinite main loop.



LED_FLASH_CODE    BCF      GPIO, GPIO_LED       ; Turn off LED to indicate update type with flash count.
                  CLRF     TMR1L                ; Ensure a full flash period is applied for first flash.
                  CLRF     TMR1H
                  MOVFW    LED_INFO             ; Flash the currently set flash count.
                  MOVWF    LED_FLASH_COUNT
                  CLRF     LED_INFO             ; Clear the flash indicator ready for the next value to be set.
                  RETURN



LOG_DATA          BSF      STATUS, RP0          ; Select Register bank 1.
                  MOVWF    EEDATA               ; Set data to write to data log.
                  MOVLW    DATA_LOG_START       ; Start of data log EEPROM Address.
                  ADDWF    LOG_DATA_POS, W      ; Add the current address offset to write to.
                  MOVWF    EEADR                ; Set the EEPROM address to write to.
                  MOVF     EEADR, F
                  BTFSC    STATUS, Z            ; Make sure EEPROM address does not wrap and overwrite beginning of EEPROM.
                  GOTO     LOG_DATA_END         ; For 128 byte EEPROM, EEADR is 7 bits, for 256 byte EEADR is 8 bits.
                  BSF      EECON1, WREN         ; Enable writes to EEPROM.
                  BCF      INTCON, GIE          ; Disable interupts.
                  MOVLW    0x55                 ; Unlock write with required code sequence.
                  MOVWF    EECON2
                  MOVLW    0xAA
                  MOVWF    EECON2
                  BSF      EECON1, WR           ; Start EEPROM write.
                  BSF      INTCON, GIE          ; Enable interupts.
EEPROM_WRITE_WAIT BTFSC    EECON1, WR           ; Wait for EEPROM write to complete.
                  GOTO     EEPROM_WRITE_WAIT
                  BCF      EECON1, WREN         ; Disable additional writes to EEPROM.
                  INCF     LOG_DATA_POS         ; Point to next data log address for next time data is logged.
LOG_DATA_END      BCF      STATUS, RP0          ; Select Register bank 0
                  RETURN



;/*************************/
;/* Write data to EEPROM. */
;/*************************/
                  ORG      0x2100

EEPROM_MEMORY
                  DE       0x00                 ; Example program data.
                  DE       0x00                 ; Example program data.
                  DE       0x00                 ; Example program data.
                  DE       0x00                 ; Example program data.
DATA_LOG_ID       DE       0xAA                 ; Two bytes to identify start of logged data when analyzing.
                  DE       0x55
DATA_LOG_START    DE       0xFF                 ; Place holder reference for start of data logging area.


                  END

