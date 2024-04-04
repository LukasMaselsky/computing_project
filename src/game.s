# PASTE LINK TO TEAM VIDEO BELOW
#
#

  .syntax unified
  .cpu cortex-m4
  .fpu softvfp
  .thumb
  
  .global Main
  .global  SysTick_Handler
  .global EXTI0_IRQHandler

  @ Definitions are in definitions.s to keep this file "clean"
  .include "./src/definitions.s"

  .equ    BLINK_PERIOD, 250
  .equ    BLINK_DECREMENT, 25
  .equ    BLINK_MINIMUM, 100
  .equ    MATCH_LED, LD3_PIN

  .section .text

Main:
  PUSH  {R4-R6,LR}

  @ initialise match led to LED3 in memory
  LDR R4, =match_led
  LDR R5, =MATCH_LED
  STR R5, [R4]

  @ initialise result in memory
  LDR R4, =result
  LDR R5, =0
  STR R5, [R4]

  @ initialise blink period in memory
  LDR R4, =blink_period
  LDR R5, =BLINK_PERIOD
  STR R5, [R4]

  @
  @ Prepare GPIO Port E Pin 9 for output (LED LD3)
  @ We'll blink LED LD3 (the orange LED)
  @

  @ Enable GPIO port E by enabling its clock
  LDR     R4, =RCC_AHBENR
  LDR     R5, [R4]
  ORR     R5, R5, #(0b1 << (RCC_AHBENR_GPIOEEN_BIT))
  STR     R5, [R4]

  @ Configure all LEDS for output byt setting all bits
  @   by setting bits 27:26 of GPIOE_MODER to 01 (GPIO Port E Mode Register)
  @   (by BIClearing then ORRing)
  LDR     R4, =GPIOE_MODER
  LDR     R5, [R4]                    @ Read ...
  LDR     R6, =0xFFFF0000
  BIC     R5, R6
  LDR     R6, =0x55550000             @ initialise bits to 010101....
  ORR     R5, R6
  STR     R5, [R4]                    @ Write 

  @ Initialise the first countdown

  LDR     R4, =blink_countdown
  LDR     R5, =BLINK_PERIOD
  STR     R5, [R4]  

  @ Configure SysTick Timer to generate an interrupt every 1ms

  LDR     R4, =SCB_ICSR               @ Clear any pre-existing interrupts
  LDR     R5, =SCB_ICSR_PENDSTCLR     @
  STR     R5, [R4]                    @

  LDR     R4, =SYSTICK_CSR            @ Stop SysTick timer
  LDR     R5, =0                      @   by writing 0 to CSR
  STR     R5, [R4]                    @   CSR is the Control and Status Register
  
  LDR     R4, =SYSTICK_LOAD           @ Set SysTick LOAD for 1ms delay
  LDR     R5, =7999                   @ Assuming 8MHz clock
  STR     R5, [R4]                    @ 

  LDR     R4, =SYSTICK_VAL            @   Reset SysTick internal counter to 0
  LDR     R5, =0x1                    @     by writing any value
  STR     R5, [R4]

  LDR     R4, =SYSTICK_CSR            @   Start SysTick timer by setting CSR to 0x7
  LDR     R5, =0x7                    @     set CLKSOURCE (bit 2) to system clock (1)
  STR     R5, [R4]                    @     set TICKINT (bit 1) to 1 to enable interrupts
                                      @     set ENABLE (bit 0) to 1


  @
  @ Prepare external interrupt Line 0 (USER pushbutton)
  @ We'll count the number of times the button is pressed
  @

  @ Initialise count to zero
  LDR   R4, =button_count             @ count = 0;
  MOV   R5, #0                        @
  STR   R5, [R4]                      @

  @ Initialise LED 
  LDR   R4, =LED_cycle
  LDR   R5, =0x100 @ first LED at 9th bit
  STR   R5, [R4]

  @ Configure USER pushbutton (GPIO Port A Pin 0 on STM32F3 Discovery
  @   kit) to use the EXTI0 external interrupt signal
  @ Determined by bits 3..0 of the External Interrrupt Control
  @   Register (EXTIICR)
  LDR     R4, =SYSCFG_EXTIICR1
  LDR     R5, [R4]
  BIC     R5, R5, #0b1111
  STR     R5, [R4]

  @ Enable (unmask) interrupts on external interrupt Line0
  LDR     R4, =EXTI_IMR
  LDR     R5, [R4]
  ORR     R5, R5, #1
  STR     R5, [R4]

  @ Set falling edge detection on Line0
  LDR     R4, =EXTI_FTSR
  LDR     R5, [R4]
  ORR     R5, R5, #1
  STR     R5, [R4]

  @ Enable NVIC interrupt #6 (external interrupt Line0)
  LDR     R4, =NVIC_ISER
  MOV     R5, #(1<<6)
  STR     R5, [R4]

  @ Nothing else to do in Main
  @ Idle loop forever (welcome to interrupts!!)

 
Idle_Loop:
  B     Idle_Loop
  
End_Main:
  POP   {R4-R6,PC}



@
@ SysTick interrupt handler (blink LEDs)
@
  .type  SysTick_Handler, %function
SysTick_Handler:

  PUSH  {R4-R7, LR}

  LDR   R4, =blink_countdown        @ if (countdown != 0) {
  LDR   R5, [R4]                    @
  CMP   R5, #0                      @
  BEQ   .LelseFire                  @

  SUB   R5, R5, #1                  @   countdown = countdown - 1;
  STR   R5, [R4]                    @

  B     .LendIfDelay                @ }

.LelseFire:                         @ else {



  LDR     R4, =GPIOE_ODR            @   Invert LD
  LDR     R5, [R4]                  @

  LDR R4, =result
  LDR R5, [R4]
  CMP R5, #0 @ don't do anything
  BNE .LwinOrLose
  

  LDR     R6, =LED_cycle
  LDR     R7, [R6]
  LSL     R7, #1                    @    MASK to Turns off previous, switches on next
  CMP     R7, #0x10000              @    if Shifted out of ranged
  BLo     .LContinueLEDCycle
  LDR     R7, =0x100                 @    reset to original

.LContinueLEDCycle:
  STR     R7, [R6]
  B .Lblink
.LwinOrLose:
  CMP R5, #1 @ lose
  BEQ .Llose
  @ otherwise win
  LDR R7, =0xff00 @ all leds on
  B .Lblink

.Llose:
  LDR R4, =match_led
  LDR R6, [R4]
  MOV R7, #1
  LSL R7, R6


.Lblink:

  CMP R5, #3
  BNE .LdontShowMatch
  @ show match
  LDR R4, =match_led
  LDR R6, [R4]
  MOV R7, #1
  LSL R7, R6

.LdontShowMatch:
  LDR     R4, =GPIOE_ODR            @   Invert LD
  LDR     R5, [R4]                  @

  BIC     R5, #0xFF00               @    clear all LED bits
  EOR     R5, R7                    @    Applies bit mask i.e. sets the bit from memory for next LED
  
  LDR     R4, =GPIOE_ODR 
  STR     R5, [R4]                  @    apply new state to LEDs to turn each one on/off

  LDR     R4, =blink_countdown      @   countdown = BLINK_PERIOD;
  LDR     R6, =blink_period
  LDR     R7, [R6]                  @ load blink period from memory
  STR     R7, [R4]                  @

.LendIfDelay:                       @ }

  LDR     R4, =SCB_ICSR             @ Clear (acknowledge) the interrupt
  LDR     R5, =SCB_ICSR_PENDSTCLR   @
  STR     R5, [R4]                  @

  @ Return from interrupt handler
  POP  {R4-R7, PC}



@
@ External interrupt line 0 interrupt handler
@   (count button presses)
@
  .type  EXTI0_IRQHandler, %function
EXTI0_IRQHandler:

  PUSH  {R4-R8,LR}

  LDR R4, =result
  LDR R5, [R4]
  CMP R5, #0
  BNE .LresetGame
 
  LDR   R4, =LED_cycle              
  LDR   R5, [R4]                    @ LED that is currently on

  @ load match point
  LDR R4, =match_led
  LDR R6, [R4]                      @ check if that LED is the same as the match point
  MOV R7, #1
  LSL R7, R6

  CMP   R5, R7
  BEQ .LplayerWon
  B .LplayerLost

.LplayerWon:
  MOV R5, #2

  @ decrement blink period
  LDR R6, =blink_period
  LDR R4, =BLINK_DECREMENT
  LDR R7, =BLINK_MINIMUM
  LDR R8, [R6] @ current blink period in memory
  CMP R8, R7
  BLE .LdontDec
  SUB R8, R8, R4
  STR R8, [R6]

.LdontDec:
  B .LendPlayerDecision

.LplayerLost: 
  MOV R5, #1
  @ reset blink period to initial
  LDR R4, =BLINK_PERIOD
  LDR R6, =blink_period
  STR R4, [R6]

.LendPlayerDecision:
  LDR R4, =result
  STR R5, [R4]                      @ store result in memory
  B .LclearInt

.LresetGame:
  CMP R5, #3
  BNE .LrandomiseLED

  LDR R5, =0
  STR R5, [R4] @ load 0 back to result
  @ reset LED cycle to start at new random LED
  LDR R4, =LED_cycle
  
  LDR R5, =match_led
  LDR R6, [R5]
  MOV R7, #1
  LSL R7, R6

  STR R7, [R4]
  B .LclearInt

.LrandomiseLED:
  @ randomise if win/lose state
  @ randomised number, isolates last 3 bits and +8 for random LED
  LDR R4, =blink_countdown
  LDR R5, [R4]
  AND R5, #0b111
  ADD R5, R5, #8
  LDR R4, =match_led
  STR R5, [R4]

  @ store state 3 in memory
  LDR R5, =3
  LDR R4, =result
  STR R5, [R4]  

.LclearInt:
  LDR   R4, =EXTI_PR                @ Clear (acknowledge) the interrupt
  MOV   R5, #(1<<0)                 @
  STR   R5, [R4]                    @

  @ Return from interrupt handler
  POP  {R4-R8,PC}


  .section .data
  
button_count:
  .space  4

blink_countdown:
  .space  4

LED_cycle:
  .space 4

result:
  .space 4 @ 0 = not decided, 1 = lose, 2 = win, 3 = show match led

blink_period:
  .space 4

match_led:
  .space 4

  .end


  