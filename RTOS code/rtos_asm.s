; RTOS asm file
; Jason Losh
;Student Names:  Surabhi Chythanya Kumar 
;-----------------------------------------------------------------------------
; Hardware Target
;-----------------------------------------------------------------------------

; Target Platform: EK-TM4C123GXL Evaluation Board
; Target uC:       TM4C123GH6PM
; System Clock:    40 MHz

; Hardware configuration:
; 6 Pushbuttons and 5 LEDs, UART
; LEDs on these pins:
; Blue:   PF2 (on-board)
; Red:    PA2
; Orange: PA3
; Yellow: PA4
; Green:  PE0
; PBs on these pins
; PB0:    PC4
; PB1:    PC5
; PB2:    PC6
; PB3:    PC7
; PB4:    PD6
; PB5:    PD7
; UART Interface:
;   U0TX (PA1) and U0RX (PA0) are connected to the 2nd controller
;   The USB on the 2nd controller enumerates to an ICDI interface and a virtual COM port
;   Configured to 115,200 baud, 8N1

;-----------------------------------------------------------------------------
; Device includes, defines, and assembler directives
;-----------------------------------------------------------------------------
   .def setTMPL
   .def setPSP
   .def setPSP1
   .def pushR4_11_PSP
   .def popR4_11_PSP
   .def getPSP
   .def getR0
   .def pushR0_xPSR
   .def exc_ret
   .def getR2
   .def getR3
   .def getR12
   .def getLR
   .def getPC
   .def getxPSR
   .def getR1
   .def get_MSP
   .def get_PSP




;-----------------------------------------------------------------------------
; Register values and large immediate values
;-----------------------------------------------------------------------------

.thumb


;-----------------------------------------------------------------------------
; Subroutines
;-----------------------------------------------------------------------------

.text
; function that sets PSP
setPSP:

         MSR PSP,R0
         MRS R5,CONTROL
         ORR R5,#2h
         MSR CONTROL,R5


         ISB
         BX LR


setPSP1:

         MSR PSP,R0
         BX LR

setTMPL:
        MRS R1, CONTROL
		ORR R1 , #1
		MSR CONTROL , R1
		BX LR

pushR4_11_PSP:
       MRS R0,PSP
       SUB R0,R0,#4
       STR R4,[R0]

       SUB R0,R0,#4
       STR R5,[R0]

       SUB R0,R0,#4
       STR R6,[R0]

       SUB R0,R0,#4
       STR R7,[R0]

       SUB R0,R0,#4
       STR R8,[R0]

       SUB R0,R0,#4
       STR R9,[R0]

       SUB R0,R0,#4
       STR R10,[R0]

       SUB R0,R0,#4
       STR R11,[R0]

       MSR PSP,R0
       BX LR

popR4_11_PSP:
       MRS R0,PSP
       LDR R11,[R0]
       ADD R0,R0,#4


       LDR R10,[R0]
       ADD R0,R0,#4

       LDR R9,[R0]
       ADD R0,R0,#4

       LDR R8,[R0]
       ADD R0,R0,#4

       LDR R7,[R0]
       ADD R0,R0,#4

       LDR R6,[R0]
       ADD R0,R0,#4

       LDR R5,[R0]
       ADD R0,R0,#4

       LDR R4,[R0]
       ADD R0,R0,#4

       MSR PSP,R0
       BX LR

getPSP:
       MRS R0,PSP
       BX LR


getR0:
       MRS R0,PSP
       LDR R0,[R0]
       BX LR

pushR0_xPSR:
       .thumb
       MOV R4,R0  ;storing PC that is in R0 to R4

       MRS R0,PSP
       SUB R0,R0,#4

       MOV R2,#0
       ORR R2, R2,#0x01000000
       STR R2,[R0]  ;XPSR


       SUB R0,R0,#4
        STR R4,[R0]  ;PC


       SUB R0,R0,#4
       STR LR,[R0]  ;LR


       SUB R0,R0,#4
       STR R12,[R0]  ;R12


       SUB R0,R0,#4
        STR R3,[R0]  ;R3


      SUB R0,R0,#4
      STR R2,[R0]  ;R2


       SUB R0,R0,#4
       STR R1,[R0]  ;R1


       SUB R0,R0,#4
      STR R0,[R0]  ;R0

       MSR PSP,R0
       BX LR

exc_ret:

     MVN LR,#2H;fdh
     BX LR



getR1:
     MRS R0, PSP
     ADD R0,#4
     LDR R0,[R0]
     BX  LR

get_MSP:
     MRS R0,MSP
     BX  LR

getR2:
     MRS R0, PSP
     ADD R0,#8
     LDR R0,[R0]
     BX  LR

getR3:
     MRS R0, PSP
     ADD R0,#12
     LDR R0,[R0]
     BX  LR

getR12:
     MRS R0, PSP
     ADD R0,#16
     LDR R0,[R0]
     BX  LR

getLR:
     MRS R0, PSP
     ADD R0,#20
     LDR R0,[R0]
     BX  LR

getPC:
     MRS R0, PSP
     ADD R0,#24
     LDR R0,[R0]
     BX  LR

getxPSR:
     MRS R0, PSP
     ADD R0,#28
     LDR R0,[R0]
     BX  LR

get_PSP:
       MRS R0,PSP
       BX  LR
.endm
