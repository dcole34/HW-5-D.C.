# Including xc.h allows use of SFR names, bit masks, etc.
#include <xc.h>

    # Interrupt vector setup    
    .globl  __vector_dispatch_11    # Timer 1 interrupt = vector #11
    .section .vector_11, code, keep
    .align 2
    .set nomips16
    .ent __vector_dispatch_11
__vector_dispatch_11:
    j  isrvector11	    # Jump to actual ISR code, which is in text section
    nop
    .end __vector_dispatch_11
    .size __vector_dispatch_11, .-__vector_dispatch_11
    .globl  __vector_dispatch_9    # Port B change notification = vector #9
    .section .vector_9, code, keep
    .align 2
    .set nomips16
    .ent __vector_dispatch_9
__vector_dispatch_9:
    j  isrvector9	    # Jump to actual ISR code, which is in text section
    nop
    .end __vector_dispatch_9
    .size __vector_dispatch_9, .-__vector_dispatch_9
    
    # Start of text section       
    .text		# Text section contains code
    .set noreorder	# Disable instruction reordering
    .globl main		# Define main label as a global entity
    .ent main		#   and an entry point    

main:
    
    # Configure port A for output
    sw	    zero, TRISA		    # TRISA = 0  --> all PORTA bits = output
    sw	    zero, ANSELA	    # ANSELA = 0 --> all PORTA bits digital
    sw	    zero, ANSELB	    # ANSELB = 0 --> all PORTB bits digital 
    li	    t0, _PORTB_RB7_MASK
    ori	    t0, t0, _PORTB_RB13_MASK
    sw	    t0, TRISB		    # TRISB = 0x00002080 --> pins 7 & 13 inputs
 
    sw	    zero, TRISC		    # TRISC = 0  --> all PORTC bits = output
    sw	    zero, ANSELC	    # ANSELC = 0 --> all PORTC bits digital

    li	    t0, _PORTA_RA0_MASK	    # t0 = 0x00000001 --> control LED1
    li	    t1, _PORTC_RC9_MASK	    # t1 = 0x00000200 --> control LED2
    li	    t4, _PORTB_RB7_MASK	    # t4 = 0x00000080 --> mask for S1
    li	    t6, _PORTB_RB13_MASK    # t6 = 0x00002000 --> mask for S2
    
    # Configure Timer 1
    sw	    zero, T1CON		    # Clear T1CON--disables timer to allow setup
    
    ori	    t3, zero, 0x0FFF	    # Set t1 = 0xFFFF = initial PR1 value
    sw	    t3, PR1		    #   (maximum possible clock period--65,535 cycles)
    
    li	    t2, _IFS0_T1IF_MASK	    # t2 = bit mask for checking Timer 1 interrupt flag
    
    # Prescale clock
    li	    t3, 0x00000030  ## Prescale by factor of 64 (TCKPS = 10) BITMASK = 0x00000020
    sw	    t3, T1CONSET
    
    # Configure time interrupts
    lui	    t3, 0x0001			# Want INTCON bit 16 (VS<0>) = 1
					#   so vectors 8 bytes apart
    ori	    t3, t3, _INTCON_MVEC_MASK   # Enable multivectored interrupt mode
    sw	    t3, INTCON
    
    li	    t3, _IPC2_T1IP_MASK	    # Set T1 interrupt priority level to 7
    sw	    t3, IPC2		    # Implicitly sets subpriority level to 0
    
    sw	    t2, IEC0		    # Enable Timer 1 interrupts (uses
				    #   same bit mask as T1 interrupt flag)
				    
				    
    li	    t3, _T1CON_TON_MASK	    # Enable Timer 1 by setting "ON" bit in T1CON
    sw	    t3, T1CONSET
    
        # Configure button interrupts
    lui	    t3, 0x0001			# Want INTCON bit 16 (VS<0>) = 1
					#   so vectors 8 bytes apart
    ori	    t3, t3, _INTCON_MVEC_MASK   # Enable multivectored interrupt mode
    lw	    t2, INTCON
    or	    t3,t2,t3
    sw	    t3, INTCON
    
    li	    t3, _IPC2_CNBIP_MASK    # Set change notification Port B interrupt priority level to 7
    lw	    t2, IPC2
    or	    t3,t2,t3
    sw	    t3, IPC2		    # Implicitly sets subpriority level to 0
    
    li	    t3, _IEC0_CNBIE_MASK    # Enable Port B change 
    lw	    t2, IEC0
    or	    t3,t2,t3
    sw	    t3, IEC0		    #    notification interrupts
    add	    t3, t4, zero	    # Set bits in CNEN1B = 1 and CNEN0B = 0
    or	    t3, t3, t6		    #   corresponding to switch positions
    sw	    t3, CNEN1B		    #   (t4 = S1 bit mask; t6 = S2 bit mask)
    sw	    zero, CNEN0B	    # Will detect falling edges on these pins
    
    li	    t3, _CNCONB_ON_MASK		    # Enables Port B change notification
    ori	    t3, t3, _CNCONB_CNSTYLE_MASK    # Enables edge detection
    sw	    t3, CNCONB
    
    # set mode
    li	    s1, 0	# mode 0
    li	    s2, 0	# mode period
    
    ei				   # Enable interrupts globally
    
    # Main loop--waiting for interrupt
mainloop:
    j	mainloop
    nop
    .end main
    
   .global delay
    .ent delay
delay:
    li	    t7, 0x61A8		    # Set delay counter to 0x61A8 = 25,000
				    # Since loop body has 3 instructions,
				    #   loop takes 25,000 * 3 = 75,000
				    #   cycles
				    # Remaining 3 instructions take 3 cycles
				    #  ~75,000 cycles / 8 MHz clock ~ 0.009375 sec delay
loop:
    addi    t7, t7, -1		    # Decrement counter
    bne	    t7, zero, loop	    #  and continue doing that until we hit 0
    nop				    
    jr	    ra
    nop

    .end delay
    
    # Handle Timer1 interrupt--clear interrupt flag and toggle LED
    .global isrvector11
    .ent isrvector11
isrvector11:
    # check modee
    li	    t4, _PORTB_RB7_MASK	    # t4 = 0x00000080 --> mask for S1
    sw	    t4, CNFBCLR		    # Clear flag for S1
    li	    s0, 0
    beq	    s1, s0, s1state0 # if the state is  0
    nop
    addi    s0,s0,1 # increment s0 after checking for state 0
    
    
    beq	    s1, s0, s1state1 # if the state is  1
    nop
    addi    s0,s0,1 # increment s0 after checking for state 1
    
    beq	    s1, s0, s1state2 # if the state is  2
    nop
    addi    s0,s0,1 # increment s0 after checking for state 2
    
    beq	    s1, s0, s1state3 # if the state is  3
    nop
    addi    s0,s0,1 # increment s0 after checking for state 3
    
    beq	    s1, s0, s1state4 # if the state is  4
    nop
    addi    s0,s0,1 # increment s0 after checking for state 4

s1state0:
    sw	    t0, LATAINV
    sw	    t1, LATCINV
    j	    timerdone
    nop
    
    
s1state1:
    sw	    t0, LATAINV
    sw	    t1, LATCCLR
    j	    timerdone
    nop   
 
s1state2:
    sw	    t0, LATAINV
    sw	    t1, LATCINV
    j	    timerdone
    nop   
    
s1state3:
    sw	    t0, LATACLR
    sw	    t1, LATCCLR
    j	    timerdone
    nop   
    
s1state4:
    sw	    t0, LATACLR
    sw	    t1, LATCINV
    
timerdone:
    li	    t2, _IFS0_T1IF_MASK    
    sw	    t2, IFS0CLR		    # Clear T1IF in software!

    eret		    # Return from interrupt
    .end isrvector11

    .global isrvector9
    .ent isrvector9
isrvector9:   
    li	    t4, _PORTB_RB7_MASK	    # t4 = 0x00000080 --> mask for S1
    li	    t6, _PORTB_RB13_MASK    # t6 = 0x00002000 --> mask for S2
    
    # Check S1
    lw	    t8, CNFB
    and	    t9, t8, t4
    beq	    t9, zero, checkS2	    # If bit 7 = 0, S1 wasn't pressed
    nop
    
    # S1 pressed--clear flag, then debounce and toggle if actually pressed
    sw	    t4, CNFBCLR		    # Clear flag for S1
    jal	    delay		    # Delay to debounce
    nop
    lw	    t2, PORTB		    
    and	    t2, t2, t4		    # Is button still pressed?
    bne	    t2, zero, checkS2	    # If not, leave LED alone and check S2
    nop
    
    # at this point we know the switch is pressed:
    li	    s5, 1		    # variable for whenever i want to use "1" 
    addi    s1, s1, 1 # increment mode by 1
    li	    s4, 4 # dummy variable 5 because the last mode for switch 1 is 4 so 
		#  we want to change mode to 1 when the switch is pressed during the 4th mode
    sw	    t0, LATACLR
    sw	    t1, LATCCLR
    sgt	    s3, s1, s4 # dummy variable s3 (flag) checks if the current mode is = 5 (boolean result 1 or 0)
    beq	    s3, s5, restarts1 # checks if flag is true if it is then it jumps to
    nop			    # the branch that will force the mode to be "1"
    j	    checkS2
    nop
restarts1:
    li	    s1, 1 # changes s1 mode back to 1	
 
checkS2:
    li	    t4, _PORTB_RB7_MASK	    # t4 = 0x00000080 --> mask for S1
    li	    t6, _PORTB_RB13_MASK    # t6 = 0x00002000 --> mask for S2
    li	    s5, 1		    # variable for whenever i want to use "1"
    lw	    t8, CNFB
    and	    t9, t8, t6
    beq	    t9, zero, intdone	    # If bit 13 = 0, S2 wasn't pressed
    nop
    
    # S2 pressed--clear flag, then debounce and toggle if actually pressed
    sw	    t6, CNFBCLR		    # Clear flag for S2
    jal	    delay		    # Delay to debounce
    nop
    lw	    t2, PORTB		    
    and	    t2, t2, t6		    # Is button still pressed?
    bne	    t2, zero, intdone	    # If not, leave LED alone and check S2
    nop
    
    
    sw	    t0, LATACLR
    sw	    t1, LATCCLR
    addi    s2, s2, 1 # increment mode by 1
    li	    s4, 5 # dummy variable 5 because the last mode for switch 1 is 4 so 
		# we want to change mode to 1 when the switch is pressed during the 4th mode
    sgt	    s3, s2, s4 # dummy variable s3 (flag) checks if the current mode is = 5 (boolean result 1 or 0)
    beq	    s3, s5, endprogram # checks if flag is true if it is then it jumps to
    nop			    # the branch that will force the mode to be "1"
    
    
    li	    s6, 0
    beq	    s2, s6, s2state0 # if the state is  0
    nop
    addi    s6,s6,1 # increment s0 after checking for state 0
    
    
    beq	    s2, s6, s2state1 # if the state is  1
    nop
    addi    s6,s6,1 # increment s0 after checking for state 1
    
    beq	    s2, s6, s2state2 # if the state is  2
    nop
    addi    s6,s6,1 # increment s0 after checking for state 2
    
    beq	    s2, s6, s2state3 # if the state is  3
    nop
    addi    s6,s6,1 # increment s0 after checking for state 3
    
    beq	    s2, s6, s2state4 # if the state is  4
    nop
    addi    s6,s6,1 # increment s0 after checking for state 4
    
    beq	    s2, s6, s2state5 # if the state is  5
    nop
    addi    s6,s6,1 # increment s0 after checking for state 5


s2state0:
    j	    intdone
    nop
    
    
s2state1:
    ori	    t3, zero, 0x1FFE	    # Set t1 = 0xFFFF = initial PR1 value
    sw	    t3, PR1		    #   (maximum possible clock period--65,535 cycles)
    j	    intdone
    nop   
 
s2state2:
    ori	    t3, zero, 0x3FFC	    # Set t1 = 0xFFFF = initial PR1 value
    sw	    t3, PR1		    #   (maximum possible clock period--65,535 cycles)
    j	    intdone
    nop   
    
s2state3:
    ori	    t3, zero, 0x7FF8	    # Set t1 = 0xFFFF = initial PR1 value
    sw	    t3, PR1		    #   (maximum possible clock period--65,535 cycles)
    j	    intdone
    nop   
    
s2state4:
    ori	    t3, zero, 0xFFF0	    # Set t1 = 0xFFFF = initial PR1 value
    sw	    t3, PR1		    #   (maximum possible clock period--65,535 cycles)
    j	    intdone
    nop
    
s2state5:
    ori	    t3, zero, 0x0FFF	    # Set t1 = 0xFFFF = initial PR1 value
    sw	    t3, PR1		    #   (maximum possible clock period--65,535 cycles)
    j	    intdone
    nop
    
endprogram:
    li	    t3, _T1CON_TON_MASK	    # Enable Timer 1 by setting "ON" bit in T1CON
    sw	    t3, T1CONCLR
    li	    t2, 0x00000000
    sw	    t2, CNCONB
    j	endprogram
    nop
    
intdone:
    li	    t3, _IFS0_CNBIF_MASK    # Clear Port B change notification flag
    sw	    t3, IFS0CLR		    #    in IFS0
    eret		    # Return from interrupt
    .end isrvector9


