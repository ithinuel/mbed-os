#include "mbed_rtx_fault_handler.h"

#ifndef __ICCARM__

__attribute((naked)) void Fault_Handler(void);

// use of naked function because we dont want to change the stack layout at this point
// They kind of act like a veneer (yet passing an argument)
__attribute((naked)) void HardFault_Handler(void) {
    __asm("MOV r2, %0\n"
          "B   Fault_Handler\n"
          ::"n" (HARD_FAULT_EXCEPTION));
}
__attribute((naked)) void MemManage_Handler(void) {
    __asm("MOV r2, %0\n"
          "B   Fault_Handler\n" // Fault_Handler will return for us
          :: "n" (MEMMANAGE_FAULT_EXCEPTION));
}
__attribute((naked)) void BusFault_Handler(void) {
    __asm("MOV r2, %0\n"
          "B   Fault_Handler\n" // Fault_Handler will return for us
          :: "n" (BUS_FAULT_EXCEPTION)); 
}
__attribute((naked)) void UsageFault_Handler(void) {
    __asm("MOV r2, %0\n"
          "B   Fault_Handler\n" // Fault_Handler will return for us
          :: "n" (USAGE_FAULT_EXCEPTION)); 
}

__attribute((naked)) void Fault_Handler(void)
{
	__asm(
	    /* save addtionnal context to the stack */
		"MRS	r0, msp\n"		/* R0<-MSP */
		"MRS	r1, psp\n"		/* R1<-PSP */
		"TST    lr, #(1<<2)\n"  /* eq if uses msp on return */
		"ITE    EQ\n"
		"MOVEQ  r3, r0\n"
		"MOVNE  r3, r1\n"
		"STMDB  r3!,{r0,r1,r4-r11}\n" /* push missing registers */
		
		"IT     EQ\n"           /* if uses msp on return update sp */
		"MOVEQ  sp, r3\n"

        "PUSH   {lr}\n"
        
        /* prepare arguments */
        "MOV    r0, r3\n"
		"UBFX   r1, lr, 4, 1\n"
		/* r2 is already set on first instr */
		"BL	    mbed_fault_handler\n"
		
		/* restore stack */
        "POP    {lr}\n"
		
		"LDMIA  r3!,{r0,r1,r4-r11}\n"
		"BX     lr\n");
}

#endif
