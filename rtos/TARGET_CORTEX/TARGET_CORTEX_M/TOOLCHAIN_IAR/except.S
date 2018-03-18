;/*
; * Copyright (c) 2014-2018 ARM Limited. All rights reserved.
; *
; * SPDX-License-Identifier: Apache-2.0
; *
; * Licensed under the Apache License, Version 2.0 (the License); you may
; * not use this file except in compliance with the License.
; * You may obtain a copy of the License at
; *
; * www.apache.org/licenses/LICENSE-2.0
; *
; * Unless required by applicable law or agreed to in writing, software
; * distributed under the License is distributed on an AS IS BASIS, WITHOUT
; * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
; * See the License for the specific language governing permissions and
; * limitations under the License.
; *
; * -----------------------------------------------------------------------------
; *
; * Title:       Cortex-M Fault Exception handlers ( Common for both ARMv7M and ARMV6M ); 
; *
; * -----------------------------------------------------------------------------
; */

                NAME    except.S
                
FAULT_TYPE_HARD_FAULT           EQU      0x10
FAULT_TYPE_MEMMANAGE_FAULT      EQU      0x20
FAULT_TYPE_BUS_FAULT            EQU      0x30
FAULT_TYPE_USAGE_FAULT          EQU      0x40

#ifndef MBED_FAULT_HANDLER_DISABLED
                
#ifndef __DOMAIN_NS 
#define __DOMAIN_NS 1
#endif
                PRESERVE8
                SECTION .rodata:DATA:NOROOT(2)
                
                THUMB
                SECTION .text:CODE:NOROOT(2)

HardFault_Handler
                EXPORT  HardFault_Handler
                LDR     R2,=FAULT_TYPE_HARD_FAULT   
                B       Fault_Handler
                
MemManage_Handler
                EXPORT  MemManage_Handler
                LDR     R2,=FAULT_TYPE_MEMMANAGE_FAULT   
                B       Fault_Handler

BusFault_Handler
                EXPORT  BusFault_Handler
                LDR     R2,=FAULT_TYPE_BUS_FAULT   
                B       Fault_Handler
                
UsageFault_Handler
                EXPORT  UsageFault_Handler
                LDR     R2,=FAULT_TYPE_USAGE_FAULT   
                B       Fault_Handler                
                
Fault_Handler
                EXPORT  Fault_Handler
#if (__DOMAIN_NS == 1)                    
                IMPORT  mbed_fault_handler    
                
		        MRS     r0, msp
		        MRS     r1, psp
		        TST     lr, #(1<<2)
            	ITE     EQ
		        MOVEQ   r3, r0
		        MOVNE   r3, r1
		        STMDB   r3!,{r0,r1,r4-r11}  ; push extra context 
		
		        IT      EQ                  ; if uses msp on return update sp
		        MOVEQ   sp, r3

                PUSH    {lr}
        
                ; prepare arguments
                MOV     r0, r3
		        UBFX    r1, lr, 4, 1
            	; r2 is already set
		        BL	    mbed_fault_handler
		
                POP     {lr}
		
		        LDMIA   r3!,{r0,r1,r4-r11}
		        BX      LR
#endif                

#endif                                            ; #if (MBED_FAULT_HANDLER_SUPPORT == 1) 

                END
