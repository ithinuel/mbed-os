/* mbed Microcontroller Library
 * Copyright (c) 2006-2018 ARM Limited
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
#include <stdint.h>
#include <stdbool.h>

//Fault type definitions
//WARNING: DO NOT CHANGE THESE VALUES WITHOUT MAKING CORRESPONDING CHANGES in except.S files.
typedef enum {
    HARD_FAULT_EXCEPTION = 0x10, //Keep some gap between values for any future insertion/expansion
    MEMMANAGE_FAULT_EXCEPTION = 0x20,
    BUS_FAULT_EXCEPTION = 0x30,
    USAGE_FAULT_EXCEPTION = 0x40,
} mbed_fault_type_t;

// Extra context to push in addition to the automatically saved one.
typedef struct {
    uint32_t MSP;
    uint32_t PSP;
    uint32_t R4;
    uint32_t R5;
    uint32_t R6;
    uint32_t R7;
    uint32_t R8;
    uint32_t R9;
    uint32_t R10;
    uint32_t R11;
} mbed_fault_extra_context_t;

// Basic context saved upon entering exception.
typedef struct {
    uint32_t R0;
    uint32_t R1;
    uint32_t R2;
    uint32_t R3;
    uint32_t R12;
    uint32_t LR;
    uint32_t PC;
    uint32_t xPSR;
} mbed_fault_context_t;

// Extra context saved for fpu registers.
typedef struct {
    uint32_t S0;
    uint32_t S1;
    uint32_t S2;
    uint32_t S3;
    uint32_t S4;
    uint32_t S5;
    uint32_t S6;
    uint32_t S7;
    uint32_t S8;
    uint32_t S9;
    uint32_t S10;
    uint32_t S11;
    uint32_t S12;
    uint32_t S13;
    uint32_t S14;
    uint32_t S15;
    uint32_t FPSCR;
    uint32_t reserved;
} mbed_fault_fpu_context_t;


//This is a handler function called from Fault handler to print the error information out.
//This runs in fault context and uses special functions(defined in mbed_rtx_fault_handler.c) to print the information without using C-lib support.
void mbed_fault_handler(uint32_t *mbed_fault_context_in, bool fpu_stacked, mbed_fault_type_t fault_type);

