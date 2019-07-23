/*
 * This file is part of the MicroPython project, http://micropython.org/
 *
 * The MIT License (MIT)
 *
 * Copyright (c) 2013-2019 Damien P. George
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */
#ifndef MICROPY_INCLUDED_STM32_MODMACHINE_H
#define MICROPY_INCLUDED_STM32_MODMACHINE_H

#include "py/obj.h"

void machine_init(void);
void machine_deinit(void);

MP_DECLARE_CONST_FUN_OBJ_VAR_BETWEEN(machine_info_obj);
MP_DECLARE_CONST_FUN_OBJ_0(machine_unique_id_obj);
MP_DECLARE_CONST_FUN_OBJ_0(machine_reset_obj);
MP_DECLARE_CONST_FUN_OBJ_0(machine_bootloader_obj);
MP_DECLARE_CONST_FUN_OBJ_VAR_BETWEEN(machine_freq_obj);
MP_DECLARE_CONST_FUN_OBJ_0(machine_sleep_obj);
MP_DECLARE_CONST_FUN_OBJ_0(machine_deepsleep_obj);

typedef struct {
    volatile uint32_t DID0;
    volatile uint32_t DID1;
    uint32_t _1[10];
    volatile uint32_t PBORCTL;
    uint32_t _2[7];
    volatile uint32_t RIS;
    volatile uint32_t IMC;
    volatile uint32_t MISC;
    volatile uint32_t RESC;
    volatile uint32_t RCC;
    uint32_t _3[2];
    volatile uint32_t GPIOHBCTL;
    volatile uint32_t RCC2;
    uint32_t _4[2];
    volatile uint32_t MOSCCTL;
    uint32_t _5[49];
    volatile uint32_t DSLPCLKCFG;
    uint32_t _6;
    volatile uint32_t SYSPROP;
    volatile uint32_t PIOSCCAL;
    volatile uint32_t PIOSCSTAT;
    uint32_t _7[2];
    volatile uint32_t PLLFREQ0;
    volatile uint32_t PLLFREQ1;
    volatile uint32_t PLLSTAT;
    uint32_t _8[7];                 // 0x16C - 0x187
    uint32_t LPCR[18];              // LPWR Config Registers 0x188 - 0x1CF
    uint32_t _9[76];                // 0x1D0 - 0x2FF
    uint32_t PPR[24];               // Peripheral Present Registers 0x300 - 0x35F
    uint32_t _10[104];              // 0x360 - 0x4FF
    uint32_t PRR[24];               // Peripheral Reset Registers 0x500 - 0x55F
    uint32_t _11[40];               // 0x560 - 0x5FF
    volatile uint32_t RCGCWD;       // Peripheral Run Mode Clock Gating Control begin
    volatile uint32_t RCGCTIMER;
    volatile uint32_t RCGCGPIO;
    volatile uint32_t RCGCDMA;
    uint32_t _12;
    volatile uint32_t RCGCHIB;
    volatile uint32_t RCGCUART;
    volatile uint32_t RCGCSSI;
    volatile uint32_t RCGCI2C;
    uint32_t _13;
    volatile uint32_t RCGCUSB;
    uint32_t _14[2];
    volatile uint32_t RCGCCAN;
    volatile uint32_t RCGCADC;
    volatile uint32_t RCGCACMP;
    volatile uint32_t RCGCPWM;
    volatile uint32_t RCGCQEI;
    uint32_t _15[4];
    volatile uint32_t RCGCEEPROM;
    volatile uint32_t RCGCWTIMER;
    uint32_t _16[40];               // 0x660 - 0x6FF
    uint32_t SCGC[24];              // Peripheral Sleep Mode Clock Gating Control
    uint32_t _17[40];               // 0x760 - 0x7FF
    uint32_t DCGC[24];              // Peripheral Deep-Sleep Mode Clock Gating Control
    uint32_t _18[104];              // 0x860 - 0x9FF
    volatile uint32_t PRWD;         // Peripheral Ready begin
    volatile uint32_t PRTIMER;
    volatile uint32_t PRGPIO;
    volatile uint32_t PRDMA;
    uint32_t _19;
    volatile uint32_t PRHIB;
    volatile uint32_t PRUART;
    volatile uint32_t PRSSI;
    volatile uint32_t PRI2C;
    uint32_t _20;
    volatile uint32_t PRUSB;
    uint32_t _21[2];
    volatile uint32_t PRCAN;
    volatile uint32_t PRADC;
    volatile uint32_t PRACMP;
    volatile uint32_t PRPWM;
    volatile uint32_t PRQEI;
    uint32_t _22[4];
    volatile uint32_t PREEPROM;
    volatile uint32_t PRWTIMER;
} periph_sysctl_t;

#define SYSCTL ((periph_sysctl_t*)  0x400FE000)

#endif // MICROPY_INCLUDED_STM32_MODMACHINE_H
