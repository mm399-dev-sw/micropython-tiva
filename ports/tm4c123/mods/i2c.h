/*
 * This file is part of the MicroPython project, http://micropython.org/
 *
 * The MIT License (MIT)
 *
 * Copyright (c) 2013, 2014 Damien P. George
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

#ifndef MICROPY_INCLUDED_TM4C_I2C_H
#define MICROPY_INCLUDED_TM4C_I2C_H

#include <stdint.h>
#include <stdio.h>
#include <stdarg.h>
#include <stdbool.h>
#include "inc/hw_i2c.h"
#include "driverlib/i2c.h"
#include "mphalport.h"

// defines
#define I2C_MODE_MASTER 0x00000000
#define I2C_MODE_SLAVE  0x00000001

#define I2C_SEND_START  false
#define I2C_SEND_STOP   true

// Control Register Op Flags
#define I2C_MASTER_CMD_STOP     0x00000004 // 1: controller generates STOP - 0: conroller does not generate STOP
#define I2C_MASTER_CMD_START    0x00000002 // 1: controller generates START or REPEADTED START - 0: controller does not generate START
#define I2C_MASTER_CMD_RUN      0x00000001 // 1: Master enable (starts i2c communication) - 0: Master disable
#define I2C_MASTER_CMD_ACK      0x00000008 // 1: Master ACK automatically - 0: Master ACK not automatically

// variables
extern const mp_obj_type_t machine_hard_i2c_type;

// enums
typedef enum
{
    I2C_NONE = 4,
    I2C_0 = 0,
    I2C_1 = 1,
    I2C_2 = 2,
    I2C_3 = 3,
} i2c_id_t;

// structs
// registers
typedef struct
{
    // Master Registers
    volatile uint32_t MSA;      // Master Slave Address; Offset: 0x000
    volatile uint32_t MCS;      // Master Control Status; 0x004
    volatile uint32_t MDR;      // Master Data; 0x008
    volatile uint32_t MTPR;     // Master Timer Period; 0x00C
    volatile uint32_t MIMR;     // Master Interrupt Mask; 0x010
    volatile uint32_t MRIS;     // Master Raw Interrupt Status; 0x014
    volatile uint32_t MMIS;     // Master Masked Interrupted Status; 0x018
    volatile uint32_t MICR;     // Master Interrupt Clear; 0x01C
    volatile uint32_t MCR;      // Master Configuration; 0x020
    volatile uint32_t MCLKOCNT; // Master Clock Low Timout Count; 0x024
    volatile uint32_t MBMON;    // Master Bus Monitor; 0x02C
    volatile uint32_t MCR2;     // Master Configuration 2; 0x038
} periph_i2c_master_t;

typedef struct
{
    // Slave Registers
    volatile uint32_t SOAR;    // Slave Own Address; 0x800
    volatile uint32_t SCSR;    // Slave Control/Status; 0x804
    volatile uint32_t SDR;     // Slave Data; 0x808
    volatile uint32_t SIMR;    // Slave Interrupt Mask; 0x80C
    volatile uint32_t SRIS;    // Slave Raw Interrupt Status; 0x810
    volatile uint32_t SMIS;    // Slave Masked Interrupt Status; 0x814
    volatile uint32_t SICR;    // Slave Interrupt Clear; 0x818
    volatile uint32_t SOAR2;   // Slave Own Address 2; 0x81C
    volatile uint32_t SACKCTL; // Slave Acknowledge Control; 0x820
} periph_i2c_slave_t;

typedef struct
{
    // Status & Control
    uint32_t _1[1000];
    volatile uint32_t PP; // Peripheral Properties; 0xFC0
    volatile uint32_t PC; // Peripheral Configuration; 0xFC4
} periph_i2c_stctl_t;

// i2c object
typedef struct _machine_hard_i2c_obj_t {
    mp_obj_base_t base;
    uint16_t mode : 2;
    uint32_t i2c_base;
    uint32_t periph;
    uint32_t baudrate;
    bool dma_flag;
    const pin_obj_t *pin_SDA;
    const pin_obj_t *pin_SCL;
    periph_i2c_master_t *master_regs;
    periph_i2c_slave_t *slave_regs;
    periph_i2c_stctl_t *status_control;
    uint32_t irqn;
    i2c_id_t i2c_id : 3;
    bool is_enabled : 1;
    uint8_t num_of_acks;
    uint32_t err_reg;
} machine_hard_i2c_obj_t;

void i2c_init0(void);
void i2c_init(machine_hard_i2c_obj_t *self_in);
void i2c_deinit(const mp_obj_t *self_in);

void i2c_master_tx(mp_obj_t *self_in, uint8_t device_address, bool address_flag, uint16_t mem_loc, bool mem_loc_flag, uint8_t *device_data, size_t size);

#endif // MICROPY_INCLUDED_TM4C_I2C_H
