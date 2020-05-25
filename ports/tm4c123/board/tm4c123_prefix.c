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

// tm4c123_prefix.c becomes the initial portion of the generated pins file.

#include <stdio.h>
#include <stdint.h>

#include "py/mpconfig.h"
#include "py/obj.h"
#include "inc/hw_gpio.h"
#include "inc/hw_memmap.h"
#include "driverlib/pin_map.h"
#include "driverlib/gpio.h"
//#include "pins.h"
#include "pin.h"
#include "mphalport.h"

// The conf variable is not applicable for ADC function, therefore a dummy is defined here
#define GPIO_ADC 0xFFFFFFFF

// For AFs with multiple Units
#define AF_ML(af_name, af_idx, af_fn, af_unit, af_type, af_short, pin_board_name) \
{ \
    .name = MP_QSTR_ ## af_name, \
    .idx = (af_idx), \
    .fn = PIN_FN_ ## af_fn, \
    .unit = (af_unit), \
    .type = AF_ ## af_fn ## _ ## af_type, \
    .conf = GPIO_ ## pin_board_name ## _ ## af_short ## af_type, \
}

// For AFs with only one Unit (not numbered)
#define AF_SL(af_name, af_idx, af_fn, af_unit, af_type, pin_board_name) \
{ \
    .name = MP_QSTR_ ## af_name, \
    .idx = (af_idx), \
    .fn = PIN_FN_ ## af_fn, \
    .unit = (af_unit), \
    .type = AF_ ## af_fn ## _ ## af_type, \
    .conf = GPIO_ ## pin_board_name ## _ ## af_type, \
}

// For analog AFs (no conf)
#define AF_AN(af_name, af_idx, af_fn, af_unit, af_type) \
{ \
    .name = MP_QSTR_ ## af_name, \
    .idx = (af_idx), \
    .fn = PIN_FN_ ## af_fn, \
    .unit = (af_unit), \
    .type = AF_ ## af_fn ## _ ## af_type, \
    .conf = GPIO_ADC, \
} 

#define PIN(p_pin_name, p_port, p_port_pin, p_pin_num, p_af_list, p_def_af, p_num_afs) \
{ \
    { &pin_type }, \
    .name           = MP_QSTR_ ## p_pin_name, \
    .gpio           = GPIO_PORT ## p_port ## _AHB_BASE, \
    .periph         = SYSCTL_PERIPH_GPIO ## p_port, \
    .regs           = (periph_gpio_t*)GPIO_PORT ## p_port ## _AHB_BASE, \
    .af_list        = (p_af_list), \
    .type           = GPIO_PIN_TYPE_STD, \
    .pin_mask       = GPIO_PIN_ ## p_port_pin, \
    .pin_num        = (p_pin_num), \
    .af             = (p_def_af), \
    .drive          = GPIO_STRENGTH_2MA, \
    .dir            = GPIO_DIR_MODE_IN, \
    .num_afs        = (p_num_afs), \
    .value          = 0, \
    .used           = false, \
    .irq_trigger    = 0, \
    .irq_flags      = 0, \
}