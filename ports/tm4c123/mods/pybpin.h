/*
 * This file is part of the MicroPython project, http://micropython.org/
 *
 * The MIT License (MIT)
 *
 * Copyright (c) 2013, 2014 Damien P. George
 * Copyright (c) 2015 Daniel Campora
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
#ifndef MICROPY_INCLUDED_TM4C123_MODS_PYBPIN_H
#define MICROPY_INCLUDED_TM4C123_MODS_PYBPIN_H

#include "py/runtime.h"
#include "py/gc.h"
#include "py/qstr.h"

//#include "inc/tm4c123gh6pm.h"
#include "inc/hw_types.h"
#include "inc/hw_gpio.h"
#include "inc/hw_ints.h"
#include "inc/hw_memmap.h"
#include "inc/hw_hibernate.h"
#include "driverlib/sysctl.h"
#include "driverlib/pin_map.h"
#include "driverlib/gpio.h"
#include "driverlib/rom_map.h"
// #include "prcm.h"
#include "driverlib/interrupt.h"

#include "misc/mpirq.h"
#include "pybsleep.h"
#include "misc/mpexception.h"
#include "misc/mperror.h"

enum {
    PORT_A = GPIO_PORTA_AHB_BASE,
    PORT_B = GPIO_PORTB_AHB_BASE,
    PORT_C = GPIO_PORTC_AHB_BASE,
    PORT_D = GPIO_PORTD_AHB_BASE,
    PORT_E = GPIO_PORTE_AHB_BASE,
    PORT_F = GPIO_PORTF_AHB_BASE,
};

enum {
    PIN_FN_ADC = 0,
    PIN_FN_UART,
    PIN_FN_SSI,
    PIN_FN_I2C,
    PIN_FN_QEI,
    PIN_FN_MTRL,
    PIN_FN_WTIM,
    PIN_FN_TIM,
    PIN_FN_CAN,
    PIN_FN_USB,
    PIN_FN_COMP,
    PIN_FN_TR = 14,
};

enum {
    PIN_TYPE_UART_TX = 0,
    PIN_TYPE_UART_RX,
    PIN_TYPE_UART_RTS,
    PIN_TYPE_UART_CTS,
};

enum {
    PIN_TYPE_SSI_CLK = 0,
    PIN_TYPE_SSI_TX,
    PIN_TYPE_SSI_RX,
    PIN_TYPE_SSI_FSS,
};

enum {
    PIN_TYPE_I2C_SDA = 0,
    PIN_TYPE_I2C_SCL,
};

enum {
    PIN_TYPE_TIM_CCP0 = 0,
    PIN_TYPE_TIM_CCP1
};

enum {
    PIN_TYPE_WTIM_CCP0 = 0,
    PIN_TYPE_WTIM_CCP1
};

enum {
    PIN_TYPE_QEI_PHA0 = 0,
    PIN_TYPE_QEI_PHA1,
    PIN_TYPE_QEI_PHB0,
    PIN_TYPE_QEI_PHB1,
    PIN_TYPE_QEI_IDX0,
    PIN_TYPE_QEI_IDX1
};

enum {
    PIN_TYPE_USB_EPEN = 0,
    PIN_TYPE_USB_PFLT,
    PIN_TYPE_USB_DM,
    PIN_TYPE_USB_DP,
    PIN_TYPE_USB_ID,
    PIN_TYPE_USB_VBUS
};

enum {
    PIN_TYPE_COMP_POS = 0,
    PIN_TYPE_COMP_NEG,
    PIN_TYPE_COMP_OUT
};

enum {
    PIN_TYPE_CAN_TX = 0,
    PIN_TYPE_CAN_RX
};

enum {
    PIN_TYPE_TR_CLK = 0,
    PIN_TYPE_TR_D0,
    PIN_TYPE_TR_D1
};

enum {
    PIN_TYPE_MTRL_FAULT0 = 0,
    PIN_TYPE_MTRL_PWM0 = 0,
    PIN_TYPE_MTRL_PWM1 = 0,
    PIN_TYPE_MTRL_PWM2 = 0,
    PIN_TYPE_MTRL_PWM3 = 0,
    PIN_TYPE_MTRL_PWM4 = 0,
    PIN_TYPE_MTRL_PWM5 = 0,
    PIN_TYPE_MTRL_PWM6 = 0,
    PIN_TYPE_MTRL_PWM7 = 0,
};

enum {
    PIN_TYPE_ADC_AIN0 = 0,
    PIN_TYPE_ADC_AIN1,
    PIN_TYPE_ADC_AIN2,
    PIN_TYPE_ADC_AIN3,
    PIN_TYPE_ADC_AIN4,
    PIN_TYPE_ADC_AIN5,
    PIN_TYPE_ADC_AIN6,
    PIN_TYPE_ADC_AIN7,
    PIN_TYPE_ADC_AIN8,
    PIN_TYPE_ADC_AIN9,
    PIN_TYPE_ADC_AIN10,
    PIN_TYPE_ADC_AIN11,
};

typedef struct {
  qstr name;
  int8_t  idx;
  uint8_t fn;
  uint8_t unit;
  uint8_t type;
} pin_af_t;

typedef struct {
    const mp_obj_base_t base;
    const qstr          name;
    const uint32_t      port;
    const pin_af_t      *af_list;
    uint16_t            pull;
    const uint8_t       bit;
    const uint8_t       pin_num;
    int8_t              af;
    uint8_t             strength;
    uint8_t             mode;        // this is now a combination of type and mode
    const uint8_t       num_afs;     // 255 AFs
    uint8_t             value;
    uint8_t             used;
    uint8_t             irq_trigger;
    uint8_t             irq_flags;
} pin_obj_t;

extern const mp_obj_type_t pin_type;

typedef struct {
    const char *name;
    const pin_obj_t *pin;
} pin_named_pin_t;

typedef struct {
    mp_obj_base_t base;
    qstr name;
    const pin_named_pin_t *named_pins;
} pin_named_pins_obj_t;

extern const mp_obj_type_t pin_board_pins_obj_type;
extern const mp_obj_dict_t pin_board_pins_locals_dict;

void pin_init0(void);
void pin_config(pin_obj_t *self, int af, uint mode, uint type, int value, uint strength);
pin_obj_t *pin_find(mp_obj_t user_obj);
void pin_assign_pins_af (mp_obj_t *pins, uint32_t n_pins, uint32_t pull, uint32_t fn, uint32_t unit);
uint8_t pin_find_peripheral_unit (const mp_obj_t pin, uint8_t fn, uint8_t type);
uint8_t pin_find_peripheral_type (const mp_obj_t pin, uint8_t fn, uint8_t unit);
int8_t pin_find_af_index (const pin_obj_t* pin, uint8_t fn, uint8_t unit, uint8_t type);;

#endif // MICROPY_INCLUDED_TM4C123_MODS_PYBPIN_H
