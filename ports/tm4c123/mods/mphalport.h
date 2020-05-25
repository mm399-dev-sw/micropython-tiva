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

#ifndef MICROPY_INCLUDED_TM4C_MPHALPORT_H
#define MICROPY_INCLUDED_TM4C_MPHALPORT_H
#define MICROPY_MPHALPORT_H "mods/mphalport.h"
#include "lib/utils/interrupt_char.h"


//extern const unsigned char mp_hal_status_to_errno_table[4];

//extern void mp_hal_set_interrupt_char(int c); // -1 to disable

void mp_hal_ticks_cpu_enable(void);


// C-level pin HAL
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include "mods/pin.h"
#include "inc/hw_gpio.h"
#include "driverlib/sysctl.h"
#include "driverlib/gpio.h"
#include "driverlib/rom.h"
#include "driverlib/rom_map.h"

//#ifdef MICROPY_DEBUG_VERBOSE
//#define DEBUG_printf printf
//#endif

#define MP_HAL_PIN_FMT                  "%q"
#define MP_HAL_PIN_MODE_INPUT           (GPIO_DIR_MODE_IN)
#define MP_HAL_PIN_MODE_OUTPUT          (GPIO_DIR_MODE_OUT)
#define MP_HAL_PIN_MODE_ALT             (GPIO_DIR_MODE_HW)
#define MP_HAL_PIN_MODE_ANALOG          (GPIO_PIN_TYPE_ANALOG)


#define MP_HAL_PIN_MODE_OPEN_DRAIN      (GPIO_PIN_TYPE_OD)
#define MP_HAL_PIN_MODE_ALT_OPEN_DRAIN  (GPIO_PIN_TYPE_OD)
#define MP_HAL_PIN_PULL_NONE            (GPIO_PIN_TYPE_STD)
#define MP_HAL_PIN_PULL_UP              (GPIO_PIN_TYPE_STD_WPU)
#define MP_HAL_PIN_PULL_DOWN            (GPIO_PIN_TYPE_STD_WPD)

#define MP_HAL_PIN_STRENGTH_LOW         (GPIO_STRENGTH_2MA)
#define MP_HAL_PIN_STRENGTH_MED         (GPIO_STRENGTH_4MA)
#define MP_HAL_PIN_STRENGTH_HI          (GPIO_STRENGTH_8MA)

enum {
    PIN_FN_ADC = 0,
    PIN_FN_UART,
    PIN_FN_SSI,
    PIN_FN_I2C,
    PIN_FN_QEI,
    PIN_FN_MTRL,
    PIN_FN_WTIM,
    PIN_FN_TIM,
    PIN_FN_JTAG,
    PIN_FN_NMI,
    PIN_FN_CAN,
    PIN_FN_USB,
    PIN_FN_COMP,
    PIN_FN_TR,
};

enum {
    AF_UART_TX = 0,
    AF_UART_RX,
    AF_UART_RTS,
    AF_UART_CTS,
};

enum {
    AF_SSI_CLK = 0,
    AF_SSI_TX,
    AF_SSI_RX,
    AF_SSI_FSS,
};

enum {
    AF_I2C_SDA = 0,
    AF_I2C_SCL,
};

enum {
    AF_TIM_CCP0 = 0,
    AF_TIM_CCP1,
};

enum {
    AF_WTIM_CCP0 = 0,
    AF_WTIM_CCP1,
};

enum {
    AF_QEI_PHA0 = 0,
    AF_QEI_PHA1,
    AF_QEI_PHB0,
    AF_QEI_PHB1,
    AF_QEI_IDX0,
    AF_QEI_IDX1,
};

enum {
    AF_USB_EPEN = 0,
    AF_USB_PFLT,
    AF_USB_DM,
    AF_USB_DP,
    AF_USB_ID,
    AF_USB_VBUS,
};

enum {
    AF_COMP_POS = 0,
    AF_COMP_NEG,
    AF_COMP_OUT,
};

enum {
    AF_CAN_TX = 0,
    AF_CAN_RX,
};

enum {
    AF_JTAG_SWO = 0,
    AF_JTAG_TDO,
    AF_JTAG_SWCLK,
    AF_JTAG_TCK,
    AF_JTAG_TDI,
    AF_JTAG_TMS,
    AF_JTAG_SWDIO,
};

enum {
    AF_TR_CLK = 0,
    AF_TR_D0,
    AF_TR_D1,
};

enum {
    AF_NMI_ = 0, // Underscore bc makro always appends one
};

enum {
    AF_MTRL_FAULT0 = 0,
    AF_MTRL_PWM0 ,
    AF_MTRL_PWM1 ,
    AF_MTRL_PWM2 ,
    AF_MTRL_PWM3 ,
    AF_MTRL_PWM4 ,
    AF_MTRL_PWM5 ,
    AF_MTRL_PWM6 ,
    AF_MTRL_PWM7 ,
};

enum {
    AF_ADC_AIN0 = 0,
    AF_ADC_AIN1,
    AF_ADC_AIN2,
    AF_ADC_AIN3,
    AF_ADC_AIN4,
    AF_ADC_AIN5,
    AF_ADC_AIN6,
    AF_ADC_AIN7,
    AF_ADC_AIN8,
    AF_ADC_AIN9,
    AF_ADC_AIN10,
    AF_ADC_AIN11,
};

#define IS_GPIO_DIR(d)      ((d == GPIO_DIR_MODE_IN) || \
                            (d == GPIO_DIR_MODE_OUT) || \
                            (d == GPIO_DIR_MODE_HW))

#define IS_GPIO_TYPE(p)     ((p == GPIO_PIN_TYPE_STD) || \
                            (p == GPIO_PIN_TYPE_STD_WPU) || \
                            (p == GPIO_PIN_TYPE_STD_WPD) || \
                            (p == GPIO_PIN_TYPE_OD) || \
                            (p == GPIO_PIN_TYPE_WAKE_LOW) || \
                            (p == GPIO_PIN_TYPE_WAKE_HIGH) || \
                            (p == GPIO_PIN_TYPE_ANALOG))

#define IS_GPIO_STRENGTH(s) ((s == GPIO_STRENGTH_2MA) || \
                            (s == GPIO_STRENGTH_4MA) || \
                            (s == GPIO_STRENGTH_8MA))

#define IS_GPIO_PORT(p)     (_GPIOBaseValid(p))

#define IS_GPIO_AF(a)     ((bool)(a > PIN_FN_UART && a < PIN_FN_TR))

#define mp_hal_pin_obj_t const pin_obj_t*
#define mp_hal_get_pin_obj(o)   pin_find(o)
#define mp_hal_pin_name(p)      ((p)->name)
#define mp_hal_pin_input(p)     mp_hal_pin_config((p), MP_HAL_PIN_MODE_INPUT, MP_HAL_PIN_PULL_NONE, MP_HAL_PIN_STRENGTH_LOW)
#define mp_hal_pin_output(p)    mp_hal_pin_config((p), MP_HAL_PIN_MODE_OUTPUT, MP_HAL_PIN_PULL_NONE, MP_HAL_PIN_STRENGTH_LOW)
#define mp_hal_pin_open_drain(p) mp_hal_pin_config((p), MP_HAL_PIN_MODE_OUTPUT, MP_HAL_PIN_MODE_OPEN_DRAIN, MP_HAL_PIN_STRENGTH_LOW)

#define mp_hal_pin_high(p)      MAP_GPIOPinWrite((p)->gpio, (p)->pin_mask, (p)->pin_mask)
#define mp_hal_pin_low(p)       MAP_GPIOPinWrite((p)->gpio, (p)->pin_mask,  0)

#define mp_hal_pin_od_low(p)    mp_hal_pin_low(p)
#define mp_hal_pin_od_high(p)   mp_hal_pin_high(p)
#define mp_hal_pin_read(p)      MAP_GPIOPinRead((p)->gpio, (p)->pin_mask)
#define mp_hal_pin_write(p, v)  MAP_GPIOPinWrite((p)->gpio, (p)->pin_mask, (v) ? (p)->pin_mask : 0)

#define mp_hal_pin_get_af(p)    (((p)->regs->PCTL >> ((p)->pin_num * 4)) & 0xF)
#define mp_hal_pin_get_dir(p)   (MAP_GPIODirModeGet((p)->gpio, (p)->pin_mask))

//#define mp_hal_pin_get_type(p)  ()
//#define mp_hal_pin_get_drive(p)


//#define mp_hal_delay_ms(ms)     (MAP_SysCtlDelay( MAP_SysCtlClockGet()/3000 * ms))
//#define mp_hal_delay_us(us)     (MAP_SysCtlDelay( MAP_SysCtlClockGet()/3000000 * us))
//#define mp_hal_ticks_ms         (mp_hal_ticks_cpu / (MAP_SysCtlClockGet()*3000))
//#define mp_hal_ticks_us         (mp_hal_ticks_cpu / (MAP_SysCtlClockGet()*3000000))


void mp_hal_unlock_special_pin(mp_hal_pin_obj_t pin);
bool mp_hal_pin_needs_unlocking(mp_hal_pin_obj_t pin);
void mp_hal_gpio_clock_enable(const uint32_t port);
bool mp_hal_pin_config(mp_hal_pin_obj_t pin_obj, uint32_t dir, uint32_t type, uint32_t drive);
bool mp_hal_pin_config_alt(mp_hal_pin_obj_t pin_obj, uint8_t fn, uint8_t unit);
void mp_hal_pin_set_af(mp_hal_pin_obj_t pin_obj, uint8_t af_id);
NORETURN void mp_hal_raise(int status);
uint32_t HAL_GetTick();

#endif // MICROPY_INCLUDED_TM4C_HAL_PINS_H
