/*
 * This file is part of the MicroPython project, http://micropython.org/
 *
 * The MIT License (MIT)
 *
 * Copyright (c) 2018, 2014 Damien P. George
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

#ifndef MICROPY_INCLUDED_TM4C_HAL_PINS_H
#define MICROPY_INCLUDED_TM4C_HAL_PINS_H

#include <stdint.h>
#include <stdio.h>
#include <string.h>

typedef struct {
    uint32_t _1[255];
    volatile uint32_t DATA;
    volatile uint32_t DIR;
    volatile uint32_t IS;
    volatile uint32_t IBE;
    volatile uint32_t IEV;
    volatile uint32_t IM;
    volatile uint32_t RIS;
    volatile uint32_t MIS;
    volatile uint32_t ICR;
    volatile uint32_t AFSEL;
    uint32_t _2[55];
    volatile uint32_t DR2R;
    volatile uint32_t DR4R;
    volatile uint32_t DR8R;
    volatile uint32_t ODR;
    volatile uint32_t PUR;
    volatile uint32_t PDR;
    volatile uint32_t SLR;
    volatile uint32_t DEN;
    volatile uint32_t LOCK;
    volatile uint32_t CR;
    volatile uint32_t AMSEL;
    volatile uint32_t PCTL;
    volatile uint32_t ADCCTL;
    volatile uint32_t DMACTL;
} periph_gpio_t;

enum {
    GPIO_DRIVE_LOW = 0,
    GPIO_DRIVE_MED,
    GPIO_DRIVE_HI
};

// simple GPIO interface
enum {
    GPIO_MODE_IN = 0,
    GPIO_MODE_OUT,
    GPIO_MODE_OPEN_DRAIN,
    GPIO_MODE_ALT_PP,
    GPIO_MODE_ALT_OD,
    GPIO_MODE_ANALOG,
};

enum {
    GPIO_PULL_UP = 0,
    GPIO_PULL_DOWN,
    GPIO_PULL_NONE,
};

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
    PIN_TYPE_MTRL_PWM0 ,
    PIN_TYPE_MTRL_PWM1 ,
    PIN_TYPE_MTRL_PWM2 ,
    PIN_TYPE_MTRL_PWM3 ,
    PIN_TYPE_MTRL_PWM4 ,
    PIN_TYPE_MTRL_PWM5 ,
    PIN_TYPE_MTRL_PWM6 ,
    PIN_TYPE_MTRL_PWM7 ,
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


#define GPIOA  ((periph_gpio_t*) 0x40058000)
#define GPIOB  ((periph_gpio_t*) 0x40059000)
#define GPIOC  ((periph_gpio_t*) 0x4005A000)
#define GPIOD  ((periph_gpio_t*) 0x4005B000)
#define GPIOE  ((periph_gpio_t*) 0x4005C000)
#define GPIOF  ((periph_gpio_t*) 0x4005D000)

#define GPIO(x) ((periph_gpio_t*) x)

// bits 9:2 as mask for GPIO ports
#define gpio_get(gpio, pin) ((gpio->DATA >> (pin)) & 1);
// https://stackoverflow.com/questions/257418/do-while-0-what-is-it-good-for#257425
#define gpio_set(gpio, pin, value) do { gpio->DATA = (gpio->DATA & ~(1 << pin)) | (value << pin); } while (0);
#define gpio_low(gpio, pin) gpio_set(gpio, pin, 0);
#define gpio_high(gpio, pin) gpio_set(gpio, pin, 1);


uint32_t pin_get_mode(const pin_obj_t *pin);
uint32_t pin_get_pull(const pin_obj_t *pin);
uint32_t pin_get_strength(const pin_obj_t *pin)
uint32_t pin_get_af(const pin_obj_t *pin);


#endif // MICROPY_INCLUDED_TM4C_HAL_PINS_H
