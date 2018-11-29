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

// GPIO ports
enum {
    PORT_A = GPIO_PORTA_AHB_BASE,
    PORT_B = GPIO_PORTB_AHB_BASE,
    PORT_C = GPIO_PORTC_AHB_BASE,
    PORT_D = GPIO_PORTD_AHB_BASE,
    PORT_E = GPIO_PORTE_AHB_BASE,
    PORT_F = GPIO_PORTF_AHB_BASE,
};

#define IS_GPIO_PORT(p)     (_GPIOBaseValid(p))

// GPIO afs
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
    PIN_FN_TR,
};

#define IS_GPIO_AF(a)     ((bool)(a > PIN_FN_UART && a < PIN_FN_TR))

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
    AF_TIM_CCP1
};

enum {
    AF_WTIM_CCP0 = 0,
    AF_WTIM_CCP1
};

enum {
    AF_QEI_PHA0 = 0,
    AF_QEI_PHA1,
    AF_QEI_PHB0,
    AF_QEI_PHB1,
    AF_QEI_IDX0,
    AF_QEI_IDX1
};

enum {
    AF_USB_EPEN = 0,
    AF_USB_PFLT,
    AF_USB_DM,
    AF_USB_DP,
    AF_USB_ID,
    AF_USB_VBUS
};

enum {
    AF_COMP_POS = 0,
    AF_COMP_NEG,
    AF_COMP_OUT
};

enum {
    AF_CAN_TX = 0,
    AF_CAN_RX
};

enum {
    AF_TR_CLK = 0,
    AF_TR_D0,
    AF_TR_D1
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

uint32_t mp_hal_convert_mode_pull_to_dir_type(pin_obj_t* pin, uint32_t* dir, uint32_t* type);


uint32_t mp_hal_pin_get_dir(const pin_obj_t *pin);
uint32_t mp_hal_pin_get_type(const pin_obj_t *pin);
uint32_t mp_hal_pin_get_drive(const pin_obj_t *pin)
uint32_t mp_hal_pin_get_af(const pin_obj_t *pin);

uint32_t mp_hal_pin_read(const pin_obj_t* pin);
void mp_hal_pin_write(const pin_obj_t* pin, uint32_t value);
void mp_hal_pin_low(const pin_obj_t* pin);
void mp_hal_pin_high(const pin_obj_t* pin);

void mp_hal_gpio_clock_enable(const uint32_t port);
void mp_hal_gpio_init(uint32_t port, uint32_t pin_mask, uint mode, uint pull, uint drive);


#endif // MICROPY_INCLUDED_TM4C_HAL_PINS_H
