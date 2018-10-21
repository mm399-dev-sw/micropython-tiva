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

#ifndef MICROPY_TM4C123_INC_GPIO_H
#define MICROPY_TM4C123_INC_GPIO_H

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
    GPIO_DRIVE_2MA = 0,
    GPIO_DRIVE_4MA,
    GPIO_DRIVE_8MA
};

// simple GPIO interface
#define GPIO_MODE_IN (0)
#define GPIO_MODE_OUT (1)
#define GPIO_MODE_ALT (2)

#define GPIO_PULL_UP (0)
#define GPIO_PULL_DOWN (1)
#define GPIO_PULL_NONE (2)



#define GPIOA  ((periph_gpio_t*) 0x40058000)
#define GPIOB  ((periph_gpio_t*) 0x40059000)
#define GPIOC  ((periph_gpio_t*) 0x4005A000)
#define GPIOD  ((periph_gpio_t*) 0x4005B000)
#define GPIOE  ((periph_gpio_t*) 0x4005C000)
#define GPIOF  ((periph_gpio_t*) 0x4005D000)

void hal_gpio_init(periph_gpio_t* gpio, int pin, int mode, int pull, int alt, int strength);
void hal_gpio_set_strength(periph_gpio_t* gpio, int pin, int strength);

void gpio_init(periph_gpio_t *gpio, int pin, int mode, int pull, int alt);

// bits 9:2 as mask for GPIO ports
#define gpio_get(gpio, pin) ((gpio->DATA >> (pin)) & 1);
// https://stackoverflow.com/questions/257418/do-while-0-what-is-it-good-for#257425
#define gpio_set(gpio, pin, value) do { gpio->DATA = (gpio->DATA & ~(1 << pin)) | (value << pin); } while (0);
#define gpio_low(gpio, pin) gpio_set(gpio, pin, 0);
#define gpio_high(gpio, pin) gpio_set(gpio, pin, 1);


#endif // MICROPY_TM4C123_INC_GPIO_H
