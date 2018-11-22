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

#include <hal_pins.h>
#include "py/obj.h"
#include "inc/hw_gpio.h"
#include "driverlib/rom.h"
#include "driverlib/rom_map.h"
#include "driverlib/gpio.h"

void gpio_init(periph_gpio_t *gpio, int pin, int mode, int pull, int alt) {
    if ( pin >= 8 ) return;
    // does not consider the locked pins, which need special treatment
    if (mode == GPIO_MODE_ALT) {
        gpio->DEN |= (1 << pin);
        gpio->AFSEL |= (1 << pin);
        gpio->PCTL = (gpio->PCTL & ~(0x000000F << (4 * pin))) | (alt << (4 * pin));
    }
    else {
        gpio->AFSEL &= ~(1 << pin);
        gpio->DEN |= (1 << pin);
        gpio->DIR = (gpio->DIR & ~(1 << pin)) | ((mode & 1) << pin);
    }

    if (pull == GPIO_PULL_UP) { gpio->PUR = (gpio->PUR & ~(1 << pin)) | (1 << pin);}
    else if (pull == GPIO_PULL_DOWN) {
        gpio->PDR = (gpio->PDR & ~(1 << pin)) | (1 << pin);
    } else if (pull == GPIO_PULL_NONE) {
        gpio->PUR &= ~(1 << pin);
        gpio->PDR &= ~(1 << pin);
        gpio->ODR &= (1 << pin);
    }
}

uint32_t pin_get_mode(const pin_obj_t *pin) {
    uint32_t mode = MAP_GPIODirModeGet(pin->port, pin->pin_mask);
    uint32_t type;
    MAP_GPIOPadConfigGet(pin->port, pin->pin_mask, NULL, &type);
    if (mode == GPIO_DIR_MODE_IN) {
        return GPIO_MODE_IN
    } else if (mode == GPIO_DIR_MODE_OUT) {
        if(type == GPIO_PIN_TYPE_OD) {
            return GPIO_MODE_OPEN_DRAIN;
        } else  {
            return GPIO_MODE_OUT;
        }
    } else if(mode == GPIO_DIR_MODE_HW) {
        if(type == GPIO_PIN_TYPE_OD) {
            return GPIO_MODE_ALT_OD;
        } else  {
            return GPIO_MODE_ALT_PP;
        }
    }
    return -1;
}

uint32_t pin_get_pull(const pin_obj_t *pin) {
    uint32_t type;
    MAP_GPIOPadConfigGet(pin->port, pin->pin_mask, NULL, &type);
    if(type == GPIO_PIN_TYPE_OD || type == GPIO_PIN_TYPE_STD) {
        return GPIO_PULL_NONE;
    } else if (type == GPIO_PIN_TYPE_STD_WPU) {
        return GPIO_PULL_UP;
    } else if (type == GPIO_PIN_TYPE_STD_WPD) {
        return GPIO_PULL_DOWN;
    }
    return -1;
}

uint32_t pin_get_strength(const pin_obj_t *pin) {
    uint32_t strength;
    MAP_GPIOPadConfigGet(pin->port, pin->pin_mask, &strength, NULL);
    if (strength == GPIO_STRENGTH_2MA) {
        return GPIO_DRIVE_LOW;
    } else if (strength == GPIO_STRENGTH_4MA) {
        return GPIO_DRIVE_MED;
    } else if (strength == GPIO_STRENGTH_8MA) {
        return GPIO_DRIVE_HI;
    }
    return -1;
}

uint32_t pin_get_af(const pin_obj_t *pin) {
    uint32_t mode = MAP_GPIODirModeGet(pin->port, pin->pin_mask);
    if (mode == GPIO_DIR_MODE_HW) {
        return (pin->gpio->PCTL >> (pin->pin_num * 4)) & 0xF;
    } else {
        return -1;
    }
    return -1;
}


