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

#include <hal/hal_pins.h>
#include "pin.h"
#include "py/obj.h"
#include "inc/hw_gpio.h"
#include "driverlib/gpio.h"
#include "driverlib/rom.h"
#include "driverlib/rom_map.h"


//void gpio_init(periph_gpio_t *gpio, int pin, int mode, int pull, int alt) {
//    if ( pin >= 8 ) return;
//    // does not consider the locked pins, which need special treatment
//    if (mode == GPIO_MODE_ALT) {
//        gpio->DEN |= (1 << pin);
//        gpio->AFSEL |= (1 << pin);
//        gpio->PCTL = (gpio->PCTL & ~(0x000000F << (4 * pin))) | (alt << (4 * pin));
//    }
//    else {
//        gpio->AFSEL &= ~(1 << pin);
//        gpio->DEN |= (1 << pin);
//        gpio->DIR = (gpio->DIR & ~(1 << pin)) | ((mode & 1) << pin);
//    }
//
//    if (pull == GPIO_PULL_UP) { gpio->PUR = (gpio->PUR & ~(1 << pin)) | (1 << pin);}
//    else if (pull == GPIO_PULL_DOWN) {
//        gpio->PDR = (gpio->PDR & ~(1 << pin)) | (1 << pin);
//    } else if (pull == GPIO_PULL_NONE) {
//        gpio->PUR &= ~(1 << pin);
//        gpio->PDR &= ~(1 << pin);
//        gpio->ODR &= (1 << pin);
//    }
//}

uint32_t pin_get_dir(const pin_obj_t *pin) {
    return MAP_GPIODirModeGet(pin->gpio, pin->pin_mask);

}

uint32_t pin_get_type(const pin_obj_t *pin) {
    uint32_t type;
    MAP_GPIOPadConfigGet(pin->gpio, pin->pin_mask, NULL, &type);
    return type;
}

uint32_t pin_get_drive(const pin_obj_t *pin) {
    uint32_t drive;
    MAP_GPIOPadConfigGet(pin->gpio, pin->pin_mask, &drive, NULL);
    return drive;
}

uint32_t pin_get_af(const pin_obj_t *pin) {
    uint32_t dir = MAP_GPIODirModeGet(pin->gpio, pin->pin_mask);
    if (dir == GPIO_DIR_MODE_HW) {
        return (pin->regs->PCTL >> (pin->pin_num * 4)) & 0xF;
    } else {
        return -1;
    }
    return -1;
}

void gpio_clock_enable(const uint32_t port) {
    MAP_SysCtlPeripheralEnable(port);
    while(!MAP_SysCtlPeripheralReady(port)){};
}

//uint32_t convert_mode_pull_to_dir_type(uint32_t mode, uint32_t pull, uint32_t* dir, uint32_t* type) {
//    uint32_t d;
//    uint32_t t;
//    switch (mode) {
//        case GPIO_MODE_IN:
//            d = GPIO_DIR_MODE_IN;
//            break;
//        case GPIO_MODE_OUT:
//            d = GPIO_DIR_MODE_OUT;
//            switch (pull) {
//                case GPIO_PULL_UP:
//                    t = GPIO_PIN_TYPE_STD_WPU;
//                    break;
//                case GPIO_PULL_DOWN:
//                    t = GPIO_PIN_TYPE_STD_WPD;
//                    break;
//                case GPIO_PULL_NONE:
//                    t = GPIO_PIN_TYPE_STD;
//                    break;
//                default:
//                    t = GPIO_PIN_TYPE_STD;
//                    break;
//            }
//            break;
//        case GPIO_MODE_OPEN_DRAIN:
//            d = GPIO_DIR_MODE_OUT;
//            t = GPIO_PIN_TYPE_OD;
//            break;
//        case GPIO_MODE_AF_PP:
//            d = GPIO_DIR_MODE_HW;
//            switch (pull) {
//                case GPIO_PULL_UP:
//                    t = GPIO_PIN_TYPE_STD_WPU;
//                    break;
//                case GPIO_PULL_DOWN:
//                    t = GPIO_PIN_TYPE_STD_WPD;
//                    break;
//                case GPIO_PULL_NONE:
//                    t = GPIO_PIN_TYPE_STD;
//                    break;
//                default:
//                    t = GPIO_PIN_TYPE_STD;
//                    break;
//            }
//            break;
//        case GPIO_MODE_AF_OD:
//            d = GPIO_DIR_MODE_HW;
//            t = GPIO_PIN_TYPE_OD;
//            break;
//        case GPIO_MODE_ANALOG:
//            d = GPIO_DIR_MODE_HW;
//            t = GPIO_PIN_TYPE_ANALOG;
//            break;
//        default:
//            d = GPIO_DIR_MODE_IN;
//            break;
//    }
//}

void gpio_init(uint32_t port, uint32_t pin_mask, uint dir, uint type, uint drive) {
    gpio_clock_enable(port);

    MAP_GPIODirModeSet(port, pin_mask, dir);
    MAP_GPIOPadConfigSet(port, pin_mask, drive, type);
}

uint32_t pin_read(const pin_obj_t* pin) {
    return MAP_GPIOPinRead(pin->gpio, pin->pin_mask);
}

void pin_write(const pin_obj_t* pin, uint32_t value) {
    MAP_GPIOPinWrite(pin->gpio, pin->pin_mask, value ? pin->pin_mask : 0);
}

void pin_low(const pin_obj_t* pin) {
    MAP_GPIOPinWrite(pin->gpio, pin->pin_mask, 0);
}

void pin_high(const pin_obj_t* pin) {
    MAP_GPIOPinWrite(pin->gpio, pin->pin_mask, pin->pin_mask);
}
