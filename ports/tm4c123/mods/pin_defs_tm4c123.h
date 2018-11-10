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

// This file contains pin definitions that are specific to the tm4c123 port.
// This file should only ever be #included by pin.h and not directly.
#include <stdint.h>

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

enum {
    PORT_A = GPIOA ,
    PORT_B = GPIOB,
    PORT_C = GPIOC,
    PORT_D = GPIOD,
    PORT_E = GPIOE,
    PORT_F = GPIOF,
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
    AF_PIN_TYPE_UART_TX = 0,
    AF_PIN_TYPE_UART_RX,
    AF_PIN_TYPE_UART_RTS,
    AF_PIN_TYPE_UART_CTS,
};

enum {
    AF_PIN_TYPE_SSI_CLK = 0,
    AF_PIN_TYPE_SSI_TX,
    AF_PIN_TYPE_SSI_RX,
    AF_PIN_TYPE_SSI_FSS,
};

enum {
    AF_PIN_TYPE_I2C_SDA = 0,
    AF_PIN_TYPE_I2C_SCL,
};

enum {
    AF_PIN_TYPE_TIM_CCP0 = 0,
    AF_PIN_TYPE_TIM_CCP1
};

enum {
    AF_PIN_TYPE_WTIM_CCP0 = 0,
    AF_PIN_TYPE_WTIM_CCP1
};

enum {
    AF_PIN_TYPE_QEI_PHA0 = 0,
    AF_PIN_TYPE_QEI_PHA1,
    AF_PIN_TYPE_QEI_PHB0,
    AF_PIN_TYPE_QEI_PHB1,
    AF_PIN_TYPE_QEI_IDX0,
    AF_PIN_TYPE_QEI_IDX1
};

enum {
    AF_PIN_TYPE_USB_EPEN = 0,
    AF_PIN_TYPE_USB_PFLT,
    AF_PIN_TYPE_USB_DM,
    AF_PIN_TYPE_USB_DP,
    AF_PIN_TYPE_USB_ID,
    AF_PIN_TYPE_USB_VBUS
};

enum {
    AF_PIN_TYPE_COMP_POS = 0,
    AF_PIN_TYPE_COMP_NEG,
    AF_PIN_TYPE_COMP_OUT
};

enum {
    AF_PIN_TYPE_CAN_TX = 0,
    AF_PIN_TYPE_CAN_RX
};

enum {
    AF_PIN_TYPE_TR_CLK = 0,
    AF_PIN_TYPE_TR_D0,
    AF_PIN_TYPE_TR_D1
};

enum {
    AF_PIN_TYPE_MTRL_FAULT0 = 0,
    AF_PIN_TYPE_MTRL_PWM0 ,
    AF_PIN_TYPE_MTRL_PWM1 ,
    AF_PIN_TYPE_MTRL_PWM2 ,
    AF_PIN_TYPE_MTRL_PWM3 ,
    AF_PIN_TYPE_MTRL_PWM4 ,
    AF_PIN_TYPE_MTRL_PWM5 ,
    AF_PIN_TYPE_MTRL_PWM6 ,
    AF_PIN_TYPE_MTRL_PWM7 ,
};

enum {
    AF_PIN_TYPE_ADC_AIN0 = 0,
    AF_PIN_TYPE_ADC_AIN1,
    AF_PIN_TYPE_ADC_AIN2,
    AF_PIN_TYPE_ADC_AIN3,
    AF_PIN_TYPE_ADC_AIN4,
    AF_PIN_TYPE_ADC_AIN5,
    AF_PIN_TYPE_ADC_AIN6,
    AF_PIN_TYPE_ADC_AIN7,
    AF_PIN_TYPE_ADC_AIN8,
    AF_PIN_TYPE_ADC_AIN9,
    AF_PIN_TYPE_ADC_AIN10,
    AF_PIN_TYPE_ADC_AIN11,
};




// The HAL uses a slightly different naming than we chose, so we provide
// some #defines to massage things. Also I2S and SPI share the same
// peripheral.




typedef periph_gpio_t GPIO_TypeDef;

