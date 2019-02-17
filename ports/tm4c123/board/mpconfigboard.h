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

#define MICROPY_HW_BOARD_NAME       "Tiva Launch Pad"
#define MICROPY_HW_MCU_NAME         "TM4C123GH6PM"

// assumption: every GPIO is interrupt capable
#define PYB_EXTI_NUM_VECTORS (43)
#define MICROPY_HW_MAX_UART (8)

// UART config
#define MICROPY_HW_UART0_NAME   "0"
#define MICROPY_HW_UART0_RX     (pin_PA0)
#define MICROPY_HW_UART0_TX     (pin_PA1)

#define MICROPY_HW_UART1_NAME   "1"
#define MICROPY_HW_UART1_TX     (pin_PB0)
#define MICROPY_HW_UART1_RX     (pin_PB1)
#define MICROPY_HW_UART1_RTS    (pin_PF0) //pin_PC4
#define MICROPY_HW_UART1_CTS    (pin_PF1) //pin_PC5

#define MICROPY_HW_UART2_NAME   "2"
#define MICROPY_HW_UART2_RX     (pin_PD6)
#define MICROPY_HW_UART2_TX     (pin_PD7)

#define MICROPY_HW_UART3_NAME   "3"
#define MICROPY_HW_UART3_RX     (pin_PC6)
#define MICROPY_HW_UART3_TX     (pin_PC7)

#define MICROPY_HW_UART4_NAME   "4"
#define MICROPY_HW_UART4_RX     (pin_PC4)
#define MICROPY_HW_UART4_TX     (pin_PC5)

#define MICROPY_HW_UART5_NAME   "5"
#define MICROPY_HW_UART5_RX     (pin_PE4)
#define MICROPY_HW_UART5_TX     (pin_PE5)

#define MICROPY_HW_UART6_NAME   "6"
#define MICROPY_HW_UART6_RX     (pin_PD4)
#define MICROPY_HW_UART6_TX     (pin_PD5)

#define MICROPY_HW_UART7_NAME   "7"
#define MICROPY_HW_UART7_RX     (pin_PE0)
#define MICROPY_HW_UART7_TX     (pin_PE1)

