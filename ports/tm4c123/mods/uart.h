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
#ifndef MICROPY_INCLUDED_TM4C_UART_H
#define MICROPY_INCLUDED_TM4C_UART_H

typedef struct {
    volatile uint32_t DR; // Data Register 0x00
    volatile uint32_t RSR; // Status Register 0x04
    uint32_t _1[4];
    volatile uint32_t FR; // Flag Register 0x18
    uint32_t _2;
    volatile uint32_t ILPR; // IrDA Low-Power Register 0x20
    volatile uint32_t IBRD; // Integer Baud Rate Divisor 0x24
    volatile uint32_t FBRD; // Fractional Baud Rate divisor 0x28
    volatile uint32_t LCRH;
    volatile uint32_t CTL;
    volatile uint32_t IFLS;
    volatile uint32_t IM;
    volatile uint32_t RIS;
    volatile uint32_t MIS;
    volatile uint32_t ICR;
    volatile uint32_t DMACTL;
    uint32_t _3[16];
    volatile uint32_t _9BITADDR;
    volatile uint32_t _9BITAMASK;
    volatile uint32_t PP;
    volatile uint32_t CC;
} periph_uart_t;

typedef enum {
    UART_NONE = -1,
    UART_0 = 0,
    UART_1 = 1,
    UART_2 = 2,
    UART_3 = 3,
    UART_4 = 4,
    UART_5 = 5,
    UART_6 = 6,
    UART_7 = 7,
} machine_uart_t;

typedef struct _machine_uart_obj_t machine_uart_obj_t;
extern const mp_obj_type_t machine_uart_type;

void uart_init0(void);
void uart_deinit(void);
void uart_irq_handler(mp_uint_t uart_id);

void uart_attach_to_repl(machine_uart_obj_t *self, bool attached);
mp_uint_t uart_rx_any(machine_uart_obj_t *uart_obj);
int uart_rx_char(machine_uart_obj_t *uart_obj);
void uart_tx_strn(machine_uart_obj_t *uart_obj, const char *str, uint len);

#endif // MICROPY_INCLUDED_TM4C_UART_H
