#include <string.h>
#include "py/mpconfig.h"

/*
 * Core UART functions to implement for a port
 */

#if MICROPY_MIN_USE_STM32_MCU
typedef struct {
    volatile uint32_t DR; // Data Register 0x00
    volatile char ph1[0x02]; //Spacer 2 Bytes
    volatile uint32_t SR; // Status Register 0x04
    volatile char ph2[0x12]; // Spacer 0x12 bytes
    volatile uint32_t FR; //Flag Register 0x18
} periph_uart_t;
#define UART0 ((periph_uart_t*)0x4000C000)
#endif

// Receive single character
int mp_hal_stdin_rx_chr(void) {
    unsigned char c = 0;
#if MICROPY_MIN_USE_STDOUT
    int r = read(0, &c, 1);
    (void)r;
#elif MICROPY_MIN_USE_STM32_MCU
    // wait for RXFE
    while ((UART0->FR & (1 << 4)) == 0) {
    }
    c = UART0->DR;
#endif
    return c;
}

// Send string of given length
void mp_hal_stdout_tx_strn(const char *str, mp_uint_t len) {
#if MICROPY_MIN_USE_STDOUT
    int r = write(1, str, len);
    (void)r;
#elif MICROPY_MIN_USE_STM32_MCU
    while (len--) {
        // wait for TXE
        while ((UART0->FR & (1 << 7)) == 0) {
        }
        UART0->DR = *str++;
    }
#endif
}
