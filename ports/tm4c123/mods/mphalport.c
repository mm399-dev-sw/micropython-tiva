#include <string.h>
#include "mphalport.h"
#include "py/runtime.h"
#include "py/mperrno.h"
#include "py/mphal.h"
#include "extmod/misc.h"
#include "usb.h"
#include "uart.h"
#include "driverlib/rom.h"
#include "driverlib/rom_map.h"
#include "driverlib/sysctl.h"

// this table converts from HAL_StatusTypeDef to POSIX errno
const byte mp_hal_status_to_errno_table[4] = {
    [HAL_OK] = 0,
    [HAL_ERROR] = MP_EIO,
    [HAL_BUSY] = MP_EBUSY,
    [HAL_TIMEOUT] = MP_ETIMEDOUT,
};

NORETURN void mp_hal_raise(HAL_StatusTypeDef status) {
    mp_raise_OSError(mp_hal_status_to_errno_table[status]);
}

MP_WEAK int mp_hal_stdin_rx_chr(void) {
    for (;;) {
#if 0
#ifdef USE_HOST_MODE
        pyb_usb_host_process();
        int c = pyb_usb_host_get_keyboard();
        if (c != 0) {
            return c;
        }
#endif
#endif

        #if MICROPY_HW_ENABLE_USB
        byte c;
        if (usb_vcp_recv_byte(&c) != 0) {
            return c;
        }
        #endif
        if (MP_STATE_PORT(pyb_stdio_uart) != NULL && uart_rx_any(MP_STATE_PORT(pyb_stdio_uart))) {
            return uart_rx_char(MP_STATE_PORT(pyb_stdio_uart));
        }
        int dupterm_c = mp_uos_dupterm_rx_chr();
        if (dupterm_c >= 0) {
            return dupterm_c;
        }
        MICROPY_EVENT_POLL_HOOK
    }
}

void mp_hal_stdout_tx_str(const char *str) {
    mp_hal_stdout_tx_strn(str, strlen(str));
}

MP_WEAK void mp_hal_stdout_tx_strn(const char *str, size_t len) {
    if (MP_STATE_PORT(pyb_stdio_uart) != NULL) {
        uart_tx_strn(MP_STATE_PORT(pyb_stdio_uart), str, len);
    }
#if 0 && defined(USE_HOST_MODE) && MICROPY_HW_HAS_LCD
    lcd_print_strn(str, len);
#endif
    #if MICROPY_HW_ENABLE_USB
    if (usb_vcp_is_enabled()) {
        usb_vcp_send_strn(str, len);
    }
    #endif
    mp_uos_dupterm_tx_strn(str, len);
}

// Efficiently convert "\n" to "\r\n"
void mp_hal_stdout_tx_strn_cooked(const char *str, size_t len) {
    const char *last = str;
    while (len--) {
        if (*str == '\n') {
            if (str > last) {
                mp_hal_stdout_tx_strn(last, str - last);
            }
            mp_hal_stdout_tx_strn("\r\n", 2);
            ++str;
            last = str;
        } else {
            ++str;
        }
    }
    if (str > last) {
        mp_hal_stdout_tx_strn(last, str - last);
    }
}

#if __CORTEX_M >= 0x03
void mp_hal_ticks_cpu_enable(void) {
    if (!(DWT->CTRL & DWT_CTRL_CYCCNTENA_Msk)) {
        CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
        #if defined(__CORTEX_M) && __CORTEX_M == 7
        // on Cortex-M7 we must unlock the DWT before writing to its registers
        DWT->LAR = 0xc5acce55;
        #endif
        DWT->CYCCNT = 0;
        DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
    }
}
#endif

void mp_hal_gpio_clock_enable(GPIO_TypeDef *gpio) {
    MAP_SysCtlPeripheralEnable(gpio);
    while(!MAP_SysCtlPeripheralReady(gpio)) {};
}

void mp_hal_pin_config(mp_hal_pin_obj_t pin_obj, uint32_t dir, uint32_t type, uint32_t strength, uint32_t alt) {
    GPIO_TypeDef *gpio = pin_obj->gpio;
    uint32_t pin = pin_obj->pin;
    mp_hal_gpio_clock_enable(gpio);
//    MAP_GPIODirModeSet(gpio, pin, mode);
//    // gpio->MODER = (gpio->MODER & ~(3 << (2 * pin))) | ((mode & 3) << (2 * pin));
//
//    //gpio->OTYPER = (gpio->OTYPER & ~(1 << pin)) | ((mode >> 2) << pin);
//    //gpio->OSPEEDR = (gpio->OSPEEDR & ~(3 << (2 * pin))) | (2 << (2 * pin)); // full speed
//    MAP_GPIOPadConfigSet(gpio, pin, strength, pull);
//    //gpio->PUPDR = (gpio->PUPDR & ~(3 << (2 * pin))) | (pull << (2 * pin));
//    //gpio->AFR[pin >> 3] = (gpio->AFR[pin >> 3] & ~(15 << (4 * (pin & 7)))) | (alt << (4 * (pin & 7)));

    if ( pin >= 8 ) return;
    // does not consider the locked pins, which need special treatment
    if (type == MP_HAL_TYPE_ANALOG) {
        gpio->DEN &= ~(1 << pin);
        gpio->AMSEL |= (1 << pin);
    } else {
        gpio->AMSEL &= ~(1 << pin);
        gpio->DEN &= ~(1 << pin);
    }

    if (dir == MP_HAL_DIR_ALT) {
        gpio->AFSEL |= (1 << pin);
        gpio->PCTL = (gpio->PCTL & ~(0x000000F << (4 * pin))) | (alt << (4 * pin));
    } else {
        gpio->AFSEL &= ~(1 << pin);
        gpio->DIR = (gpio->DIR & ~(1 << pin)) | ((dir & 1) << pin);
    }

    if (type == MP_HAL_TYPE_PULL_UP) {
        gpio->PUR = (gpio->PUR & ~(1 << pin)) | (1 << pin);
    } else if (type == MP_HAL_TYPE_PULL_DOWN) {
        gpio->PDR = (gpio->PDR & ~(1 << pin)) | (1 << pin);
    } else if (type == MP_HAL_TYPE_PULL_NONE) {
        gpio->PUR &= ~(1 << pin);
        gpio->PDR &= ~(1 << pin);
    } else if (type == MP_HAL_TYPE_OPEN_DRAIN) {
        gpio->PUR &= ~(1 << pin);
        gpio->PDR &= ~(1 << pin);
        gpio->ODR &= (1 << pin);
    }
}

bool mp_hal_pin_config_alt(mp_hal_pin_obj_t pin, uint32_t mode, uint32_t pull, uint8_t fn, uint8_t unit) {
    const pin_af_obj_t *af = pin_find_af(pin, fn, unit);
    if (af == NULL) {
        return false;
    }
    mp_hal_pin_config(pin, mode, pull, af->idx);
    return true;
}
