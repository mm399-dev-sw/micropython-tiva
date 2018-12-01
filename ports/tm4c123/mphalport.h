extern const unsigned char mp_hal_status_to_errno_table[4];

void mp_hal_set_interrupt_char(int c); // -1 to disable

// C-level pin HAL
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include "hal/hal_pins.h"
#include "mods/pin.h"
#include "driverlib/gpio.h"

#define MP_HAL_PIN_FMT                  "%q"
#define MP_HAL_PIN_MODE_INPUT           (GPIO_DIR_MODE_IN)
#define MP_HAL_PIN_MODE_OUTPUT          (GPIO_DIR_MODE_OUT)
#define MP_HAL_PIN_MODE_ALT             (GPIO_DIR_MODE_HW)
#define MP_HAL_PIN_MODE_ANALOG          (GPIO_PIN_TYPE_ANALOG)


#define MP_HAL_PIN_MODE_OPEN_DRAIN      (GPIO_PIN_TYPE_OD)
#define MP_HAL_PIN_MODE_ALT_OPEN_DRAIN  (GPIO_PIN_TYPE_OD)
#define MP_HAL_PIN_PULL_NONE            (GPIO_PIN_TYPE_STD)
#define MP_HAL_PIN_PULL_UP              (GPIO_PIN_TYPE_STD_WPU)
#define MP_HAL_PIN_PULL_DOWN            (GPIO_PIN_TYPE_STD_WPD)

#define mp_hal_pin_obj_t const pin_obj_t*
#define mp_hal_get_pin_obj(o)   pin_find(o)
#define mp_hal_pin_name(p)      ((p)->name)
#define mp_hal_pin_input(p)     gpio_init((p)->port, (p)->pin_mask, MP_HAL_PIN_MODE_INPUT, MP_HAL_PIN_PULL_NONE, GPIO_STRENGTH_2MA)
#define mp_hal_pin_output(p)    gpio_init((p)->port, (p)->pin_mask, MP_HAL_PIN_MODE_OUTPUT, MP_HAL_PIN_PULL_NONE, GPIO_STRENGTH_2MA)
#define mp_hal_pin_open_drain(p) gpio_init((p)->port, (p)->pin_mask, MP_HAL_PIN_MODE_OUTPUT, MP_HAL_PIN_MODE_OPEN_DRAIN, GPIO_STRENGTH_2MA)

#define mp_hal_pin_high(p)      pin_high(p)
#define mp_hal_pin_low(p)       pin_low(p)

#define mp_hal_pin_od_low(p)    mp_hal_pin_low(p)
#define mp_hal_pin_od_high(p)   mp_hal_pin_high(p)
#define mp_hal_pin_read(p)      pin_read(p)
#define mp_hal_pin_write(p, v)  pin_write(p, v)
