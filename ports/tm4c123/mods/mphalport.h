// We use the ST Cube HAL library for most hardware peripherals
#include <stdbool.h>
#include "driverlib/gpio.h"
#include "inc/hw_gpio.h"
#include "driverlib/sysctl.h"
#include "driverlib/rom_map.h"
#include "driverlib/rom.h"
#include "pin.h"

extern const unsigned char mp_hal_status_to_errno_table[4];

NORETURN void mp_hal_raise(int status);
void mp_hal_set_interrupt_char(int c); // -1 to disable

// timing functions

#include "irq.h"

#if __CORTEX_M == 0
// Don't have raise_irq_pri on Cortex-M0 so keep IRQs enabled to have SysTick timing
#define mp_hal_quiet_timing_enter() (1)
#define mp_hal_quiet_timing_exit(irq_state) (void)(irq_state)
#else
#define mp_hal_quiet_timing_enter() raise_irq_pri(1)
#define mp_hal_quiet_timing_exit(irq_state) restore_irq_pri(irq_state)
#endif
#define mp_hal_delay_us_fast(us) mp_hal_delay_us(us)

void mp_hal_ticks_cpu_enable(void);
static inline mp_uint_t mp_hal_ticks_cpu(void) {
    #if __CORTEX_M == 0
    return 0;
    #else
    if (!(DWT->CTRL & DWT_CTRL_CYCCNTENA_Msk)) {
        mp_hal_ticks_cpu_enable();
    }
    return DWT->CYCCNT;
    #endif
}

// C-level pin HAL

#include "pin.h"


#define MP_HAL_PIN_FMT              "%q"
#define MP_HAL_DIR_INPUT            (GPIO_DIR_MODE_IN)
#define MP_HAL_DIR_OUTPUT           (GPIO_DIR_MODE_OUT)
#define MP_HAL_DIR_ALT              (GPIO_DIR_MODE_HW)
#define MP_HAL_TYPE_ANALOG          (GPIO_PIN_TYPE_ANALOG)
#define MP_HAL_TYPE_OPEN_DRAIN      (GPIO_PIN_TYPE_OD)
#define MP_HAL_TYPE_ALT_OPEN_DRAIN  (GPIO_PIN_TYPE_OD)
#define MP_HAL_TYPE_PULL_NONE       (GPIO_PIN_TYPE_STD)
#define MP_HAL_TYPE_PULL_UP         (GPIO_PIN_TYPE_STD_WPU)
#define MP_HAL_TYPE_PULL_DOWN       (GPIO_PIN_TYPE_STD_WPD)

#define MP_HAL_INT_RISING           (GPIO_RISING_EDGE)
#define MP_HAL_INT_FALLING          (GPIO_FALLING_EDGE)
#define MP_HAL_INT_BOTH             (GPIO_BOTH_EDGES)
#define MP_HAL_INT_HIGH             (GPIO_HIGH_LEVEL)
#define MP_HAL_INT_LOW              (GPIO_LOW_LEVEL)

#define MP_HAL_STREN_WEAK           (GPIO_STRENGTH_2MA)
#define MP_HAL_STREN_MED            (GPIO_STRENGTH_4MA)
#define MP_HAL_STREN_STRONG         (GPIO_STRENGTH_8MA)

#define GPIO_FETCH_PIN_CONF(p, a)

#define mp_hal_pin_obj_t const pin_obj_t*
#define mp_hal_get_pin_obj(o)   pin_find(o)
#define mp_hal_pin_name(p)      ((p)->name)
#define mp_hal_pin_input(p)     mp_hal_pin_config((p), MP_HAL_PIN_MODE_INPUT, MP_HAL_PIN_PULL_NONE, 0)
#define mp_hal_pin_output(p)    mp_hal_pin_config((p), MP_HAL_PIN_MODE_OUTPUT, MP_HAL_PIN_PULL_NONE, 0)
#define mp_hal_pin_open_drain(p) mp_hal_pin_config((p), MP_HAL_PIN_MODE_OPEN_DRAIN, MP_HAL_PIN_PULL_NONE, 0)

#define mp_hal_pin_high(p)      (MAP_GPIOPinWrite((p)->gpio, (p)->pin, (p)->pin))
#define mp_hal_pin_low(p)       (MAP_GPIOPinWrite((p)->gpio, (p)->pin, 0))

#define mp_hal_pin_od_low(p)    mp_hal_pin_low(p)
#define mp_hal_pin_od_high(p)   mp_hal_pin_high(p)
#define mp_hal_pin_read(p)      (MAP_GPIOPinRead((p)->gpio, (p)->pin))
#define mp_hal_pin_write(p, v)  do { if (v) { mp_hal_pin_high(p); } else { mp_hal_pin_low(p); } } while (0)


void mp_hal_gpio_clock_enable(GPIO_TypeDef *gpio);
void mp_hal_pin_config(mp_hal_pin_obj_t pin, uint32_t dir, uint32_t type, uint32_t strength, uint32_t alt);
bool mp_hal_pin_config_alt(mp_hal_pin_obj_t pin, uint32_t dir, uint32_t type, uint8_t fn, uint8_t unit);
