#include "py/obj.h"
#include "pin.h"
#include "inc/hw_gpio.h"
#include "driverlib/rom_map.h"
#include "driverlib/gpio.h"
#include "driverlib/rom.h"

// Returns the pin mode. This value returned by this macro should be one of:
// GPIO_MODE_INPUT, GPIO_MODE_OUTPUT_PP, GPIO_MODE_OUTPUT_OD,
// GPIO_MODE_AF_PP, GPIO_MODE_AF_OD, or GPIO_MODE_ANALOG.

uint32_t pin_get_dir(const pin_obj_t *pin_obj) {
    GPIO_TypeDef *gpio = pin_obj->gpio;
    //uint32_t pin = pin_obj->pin;
    uint32_t dir = MAP_GPIODirModeGet(gpio);
//    uint32_t dir;
//
//    if (gpio->AFSEL & (1 << pin)) {
//        dir = MP_HAL_DIR_ALT;
//    } else {
//        if ((gpio->DIR & (1 << pin)) {
//            dir = MP_HAL_DIR_OUTPUT;
//        } else {
//            dir = MP_HAL_DIR_INPUT;
//        }
//    }
    return dir;
}

uint32_t pin_get_type(const pin_obj_t *pin) {
    GPIO_TypeDef *gpio = pin->gpio;
    uint32_t type;
    uint32_t strength;
    MAP_GPIOPadConfigGet(pin->gpio, pin->pin, &strength, &type);
    return type;
}

// Returns the pin pullup/pulldown. The value returned by this macro should
// be one of GPIO_NOPULL, GPIO_PULLUP, or GPIO_PULLDOWN.

uint32_t pin_get_strength(const pin_obj_t *pin) {
    GPIO_TypeDef *gpio = pin->gpio;
    uint32_t type;
    uint32_t strength;
    MAP_GPIOPadConfigGet(pin->gpio, pin->pin, &strength, &type);
    return strength;
}

// Returns the af (alternate function) index currently set for a pin.

uint32_t pin_get_af(const pin_obj_t *pin) {
    if (pin->gpio->AFSEL & (1 << pin->pin)) {
        return (pin->gpio->PCTL >> ((pin->pin & 7) * 4)) & 0xf;
    } else return NULL;

}

