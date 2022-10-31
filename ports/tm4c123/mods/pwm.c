/*
 * This file is part of the Micro Python project, http://micropython.org/
 *
 * The MIT License (MIT)
 *
 * Copyright (c) 2016-2021 Damien P. George
 * Copyright (c) 2018 Alan Dragomirecky
 * Copyright (c) 2020 Antoine Aubert
 * Copyright (c) 2021 Ihor Nehrutsa
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

#include <stdint.h>
#include "py/runtime.h"
#include "py/gc.h"
#include "py/mphal.h"
// #include "inc/tm4c123gh6pm.h"
#include "inc/hw_memmap.h"
#include "inc/hw_sysctl.h"
#include "inc/hw_gpio.h"
#include "inc/hw_pwm.h"
#include "driverlib/sysctl.h"
#include "driverlib/gpio.h"
#include "driverlib/pwm.h"
#include "driverlib/interrupt.h"

#include "pwm.h"
#include "pin.h"

// individual pwm modules
enum {PWM_M0PWM0, PWM_M0PWM1, PWM_M0PWM2, PWM_M0PWM3, PWM_M0PWM4, PWM_M0PWM5, PWM_M0PWM6, PWM_M0PWM7,
        PWM_M1PWM0, PWM_M1PWM1, PWM_M1PWM2, PWM_M1PWM3, PWM_M1PWM4, PWM_M1PWM5, PWM_M1PWM6, PWM_M1PWM7};
// deadband identifier
enum {PWM_DB_FALLING, PWM_DB_RISING};
// pwm count mode
enum {PWM_COUNT_DOWN, PWM_COUNT_UP_DOWN};
// action to take for pwm pin
enum {PWM_ACT_NOTHING, PWM_ACT_INVERT, PWM_ACT_LOW, PWM_ACT_HIGH};
// event triggering pwm pin action
enum {PWM_EVENT_ZERO = 0, PWM_EVENT_LOAD = 2, PWM_EVENT_CMPA_UP = 4, PWM_EVENT_CMPA_DOWN = 6, PWM_EVENT_CMPB_UP = 8, PWM_EVENT_CMPB_DOWN = 10};
// pwm clock divider
enum {PWM_CLK_DIV_2, PWM_CLK_DIV_4, PWM_CLK_DIV_8, PWM_CLK_DIV_16, PWM_CLK_DIV_32, PWM_CLK_DIV_64, PWM_CLK_DIV_1 = 0xff};

static const uint8_t pwm_pin_num_list[MICROPY_HW_MAX_PWM][2] = 
{
    {1,     0xff},  // 0xff to indicate no alternative pin
    {4,     0xff},
    {58,    0xff},
    {57,    0xff},
    {59,    0xff},
    {60,    0xff},
    {16,    61},
    {15,    62},

    {61,    0xff},
    {62,    0xff},
    {23,    59},
    {24,    60},
    {28,    0xff},
    {29,    0xff},
    {30,    0xff},
    {31,    0xff},
};

typedef struct _machine_pwm_obj_t
{
    mp_obj_base_t       base;
    const pin_obj_t*    pin;            // output pin for pwm, can be NULL (None)
    uint8_t             id;             // id 0..15 for pwm generator
    uint8_t             id_mod;         // pwm module number
    uint8_t             id_gen;         // pwm generator number

    uint16_t            mode;           // pwm count mode
    bool                active;         // pwm output active
    bool                invert;         // invert pwm output
    uint32_t            freq;           // pwm frequency
    uint16_t            load;           // internal pwm load value
    uint8_t             duty;           // pwm duty cycle 0..100
    mp_obj_t            irq;            // interrupt isr
    uint32_t            db_falling;     // deadband delay for falling edge
    uint32_t            db_rising;      // deadband delay for rising edge
    const pin_obj_t*    fault_pin;      // pin to assert fault condition, can be NULL (None)

} machine_pwm_obj_t;

static machine_pwm_obj_t machine_pwm_obj[MICROPY_HW_MAX_PWM] = {};
static bool machine_pwm_obj_in_use[MICROPY_HW_MAX_PWM] = {};
static uint8_t machine_pwm_clock_divider_value = PWM_CLK_DIV_1;

void delay_cycles(uint16_t n)
{
    for (uint16_t i = 0; i < n; i++)
        asm("nop");
}

uint8_t pwm_get_id_from_pin(const pin_obj_t* pin, bool alternate_pin)
{
    for (uint8_t i = 0; i < MICROPY_HW_MAX_PWM; i++)
    {
        if (pwm_pin_num_list[i][alternate_pin] == pin->pin_num)
            return i;
    }
    return 0xff;
}

uint8_t pwm_get_port_index_from_pin(const pin_obj_t* pin)
{
    return (uint8_t) qstr_str(pin->name)[1] - (uint8_t) 'A';
}

uint8_t pwm_get_pin_index_from_pin(const pin_obj_t* pin)
{
    return (uint8_t) qstr_str(pin->name)[2] - (uint8_t) '0';
}

uint32_t pwm_get_pwm_base_reg(machine_pwm_obj_t* self)
{
    return (self->id_mod ? PWM1_BASE: PWM0_BASE);
}

void pwm_set_clock(uint8_t id_mod, bool val)
{
    if (val)
    {
        HWREG(SYSCTL_RCGCPWM) |= 1 << id_mod;
        // ensure pwm is ready
        delay_cycles(3);
    }
    else
        HWREG(SYSCTL_RCGCPWM) &= ~(1 << id_mod);
}

void pwm_set_enable(machine_pwm_obj_t* self, bool val)
{
    // get register address of MxPWMxCTL
    uint32_t reg = pwm_get_pwm_base_reg(self) + PWM_O_0_CTL + self->id_gen * 0x40;
    // start timer in MxPWMxCTL
    if (val)
        HWREG(reg) |= 1;
    else
        HWREG(reg) &= ~1;
}

void pwm_set_sync(machine_pwm_obj_t* self, bool val)
{
    // get register address of MxPWMxCTL
    uint32_t reg = pwm_get_pwm_base_reg(self) + PWM_O_0_CTL + self->id_gen * 0x40;
    // set global sync in MxPWMxCTL
    if (val)
        HWREG(reg) |= 0xfff8;
    // set immediate sync in MxPWMxCTL
    else
        HWREG(reg) &= ~0xfff8;
}

void pwm_set_gpio(machine_pwm_obj_t* self, bool val)
{
    // abort if no pin assigned
    if (!self->pin)
        return;
    uint32_t gpio_port_base_map[] = {GPIO_PORTA_AHB_BASE, GPIO_PORTB_AHB_BASE, GPIO_PORTC_AHB_BASE, GPIO_PORTD_AHB_BASE, GPIO_PORTE_AHB_BASE, GPIO_PORTF_AHB_BASE};
    
    uint8_t port_index = pwm_get_port_index_from_pin(self->pin);
    uint8_t pin_index = pwm_get_pin_index_from_pin(self->pin);
    uint8_t pctl_val = (self->id < PWM_M1PWM0 ? 4: 5);
    if (val)
    {
        // enable clock for gpio module
        HWREG(SYSCTL_RCGCGPIO) |= 1 << port_index;
        delay_cycles(3);

        // unlock gpio
        HWREG(gpio_port_base_map[port_index] + GPIO_O_LOCK) = GPIO_LOCK_KEY;
        HWREG(gpio_port_base_map[port_index] + GPIO_O_CR) |= 1 << pin_index;
        
        // enable alternate pin function
        HWREG(gpio_port_base_map[port_index] + GPIO_O_AFSEL) |= 1 << pin_index;
        HWREG(gpio_port_base_map[port_index] + GPIO_O_PCTL) |= pctl_val << (4 * pin_index);
        HWREG(gpio_port_base_map[port_index] + GPIO_O_DEN) |= 1 << pin_index;
    }
    else
    {
        // disable alternate pin function
        HWREG(gpio_port_base_map[port_index] + GPIO_O_DEN) &= ~(1 << pin_index);
        HWREG(gpio_port_base_map[port_index] + GPIO_O_PCTL) &= ~(pctl_val << (4 * pin_index));
        HWREG(gpio_port_base_map[port_index] + GPIO_O_AFSEL) &= ~(1 << pin_index);
    }
}

void pwm_set_mode(machine_pwm_obj_t* self, uint16_t val)
{
    // get register address of MxPWMxCTL
    uint32_t reg = pwm_get_pwm_base_reg(self) + PWM_O_0_CTL + self->id_gen * 0x40;
    // set mode in MxPWMxCTL
    if (val & 1)
        HWREG(reg) |= 1 << 1;
    else
        HWREG(reg) &= ~(1 << 1);
    
    // get offset for register address MxPWMxGENx
    uint8_t reg_offset = (self->id % 2 ? PWM_O_0_GENB: PWM_O_0_GENA);
    // get register address of MxPWMxGENx
    reg = pwm_get_pwm_base_reg(self) + reg_offset + self->id_gen * 0x40;
    // set match behavior
    HWREG(reg) &= ~0xfff;
    HWREG(reg) |= val >> 1;
}

void pwm_clear_counter(machine_pwm_obj_t* self)
{
    // get register address of PWMxCOUNT
    uint32_t reg = pwm_get_pwm_base_reg(self) + PWM_O_SYNC;
    // clear counter value
    HWREG(reg) |= 1 << self->id_gen;
}

void pwm_update_active(machine_pwm_obj_t* self, bool val)
{
    // get register address of MxPWMENABLE
    uint32_t reg = pwm_get_pwm_base_reg(self) + PWM_O_ENABLE;
    // enable pin output in MxPWMENABLE
    if (val)
        HWREG(reg) |= 1 << (self->id % PWM_M1PWM0);
    else
        HWREG(reg) &= ~(1 << (self->id % PWM_M1PWM0));
    self->active = val;
}

void pwm_update_invert(machine_pwm_obj_t* self, bool val)
{
    // get register address of MxPWMINVERT
    uint32_t reg = pwm_get_pwm_base_reg(self) + PWM_O_INVERT;
    if (val)
        HWREG(reg) |= 1 << (self->id % PWM_M1PWM0);

    self->invert = val;
}

void pwm_update_clock_divider(uint8_t val)
{
    if (val > PWM_CLK_DIV_64 && val != PWM_CLK_DIV_1)
        mp_raise_ValueError(MP_ERROR_TEXT("Value out of range"));
    
    // change requested
    if (val != machine_pwm_clock_divider_value)
    {
        pwm_set_clock(0, 0);
        pwm_set_clock(1, 0);
        // disable divider
        if (val == PWM_CLK_DIV_1)
            HWREG(SYSCTL_RCC) &= ~(1<<20);
        // set divider
        else
        {
            uint32_t write_val = val << 17;
            // enable divider
            if (machine_pwm_clock_divider_value == PWM_CLK_DIV_1)
                write_val |= 1<<20;
            // clear and set divider
            HWREG(SYSCTL_RCC) &= ~(0x0f < 17);
            HWREG(SYSCTL_RCC) |= write_val;
        }
        machine_pwm_clock_divider_value = val;
        pwm_set_clock(0, 1);
        pwm_set_clock(1, 1);
    }
}

void pwm_update_freq(machine_pwm_obj_t* self, uint32_t val)
{
    // disable timer if freq is 0
    if (!val)
    {
        pwm_set_enable(self, 0);
        self->freq = 0;
        return;
    }
    // reenable timer if freq was 0
    else if (val && !self->freq)
        pwm_set_enable(self, 1);

    uint32_t sys_freq = SysCtlClockGet();
    uint32_t pwm_freq = (machine_pwm_clock_divider_value != PWM_CLK_DIV_1 ? sys_freq / 2<<machine_pwm_clock_divider_value : sys_freq);

    if (val > pwm_freq / 10)
        mp_raise_ValueError(MP_ERROR_TEXT("Value out of range, must be <= sys_clk / clk_div / 10"));
    
    // get register address of MxPWMxLOAD
    uint32_t reg = pwm_get_pwm_base_reg(self) + PWM_O_0_LOAD + self->id_gen * 0x40;
    // get second pwm of generator
    machine_pwm_obj_t* twin = &machine_pwm_obj[(self->id % 2 ? self->id - 1: self->id + 1)];
    // if twin pwm is active dont allow different frequency
    if (twin->freq && val != twin->freq)
        mp_raise_ValueError(MP_ERROR_TEXT("Frequency must be the same inside one PWM generator"));
    
    uint32_t load_val = pwm_freq / val;
    if (load_val > 0xffff)
        mp_raise_ValueError(MP_ERROR_TEXT("Frequency to low, consider adjusting the PWM clock divider via PWM.clock_divider(PWM.CLK_DIV_x)"));
    HWREG(reg) &= ~0xffff;
    HWREG(reg) |= load_val;

    pwm_clear_counter(self);
    self->load = load_val;
    self->freq = val;
}

void pwm_update_duty(machine_pwm_obj_t* self, uint8_t val)
{
    if (val > 100)
        mp_raise_ValueError(MP_ERROR_TEXT("Value out of range"));

    // get offset for register address MxPWMxCMPx
    uint8_t reg_offset = (self->id % 2 ? PWM_O_0_CMPB: PWM_O_0_CMPA);
    // get register address of MxPWMxCMPx
    uint32_t reg = pwm_get_pwm_base_reg(self) + reg_offset + self->id_gen * 0x40;

    uint16_t comp_val = 0;
    if (self->mode & 1)
        ;
    else
        comp_val = val * self->load / 100;
    
    HWREG(reg) &= ~0xffff;
    HWREG(reg) |= (comp_val == self->load ? comp_val - 1: comp_val);
    self->duty = val;
}

void pwm_update_sync(machine_pwm_obj_t* self, const machine_pwm_obj_t* tbs, uint8_t n_tbs)
{
    if (n_tbs)
    {

    }
    else
    ;
}

void pwm_update_irq(machine_pwm_obj_t* self, mp_obj_t irq)
{

}

void pwm_update_db(machine_pwm_obj_t* self, uint32_t db, uint8_t db_type)
{
    switch (db_type)
    {
    case PWM_DB_FALLING:    self->db_falling = db;  break;
    case PWM_DB_RISING:     self->db_rising = db;   break;
    
    default: mp_raise_ValueError(MP_ERROR_TEXT("Deadband type out of range"));   break;
    }
}

void pwm_update_fault(machine_pwm_obj_t* self, bool val)
{

}

void pwm_init(machine_pwm_obj_t* self, const pin_obj_t* pin, uint16_t mode, bool active, bool invert, uint32_t freq, uint8_t duty, uint32_t db_falling, uint32_t db_rising, mp_obj_t irq, mp_obj_t fault)
{
    pwm_set_clock(self->id_mod, 1);
    pwm_set_enable(self, 0);
    pwm_update_active(self, 0);
    if (pin)
        self->pin = pin;
    pwm_set_gpio(self, 1);
    pwm_set_mode(self, mode);
    // pwm_set_sync(self, 1);
    pwm_update_freq(self, freq);
    pwm_update_duty(self, duty);

    pwm_update_invert(self, invert);
    pwm_update_db(self, db_falling, PWM_DB_FALLING);
    pwm_update_db(self, db_rising, PWM_DB_RISING);
    pwm_update_irq(self, irq);
    pwm_update_fault(self, fault);

    pwm_set_enable(self, 1);
    pwm_update_active(self, 1);
}

void pwm_deinit(machine_pwm_obj_t* self)
{
    pwm_update_active(self, 0);
    // get second pwm of generator
    uint8_t twin_id = (self->id % 2 ? self->id - 1: self->id + 1);
    // if twin pwm is active dont disable generator
    if (!machine_pwm_obj_in_use[twin_id])
        pwm_set_enable(self, 0);
    if (self->pin)
        pwm_set_gpio(self, 0);
    self->freq = 0;
    machine_pwm_obj_in_use[self->id] = false;
}


// ##### helper functions #####
mp_obj_t machine_pwm_init_helper(machine_pwm_obj_t* self, size_t n_args, const mp_obj_t* pos_args, mp_map_t* kw_args, const pin_obj_t* pin)
{
    enum {ARG_pin, ARG_active, ARG_invert, ARG_mode, ARG_freq, ARG_duty, ARG_db_falling, ARG_db_rising, ARG_irq, ARG_fault};
    static const mp_arg_t allowed_args[] = 
    {
        {MP_QSTR_pin,           MP_ARG_KW_ONLY | MP_ARG_OBJ,    {.u_obj = mp_const_none}},
        {MP_QSTR_active,        MP_ARG_KW_ONLY | MP_ARG_BOOL,   {.u_int = true}},
        {MP_QSTR_invert,        MP_ARG_KW_ONLY | MP_ARG_BOOL,   {.u_int = false}},
        {MP_QSTR_mode,          MP_ARG_KW_ONLY | MP_ARG_BOOL,   {.u_int = PWM_COUNT_DOWN}},
        {MP_QSTR_freq,          MP_ARG_KW_ONLY | MP_ARG_INT,    {.u_int = 0}},
        {MP_QSTR_duty,          MP_ARG_KW_ONLY | MP_ARG_INT,    {.u_int = 0}},
        {MP_QSTR_db_falling,    MP_ARG_KW_ONLY | MP_ARG_INT,    {.u_int = 0}},
        {MP_QSTR_db_rising,     MP_ARG_KW_ONLY | MP_ARG_INT,    {.u_int = 0}},
        {MP_QSTR_irq,           MP_ARG_KW_ONLY | MP_ARG_OBJ,    {.u_obj = mp_const_none}},
        {MP_QSTR_fault,         MP_ARG_KW_ONLY | MP_ARG_OBJ,    {.u_obj = mp_const_none}},
    };
    mp_arg_val_t args[MP_ARRAY_SIZE(allowed_args)];
    mp_arg_parse_all(n_args, pos_args, kw_args, MP_ARRAY_SIZE(allowed_args), allowed_args, args);

    // get arguments
    if (!pin)               // dont overwrite if created via pin
        pin =               (args[ARG_pin].u_obj == mp_const_none ? NULL: args[ARG_pin].u_obj);
    uint16_t mode =         (args[ARG_mode].u_int == PWM_COUNT_DOWN ? PWM_COUNT_DOWN | (PWM_ACT_LOW << PWM_EVENT_ZERO | PWM_ACT_HIGH << PWM_EVENT_CMPA_DOWN) << 1 : args[ARG_mode].u_int);
    bool active =           args[ARG_active].u_int;
    bool invert =           args[ARG_invert].u_int;
    uint32_t freq =         args[ARG_freq].u_int;
    uint8_t duty =          args[ARG_duty].u_int;
    uint32_t db_falling =   args[ARG_db_falling].u_int;
    uint32_t db_rising =    args[ARG_db_rising].u_int;
    mp_obj_t irq =          args[ARG_irq].u_obj;
    mp_obj_t fault =        args[ARG_fault].u_obj;

    pwm_init(self, pin, mode, active, invert, freq, duty, db_falling, db_rising, irq, fault);
    
    return self;
}

mp_obj_t machine_pwm_deinit_helper(machine_pwm_obj_t* self)
{
    pwm_deinit(self);
    self = (mp_obj_t) mp_const_none;
    return self;
}

mp_obj_t machine_pwm_active_helper(machine_pwm_obj_t* self, size_t n_args, const mp_obj_t* pos_args)
{
    if (n_args)
    {
        if (mp_obj_is_bool(pos_args[0]) || mp_obj_is_int(pos_args[0]))
            pwm_update_active(self, MP_OBJ_SMALL_INT_VALUE(pos_args[0]));
        else
            mp_raise_ValueError(MP_ERROR_TEXT("Type mismatch: not an int"));
    }
    return mp_obj_new_bool(self->active);
}

mp_obj_t machine_pwm_invert_helper(machine_pwm_obj_t* self, size_t n_args, const mp_obj_t* pos_args)
{
    if (n_args)
    {
        if (mp_obj_is_bool(pos_args[0]) || mp_obj_is_int(pos_args[0]))
            pwm_update_invert(self, MP_OBJ_SMALL_INT_VALUE(pos_args[0]));
        else
            mp_raise_ValueError(MP_ERROR_TEXT("Type mismatch: not an int"));
    }
    return mp_obj_new_bool(self->active);
}

mp_obj_t machine_pwm_clock_divider_helper(machine_pwm_obj_t* self, size_t n_args, const mp_obj_t* pos_args)
{
    if (n_args)
    {
        if (mp_obj_is_int(pos_args[0]))
            pwm_update_clock_divider(MP_OBJ_SMALL_INT_VALUE(pos_args[0]));
        else
            mp_raise_ValueError(MP_ERROR_TEXT("Type mismatch: not an int"));
    }
    return mp_obj_new_int(machine_pwm_clock_divider_value);
}

mp_obj_t machine_pwm_freq_helper(machine_pwm_obj_t* self, size_t n_args, const mp_obj_t* pos_args)
{
    if (n_args)
    {
        if (mp_obj_is_int(pos_args[0]))
            pwm_update_freq(self, (uint32_t) MP_OBJ_SMALL_INT_VALUE(pos_args[0]));
        else
            mp_raise_ValueError(MP_ERROR_TEXT("Type mismatch: not an int"));
    }
    return mp_obj_new_int(self->freq);
}

mp_obj_t machine_pwm_duty_helper(machine_pwm_obj_t* self, size_t n_args, const mp_obj_t* pos_args)
{
    if (n_args)
    {
        if (mp_obj_is_int(pos_args[0]))
            pwm_update_duty(self, MP_OBJ_SMALL_INT_VALUE(pos_args[0]));
        else
            mp_raise_ValueError(MP_ERROR_TEXT("Type mismatch: not an int"));
    }
    return mp_obj_new_int(self->duty);
}

mp_obj_t machine_pwm_sync_helper(machine_pwm_obj_t* self, size_t n_args, const mp_obj_t* pos_args)
{
    if (n_args)
    {
        for (uint8_t i = 0; i < n_args; i++)
        {
            if (!mp_obj_is_type(pos_args[i], &machine_pwm_type))
                mp_raise_ValueError(MP_ERROR_TEXT("Type mismatch: not a PWM object"));
        }
    }
    pwm_update_sync(self, (machine_pwm_obj_t*) pos_args, n_args);
    return self;
}

mp_obj_t machine_pwm_db_helper(machine_pwm_obj_t* self, size_t n_args, const mp_obj_t* pos_args, mp_map_t* kw_args)
{
    enum {ARG_db_falling, ARG_db_rising};
    static const mp_arg_t allowed_args[] = 
    {
        {MP_QSTR_db_falling,    MP_ARG_KW_ONLY | MP_ARG_INT,    {.u_int = -1}},
        {MP_QSTR_db_rising,     MP_ARG_KW_ONLY | MP_ARG_INT,    {.u_int = -1}},
    };
    mp_arg_val_t args[MP_ARRAY_SIZE(allowed_args)];
    mp_arg_parse_all(n_args, pos_args, kw_args, MP_ARRAY_SIZE(allowed_args), allowed_args, args);

    if (args[ARG_db_falling].u_int != -1)
        pwm_update_db(self, args[ARG_db_falling].u_int, PWM_DB_FALLING);
    if (args[ARG_db_rising].u_int != -1)
        pwm_update_db(self, args[ARG_db_rising].u_int, PWM_DB_RISING);

    return self;
}

mp_obj_t machine_pwm_irq_helper(machine_pwm_obj_t* self, size_t n_args, const mp_obj_t* pos_args)
{
    return self;
}

mp_obj_t machine_pwm_fault_helper(machine_pwm_obj_t* self, size_t n_args, const mp_obj_t* pos_args)
{
    return self;
}


// ##### micropython functions and declarations #####
static mp_obj_t mp_machine_pwm_make_new(const mp_obj_type_t* type, size_t n_args, size_t n_kw, const mp_obj_t* args)
{
    uint8_t pwm_id = 0xff;
    const pin_obj_t* pin = NULL;

    // init via pin
    if (mp_obj_is_type(args[0], &pin_type))
    {
        pin = pin_find(args[0]);
        pwm_id = pwm_get_id_from_pin(pin, 0);
        if (pwm_id == 0xff)
            mp_raise_ValueError(MP_ERROR_TEXT("Pin incapable of PWM function"));
    }
    // init via PWM module id
    else if (mp_obj_is_int(args[0]) && MP_OBJ_SMALL_INT_VALUE(args[0]) >= PWM_M0PWM0 && MP_OBJ_SMALL_INT_VALUE(args[0]) <= PWM_M1PWM7)
        pwm_id = MP_OBJ_SMALL_INT_VALUE(args[0]);
    // invalid argument
    else
        mp_raise_ValueError(MP_ERROR_TEXT("Type mismatch: not a Pin object or valid MxPWMx pwm identifier"));


    // create (select) pwm object
    machine_pwm_obj_t* self = &machine_pwm_obj[pwm_id];
    machine_pwm_obj_in_use[pwm_id] = true;                  // flag pwm object as in use

    self->base.type = &machine_pwm_type;
    self->id = pwm_id;
    self->id_mod = !(pwm_id < PWM_M1PWM0);
    self->id_gen = (pwm_id % PWM_M1PWM0) - (pwm_id % 2);

    self->pin = pin;
    self->active = false;
    self->invert = false;
    self->mode = PWM_COUNT_DOWN;
    self->duty = 0;
    self->freq = 0;
    self->load = 0;
    self->db_falling = 0;
    self->db_rising = 0;
    self->irq = mp_const_none;

    mp_arg_check_num(n_args, n_kw, 1, MP_OBJ_FUN_ARGS_MAX, true);
    mp_map_t kw_args;
    mp_map_init_fixed_table(&kw_args, n_kw, args + n_args);

    return machine_pwm_init_helper(self, n_args - 1, args + 1, &kw_args, pin);
}

static mp_obj_t machine_pwm_init(size_t n_args, const mp_obj_t* args, mp_map_t* kw_args)
{
    return machine_pwm_init_helper(args[0], n_args - 1, args + 1, kw_args, NULL);
}
static MP_DEFINE_CONST_FUN_OBJ_KW(machine_pwm_init_obj, 1, machine_pwm_init);

static mp_obj_t machine_pwm_deinit(mp_obj_t self)
{
    return machine_pwm_deinit_helper(self);
}
static MP_DEFINE_CONST_FUN_OBJ_1(machine_pwm_deinit_obj, machine_pwm_deinit);

static mp_obj_t machine_pwm_active(size_t n_args, const mp_obj_t* args)
{
    return machine_pwm_active_helper(args[0], n_args - 1, args + 1);
}
static MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(machine_pwm_active_obj, 1, 2, machine_pwm_active);

static mp_obj_t machine_pwm_invert(size_t n_args, const mp_obj_t* args)
{
    return machine_pwm_invert_helper(args[0], n_args - 1, args + 1);
}
static MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(machine_pwm_invert_obj, 1, 2, machine_pwm_invert);

static mp_obj_t machine_pwm_clock_divider(size_t n_args, const mp_obj_t* args)
{
    return machine_pwm_clock_divider_helper(args[0], n_args - 1, args + 1);
}
static MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(machine_pwm_clock_divider_obj, 1, 2, machine_pwm_clock_divider);

static mp_obj_t machine_pwm_freq(size_t n_args, const mp_obj_t* args)
{
    return machine_pwm_freq_helper(args[0], n_args - 1, args + 1);
}
static MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(machine_pwm_freq_obj, 1, 2, machine_pwm_freq);

static mp_obj_t machine_pwm_duty(size_t n_args, const mp_obj_t* args)
{
    return machine_pwm_duty_helper(args[0], n_args - 1, args + 1);
}
static MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(machine_pwm_duty_obj, 1, 2, machine_pwm_duty);

static mp_obj_t machine_pwm_sync(size_t n_args, const mp_obj_t* args)
{
    return machine_pwm_sync_helper(args[0], n_args - 1, args + 1);
}
static MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(machine_pwm_sync_obj, 1, MICROPY_HW_MAX_PWM, machine_pwm_sync);

static mp_obj_t machine_pwm_db(size_t n_args, const mp_obj_t* args, mp_map_t* kw_args)
{
    return machine_pwm_db_helper(args[0], n_args - 1, args + 1, kw_args);
}
static MP_DEFINE_CONST_FUN_OBJ_KW(machine_pwm_db_obj, 1, machine_pwm_db);

static mp_obj_t machine_pwm_irq(size_t n_args, const mp_obj_t* args)
{
    return machine_pwm_irq_helper(args[0], n_args - 1, args + 1);
}
static MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(machine_pwm_irq_obj, 1, 2, machine_pwm_irq);

static mp_obj_t machine_pwm_fault(size_t n_args, const mp_obj_t* args)
{
    return machine_pwm_fault_helper(args[0], n_args - 1, args + 1);
}
static MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(machine_pwm_fault_obj, 1, 2, machine_pwm_fault);


void pwm_init0(void)
{
    mp_obj_list_init(&MP_STATE_PORT(mp_pwm_obj_list), 0);
    // setup objects frequencies for correct initialization later
    for (uint8_t i = 0; i < MICROPY_HW_MAX_PWM; i++)
        machine_pwm_obj[i].freq = 0;
}

static void machine_pwm_print(const mp_print_t* print, mp_obj_t self_in, mp_print_kind_t kind)
{
    machine_pwm_obj_t* self = MP_OBJ_TO_PTR(self_in);
    mp_printf(print, "PWM(Pin(%s), id = %u, mode = %u, active = %u, invert = %u, freq = %lu, duty = %u, irq = %s, db_falling = %lu, db_rising = %lu)",
               (self->pin ? qstr_str(self->pin->name): "None"), self->id, self->mode, self->active, self->invert, self->freq, self->duty, "None", self->db_falling, self->db_rising);
}

static const mp_rom_map_elem_t machine_pwm_locals_dict_table[] = 
{
    // class methods
    {MP_ROM_QSTR(MP_QSTR_init),             MP_ROM_PTR(&machine_pwm_init_obj)},
    {MP_ROM_QSTR(MP_QSTR_deinit),           MP_ROM_PTR(&machine_pwm_deinit_obj)},
    {MP_ROM_QSTR(MP_QSTR_active),           MP_ROM_PTR(&machine_pwm_active_obj)},
    {MP_ROM_QSTR(MP_QSTR_invert),           MP_ROM_PTR(&machine_pwm_invert_obj)},
    {MP_ROM_QSTR(MP_QSTR_clock_divider),    MP_ROM_PTR(&machine_pwm_clock_divider_obj)},
    {MP_ROM_QSTR(MP_QSTR_freq),             MP_ROM_PTR(&machine_pwm_freq_obj)},
    {MP_ROM_QSTR(MP_QSTR_duty),             MP_ROM_PTR(&machine_pwm_duty_obj)},
    {MP_ROM_QSTR(MP_QSTR_sync),             MP_ROM_PTR(&machine_pwm_sync_obj)},
    {MP_ROM_QSTR(MP_QSTR_db),               MP_ROM_PTR(&machine_pwm_db_obj)},
    {MP_ROM_QSTR(MP_QSTR_irq),              MP_ROM_PTR(&machine_pwm_irq_obj)},
    {MP_ROM_QSTR(MP_QSTR_fault),            MP_ROM_PTR(&machine_pwm_fault_obj)},

    // class constants
    {MP_ROM_QSTR(MP_QSTR_M0PWM0),           MP_ROM_INT(PWM_M0PWM0)},
    {MP_ROM_QSTR(MP_QSTR_M0PWM1),           MP_ROM_INT(PWM_M0PWM1)},
    {MP_ROM_QSTR(MP_QSTR_M0PWM2),           MP_ROM_INT(PWM_M0PWM2)},
    {MP_ROM_QSTR(MP_QSTR_M0PWM3),           MP_ROM_INT(PWM_M0PWM3)},
    {MP_ROM_QSTR(MP_QSTR_M0PWM4),           MP_ROM_INT(PWM_M0PWM4)},
    {MP_ROM_QSTR(MP_QSTR_M0PWM5),           MP_ROM_INT(PWM_M0PWM5)},
    {MP_ROM_QSTR(MP_QSTR_M0PWM6),           MP_ROM_INT(PWM_M0PWM6)},
    {MP_ROM_QSTR(MP_QSTR_M0PWM7),           MP_ROM_INT(PWM_M0PWM7)},
    {MP_ROM_QSTR(MP_QSTR_M1PWM0),           MP_ROM_INT(PWM_M1PWM0)},
    {MP_ROM_QSTR(MP_QSTR_M1PWM1),           MP_ROM_INT(PWM_M1PWM1)},
    {MP_ROM_QSTR(MP_QSTR_M1PWM2),           MP_ROM_INT(PWM_M1PWM2)},
    {MP_ROM_QSTR(MP_QSTR_M1PWM3),           MP_ROM_INT(PWM_M1PWM3)},
    {MP_ROM_QSTR(MP_QSTR_M1PWM4),           MP_ROM_INT(PWM_M1PWM4)},
    {MP_ROM_QSTR(MP_QSTR_M1PWM5),           MP_ROM_INT(PWM_M1PWM5)},
    {MP_ROM_QSTR(MP_QSTR_M1PWM6),           MP_ROM_INT(PWM_M1PWM6)},
    {MP_ROM_QSTR(MP_QSTR_M1PWM7),           MP_ROM_INT(PWM_M1PWM7)},

    {MP_ROM_QSTR(MP_QSTR_COUNT_DOWN),       MP_ROM_INT(PWM_COUNT_DOWN)},
    {MP_ROM_QSTR(MP_QSTR_COUNT_UP_DOWN),    MP_ROM_INT(PWM_COUNT_UP_DOWN)},
    {MP_ROM_QSTR(MP_QSTR_DB_FALLING),       MP_ROM_INT(PWM_DB_FALLING)},
    {MP_ROM_QSTR(MP_QSTR_DB_RISING),        MP_ROM_INT(PWM_DB_RISING)},
    {MP_ROM_QSTR(MP_QSTR_CLK_DIV_1),        MP_ROM_INT(PWM_CLK_DIV_1)},
    {MP_ROM_QSTR(MP_QSTR_CLK_DIV_2),        MP_ROM_INT(PWM_CLK_DIV_2)},
    {MP_ROM_QSTR(MP_QSTR_CLK_DIV_4),        MP_ROM_INT(PWM_CLK_DIV_4)},
    {MP_ROM_QSTR(MP_QSTR_CLK_DIV_8),        MP_ROM_INT(PWM_CLK_DIV_8)},
    {MP_ROM_QSTR(MP_QSTR_CLK_DIV_16),       MP_ROM_INT(PWM_CLK_DIV_16)},
    {MP_ROM_QSTR(MP_QSTR_CLK_DIV_32),       MP_ROM_INT(PWM_CLK_DIV_32)},
    {MP_ROM_QSTR(MP_QSTR_CLK_DIV_64),       MP_ROM_INT(PWM_CLK_DIV_64)},
};
static MP_DEFINE_CONST_DICT(machine_pwm_locals_dict, machine_pwm_locals_dict_table);

const mp_obj_type_t machine_pwm_type = 
{
    { &mp_type_type },
    .name = MP_QSTR_PWM,
    .print = machine_pwm_print,
    .make_new = mp_machine_pwm_make_new,
    .locals_dict = (mp_obj_dict_t*) &machine_pwm_locals_dict,
};
