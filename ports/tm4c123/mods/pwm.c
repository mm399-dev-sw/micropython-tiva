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
#include "driverlib/pin_map.h"
#include "driverlib/pwm.h"
#include "driverlib/interrupt.h"

#include "pwm.h"
#include "pin.h"

#ifndef PART_TM4C123GH6PM
    #define PART_TM4C123GH6PM
#endif

// individual pwm modules
enum {PWM_M0PWM0, PWM_M0PWM1, PWM_M0PWM2, PWM_M0PWM3, PWM_M0PWM4, PWM_M0PWM5, PWM_M0PWM6, PWM_M0PWM7,
        PWM_M1PWM0, PWM_M1PWM1, PWM_M1PWM2, PWM_M1PWM3, PWM_M1PWM4, PWM_M1PWM5, PWM_M1PWM6, PWM_M1PWM7};

static const uint32_t pwm_periph_reg_map[2] = {SYSCTL_PERIPH_PWM0, SYSCTL_PERIPH_PWM1};
static const uint32_t pwm_mod_reg_map[2] = {PWM0_BASE, PWM1_BASE};
static const uint16_t pwm_gen_reg_map[4] = {PWM_GEN_0, PWM_GEN_1, PWM_GEN_2, PWM_GEN_3};
static const uint16_t pwm_out_reg_map[8] = {PWM_OUT_0, PWM_OUT_1, PWM_OUT_2, PWM_OUT_3, PWM_OUT_4, PWM_OUT_5, PWM_OUT_6, PWM_OUT_7};
static const uint32_t gpio_af_val_map[MICROPY_HW_MAX_PWM][2] = 
{
    {GPIO_PB6_M0PWM0,   0},
    {GPIO_PB7_M0PWM1,   0},
    {GPIO_PB4_M0PWM2,   0},
    {GPIO_PB5_M0PWM3,   0},
    {GPIO_PE4_M0PWM4,   0},
    {GPIO_PE5_M0PWM5,   0},
    {GPIO_PC4_M0PWM6,   GPIO_PD0_M0PWM6},
    {GPIO_PC5_M0PWM7,   GPIO_PD1_M0PWM7},

    {GPIO_PD0_M1PWM0,   0},
    {GPIO_PD1_M1PWM1,   0},
    {GPIO_PA6_M1PWM2,   GPIO_PE4_M1PWM2},
    {GPIO_PA7_M1PWM3,   GPIO_PE5_M1PWM3},
    {GPIO_PF0_M1PWM4,   0},
    {GPIO_PF1_M1PWM5,   0},
    {GPIO_PF2_M1PWM6,   0},
    {GPIO_PF3_M1PWM7,   0},
};

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

    const pin_obj_t*    pin;            // output pin
    bool                alternate_pin;  // use alternative pwm output pin if available

    uint8_t             id;             // internal id 0..15
    uint8_t             mod_id;         // pwm module id 0..1
    uint8_t             gen_id;         // pwm generator id 0..3
    
    bool                active;         // output active
    bool                invert;         // output inverted
    uint32_t            freq;           // pwm frequency
    uint16_t            load;           // internal pwm load value
    uint8_t             duty;           // pwm duty 0..100
    mp_obj_t            irq;            // pwm interrupt service callback
    uint16_t            irq_mode;       // pwm interrupt configuration
    uint16_t            db_falling;     // deadband delay falling edge
    uint16_t            db_rising;      // deadband delay rising edge

    uint32_t            mode;           // counting mode

} machine_pwm_obj_t;

static machine_pwm_obj_t machine_pwm_obj[MICROPY_HW_MAX_PWM] = {};
static bool pwm_obj_in_use[MICROPY_HW_MAX_PWM] = {};
static uint16_t pwm_clk_div_val = PWM_SYSCLK_DIV_1;

uint8_t pwm_get_id_from_pin(const pin_obj_t* pin, bool alternate_pin)
{
    for (uint8_t i = 0; i < MICROPY_HW_MAX_PWM; i++)
    {
        if (pwm_pin_num_list[i][alternate_pin] == pin->pin_num)
            return i;
    }
    return 0xff;
}

uint8_t get_port_index_from_pin(const pin_obj_t* pin)
{
    return (uint8_t) qstr_str(pin->name)[1] - (uint8_t) 'A';
}

uint8_t get_pin_index_from_pin(const pin_obj_t* pin)
{
    return (uint8_t) qstr_str(pin->name)[2] - (uint8_t) '0';
}

void pwm_update_active(machine_pwm_obj_t* self, bool val)
{
    PWMOutputState(pwm_mod_reg_map[self->mod_id], 1 << (self->id % PWM_M1PWM0), val);
    self->active = val;
}

void pwm_update_invert(machine_pwm_obj_t* self, bool val)
{
    PWMOutputInvert(pwm_mod_reg_map[self->mod_id], 1 << (self->id % PWM_M1PWM0), val);
    self->invert = val;
}

void pwm_update_clock_divider(machine_pwm_obj_t* self, uint16_t val)
{
    if (val == PWM_SYSCLK_DIV_1 || (val >= PWM_SYSCLK_DIV_2 && val <= PWM_SYSCLK_DIV_64))
        PWMClockSet(pwm_mod_reg_map[self->mod_id], val);
    else
        mp_raise_ValueError(MP_ERROR_TEXT("Value out of range"));
    pwm_clk_div_val = val;
}

void pwm_update_freq(machine_pwm_obj_t* self, uint32_t val)
{
    uint32_t sys_freq = SysCtlClockGet();
    uint32_t pwm_freq = sys_freq;
    if (pwm_clk_div_val & PWM_SYSCLK_DIV_2)
        pwm_freq = sys_freq / (2 << (pwm_clk_div_val & PWM_SYSCLK_DIV_2));
    
    if (val * 10 > pwm_freq)
        mp_raise_ValueError(MP_ERROR_TEXT("Value out of range, must be <= sys_clk / clk_div / 10"));

    machine_pwm_obj_t* twin = &machine_pwm_obj[(self->id % 2 ? self->id - 1: self->id + 1)];
    // if twin pwm is active dont allow different frequency
    if (twin->freq && val != twin->freq)
        mp_raise_ValueError(MP_ERROR_TEXT("Frequency must be the same inside one PWM generator"));

    uint32_t load_val = pwm_freq / val;
    if (load_val > 0xffff)
        mp_raise_ValueError(MP_ERROR_TEXT("Frequency to low, consider adjusting the PWM clock divider via PWM.clock_divider(PWM.CLK_DIV_x)"));
    
    PWMGenPeriodSet(pwm_mod_reg_map[self->mod_id], pwm_gen_reg_map[self->gen_id], load_val);
    PWMSyncTimeBase(pwm_mod_reg_map[self->mod_id], 1 << self->gen_id);
    self->load = load_val;
    self->freq = val;
}

void pwm_update_duty(machine_pwm_obj_t* self, uint8_t val)
{
    if (val > 100)
        mp_raise_ValueError(MP_ERROR_TEXT("Value out of range"));
    
    uint16_t comp_val = 0;
    comp_val = val * self->load / 100;
    
    // reenable output if duty was 0%
    if (val && self->duty == 0)
        PWMOutputState(pwm_mod_reg_map[self->mod_id], 1 << (self->id % PWM_M1PWM0), 1);
    
    // set pulse width
    if (val)
        PWMPulseWidthSet(pwm_mod_reg_map[self->mod_id], pwm_out_reg_map[self->id % PWM_M1PWM0], comp_val);
    // silently disable output as left aligned PWM does not allow for 0% duty cycle
    else
        PWMOutputState(pwm_mod_reg_map[self->mod_id], 1 << (self->id % PWM_M1PWM0), 0);
    
    self->duty = val;
}

void pwm_update_sync(machine_pwm_obj_t* self, const machine_pwm_obj_t* tbs, uint8_t n_tbs)
{
    uint8_t tbs_flag = (1 << self->gen_id) << (4 * self->mod_id);
    for (uint8_t i = 0; i < n_tbs; i++)
        tbs_flag |= (1 << tbs[i].gen_id) << (4 * tbs[i].mod_id);
    if (tbs_flag & 0x0f)
        PWMSyncTimeBase(pwm_mod_reg_map[0], tbs_flag & 0x0f);
    if (tbs_flag & 0xf0)
        PWMSyncTimeBase(pwm_mod_reg_map[1], (tbs_flag >> 4) & 0x0f);
}

void pwm_update_db(machine_pwm_obj_t* self, uint16_t db_falling, uint16_t db_rising)
{
    PWMDeadBandEnable(pwm_mod_reg_map[self->mod_id], pwm_gen_reg_map[self->gen_id], db_rising, db_falling);
    self->db_falling = db_falling;
    self->db_rising = db_rising;
    // update values for twin
    machine_pwm_obj[(self->id % 2 ? self->id - 1: self->id + 1)].db_falling = db_falling;
    machine_pwm_obj[(self->id % 2 ? self->id - 1: self->id + 1)].db_rising = db_rising;
}

void pwm_update_irq(machine_pwm_obj_t* self, mp_obj_t irq, uint16_t irq_mode)
{
    // disable irq
    if (mp_obj_is_type(irq, &mp_type_NoneType))
    {

    }
    // setup irq
    else if (mp_obj_is_fun(irq))
    {

    }
    self->irq_mode = irq_mode;
}

void pwm_update_fault(machine_pwm_obj_t* self, bool val)
{

}

void pwm_set_gpio(machine_pwm_obj_t* self, bool val)
{
    // alternate pin
    if (pwm_get_id_from_pin(self->pin, 1) == self->id)
        self->alternate_pin = 1;
    // standard pin not found
    else if (pwm_get_id_from_pin(self->pin, 0) != self->id)
        mp_raise_ValueError(MP_ERROR_TEXT("Can't setup pin for PWM"));

    SysCtlPeripheralEnable(self->pin->periph);
    while (!SysCtlPeripheralReady(self->pin->periph));
    GPIOPinConfigure(gpio_af_val_map[self->id][self->alternate_pin]);
    GPIOPinTypePWM(self->pin->gpio, 1 << get_pin_index_from_pin(self->pin));
}

void pwm_init(machine_pwm_obj_t* self, const pin_obj_t* pin, bool active, bool invert, uint32_t freq, uint8_t duty, uint16_t db_falling, uint16_t db_rising, uint32_t mode, mp_obj_t irq, uint16_t irq_mode, mp_obj_t fault)
{
    SysCtlPeripheralEnable(pwm_periph_reg_map[self->mod_id]);
    while (!SysCtlPeripheralReady(pwm_periph_reg_map[self->mod_id]));

    machine_pwm_obj_t* twin = &machine_pwm_obj[(self->id % 2 ? self->id - 1: self->id + 1)];
    // if twin pwm is active dont allow different mode
    if (twin->freq && mode != twin->mode)
        mp_raise_ValueError(MP_ERROR_TEXT("Mode must be the same inside one PWM generator"));
    // configure pwm generator
    PWMGenConfigure(pwm_mod_reg_map[self->mod_id], pwm_gen_reg_map[self->gen_id], mode);
    // setup output pin
    if (pin)
    {
        self->pin = pin;
        pwm_set_gpio(self, 1);
    }

    pwm_update_freq(self, freq);
    pwm_update_duty(self, duty);
    pwm_update_invert(self, invert);
    pwm_update_db(self, db_falling, db_rising);
    pwm_update_irq(self, irq, irq_mode);
    pwm_update_fault(self, fault);
    PWMGenEnable(pwm_mod_reg_map[self->mod_id], pwm_gen_reg_map[self->gen_id]);
    pwm_update_active(self, active);
}

void pwm_deinit(machine_pwm_obj_t* self)
{
    pwm_update_active(self, 0);
    // get second pwm of generator
    uint8_t twin_id = (self->id % 2 ? self->id - 1: self->id + 1);
    // if twin pwm is active dont disable generator
    if (!pwm_obj_in_use[twin_id])
        PWMGenDisable(pwm_mod_reg_map[self->mod_id], pwm_gen_reg_map[self->gen_id]);
    if (self->pin)
        pwm_set_gpio(self, 0);
    self->freq = 0;
    pwm_obj_in_use[self->id] = false;
}


// ##### helper functions #####
mp_obj_t machine_pwm_init_helper(machine_pwm_obj_t* self, size_t n_args, const mp_obj_t* pos_args, mp_map_t* kw_args, const pin_obj_t* pin)
{
    enum {ARG_pin, ARG_active, ARG_invert, ARG_freq, ARG_duty, ARG_db_falling, ARG_db_rising, ARG_mode, ARG_irq, ARG_irq_mode, ARG_fault};
    static const mp_arg_t allowed_args[] = 
    {
        {MP_QSTR_pin,           MP_ARG_KW_ONLY | MP_ARG_OBJ,    {.u_obj = mp_const_none}},
        {MP_QSTR_active,        MP_ARG_KW_ONLY | MP_ARG_BOOL,   {.u_int = true}},
        {MP_QSTR_invert,        MP_ARG_KW_ONLY | MP_ARG_BOOL,   {.u_int = false}},
        {MP_QSTR_freq,          MP_ARG_KW_ONLY | MP_ARG_INT,    {.u_int = 0}},
        {MP_QSTR_duty,          MP_ARG_KW_ONLY | MP_ARG_INT,    {.u_int = 0}},
        {MP_QSTR_db_falling,    MP_ARG_KW_ONLY | MP_ARG_INT,    {.u_int = 0}},
        {MP_QSTR_db_rising,     MP_ARG_KW_ONLY | MP_ARG_INT,    {.u_int = 0}},
        {MP_QSTR_mode,          MP_ARG_KW_ONLY | MP_ARG_INT,    {.u_int = PWM_GEN_MODE_DOWN}},
        {MP_QSTR_irq,           MP_ARG_KW_ONLY | MP_ARG_OBJ,    {.u_obj = mp_const_none}},
        {MP_QSTR_irq_mode,      MP_ARG_KW_ONLY | MP_ARG_OBJ,    {.u_int = 0}},
        {MP_QSTR_fault,         MP_ARG_KW_ONLY | MP_ARG_OBJ,    {.u_obj = mp_const_none}},
    };
    mp_arg_val_t args[MP_ARRAY_SIZE(allowed_args)];
    mp_arg_parse_all(n_args, pos_args, kw_args, MP_ARRAY_SIZE(allowed_args), allowed_args, args);

    // get arguments
    if (!pin)               // dont overwrite if created via pin
        pin =               (args[ARG_pin].u_obj == mp_const_none ? NULL: pin_find(args[ARG_pin].u_obj));

    bool active =           args[ARG_active].u_int;
    bool invert =           args[ARG_invert].u_int;
    uint32_t freq =         args[ARG_freq].u_int;
    uint8_t duty =          args[ARG_duty].u_int;
    uint16_t db_falling =   args[ARG_db_falling].u_int;
    uint16_t db_rising =    args[ARG_db_rising].u_int;
    uint32_t mode =         args[ARG_mode].u_int;
    mp_obj_t irq =          args[ARG_irq].u_obj;
    uint16_t irq_mode =     args[ARG_irq_mode].u_int;
    mp_obj_t fault =        args[ARG_fault].u_obj;

    pwm_init(self, pin, active, invert, freq, duty, db_falling, db_rising, mode, irq, irq_mode, fault);
    
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
    return mp_obj_new_bool(self->invert);
}

mp_obj_t machine_pwm_clock_divider_helper(machine_pwm_obj_t* self, size_t n_args, const mp_obj_t* pos_args)
{
    if (n_args)
    {
        if (mp_obj_is_int(pos_args[0]))
            pwm_update_clock_divider(self, MP_OBJ_SMALL_INT_VALUE(pos_args[0]));
        else
            mp_raise_ValueError(MP_ERROR_TEXT("Type mismatch: not an int"));
    }
    return mp_obj_new_int(pwm_clk_div_val);
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
    return mp_obj_new_bool(1);
}

mp_obj_t machine_pwm_db_helper(machine_pwm_obj_t* self, size_t n_args, const mp_obj_t* pos_args, mp_map_t* kw_args)
{
    enum {ARG_db_falling, ARG_db_rising};
    static const mp_arg_t allowed_args[] = 
    {
        {MP_QSTR_db_falling,    MP_ARG_KW_ONLY | MP_ARG_INT,    {.u_int = 0}},
        {MP_QSTR_db_rising,     MP_ARG_KW_ONLY | MP_ARG_INT,    {.u_int = 0}},
    };
    mp_arg_val_t args[MP_ARRAY_SIZE(allowed_args)];
    mp_arg_parse_all(n_args, pos_args, kw_args, MP_ARRAY_SIZE(allowed_args), allowed_args, args);

    pwm_update_db(self, args[ARG_db_falling].u_int, args[ARG_db_rising].u_int);

    return self;
}

mp_obj_t machine_pwm_irq_helper(machine_pwm_obj_t* self, size_t n_args, const mp_obj_t* pos_args, mp_map_t* kw_args)
{
    enum {ARG_mode};
    static const mp_arg_t allowed_args[] = 
    {
        {MP_QSTR_mode,          MP_ARG_KW_ONLY | MP_ARG_INT,    {.u_int = 0}},
    };
    mp_arg_val_t args[MP_ARRAY_SIZE(allowed_args)];
    mp_arg_parse_all(n_args, pos_args, kw_args, MP_ARRAY_SIZE(allowed_args), allowed_args, args);

    pwm_update_irq(self, pos_args[0], args[ARG_mode].u_int);

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

    // init via pin name
    if (mp_obj_is_str(args[0]))
    {
        pin = pin_find(args[0]);
        pwm_id = pwm_get_id_from_pin(pin, 0);
        if (pwm_id == 0xff)
            mp_raise_ValueError(MP_ERROR_TEXT("Pin incapable of PWM function"));
    }
    // init via pin obj
    else if (mp_obj_is_type(args[0], &pin_type))
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
    pwm_obj_in_use[pwm_id] = true;                  // flag pwm object as in use

    self->base.type = &machine_pwm_type;
    self->id = pwm_id;
    self->mod_id = !(pwm_id < PWM_M1PWM0);
    self->gen_id = ((pwm_id % PWM_M1PWM0) - (pwm_id % 2)) / 2;

    self->pin = pin;
    self->alternate_pin = 0;
    self->active = false;
    self->invert = false;
    self->mode = 0;
    self->duty = 0;
    self->freq = 0;
    self->load = 0;
    self->db_falling = 0;
    self->db_rising = 0;
    self->irq = mp_const_none;
    self->irq_mode = 0;

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

static mp_obj_t machine_pwm_irq(size_t n_args, const mp_obj_t* args, mp_map_t* kw_args)
{
    return machine_pwm_irq_helper(args[0], n_args - 1, args + 1, kw_args);
}
static MP_DEFINE_CONST_FUN_OBJ_KW(machine_pwm_irq_obj, 1, machine_pwm_irq);

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
    {
        machine_pwm_obj[i].freq = 0;
        machine_pwm_obj[i].mode = 0;
    }
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

    {MP_ROM_QSTR(MP_QSTR_COUNT_DOWN),       MP_ROM_INT(PWM_GEN_MODE_DOWN)},
    {MP_ROM_QSTR(MP_QSTR_COUNT_UP_DOWN),    MP_ROM_INT(PWM_GEN_MODE_UP_DOWN)},
    {MP_ROM_QSTR(MP_QSTR_CLK_DIV_1),        MP_ROM_INT(PWM_SYSCLK_DIV_1)},
    {MP_ROM_QSTR(MP_QSTR_CLK_DIV_2),        MP_ROM_INT(PWM_SYSCLK_DIV_2)},
    {MP_ROM_QSTR(MP_QSTR_CLK_DIV_4),        MP_ROM_INT(PWM_SYSCLK_DIV_4)},
    {MP_ROM_QSTR(MP_QSTR_CLK_DIV_8),        MP_ROM_INT(PWM_SYSCLK_DIV_8)},
    {MP_ROM_QSTR(MP_QSTR_CLK_DIV_16),       MP_ROM_INT(PWM_SYSCLK_DIV_16)},
    {MP_ROM_QSTR(MP_QSTR_CLK_DIV_32),       MP_ROM_INT(PWM_SYSCLK_DIV_32)},
    {MP_ROM_QSTR(MP_QSTR_CLK_DIV_64),       MP_ROM_INT(PWM_SYSCLK_DIV_64)},
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
