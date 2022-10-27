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
#include "inc/hw_memmap.h"
#include "inc/hw_pwm.h"
#include "inc/hw_gpio.h"
#include "inc/hw_sysctl.h"
#include "pwm.h"
#include "pin.h"


enum {PWM_COUNT_DOWN, PWM_COUNT_UP_DOWN, PWM_DB_FALLING, PWM_DB_RISING};

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
    mp_obj_base_t base;
    const pin_obj_t* pin;
    uint8_t id;

    int mode;
    bool active;
    bool invert;
    uint32_t freq;
    uint8_t duty;
    mp_obj_t irq;
    uint32_t db_rising;
    uint32_t db_falling;

} machine_pwm_obj_t;

static machine_pwm_obj_t machine_pwm_obj[MICROPY_HW_MAX_PWM] = {};

uint8_t pwm_get_id_from_pin(const pin_obj_t* pin, bool alternate_pin)
{
    for (uint8_t i = 0; i < MICROPY_HW_MAX_PWM; i++)
    {
        if (pwm_pin_num_list[i][alternate_pin] == pin->pin_num)
            return i;
    }
    return 0xff;
}

void pwm_init(machine_pwm_obj_t* self, int mode, bool active, bool invert, uint32_t freq, uint8_t duty, mp_obj_t irq, uint32_t db_rising, uint32_t db_falling)
{

}

void pwm_deinit(machine_pwm_obj_t* self)
{

}

void pwm_update_active(machine_pwm_obj_t* self, bool val)
{
    self->active = val;

}

void pwm_update_invert(machine_pwm_obj_t* self, bool val)
{
    self->invert = val;

}

void pwm_update_freq(machine_pwm_obj_t* self, uint32_t val)
{
    self->freq = val;

}

void pwm_update_duty(machine_pwm_obj_t* self, uint8_t val)
{
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

// ##### internal PWM functions #####

// enable pwm module

// enable gpio module
// enable alternate functions
// assign pwm to gpio

// set pwm clock divider
// set count mode
// 


// ##### helper functions #####
mp_obj_t machine_pwm_init_helper(machine_pwm_obj_t* self, size_t n_args, const mp_obj_t* pos_args, mp_map_t* kw_args)
{
    enum {ARG_active, ARG_invert, ARG_alternate_pin, ARG_mode, ARG_freq, ARG_duty, ARG_irq, ARG_db_rising, ARG_db_falling};
    static const mp_arg_t allowed_args[] = 
    {
        {MP_QSTR_active,        MP_ARG_KW_ONLY | MP_ARG_BOOL,   {.u_int = 1}},
        {MP_QSTR_invert,        MP_ARG_KW_ONLY | MP_ARG_BOOL,   {.u_int = 0}},
        {MP_QSTR_alternate_pin, MP_ARG_KW_ONLY | MP_ARG_BOOL,   {.u_int = 0}},
        {MP_QSTR_mode,          MP_ARG_KW_ONLY | MP_ARG_INT,    {.u_int = PWM_COUNT_DOWN}},
        {MP_QSTR_freq,          MP_ARG_KW_ONLY | MP_ARG_INT,    {.u_int = PWM_COUNT_UP_DOWN}},
        {MP_QSTR_duty,          MP_ARG_KW_ONLY | MP_ARG_INT,    {.u_int = 0}},
        {MP_QSTR_irq,           MP_ARG_KW_ONLY | MP_ARG_OBJ,    {.u_obj = mp_const_none}},
        {MP_QSTR_db_rising,     MP_ARG_KW_ONLY | MP_ARG_INT,    {.u_int = 0}},
        {MP_QSTR_db_falling,    MP_ARG_KW_ONLY | MP_ARG_INT,    {.u_int = 0}},
    };
    mp_arg_val_t args[MP_ARRAY_SIZE(allowed_args)];

    // create pwm object if not exist
    if (self == NULL)
    {
        mp_arg_parse_all(n_args - 1, pos_args + 1, kw_args, MP_ARRAY_SIZE(allowed_args), allowed_args, args);
        if (!mp_obj_is_type(pos_args[0], &pin_type))
            mp_raise_ValueError(MP_ERROR_TEXT("Type mismatch: not a Pin object"));

        const pin_obj_t* pin = pin_find(pos_args[0]);
        uint8_t pwm_id = pwm_get_id_from_pin(pin, args[ARG_alternate_pin].u_int);
        if (pwm_id == 0xff)
            mp_raise_ValueError(MP_ERROR_TEXT("Pin incapable of PWM function"));

        // create pwm object
        self = &machine_pwm_obj[pwm_id];
        self->base.type = &machine_pwm_type;
        self->id = pwm_id;
        self->pin = pin;
        self->active = 0;
        self->invert = 0;
        self->mode = PWM_COUNT_DOWN;
        self->duty = 0;
        self->freq = 0;
        self->irq = mp_const_none;
        self->db_falling = 0;
        self->db_rising = 0;
    }
    else
        mp_arg_parse_all(n_args, pos_args, kw_args, MP_ARRAY_SIZE(allowed_args), allowed_args, args);


    return self;
}

mp_obj_t machine_pwm_deinit_helper(machine_pwm_obj_t* self)
{
    return mp_const_none;
}

mp_obj_t machine_pwm_active_helper(machine_pwm_obj_t* self, size_t n_args, const mp_obj_t* args)
{
    if (n_args)
    {
        if (mp_obj_is_bool(args[0]) || mp_obj_is_int(args[0]))
            pwm_update_active(self, MP_OBJ_SMALL_INT_VALUE(args[0]));
        else
            mp_raise_ValueError(MP_ERROR_TEXT("Type mismatch: not an int"));
    }
    return mp_obj_new_bool(self->active);
}

mp_obj_t machine_pwm_invert_helper(machine_pwm_obj_t* self, size_t n_args, const mp_obj_t* args)
{
    if (n_args)
    {
        if (mp_obj_is_bool(args[0]) || mp_obj_is_int(args[0]))
            pwm_update_invert(self, MP_OBJ_SMALL_INT_VALUE(args[0]));
        else
            mp_raise_ValueError(MP_ERROR_TEXT("Type mismatch: not an int"));
    }
    return mp_obj_new_bool(self->active);
}

mp_obj_t machine_pwm_freq_helper(machine_pwm_obj_t* self, size_t n_args, const mp_obj_t* args)
{
    if (n_args)
    {
        if (mp_obj_is_int(args[0]))
            pwm_update_freq(self, (uint32_t) MP_OBJ_SMALL_INT_VALUE(args[0]));
        else
            mp_raise_ValueError(MP_ERROR_TEXT("Type mismatch: not an int"));
    }
    return mp_obj_new_int(self->freq);
}

mp_obj_t machine_pwm_duty_helper(machine_pwm_obj_t* self, size_t n_args, const mp_obj_t* args)
{
    if (n_args)
    {
        if (mp_obj_is_int(args[0]))
            pwm_update_duty(self, MP_OBJ_SMALL_INT_VALUE(args[0]));
        else
            mp_raise_ValueError(MP_ERROR_TEXT("Type mismatch: not an int"));
    }
    return mp_obj_new_int(self->duty);
}

mp_obj_t machine_pwm_sync_helper(machine_pwm_obj_t* self, size_t n_args, const mp_obj_t* args)
{
    if (n_args)
    {
        for (uint8_t i = 0; i < n_args; i++)
        {
            if (!mp_obj_is_type(args[i], &machine_pwm_type))
                mp_raise_ValueError(MP_ERROR_TEXT("Type mismatch: not a PWM object"));
        }
    }
    pwm_update_sync(self, (machine_pwm_obj_t*) args, n_args);
    return self;
}

mp_obj_t machine_pwm_irq_helper(machine_pwm_obj_t* self, size_t n_args, const mp_obj_t* args)
{
    return self;
}

mp_obj_t machine_pwm_db_helper(machine_pwm_obj_t* self, size_t n_args, const mp_obj_t* args)
{
    return self;
}

mp_obj_t machine_pwm_fault_helper(machine_pwm_obj_t* self, size_t n_args, const mp_obj_t* args)
{
    return self;
}


// ##### micropython functions and declarations #####
static mp_obj_t mp_machine_pwm_make_new(const mp_obj_type_t* type, size_t n_args, size_t n_kw, const mp_obj_t* args)
{
    mp_arg_check_num(n_args, n_kw, 1, MP_OBJ_FUN_ARGS_MAX, true);
    mp_map_t kw_args;
    mp_map_init_fixed_table(&kw_args, n_kw, args + n_args);

    return machine_pwm_init_helper(NULL, n_args, args, &kw_args);
}

static mp_obj_t machine_pwm_init(size_t n_args, const mp_obj_t* args, mp_map_t* kw_args)
{
    return machine_pwm_init_helper(args[0], n_args - 1, args + 1, kw_args);
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

static mp_obj_t machine_pwm_irq(size_t n_args, const mp_obj_t* args)
{
    return machine_pwm_irq_helper(args[0], n_args - 1, args + 1);
}
static MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(machine_pwm_irq_obj, 1, 2, machine_pwm_irq);

static mp_obj_t machine_pwm_db(size_t n_args, const mp_obj_t* args, mp_map_t* kw_args)
{
    return machine_pwm_db_helper(args[0], n_args - 1, args + 1);
}
static MP_DEFINE_CONST_FUN_OBJ_KW(machine_pwm_db_obj, 1, machine_pwm_db);

static mp_obj_t machine_pwm_fault(size_t n_args, const mp_obj_t* args)
{
    return machine_pwm_fault_helper(args[0], n_args - 1, args + 1);
}
static MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(machine_pwm_fault_obj, 1, 2, machine_pwm_fault);

void pwm_init0(void)
{
    mp_obj_list_init(&MP_STATE_PORT(mp_pwm_obj_list), 0);
}

static void machine_pwm_print(const mp_print_t* print, mp_obj_t self_in, mp_print_kind_t kind)
{
    machine_pwm_obj_t* self = MP_OBJ_TO_PTR(self_in);
    mp_printf(print, "PWM(Pin(%s), id = %u, mode = %u, active = %u, invert = %u, freq = %lu, duty = %u, irq = %s, db_rising = %lu, db_falling = %lu)",
                qstr_str(self->pin->name), self->id, self->mode, self->active, self->invert, self->freq, self->duty, "None", self->db_rising, self->db_falling);
}

static const mp_rom_map_elem_t machine_pwm_locals_dict_table[] = 
{
    // class methods
    {MP_ROM_QSTR(MP_QSTR_init),             MP_ROM_PTR(&machine_pwm_init_obj)},
    {MP_ROM_QSTR(MP_QSTR_deinit),           MP_ROM_PTR(&machine_pwm_deinit_obj)},
    {MP_ROM_QSTR(MP_QSTR_active),           MP_ROM_PTR(&machine_pwm_active_obj)},
    {MP_ROM_QSTR(MP_QSTR_invert),           MP_ROM_PTR(&machine_pwm_invert_obj)},
    {MP_ROM_QSTR(MP_QSTR_freq),             MP_ROM_PTR(&machine_pwm_freq_obj)},
    {MP_ROM_QSTR(MP_QSTR_duty),             MP_ROM_PTR(&machine_pwm_duty_obj)},
    {MP_ROM_QSTR(MP_QSTR_sync),             MP_ROM_PTR(&machine_pwm_sync_obj)},
    {MP_ROM_QSTR(MP_QSTR_irq),              MP_ROM_PTR(&machine_pwm_irq_obj)},
    {MP_ROM_QSTR(MP_QSTR_db),               MP_ROM_PTR(&machine_pwm_db_obj)},
    {MP_ROM_QSTR(MP_QSTR_fault),            MP_ROM_PTR(&machine_pwm_fault_obj)},

    // class constants
    {MP_ROM_QSTR(MP_QSTR_COUNT_DOWN),       MP_ROM_INT(PWM_COUNT_DOWN)},
    {MP_ROM_QSTR(MP_QSTR_COUNT_UP_DOWN),    MP_ROM_INT(PWM_COUNT_UP_DOWN)},
    {MP_ROM_QSTR(MP_QSTR_DB_FALLING),       MP_ROM_INT(PWM_DB_FALLING)},
    {MP_ROM_QSTR(MP_QSTR_DB_RISING),        MP_ROM_INT(PWM_DB_RISING)},

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
