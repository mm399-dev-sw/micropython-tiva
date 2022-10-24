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
#include "inc/hw_memmap.h"
#include "inc/hw_pwm.h"
#include "inc/hw_gpio.h"
#include "inc/hw_sysctl.h"
#include "pwm.h"
#include "pin.h"

static bool pwm_global_init_done = false;

typedef struct _machine_pwm_obj_t
{
    mp_obj_base_t base;
    const pin_obj_t* pin;
    bool active;

    int mode;
    uint32_t freq;
    uint8_t duty;

} machine_pwm_obj_t;

static machine_pwm_obj_t machine_pwm_obj_array[2][4] = {};

void pwm_global_init(void)
{
    pwm_global_init_done = true;
}

void pwm_global_deinit(void)
{
    pwm_global_init_done = false;
}

void pwm_update(machine_pwm_obj_t* self)
{

}

// enable pwm module

// enable gpio module
// enable alternate functions
// assign pwm to gpio

// set pwm clock divider
// set count mode
// 

// ##### micropython related stuff #####
static mp_obj_t machine_pwm_init_helper(machine_pwm_obj_t* self, size_t n_args, const mp_obj_t* args, mp_map_t* kw_args)
{
    return MP_OBJ_FROM_PTR(self);
}

static mp_obj_t machine_pwm_deinit_helper(machine_pwm_obj_t* self)
{
    return mp_const_none;
}

// stand alone PWM.init() function
static mp_obj_t machine_pwm_init(size_t n_args, const mp_obj_t* args, mp_map_t* kw_args)
{
    return machine_pwm_init_helper(args[0], n_args - 1, args + 1, kw_args);
}

static mp_obj_t machine_pwm_deinit(mp_obj_t self)
{
    return machine_pwm_deinit_helper((machine_pwm_obj_t*) self);
}


static MP_DEFINE_CONST_FUN_OBJ_KW(machine_pwm_init_obj, 1, machine_pwm_init);
static MP_DEFINE_CONST_FUN_OBJ_1(machine_pwm_deinit_obj, machine_pwm_deinit);
// static MP_DEFINE_CONST_FUN_OBJ_KW(machine_pwm_freq_obj, 1, machine_pwm_freq);
// static MP_DEFINE_CONST_FUN_OBJ_KW(machine_pwm_duty_obj, 1, machine_pwm_duty);
// static MP_DEFINE_CONST_FUN_OBJ_KW(machine_pwm_sync_obj, 1, machine_pwm_sync);
// static MP_DEFINE_CONST_FUN_OBJ_KW(machine_pwm_irq_obj, 1, machine_pwm_irq);

static mp_obj_t mp_machine_pwm_make_new(const mp_obj_type_t* type, size_t n_args, size_t n_kw, const mp_obj_t* args)
{
    mp_arg_check_num(n_args, n_kw, 1, 2, true);
    const pin_obj_t* pin = pin_find(args[0]);

    // create pwm object
    machine_pwm_obj_t* self = &machine_pwm_obj_array[pin->gpio][pin->pin_num];
    self->pin = pin;
    self->active = 0;
    self->mode = -1;
    self->duty = 0;
    self->freq = 0;

    if (!pwm_global_init_done)
        pwm_global_init();

    mp_map_t kw_args;
    mp_map_init_fixed_table(&kw_args, n_kw, args + n_args);

    return machine_pwm_init_helper(self, n_args - 1, args + 1, &kw_args);
}

static const mp_rom_map_elem_t machine_pwm_locals_dict_table[] = 
{
    // class functions
    {MP_ROM_QSTR(MP_QSTR_init),             MP_ROM_PTR(&machine_pwm_init_obj)},
    {MP_ROM_QSTR(MP_QSTR_deinit),           MP_ROM_PTR(&machine_pwm_deinit_obj)},
    // {MP_ROM_QSTR(MP_QSTR_freq),             MP_ROM_PTR(&machine_pwm_freq_obj)},
    // {MP_ROM_QSTR(MP_QSTR_duty),             MP_ROM_PTR(&machine_pwm_duty_obj)},
    // {MP_ROM_QSTR(MP_QSTR_sync),             MP_ROM_PTR(&machine_pwm_sync_obj)},
    // {MP_ROM_QSTR(MP_QSTR_irq),              MP_ROM_PTR(&machine_pwm_irq_obj)},

    // class constants
    {MP_ROM_QSTR(MP_QSTR_COUNT_UP),         MP_ROM_INT(0)},
    {MP_ROM_QSTR(MP_QSTR_COUNT_DOWN),       MP_ROM_INT(1)},

};
static MP_DEFINE_CONST_DICT(machine_pwm_locals_dict, machine_pwm_locals_dict_table);

const mp_obj_type_t machine_pwm_type = 
{
    {&mp_type_type},
    .name = MP_QSTR_PWM,
    .make_new = mp_machine_pwm_make_new,
    .locals_dict = (mp_obj_t) &machine_pwm_locals_dict,
};