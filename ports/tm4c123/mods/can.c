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

#include <stdio.h>
#include <string.h>

#include "py/runtime.h"
#include "py/obj.h"
#include "py/builtin.h"
#include "py/mphal.h"
#include "py/objarray.h" 
#include "mpconfigboard.h"
#include "can.h"
#include "pin.h"
#include "mphalport.h"
#include "handlers.h"
#include "mpirq.h"

#include "inc/hw_memmap.h"
#include "driverlib/can.h"
#include "inc/hw_can.h"

#include "driverlib/sysctl.h"
#include "inc/hw_ints.h"

#define MAX_TIMEOUT_MS              (1000000) // ca. 11.5 days

// Right now only normal mode is supported
#define CAN_MODE_NORMAL             (0)

// Defaults for CAN Bit Time
#define CAN_DEFAULT_PRESCALER       (10)
#define CAN_DEFAULT_SJW             (1)
#define CAN_DEFAULT_BS1             (7)
#define CAN_DEFAULT_BS2             (3)


// empty struct used for reseting msg objs 
STATIC const tCANMsgObject EmptyMsgObjStruct;

// Holds the global pointers for the 2 CAN Modules that are needed in the CAN IRQ Handler
STATIC machine_hard_can_obj_t *glob_self[2];    


// global byte array, gets allocated in can_make_new
// used by functions which return CAN data (eg. CAN.receive())
// Reason: While in the callback function the gc is diabled an no memory can be allocated
// So in oder to use receive methods with in a callback funktion, this allocation needs 
// to happen beforehand 
mp_obj_t byte_array_8;

// copys 8 bytes into an existing byte array
mp_obj_t copy_buffer_to_bytarray(mp_obj_t self_in, void *items){
    mp_obj_array_t *o = MP_OBJ_TO_PTR(self_in);
    memcpy(o->items,items,8);
    return MP_OBJ_FROM_PTR(o);
}

// checks if the Message ID is within the valid range
STATIC void check_value_range_ID(bool extended_id, uint id){
    if(extended_id){
        if(id>0x1FFFFFFF){
            mp_raise_TypeError(MP_ERROR_TEXT("With extended ID: Max ID = 0x1FFFFFFF"));
        }
    }
    else{
        if(id>0x7FF){
            mp_raise_TypeError(MP_ERROR_TEXT("Without extended ID: Max ID = 0x7FF"));
        }
    }
}

// checks max timeout
STATIC void check_value_range_timeout(uint timeout){
    if(timeout>MAX_TIMEOUT_MS){
        mp_raise_TypeError(MP_ERROR_TEXT("Timeout too long"));
    }
}

// Works out the Port ID
STATIC int can_find(mp_obj_t id) {
    if (MP_OBJ_IS_STR(id)) {
        // given a string id
        const char *port = mp_obj_str_get_str(id);
        if (0) {
            #ifdef MICROPY_HW_CAN0_NAME
        } else if (strcmp(port, MICROPY_HW_CAN0_NAME) == 0) {
            return CAN_0;
            #endif
            #ifdef MICROPY_HW_CAN1_NAME
        } else if (strcmp(port, MICROPY_HW_CAN1_NAME) == 0) {
            return CAN_1;
            #endif
        }
        nlr_raise(mp_obj_new_exception_msg_varg(&mp_type_ValueError,
            MP_ERROR_TEXT("CAN(%s) doesn't exist"), port));
    } else {
        // given an integer id
        int can_id = mp_obj_get_int(id);
        if (can_id >= 0 && can_id < MP_ARRAY_SIZE(MP_STATE_PORT(machine_can_obj_all))) {
            return can_id;
        }
        nlr_raise(mp_obj_new_exception_msg_varg(&mp_type_ValueError,
            MP_ERROR_TEXT("CAN(%d) doesn't exist"), can_id));
    }
}

// Creates Message Objects used for data transmission by the CAN controller and returns its index
STATIC uint32_t can_manage_msg_obj(machine_hard_can_obj_t *self, uint32_t message_id,size_t data_len, uint8_t *data,bool rtr,bool irq){

    // returns the value of the MSGVAL Register. With this not configured Msg Objs can be determind (not used => 0)  
    uint32_t used_msg_objs=CANStatusGet(self->can_base,CAN_STS_MSGVAL);

    // finds unused msg obj
    uint32_t idx_msg_obj=0;
    self->can_msg_obj[idx_msg_obj].ui32Flags=0;

    // Checks LSB, if its 0 -> unused msg obj found 
    while((used_msg_objs & 0x01)){

        // all msg obj are used
        if(idx_msg_obj>=32){
            mp_raise_TypeError(MP_ERROR_TEXT("All Message Objects are used"));
            return 0;
        }
        used_msg_objs=used_msg_objs>>1;
        idx_msg_obj++;
    }
    // Message ID 
    self->can_msg_obj[idx_msg_obj].ui32MsgID = message_id;
    
    if (self->mask!=0x00){
        // Mask for Filtering
        self->can_msg_obj[idx_msg_obj].ui32MsgIDMask=self->mask;
        self->can_msg_obj[idx_msg_obj].ui32Flags|=MSG_OBJ_USE_ID_FILTER;
    }

    // Set Extendend ID Flag
    if(self->extframe){
        self->can_msg_obj[idx_msg_obj].ui32Flags|=MSG_OBJ_EXTENDED_ID;
    }

    // Sends remote Frame -> no Data is transmitted
    if (rtr){
        self->can_msg_obj[idx_msg_obj].ui32Flags|=MSG_OBJ_REMOTE_FRAME;
    }

    // enables IRQ
    if(irq){
        // right now only RX msg obj supprort IRQ
        self->can_msg_obj[idx_msg_obj].ui32Flags|=MSG_OBJ_RX_INT_ENABLE;
    }
    
    // CAN Data 
    self->can_msg_obj[idx_msg_obj].pui8MsgData=data;
    

    // Bytes of Data
    self->can_msg_obj[idx_msg_obj].ui32MsgLen=data_len;

    return idx_msg_obj;
}

// Adds a new Message Object to the obj RAM and to self->can_msg_obj/self->can_msg_obj_types
STATIC uint32_t can_add_msg_obj(machine_hard_can_obj_t *self, uint32_t message_id,size_t data_len, uint8_t *data,bool rtr,bool irq,tMsgObjType msg_type){
    // finds index of free msg_obj
    uint32_t idx_msg_obj=can_manage_msg_obj(self,message_id,data_len,data,rtr,irq);

    // Sets (and sends if tx type) a can msg
    CANMessageSet(self->can_base,idx_msg_obj+1,&self->can_msg_obj[idx_msg_obj],msg_type);
    // Safes the type of the set msg
    self->can_msg_obj_types[idx_msg_obj]=msg_type+1;
    // returns index of msg obj
    return idx_msg_obj;
}

// removes msg obj from msg RAM and from msg_obj Array
STATIC void can_remove_msg_obj(machine_hard_can_obj_t *self,uint32_t idx_msg_obj){
    // Resets msg struct
    self->can_msg_obj[idx_msg_obj]=EmptyMsgObjStruct;
    // Clears msg obj from msg RAM
    CANMessageClear(self->can_base,idx_msg_obj+1);
    // Type -> unsed
    self->can_msg_obj_types[idx_msg_obj]=0;
    // no irq
    self->call_back_fun[idx_msg_obj]=mp_const_none;
}

// returns false, when new data is received within timeout
STATIC bool can_receive_wait(machine_hard_can_obj_t *self, uint32_t timeout,uint32_t idx_msg_obj) {
    uint32_t start = mp_hal_ticks_ms();
    for (;;) {
        // checks if new data was received
        if ((CANStatusGet(self->can_base, CAN_STS_NEWDAT) & 0x01u<<idx_msg_obj) == 0x1u<<idx_msg_obj) {
            return false;
        }
        // time ran out
        if (timeout == 0 || mp_hal_ticks_ms() - start >= timeout) {
            return true; // timeout
        }
        //MICROPY_EVENT_POLL_HOOK
    }
}

// returns false, when new data is send within timeout
STATIC bool can_send_wait(machine_hard_can_obj_t *self, uint32_t timeout,uint32_t idx_msg_obj) {
    uint32_t start = mp_hal_ticks_ms();
    for (;;) {
        // checks if a message is pending
        if (!((CANStatusGet(self->can_base, CAN_STS_TXREQUEST) & 0x01u<<idx_msg_obj) == 1u<<idx_msg_obj)) {
            return false;
        }
        // time ran out
        if (timeout == 0 || mp_hal_ticks_ms() - start >= timeout) {
            return true; // timeout
        }
        //MICROPY_EVENT_POLL_HOOK
    }
}

// reads the received data, if idx_msg_obj is the index of a rx obj and the data is received within timeout
STATIC void machine_hard_can_receive(mp_obj_base_t  *self_in,uint32_t idx_msg_obj, uint8_t* rx_data,uint32_t timeout){
    machine_hard_can_obj_t *self=(machine_hard_can_obj_t *)self_in;

    // check value ranges
    check_value_range_timeout(timeout);

    // checks if given id corresponds to msg obj of type 'MSG_OBJ_TYPE_RX'
    if(self->can_msg_obj_types[idx_msg_obj]-1!=MSG_OBJ_TYPE_RX){
        mp_raise_TypeError(MP_ERROR_TEXT("Given Obj ID is not of type 'MSG_OBJ_TYPE_RX'"));
        return;
    }

    // checks if new data was/is received within timeout
    if((can_receive_wait(self,timeout,idx_msg_obj))){
        mp_printf(MICROPY_ERROR_PRINTER,"Timeout: Nothing received\n");
        return;
    }
    // changes the data ptr of the msg obj to the rx_data ptr
    self->can_msg_obj[idx_msg_obj].pui8MsgData=rx_data;
    // saves the received message in msg obj
    CANMessageGet(self->can_base,idx_msg_obj+1,&self->can_msg_obj[idx_msg_obj],true);
}

// creates msg obj, waits until timeout, removes the created obj after successful transmit
STATIC uint8_t machine_hard_can_send(mp_obj_base_t *self_in,uint32_t message_id, size_t size,uint8_t *buf,uint32_t timeout){
    machine_hard_can_obj_t *self=(machine_hard_can_obj_t *)self_in;

    // check value ranges
    check_value_range_ID(self->extframe,message_id);
    check_value_range_timeout(timeout);

    // configures TX Message object => Message gets automatically send
    uint32_t idx_msg_obj=can_add_msg_obj(self, message_id, size,buf,false,false,MSG_OBJ_TYPE_TX);

    // Error if nothing is send within timeout
    if((can_send_wait(self,timeout,idx_msg_obj))){
        // clears the created msg obj
        can_remove_msg_obj(self,idx_msg_obj);
        mp_printf(MICROPY_ERROR_PRINTER, "Timeout: Nothing sent\n");
        return 0;
    }
    // clears the created msg obj
    can_remove_msg_obj(self,idx_msg_obj);
    return message_id;
}

// sends a remoteframe and wait for an answer until timeout ran out
STATIC void machine_hard_can_request_rtr(mp_obj_base_t *self_in,uint32_t message_id, size_t size, uint8_t *data, uint32_t timeout){
    machine_hard_can_obj_t *self=(machine_hard_can_obj_t *)self_in;

    // check value ranges
    check_value_range_ID(self->extframe,message_id);
    check_value_range_timeout(timeout);

    // configures TX Remote Message object => Remoteframe gets automatically send
    uint32_t idx_msg_obj=can_add_msg_obj(self, message_id, size, data,true,false,MSG_OBJ_TYPE_TX_REMOTE);

    // Error if nothing is send within timeout
    if((can_send_wait(self,timeout,idx_msg_obj))){
        // clears the created msg obj
        can_remove_msg_obj(self,idx_msg_obj);
        mp_printf(MICROPY_ERROR_PRINTER,"Timeout: Remote request not sent\n");
        return;
    }

    // Error if nothing is received within timeout
    if((can_receive_wait(self,timeout,idx_msg_obj))){
        // clears the created msg obj
        can_remove_msg_obj(self,idx_msg_obj);
        mp_printf(MICROPY_ERROR_PRINTER,"Timeout: Requested data not received\n");
        return;
    }

    // safes the answer in a msg obj
    CANMessageGet(self->can_base,idx_msg_obj+1,&self->can_msg_obj[idx_msg_obj],true);

    // clears the created msg obj
    can_remove_msg_obj(self,idx_msg_obj);
}

// sets a msg obj, which is automatically send by the controller as an answer to a coresponding remoteframe
STATIC uint32_t machine_hard_can_set_rtr_tx(mp_obj_base_t *self_in, uint32_t message_id,size_t size, uint8_t *data){
    machine_hard_can_obj_t *self=(machine_hard_can_obj_t *)self_in;

    // check value ranges
    check_value_range_ID(self->extframe,message_id);

    // answersize max -> 8
    if(size>8){
        mp_raise_TypeError(MP_ERROR_TEXT("Remote answer can not be longer then 8 Bytes"));
    }
    // configures answer for a remote request => on receiving the remote frame the answer gets send
    uint32_t idx_msg_obj=can_add_msg_obj(self, message_id, size,data,true,false,MSG_OBJ_TYPE_RXTX_REMOTE);

    return idx_msg_obj;
}

/* ----------- Interrupt functions -------------- */
/* ---------------------------------------------- */

// enables irq for msg_obj (idx=0-31) or bus_off (idx=33)
STATIC void machine_CAN_irq_enable (mp_obj_t self_in,uint32_t idx_msg_obj) {
    machine_hard_can_obj_t *self = self_in;
    CANIntClear(self->can_base,idx_msg_obj+1);
    // Enables CAN irqs (eg. msg_obj) 
    MAP_CANIntEnable(self->can_base,CAN_INT_MASTER);
    if(idx_msg_obj==33){
        // Enables bus_off irq
        MAP_CANIntEnable(self->can_base,CAN_INT_ERROR);
    }
}

// diables CAN irqs
STATIC void machine_CAN_irq_disable (mp_obj_t self_in) {
    machine_hard_can_obj_t *self = self_in;
    MAP_CANIntDisable(self->can_base,CAN_INT_MASTER);
}

// Sets the callback funktion and registers irq handler. msg_obj irq idx=0-31, bus_off irq idx=33
STATIC void machine_CAN_callback_set(mp_obj_t self_in, uint32_t idx_msg_obj, mp_obj_t handler,uint32_t prio){
    machine_hard_can_obj_t *self= (machine_hard_can_obj_t *)self_in;

    // disable the callback first
    machine_CAN_irq_disable(self_in);

    if(idx_msg_obj<32){
        // msg_obj callback
        self->call_back_fun[idx_msg_obj]=handler;
    }else{
        // bus_off callback
        self->call_back_fun_sts=handler;
    }

    // register the interrupt and configure the priority
    IntPrioritySet(INT_CAN0_TM4C123, prio);
    if(CAN0_BASE==self->can_base){
        CANIntRegister(self->can_base, &CAN0_IRQHandler);
    }else{
        CANIntRegister(self->can_base, &CAN1_IRQHandler);
    }

    // enable the callback before returning
    machine_CAN_irq_enable(self_in,idx_msg_obj);
}

// Calls the micropython callbackfunktion
STATIC void machine_CAN_handle_callback(machine_hard_can_obj_t *self, uint idx_msg_obj, mp_obj_t callback) {
    if (callback != mp_const_none) {
        //mp_sched_lock();
        // no allocation allowed
        gc_lock();
        nlr_buf_t nlr;
        if (nlr_push(&nlr) == 0) {
            if(idx_msg_obj<32){
                // msg_obj irq (idx_msg_obj = 0-31) -> def callbackfun(c, msg_idx): ...
                mp_call_function_2(callback, MP_OBJ_FROM_PTR(self), MP_OBJ_NEW_SMALL_INT(idx_msg_obj));
            }else{
                // bus_off irq (idx_msg_obj = 33)   -> def callbackfun(c): ...
                mp_call_function_1(callback,MP_OBJ_FROM_PTR(self));
            }
            nlr_pop();
        } else {
            // Uncaught exception; disable the callback so it doesn't run again.
            machine_CAN_callback_set((mp_obj_t)self, idx_msg_obj, mp_const_none,0);
            mp_printf(MICROPY_ERROR_PRINTER, "uncaught exception in Message Object(%u) interrupt handler\n", self->can_id);
            mp_obj_print_exception(&mp_plat_print, MP_OBJ_FROM_PTR(nlr.ret_val));
        }
        gc_unlock();
        //mp_sched_unlock();
    }
}

// Sets rx obj. If handler is given also sets the callback function. Returns the idx of the created msg_obj
STATIC uint32_t machine_hard_can_set_rx(mp_obj_base_t *self_in, uint32_t message_id,mp_obj_t handler,uint32_t prio){
    machine_hard_can_obj_t *self=(machine_hard_can_obj_t *)self_in;
    uint32_t idx_msg_obj;

    // check value ranges
    check_value_range_ID(self->extframe,message_id);

    // no interrupt set
    if(handler==MP_OBJ_NULL){
        // set new RX msg_obj 
        idx_msg_obj=can_add_msg_obj(self, message_id, 8,NULL,false,false,MSG_OBJ_TYPE_RX);
    // interrupt activated
    }else{
        // check if funktion is callable
        if(!mp_obj_is_callable(handler)){
            mp_raise_TypeError(MP_ERROR_TEXT("Callback handler is not callable"));
            return 0;
        }

        // Prio value range
        if(prio>7){
            mp_raise_TypeError(MP_ERROR_TEXT("prio must be within [1,...,7]"));
        }

        idx_msg_obj=can_add_msg_obj(self, message_id, 8,NULL,false,true,MSG_OBJ_TYPE_RX);
        // Sets callback function, which is called on msg_obj_irq
        machine_CAN_callback_set((mp_obj_t) self_in,idx_msg_obj,handler,prio);
    }
    return idx_msg_obj;
}

// Returns true if the CAN Controller is in Bus off due to too many TX errors. 
// If Handler is given also sets the callback function.
STATIC bool machine_hard_can_bus_status(mp_obj_base_t *self_in, mp_obj_t handler, uint32_t prio){
    if(!(handler==MP_OBJ_NULL)){
        // check if funktion is callable
        if(!mp_obj_is_callable(handler)){
            mp_raise_TypeError(MP_ERROR_TEXT("Callback handler is not callable"));
            return 0;
        }

        // Prio value range
        if(prio>7){
            mp_raise_TypeError(MP_ERROR_TEXT("prio must be within [1,...,7]"));
        }

        // msg_idx con only be 0-31, so with idx=33 the status callback function is set 
        machine_CAN_callback_set((mp_obj_t) self_in,33,handler,prio);
    }
    machine_hard_can_obj_t *self = MP_OBJ_TO_PTR(self_in);

    // Checks if bus_off flag in the Status Reg is set
    uint32_t reg=CANStatusGet(self->can_base,CAN_STS_CONTROL);
    bool ret=false;
    if((reg&CAN_STATUS_BUS_OFF)==CAN_STATUS_BUS_OFF){
        ret=true;
    }
    return ret;
}

// IRQ Handler
void CANGenericIntHandler(uint32_t base) {
    machine_hard_can_obj_t *self=glob_self[CAN0_BASE==base? 0 : 1];
    uint32_t idx=0;

    if((CANStatusGet(base,CAN_STS_CONTROL)&CAN_STATUS_BUS_OFF)==CAN_STATUS_BUS_OFF){
        idx=33;
        // clears irq
        CANIntClear(base,CAN_INT_INTID_STATUS);
        machine_CAN_handle_callback(self,idx,self->call_back_fun_sts);
    }else{
        // returns 1-32 for the msg obj, which causes the irq
        uint32_t status = CANIntStatus(base, CAN_INT_STS_OBJECT);
        if(status==0){
            return;
        }
        // finds the idx of the irq msg obj
        for(;idx<32;idx++){
            if(((status>>idx) & 0x01) == 0x01){
                break;
            }
        }
        // clears irq
        CANIntClear(base,idx+1);
        // calls callbackfunction
        machine_CAN_handle_callback(self,idx,self->call_back_fun[idx]);
    }   
}

/* ------------ init/deinit Helper--------------- */
/* ---------------------------------------------- */

// Deinitializes the can obj
STATIC void machine_hard_can_deinit(mp_obj_t self_in){
    machine_hard_can_obj_t *self = MP_OBJ_TO_PTR(self_in);
    // removes msg objs
    for(int i=0;i<32;i++){
        can_remove_msg_obj(self,i);
    }
    machine_CAN_irq_disable(self);
    CANDisable(self->can_base);
    self->is_enabled=false;
}

// Initializes the CAN Module und it's Pins
void can_init(machine_hard_can_obj_t *self){
    const pin_obj_t *pins[2]={NULL,NULL};

    if(0){

    #if defined(MICROPY_HW_CAN0_RX)
    } else if(self->can_id==CAN_0){
    
        self->can_base=CAN0_BASE;
        self->periph=SYSCTL_PERIPH_CAN0;
        self->regs=(periph_can_t*)CAN0_BASE;
        
        pins[0]=MICROPY_HW_CAN0_RX;
        #if defined(MICROPY_HW_CAN0_TX)
        pins[1]=MICROPY_HW_CAN0_TX;
        #endif
    #endif
    // TODO Alternative: Cause these PINs are used by UART0
    #if defined(MICROPY_HW_CAN1_RX) 
    }else if(self->can_id==CAN_1){
        self->can_base=CAN1_BASE;
        self->periph=SYSCTL_PERIPH_CAN1;
        self->regs=(periph_can_t*)CAN1_BASE;
        
        pins[0]=MICROPY_HW_CAN1_RX;
        #if defined(MICROPY_HW_CAN1_TX)
        pins[1]=MICROPY_HW_CAN1_TX;
        #endif
    #endif
    } else{
         // CAN does not exist for this board (shouldn't get here, should be checked by caller)
        return;
    }
    // CONFIG PINS  
    mp_hal_pin_config_alt(pins[0], PIN_FN_CAN, self->can_id);
    mp_hal_pin_config_alt(pins[1], PIN_FN_CAN, self->can_id);

    // disable CAN module 
    SysCtlPeripheralDisable(self->periph);

    // reset module
    SysCtlPeripheralReset(self->periph);

    // enable CAN module 0
    SysCtlPeripheralEnable(self->periph);

    // Wait for the CAN0 module to be ready.
    while(!SysCtlPeripheralReady(self->periph))
    {
    }

    // Reset the state of all the message objects and the state of the CAN
    // module to a known state. 
    CANInit(self->can_base);

    // Set Bit Timing 
    if(self->baudrate==0){
        tCANBitClkParms Timing={self->bs1,self->bs2,self->sjw,self->prescaler};
        CANBitTimingSet(self->can_base,&Timing);
    }else{
        CANBitRateSet(CAN0_BASE, SysCtlClockGet(), self->baudrate);
    }
    
    // Enable CAN 
    CANEnable(self->can_base);

    self->is_enabled=true;
}

// init(extframe=False, prescaler=10, *, sjw=1, bs1=7, bs2=3,baudrate)
STATIC mp_obj_t machine_hard_can_init_helper(machine_hard_can_obj_t *self_in, size_t n_args, const mp_obj_t *pos_args, mp_map_t *kw_args) {
    enum { ARG_extframe, ARG_prescaler, ARG_sjw, ARG_bs1, ARG_bs2, ARG_auto_restart, ARG_baudrate, ARG_sample_point };
    static const mp_arg_t allowed_args[] = {
        //{ MP_QSTR_mode,         MP_ARG_REQUIRED | MP_ARG_INT,   {.u_int = CAN_MODE_NORMAL} },
        { MP_QSTR_extframe,     MP_ARG_BOOL,                    {.u_bool = false} },
        { MP_QSTR_prescaler,    MP_ARG_INT,                     {.u_int = CAN_DEFAULT_PRESCALER} },
        { MP_QSTR_sjw,          MP_ARG_KW_ONLY | MP_ARG_INT,    {.u_int = CAN_DEFAULT_SJW} },
        { MP_QSTR_bs1,          MP_ARG_KW_ONLY | MP_ARG_INT,    {.u_int = CAN_DEFAULT_BS1} },
        { MP_QSTR_bs2,          MP_ARG_KW_ONLY | MP_ARG_INT,    {.u_int = CAN_DEFAULT_BS2} },
        { MP_QSTR_auto_restart, MP_ARG_KW_ONLY | MP_ARG_BOOL,   {.u_bool = false} },
        { MP_QSTR_baudrate,     MP_ARG_KW_ONLY | MP_ARG_INT,    {.u_int = 0} },
        { MP_QSTR_sample_point, MP_ARG_KW_ONLY | MP_ARG_INT,    {.u_int = 75} }, // 75% sampling point
    };

    // parse args
    mp_arg_val_t args[MP_ARRAY_SIZE(allowed_args)];
    mp_arg_parse_all(n_args, pos_args, kw_args, MP_ARRAY_SIZE(allowed_args), allowed_args, args);

    machine_hard_can_obj_t *self = (machine_hard_can_obj_t *)self_in;

    // --Check the ranges of values-- 
    if(args[ARG_prescaler].u_int>64){
        mp_raise_TypeError(MP_ERROR_TEXT("prescaler must be within [1,...,64]"));
    }

    if(args[ARG_sjw].u_int>4){
        mp_raise_TypeError(MP_ERROR_TEXT("sjw must be within [1,...,4]"));
    }

    if(args[ARG_bs1].u_int>8||args[ARG_bs2].u_int>8){
        mp_raise_TypeError(MP_ERROR_TEXT("bs1 and bs2 must be within [1,...,8]"));
    }
    
    if(args[ARG_baudrate].u_int>1000000){
        mp_raise_TypeError(MP_ERROR_TEXT("Max Bauderate is 1000000"));
    }


    // --Saving the Settings--
    self->extframe = args[ARG_extframe].u_bool;

    if(args[ARG_baudrate].u_int==0){
        self->prescaler=args[ARG_prescaler].u_int;
        self->sjw=args[ARG_sjw].u_int;
        self->bs1=args[ARG_bs1].u_int;
        self->bs2=args[ARG_bs2].u_int;
        self->baudrate=0;
    }else{
        self->baudrate=args[ARG_baudrate].u_int;
    }

    // --Initalise the rest--
    self->mask=0x00;

    for(int i=0;i<32;i++){
        self->call_back_fun[i]=mp_const_none;
        self->can_msg_obj_types[i]=0;
        self->can_msg_obj[i]=EmptyMsgObjStruct;
    }
    
    // calling CAN-Init Function 
    can_init(self);

    return mp_const_none;
}

/* --------- Binding for Micropython ------------ */
/* ---------------------------------------------- */

// receive(msg_obj_id,*,timeout=5000)
STATIC mp_obj_t mp_machine_hard_can_receive(size_t n_args, const mp_obj_t *pos_args, mp_map_t *kw_args){
    enum { ARG_msg_obj_id, ARG_timeout};
    static const mp_arg_t allowed_args[] = {
        { MP_QSTR_obj_id,    MP_ARG_REQUIRED | MP_ARG_INT, {.u_int = 0} },
        { MP_QSTR_timeout, MP_ARG_KW_ONLY | MP_ARG_INT, {.u_int = 5000} },
    };
    // parse args
    machine_hard_can_obj_t *self = MP_OBJ_TO_PTR(pos_args[0]);
    mp_arg_val_t args[MP_ARRAY_SIZE(allowed_args)];
    mp_arg_parse_all(n_args - 1, pos_args + 1, kw_args, MP_ARRAY_SIZE(allowed_args), allowed_args, args);

    // rx_buffer
    uint8_t rx_data[8]={};
    uint8_t empty[8]={};

    // receives 8 Bytes
    machine_hard_can_receive((mp_obj_base_t *)self,args[ARG_msg_obj_id].u_int,rx_data,args[ARG_timeout].u_int);
    if(*rx_data==*empty){
        return mp_const_none;
    }
    else{
        return copy_buffer_to_bytarray(byte_array_8,rx_data);
    }
}
STATIC MP_DEFINE_CONST_FUN_OBJ_KW(mp_machine_hard_can_receive_obj, 2, mp_machine_hard_can_receive);

// send(send, id, *, timeout=1000)
STATIC mp_obj_t mp_machine_hard_can_send(size_t n_args, const mp_obj_t *pos_args, mp_map_t *kw_args) {
    enum { ARG_data, ARG_id, ARG_timeout};
    static const mp_arg_t allowed_args[] = {
        { MP_QSTR_data,    MP_ARG_REQUIRED | MP_ARG_OBJ, {.u_obj = MP_OBJ_NULL} },
        { MP_QSTR_id,      MP_ARG_REQUIRED | MP_ARG_INT, {.u_int = 0} },
        { MP_QSTR_timeout, MP_ARG_KW_ONLY | MP_ARG_INT, {.u_int = 1000} },
    };

    // parse args
    machine_hard_can_obj_t *self = MP_OBJ_TO_PTR(pos_args[0]);
    mp_arg_val_t args[MP_ARRAY_SIZE(allowed_args)];
    mp_arg_parse_all(n_args - 1, pos_args + 1, kw_args, MP_ARRAY_SIZE(allowed_args), allowed_args, args);

    // get the buffer to send from
    mp_buffer_info_t src;
    mp_get_buffer_raise(args[ARG_data].u_obj, &src, MP_BUFFER_READ);
    // loops through the buffer and sends 8 byte until its empty 
    size_t size=src.len;
    uint8_t * buf=(uint8_t *)src.buf;

    for(;size+8>8;size-=8,buf+=8){
        int length=0;
        if(size>=8){
            length=8;       // sends 8 byte
        }
        else{
            length=size;    // remainder = bytes in last transmission
        }
        
        // Send 
        machine_hard_can_send((mp_obj_base_t *)self,args[ARG_id].u_int,length,buf, args[ARG_timeout].u_int);
    }
    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_KW(mp_machine_hard_can_send_obj, 2, mp_machine_hard_can_send);

// rtr_request(id, *,timeout=1000)
STATIC mp_obj_t mp_machine_hard_can_request_rtr(size_t n_args, const mp_obj_t *pos_args, mp_map_t *kw_args){
    enum { ARG_id, ARG_timeout};
    static const mp_arg_t allowed_args[] = {
        { MP_QSTR_id,      MP_ARG_REQUIRED | MP_ARG_INT, {.u_int = 0} },
        { MP_QSTR_timeout, MP_ARG_KW_ONLY | MP_ARG_INT, {.u_int = 1000} },
    };

    // parse args
    machine_hard_can_obj_t *self = MP_OBJ_TO_PTR(pos_args[0]);
    mp_arg_val_t args[MP_ARRAY_SIZE(allowed_args)];
    mp_arg_parse_all(n_args - 1, pos_args + 1, kw_args, MP_ARRAY_SIZE(allowed_args), allowed_args, args);
    uint8_t rtr_data[8]={};
    uint8_t empty[8]={};
    machine_hard_can_request_rtr((mp_obj_base_t *)self,args[ARG_id].u_int,8,rtr_data,args[ARG_timeout].u_int);

    if(*rtr_data==*empty){
        return mp_const_none;
    }
    else{
        return copy_buffer_to_bytarray(byte_array_8,rtr_data);
    }
}
STATIC MP_DEFINE_CONST_FUN_OBJ_KW(mp_machine_hard_can_request_rtr_obj, 2, mp_machine_hard_can_request_rtr);

// rtr_set_tx(answer, id)
STATIC mp_obj_t mp_machine_hard_can_set_rtr_tx(size_t n_args, const mp_obj_t *pos_args, mp_map_t *kw_args){
    enum { ARG_data, ARG_id};
    static const mp_arg_t allowed_args[] = {
        { MP_QSTR_data,    MP_ARG_REQUIRED | MP_ARG_OBJ, {.u_obj = MP_OBJ_NULL} },
        { MP_QSTR_id,      MP_ARG_REQUIRED | MP_ARG_INT, {.u_int = 0} },
    };

    // parse args
    machine_hard_can_obj_t *self = MP_OBJ_TO_PTR(pos_args[0]);
    mp_arg_val_t args[MP_ARRAY_SIZE(allowed_args)];
    mp_arg_parse_all(n_args - 1, pos_args + 1, kw_args, MP_ARRAY_SIZE(allowed_args), allowed_args, args);

    // get the buffer to send from
    mp_buffer_info_t src;
    mp_get_buffer_raise(args[ARG_data].u_obj, &src, MP_BUFFER_READ);

    uint32_t idx_msg_obj=machine_hard_can_set_rtr_tx((mp_obj_base_t *)self,args[ARG_id].u_int,src.len,src.buf);

    return mp_obj_new_int_from_uint(idx_msg_obj);
}
STATIC MP_DEFINE_CONST_FUN_OBJ_KW(mp_machine_hard_can_set_rtr_tx_obj, 3, mp_machine_hard_can_set_rtr_tx);

// msg_obj_id=set_rx(id,*,handler=Null,prio=7)
STATIC mp_obj_t mp_machine_hard_can_set_rx(size_t n_args, const mp_obj_t *pos_args, mp_map_t *kw_args){
    enum { ARG_id, ARG_handler, ARG_prio};
    static const mp_arg_t allowed_args[] = {
        { MP_QSTR_id,      MP_ARG_REQUIRED | MP_ARG_INT, {.u_int = 0} },
        { MP_QSTR_handler, MP_ARG_KW_ONLY | MP_ARG_OBJ, {.u_obj = MP_OBJ_NULL} },
        { MP_QSTR_prio, MP_ARG_KW_ONLY | MP_ARG_OBJ, {.u_int = 7} },
    };
    // parse args
    machine_hard_can_obj_t *self = MP_OBJ_TO_PTR(pos_args[0]);
    mp_arg_val_t args[MP_ARRAY_SIZE(allowed_args)];
    mp_arg_parse_all(n_args - 1, pos_args + 1, kw_args, MP_ARRAY_SIZE(allowed_args), allowed_args, args);

    uint32_t msg_obj_idx=machine_hard_can_set_rx((mp_obj_base_t *)self,args[ARG_id].u_int,args[ARG_handler].u_obj,args[ARG_prio].u_int);

    return mp_obj_new_int_from_uint(msg_obj_idx);
}
STATIC MP_DEFINE_CONST_FUN_OBJ_KW(mp_machine_hard_can_set_rx_obj, 1, mp_machine_hard_can_set_rx);

// rtr_remove_tx(msg_obj_idx)
STATIC mp_obj_t mp_machine_hard_can_remove_obj(mp_obj_t self_in, mp_obj_t msg_obj_idx_in){
    machine_hard_can_obj_t *self = MP_OBJ_TO_PTR(self_in);

    // index of the msg obj
    uint32_t msg_obj_idx= (uint32_t) mp_obj_get_int(msg_obj_idx_in);

    // removes msg_obj from msg RAM and the msg_obj Array: self->can_msg_obj
    can_remove_msg_obj(self,msg_obj_idx);
    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_2(mp_machine_hard_can_remove_obj_obj, mp_machine_hard_can_remove_obj);

// view_msg_objs()
STATIC mp_obj_t mp_machine_hard_can_view_objs(mp_obj_t self_in){
    machine_hard_can_obj_t *self = MP_OBJ_TO_PTR(self_in);
    vstr_t vstr[6];

    // Strings for displaying the Type of the msg objs ind the obj RAM 
    vstr_init_len(&vstr[0], 5);
    memcpy(vstr[0].buf,"Unused",6);
    vstr_init_len(&vstr[1], 2);
    memcpy(vstr[1].buf,"TX",3);
    vstr_init_len(&vstr[2], 10);
    memcpy(vstr[2].buf,"TX Remote",10);
    vstr_init_len(&vstr[3], 2);
    memcpy(vstr[3].buf,"RX",3);
    vstr_init_len(&vstr[4], 9);
    memcpy(vstr[4].buf,"RX Remote",10);
    vstr_init_len(&vstr[5], 12);
    memcpy(vstr[5].buf,"RX_TX Remote",13);

    // Prints infos to all the 32 message objs 
    for(int i=0;i<32;i++){
        uint32_t idx=self->can_msg_obj_types[i];
        uint32_t ID=self->can_msg_obj[i].ui32MsgID;      
        // prints following info: "(idx_obj)| ID:(ID) | Obj Type: (Type) (irq)"
        mp_printf(MP_PYTHON_PRINTER,"(%u)| ID: (%u)  | Obj Type:",i,ID);
        mp_printf(MP_PYTHON_PRINTER,vstr[idx].buf);
        mp_printf(MP_PYTHON_PRINTER,(self->call_back_fun[i]==mp_const_none)? "\n" : " irq\n");
    }
    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_1(mp_machine_hard_can_view_objs_obj,mp_machine_hard_can_view_objs);

// set_filter(filter)
STATIC mp_obj_t mp_machine_hard_can_set_filter(mp_obj_t self_in,mp_obj_t filter){
    machine_hard_can_obj_t *self = MP_OBJ_TO_PTR(self_in);
    // saves mask 
    self->mask=(uint32_t) mp_obj_get_int(filter);
    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_2(mp_machine_hard_can_set_filter_obj, mp_machine_hard_can_set_filter);

// bus_off=status(*,handler=Null,prio=7)
STATIC mp_obj_t mp_machine_hard_can_bus_status(size_t n_args, const mp_obj_t *pos_args, mp_map_t *kw_args){
    enum {ARG_handler, ARG_prio};
    static const mp_arg_t allowed_args[] = {
        { MP_QSTR_handler, MP_ARG_KW_ONLY | MP_ARG_OBJ, {.u_obj = MP_OBJ_NULL} },
        { MP_QSTR_prio, MP_ARG_KW_ONLY | MP_ARG_OBJ, {.u_int = 7} },
    };
    // parse args
    machine_hard_can_obj_t *self = MP_OBJ_TO_PTR(pos_args[0]);
    mp_arg_val_t args[MP_ARRAY_SIZE(allowed_args)];
    mp_arg_parse_all(n_args - 1, pos_args + 1, kw_args, MP_ARRAY_SIZE(allowed_args), allowed_args, args);

    bool off = machine_hard_can_bus_status((mp_obj_base_t *)self,args[ARG_handler].u_obj,args[ARG_prio].u_int);
    return mp_obj_new_bool(off);
}
STATIC MP_DEFINE_CONST_FUN_OBJ_KW(mp_machine_hard_can_bus_status_obj, 1, mp_machine_hard_can_bus_status);

// deinit()
STATIC mp_obj_t mp_machine_hard_can_deinit(mp_obj_t self_in){
    machine_hard_can_deinit(self_in);
    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_1(mp_machine_hard_can_deinit_obj,mp_machine_hard_can_deinit);

// restart()
STATIC mp_obj_t mp_machine_hard_can_restart(mp_obj_t self_in){
    machine_hard_can_obj_t *self = MP_OBJ_TO_PTR(self_in);
    CANEnable(self->can_base);
    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_1(mp_machine_hard_can_restart_obj,mp_machine_hard_can_restart);

// CAN(CANModul, ...)
STATIC mp_obj_t machine_hard_can_make_new(const mp_obj_type_t *type, size_t n_args, size_t n_kw, const mp_obj_t *args) {
    // check arguments
    mp_arg_check_num(n_args, n_kw, 1, MP_OBJ_FUN_ARGS_MAX, true);

    // find can Port id
    mp_uint_t can_idx=can_find(args[0]);
    
    // Pins of CAN1 are already used by UART0 for REPL 
    if(can_idx==1){
        mp_raise_TypeError(MP_ERROR_TEXT("CAN1 can not be used. It uses the same PINs as UART0"));
    }

    // get can object
    if (MP_STATE_PORT(machine_can_obj_all)[can_idx] == NULL) {
        // create new can object
        glob_self[can_idx] = m_new_obj(machine_hard_can_obj_t);
        glob_self[can_idx]->base.type = &machine_hard_can_type;
        glob_self[can_idx]->can_id = can_idx;
        glob_self[can_idx]->is_enabled = false;
        MP_STATE_PORT(machine_can_obj_all)[can_idx] = glob_self[can_idx];
    } else {
        // reference existing can object
        glob_self[can_idx] = MP_STATE_PORT(machine_can_obj_all)[can_idx];
    }

    if (!glob_self[can_idx]->is_enabled || n_args > 1) {
        if (glob_self[can_idx]->is_enabled) {
            // The caller is requesting a reconfiguration of the hardware
            // this can only be done if the hardware is in init mode
            machine_hard_can_deinit((mp_obj_t)glob_self[can_idx]);
        }
    }
    
    if (n_args > 1 || n_kw > 0) {
        // start the peripheral
        mp_map_t kw_args;
        mp_map_init_fixed_table(&kw_args, n_kw, args + n_args);
        machine_hard_can_init_helper(glob_self[can_idx], n_args - 1, args + 1, &kw_args);
    }
    // Initialises global bufferarray, used for returning received data
    byte_array_8=mp_obj_new_bytearray(8,NULL);

    return MP_OBJ_FROM_PTR(glob_self[can_idx]);
}

// prints importent information about the CAN obj
STATIC void machine_hard_can_print(const mp_print_t *print, mp_obj_t self_in, mp_print_kind_t kind) {
    machine_hard_can_obj_t *self = MP_OBJ_TO_PTR(self_in);

    if (!self->is_enabled) {
        mp_printf(print, "CAN(%u) not enabled", self->can_id);
    } else {

        mp_printf(print, "CAN(%u, extframe=%q, enabled=%q\n",
            self->can_id,
            self->extframe ? MP_QSTR_True : MP_QSTR_False,
            self->is_enabled ? MP_QSTR_True : MP_QSTR_False
            );
        
        mp_printf(print,"prescaler=%u, sjw=%u, bs1=%u, bs2=%u\n",
            self->prescaler,
            self->sjw,
            self->bs1,
            self->bs2
            );

        uint32_t txErr;
        uint32_t rxErr;
        CANErrCntrGet(self->can_base,&rxErr,&txErr);
        uint32_t reg=CANStatusGet(self->can_base,CAN_STS_CONTROL);
        bool off=false;
        if((reg&CAN_STATUS_BUS_OFF)==CAN_STATUS_BUS_OFF){
            off=true;
        }
        mp_printf(print,"TXErrors=%u ,RXErrors=%u, Bus_Off=%q)\n",
        txErr,
        rxErr,
        off? MP_QSTR_True : MP_QSTR_False
        );
    }

}

STATIC const mp_rom_map_elem_t machine_hard_can_locals_dict_table[] = {

    { MP_ROM_QSTR(MP_QSTR_send),            MP_ROM_PTR(&mp_machine_hard_can_send_obj) },
    { MP_ROM_QSTR(MP_QSTR_receive),         MP_ROM_PTR(&mp_machine_hard_can_receive_obj) },
    { MP_ROM_QSTR(MP_QSTR_rtr_request),     MP_ROM_PTR(&mp_machine_hard_can_request_rtr_obj) },
    { MP_ROM_QSTR(MP_QSTR_rtr_set_tx),      MP_ROM_PTR(&mp_machine_hard_can_set_rtr_tx_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_rx),          MP_ROM_PTR(&mp_machine_hard_can_set_rx_obj) },
    { MP_ROM_QSTR(MP_QSTR_remove_obj),      MP_ROM_PTR(&mp_machine_hard_can_remove_obj_obj) },
    { MP_ROM_QSTR(MP_QSTR_view_objs),       MP_ROM_PTR(&mp_machine_hard_can_view_objs_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_filter),      MP_ROM_PTR(&mp_machine_hard_can_set_filter_obj) },
    { MP_ROM_QSTR(MP_QSTR_deinit),          MP_ROM_PTR(&mp_machine_hard_can_deinit_obj) },
    { MP_ROM_QSTR(MP_QSTR_restart),         MP_ROM_PTR(&mp_machine_hard_can_restart_obj) },
    { MP_ROM_QSTR(MP_QSTR_status),          MP_ROM_PTR(&mp_machine_hard_can_bus_status_obj) },
};

STATIC MP_DEFINE_CONST_DICT(machine_hard_can_locals_dict, machine_hard_can_locals_dict_table);

const mp_obj_type_t machine_hard_can_type = {
    { &mp_type_type },
    .name = MP_QSTR_CAN,
    .print = machine_hard_can_print,
    .make_new = machine_hard_can_make_new,
    //.protocol = &can_stream_p,
    .locals_dict = (mp_obj_dict_t *)&machine_hard_can_locals_dict,
};

// TODO Filter