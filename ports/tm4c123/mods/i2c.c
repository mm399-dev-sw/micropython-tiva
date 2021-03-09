#include <stdint.h>
#include <stdarg.h>
#include <stdbool.h>
#include "inc/hw_i2c.h"
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "inc/hw_gpio.h"
#include "driverlib/i2c.h"
#include "driverlib/sysctl.h"
#include "driverlib/gpio.h"
#include "driverlib/pin_map.h"
#include "i2c.h"

#include "py/runtime.h"
#include "py/obj.h"
#include "py/builtin.h"
#include "driverlib/rom.h"
#include "driverlib/rom_map.h"
#include "py/mpprint.h"

void i2c_init0() {
}

STATIC int i2c_find(mp_obj_t id) {
    if (MP_OBJ_IS_STR(id)) {
        // given a string id
        const char *port = mp_obj_str_get_str(id);
        if (0) {
            #ifdef MICROPY_HW_I2C0_NAME
        } else if (strcmp(port, MICROPY_HW_I2C0_NAME) == 0) {
            return I2C_0;
            #endif
            #ifdef MICROPY_HW_I2C1_NAME
        } else if (strcmp(port, MICROPY_HW_I2C1_NAME) == 0) {
            return I2C_1;
            #endif
            #ifdef MICROPY_HW_I2C2_NAME
        } else if (strcmp(port, MICROPY_HW_I2C2_NAME) == 0) {
            return I2C_2;
            #endif
            #ifdef MICROPY_HW_I2C3_NAME
        } else if (strcmp(port, MICROPY_HW_I2C3_NAME) == 0) {
            return I2C_3;
            #endif
        }
        nlr_raise(mp_obj_new_exception_msg_varg(&mp_type_ValueError,
            "I2C(%s) doesn't exist", port));
    } else {
        // given an integer id
        int i2c_id = mp_obj_get_int(id);
        if (i2c_id >= 0 && i2c_id <= MP_ARRAY_SIZE(MP_STATE_PORT(machine_spi_obj_all))) {
            return i2c_id;
        }
        nlr_raise(mp_obj_new_exception_msg_varg(&mp_type_ValueError,
            "I2C(%d) doesn't exist", i2c_id));
    }
}

// INITIALIZATION
// initialize I2C module 0
// Slightly modified version of TI's example code

void i2c_init(machine_hard_i2c_obj_t *self) {
    const pin_obj_t *pins[2] = {NULL, NULL};

    if (0) {
        #if defined(MICROPY_HW_I2C0_SCL)
    } else if (self->i2c_id == I2C_0) {
        self->i2c_base = I2C0_BASE;
        self->periph = SYSCTL_PERIPH_I2C0;
        self->master_regs = (periph_i2c_master_t *)I2C0_BASE;
        self->slave_regs = (periph_i2c_slave_t *)(I2C0_BASE + 0x800);
        self->status_control = (periph_i2c_stctl_t *)(I2C0_BASE + 0xFC0);
        self->irqn = INT_I2C0;
        pins[0] = MICROPY_HW_I2C0_SCL;
        #if defined(MICROPY_HW_I2C0_SDA)
        pins[1] = MICROPY_HW_I2C0_SDA;
        #endif
        #endif
        #if defined(MICROPY_HW_I2C1_SCL)
    } else if (self->i2c_id == I2C_1) {
        self->i2c_base = I2C1_BASE;
        self->periph = SYSCTL_PERIPH_I2C1;
        self->master_regs = (periph_i2c_master_t *)I2C1_BASE;
        self->slave_regs = (periph_i2c_slave_t *)(I2C1_BASE + 0x800);
        self->status_control = (periph_i2c_stctl_t *)(I2C1_BASE + 0xFC0);
        self->irqn = INT_I2C1;
        pins[0] = MICROPY_HW_I2C1_SCL;
        #if defined(MICROPY_HW_I2C1_SDA)
        pins[1] = MICROPY_HW_I2C1_SDA;
        #endif
        #endif
        #if defined(MICROPY_HW_I2C2_SCL)
    } else if (self->i2c_id == I2C_2) {
        self->i2c_base = I2C2_BASE;
        self->periph = SYSCTL_PERIPH_I2C2;
        self->master_regs = (periph_i2c_master_t *)I2C2_BASE;
        self->slave_regs = (periph_i2c_slave_t *)(I2C2_BASE + 0x800);
        self->status_control = (periph_i2c_stctl_t *)(I2C2_BASE + 0xFC0);
        self->irqn = INT_I2C2;
        pins[0] = MICROPY_HW_I2C2_SCL;
        #if defined(MICROPY_HW_I2C2_SDA)
        pins[1] = MICROPY_HW_I2C2_SDA;
        #endif
        #endif
        #if defined(MICROPY_HW_I2C3_SCL)
    } else if (self->i2c_id == I2C_3) {
        self->i2c_base = I2C3_BASE;
        self->periph = SYSCTL_PERIPH_I2C3;
        self->master_regs = (periph_i2c_master_t *)I2C3_BASE;
        self->slave_regs = (periph_i2c_slave_t *)(I2C3_BASE + 0x800);
        self->status_control = (periph_i2c_stctl_t *)(I2C3_BASE + 0xFC0);
        self->irqn = INT_I2C3;
        pins[0] = MICROPY_HW_I2C3_SCL;
        #if defined(MICROPY_HW_I2C3_SDA)
        pins[1] = MICROPY_HW_I2C3_SDA;
        #endif
        #endif
    } else {
        // I2C does not exist for this board (shouldn't get here, should be checked by caller)
        return;
    }

    // disable I2C module 0
    SysCtlPeripheralDisable(self->periph);

    // reset module
    SysCtlPeripheralReset(self->periph);

    // enable I2C module 0
    SysCtlPeripheralEnable(self->periph);

    // config of Alternative Function for Pins
    mp_hal_pin_config_alt(pins[0], PIN_FN_I2C, self->i2c_id);
    mp_hal_pin_config_alt(pins[1], PIN_FN_I2C, self->i2c_id);

    self->pin_SCL = pins[0];
    self->pin_SDA = pins[1];

    // Enable and initialize the I2C0 master module.  Use the system clock for
    // the I2C0 module.  The last parameter sets the I2C data transfer rate.
    // If false the data rate is set to 100kbps and if true the data rate will
    // be set to 400kbps.
    if (self->mode == I2C_MODE_MASTER) {
        // Init Master Module for given I2C Port with normal Clock Frequency
        I2CMasterInitExpClk(self->i2c_base, SysCtlClockGet(), (self->baudrate == 100000) ? false : true);
        if (self->dma_flag == true) {
            I2CTxFIFOConfigSet(self->i2c_base, I2C_FIFO_CFG_TX_MASTER_DMA);
            I2CRxFIFOConfigSet(self->i2c_base, I2C_FIFO_CFG_RX_MASTER_DMA);
        }
    } else if (self->mode == I2C_MODE_SLAVE) {
        // Init Slave Module for given I2C Port and Slave Address
        I2CSlaveInit(self->i2c_base, (uint8_t)self->i2c_id);
        if (self->dma_flag == true) {
            I2CTxFIFOConfigSet(self->i2c_base, I2C_FIFO_CFG_TX_SLAVE_DMA);
            I2CRxFIFOConfigSet(self->i2c_base, I2C_FIFO_CFG_RX_SLAVE_DMA);
        }
    }

    // clear I2C FIFOs
    HWREG(self->i2c_base + I2C_O_FIFOCTL) = 80008000;
}

STATIC void i2c_start_stop_condition(mp_obj_t *self_in, bool start_stop) {
    machine_hard_i2c_obj_t *self = (machine_hard_i2c_obj_t *)self_in;

    // start == false ; stop == true
    (start_stop == false) ? I2CMasterControl(self->i2c_base, I2C_MASTER_CMD_BURST_SEND_START) : I2CMasterControl(self->i2c_base, I2C_MASTER_CMD_BURST_SEND_STOP);

    if (start_stop == I2C_SEND_STOP) {
        self->err_reg = I2CMasterErr(self->i2c_base);
    }
}

STATIC void initialize_i2c_write(mp_obj_t *self_in, uint8_t device_address) {
    machine_hard_i2c_obj_t *self = (machine_hard_i2c_obj_t *)self_in;

    // specify that we want to communicate to device address with an intended write to bus
    // set bitrate to 100 kHz
    I2CMasterSlaveAddrSet(self->i2c_base, device_address, false);
}

STATIC void initialize_i2c_read(mp_obj_t *self_in, uint8_t device_address) {
    machine_hard_i2c_obj_t *self = (machine_hard_i2c_obj_t *)self_in;

    // set slave address and initiate read
    I2CMasterSlaveAddrSet(self->i2c_base, device_address, true);
}

// changed by Danny, vorher (mp_obj_t * self_in, uint8_t *i2c_data, size_t size)
STATIC uint8_t i2c_write_bytes_to_bus(mp_obj_t *self_in, uint8_t *i2c_data, size_t size) {
    uint8_t num_of_ack = 0;
    machine_hard_i2c_obj_t *self = (machine_hard_i2c_obj_t *)self_in;
    uint8_t *local_buf;
    local_buf = i2c_data;

    for (size_t i = 0; i < size; i++)
    {
        // write data to FIFO
        I2CMasterDataPut(self->i2c_base, *(local_buf++));

        // send data to the slave
        I2CMasterControl(self->i2c_base, I2C_MASTER_CMD_BURST_SEND_CONT);

        // wait for Master Control Unit to finish transaction
        while (I2CMasterBusy(self->i2c_base)) {
        }

        // check for errors
        if (I2CMasterErr(self->i2c_base) != I2C_MASTER_ERR_NONE) {
            // if error occured, cancel write routine
            I2CMasterControl(self->i2c_base, I2C_MASTER_CMD_BURST_SEND_ERROR_STOP);

            // return error code
            return num_of_ack;
        }
        num_of_ack++;
    }
    // mp_printf(MP_PYTHON_PRINTER, "bytes_to_bus base %d\n", self->i2c_base);
    return num_of_ack;
}


STATIC void i2c_timeout_set_ms_100(mp_obj_t *self_in, uint32_t timeout_val_ms) {
    // calculate timeout in ms for 100kHz rate
    // API tivaware: 0x7D -> 20ms / 0x01 -> 0,16ms / 6,25 -> 1ms / 0x06 -> 0,96ms ~ 1ms
    uint32_t timeout_val_100 = (timeout_val_ms * 6);
    machine_hard_i2c_obj_t *self = (machine_hard_i2c_obj_t *)self_in;

    I2CMasterTimeoutSet(self->i2c_base, timeout_val_100);
}

// reads number of bytes (len) from bus to address buf
STATIC void i2c_read_bytes_from_bus(mp_obj_t *self_in, uint8_t *buf, size_t len, uint8_t terminate) {
    uint8_t *buf_local;
    machine_hard_i2c_obj_t *self = (machine_hard_i2c_obj_t *)self_in;

    // initialize buffer root pointer
    buf_local = buf;

    // read bytes from bus
    for (size_t i = 0; i < len; i++)
    {
        if ((i == (len - 1)) && (terminate == 1)) {
            I2CMasterControl(self->i2c_base, I2C_MASTER_CMD_BURST_RECEIVE_FINISH);

            // wait for Master Control Unit to finish transaction
            while (I2CMasterBusy(self->i2c_base)) {
            }

            // check for errors
            if (I2CMasterErr(self->i2c_base)) {
                // if error occured, cancel write routine
                I2CMasterControl(self->i2c_base, I2C_MASTER_CMD_BURST_RECEIVE_ERROR_STOP);

                // return error code
                nlr_raise(mp_obj_new_exception_msg(&mp_type_ValueError, "data read failed\n"));
            }

            *buf_local = (uint8_t)I2CMasterDataGet(self->i2c_base);

            return;
        }
        // mp_printf(MP_PYTHON_PRINTER, "rest of buffer read reached\nsize of data buffer: %d\n", len);

        // read data from slave
        I2CMasterControl(self->i2c_base, (i == 0) ? I2C_MASTER_CMD_BURST_RECEIVE_START : I2C_MASTER_CMD_BURST_RECEIVE_CONT);

        // wait for Master Control Unit to finish transaction
        while (I2CMasterBusy(self->i2c_base)) {
        }

        // check for errors
        if (I2CMasterErr(self->i2c_base)) {
            // if error occured, cancel write routine
            I2CMasterControl(self->i2c_base, I2C_MASTER_CMD_BURST_RECEIVE_ERROR_STOP);

            // return error code
            nlr_raise(mp_obj_new_exception_msg(&mp_type_ValueError, "data read failed\n"));
        }

        // get received byte from initialized I2C port
        *buf_local++ = (uint8_t)I2CMasterDataGet(self->i2c_base);
    }
}

void i2c_deinit(const mp_obj_t *self_in) {
    machine_hard_i2c_obj_t *self = (machine_hard_i2c_obj_t *)self_in;
    I2CMasterDisable(self->i2c_base);
    SysCtlPeripheralDisable(self->periph);
}

/* --------- Binding for Micropython ------------ */
/* ---------------------------------------------- */

STATIC void machine_hard_i2c_print(const mp_print_t *print, mp_obj_t self_in, mp_print_kind_t kind) {
    machine_hard_i2c_obj_t *i2c = (machine_hard_i2c_obj_t *)self_in;

    mp_printf(print, "I2C(%u, scl=%q, sda=%q, baudrate=%d bps", i2c->i2c_id, i2c->pin_SCL->name, i2c->pin_SDA->name, i2c->baudrate);

    if (i2c->mode == I2C_MODE_MASTER) {
        mp_printf(print, ", I2C.MASTER");
    } else if (i2c->mode == I2C_MODE_SLAVE) {
        mp_printf(print, ", I2C.SLAVE");
    }

    (i2c->dma_flag == true) ? mp_printf(print, ", DMA on)") : mp_printf(print, ", DMA off)");
}

STATIC mp_obj_t machine_hard_i2c_init_helper(mp_obj_t *self_in, size_t n_args, const mp_obj_t *pos_args, mp_map_t *kw_args) {
    enum
    {
        ARG_mode,
        ARG_id,
        ARG_baudrate,
        ARG_dma
    };

    static const mp_arg_t allowed_args[] = {
        {MP_QSTR_mode, MP_ARG_REQUIRED | MP_ARG_INT, {.u_int = I2C_MODE_MASTER}}, // default: Master
        {MP_QSTR_id, MP_ARG_INT, {.u_int = I2C_0}},
        {MP_QSTR_baudrate, MP_ARG_INT, {.u_int = 400000}}, // default: 400Kbps
        {MP_QSTR_dma, MP_ARG_KW_ONLY | MP_ARG_BOOL, {.u_bool = false}}, // default: Master
    };

    mp_arg_val_t args[MP_ARRAY_SIZE(allowed_args)];
    mp_arg_parse_all(n_args, pos_args, kw_args, MP_ARRAY_SIZE(allowed_args), allowed_args, args);

    machine_hard_i2c_obj_t *self = (machine_hard_i2c_obj_t *)self_in;
    // set the I2C mode value
    if (args[ARG_mode].u_int > I2C_MODE_SLAVE || args[ARG_mode].u_int < I2C_MODE_MASTER) {
        nlr_raise(mp_obj_new_exception_msg(&mp_type_ValueError, "Mode accepts only MASTER or SLAVE"));
    }
    self->mode = (uint16_t)(args[ARG_mode].u_int & 0xFFFF);
    ;

    // set the I2C id value
    if (args[ARG_id].u_int > 3) {
        nlr_raise(mp_obj_new_exception_msg(&mp_type_ValueError, "Available I2C-Ports: 0 - 3"));
    }
    self->i2c_id = args[ARG_id].u_int;

    // set the I2C mode
    if (!((args[ARG_baudrate].u_int == 100000) || (args[ARG_baudrate].u_int == 400000))) {
        nlr_raise(mp_obj_new_exception_msg(&mp_type_ValueError, "Available baudrates: 100000 Kbps / 400000 Kbps"));
    }
    self->baudrate = args[ARG_baudrate].u_int;

    // store dma flag
    self->dma_flag = args[ARG_dma].u_bool;

    // calling I2C-Init functioni2c_init
    i2c_init(self);

    return mp_const_none;
}

STATIC void machine_hard_i2c_start(mp_obj_base_t *self_in) {
    i2c_start_stop_condition((mp_obj_t *)self_in, false);
}

STATIC void machine_hard_i2c_stop(mp_obj_base_t *self_in) {
    i2c_start_stop_condition((mp_obj_t *)self_in, true);
}

STATIC uint8_t machine_hard_i2c_write(mp_obj_base_t *self_in, uint8_t *buf, size_t size) {
    uint8_t num_of_ack = 0;
    uint8_t *plocal_buf = buf;
    machine_hard_i2c_obj_t *self = (machine_hard_i2c_obj_t *)self_in;

    initialize_i2c_write((mp_obj_t *)self_in, *(plocal_buf++));
    machine_hard_i2c_start(self_in);

    num_of_ack = i2c_write_bytes_to_bus((mp_obj_t *)self_in, plocal_buf, size - 1);

    machine_hard_i2c_stop(self_in);

    if (self->err_reg == I2C_MASTER_ERR_NONE) {
        return num_of_ack + 1;
    } else {
        ((self->err_reg & I2C_MASTER_ERR_ADDR_ACK) == I2C_MASTER_ERR_ADDR_ACK) ? nlr_raise(mp_obj_new_exception_msg(&mp_type_ValueError, "Device address not found on bus")) : nlr_raise(mp_obj_new_exception_msg(&mp_type_ValueError, "No Data ACK"));
    }
}

STATIC uint8_t machine_hard_i2c_send(mp_obj_base_t *self_in, uint8_t dev_addr, size_t size, uint8_t *buf, uint32_t timeout) {
    uint8_t num_of_ack = 0;
    uint8_t *plocal_buf = buf;
    machine_hard_i2c_obj_t *self = (machine_hard_i2c_obj_t *)self_in;

    i2c_timeout_set_ms_100((mp_obj_t *)self_in,timeout);

    initialize_i2c_write((mp_obj_t *)self_in, dev_addr);
    machine_hard_i2c_start(self_in);

    num_of_ack = i2c_write_bytes_to_bus((mp_obj_t *)self_in, plocal_buf, size);

    machine_hard_i2c_stop(self_in);

    if (self->err_reg == I2C_MASTER_ERR_NONE) {
        return num_of_ack + 1;
    } else {
        ((self->err_reg & I2C_MASTER_ERR_ADDR_ACK) == I2C_MASTER_ERR_ADDR_ACK) ? nlr_raise(mp_obj_new_exception_msg(&mp_type_ValueError, "Device address not found on bus")) : nlr_raise(mp_obj_new_exception_msg(&mp_type_ValueError, "No Data ACK"));
    }
}

STATIC void machine_hard_i2c_mem_write(mp_obj_base_t *self_in, uint8_t dev_addr, uint16_t mem_address, size_t size, uint8_t *buf, uint32_t timeout_val, uint8_t size_mem_addr) {
    uint8_t *plocal_buf = buf;
    uint8_t mem_loc_local8bit = 0x00;
    uint16_t mem_loc_local16bit = 0x0000;
    uint8_t *pmem_loc = (uint8_t *)&mem_loc_local16bit;

    // check for address size
    if (size_mem_addr == 8) {
        mem_loc_local8bit = (uint8_t)(mem_address & 0x00FF);
    } else {
        // send MSB first
        *(pmem_loc + 1) = (uint8_t)(mem_address & 0x00FF);
        *pmem_loc = (uint8_t)((mem_address >> 8) & 0x00FF);
    }

    initialize_i2c_write((mp_obj_t *)self_in, dev_addr);
    machine_hard_i2c_start(self_in);

    (size_mem_addr == 8) ? i2c_write_bytes_to_bus((mp_obj_t *)self_in, &mem_loc_local8bit, 1) : i2c_write_bytes_to_bus((mp_obj_t *)self_in, (uint8_t *)&mem_loc_local16bit, 2);

    i2c_write_bytes_to_bus((mp_obj_t *)self_in, plocal_buf, size);
    machine_hard_i2c_stop(self_in);
}

STATIC void machine_hard_i2c_readinto(mp_obj_base_t *self_in, uint8_t *buf, uint8_t buf_size, uint8_t nack) {
    uint8_t *plocal_buf = buf;

    // read bytes from bus into given buffer
    i2c_read_bytes_from_bus((mp_obj_t *)self_in, plocal_buf, buf_size, 1);
}

STATIC void machine_hard_i2c_mem_read(mp_obj_base_t *self_in, uint8_t *buf, uint8_t dev_address, uint16_t mem_address, uint8_t buf_size, uint32_t timeout, uint8_t addr_size) {
    uint8_t *local_buf = buf;
    uint8_t mem_loc_local8bit = 0x00;
    uint16_t mem_loc_local16bit = 0x0000;
    uint8_t *pmem_loc = (uint8_t *)&mem_loc_local16bit;

    // set desired timeout value in ms for 100kHz
    i2c_timeout_set_ms_100((mp_obj_t *)self_in, timeout);

    // check for address size
    if (addr_size == 8) {
        mem_loc_local8bit = (uint8_t)(mem_address & 0x00FF);
    } else {
        // send MSB first
        *(pmem_loc + 1) = (uint8_t)(mem_address & 0x00FF);
        *pmem_loc = (uint8_t)((mem_address >> 8) & 0x00FF);
    }

    // set msa reg and direction bit
    initialize_i2c_write((mp_obj_t *)self_in, dev_address);

    // start transfer
    machine_hard_i2c_start(self_in);

    // send mem location to either 8 or 16 bit mem location
    (addr_size == 8) ? i2c_write_bytes_to_bus((mp_obj_t *)self_in, &mem_loc_local8bit, 1) : i2c_write_bytes_to_bus((mp_obj_t *)self_in, (uint8_t *)&mem_loc_local16bit, 2);

    // repeated start for receive
    initialize_i2c_read((mp_obj_t *)self_in, dev_address);

    // read from bus
    i2c_read_bytes_from_bus((mp_obj_t *)self_in, local_buf, buf_size, 1);
}

STATIC mp_obj_t machine_hard_i2c_recv(mp_obj_base_t *self_in, uint8_t dev_address, uint8_t *buf, uint8_t nbytes, uint32_t timeout) {
    // utilize API functionalities
    machine_hard_i2c_obj_t *self = (machine_hard_i2c_obj_t *)self_in;
    uint8_t *local_buffer;

    local_buffer = buf;

    // set timeout for i2c communication
    i2c_timeout_set_ms_100((mp_obj_t *)self_in, timeout);

    // start i2c read routine
    initialize_i2c_read((mp_obj_t *)self_in, dev_address);
    machine_hard_i2c_start((mp_obj_base_t *)self_in);

    // read bytes from bus into buffer
    i2c_read_bytes_from_bus((mp_obj_t *)self, local_buffer, nbytes, 1);

    // terminate i2c read with stop
    machine_hard_i2c_stop(self_in);

    return mp_const_none;
}

STATIC mp_obj_t machine_hard_i2c_scan(mp_obj_base_t *self_in) {
    machine_hard_i2c_obj_t *self = (machine_hard_i2c_obj_t *)self_in;

    mp_obj_t list = mp_obj_new_list(0, NULL);

    I2CMasterDataPut(self->i2c_base, 0);

    // set timeout for i2c communication
    i2c_timeout_set_ms_100((mp_obj_t *)self_in, 5000);

    for (uint addr = 0x08; addr <= 0x77; addr++)
    {
        initialize_i2c_write((mp_obj_t *)self_in, addr);
        // send start condition, address byte and bit for write direction to slave device
        I2CMasterControl(self->i2c_base, I2C_MASTER_CMD_SINGLE_SEND);

        // wait for Master Control Unit to finish transaction
        while (I2CMasterBusy(self->i2c_base)) {
        }

        // check for errors
        if (I2CMasterErr(self->i2c_base) == I2C_MASTER_ERR_NONE) {
            // if error occured, cancel write routine
            mp_obj_list_append(list, MP_OBJ_NEW_SMALL_INT(addr));
        }
    }

    return list;
}

STATIC bool machine_hard_i2c_is_ready(mp_obj_base_t *self_in, uint8_t dev_addr) {
    machine_hard_i2c_obj_t *self = (machine_hard_i2c_obj_t *)self_in;

    I2CMasterDataPut(self->i2c_base, 0);

    initialize_i2c_write((mp_obj_t *)self_in, dev_addr);
    // send start condition, address byte and bit for write direction to slave device
    I2CMasterControl(self->i2c_base, I2C_MASTER_CMD_SINGLE_SEND);

    // wait for Master Control Unit to finish transaction
    while (I2CMasterBusy(self->i2c_base)) {
    }

    // check for errors
    if (I2CMasterErr(self->i2c_base) == I2C_MASTER_ERR_NONE) {
        // if error occured, cancel write routine
        return true;
    } else {
        return false;
    }
}

/* MP WRAPPING FUNCTIONS*/

STATIC mp_obj_t mp_machine_i2c_is_ready(mp_obj_t self, mp_obj_t dev_addr) {
    uint8_t addr = mp_obj_get_int(dev_addr);
    if (machine_hard_i2c_is_ready((mp_obj_base_t *)self, addr) == true) {
        return mp_const_true;
    } else {
        return mp_const_false;
    }
}
STATIC MP_DEFINE_CONST_FUN_OBJ_2(mp_machine_i2c_is_ready_obj, mp_machine_i2c_is_ready);


STATIC mp_obj_t mp_machine_hard_i2c_readinto(mp_obj_t self, mp_obj_t buf, mp_obj_t nack) {
    // get read buffer pointer from mp
    mp_buffer_info_t i2c_read_buf;
    mp_get_buffer_raise(buf, &i2c_read_buf, MP_BUFFER_WRITE);

    // get NACK
    uint8_t int_nack = mp_obj_get_int(nack);

    // pass to c-interface function
    machine_hard_i2c_readinto(self, (uint8_t *)i2c_read_buf.buf, i2c_read_buf.len, int_nack);

    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_3(mp_machine_hard_i2c_readinto_obj, mp_machine_hard_i2c_readinto);

STATIC mp_obj_t mp_machine_hard_i2c_write(mp_obj_t self_in, mp_obj_t wr_buf) {
    // get write buffer
    mp_buffer_info_t i2c_data;
    mp_get_buffer_raise(wr_buf, &i2c_data, MP_BUFFER_READ);

    // write bytes to bus and return number of acks
    return mp_obj_new_int((mp_int_t)machine_hard_i2c_write((mp_obj_base_t *)self_in, (uint8_t *)i2c_data.buf, i2c_data.len));
}
STATIC MP_DEFINE_CONST_FUN_OBJ_2(mp_machine_hard_i2c_write_obj, mp_machine_hard_i2c_write);

STATIC mp_obj_t mp_machine_hard_i2c_recv(size_t n_args, const mp_obj_t *args) {

    // casting Micropython data types to c data types
    uint8_t int_nbytes = mp_obj_get_int(args[1]);
    uint8_t int_dev_address = 0;
    uint32_t int_timeout_val = 5000;

    vstr_t vstr;
    vstr_init_len(&vstr, int_nbytes);

    if (n_args == 3) {
        int_dev_address = mp_obj_get_int(args[2]);
    }

    if (n_args == 4) {
        int_dev_address = mp_obj_get_int(args[2]);
        int_timeout_val = mp_obj_get_int(args[3]);

    }
    // read bytes into vstr buffer
    machine_hard_i2c_recv((mp_obj_base_t *)args[0], int_dev_address, (uint8_t *)vstr.buf, vstr.len, int_timeout_val);

    // return vstr buffer
    return mp_obj_new_str_from_vstr(&mp_type_bytes, &vstr);
}
STATIC MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mp_machine_hard_i2c_recv_obj, 2, 4, mp_machine_hard_i2c_recv);

STATIC mp_obj_t mp_machine_hard_i2c_mem_read(size_t n_args, const mp_obj_t *args) {
    uint8_t int_mem_addr_size = 8;
    uint32_t int_timeout = 5000;
    // casting Micropython data types to c data types

    // get buffer to read to
    mp_buffer_info_t i2c_data;
    mp_get_buffer_raise(args[1], &i2c_data, MP_BUFFER_READ);

    // get device address
    uint8_t int_dev_address = mp_obj_get_int(args[2]);

    // get stop
    uint16_t int_mem_addr = mp_obj_get_int(args[3]);

    if (n_args == 5) {
        int_timeout = mp_obj_get_int(args[4]);
    }

    if (n_args == 6) {
        int_timeout = mp_obj_get_int(args[4]);
        int_mem_addr_size = mp_obj_get_int(args[5]);

        if (!((int_mem_addr_size == 16) || (int_mem_addr_size == 8))) {
            nlr_raise(mp_obj_new_exception_msg(&mp_type_ValueError, "addr_size can only be 8 or 16 bits"));
        }
    }

    // pass to c-interface function
    machine_hard_i2c_mem_read(args[0], i2c_data.buf, int_dev_address, int_mem_addr, i2c_data.len, int_timeout, int_mem_addr_size);

    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mp_machine_hard_i2c_mem_read_obj, 4, 6, mp_machine_hard_i2c_mem_read);

STATIC mp_obj_t mp_machine_hard_i2c_send(size_t n_args, const mp_obj_t *args) {
    // casting Micropython data types to c data types
    uint8_t int_nbytes = mp_obj_get_int(args[1]);
    uint8_t int_dev_address = 0;
    uint32_t int_timeout_val = 5000;

    vstr_t vstr;
    vstr_init_len(&vstr, int_nbytes);

    if (n_args == 3) {
        int_dev_address = mp_obj_get_int(args[2]);
    }

    if (n_args == 4) {
        int_dev_address = mp_obj_get_int(args[2]);
        int_timeout_val = mp_obj_get_int(args[3]);
    }

    // calling I2C-Send Function
    machine_hard_i2c_send((mp_obj_base_t *)args[0], int_dev_address, vstr.len, (uint8_t *)vstr.buf, int_timeout_val);

    return mp_obj_new_str_from_vstr(&mp_type_bytes, &vstr);
    ;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mp_machine_hard_i2c_send_obj, 2, 4, mp_machine_hard_i2c_send);

/* Write Function */

STATIC mp_obj_t mp_machine_hard_i2c_mem_write(size_t n_args, const mp_obj_t *args) {
    // casting Micropython data types to c data types
    mp_buffer_info_t i2c_data;
    mp_get_buffer_raise(args[1], &i2c_data, MP_BUFFER_READ);
    uint8_t int_dev_address = mp_obj_get_int(args[2]);
    uint16_t int_mem_address = mp_obj_get_int(args[3]);
    uint32_t int_timeout = 5000;
    uint8_t int_mem_addr_size = 8;

    if (n_args == 5) {
        int_timeout = mp_obj_get_int(args[4]);
    } else if (n_args == 6) {
        int_timeout = mp_obj_get_int(args[4]);
        int_mem_addr_size = mp_obj_get_int(args[4]);

        if (!((int_mem_addr_size == 16) || (int_mem_addr_size == 8))) {
            nlr_raise(mp_obj_new_exception_msg(&mp_type_ValueError, "addr_size can only be 8 or 16 bits"));
        }
    }
    // calling I2C-Send Function
    machine_hard_i2c_mem_write(args[0], int_dev_address, int_mem_address, i2c_data.len, (uint8_t *)i2c_data.buf, int_timeout, int_mem_addr_size);

    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mp_machine_hard_i2c_mem_write_obj, 4, 6, mp_machine_hard_i2c_mem_write);

/* Scan Function */

STATIC mp_obj_t mp_machine_hard_i2c_scan(mp_obj_t self) {
    return machine_hard_i2c_scan(self);
}
STATIC MP_DEFINE_CONST_FUN_OBJ_1(mp_machine_hard_i2c_scan_obj, mp_machine_hard_i2c_scan);

/* DeInit Function */

STATIC mp_obj_t mp_machine_hard_i2c_deinit(mp_obj_t self_in) {
    i2c_deinit(&self_in);

    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_1(mp_machine_hard_i2c_deinit_obj, mp_machine_hard_i2c_deinit);

/* Init Helper Function */

STATIC mp_obj_t machine_hard_i2c_init(size_t n_args, const mp_obj_t *args, mp_map_t *kw_args) {
    return machine_hard_i2c_init_helper(args[0], n_args - 1, args + 1, kw_args);
}
STATIC MP_DEFINE_CONST_FUN_OBJ_KW(machine_i2c_init_obj, 1, machine_hard_i2c_init);

/*
        Define uPy-Fuctions
*/

/*
        Table with all globals
*/
STATIC const mp_rom_map_elem_t machine_hard_i2c_locals_dict_table[] = {
    // methods
    {MP_OBJ_NEW_QSTR(MP_QSTR___name__), MP_OBJ_NEW_QSTR(MP_QSTR_i2c)},
    {MP_OBJ_NEW_QSTR(MP_QSTR_init), MP_ROM_PTR(&machine_i2c_init_obj)},
    {MP_OBJ_NEW_QSTR(MP_QSTR_is_ready), MP_ROM_PTR(&mp_machine_i2c_is_ready_obj)},
    {MP_OBJ_NEW_QSTR(MP_QSTR_readinto), MP_ROM_PTR(&mp_machine_hard_i2c_readinto_obj)},
    {MP_OBJ_NEW_QSTR(MP_QSTR_write), MP_ROM_PTR(&mp_machine_hard_i2c_write_obj)},
    {MP_OBJ_NEW_QSTR(MP_QSTR_recv), MP_ROM_PTR(&mp_machine_hard_i2c_recv_obj)},
    {MP_OBJ_NEW_QSTR(MP_QSTR_mem_read), MP_ROM_PTR(&mp_machine_hard_i2c_mem_read_obj)},
    {MP_OBJ_NEW_QSTR(MP_QSTR_send), MP_ROM_PTR(&mp_machine_hard_i2c_send_obj)},
    {MP_OBJ_NEW_QSTR(MP_QSTR_mem_write), MP_ROM_PTR(&mp_machine_hard_i2c_mem_write_obj)},
    {MP_OBJ_NEW_QSTR(MP_QSTR_scan), MP_ROM_PTR(&mp_machine_hard_i2c_scan_obj)},
    {MP_OBJ_NEW_QSTR(MP_QSTR_deinit), MP_ROM_PTR(&mp_machine_hard_i2c_deinit_obj)},

    // constants
    {MP_ROM_QSTR(MP_QSTR_MASTER), MP_ROM_INT(I2C_MODE_MASTER)},
    {MP_ROM_QSTR(MP_QSTR_SLAVE), MP_ROM_INT(I2C_MODE_SLAVE)},
};

/*
        Define all globals as uPy-globals
*/
STATIC MP_DEFINE_CONST_DICT(machine_hard_i2c_locals_dict, machine_hard_i2c_locals_dict_table);

mp_obj_t machine_hard_i2c_make_new(const mp_obj_type_t *type, size_t n_args, size_t n_kw, const mp_obj_t *all_args) {

    // check arguments
    mp_arg_check_num(n_args, n_kw, 1, MP_OBJ_FUN_ARGS_MAX, true);

    // find I2C port
    i2c_id_t i2c_id = i2c_find(all_args[0]);

    // create dynamic peripheral object
    machine_hard_i2c_obj_t *self;

    // get I2C object
    if (MP_STATE_PORT(machine_i2c_obj_all)[i2c_id] == NULL) {
        // create new I2C object
        self = m_new0(machine_hard_i2c_obj_t, 1);
        self->base.type = &machine_hard_i2c_type;
        self->i2c_id = i2c_id;
        MP_STATE_PORT(machine_i2c_obj_all)[i2c_id] = self;
    } else {
        // reference existing I2C object
        self = MP_STATE_PORT(machine_i2c_obj_all)[i2c_id];
    }

    if (n_args > 1 || n_kw > 0) {
        // start the peripheral
        mp_map_t kw_args;
        mp_map_init_fixed_table(&kw_args, n_kw, all_args + n_args);
        mp_obj_t *self_out = (mp_obj_t *)self;
        machine_hard_i2c_init_helper(self_out, n_args - 1, all_args + 1, &kw_args);
    }

    return MP_OBJ_FROM_PTR(self);
}

const mp_obj_type_t machine_hard_i2c_type = {
    {&mp_type_type},
    .name = MP_QSTR_I2C,
    .print = machine_hard_i2c_print,
    .make_new = machine_hard_i2c_make_new,
    .locals_dict = (mp_obj_t)&machine_hard_i2c_locals_dict,
};
