/*
 * This file is part of the MicroPython project, http://micropython.org/
 *
 * The MIT License (MIT)
 *
 * Copyright (c) 2013, 2014 Damien P. George
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

#include <string.h>

#include "py/runtime.h"
#include "py/mphal.h"
#include "lib/oofatfs/ff.h"
#include "extmod/vfs_fat.h"
#include "mperrno.h"

#include "driverlib/ssi.h"
#include "driverlib/rom_map.h"

#include "sd_spi_driver.h"
#include "sdcard.h"
#include "pin.h"
#include "bufhelper.h"
#include "dma.h"
#include "irq.h"

#if MICROPY_HW_HAS_SDCARD

#define MICROPY_HW_SDCARD_DETECT_PRESENT (1)



void sdcard_init(void) {
    disk_initialize();
}

// void HAL_SD_MspDeInit(SD_HandleTypeDef *hsd) {
//     HAL_NVIC_DisableIRQ(SDMMC_IRQn);
//     SDMMC_CLK_DISABLE();
// }

bool sdcard_is_present(void) {
    return mp_hal_pin_read(MICROPY_HW_SDCARD_DETECT_PIN) == MICROPY_HW_SDCARD_DETECT_PRESENT;
}

bool sdcard_power_on(void) {
    if (!sdcard_is_present()) {
        return false;
    }
    power_on();

    if (sdc) {
        sdcard_power_off();
        goto error;
    }

    return true;

error:

    return false;
}

void sdcard_power_off(void) {
    power_off();
}

void sdcard_read_csd(uint8_t *data) {
    send_cmd(CMD9, 0);
    rcvr_datablock(data, 16);
}

uint64_t sdcard_get_capacity_in_bytes(void) {
    uint8_t csd[16];
    sdcard_read_csd(csd);
    if(csd[127] & 0b11000000) {
        // SDHC XC
        uint32_t c_size = uint32_t(csd[6]) + (uint32_t(csd[7]) << 8) + ((uint32_t(csd[8]) & 0b00111111) << 16);
        return uint64_t((c_size + 1) * 512000);
    } else {
        // SD
        uint16_t c_size = ((csd[7] & 0b11000000) >> 6) +  (uint16_t(csd[8] & 0b11111111) << 2) + (uint16_t(csd[9] & 0b00000011) << 10);
        uint16_t c_size_multi = (csd[5] & 0b00000001) + ((csd[6] & 0b00000011) << 1);
        uint16_t bl_len = (csd[10] & 0b00001111);
        bl_len = (1 << bl_len);
        c_size_multi = (1 << (c_size_multi+2));
        return uint64_t((c_size + 1) * c_size_multi * bl_len);
    }
    return 0;
}


void SD_IRQHandler(void) {
    IRQ_ENTER(INT_SSI2);
    // HAL_SD_IRQHandler(&sd_handle);
    IRQ_EXIT(INT_SSI2);
}


STATIC errno_t sdcard_wait_finished(uint32_t timeout) {
    // Wait for HAL driver to be ready (eg for DMA to finish)
    uint32_t start = HAL_GetTick();
    for (;;) {
        // Do an atomic check of the state; WFI will exit even if IRQs are disabled
        uint32_t irq_state = disable_irq();
        if (!MAP_SSIBusy(SDC_GPIO_PORT_BASE)) {
            enable_irq(irq_state);
            break;
        }
        __WFI();
        enable_irq(irq_state);
        if (HAL_GetTick() - start >= timeout) {
            return MP_ETIMEDOUT;
        }
    }

    // Wait for SD card to complete the operation
    // for (;;) {
    //     HAL_SD_CardStateTypedef state = HAL_SD_GetCardState(sd);
    //     if (state == HAL_SD_CARD_TRANSFER) {
    //         return HAL_OK;
    //     }
    //     if (!(state == HAL_SD_CARD_SENDING || state == HAL_SD_CARD_RECEIVING || state == HAL_SD_CARD_PROGRAMMING)) {
    //         return HAL_ERROR;
    //     }
    //     if (HAL_GetTick() - start >= timeout) {
    //         return HAL_TIMEOUT;
    //     }
    //     __WFI();
    // }
    return RES_OK;
}

mp_uint_t sdcard_read_blocks(uint8_t *dest, uint32_t block_num, uint32_t num_blocks) {
    // check that SD card is initialised
    if (sd_handle.Instance == NULL) {
        return HAL_ERROR;
    }

    DRESULT err = RES_OK;

    // check that dest pointer is aligned on a 4-byte boundary
    uint8_t *orig_dest = NULL;
    uint32_t saved_word;
    if (((uint32_t)dest & 3) != 0) {
        // Pointer is not aligned so it needs fixing.
        // We could allocate a temporary block of RAM (as sdcard_write_blocks
        // does) but instead we are going to use the dest buffer inplace.  We
        // are going to align the pointer, save the initial word at the aligned
        // location, read into the aligned memory, move the memory back to the
        // unaligned location, then restore the initial bytes at the aligned
        // location.  We should have no trouble doing this as those initial
        // bytes at the aligned location should be able to be changed for the
        // duration of this function call.
        orig_dest = dest;
        dest = (uint8_t*)((uint32_t)dest & ~3);
        saved_word = *(uint32_t*)dest;
    }

    if (query_irq() == IRQ_STATE_ENABLED) {
        // we must disable USB irqs to prevent MSC contention with SD card
        uint32_t basepri = raise_irq_pri(IRQ_PRI_OTG_FS);

        #if SDIO_USE_GPDMA
        dma_init(&sd_rx_dma, &SDMMC_RX_DMA, &sd_handle);
        sd_handle.hdmarx = &sd_rx_dma;
        #endif

        // make sure cache is flushed and invalidated so when DMA updates the RAM
        // from reading the peripheral the CPU then reads the new data
        MP_HAL_CLEANINVALIDATE_DCACHE(dest, num_blocks * SDCARD_BLOCK_SIZE);

        err = disk_read_dma(dest, block_num, num_blocks);
        if (err == RES_OK) {
            err = sdcard_wait_finished(60000);
        }

        #if SDIO_USE_GPDMA
        dma_deinit(&SDMMC_RX_DMA);
        sd_handle.hdmarx = NULL;
        #endif

        restore_irq_pri(basepri);
    } else {
        err = disk_read(0, dest, block_num, num_blocks);
        if (err == RES_OK) {
            err = sdcard_wait_finished(60000);
        }
    }

    if (orig_dest != NULL) {
        // move the read data to the non-aligned position, and restore the initial bytes
        memmove(orig_dest, dest, num_blocks * SDCARD_BLOCK_SIZE);
        memcpy(dest, &saved_word, orig_dest - dest);
    }

    return err;
}

mp_uint_t sdcard_write_blocks(const uint8_t *src, uint32_t block_num, uint32_t num_blocks) {
    // check that SD card is initialised
    if (sd_handle.Instance == NULL) {
        return HAL_ERROR;
    }

    DRESULT err = RES_OK;

    // check that src pointer is aligned on a 4-byte boundary
    if (((uint32_t)src & 3) != 0) {
        // pointer is not aligned, so allocate a temporary block to do the write
        uint8_t *src_aligned = m_new_maybe(uint8_t, SDCARD_BLOCK_SIZE);
        if (src_aligned == NULL) {
            return HAL_ERROR;
        }
        for (size_t i = 0; i < num_blocks; ++i) {
            memcpy(src_aligned, src + i * SDCARD_BLOCK_SIZE, SDCARD_BLOCK_SIZE);
            err = sdcard_write_blocks(src_aligned, block_num + i, 1);
            if (err != RES_OK) {
                break;
            }
        }
        m_del(uint8_t, src_aligned, SDCARD_BLOCK_SIZE);
        return err;
    }

    if (query_irq() == IRQ_STATE_ENABLED) {
        // we must disable USB irqs to prevent MSC contention with SD card
        uint32_t basepri = raise_irq_pri(IRQ_PRI_OTG_FS);

        #if SDIO_USE_GPDMA
        dma_init(&sd_tx_dma, &SDMMC_TX_DMA, &sd_handle);
        sd_handle.hdmatx = &sd_tx_dma;
        #endif

        // make sure cache is flushed to RAM so the DMA can read the correct data
        MP_HAL_CLEAN_DCACHE(src, num_blocks * SDCARD_BLOCK_SIZE);

        err = disk_write_dma(0,(BYTE*)src, block_num, num_blocks);
        if (err == HAL_OK) {
            err = sdcard_wait_finished(60000);
        }

        #if SDIO_USE_GPDMA
        dma_deinit(&SDMMC_TX_DMA);
        sd_handle.hdmatx = NULL;
        #endif

        restore_irq_pri(basepri);
    } else {
        err = disk_write(0,(BYTE*)src, block_num, num_blocks);
        if (err == RES_OK) {
            err = sdcard_wait_finished(&60000);
        }
    }

    return err;
}

/******************************************************************************/
// MicroPython bindings
//
// Expose the SD card as an object with the block protocol.

// there is a singleton SDCard object
const mp_obj_base_t pyb_sdcard_obj = {&pyb_sdcard_type};

STATIC mp_obj_t pyb_sdcard_make_new(const mp_obj_type_t *type, size_t n_args, size_t n_kw, const mp_obj_t *args) {
    // check arguments
    mp_arg_check_num(n_args, n_kw, 0, 0, false);

    // return singleton object
    return (mp_obj_t)&pyb_sdcard_obj;
}

STATIC mp_obj_t sd_present(mp_obj_t self) {
    return mp_obj_new_bool(sdcard_is_present());
}
STATIC MP_DEFINE_CONST_FUN_OBJ_1(sd_present_obj, sd_present);

STATIC mp_obj_t sd_power(mp_obj_t self, mp_obj_t state) {
    bool result;
    if (mp_obj_is_true(state)) {
        result = sdcard_power_on();
    } else {
        sdcard_power_off();
        result = true;
    }
    return mp_obj_new_bool(result);
}
STATIC MP_DEFINE_CONST_FUN_OBJ_2(sd_power_obj, sd_power);

STATIC mp_obj_t sd_info(mp_obj_t self) {
    uint8_t csd[16];
    sdcard_read_csd(csd);
    uint64_t size = 0;
    uint bl_size = 0;
    int card_type = (csd[127] & 0b11000000) >> 6; // 0 == v1 SD, 1 == v2 SDHC/XC
    if(card_type) {
        // SDHC XC
        uint32_t c_size = (uint32_t)(csd[6]) + ((uint32_t)(csd[7]) << 8) + (((uint32_t)(csd[8]) & 0b00111111) << 16);
        size = (uint64_t)((c_size + 1) * 512000);
    } else {
        // SD
        uint16_t c_size = ((csd[7] & 0b11000000) >> 6) +  ((uint16_t)(csd[8] & 0b11111111) << 2) + ((uint16_t)(csd[9] & 0b00000011) << 10);
        uint16_t c_size_multi = (csd[5] & 0b00000001) + ((csd[6] & 0b00000011) << 1);
        uint16_t bl_len = (csd[10] & 0b00001111);
        bl_len = (1 << bl_len);
        bl_size = bl_len;
        c_size_multi = (1 << (c_size_multi+2));
        size = (uint64_t)((c_size + 1) * c_size_multi * bl_len);
    }
    mp_obj_t tuple[3] = {
        mp_obj_new_int_from_ull(size),
        mp_obj_new_int_from_uint(bl_size),
        mp_obj_new_int(card_type),
    };
    return mp_obj_new_tuple(3, tuple);
}
STATIC MP_DEFINE_CONST_FUN_OBJ_1(sd_info_obj, sd_info);

// now obsolete, kept for backwards compatibility
STATIC mp_obj_t sd_read(mp_obj_t self, mp_obj_t block_num) {
    uint8_t *dest = m_new(uint8_t, SDCARD_BLOCK_SIZE);
    mp_uint_t ret = sdcard_read_blocks(dest, mp_obj_get_int(block_num), 1);

    if (ret != 0) {
        m_del(uint8_t, dest, SDCARD_BLOCK_SIZE);
        nlr_raise(mp_obj_new_exception_msg_varg(&mp_type_Exception, "sdcard_read_blocks failed [%u]", ret));
    }

    return mp_obj_new_bytearray_by_ref(SDCARD_BLOCK_SIZE, dest);
}
STATIC MP_DEFINE_CONST_FUN_OBJ_2(sd_read_obj, sd_read);

// now obsolete, kept for backwards compatibility
STATIC mp_obj_t sd_write(mp_obj_t self, mp_obj_t block_num, mp_obj_t data) {
    mp_buffer_info_t bufinfo;
    mp_get_buffer_raise(data, &bufinfo, MP_BUFFER_READ);
    if (bufinfo.len % SDCARD_BLOCK_SIZE != 0) {
        nlr_raise(mp_obj_new_exception_msg_varg(&mp_type_ValueError, "writes must be a multiple of %d bytes", SDCARD_BLOCK_SIZE));
    }

    mp_uint_t ret = sdcard_write_blocks(bufinfo.buf, mp_obj_get_int(block_num), bufinfo.len / SDCARD_BLOCK_SIZE);

    if (ret != 0) {
        nlr_raise(mp_obj_new_exception_msg_varg(&mp_type_Exception, "sdcard_write_blocks failed [%u]", ret));
    }

    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_3(sd_write_obj, sd_write);

STATIC mp_obj_t pyb_sdcard_readblocks(mp_obj_t self, mp_obj_t block_num, mp_obj_t buf) {
    mp_buffer_info_t bufinfo;
    mp_get_buffer_raise(buf, &bufinfo, MP_BUFFER_WRITE);
    mp_uint_t ret = sdcard_read_blocks(bufinfo.buf, mp_obj_get_int(block_num), bufinfo.len / SDCARD_BLOCK_SIZE);
    return mp_obj_new_bool(ret == 0);
}
STATIC MP_DEFINE_CONST_FUN_OBJ_3(pyb_sdcard_readblocks_obj, pyb_sdcard_readblocks);

STATIC mp_obj_t pyb_sdcard_writeblocks(mp_obj_t self, mp_obj_t block_num, mp_obj_t buf) {
    mp_buffer_info_t bufinfo;
    mp_get_buffer_raise(buf, &bufinfo, MP_BUFFER_READ);
    mp_uint_t ret = sdcard_write_blocks(bufinfo.buf, mp_obj_get_int(block_num), bufinfo.len / SDCARD_BLOCK_SIZE);
    return mp_obj_new_bool(ret == 0);
}
STATIC MP_DEFINE_CONST_FUN_OBJ_3(pyb_sdcard_writeblocks_obj, pyb_sdcard_writeblocks);

STATIC mp_obj_t pyb_sdcard_ioctl(mp_obj_t self, mp_obj_t cmd_in, mp_obj_t arg_in) {
    mp_int_t cmd = mp_obj_get_int(cmd_in);
    switch (cmd) {
        case BP_IOCTL_INIT:
            if (!disk_initialize(0)) {
                return MP_OBJ_NEW_SMALL_INT(-1); // error
            }
            return MP_OBJ_NEW_SMALL_INT(0); // success

        case BP_IOCTL_DEINIT:
            sdcard_power_off();
            return MP_OBJ_NEW_SMALL_INT(0); // success

        case BP_IOCTL_SYNC:
            // nothing to do
            return MP_OBJ_NEW_SMALL_INT(0); // success

        case BP_IOCTL_SEC_COUNT:
            return MP_OBJ_NEW_SMALL_INT(sdcard_get_capacity_in_bytes() / SDCARD_BLOCK_SIZE);

        case BP_IOCTL_SEC_SIZE:
            return MP_OBJ_NEW_SMALL_INT(SDCARD_BLOCK_SIZE);

        default: // unknown command
            return MP_OBJ_NEW_SMALL_INT(-1); // error
    }
}
STATIC MP_DEFINE_CONST_FUN_OBJ_3(pyb_sdcard_ioctl_obj, pyb_sdcard_ioctl);

STATIC const mp_rom_map_elem_t pyb_sdcard_locals_dict_table[] = {
    { MP_ROM_QSTR(MP_QSTR_present), MP_ROM_PTR(&sd_present_obj) },
    { MP_ROM_QSTR(MP_QSTR_power), MP_ROM_PTR(&sd_power_obj) },
    { MP_ROM_QSTR(MP_QSTR_info), MP_ROM_PTR(&sd_info_obj) },
    { MP_ROM_QSTR(MP_QSTR_read), MP_ROM_PTR(&sd_read_obj) },
    { MP_ROM_QSTR(MP_QSTR_write), MP_ROM_PTR(&sd_write_obj) },
    // block device protocol
    { MP_ROM_QSTR(MP_QSTR_readblocks), MP_ROM_PTR(&pyb_sdcard_readblocks_obj) },
    { MP_ROM_QSTR(MP_QSTR_writeblocks), MP_ROM_PTR(&pyb_sdcard_writeblocks_obj) },
    { MP_ROM_QSTR(MP_QSTR_ioctl), MP_ROM_PTR(&pyb_sdcard_ioctl_obj) },
};

STATIC MP_DEFINE_CONST_DICT(pyb_sdcard_locals_dict, pyb_sdcard_locals_dict_table);

const mp_obj_type_t pyb_sdcard_type = {
    { &mp_type_type },
    .name = MP_QSTR_SDCard,
    .make_new = pyb_sdcard_make_new,
    .locals_dict = (mp_obj_dict_t*)&pyb_sdcard_locals_dict,
};

void sdcard_init_vfs(fs_user_mount_t *vfs, int part) {
    vfs->base.type = &mp_fat_vfs_type;
    vfs->flags |= FSUSER_NATIVE | FSUSER_HAVE_IOCTL;
    vfs->fatfs.drv = vfs;
    vfs->fatfs.part = part;
    vfs->readblocks[0] = (mp_obj_t)&pyb_sdcard_readblocks_obj;
    vfs->readblocks[1] = (mp_obj_t)&pyb_sdcard_obj;
    vfs->readblocks[2] = (mp_obj_t)sdcard_read_blocks; // native version
    vfs->writeblocks[0] = (mp_obj_t)&pyb_sdcard_writeblocks_obj;
    vfs->writeblocks[1] = (mp_obj_t)&pyb_sdcard_obj;
    vfs->writeblocks[2] = (mp_obj_t)sdcard_write_blocks; // native version
    vfs->u.ioctl[0] = (mp_obj_t)&pyb_sdcard_ioctl_obj;
    vfs->u.ioctl[1] = (mp_obj_t)&pyb_sdcard_obj;
}

#endif // MICROPY_HW_HAS_SDCARD
