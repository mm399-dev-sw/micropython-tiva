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

#include "py/mpconfig.h"
#include "py/misc.h"
#include "flash.h"

#include "inc/hw_flash.h"
#include "driverlib/flash.h"
#include "driverlib/rom_map.h"

extern uint32_t _flash_fs_start;

typedef struct {
    uint32_t base_address;
    uint32_t sector_size;
    uint32_t sector_count;
} flash_layout_t;

static const flash_layout_t flash_layout[] = {
    { (uint32_t)&_flash_fs_start, 0x00400, 256 }
};

uint32_t flash_get_sector_info(uint32_t addr, uint32_t *start_addr, uint32_t *size) {
    if (addr >= flash_layout[0].base_address) {
        uint32_t sector_index = 0;
        for (int i = 0; i < MP_ARRAY_SIZE(flash_layout); ++i) {
            for (int j = 0; j < flash_layout[i].sector_count; ++j) {
                uint32_t sector_start_next = flash_layout[i].base_address
                    + (j + 1) * flash_layout[i].sector_size;
                if (addr < sector_start_next) {
                    if (start_addr != NULL) {
                        *start_addr = flash_layout[i].base_address
                            + j * flash_layout[i].sector_size;
                    }
                    if (size != NULL) {
                        *size = flash_layout[i].sector_size;
                    }
                    return sector_index;
                }
                ++sector_index;
            }
        }
    }
    return 0;
}

void flash_erase(uint32_t flash_dest, uint32_t num_word32) {
    // check there is something to write
    uint num_sec = 1;
    int addr_offset = 0;
    if (num_word32 == 0) {
        return;
    }

    if(!(flash_dest % 4)) {
        // DEBUG_printf("mods/flash.c/flash_erase(): Flash address must be 4 byte aligned!");
        return;
    }

    // check for write permissions
    if (FlashProtectGet(flash_dest) != FlashReadWrite)
    {
        // DEBUG_printf("Flash Block 0x%.8X write protected! Not erasing!", flash_dest);
        return; 
    }

    // Address needs to point to sector start
    if(!(flash_dest & (FLASH_ERASE_SIZE -1))) {
        addr_offset = flash_dest & (FLASH_ERASE_SIZE -1);
        flash_dest = flash_dest & ~(FLASH_ERASE_SIZE -1); 
        if(num_word32 > addr_offset) {
            num_sec ++;
            num_word32 -= addr_offset/4;    // 32 bits address increments in byte steps > DIVIDE BY 4
        }
    }

    while(num_word32 > FLASH_ERASE_SIZE) {
            num_sec ++;
            num_word32 -= FLASH_ERASE_SIZE;
            
    }  

    for(; num_sec > 0 ; num_sec--) {
        if (MAP_FlashErase(flash_dest) != 0) {
            // error occurred during sector erase
            return;
        }
    }
}

/*
// erase the sector using an interrupt
void flash_erase_it(uint32_t flash_dest, uint32_t num_word32) {
    // check there is something to write
    if (num_word32 == 0) {
        return;
    }

    // unlock
    HAL_FLASH_Unlock();

    // Clear pending flags (if any)
    __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_EOP | FLASH_FLAG_OPERR | FLASH_FLAG_WRPERR |
                           FLASH_FLAG_PGAERR | FLASH_FLAG_PGPERR|FLASH_FLAG_PGSERR);

    // erase the sector(s)
    FLASH_EraseInitTypeDef EraseInitStruct;
    EraseInitStruct.TypeErase = TYPEERASE_SECTORS;
    EraseInitStruct.VoltageRange = VOLTAGE_RANGE_3; // voltage range needs to be 2.7V to 3.6V
    EraseInitStruct.Sector = flash_get_sector_info(flash_dest, NULL, NULL);
    EraseInitStruct.NbSectors = flash_get_sector_info(flash_dest + 4 * num_word32 - 1, NULL, NULL) - EraseInitStruct.Sector + 1;
    if (HAL_FLASHEx_Erase_IT(&EraseInitStruct) != HAL_OK) {
        // error occurred during sector erase
        HAL_FLASH_Lock(); // lock the flash
        return;
    }
}
*/

void flash_write(uint32_t flash_dest, const uint32_t *src, uint32_t num_word32) {
    
    // program the flash word by word
    if(flash_dest % 4 != 0) {
        // DEBUG_printf("mods/flash.c/flash_erase(): Flash address must be 4 byte aligned!");
        return;
    }

    // check for write permissions
    if (FlashProtectGet(flash_dest) != FlashReadWrite)
    {
        // DEBUG_printf("Flash Block 0x%.8X write protected! Not programming!", flash_dest);
        return; 
    }

    if (MAP_FlashProgram((uint32_t*) src, flash_dest, 4 * num_word32) != 0) {
        // error occurred during flash write
        return;
    }

}

/*
 use erase, then write
void flash_erase_and_write(uint32_t flash_dest, const uint32_t *src, uint32_t num_word32) {
    // check there is something to write
    if (num_word32 == 0) {
        return;
    }

    // unlock
    HAL_FLASH_Unlock();

    // Clear pending flags (if any)
    __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_EOP | FLASH_FLAG_OPERR | FLASH_FLAG_WRPERR |
                           FLASH_FLAG_PGAERR | FLASH_FLAG_PGPERR|FLASH_FLAG_PGSERR);

    // erase the sector(s)
    FLASH_EraseInitTypeDef EraseInitStruct;
    EraseInitStruct.TypeErase = TYPEERASE_SECTORS;
    EraseInitStruct.VoltageRange = VOLTAGE_RANGE_3; // voltage range needs to be 2.7V to 3.6V
    EraseInitStruct.Sector = flash_get_sector_info(flash_dest, NULL, NULL);
    EraseInitStruct.NbSectors = flash_get_sector_info(flash_dest + 4 * num_word32 - 1, NULL, NULL) - EraseInitStruct.Sector + 1;
    uint32_t SectorError = 0;
    if (HAL_FLASHEx_Erase(&EraseInitStruct, &SectorError) != HAL_OK) {
        // error occurred during sector erase
        HAL_FLASH_Lock(); // lock the flash
        return;
    }

    // program the flash word by word
    for (int i = 0; i < num_word32; i++) {
        if (HAL_FLASH_Program(TYPEPROGRAM_WORD, flash_dest, *src) != HAL_OK) {
            // error occurred during flash write
            HAL_FLASH_Lock(); // lock the flash
            return;
        }
        flash_dest += 4;
        src += 1;
    }

    // lock the flash
    HAL_FLASH_Lock();
}
*/
