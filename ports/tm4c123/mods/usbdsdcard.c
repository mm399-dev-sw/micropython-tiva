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
#include "mpconfigport.h"
#if MICROPY_HW_ENABLE_USB
#include <stdint.h>
#include <stdbool.h>
#include "inc/hw_ints.h"
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "driverlib/debug.h"
#include "driverlib/sysctl.h"
#include "driverlib/gpio.h"
#include "driverlib/interrupt.h"
#include "driverlib/rom.h"
#include "driverlib/systick.h"
// #include "grlib/grlib.h"
#include "usblib/usblib.h"
#include "usblib/usb-ids.h"
#include "usblib/device/usbdevice.h"
#include "usblib/device/usbdmsc.h"
// #include "usb_msc_structs.h"
#include "third_party/fatfs/src/diskio.h"
#include "sdcard.h"

#define SDCARD_PRESENT          MICROPY_HW_SDCARD_DETECT_PRESENT
#define SDCARD_IN_USE           0x00000002
struct
{
    uint32_t ui32Flags;
}
g_sDriveInformation;

//*****************************************************************************
//
// This function opens the drive number and prepares it for use by the Mass
// storage class device.
//
// /param ui32Drive is the driver number to open.
//
// This function is used to initialize and open the physical drive number
// associated with the parameter /e ui32Drive.  The function will return zero if
// the drive could not be opened for some reason.  In the case of removable
// device like an SD card this function should return zero if the SD card is
// not present.
//
// /return Returns a pointer to data that should be passed to other APIs or it
// will return 0 if no drive was found.
//
//*****************************************************************************
void *
usb_mcs_storage_open(uint_fast32_t ui32Drive)
{
    uint_fast32_t ui32Temp;

    ASSERT(ui32Drive == 0);

    //
    // Return if already in use.
    //
    if(g_sDriveInformation.ui32Flags & SDCARD_IN_USE)
    {
        return(0);
    }

    //
    // Initialize the drive if it is present.
    //
    if (!sdcard_is_initialized())
    {
        ui32Temp = sd_disk_init(0);
    }
    else
    {
        g_sDriveInformation.ui32Flags = SDCARD_PRESENT | SDCARD_IN_USE;
        return (0);
    }

    if(ui32Temp == RES_OK)
    {
        //
        // Card is present and in use.
        //
        g_sDriveInformation.ui32Flags = SDCARD_PRESENT | SDCARD_IN_USE;
    }
    else if(ui32Temp == STA_NODISK)
    {
        //
        // Allocate the card but it is not present.
        //
        g_sDriveInformation.ui32Flags = SDCARD_IN_USE;
    }
    else
    {
        return(0);
    }

    return((void *)&g_sDriveInformation);
}

//*****************************************************************************
//
// This function close the drive number in use by the mass storage class device.
//
// /param pvDrive is the pointer that was returned from a call to
// USBDMSCStorageOpen().
//
// This function is used to close the physical drive number associated with the
// parameter /e pvDrive.  This function will return 0 if the drive was closed
// successfully and any other value will indicate a failure.
//
// /return Returns 0 if the drive was successfully closed or non-zero for a
// failure.
//
//*****************************************************************************
void
usb_mcs_storage_close(void * pvDrive)
{
    uint_fast8_t ui8Power;

    ASSERT(pvDrive != 0);

    //
    // Clear all flags.
    //
    g_sDriveInformation.ui32Flags = 0;

    //
    // Power up the card.
    //
    ui8Power = 0;

    //
    // Turn off the power to the card.
    //
    sd_disk_ioctl(0, CTRL_POWER, &ui8Power);
}

//*****************************************************************************
//
// This function will read a block from a device opened by the
// USBDMSCStorageOpen() call.
//
// /param pvDrive is the pointer that was returned from a call to
// USBDMSCStorageOpen().
// /param pui8Data is the buffer that data will be written into.
// /param ui32NumBlocks is the number of blocks to read.
//
// This function is use to read blocks from a physical device and return them
// in the /e pui8Data buffer.  The data area pointed to by /e pui8Data should be
// at least /e ui32NumBlocks * Block Size bytes to prevent overwriting data.
//
// /return Returns the number of bytes that were read from the device.
//
//*****************************************************************************
uint32_t usb_msc_storage_read(void * pvDrive,
                                 uint8_t *pui8Data,
                                 uint_fast32_t ui32Sector,
                                 uint_fast32_t ui32NumBlocks)
{    ASSERT(pvDrive != 0);

    if(sd_disk_read (0, pui8Data, ui32Sector, ui32NumBlocks) == RES_OK)
    {
        // TODO remove fixed 512
        return(ui32NumBlocks * 512);
    }
    return(0);
}

//*****************************************************************************
//
// This function will write a block to a device opened by the
// USBDMSCStorageOpen() call.
//
// /param pvDrive is the pointer that was returned from a call to
// USBDMSCStorageOpen().
// /param pui8Data is the buffer that data will be used for writing.
// /param ui32NumBlocks is the number of blocks to write.
//
// This function is use to write blocks to a physical device from the buffer
// pointed to by the /e pui8Data buffer.  If the number of blocks is greater than
// one then the block address will increment and write to the next block until
// /e ui32NumBlocks * Block Size bytes have been written.
//
// /return Returns the number of bytes that were written to the device.
//
//*****************************************************************************
uint32_t usb_msc_storage_write(void * pvDrive,
                                  uint8_t *pui8Data,
                                  uint_fast32_t ui32Sector,
                                  uint_fast32_t ui32NumBlocks)
{
    ASSERT(pvDrive != 0);

    if(sd_disk_write (0, pui8Data, ui32Sector, ui32NumBlocks) == RES_OK)
    {
        return(ui32NumBlocks * 512);
    }
    return(0);
}

//*****************************************************************************
//
// This function will return the number of blocks present on a device.
//
// /param pvDrive is the pointer that was returned from a call to
// USBDMSCStorageOpen().
//
// This function is used to return the total number of blocks on a physical
// device based on the /e pvDrive parameter.
//
// /return Returns the number of blocks that are present in a device.
//
//*****************************************************************************
uint32_t
usb_mcs_storage_enum_blocks(void * pvDrive)
{
    uint_fast32_t ui32SectorCount;

    //
    // Read the number of sectors.
    //
    sd_disk_ioctl(0, GET_SECTOR_COUNT, &ui32SectorCount);

    return(ui32SectorCount);
}

//*****************************************************************************
//
// This function will return the current status of a device.
//
// /param pvDrive is the pointer that was returned from a call to
// USBDMSCStorageOpen().
//
// This function is used to return the current status of the device indicated by
// the /e pvDrive parameter.  This can be used to see if the device is busy,
// or if it is present.
//
// /return Returns the size in bytes of blocks that in a device.
//
//*****************************************************************************
#define USBDMSC_IDLE            0x00000000
#define USBDMSC_NOT_PRESENT     0x00000001
uint32_t USB_MSCStorageStatus(void * pvDrive);

#endif // MICROPY_HW_ENABLE_USB