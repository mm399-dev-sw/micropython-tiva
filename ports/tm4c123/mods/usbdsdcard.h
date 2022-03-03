//*****************************************************************************
//
// usbdsdcard.h - Prototypes for functions supplied for use by the mass storage
// class device.
//
// Copyright (c) 2012-2017 Texas Instruments Incorporated.  All rights reserved.
// Software License Agreement
// 
// Texas Instruments (TI) is supplying this software for use solely and
// exclusively on TI's microcontroller products. The software is owned by
// TI and/or its suppliers, and is protected under applicable copyright
// laws. You may not combine this software with "viral" open-source
// software in order to form a larger program.
// 
// THIS SOFTWARE IS PROVIDED "AS IS" AND WITH ALL FAULTS.
// NO WARRANTIES, WHETHER EXPRESS, IMPLIED OR STATUTORY, INCLUDING, BUT
// NOT LIMITED TO, IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
// A PARTICULAR PURPOSE APPLY TO THIS SOFTWARE. TI SHALL NOT, UNDER ANY
// CIRCUMSTANCES, BE LIABLE FOR SPECIAL, INCIDENTAL, OR CONSEQUENTIAL
// DAMAGES, FOR ANY REASON WHATSOEVER.
// 
// This is part of revision 2.1.4.178 of the DK-TM4C123G Firmware Package.
//
//*****************************************************************************

#ifndef __USBDSDCARD_H__
#define __USBDSDCARD_H__

#include "stdint.h"
//*****************************************************************************
//
//
//*****************************************************************************
extern void * USB_MSCStorageOpen(uint32_t ui32Drive);
extern void USB_MSCStorageClose(void * pvDrive);
extern uint32_t USB_MSCStorageRead(void * pvDrive, uint8_t *pui8Data, uint32_t ui32Sector, uint32_t ui32NumBlocks);
extern uint32_t USB_MSCStorageWrite(void * pvDrive, uint8_t *pui8Data, uint32_t ui32Sector, uint32_t ui32NumBlocks);
uint32_t USB_MSCStorageNumBlocks(void * pvDrive);

#endif
