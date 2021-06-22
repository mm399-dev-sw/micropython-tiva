
#ifndef __USB_MSC_STRUCTS_H__
#define __USB_MSC_STRUCTS_H__

#include "stdint.h"
#include "stdbool.h"
#include "inc/hw_ints.h"
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "driverlib/debug.h"
#include "driverlib/sysctl.h"
#include "driverlib/rom.h"
#include "driverlib/systick.h"
#include "driverlib/usb.h"
#include "driverlib/gpio.h"
#include "driverlib/udma.h"
#include "driverlib/pin_map.h"
#include "grlib/grlib.h"
#include "usblib/usblib.h"
#include "usblib/usb-ids.h"
#include "usblib/device/usbdevice.h"
#include "usblib/device/usbdmsc.h"
#include "usb_dev_msc.h"
//#include "lib/oofatfs/diskio.h" // ersetzt
// #include "lib/oofatfs/ff.h"
#include "usbdsdcard.h"
#include "sdcard.h"

//*****************************************************************************
//
// The mass storage class device structure.
//
//*****************************************************************************
extern tUSBDMSCDevice g_sMSCDevice;

//*****************************************************************************
//
// The externally provided mass storage class event call back function.
//
//*****************************************************************************
extern uint32_t USBDMSCEventCallback(void *pvCBData, uint32_t ui32Event,
                                     uint32_t ui32MsgParam, void *pvMsgData);

//*****************************************************************************
//
// The externally provided mass storage class event call back function.
//
//*****************************************************************************

int usb_msc_device_main(void);


#endif
