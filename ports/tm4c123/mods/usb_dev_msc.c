//*****************************************************************************
//
// usb_dev_msc.c - Main routines for the device mass storage class example.
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

#include "usb_dev_msc.h"
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
//#include "usblib/usblib.h"
#include "usblib/usb-ids.h"
//#include "usblib/device/usbdevice.h"
//#include "usblib/device/usbdmsc.h"
//#include "lib/oofatfs/diskio.h" // ersetzt
// #include "lib/oofatfs/ff.h"
#include "usbdsdcard.h"
#include "sdcard.h"
#include "mpconfigboard.h"
#include "mphalport.h"
#include "rom_map.h"
#include "pin.h"
#include "irq.h"
#include "dma.h"


#define USB_EPEN            GPIO_PIN_4
#define USB_EPEN_GPIO_BASE  GPIO_PORTF_BASE
#define USB_EPEN_PERIPH     SYSCTL_PERIPH_GPIOF

#define USB_IRQn            INT_USB0_TM4C123

//uint32_t DMA_RX32Buffer[0x1000>>2];


//*****************************************************************************
//
// A N F A N G usb_msc_structs.c
//
//*****************************************************************************

// #include "usbdsdcard.h"

//*****************************************************************************
//
// The languages supported by this device.
//
//*****************************************************************************
const uint8_t g_pui8LangDescriptor[] =
{
    4,
    USB_DTYPE_STRING,
    USBShort(USB_LANG_EN_US)
};

//*****************************************************************************
//
// The manufacturer string.
//
//*****************************************************************************
const uint8_t g_pui8ManufacturerString[] =
{
    (17 + 1) * 2,
    USB_DTYPE_STRING,
    'T', 0, 'e', 0, 'x', 0, 'a', 0, 's', 0, ' ', 0, 'I', 0, 'n', 0, 's', 0,
    't', 0, 'r', 0, 'u', 0, 'm', 0, 'e', 0, 'n', 0, 't', 0, 's', 0,
};

//*****************************************************************************
//
// The product string.
//
//*****************************************************************************
const uint8_t g_pui8ProductString[] =
{
    (19 + 1) * 2,
    USB_DTYPE_STRING,
    'M', 0, 'a', 0, 's', 0, 's', 0, ' ', 0, 'S', 0, 't', 0, 'o', 0, 'r', 0,
    'a', 0, 'g', 0, 'e', 0, ' ', 0, 'D', 0, 'e', 0, 'v', 0, 'i', 0, 'c', 0,
    'e', 0
};

//*****************************************************************************
//
// The serial number string.
//
//*****************************************************************************
const uint8_t g_pui8SerialNumberString[] =
{
    (8 + 1) * 2,
    USB_DTYPE_STRING,
    '1', 0, '2', 0, '3', 0, '4', 0, '5', 0, '6', 0, '7', 0, '8', 0
};

//*****************************************************************************
//
// The data interface description string.
//
//*****************************************************************************
const uint8_t g_pui8DataInterfaceString[] =
{
    (19 + 1) * 2,
    USB_DTYPE_STRING,
    'B', 0, 'u', 0, 'l', 0, 'k', 0, ' ', 0, 'D', 0, 'a', 0, 't', 0,
    'a', 0, ' ', 0, 'I', 0, 'n', 0, 't', 0, 'e', 0, 'r', 0, 'f', 0,
    'a', 0, 'c', 0, 'e', 0
};

//*****************************************************************************
//
// The configuration description string.
//
//*****************************************************************************
const uint8_t g_pui8ConfigString[] =
{
    (23 + 1) * 2,
    USB_DTYPE_STRING,
    'B', 0, 'u', 0, 'l', 0, 'k', 0, ' ', 0, 'D', 0, 'a', 0, 't', 0,
    'a', 0, ' ', 0, 'C', 0, 'o', 0, 'n', 0, 'f', 0, 'i', 0, 'g', 0,
    'u', 0, 'r', 0, 'a', 0, 't', 0, 'i', 0, 'o', 0, 'n', 0
};

//*****************************************************************************
//
// The descriptor string table.
//
//*****************************************************************************
const uint8_t * const g_ppui8StringDescriptors[] =
{
    g_pui8LangDescriptor,
    g_pui8ManufacturerString,
    g_pui8ProductString,
    g_pui8SerialNumberString,
    g_pui8DataInterfaceString,
    g_pui8ConfigString
};

#define NUM_STRING_DESCRIPTORS (sizeof(g_ppui8StringDescriptors) /            \
                                sizeof(uint8_t *))

//*****************************************************************************
//
// The bulk device initialization and customization structures. In this case,
// we are using USBBuffers between the bulk device class driver and the
// application code. The function pointers and callback data values are set
// to insert a buffer in each of the data channels, transmit and receive.
//
// With the buffer in place, the bulk channel callback is set to the relevant
// channel function and the callback data is set to point to the channel
// instance data. The buffer, in turn, has its callback set to the application
// function and the callback data set to our bulk instance structure.
//
//*****************************************************************************
tUSBDMSCDevice g_sMSCDevice =
{
    //
    // Vendor ID.
    //
    USB_VID_TI_1CBE,

    //
    // Product ID.
    //
    USB_PID_MSC,

    //
    // Vendor Information.
    //
    "TI      ",

    //
    // Product Identification.
    //
    "Mass Storage    ",

    //
    // Revision.
    //
    "1.00",
    500,
    USB_CONF_ATTR_SELF_PWR,
    g_ppui8StringDescriptors,
    NUM_STRING_DESCRIPTORS,
    {
        USB_MSCStorageOpen,
        USB_MSCStorageClose,
        USB_MSCStorageRead,
        USB_MSCStorageWrite,
        USB_MSCStorageNumBlocks,
        0,
    },
    USBDMSCEventCallback
};

//*****************************************************************************
//
// E N D E usb_msc_structs.c
//
//*****************************************************************************
 
//*****************************************************************************
//
// The size of the transmit and receive buffers used.
//
//*****************************************************************************
#define MSC_BUFFER_SIZE 512



//*****************************************************************************
//
//! \addtogroup example_list
//! <h1>USB MSC Device (usb_dev_msc)</h1>
//!
//! This example application turns the evaluation board into a USB mass storage
//! class device.  The application will use the microSD card for the storage
//! media for the mass storage device.  The screen will display the current
//! action occurring on the device ranging from disconnected, no media, reading,
//! writing and idle.
//
//*****************************************************************************


//*****************************************************************************
//
// The number of ticks to wait before falling back to the idle state.  Since
// the tick rate is 100Hz this is approximately 3 seconds.
//
//*****************************************************************************
#define USBMSC_ACTIVITY_TIMEOUT 300

//*****************************************************************************
//
// This enumeration holds the various states that the device can be in during
// normal operation.
//
//*****************************************************************************
volatile enum
{
    // Unconfigured.
    MSC_DEV_DISCONNECTED,

    // Connected but not yet fully enumerated.
    MSC_DEV_CONNECTED,

    // Connected and fully enumerated but not currently handling a command.
    MSC_DEV_IDLE,

    // Currently reading the SD card.
    MSC_DEV_READ,

    // Currently writing the SD card.
    MSC_DEV_WRITE,
}
g_eMSCState;

//*****************************************************************************
//
// The Flags that handle updates to the status area to avoid drawing when no
// updates are required..
//
//*****************************************************************************
#define FLAG_UPDATE_STATUS      1
static uint32_t g_ui32Flags;
static uint32_t g_ui32IdleTimeout;


//******************************************************************************
//
// The DMA control structure table.
//
//******************************************************************************

tDMAControlTable sDMAControlTable[64] __attribute__ ((aligned(1024)));


//*****************************************************************************
//
// Handles bulk driver notifications related to the receive channel (data from
// the USB host).
//
// \param pvCBData is the client-supplied callback pointer for this channel.
// \param ui32Event identifies the event we are being notified about.
// \param ui32MsgValue is an event-specific value.
// \param pvMsgData is an event-specific pointer.
//
// This function is called by the bulk driver to notify us of any events
// related to operation of the receive data channel (the OUT channel carrying
// data from the USB host).
//
// \return The return value is event-specific.
//
//*****************************************************************************
uint32_t
RxHandler(void *pvCBData, uint_fast32_t ui32Event,
               uint_fast32_t ui32MsgValue, void *pvMsgData)
{
    return(0);
}

//*****************************************************************************
//
// Handles bulk driver notifications related to the transmit channel (data to
// the USB host).
//
// \param pvCBData is the client-supplied callback pointer for this channel.
// \param ui32Event identifies the event we are being notified about.
// \param ui32MsgValue is an event-specific value.
// \param pvMsgData is an event-specific pointer.
//
// This function is called by the bulk driver to notify us of any events
// related to operation of the transmit data channel (the IN channel carrying
// data to the USB host).
//
// \return The return value is event-specific.
//
//*****************************************************************************
uint32_t
TxHandler(void *pvCBData, uint_fast32_t ui32Event, uint_fast32_t ui32MsgValue,
          void *pvMsgData)
{
    return(0);
}


//*****************************************************************************
//
// This function is the call back notification function provided to the USB
// library's mass storage class.
//
//*****************************************************************************
uint32_t
USBDMSCEventCallback(void *pvCBData, uint32_t ui32Event, uint32_t ui32MsgParam,
                     void *pvMsgData)
{
    //
    // Reset the time out every time an event occurs.
    //
    g_ui32IdleTimeout = USBMSC_ACTIVITY_TIMEOUT;

    switch(ui32Event)
    {
        //
        // Writing to the device.
        //
        case USBD_MSC_EVENT_WRITING:
        {
            // Only update if this is a change.
            if(g_eMSCState != MSC_DEV_WRITE)
            {
                // Go to the write state.
                g_eMSCState = MSC_DEV_WRITE;

                // Cause the main loop to update the screen.
                g_ui32Flags |= FLAG_UPDATE_STATUS;
            }

            break;
        }

        // Reading from the device.
        case USBD_MSC_EVENT_READING:
        {
            // Only update if this is a change.
            if(g_eMSCState != MSC_DEV_READ)
            {
                // Go to the read state.
                g_eMSCState = MSC_DEV_READ;

                // Cause the main loop to update the screen.
                g_ui32Flags |= FLAG_UPDATE_STATUS;
            }

            break;
        }
        // The USB host has disconnected from the device.
        case USB_EVENT_DISCONNECTED:
        {
            // Go to the disconnected state.
            g_eMSCState = MSC_DEV_DISCONNECTED;

            // Cause the main loop to update the screen.
            g_ui32Flags |= FLAG_UPDATE_STATUS;

            break;
        }
        // The USB host has connected to the device.
        case USB_EVENT_CONNECTED:
        {
            // Go to the idle state to wait for read/writes.
            g_eMSCState = MSC_DEV_IDLE;

            break;
        }
        case USBD_MSC_EVENT_IDLE:
        default:
        {
            break;
        }
    }

    return(0);
}

//*****************************************************************************
//
// This is the handler for this SysTick interrupt.  FatFs requires a timer tick
// every 10 ms for internal timing purposes.
//
//*****************************************************************************
// void
// SysTickHandler(void)
// {
//     // Call the FatFs tick timer.
//     disk_timerproc();

//     if(g_ui32IdleTimeout != 0)
//     {
//         g_ui32IdleTimeout--;
//     }
// }


//*****************************************************************************
//
// This is the main loop that runs the application.
//
//*****************************************************************************
int usb_msc_device(void)
{
    uint_fast32_t ui32Retcode;

    // Configure SysTick for a 100Hz interrupt.  The FatFs driver wants a 10 ms
    // tick.
    // ROM_SysTickPeriodSet(ROM_SysCtlClockGet() / 100);
    // ROM_SysTickEnable();
    // ROM_SysTickIntEnable();

    // Configure and enable uDMA
    // ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_UDMA);  //BEREITS AUFGERUFEN DURCH MAIN.C
    // SysCtlDelay(10);
    // uDMAControlBaseSet(&sDMAControlTable[0]);        //BEREITS AUFGERUFEN VON DMA.C -> dma_init(...)
    // ROM_uDMAEnable();                                //BEREITS AUFGERUFEN VON DMA.C -> dma_hw_init(...)
    
    // Own config of uDMA for USB0
    dma_init(UDMA_CHANNEL_USBEP1RX, DMA_CHANNEL_RX);
    dma_init(UDMA_CHANNEL_USBEP1TX, DMA_CHANNEL_TX);
    //usb_dma_tx(UDMA_CHANNEL_USBEP1TX);
    //usb_dma_rx(UDMA_CHANNEL_USBEP1RX);



    // Configure USB Interrupt 
    // NVIC_SetPriority(USB_IRQn, IRQ_PRI_USB_MSC);
    IntEnable(USB_IRQn);
    // NVIC_EnableIRQ(USB_IRQn);

    // Initialize the idle timeout and reset all flags.
    g_ui32IdleTimeout = 0;
    g_ui32Flags = 0;

    // Initialize the state to idle.
    g_eMSCState = MSC_DEV_DISCONNECTED;



    // Init USB module

    // disable USB module
    SysCtlPeripheralDisable(SYSCTL_PERIPH_USB0);
    // reset module
    SysCtlPeripheralReset(SYSCTL_PERIPH_USB0);
    // Enable the USB controller. (SYSCTL->RCGCUSB)
    SysCtlPeripheralEnable(SYSCTL_PERIPH_USB0);
    // Wait until ready
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_USB0))
    {
    }

    // Set the USB pins to be controlled by the USB controller.
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOB))
    {
    }
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOD))
    {
    }
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOF))
    {
    }

    // configure PF4 as USB0 EPEN 
    GPIOPinConfigure(GPIO_PF4_USB0EPEN);
    mp_hal_pin_config_alt(MICROPY_HW_USB0_EPEN, PIN_FN_USB, 0);

    // configure PD4 as USB0 DM; PD5 as USB0 DP
    mp_hal_pin_config_alt(MICROPY_HW_USB0_DM, PIN_FN_USB, 0);   // bei TM4C123G: PL6, PL7; USB0DM=PD4, USB0DP=PD5
    mp_hal_pin_config_alt(MICROPY_HW_USB0_DP, PIN_FN_USB, 0);

    // configure PB0 as USB0 ID; PB1 as USB0 VBUS 
    // gleich wie bei TM4C123G: PB0 = USB0ID; PB1 = USB0VBUS
    mp_hal_pin_config_alt(MICROPY_HW_USB0_ID, PIN_FN_USB, 0);
    mp_hal_pin_config_alt(MICROPY_HW_USB0_VBUS, PIN_FN_USB, 0);

    // Set the USB stack mode to Device mode WITHOUT (!) VBUS monitoring.
    USBStackModeSet(0, eUSBModeForceDevice, 0);

    // Pass our device information to the USB library and place the device
    // on the bus.
    USBDMSCInit(0, &g_sMSCDevice);

    // Determine whether or not an SDCard is installed.  If not, print a
    // warning and have the user install one and restart.
    ui32Retcode = sd_disk_init(0);

    if(ui32Retcode != RES_OK) 
    {
        // not found
        mp_printf(MP_PYTHON_PRINTER, "SD Card not found.\nInstall one and reset board.\n");
    }
    else
    {
        // found
        mp_printf(MP_PYTHON_PRINTER, "SD Card has no Problems.\nTo exit MSC mode reset the board.\n");
    }
    // uint32_t USBIRQ_Prio, DMAIRQ_Prio, FLASHIRQ_Prio;

    // USBIRQ_Prio = NVIC_GetPriority(INT_USB0_TM4C123);
    // DMAIRQ_Prio = NVIC_GetPriority(INT_UDMA_TM4C123);
    // FLASHIRQ_Prio = NVIC_GetPriority(INT_FLASH);

    // mp_printf(MP_PYTHON_PRINTER, "USBIRQ_Prio: %d, DMAIRQ_Prio: %d, FLASHIRQ_Prio: %d\n",USBIRQ_Prio,DMAIRQ_Prio,FLASHIRQ_Prio);

    //mp_printf(MP_PYTHON_PRINTER, "Alles bis zur Schleife done %u", g_eMSCState);
    //
    // Drop into the main loop.
    //

    // for (;;)
    // {
    //     switch(g_eMSCState)
    //     {
    //         case MSC_DEV_READ:
    //         {
    //             // Update the screen if necessary.
    //             if(g_ui32Flags & FLAG_UPDATE_STATUS)
    //             {
    //                 g_ui32Flags &= ~FLAG_UPDATE_STATUS;
    //             }

    //             // If there is no activity then return to the idle state.
    //             if(g_ui32IdleTimeout == 0)
    //             {
    //                 g_eMSCState = MSC_DEV_IDLE;
    //             }

    //             break;
    //         }
    //         case MSC_DEV_WRITE:
    //         {
    //             // Update the screen if necessary.
    //             if(g_ui32Flags & FLAG_UPDATE_STATUS)
    //             {
    //                 g_ui32Flags &= ~FLAG_UPDATE_STATUS;
    //             }

    //             //
    //             // If there is no activity then return to the idle state.
    //             //
    //             if(g_ui32IdleTimeout == 0)
    //             {
    //                 g_eMSCState = MSC_DEV_IDLE;
    //             }
    //             break;
    //         }
    //         case MSC_DEV_DISCONNECTED:
    //         {   
    //             // Update the screen if necessary.
    //             if(g_ui32Flags & FLAG_UPDATE_STATUS)
    //             {
    //                 g_ui32Flags &= ~FLAG_UPDATE_STATUS;
    //             }
    //             break;
    //         }
    //         case MSC_DEV_IDLE:
    //         {   
                
    //             break;
    //         }
    //         default:
    //         {
    //             break;
    //         }
    //     }
    // }
    return 0;
}

//*****************************************************************************
//
// MicroPython Binding
//
//*****************************************************************************


STATIC mp_obj_t mp_machine_usb_device_msc() {
    // if (machine_hard_i2c_is_ready((mp_obj_base_t *)self, addr) == true) {
    //     return mp_const_true;
    // } else {
    //     return mp_const_false;
    // }
    usb_msc_device();

    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_0(mp_machine_usb_msc_device_obj, mp_machine_usb_device_msc);

/*
        Table with all globals
*/
STATIC const mp_map_elem_t usb_dev_globals_table[]= { 
{ MP_OBJ_NEW_QSTR(MP_QSTR___name__), MP_OBJ_NEW_QSTR(MP_QSTR_usbmsc) }, 
{ MP_OBJ_NEW_QSTR(MP_QSTR_mscdev), (mp_obj_t)&mp_machine_usb_msc_device_obj }, 
}; 


/*
        Define all globals as uPy-globals
*/
STATIC MP_DEFINE_CONST_DICT(usb_msc_globals, usb_dev_globals_table); 


const mp_obj_module_t usb_msc_module = { 
.base = { &mp_type_module }, 
.globals = (mp_obj_dict_t*)&usb_msc_globals, 
}; 
//*****************************************************************************
//
// The error routine that is called if the driver library encounters an error.
//
//*****************************************************************************
#ifdef DEBUG
void
__error__(char *pcFilename, uint32_t ui32Line)
{
    while(1)
    {
        //
        // Hang on runtime error.
        //
    }
}
#endif
