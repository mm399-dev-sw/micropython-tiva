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

#include <stdarg.h>
#include <string.h>


#include "py/objstr.h"
#include "py/runtime.h"
#include "py/stream.h"
#include "py/mperrno.h"
#include "py/mphal.h"
#include "bufhelper.h"
#include "usb.h"


// Modified Version of MSC Example - tivaware
#if MICROPY_HW_ENABLE_USB

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
#include "usblib/usb-ids.h"
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


//*****************************************************************************
//
// Structs & Config
//
//*****************************************************************************

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
tUSBDMSCDevice t_MSCDevice =
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
// The size of the transmit and receive buffers used.
//
//*****************************************************************************
#define MSC_BUFFER_SIZE 512



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
MSCState;

//*****************************************************************************
//
// The Flags that handle updates to the status area to avoid drawing when no
// updates are required..
//
//*****************************************************************************
#define FLAG_UPDATE_STATUS      1
static uint32_t g_ui32Flags;
static uint32_t g_ui32IdleTimeout;



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
            if(MSCState != MSC_DEV_WRITE)
            {
                // Go to the write state.
                MSCState = MSC_DEV_WRITE;

                // Cause the main loop to update the screen.
                g_ui32Flags |= FLAG_UPDATE_STATUS;
            }

            break;
        }

        // Reading from the device.
        case USBD_MSC_EVENT_READING:
        {
            // Only update if this is a change.
            if(MSCState != MSC_DEV_READ)
            {
                // Go to the read state.
                MSCState = MSC_DEV_READ;

                // Cause the main loop to update the screen.
                g_ui32Flags |= FLAG_UPDATE_STATUS;
            }

            break;
        }
        // The USB host has disconnected from the device.
        case USB_EVENT_DISCONNECTED:
        {
            // Go to the disconnected state.
            MSCState = MSC_DEV_DISCONNECTED;

            // Cause the main loop to update the screen.
            g_ui32Flags |= FLAG_UPDATE_STATUS;

            break;
        }
        // The USB host has connected to the device.
        case USB_EVENT_CONNECTED:
        {
            // Go to the idle state to wait for read/writes.
            MSCState = MSC_DEV_IDLE;

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
// MicroPy Instance of a USB MSC Device
//
//*****************************************************************************

// this will be persistent across a soft-reset
mp_uint_t pyb_usb_flags = 0;

typedef struct _usb_device_t {
    uint32_t enabled;
    uint8_t mode;
    tUSBDMSCDevice *MSCDevice;
    void *initedMSCDeviceInst;
} usb_device_t;

usb_device_t mp_usb_device;

//*****************************************************************************
//
// This is the Init function for a USB MSC Device
//
//*****************************************************************************

int usb_msc_device(void)
{
    uint_fast32_t ui32Retcode;
    usb_device_t *usb_dev = &mp_usb_device;
    usb_dev->MSCDevice = &t_MSCDevice;

    // Configure and enable uDMA for USB0
    dma_init(UDMA_CHANNEL_USBEP1RX, DMA_CHANNEL_RX);
    dma_init(UDMA_CHANNEL_USBEP1TX, DMA_CHANNEL_TX);

    // Configure USB Interrupt 
    // NVIC_SetPriority(USB_IRQn, IRQ_PRI_USB_MSC);
    IntEnable(USB_IRQn);

    // Initialize the idle timeout and reset all flags.
    g_ui32IdleTimeout = 0;
    g_ui32Flags = 0;

    // Initialize the state to idle.
    MSCState = MSC_DEV_DISCONNECTED;


    // Init USB module

    // disable USB module
    ROM_SysCtlPeripheralDisable(SYSCTL_PERIPH_USB0);
    // reset module
    ROM_SysCtlPeripheralReset(SYSCTL_PERIPH_USB0);
    // Enable the USB controller. (SYSCTL->RCGCUSB)
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_USB0);
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
    ROM_GPIOPinConfigure(GPIO_PF4_USB0EPEN);
    mp_hal_pin_config_alt(MICROPY_HW_USB0_EPEN, PIN_FN_USB, 0);

    // configure PD4 as USB0 DM; PD5 as USB0 DP
    mp_hal_pin_config_alt(MICROPY_HW_USB0_DM, PIN_FN_USB, 0);
    mp_hal_pin_config_alt(MICROPY_HW_USB0_DP, PIN_FN_USB, 0);

    // configure PB0 as USB0 ID; PB1 as USB0 VBUS 
    mp_hal_pin_config_alt(MICROPY_HW_USB0_ID, PIN_FN_USB, 0);
    mp_hal_pin_config_alt(MICROPY_HW_USB0_VBUS, PIN_FN_USB, 0);

    // Set the USB stack mode to Device mode WITHOUT (!) VBUS monitoring.
    USBStackModeSet(0, eUSBModeForceDevice, 0);

    // Pass our device information to the USB library and place the device
    // on the bus.
    usb_dev->initedMSCDeviceInst = USBDMSCInit(0, usb_dev->MSCDevice);

    // Determine whether or not an SDCard is installed.  If not, print a
    // warning and have the user install one and restart.
    ui32Retcode = sd_disk_init(0);

    if(ui32Retcode != RES_OK) 
    {
        // SD not found
        mp_printf(MP_PYTHON_PRINTER, "SD Card not found.\nInstall one and reset board.\n");
    }
    else
    {
        // SD found
        mp_printf(MP_PYTHON_PRINTER, "SD Card has no Problems.\nTo exit MSC mode reset the board.\n");
    }
    
    return 0;
}

//*****************************************************************************
//
// MicroPython Helper Functions
//
//*****************************************************************************


pyb_usb_storage_medium_t pyb_usb_storage_medium = PYB_USB_STORAGE_MEDIUM_NONE;


// General USB Device Init Function
// Currently just for MSC Device
bool pyb_usb_dev_init(usb_device_mode_t mode) {
    usb_device_t *usb_dev = &mp_usb_device;
    usb_dev->MSCDevice = &t_MSCDevice;

    // USB Dev may only be initialised once in its power-lifetime
    if (!usb_dev->enabled) {

        if(mode == USBD_MODE_MSC){
            usb_msc_device();
        }

        // configure the VID, PID and the USBD mode (interfaces it will expose)
        //USBD_SetVIDPIDRelease(&usb_dev->usbd_cdc_msc_hid_state, vid, pid, 0x0200, mode == USBD_MODE_CDC);
        // if (USBD_SelectMode(&usb_dev->usbd_cdc_msc_hid_state, mode, hid_info) != 0) {
        //     return false;
        // }
        // set up the USBD state
        // USBD_HandleTypeDef *usbd = &usb_dev->hUSBDDevice;
        // usbd->id = MICROPY_HW_USB_MAIN_DEV;
        // usbd->dev_state  = USBD_STATE_DEFAULT;
        // usbd->pDesc = (USBD_DescriptorsTypeDef*)&USBD_Descriptors;
        // usbd->pClass = &USBD_CDC_MSC_HID;
        // usb_dev->usbd_cdc_msc_hid_state.pdev = usbd;
        // usb_dev->usbd_cdc_msc_hid_state.cdc = &usb_dev->usbd_cdc_itf;
        // usb_dev->usbd_cdc_msc_hid_state.hid = &usb_dev->usbd_hid_itf;
        // usbd->pClassData = &usb_dev->usbd_cdc_msc_hid_state;
    
        usb_dev->enabled = true;
    }
    else{
        mp_printf(MP_PYTHON_PRINTER, "Mode already active\n");
    }

    return true;
}

void pyb_usb_dev_deinit(void) {
    usb_device_t *usb_dev = &mp_usb_device;
    if (usb_dev->enabled) {
        mp_printf(MP_PYTHON_PRINTER, "Please reset the Device\n");
        USBDMSCTerm(usb_dev->initedMSCDeviceInst);
        usb_dev->enabled = false;
    }
}

bool usb_vcp_is_enabled(void) {
    return mp_usb_device.enabled;
}

// int usb_vcp_recv_byte(uint8_t *c) {
//     return usbd_cdc_rx(&usb_device.usbd_cdc_itf, c, 1, 0);
// }

// void usb_vcp_send_strn(const char *str, int len) {
//     if (usb_device.enabled) {
//         usbd_cdc_tx_always(&usb_device.usbd_cdc_itf, (const uint8_t*)str, len);
//     }
// }



/******************************************************************************/
// MicroPython bindings for USB

/*
  Philosophy of USB driver and Python API: pyb.usb_mode(...) configures the USB
  on the board.

  We have:

    umachine.USB.usb_mode()          # return the current usb mode
    umachine.USB.usb_mode(None)      # disable USB
    umachine.USB.usb_mode('MSC')     # enable MSC interface
    
    vcp = pyb.USB_VCP() # get the VCP device for read/write
    hid = pyb.USB_HID() # get the HID device for write/poll

  Possible extensions:
    umachine.USB.usb_mode('host', ...)
    umachine.USB.usb_mode('OTG', ...)
    umachine.USB.usb_mode('VCP+HID') # enable with VCP and HID, defaulting to mouse protocol
    umachine.USB.usb_mode('VCP+HID', vid=0xf055, pid=0x9800) # specify VID and PID
    umachine.USB.usb_mode('VCP+HID', hid=umachine.USB.hid_mouse)
    umachine.USB.usb_mode('VCP+HID', hid=umachine.USB.hid_keyboard)
    umachine.USB.usb_mode('VCP+HID', pid=0x1234, hid=(subclass, protocol, max_packet_len, polling_interval, report_desc))
    umachine.USB.usb_mode('VCP')     # enable with VCP interface
    umachine.USB.usb_mode(..., port=2) # for second USB port

    vcp = pyb.USB_VCP() # get the VCP device for read/write
    hid = pyb.USB_HID() # get the HID device for write/poll
*/

STATIC mp_obj_t pyb_usb_mode(size_t n_args, const mp_obj_t *pos_args, mp_map_t *kw_args) {
    static const mp_arg_t allowed_args[] = {
        { MP_QSTR_mode, MP_ARG_REQUIRED | MP_ARG_OBJ, {.u_obj = mp_const_none} },
        { MP_QSTR_vid, MP_ARG_KW_ONLY | MP_ARG_INT, {.u_int = USBD_VID} },
        { MP_QSTR_pid, MP_ARG_KW_ONLY | MP_ARG_INT, {.u_int = -1} }
    };
    // { MP_QSTR_hid, MP_ARG_KW_ONLY | MP_ARG_OBJ, {.u_obj = (mp_obj_t)&pyb_usb_hid_mouse_obj} }

    // fetch the current usb mode -> pyb.usb_mode()
    if (n_args == 0) {
    #if defined(USE_HOST_MODE)
        return MP_OBJ_NEW_QSTR(MP_QSTR_host);
    #else
        // uint8_t mode = USBD_GetMode(&usb_device.usbd_cdc_msc_hid_state);
        // switch (mode) {
        //     case USBD_MODE_CDC:
        //         return MP_OBJ_NEW_QSTR(MP_QSTR_VCP);
        //     case USBD_MODE_MSC:
                return MP_OBJ_NEW_QSTR(MP_QSTR_MSC);
        //     case USBD_MODE_HID:
        //         return MP_OBJ_NEW_QSTR(MP_QSTR_HID);
        //     case USBD_MODE_CDC_MSC:
        //         return MP_OBJ_NEW_QSTR(MP_QSTR_VCP_plus_MSC);
        //     case USBD_MODE_CDC_HID:
        //         return MP_OBJ_NEW_QSTR(MP_QSTR_VCP_plus_HID);
        //     case USBD_MODE_MSC_HID:
        //         return MP_OBJ_NEW_QSTR(MP_QSTR_MSC_plus_HID);
        //     default:
        //         return mp_const_none;
    #endif
    }

    // parse args
    mp_arg_val_t args[MP_ARRAY_SIZE(allowed_args)];
    mp_arg_parse_all(n_args, pos_args, kw_args, MP_ARRAY_SIZE(allowed_args), allowed_args, args);

    // record the fact that the usb has been explicitly configured
    pyb_usb_flags |= PYB_USB_FLAG_USB_MODE_CALLED;

    // check if user wants to disable the USB
    if (args[0].u_obj == mp_const_none) {
        // disable usb
        pyb_usb_dev_deinit();
        return mp_const_none;
    }

    // get mode string
    const char *mode_str = mp_obj_str_get_str(args[0].u_obj);


    // hardware configured for USB device mode
    usb_device_mode_t mode;
    
    if (strcmp(mode_str, "MSC") == 0) {
        if (args[2].u_int == -1) {
            //pid = USBD_PID_MSC;
        }
        mode = USBD_MODE_MSC;
    } else {
        goto bad_mode;
    }

    // init the USB device
    if (!pyb_usb_dev_init(mode)) {
        goto bad_mode;
    }

    return mp_const_none;

bad_mode:
    mp_raise_ValueError(MP_ERROR_TEXT("bad USB mode"));
}

MP_DEFINE_CONST_FUN_OBJ_KW(pyb_usb_mode_obj, 0, pyb_usb_mode);


// MP Class

typedef struct _pyb_usb_msc_obj_t {
    mp_obj_base_t base;
    usb_device_t *usb_dev;
} pyb_usb_msc_obj_t;

STATIC const pyb_usb_msc_obj_t pyb_usb_msc_obj = {{&pyb_usb_msc_type}, &mp_usb_device};

STATIC void pyb_usb_msc_print(const mp_print_t *print, mp_obj_t self_in, mp_print_kind_t kind) {
    mp_print_str(print, "USB_MSC()");
}

/// \classmethod \constructor()
/// Create a new USB MSC object
STATIC mp_obj_t pyb_usb_msc_make_new(const mp_obj_type_t *type, size_t n_args, size_t n_kw, const mp_obj_t *args) {
    // check arguments
    mp_arg_check_num(n_args, n_kw, 0, 0, false);

    // return the USB MSC object
    return (mp_obj_t)&pyb_usb_msc_obj;
}

STATIC const mp_rom_map_elem_t pyb_usb_locals_dict_table[] = {
    { MP_ROM_QSTR(MP_QSTR_usb_mode), MP_ROM_PTR(&pyb_usb_mode_obj) },
};

STATIC MP_DEFINE_CONST_DICT(pyb_usb_locals_dict, pyb_usb_locals_dict_table);

const mp_obj_type_t pyb_usb_msc_type = {
    { &mp_type_type },
    .name = MP_QSTR_USB,
    .make_new = pyb_usb_msc_make_new,
    .print = pyb_usb_msc_print,
    .locals_dict = (mp_obj_dict_t*)&pyb_usb_locals_dict,
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

#endif // MICROPY_HW_ENABLE_USB
