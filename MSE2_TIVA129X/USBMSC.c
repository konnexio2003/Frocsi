/*
 * Copyright (c) 2014, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
/*
 * Fordít OK
 * Debug
 * Teszt
 * Közben ir olvas
 * Ha kell fordít újra sdcard kezelő
 * gat mutex melyik?
 * tcp/ip
 *
 * DEBUG disk műveletekre és teszt
 *
 */
 /*
 *  ======== USBKBD.c ========
 */

/* XDCtools Header files */
#include <xdc/std.h>
#include <xdc/runtime/Error.h>
#include <xdc/runtime/System.h>

/* BIOS Header files */
#include <ti/sysbios/BIOS.h>
#include <ti/sysbios/gates/GateMutex.h>
#include <ti/sysbios/hal/Hwi.h>
#include <ti/sysbios/knl/Semaphore.h>

#include <stdbool.h>
#include <stdint.h>

/* driverlib Header files */
#include <inc/hw_ints.h>
#include <inc/hw_types.h>

/* usblib Header files */
#include <usblib/usb-ids.h>
#include <usblib/usblib.h>
#include <usblib/usbmsc.h>
#include <usblib/device/usbdevice.h>
#include <usblib/device/usbdmsc.h>
#include <ti/sysbios/fatfs/diskio.h>
/* Board Header files */
#include "USBMSC.h"
#include "Board.h"
#include <ti/drivers/SDSPI.h>
#include <ti/sysbios/fatfs/ff.h>
#define SDCARD_PRESENT          0x00000001
#define SDCARD_IN_USE           0x00000002
/* Drive number used for FatFs */
#define DRIVE_NUM           0
struct
{
    uint32_t ui32Flags;
}
g_sDriveInformation;

#if defined(TIVAWARE)
typedef uint32_t            USBMSCEventType;
#else
#define eUSBModeForceDevice USB_MODE_FORCE_DEVICE
typedef unsigned long       USBKBDEventType;
#endif

/* Typedefs */
typedef volatile enum {
	  //
	    // Unconfigured.
	    //
	    MSC_DEV_DISCONNECTED,

	    //
	    // Connected and fully enumerated but not currently handling a command.
	    //
	    MSC_DEV_IDLE,

	    //
	    // Currently reading the device.
	    //
	    MSC_DEV_READ,

	    //
	    // Currently writing the device.
	    //
	    MSC_DEV_WRITE,
} USBMSC_USBState;
FATFS FatFs;   /* Work area (file system object) for logical drive */
/* Static variables and handles */
static volatile USBMSC_USBState state;
static Semaphore_Handle semMSC;
static Semaphore_Handle semUSBConnected;
static int wren=0;

/* Function prototypes */
static USBMSCEventType cbMSCHandler(void *cbData, USBMSCEventType event,
		                                 USBMSCEventType eventMsg,
                                         void *eventMsgPtr);
static Void USBMSC_hwiHandler(UArg arg0);
//static int sendChar(int ch, unsigned int timeout);
//static bool waitUntilSent(unsigned int timeout);
void USB_getState(USBMSC_USBState *usbState);
void USBMSC_Init(void);
//int USBKBD_putChar(int ch, unsigned int timeout);
//int USBKBD_putString(String chArray, unsigned int length, unsigned int timeout);
bool USBMSC_waitForConnect(unsigned int timeout);

/* The languages supported by this device. */
const unsigned char langDescriptor[] =
{
    4,
    USB_DTYPE_STRING,
    USBShort(USB_LANG_EN_US)
};

/* The manufacturer string. */
const unsigned char manufacturerString[] =
{
    (17 + 1) * 2,
    USB_DTYPE_STRING,
    'T', 0, 'e', 0, 'x', 0, 'a', 0, 's', 0, ' ', 0, 'I', 0, 'n', 0, 's', 0,
    't', 0, 'r', 0, 'u', 0, 'm', 0, 'e', 0, 'n', 0, 't', 0, 's', 0,
};

/* The product string. */
const unsigned char productString[] =
{
	(19 + 1) * 2,
	USB_DTYPE_STRING,
	'M', 0, 'a', 0, 's', 0, 's', 0, ' ', 0, 'S', 0, 't', 0, 'o', 0, 'r', 0,
	'a', 0, 'g', 0, 'e', 0, ' ', 0, 'D', 0, 'e', 0, 'v', 0, 'i', 0, 'c', 0,
	'e', 0
};

/* The serial number string. */
const unsigned char serialNumberString[] =
{
	(8 + 1) * 2,
	USB_DTYPE_STRING,
	'1', 0, '2', 0, '3', 0, '4', 0, '5', 0, '6', 0, '7', 0, '8', 0
};

/* The interface description string. */
const unsigned char mscInterfaceString[] =
{
	    (19 + 1) * 2,
	    USB_DTYPE_STRING,
	    'B', 0, 'u', 0, 'l', 0, 'k', 0, ' ', 0, 'D', 0, 'a', 0, 't', 0,
	    'a', 0, ' ', 0, 'I', 0, 'n', 0, 't', 0, 'e', 0, 'r', 0, 'f', 0,
	    'a', 0, 'c', 0, 'e', 0
};

/* The configuration description string. */
const unsigned char configString[] =
{
	    (23 + 1) * 2,
	    USB_DTYPE_STRING,
	    'B', 0, 'u', 0, 'l', 0, 'k', 0, ' ', 0, 'D', 0, 'a', 0, 't', 0,
	    'a', 0, ' ', 0, 'C', 0, 'o', 0, 'n', 0, 'f', 0, 'i', 0, 'g', 0,
	    'u', 0, 'r', 0, 'a', 0, 't', 0, 'i', 0, 'o', 0, 'n', 0
};

/* The descriptor string table. */
const unsigned char * const stringDescriptors[] =
{
    langDescriptor,
    manufacturerString,
    productString,
    serialNumberString,
    mscInterfaceString,
    configString
};

#define STRINGDESCRIPTORSCOUNT (sizeof(stringDescriptors) / \
                                sizeof(unsigned char *))

#if defined(TIVAWARE)
static tUSBDMSCDevice mscDevice =
{
	USB_VID_TI_1CBE,
	USB_PID_MSC,
	"TI      ",
	"Mass Storage    ",
	"1.00",
    500,
    USB_CONF_ATTR_SELF_PWR,
    stringDescriptors,
    STRINGDESCRIPTORSCOUNT,
        {
            USBDMSCStorageOpen,
            USBDMSCStorageClose,
            USBDMSCStorageRead,
            USBDMSCStorageWrite,
            USBDMSCStorageNumBlocks,
            0 //USBDMSCStorageBlockSize ? this is the good value?
        },
        cbMSCHandler
};
#else  /* MWARE */
static tHIDKeyboardInstance keyboardInstance;
const tUSBDHIDKeyboardDevice keyboardDevice =
{
    USB_VID_TI,
    USB_PID_KEYBOARD,
    500,
    USB_CONF_ATTR_SELF_PWR | USB_CONF_ATTR_RWAKE,
    cbKeyboardHandler,
    NULL,
    stringDescriptors,
    STRINGDESCRIPTORSCOUNT,
    &keyboardInstance /* Old usblib stores a pointer */
};
#endif


/*
 *  ======== USBDMSCEventCallback ========
 *  Callback handler for the USB stack.
 *
 *  Callback handler call by the USB stack to notify us on what has happened in
 *  regards to the MSC.
 *
 *  @param(cbData)          A callback pointer provided by the client.
 *
 *  @param(event)           Identifies the event that occurred in regards to
 *                          this device.
 *
 *  @param(eventMsgData)    A data value associated with a particular event.
 *
 *  @param(eventMsgPtr)     A data pointer associated with a particular event.
 *
 */
static USBMSCEventType cbMSCHandler(void *cbData, USBMSCEventType event,
         USBMSCEventType eventMsg,
         void *eventMsgPtr)

{
int test=0;
    /* Determine what event has happened */
    switch (event) {
        case USB_EVENT_CONNECTED:
            state = MSC_DEV_IDLE;
            Semaphore_post(semUSBConnected);
            break;

        case USB_EVENT_DISCONNECTED:
            state = MSC_DEV_DISCONNECTED;
            break;

        case USBD_MSC_EVENT_READING:
            state = MSC_DEV_READ;
            break;

        case USBD_MSC_EVENT_WRITING:
            state = MSC_DEV_WRITE;
           // Semaphore_post(semKeyboard);
            break;

        case USBD_MSC_EVENT_IDLE:
        default:
        {
        	test++;
            break;

        }
    }
if (test>0)
	;
    return (0);
}

/*
 *  ======== USBMSC_hwiHandler ========
 *  This function calls the USB library's device interrupt handler.
 */
static Void USBMSC_hwiHandler(UArg arg0)
{
    USB0DeviceIntHandler();
}

/*
 *  ======== USBDMSC_init ========
 */
void USBMSC_Init(void)
 {
//	uint_fast32_t ui32Retcode;
    Hwi_Handle hwi;
    Error_Block eb;
    Semaphore_Params semParams;

    Error_init(&eb);

    /* Install interrupt handler */
    hwi = Hwi_create(INT_USB0, USBMSC_hwiHandler, NULL, &eb);
    if (hwi == NULL) {
        System_abort("Can't create USB Hwi");
    }

    /* RTOS primitives */
    Semaphore_Params_init(&semParams);
    semParams.mode = Semaphore_Mode_BINARY;
    semMSC = Semaphore_create(0, &semParams, &eb);
    if (semMSC == NULL) {
        System_abort("Can't create msc semaphore");
    }

    semUSBConnected = Semaphore_create(0, &semParams, &eb);
    if (semUSBConnected == NULL) {
        System_abort("Can't create MSC semaphore");
    }

    gateMSC = GateMutex_create(NULL, &eb);
    if (gateMSC == NULL) {
        System_abort("Can't create MSC gate");
    }

    gateUSBWait = GateMutex_create(NULL, &eb);
    if (gateUSBWait == NULL) {
        System_abort("Could not create USB Wait gate");
    }

    /* State specific variables */
    state = MSC_DEV_DISCONNECTED;

    /* Set the USB stack mode to Device mode with VBUS monitoring */
    USBStackModeSet(0, eUSBModeDevice, 0);

    /*
     * Pass our device information to the USB msc device class driver,
     * initialize the USB controller and connect the device to the bus.
     */
    if (!USBDMSCInit(0, (tUSBDMSCDevice*)&mscDevice)) {
        System_abort("Error initializing the msc");
    }
   /* ui32Retcode = */disk_initialize(0);

}

/*
 *  ======== USBMSC_waitForConnect ========
 */
bool USBMSC_waitForConnect(unsigned int timeout)
{
    bool ret = true;
    unsigned int key;

    /* Need exclusive access to prevent a race condition */
    key = GateMutex_enter(gateUSBWait);

    if (state == MSC_DEV_DISCONNECTED) {
        if (!Semaphore_pend(semUSBConnected, timeout)) {
            ret = false;
        }
    }

    GateMutex_leave(gateUSBWait, key);

    return (ret);
}
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
USBDMSCStorageOpen(uint_fast32_t ui32Drive)
{

    FRESULT fresult;
    SDSPI_Handle sdspiHandle;
    SDSPI_Params sdspiParams;
    unsigned int key;
    uint_fast32_t ui32Temp;

    //ASSERT(ui32Drive == 0);

    //
    // Return if already in use.
    //
    if(g_sDriveInformation.ui32Flags & SDCARD_IN_USE)
    {
        return(0);
    }
    /* Mount and register the SD Card */
        SDSPI_Params_init(&sdspiParams);
        sdspiHandle = SDSPI_open(Board_SDSPI0, DRIVE_NUM, &sdspiParams);
        if (sdspiHandle == NULL) {
            System_abort("Error starting the SD card\n");
        }
        else {
            System_printf("Drive %u is mounted\n", DRIVE_NUM);
        }
    //
    // Initialize the drive if it is present.
    //

    key = GateMutex_enter(gateUSBWait);
    ui32Temp = disk_initialize(0);
	GateMutex_leave(gateUSBWait, key);
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
USBDMSCStorageClose(void * pvDrive)
{
    uint_fast8_t ui8Power;
    unsigned int key;
    //ASSERT(pvDrive != 0);

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
    key = GateMutex_enter(gateUSBWait);
    disk_ioctl(0, CTRL_POWER, &ui8Power);
	GateMutex_leave(gateUSBWait, key);
	System_printf("MSC Storage close /n");
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
uint32_t USBDMSCStorageRead(void * pvDrive,
                                 uint8_t *pui8Data,
                                 uint_fast32_t ui32Sector,
                                 uint_fast32_t ui32NumBlocks)
{
	unsigned int key;
    //ASSERT(pvDrive != 0);
	key = GateMutex_enter(gateUSBWait);
    if(disk_read (0, pui8Data, ui32Sector, ui32NumBlocks) == RES_OK)
    {
    	GateMutex_leave(gateUSBWait, key);
        // TODO remove fixed 512
if (wren)
    	System_printf("Read Sector: %d Blocks:%d\n", ui32Sector, ui32NumBlocks) ;
        return(ui32NumBlocks * 512);
    }
    GateMutex_leave(gateUSBWait, key);
    System_printf("Read Sector: 0 reading\n") ;
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
uint32_t USBDMSCStorageWrite(void * pvDrive,
                                  uint8_t *pui8Data,
                                  uint_fast32_t ui32Sector,
                                  uint_fast32_t ui32NumBlocks)
{
static	int i=0;
unsigned int key,key1;
wren=1;
    //ASSERT(pvDrive != 0);
   char line[82]; /* Line buffer */
   FRESULT fr;    /* FatFs return code */
   key1 = GateMutex_enter(gateMSC);
   key = GateMutex_enter(gateUSBWait);

    if(disk_write(0, pui8Data, ui32Sector, ui32NumBlocks) == RES_OK)
    {
    	GateMutex_leave(gateUSBWait, key);
    System_printf("Write ** Sector: %d Blocks:%d\n", ui32Sector, ui32NumBlocks) ;
	if ((ui32Sector >9437 && ui32Sector < 17007) || (ui32Sector==8193))
	{
		GateMutex_leave(gateMSC, key1);
		System_printf("Gate Leave..\n") ;
	}
    return(ui32NumBlocks * 512);
    }
    GateMutex_leave(gateUSBWait, key);
    if ((ui32Sector >9437 && ui32Sector < 17007) || (ui32Sector==8193))
        {
        	GateMutex_leave(gateMSC, key1);
        	 System_printf("Gate Leave out\n") ;
        }
    System_printf("Write 0 success\n") ;
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
USBDMSCStorageNumBlocks(void * pvDrive)
{
    uint_fast32_t ui32SectorCount;
    unsigned int key;
    //
    // Read the number of sectors.
    //
    key = GateMutex_enter(gateUSBWait);
    disk_ioctl(0, GET_SECTOR_COUNT, &ui32SectorCount);
    System_printf("MSC Numblocks read: %d \n",ui32SectorCount ) ;
    GateMutex_leave(gateUSBWait, key);
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
uint32_t USBDMSCStorageStatus(void * pvDrive);
