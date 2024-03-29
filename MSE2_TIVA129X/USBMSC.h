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
 *  ======== USBKBD.h ========
 */

#ifndef USBMSC_H_
#define USBMSC_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <xdc/std.h>
#include <ti/sysbios/gates/GateMutex.h>

//*****************************************************************************
//
// The externally provided mass storage class event call back function.
//
//*****************************************************************************
extern uint32_t USBDMSCEventCallback(void *pvCBData, uint32_t ui32Event,
                                     uint32_t ui32MsgParam,
                                     void *pvMsgData);

/*!
 *  ======== USBMSC_init ========
 *  Function to initialize the USB mouse reference module.
 *
 *  Note: This function is not reentrant safe.
 */
extern void USBMSC_Init(void);
extern void * USBDMSCStorageOpen(uint32_t ui32Drive);
extern void USBDMSCStorageClose(void * pvDrive);
extern uint32_t USBDMSCStorageRead(void * pvDrive, uint8_t *pui8Data,
                                        uint32_t ui32Sector,
                                        uint32_t ui32NumBlocks);
extern uint32_t USBDMSCStorageWrite(void * pvDrive, uint8_t *pui8Data,
                                         uint32_t ui32Sector,
                                         uint32_t ui32NumBlocks);
uint32_t USBDMSCStorageNumBlocks(void * pvDrive);

extern bool USBMSC_waitForConnect(unsigned int timeout);
//extern bool USBKBD_waitForConnect(unsigned int timeout);
extern  GateMutex_Handle gateMSC;
extern  GateMutex_Handle gateUSBWait;

#ifdef __cplusplus
}
#endif

#endif /* USBMSC_H_ */
