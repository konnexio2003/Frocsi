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
 *  ======== fatsdraw.c ========
 */

/* XDCtools Header files */
#include <xdc/std.h>
#include <xdc/cfg/global.h>
#include <xdc/runtime/Error.h>
#include <xdc/runtime/System.h>

/* BIOS Header files */
#include <ti/sysbios/BIOS.h>
#include <ti/sysbios/fatfs/ff.h>
#include <ti/sysbios/knl/Clock.h>

/* TI-RTOS Header files */
#include <ti/drivers/GPIO.h>
#include <ti/drivers/SDSPI.h>

/* Example/Board Header files */
#include "Board.h"

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <ti/sysbios/gates/GateMutex.h>
#include "USBMSC.h"

/* Buffer size used for the file copy process */
#ifndef CPY_BUFF_SIZE
#define CPY_BUFF_SIZE       600
#endif

/* String conversion macro */
#define STR_(n)             #n
#define STR(n)              STR_(n)

/* Drive number used for FatFs */
#define DRIVE_NUM           0

const char  inputfile[] = STR(DRIVE_NUM)":input.txt";
const char outputfile[] = STR(DRIVE_NUM)":output.txt";

const char textarray[] = \
"***********************************************************************\n"
"0         1         2         3         4         5         6         7\n"
"01234567890123456789012345678901234567890123456789012345678901234567890\n"
"This is some text to be inserted into the inputfile if there isn't     \n"
"already an existing file located on the SDCard.                        \n"
"If an inputfile already exists, or if the file was already once        \n"
"generated, then the inputfile will NOT be modified.                    \n"
"***********************************************************************\n";
unsigned char cpy_buff[512 + 1];
FIL src;
FIL dst;
GateMutex_Handle gateMSC;
GateMutex_Handle gateUSBWait;
/*
 *  ======== printDrive ========
 *  Function to print drive information such as the total disk space
 *  This function was created by referencing FatFs's API documentation
 *  http://elm-chan.org/fsw/ff/en/getfree.html
 *
 *  This function call may take a while to process, depending on the size of
 *  SD Card used.
 */
void printDrive(const char *driveNumber, FATFS **fatfs)
{
    FRESULT        fresult;
    DWORD          freeClusterCount;
    DWORD          totalSectorCount;
    DWORD          freeSectorCount;

    System_printf("Reading disk information...");
    System_flush();

    fresult = f_getfree(driveNumber, &freeClusterCount, fatfs);
    if (fresult) {
        System_abort("Error getting the free cluster count from the FatFs object");
    }
    else {
        System_printf("done\n");

        /* Get total sectors and free sectors */
        totalSectorCount = ((*fatfs)->n_fatent - 2) * (*fatfs)->csize;
        freeSectorCount  = freeClusterCount * (*fatfs)->csize;

        /* Print the free space (assuming 512 bytes/sector) */
        System_printf("Total Disk size: %10lu KiB\n"
                      "Free Disk space: %10lu KiB\n",
                      totalSectorCount / 2,
                      freeSectorCount  / 2);
    }
}
/*
 *  ========IPtaskFxn ========
 *  Task to IP sending
 *
 *
 *  Task for this function is created statically. See the project's .cfg file.
 */
Void IPTaskFxn(UArg arg0, UArg arg1)
{
	unsigned int key;
    unsigned int bytesRead = 0;
    unsigned int bytesWritten = 0;
    unsigned int filesize;
    unsigned int cursor=0;
	FRESULT        fresult;
	Task_sleep(10000);
	while (true)
	{
		key = GateMutex_enter(gateUSBWait);
		fresult = f_open(&src, outputfile, FA_READ);
		fresult = f_lseek(&src, cursor);
		/*  Read from source file */
		fresult = f_read(&src, cpy_buff, 512, &bytesRead);
		f_close(&src);
		GateMutex_leave(gateUSBWait, key);
		cursor += bytesRead;
		if (bytesRead == 0) // end of file
			cursor = 0;
		Task_sleep(1000);
	}


}
/*
 *  ======== MSCtaskFxn ========
 *  Task to USB connection and reconnection
 *
 *
 *  Task for this function is created statically. See the project's .cfg file.
 */
Void MSCtaskFxn(UArg arg0, UArg arg1)
{

	 while (true) {

	        /* Block while the device is NOT connected to the USB */
		 	USBMSC_waitForConnect(BIOS_WAIT_FOREVER);

		 	Task_sleep(100);

	        }



   // BIOS_exit(0);
}

/*
 *  ======== main ========
 */
int main(void)
{
    /* Call board init functions */
    Board_initGeneral();
    Board_initGPIO();
    Board_initSDSPI();
    Board_initUSB(Board_USBDEVICE);
    /* Turn on user LED */
    GPIO_write(Board_LED0, Board_LED_ON);

    System_printf("Starting the USB MSC program\nSystem provider "
                  "is set to SysMin. Halt the target to view any SysMin"
                  " contents in ROV.\n");
    System_flush();
    USBMSC_Init();

    /* Start BIOS */
    BIOS_start();

    return (0);
}
