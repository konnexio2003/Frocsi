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


/* XDCtools Header files */
#include <xdc/std.h>
#include <xdc/cfg/global.h>
#include <xdc/runtime/Error.h>
#include <xdc/runtime/Memory.h>
#include <xdc/runtime/System.h>

/* BIOS Header files */
#include <ti/sysbios/BIOS.h>
#include <ti/sysbios/fatfs/ff.h>
#include <ti/sysbios/knl/Clock.h>
#include <ti/sysbios/knl/Task.h>
#include <ti/sysbios/knl/Semaphore.h>

/* NDK Header files */
#include <ti/ndk/inc/netmain.h>
#include <ti/ndk/inc/_stack.h>

/* TI-RTOS Header files */
#include <ti/drivers/GPIO.h>
#include <ti/drivers/SDSPI.h>

/* Example/Board Header files */
#include "Board.h"
#include "MSETCPprot.h"

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <ti/sysbios/gates/GateMutex.h>
#include "USBMSC.h"

#define TCPPACKETSIZE 1024
#define TCPPORT 1000
#define NUMTCPWORKERS 3
static Semaphore_Handle semTCP;
SOCKET clientfd;

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

	int nbytes;
	unsigned int testi;
	bool flag = true;
	char *buffer;
	Error_Block eb;
	pCommand_t commd;
	pOpenDir_t odircommand;
	pCloseDir_t cdircommand;
	pReadDir_t rdircommand;
	pStat_t statcommand;
	pOpen_t opencommand;
	pClose_t closecommand;
	pRead_t readcommand;

	pOpenDirA_t odiranswer;
	pCloseDirA_t cdiranswer;
	pReadDirA_t rdiranswer;
	pStatA_t statanswer;
	pOpenA_t openanswer;
	pCloseA_t closeanswer;
	pReadA_t readanswer;

	Task_sleep(10000);

	//printDrive(STR(DRIVE_NUM), &(src.fs));
    /* Make sure Error_Block is initialized */
    Error_init(&eb);

    /* Get a buffer to receive incoming packets. Use the default heap. */
    buffer = Memory_alloc(NULL, TCPPACKETSIZE, 0, &eb);
    if (buffer == NULL) {
        System_printf("tcpWorker: failed to alloc memory\n");
        Task_exit();
    }
    while (TRUE)
    	{
        Semaphore_pend(semTCP, BIOS_WAIT_FOREVER);
    	System_printf("tcpWorker: start clientfd = 0x%x\n", clientfd);

    	/* Loop while we receive data */
    		while (flag) {
    			fdOpenSession(TaskSelf());
    			nbytes = recv(clientfd, (char *)buffer, TCPPACKETSIZE, 0);
    			System_printf("bytes received: %d\n",nbytes);
    			if (nbytes > 0) {
    			commd = (pCommand_t)buffer;

    				switch (commd->command)
    				{
    				case OPENDIR:
    					//Receive COMMAND
    					odircommand = (pOpenDir_t)buffer;
    					System_printf("hossz: %d senderid: %d parancs: %d strtype %d dir: %s\n "
    							 ,ntohs(odircommand->lenght),ntohs(odircommand->senderid)
    							,odircommand->command,ntohl(odircommand->strtype), odircommand->path);
    					//Send Answer
    					odiranswer=(pOpenDirA_t)buffer;
    					odiranswer->lenght=htons(6);
    					odiranswer->senderid=htons(SENDERID);
    					odiranswer->fresult=htons(FR_OK);
    					testi=sizeof(OpenDirA_t);
    					send(clientfd, (char *)buffer, sizeof(OpenDirA_t), 0 );

						System_printf("hossz: %d senderid: %d fresult: %d\n "
								 ,ntohs(odiranswer->lenght ),ntohs(odiranswer->senderid)
								,ntohs(odiranswer->fresult));
    					break;
    				case CLOSEDIR:
						//Receive COMMAND
    					cdircommand = (pCloseDir_t)buffer;
						System_printf("hossz: %d senderid: %d parancs: %d \n "
								 ,ntohs(cdircommand->lenght ),ntohs(cdircommand->senderid)
								,cdircommand->command);
						//Send Answer
						cdiranswer=(pCloseDirA_t)buffer;
						cdiranswer->lenght=htons(6);
						cdiranswer->senderid=htons(SENDERID);
						cdiranswer->fresult=htons(FR_OK);
						send(clientfd, (char *)buffer, sizeof(CloseDirA_t), 0 );
						System_printf("hossz: %d senderid: %d fresult: %d\n "
								 ,ntohs(cdiranswer->lenght ),ntohs(cdiranswer->senderid)
								,cdiranswer->fresult);
						break;

    				case READDIR:
						//Receive COMMAND
    					rdircommand = (pReadDir_t)buffer;
						System_printf("hossz: %d senderid: %d parancs: %d \n "
								 ,ntohs(rdircommand->lenght ),ntohs(rdircommand->senderid)
								,rdircommand->command);
						//Send Answer
						rdiranswer=(pReadDirA_t)buffer;
						rdiranswer->lenght=htons(32);
						rdiranswer->senderid=htons(SENDERID);
						rdiranswer->fresult=htons(FR_OK);
						rdiranswer->fileinfo.fattrib=AM_DIR;
						rdiranswer->fileinfo.fdate=htons(123);
						strcpy(rdiranswer->fileinfo.fname,"file1234.txt\0");
						rdiranswer->fileinfo.fsize=htonl(234);
						rdiranswer->fileinfo.ftime=htons(456);
						testi=sizeof(ReadDirA_t);
						send(clientfd, (char *)buffer,sizeof(ReadDirA_t), 0 );
						System_printf("hossz: %d senderid: %d fresult: %d\n "
								 ,ntohs(rdiranswer->lenght ),ntohs(rdiranswer->senderid)
								,rdiranswer->fresult);
						break;
    				case STAT:
						//Receive COMMAND
						statcommand = (pStat_t)buffer;
						System_printf("hossz: %d senderid: %d parancs: %d strtype %d dir: %s\n "
								 ,ntohs(statcommand->lenght),ntohs(statcommand->senderid)
								,statcommand->command,ntohl(statcommand->strtype), statcommand->path);
						//Send Answer
						statanswer=(pStatA_t)buffer;
						statanswer->lenght=htons(6);
						statanswer->senderid=htons(SENDERID);
						statanswer->fresult=htons(FR_OK);
						statanswer->fileinfo.fattrib=AM_DIR;
						statanswer->fileinfo.fdate=htons(123);
						strcpy(statanswer->fileinfo.fname,"file1234.txt\0");
						statanswer->fileinfo.fsize=htonl(234);
						statanswer->fileinfo.ftime=htons(456);
						statanswer->fsize=htonl(234);
						send(clientfd, (char *)buffer, sizeof(StatA_t), 0 );

						System_printf("hossz: %d senderid: %d fresult: %d\n "
								 ,ntohs(statanswer->lenght ),ntohs(statanswer->senderid)
								,ntohs(statanswer->fresult));
						break;
    				case OPEN:
						//Receive COMMAND
						opencommand = (pOpen_t)buffer;
						System_printf("hossz: %d senderid: %d parancs: %d mode: %d date: %d time: %d path: %s \n "
								 ,ntohs(opencommand->lenght ),ntohs(opencommand->senderid)
								,opencommand->command,opencommand->mode, ntohs(opencommand->date)
								,ntohs(opencommand->time),opencommand->path );
						//Send Answer
						openanswer=(pOpenA_t)buffer;
						openanswer->lenght=htons(32);
						openanswer->senderid=htons(SENDERID);
						openanswer->fresult=htons(FR_OK);
						//testi=sizeof(OpenDirA_t);
						send(clientfd, (char *)buffer, sizeof(OpenA_t), 0 );
						System_printf("hossz: %d senderid: %d fresult: %d\n "
								 ,ntohs(openanswer->lenght ),ntohs(openanswer->senderid)
								,openanswer->fresult);
						break;
    				case CLOSE:
						//Receive COMMAND
						closecommand = (pClose_t)buffer;
						System_printf("hossz: %d senderid: %d parancs: %d \n "
								 ,ntohs(closecommand->lenght ),ntohs(closecommand->senderid)
								,closecommand->command);
						//Send Answer
						closeanswer=(pCloseA_t)buffer;
						closeanswer->lenght=htons(6);
						closeanswer->senderid=htons(SENDERID);
						closeanswer->fresult=htons(FR_OK);
						send(clientfd, (char *)buffer, sizeof(CloseA_t), 0 );
						System_printf("hossz: %d senderid: %d fresult: %d\n "
								 ,ntohs(closeanswer->lenght ),ntohs(closeanswer->senderid)
								,closeanswer->fresult);
						break;
    				case READ:
						//Receive COMMAND
						readcommand = (pRead_t)buffer;
						System_printf("hossz: %d senderid: %d parancs: %d from: %d count: %d \n "
								 ,ntohs(readcommand->lenght ),ntohs(readcommand->senderid)
								,readcommand->command,ntohs(readcommand->from), ntohs(readcommand->count));
						//Send Answer
						readanswer=(pReadA_t)buffer;
						readanswer->lenght=htons(32);
						readanswer->senderid=htons(SENDERID);
						readanswer->fresult=htons(FR_OK);
						readanswer->count=htons(21);
						strcpy(readanswer->data,"adat5678901234567890\0");
						testi=sizeof(readanswer);
						send(clientfd, (char *)buffer, sizeof(ReadA_t), 0 );
						System_printf("hossz: %d senderid: %d fresult: %d\n "
								 ,ntohs(readanswer->lenght ),ntohs(readanswer->senderid)
								,readanswer->fresult);
						break;

    				case TEST:
    				default:
    					{
    					odircommand = (pOpenDir_t)buffer;
    					//testi= odircommand->lenght;
    					System_printf("hossz: %d senderid:%d parancs: %d\n"
    					,ntohs(odircommand->lenght), ntohs(odircommand->senderid), odircommand->command);
    					/* Echo the data back */
    					send(clientfd, (char *)buffer, nbytes, 0 );
    					break;
    					}

    				}



    				 System_flush();
    			}
    			else {
    				fdClose(clientfd);
    				//flag = false;
    			}
    		}
    	System_printf("tcpWorker stop clientfd = 0x%x\n", clientfd);

    	/* Free the buffer back to the heap */

    	fdCloseSession(TaskSelf());
    	}
		//key = GateMutex_enter(gateUSBWait);
		//fresult = f_open(&src, outputfile, FA_READ);
		//fresult = f_lseek(&src, cursor);
		/*  Read from source file */
		//fresult = f_read(&src, cpy_buff, 512, &bytesRead);
		//f_close(&src);
		//GateMutex_leave(gateUSBWait, key);
		//cursor += bytesRead;
		//if (bytesRead == 0) // end of file
		//	cursor = 0;
		//Task_sleep(1000);



}
/*
 *  ======== tcpHandler ========
 *  Creates new Task to handle new TCP connections.
 */
Void tcpHandler(UArg arg0, UArg arg1)
{
    SOCKET lSocket;
    struct sockaddr_in sLocalAddr;
//    SOCKET clientfd;
    struct sockaddr_in client_addr;
    int addrlen=sizeof(client_addr);
    int optval;
    int optlen = sizeof(optval);
    int status;
    Error_Block eb;
    Semaphore_Params semParams;
    Semaphore_Params_init(&semParams);
    semParams.mode = Semaphore_Mode_BINARY;
    semTCP = Semaphore_create(0, &semParams, &eb);
    if (semTCP == NULL) {
       System_abort("Can't create msc semaphore");
       }

    fdOpenSession(TaskSelf());

    lSocket = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
    if (lSocket < 0) {
        System_printf("tcpHandler: socket failed\n");
        Task_exit();
        return;
    }

    memset((char *)&sLocalAddr, 0, sizeof(sLocalAddr));
    sLocalAddr.sin_family = AF_INET;
    sLocalAddr.sin_len = sizeof(sLocalAddr);
    sLocalAddr.sin_addr.s_addr = htonl(INADDR_ANY);
    sLocalAddr.sin_port = htons(1000);

    status = bind(lSocket, (struct sockaddr *)&sLocalAddr, sizeof(sLocalAddr));
    if (status < 0) {
        System_printf("tcpHandler: bind failed\n");
        fdClose(lSocket);
        Task_exit();
        return;
    }

    if (listen(lSocket, NUMTCPWORKERS) != 0){
        System_printf("tcpHandler: listen failed\n");
        fdClose(lSocket);
        Task_exit();
        return;
    }

    if (setsockopt(lSocket, SOL_SOCKET, SO_KEEPALIVE, &optval, optlen) < 0) {
        System_printf("tcpHandler: setsockopt failed\n");
        fdClose(lSocket);
        Task_exit();
        return;
    }

    while (true) {
        /* Wait for incoming request */
        clientfd = accept(lSocket, (struct sockaddr*)&client_addr, &addrlen);
        System_printf("tcpHandler: Creating thread clientfd = %d\n", clientfd);

        /* Init the Error_Block */
        Error_init(&eb);
        Semaphore_post(semTCP);
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
	    Board_initEMAC();
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
