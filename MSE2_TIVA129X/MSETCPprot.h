/*
 * MSETCPprot.h
 *
 *  Created on: 2014.09.28.
 *      Author: marci
 */

#ifndef MSETCPPROT_H_
#define MSETCPPROT_H_
#include <ti/sysbios/fatfs/ff.h>

#define OPENDIR              255
#define CLOSEDIR             254
#define READDIR              253
#define STAT              	 252
#define OPEN              	 251
#define CLOSE	             250
#define READ                 249
#define MKDIR                248
#define WRITE                247
#define DELETE				 246
#define TEST				100

#define SENDERID			87

typedef struct _Command_t
{
uint16_t lenght;
uint16_t senderid;
uint8_t	 command;
}Command_t,*pCommand_t,CloseDir_t,*pCloseDir_t,ReadDir_t,*pReadDir_t,Close_t,*pClose_t;

typedef struct __attribute__((packed)) _OpenDirCommandType
{
uint16_t lenght;
uint16_t senderid;
uint8_t	 command;
uint32_t strtype;
uint8_t path[1024-9];

}OpenDir_t,*pOpenDir_t, Stat_t,*pStat_t,Delete_t,*pDelete_t;

typedef struct __attribute__((packed)) _OpenCommandType
{
uint16_t lenght;
uint16_t senderid;
uint8_t	 command;
uint8_t  mode;
uint16_t date;
uint16_t time;
uint8_t path[1024-10];

}Open_t,*pOpen_t;

typedef struct __attribute__((packed)) _ReadCommandType
{
uint16_t lenght;
uint16_t senderid;
uint8_t	 command;
uint16_t from;
uint16_t count;
}Read_t,*pRead_t;

typedef struct __attribute__((packed)) _MkDirCommandType
{
uint16_t lenght;
uint16_t senderid;
uint8_t	 command;
uint16_t date;
uint16_t time;
uint8_t path[1024-9];
}MkDir_t,*pMkDir_t;

typedef struct __attribute__((packed)) _WriteCommandType
{
uint16_t lenght;
uint16_t senderid;
uint8_t	 command;
uint16_t from;
uint16_t count;
uint16_t date;
uint16_t time;
uint8_t data[1024-13];
}Write_t,*pWrite_t;

typedef struct __attribute__((packed)) _OpenDirAnswerType
{
uint16_t lenght;
uint16_t senderid;
uint16_t fresult;
}OpenDirA_t,*pOpenDirA_t, CloseDirA_t,*pCloseDirA_t,OpenA_t,*pOpenA_t,CloseA_t,*pCloseA_t;

typedef struct __attribute__((packed)) _ReadDirAnswerType
{
uint16_t lenght;
uint16_t senderid;
uint16_t fresult;
FILINFO fileinfo;
uint32_t fsize;

}ReadDirA_t,*pReadDirA_t, StatA_t,*pStatA_t;//,OpenA_t,*pOpenA_t,CloseA_t,*pCloseA_t

typedef struct __attribute__((packed)) _ReadAnswerType
{
uint16_t lenght;
uint16_t senderid;
uint16_t fresult;
uint16_t count;
uint8_t data[1024-8];

}ReadA_t,*pReadA_t ;//, StatA_t,*pStatA_t;

typedef struct __attribute__((packed)) _MkDirAnswerType
{
uint16_t lenght;
uint16_t senderid;
uint16_t fresult;
FILINFO fileinfo;

}MkDirA_t,*pMkDirA_t;
typedef struct __attribute__((packed)) _WriteAnswerType
{
uint16_t lenght;
uint16_t senderid;
uint16_t fresult;
uint16_t count;

}WriteA_t,*pWriteA_t;


#endif /* MSETCPPROT_H_ */
