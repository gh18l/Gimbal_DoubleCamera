#ifndef _DATAINIT_H
#define _DATAINIT_H
#include "main.h"
#include "Flash.h"

typedef union
{
	SINT32 siRead;
	FP32 fpRead;
}UN_READ;

/*Êý¾Ý¶ÁÈ¡ÃüÁî*/
#define ReadInt(Data)   unRead.siRead = GetOneInt32(ucReadPit); Data = unRead.siRead; ucReadPit++;
#define ReadFp(Data)	unRead.siRead = GetOneInt32(ucReadPit); Data = unRead.fpRead; ucReadPit++; 


void GlobalDataInit(void);

#endif
