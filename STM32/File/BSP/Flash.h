#ifndef _FLASH_H
#define _FLASH_H

#include <stm32f4xx.h>
#include "main.h"

#define  MAX_FLASH_LEN   400

extern UCHAR8 g_ucFlashFlag;
extern UCHAR8 g_ucFlashWrongNum;
extern SINT32 g_siFlashValue[MAX_FLASH_LEN];

/* Base address of the Flash sectors */
#define ADDR_FLASH_SECTOR_0     ((uint32_t)0x08000000) /* Base @ of Sector 0, 16 Kbytes */
#define ADDR_FLASH_SECTOR_1     ((uint32_t)0x08004000) /* Base @ of Sector 1, 16 Kbytes */
#define ADDR_FLASH_SECTOR_2     ((uint32_t)0x08008000) /* Base @ of Sector 2, 16 Kbytes */
#define ADDR_FLASH_SECTOR_3     ((uint32_t)0x0800C000) /* Base @ of Sector 3, 16 Kbytes */
#define ADDR_FLASH_SECTOR_4     ((uint32_t)0x08010000) /* Base @ of Sector 4, 64 Kbytes */
#define ADDR_FLASH_SECTOR_5     ((uint32_t)0x08020000) /* Base @ of Sector 5, 128 Kbytes */
#define ADDR_FLASH_SECTOR_6     ((uint32_t)0x08040000) /* Base @ of Sector 6, 128 Kbytes */
#define ADDR_FLASH_SECTOR_7     ((uint32_t)0x08060000) /* Base @ of Sector 7, 128 Kbytes */
#define ADDR_FLASH_SECTOR_8     ((uint32_t)0x08080000) /* Base @ of Sector 8, 128 Kbytes */
#define ADDR_FLASH_SECTOR_9     ((uint32_t)0x080A0000) /* Base @ of Sector 9, 128 Kbytes */
#define ADDR_FLASH_SECTOR_10    ((uint32_t)0x080C0000) /* Base @ of Sector 10, 128 Kbytes */
#define ADDR_FLASH_SECTOR_11    ((uint32_t)0x080E0000) /* Base @ of Sector 11, 128 Kbytes */


#define SAVE_DATA_START_ADDR 	  	ADDR_FLASH_SECTOR_11

SSHORT16 GetOneShort16(UINT32 uiReadAddr);//读取一个16位数据
SINT32 GetOneInt32(UINT32 uiReadAddr);//读取一个32位数据
UCHAR8 SaveOneWord32(SINT32 siData, UINT32 uiWriteAddr);//向指定地址写入一个32位数据
UCHAR8 SaveParaWord32(UINT32 uiLength, UINT32 uiFlashAddr, SINT32 *p);//从指定地址开始写入一组32位数据

#endif
