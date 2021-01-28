#include "Flash.h"
#include "stm32f4xx_flash.h"

UCHAR8 g_ucFlashFlag;
UCHAR8 g_ucFlashWrongNum;
SINT32 g_siFlashValue[MAX_FLASH_LEN];

//读取一个16位数据
SSHORT16 GetOneShort16(UINT32 uiReadAddr)
{
	SSHORT16 ssTemp;
	ssTemp = (SSHORT16)(*(__IO SINT32*)(ADDR_FLASH_SECTOR_11 + (uiReadAddr << 2)));
	return ssTemp;
}

//读取一个32位数据
SINT32 GetOneInt32(UINT32 uiReadAddr)
{
	SINT32 ssTemp;
	ssTemp = *(__IO SINT32*)(ADDR_FLASH_SECTOR_11 + (uiReadAddr << 2));
	return ssTemp;
}

//向指定地址写入一个32位数据
UCHAR8 SaveOneWord32(SINT32 siData, UINT32 uiWriteAddr)
{
	UCHAR8 ucRet = 0; 
	UINT32 i = 0;
	/* Unlock the Flash to enable the flash control register access *************/ 
	FLASH_Unlock();
	//写之前先读取数据
	for(i = 0;i < MAX_FLASH_LEN;i ++)
	{
		g_siFlashValue[i] = *(__IO SINT32*)(ADDR_FLASH_SECTOR_11 + (i << 2));
	}
  
	/* Clear pending flags (if any) */  
	FLASH_ClearFlag(FLASH_FLAG_EOP | FLASH_FLAG_OPERR | FLASH_FLAG_WRPERR | 
	              FLASH_FLAG_PGAERR | FLASH_FLAG_PGPERR | FLASH_FLAG_PGSERR); 


	/* Device voltage range supposed to be [2.7V to 3.6V], the operation will
	       be done by word */ 
	if(FLASH_EraseSector(FLASH_Sector_11, VoltageRange_3) != FLASH_COMPLETE)
	{ 
		 /* Error occurred while sector erase. 
		     User can add here some code to deal with this error  */
		return 0;
    }

	g_siFlashValue[uiWriteAddr] = siData;

	for(i = 0;i < MAX_FLASH_LEN;i ++)
	{
		if(FLASH_ProgramWord(ADDR_FLASH_SECTOR_11 + (i << 2), g_siFlashValue[i]) == FLASH_COMPLETE)
		 {
		     ucRet = 0x01;
		 }  
		 else
		 {
		 	 ucRet = 0x00;
			 g_ucFlashWrongNum++;
		 }
	}

   /* Lock the Flash to disable the flash control register access (recommended
     to protect the FLASH memory against possible unwanted operation) *********/
    FLASH_Lock(); 
	g_ucFlashFlag = ucRet;
	return ucRet;
}

//从指定地址开始写入一个32位数据的数组
UCHAR8 SaveParaWord32(UINT32 uiLength, UINT32 uiFlashAddr, SINT32 *p)
{
	UCHAR8 ucRet = 0; 
	UINT32 i = 0;

	/* Unlock the Flash to enable the flash control register access *************/ 
	FLASH_Unlock();
	//写入之前先读flash
	for(i = 0;i < MAX_FLASH_LEN;i ++)
	{
		g_siFlashValue[i] = *(__IO SINT32*)(ADDR_FLASH_SECTOR_11 + (i << 2));
	}
  
	/* Clear pending flags (if any) */  
	FLASH_ClearFlag(FLASH_FLAG_EOP | FLASH_FLAG_OPERR | FLASH_FLAG_WRPERR | 
	              FLASH_FLAG_PGAERR | FLASH_FLAG_PGPERR|FLASH_FLAG_PGSERR); 


	    /* Device voltage range supposed to be [2.7V to 3.6V], the operation will
	       be done by word */ 
   if(FLASH_EraseSector(FLASH_Sector_11, VoltageRange_3) != FLASH_COMPLETE)
    { 
      /* Error occurred while sector erase. 
         User can add here some code to deal with this error  */
      	return 0;
    }

	for(i = 0;i < uiLength;i ++)
	{
		g_siFlashValue[uiFlashAddr + i] = p[i];
	}

	for(i = 0;i < MAX_FLASH_LEN;i ++)
	{
		if(FLASH_ProgramWord(ADDR_FLASH_SECTOR_11 + (i << 2), g_siFlashValue[i]) == FLASH_COMPLETE)
	    {
	      	
			ucRet = 0x01;
	    }  
	    else
	    {
	      	ucRet = 0x00;
			g_ucFlashWrongNum++;
	    }
	}

   /* Lock the Flash to disable the flash control register access (recommended
     to protect the FLASH memory against possible unwanted operation) *********/
    FLASH_Lock(); 
	g_ucFlashFlag = ucRet;
	return ucRet;
}
