#ifndef __MPU6050_H__
#define __MPU6050_H__

#include <stm32f4xx.h>

#include "main.h"

void IIC_GPIO_Init(void);
SINT32 IIC_WriteData(u8 dev_addr,u8 reg_addr,u8 data);
SINT32 IIC_ReadData(u8 dev_addr,u8 reg_addr,u8 *pdata,u8 count);

#endif
