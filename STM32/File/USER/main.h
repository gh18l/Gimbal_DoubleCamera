#ifndef __MAIN_H__
#define __MAIN_H__

/********************************************************
**功能：看是否是编译C++，因为C++里面是有布尔类型的变量的，
**      所以如果不是编译C++，才能定义布尔变量
**备注：必须放在"main.h"最开头，因为文件从最开始编译
*********************************************************/
#ifndef __cplusplus
typedef enum {FALSE = 0, TRUE = !FALSE} bool;
#endif

#include "RM_Types.h"
#include "RM_MotorCtrlTypes.h"


#define abs(x) ((x)>0? (x):(-(x)))


#endif 









