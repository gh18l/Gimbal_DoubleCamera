/*******************************************************************
版权声明： RoboMasters(HITCRT_iHiter 战队)
文件名：   RM_Types.h
最近修改日期： 2016/1/25
版本： 1.0
--------------------------------------------------------------------
模块描述：该模块申明常用的数据类型与通用的结构体。
--------------------------------------------------------------------
修改记录：
作者                              时间         版本   说明
彭季超、彭毓兴、张冰、李四林    2016/1/25     1.0     建立此文件
********************************************************************/
#ifndef __RM_TYPES_H__
#define	__RM_TYPES_H__

#define DEC			10
#define HEX			16


typedef unsigned char  		UCHAR8;                   /* defined for unsigned 8-bits integer variable 	无符号8位整型变量  */
typedef signed   char  		SCHAR8;                    /* defined for signed 8-bits integer variable		有符号8位整型变量  */
typedef unsigned short 		USHORT16;                  /* defined for unsigned 16-bits integer variable 	无符号16位整型变量 */
typedef signed   short 		SSHORT16;                   /* defined for signed 16-bits integer variable 		有符号16位整型变量 */
typedef unsigned int   		UINT32;                  /* defined for unsigned 32-bits integer variable 	无符号32位整型变量 */
typedef int   				SINT32;                   /* defined for signed 32-bits integer variable 		有符号32位整型变量 */
typedef float          		FP32;                    /* single precision floating point variable (32bits) 单精度浮点数（32位长度） */
typedef double         		DB64;                    /* double precision floating point variable (64bits) 双精度浮点数（64位长度） */

#endif
