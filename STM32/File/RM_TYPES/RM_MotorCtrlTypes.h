#ifndef __RM_MOTORCTRLTYPES_H__
#define __RM_MOTORCTRLTYPES_H__
#include <stm32f4xx.h>
#include "RM_Types.h"

/*驱动运行状态*/
typedef enum
{
	FIRST,		    //首次运行
	RUN		    	//已经运行
}EM_RUN_STATE;      

/*驱动器状态*/
typedef enum
{
	OFF,
	ON
}EM_DRIVER_STATE;
 
/*控制模式*/ 
typedef enum
{
    OPEN_LOOP,     //开环控制
	POS_LOOP,      //位置控制
	VELT_LOOP,     //速度控制
	CURRENT_LOOP,  //电流控制
}EM_CTRL_MODE;

/*加/解锁*/
typedef enum
{
	UNLOCK,	    	//解锁
	LOCK		    //上锁		  
}EM_LOCK_STATE;	    

/*电机相关的参数*/
typedef struct
{
	FP32     Acc;       //加速度,单位: r/s^2
    FP32     I;         //减速比
	USHORT16 NE_ABS;    //绝对式码盘线数
    USHORT16 NE_INC;    //增量式码盘线数
	USHORT16 MaxI;      //最大电流,单位mA
	USHORT16 CtnI;      //最大连续工作电流
    USHORT16 MaxV;      //最高电压限制
	USHORT16 MinV;      //最低电压限制
}ST_MOTOR_PARA;

/*定义PID参数数据结构*/
typedef struct 
{
	FP32 KP;		    //比例系数KP
	FP32 KI;		    //积分系数KI
	FP32 KD;		    //微分系数KD
	FP32 E;			  	//本次偏差
	FP32 PreE;		  	//上次偏差
	FP32 PrePreE;		//上上次偏差
    FP32 SumE;          //总偏差
	FP32 U;	  			//本次PID运算结果
    SINT32 UMax;        //PID输出最大限制
    SINT32 UMin;        //PID输出最小限制
	SINT32 ELimit;      //做积分分离运算时的极限偏差
	SINT32 ULimit;		//做遇限削弱时的上限值
	SINT32 EDead;			//死区间隔
}ST_PID;     

/*码盘结构体*/
typedef struct
{
    SINT32 siCode;   	    //当前码盘读数
	SINT32 siPreCode;	    //上一次码盘读数
	SINT32 siDiffCode;  	//码盘读数差值
	SINT32 siCycleCode;     //转过的圈数
}ST_CODER;

typedef struct
{
	UCHAR8 SendPosFlag		:1;
	UCHAR8 SendVeltFlag	    :1;
	UCHAR8 SendCrtFlag		:1;
}ST_SEND_FLAG;

/*核心控制结构体*/
typedef struct
{
	UINT32          SlaveAddr;       //从动机地址
	UINT32          MasterAddr;      //主动机地址
	EM_RUN_STATE    emRunState;      //驱动运动状态
	EM_DRIVER_STATE emDriverState;   //驱动器状态
	EM_CTRL_MODE    emCtrlMode;      //电机控制模式
	EM_LOCK_STATE   emLockState;     //驱动上锁状态

	ST_MOTOR_PARA   stMotorPara;     //电机参数
    ST_CODER        stCoder;         //码盘结构体
	ST_PID          stPosPID;        //位置PID
	ST_PID          stVeltPID;       //速度PID
	ST_PID          stCrtPID;        //电流PID

    FP32            fpPosDes;        //目标位置
    FP32            fpPosFB;         //位置反馈
    FP32            fpVeltDes;       //目标速度
	FP32            fpVeltFB;        //速度反馈
    SINT32          siCrtDes;        //目标电流
	SINT32          siCrtFB;         //电流反馈

    SINT32          MaxPos;          //位置环输入最大位置限制
	SINT32          MinPos;          //位置环输入最小位置限制
	SINT32          MaxVelt;         //速度环输入最大速度限制
	SINT32          MinVelt;         //速度环输入最小速度限制
    SINT32          MaxCrt;          //速度环输入最大速度限制
	SINT32          MinCrt;          //速度环输入最小速度限制
}ST_MOTOR_CTRL;

typedef enum 
{
	DEBUG_MODE,
	NORMAL_MODE
}EN_WORK_MODE;

#endif
