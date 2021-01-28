#ifndef __UNDERPAN_TASK_H__
#define __UNDERPAN_TASK_H__

#include "USART1_Deal.h"
/*定义遥控器下高中低三种速度模式*/
#define HighSpeedMode 1
#define MidSpeedMode  0.5f
#define LowSpeedMode 0.25f

#define TankSpeedMax 600   //r/min

/*轮子地盘尺寸*/
#define L1 320   
#define L2 320
#define R 70.0f
#define PI 3.1416f

#define Mec_Weel_Coe 320/PI/R
#define Mec_Weel_Row (L1+L2)*320/360/R/10

/*底盘电机速度限制范围 EC60电机空载转速450r/min PID调试后测试为650*/
#define SpeedMax 650  
#define SpeedMin -650

/*遥控器接受速度分配结构体*/
typedef struct
{
    FP32 x;
	FP32 y;
	FP32 r;
}ST_VELT_DISTRIBUTE;

extern ST_VELT_DISTRIBUTE g_stVeltDistribute;

extern ST_MOTOR_CTRL g_stUndePan1;
extern ST_MOTOR_CTRL g_stUndePan2;
extern ST_MOTOR_CTRL g_stUndePan3;
extern ST_MOTOR_CTRL g_stUndePan4;




void Underpan_Control(void);
void Velt_Distribute(FP32 Vx,FP32 Vy,FP32 Vr,FP32 SpeedCoefficient);
void Send_Current_To_UnderPan(void);

#endif
