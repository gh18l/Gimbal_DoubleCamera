#ifndef __UNDERPAN_TASK_H__
#define __UNDERPAN_TASK_H__

#include "USART1_Deal.h"
/*����ң�����¸��е������ٶ�ģʽ*/
#define HighSpeedMode 1
#define MidSpeedMode  0.5f
#define LowSpeedMode 0.25f

#define TankSpeedMax 600   //r/min

/*���ӵ��̳ߴ�*/
#define L1 320   
#define L2 320
#define R 70.0f
#define PI 3.1416f

#define Mec_Weel_Coe 320/PI/R
#define Mec_Weel_Row (L1+L2)*320/360/R/10

/*���̵���ٶ����Ʒ�Χ EC60�������ת��450r/min PID���Ժ����Ϊ650*/
#define SpeedMax 650  
#define SpeedMin -650

/*ң���������ٶȷ���ṹ��*/
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
