#ifndef __GIMBAL_TASK_H__
#define __GIMBAL_TASK_H__

#include <stm32f4xx.h>
#include "main.h"

#define ST_PITCH_POSPID_INIT {2.5,0.005,0,0,0,0,0,0,5000,-5000,0,0,0} 
#define ST_PITCH_VELTPID_INIT {4.2,0.001,0,0,0,0,0,0,5000,-5000,0,0,0}
#define ST_YAW_POSPID_INIT {1,0,0,0,0,0,0,0,0,0,0,0,0}
#define ST_YAW_VELTPID_INIT {10,0,0,0,0,0,0,0,0,0,0,0,0}

extern ST_MOTOR_CTRL g_stPitch;
extern ST_MOTOR_CTRL g_stYaw;


void Gimbal_Control(void);
void Send_Current_To_Gimbal(void);
#endif 

