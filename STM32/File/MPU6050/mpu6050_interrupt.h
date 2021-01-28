#ifndef __MPU6050_INTERRUPT_H__
#define __MPU6050_INTERRUPT_H__
#include <stm32f4xx.h>

#include "main.h"

void MPU6050_Interrupt_Configuration(void);
extern FP32 target_pitch_angle;
extern FP32 target_yaw_angle;
extern FP32 this_203_angle;
extern FP32 velocity_203_output;  //yaw轴速度环函数的输出值
extern FP32 position_203_output;
#endif
