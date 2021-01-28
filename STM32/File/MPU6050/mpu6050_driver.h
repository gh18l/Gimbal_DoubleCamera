#ifndef __MPU6050_DRIVER_H__
#define __MPU6050_DRIVER_H__
#include "main.h"


typedef struct __MPU6050_RAW_Data__
{
    SSHORT16 Accel_X;  //寄存器原生X轴加速度表示值
    SSHORT16 Accel_Y;  //寄存器原生Y轴加速度表示值
    SSHORT16 Accel_Z;  //寄存器原生Z轴加速度表示值
    SSHORT16 Temp;     //寄存器原生温度表示值
    SSHORT16 Gyro_X;   //寄存器原生X轴陀螺仪表示值
    SSHORT16 Gyro_Y;   //寄存器原生Y轴陀螺仪表示值
    SSHORT16 Gyro_Z;   //寄存器原生Z轴陀螺仪表示值
}MPU6050_RAW_DATA;

typedef struct __MPU6050_REAL_Data__
{
    FP32 Accel_X;  //转换成实际的X轴加速度，
    FP32 Accel_Y;  //转换成实际的Y轴加速度，
    FP32 Accel_Z;  //转换成实际的Z轴加速度，
    FP32 Temp;     //转换成实际的温度，单位为摄氏度
    FP32 Gyro_X;   //转换成实际的X轴角加速度，
    FP32 Gyro_Y;   //转换成实际的Y轴角加速度，
    FP32 Gyro_Z;   //转换成实际的Z轴角加速度
}MPU6050_REAL_DATA;

extern MPU6050_RAW_DATA    MPU6050_Raw_Data; 
extern MPU6050_REAL_DATA   MPU6050_Real_Data;

SINT32 MPU6050_Initialization(void);
SINT32 MPU6050_ReadData(void);
void MPU6050_Gyro_Calibration(void);

#endif
