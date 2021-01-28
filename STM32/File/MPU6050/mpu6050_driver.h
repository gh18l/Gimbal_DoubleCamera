#ifndef __MPU6050_DRIVER_H__
#define __MPU6050_DRIVER_H__
#include "main.h"


typedef struct __MPU6050_RAW_Data__
{
    SSHORT16 Accel_X;  //�Ĵ���ԭ��X����ٶȱ�ʾֵ
    SSHORT16 Accel_Y;  //�Ĵ���ԭ��Y����ٶȱ�ʾֵ
    SSHORT16 Accel_Z;  //�Ĵ���ԭ��Z����ٶȱ�ʾֵ
    SSHORT16 Temp;     //�Ĵ���ԭ���¶ȱ�ʾֵ
    SSHORT16 Gyro_X;   //�Ĵ���ԭ��X�������Ǳ�ʾֵ
    SSHORT16 Gyro_Y;   //�Ĵ���ԭ��Y�������Ǳ�ʾֵ
    SSHORT16 Gyro_Z;   //�Ĵ���ԭ��Z�������Ǳ�ʾֵ
}MPU6050_RAW_DATA;

typedef struct __MPU6050_REAL_Data__
{
    FP32 Accel_X;  //ת����ʵ�ʵ�X����ٶȣ�
    FP32 Accel_Y;  //ת����ʵ�ʵ�Y����ٶȣ�
    FP32 Accel_Z;  //ת����ʵ�ʵ�Z����ٶȣ�
    FP32 Temp;     //ת����ʵ�ʵ��¶ȣ���λΪ���϶�
    FP32 Gyro_X;   //ת����ʵ�ʵ�X��Ǽ��ٶȣ�
    FP32 Gyro_Y;   //ת����ʵ�ʵ�Y��Ǽ��ٶȣ�
    FP32 Gyro_Z;   //ת����ʵ�ʵ�Z��Ǽ��ٶ�
}MPU6050_REAL_DATA;

extern MPU6050_RAW_DATA    MPU6050_Raw_Data; 
extern MPU6050_REAL_DATA   MPU6050_Real_Data;

SINT32 MPU6050_Initialization(void);
SINT32 MPU6050_ReadData(void);
void MPU6050_Gyro_Calibration(void);

#endif
