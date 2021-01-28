#include "mpu6050_i2c.h"
#include "mpu6050_interrupt.h"
#include "mpu6050_driver.h"
#include "Delay.h"


//定义MPU6050内部地址
#define	SMPLRT_DIV		0x19	//陀螺仪采样率 典型值 0X07 125Hz
#define	CONFIG			0x1A	//低通滤波频率 典型值 0x00 
#define	GYRO_CONFIG		0x1B	//陀螺仪自检及测量范围                 典型值 0x18 不自检 2000deg/s
#define	ACCEL_CONFIG	0x1C	//加速度计自检及测量范围及高通滤波频率 典型值 0x01 不自检 2G 5Hz

#define INT_PIN_CFG     0x37
#define INT_ENABLE      0x38
#define INT_STATUS      0x3A    //只读


#define	ACCEL_XOUT_H	0x3B
#define	ACCEL_XOUT_L	0x3C

#define	ACCEL_YOUT_H	0x3D
#define	ACCEL_YOUT_L	0x3E

#define	ACCEL_ZOUT_H	0x3F
#define	ACCEL_ZOUT_L	0x40

#define	TEMP_OUT_H		0x41
#define	TEMP_OUT_L		0x42

#define	GYRO_XOUT_H		0x43
#define	GYRO_XOUT_L		0x44	

#define	GYRO_YOUT_H		0x45
#define	GYRO_YOUT_L		0x46

#define	GYRO_ZOUT_H		0x47
#define	GYRO_ZOUT_L		0x48

#define	PWR_MGMT_1		0x6B	//电源管理 典型值 0x00 正常启用
#define	WHO_AM_I		0x75	//只读  默认读出应该是 MPU6050_ID = 0x68


#define MPU6050_ID              0x68
#define MPU6050_DEVICE_ADDRESS  0xD0
#define MPU6050_DATA_START      ACCEL_XOUT_H   //由于数据存放地址是连续的，所以一并读出

MPU6050_RAW_DATA    MPU6050_Raw_Data; 
MPU6050_REAL_DATA   MPU6050_Real_Data;

SINT32 gyroADC_X_offset=0,gyroADC_Y_offset=0,gyroADC_Z_offset=0;

//MPU6050 初始化，成功返回0  失败返回 0xff
SINT32 MPU6050_Initialization(void)
{
    UCHAR8 temp_data = 0x00;

    IIC_GPIO_Init();  //初始化IIC接口
    
    if(IIC_ReadData(MPU6050_DEVICE_ADDRESS,WHO_AM_I ,&temp_data ,1)==0) //确定IIC总线上挂接的是否是MPU6050
    {
        if(temp_data != MPU6050_ID)
        {
            return 0xff; //校验失败，返回0xff
        }
    }
    else
    {
        return 0xff; //读取失败 返回0xff
    }
    
    if(IIC_WriteData(MPU6050_DEVICE_ADDRESS,PWR_MGMT_1,0x01) == 0xff)    //解除休眠状态
    {
        return 0xff;
    }
    if(IIC_WriteData(MPU6050_DEVICE_ADDRESS,SMPLRT_DIV,0x07) == 0xff)//cyq：07 更新频率1khz
    {
        return 0xff;
    }
    if(IIC_WriteData(MPU6050_DEVICE_ADDRESS,CONFIG,0x00) == 0xff)
    {
        return 0xff;
    }
    if(IIC_WriteData(MPU6050_DEVICE_ADDRESS,GYRO_CONFIG,0x08) == 0xff)
    {
        return 0xff;
    }
    if(IIC_WriteData(MPU6050_DEVICE_ADDRESS,ACCEL_CONFIG,0x08) == 0xff)
    {
        return 0xff;
    }
    if(IIC_WriteData(MPU6050_DEVICE_ADDRESS,INT_PIN_CFG,0x00) == 0xff)
    {
        return 0xff;
    }
    if(IIC_WriteData(MPU6050_DEVICE_ADDRESS,INT_ENABLE,0x01) == 0xff)
    {
        return 0xff;
    }
	//Debug
   // MPU6050_Interrupt_Configuration();    
    return 0;
}

//MPU6050  数据读取，成功返回0  失败返回 0xff
SINT32 MPU6050_ReadData(void)
{
    u8 buf[14];
    
    if(IIC_ReadData(MPU6050_DEVICE_ADDRESS,MPU6050_DATA_START,buf,14) == 0xff)
    {
        return 0xff;
    }
    else
    {
        //读取寄存器原生数据
           
        MPU6050_Raw_Data.Accel_X = (buf[0]<<8 | buf[1]);
        MPU6050_Raw_Data.Accel_Y = (buf[2]<<8 | buf[3]);
        MPU6050_Raw_Data.Accel_Z = (buf[4]<<8 | buf[5]); 
        MPU6050_Raw_Data.Temp =    (buf[6]<<8 | buf[7]);  
        MPU6050_Raw_Data.Gyro_X = (buf[8]<<8 | buf[9]);
        MPU6050_Raw_Data.Gyro_Y = (buf[10]<<8 | buf[11]);
        MPU6050_Raw_Data.Gyro_Z = (buf[12]<<8 | buf[13]);
       
        //将原生数据转换为实际值，计算公式跟寄存器的配置有关
        MPU6050_Real_Data.Accel_X = (FP32)(MPU6050_Raw_Data.Accel_X)/8192.0f; //见datasheet 30 of 47
        MPU6050_Real_Data.Accel_Y = (FP32)(MPU6050_Raw_Data.Accel_Y)/8192.0f; //见datasheet 30 of 47
        MPU6050_Real_Data.Accel_Z = (FP32)(MPU6050_Raw_Data.Accel_Z)/8192.0f; //见datasheet 30 of 47
        MPU6050_Real_Data.Temp =   (FP32)(MPU6050_Raw_Data.Temp)/340.0f+36.53f;//见datasheet 31 of 47
        MPU6050_Real_Data.Gyro_X = (FP32)(MPU6050_Raw_Data.Gyro_X - gyroADC_X_offset)/65.5f;     //见datasheet 32 of 47
        MPU6050_Real_Data.Gyro_Y = (FP32)(MPU6050_Raw_Data.Gyro_Y - gyroADC_Y_offset)/65.5f;     //见datasheet 32 of 47
        MPU6050_Real_Data.Gyro_Z = (FP32)(MPU6050_Raw_Data.Gyro_Z - gyroADC_Z_offset)/65.5f;     //见datasheet 32 of 47
    } 
        
    return 0;
}


void MPU6050_Gyro_Calibration(void)
{
	u16 i;
	FP32 x_temp=0,y_temp=0,z_temp=0;
	
	for(i=0; i<1000; i++)
	{
		MPU6050_ReadData();
		delay_ms(1);
		x_temp=x_temp+MPU6050_Raw_Data.Gyro_X;
		y_temp=y_temp+MPU6050_Raw_Data.Gyro_Y;
		z_temp=z_temp+MPU6050_Raw_Data.Gyro_Z;
	}			
	
	x_temp=x_temp/1000.00f;
	y_temp=y_temp/1000.00f;	
	z_temp=z_temp/1000.00f;

	gyroADC_X_offset=(SINT32)x_temp;
	gyroADC_Y_offset=(SINT32)y_temp;
	gyroADC_Z_offset=(SINT32)z_temp;
}
