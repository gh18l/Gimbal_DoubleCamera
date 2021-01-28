#include <stm32f4xx.h>
#include "Led.h"
#include "Delay.h"
#include "Can1.h"
#include "Can2.h"
#include "Usart1.h"
#include "TIM7.h"
#include "SysTick.h"
#include "TIM4.h"
#include "USART3.h"
#include "DataInit.h"
#include "mpu6050_driver.h"
#include "mpu6050_interrupt.h"

/*************************************************************************
函数名：BSP_Configuration
功能：初始化所有底层配置
*************************************************************************/
void BSP_Configuration(void)
{
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
    LED_Configuration();
    CAN1_Configuration();
    CAN2_Configuration();
    delay_ms(1000);
    USART1_Configuration();
	USART3_Configuration();	   //蓝牙调试
    TIM7_Configuration();
    TIM4_Configuration();
    SysTick_Configuration();

	while (MPU6050_Initialization() == 0xff);
    MPU6050_Gyro_Calibration();
	MPU6050_Interrupt_Configuration(); //MPU6050中断初始化
	/*放到最后面*/
	//GlobalDataInit();
}

