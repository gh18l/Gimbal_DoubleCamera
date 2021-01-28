#include <stm32f4xx.h>

/*************************************************************************
函数名：SysTick_Configuration
功能：初始化SysTick_Configuration配置为1ms中断
*************************************************************************/
void SysTick_Configuration(void)
{
	
    //SYSTICK分频--1ms的系统时钟中断
    //SystemCoreClock / 1000     1ms
    //SystemCoreClock / 100000   10us
    //SystemCoreClock / 1000000  1us
	if (SysTick_Config(SystemCoreClock / 1000))
  	{ 
  	  	/* Capture error */ 
    	while (1);
  	}
}
