#include <stm32f4xx.h>

//PA8---LASER

/*************************************************************************
��������LASER_Configuration
���ܣ��������ó�ʼ��
*************************************************************************/
void LASER_Configuration(void)
{
	GPIO_InitTypeDef gpio;   

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA,ENABLE);
		
	gpio.GPIO_Pin = GPIO_Pin_8;	
    gpio.GPIO_Mode = GPIO_Mode_OUT;
	gpio.GPIO_OType = GPIO_OType_PP;
	gpio.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_Init(GPIOA, &gpio);
	GPIO_SetBits(GPIOA,GPIO_Pin_8);
}

