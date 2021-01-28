#include "main.h"
#include "LED.h"

/*----LED_GREEN----PC1-----'0' is on,'1' is off */
/*----LED_RED------PC2-----'0' is on,'1' is off */

void LED_Configuration(void)
{
    GPIO_InitTypeDef gpio;
    
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC,ENABLE);
	
	gpio.GPIO_Pin = GPIO_Pin_1 | GPIO_Pin_2;
	gpio.GPIO_Mode = GPIO_Mode_OUT;
	gpio.GPIO_OType = GPIO_OType_PP;
	gpio.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_Init(GPIOC,&gpio);
    
    LED_GREEN_OFF();
    LED_RED_OFF();
}


