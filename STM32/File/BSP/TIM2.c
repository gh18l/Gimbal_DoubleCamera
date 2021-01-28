#include <stm32f4xx.h>
#include "main.h"

//TIM2_CH3---PA2  ²¦µ¯µç»úPWMÊä³ö

void TIM2_Configuration(void)
{
    GPIO_InitTypeDef          GPIO_InitStructure;
    TIM_TimeBaseInitTypeDef   TIM;
    TIM_OCInitTypeDef         OC;
    
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA,ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
   
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_Init(GPIOA,&GPIO_InitStructure);
    
    GPIO_PinAFConfig(GPIOA,GPIO_PinSource2,GPIO_AF_TIM2);
       
    TIM.TIM_Prescaler = 0;
    TIM.TIM_CounterMode = TIM_CounterMode_Up;
    TIM.TIM_Period = 4200-1;   //84MHz/4200 =  20KHz 
    TIM.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseInit(TIM2,&TIM);
    
    OC.TIM_OCMode = TIM_OCMode_PWM2;
    OC.TIM_OutputState = TIM_OutputState_Enable;
    OC.TIM_OutputNState = TIM_OutputState_Disable;
    OC.TIM_Pulse = 0;
    OC.TIM_OCPolarity = TIM_OCPolarity_Low;
    OC.TIM_OCNPolarity = TIM_OCPolarity_High;
    OC.TIM_OCIdleState = TIM_OCIdleState_Reset;
    OC.TIM_OCNIdleState = TIM_OCIdleState_Set;

    TIM_OC3Init(TIM2,&OC);

    TIM_OC3PreloadConfig(TIM2,TIM_OCPreload_Enable);
           
    TIM_ARRPreloadConfig(TIM2,ENABLE);
    
    TIM_CtrlPWMOutputs(TIM2,ENABLE);

    TIM_Cmd(TIM2,ENABLE);
}

