#include <stm32f4xx.h>

/*TIM4_CH1---PB6 ÖÐ¶Ï */
void TIM4_Configuration(void)
{
    TIM_TimeBaseInitTypeDef  TIM_InitStructure;
    NVIC_InitTypeDef         NVIC_InitStructure;

    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4,ENABLE);

    NVIC_InitStructure.NVIC_IRQChannel = TIM4_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    TIM_InitStructure.TIM_Prescaler = 42-1;
    TIM_InitStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_InitStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_InitStructure.TIM_Period = 1000-1; //0.5ms
    TIM_TimeBaseInit(TIM4,&TIM_InitStructure);

    TIM_Cmd(TIM4, ENABLE);	 
    TIM_ITConfig(TIM4, TIM_IT_Update,ENABLE);
    TIM_ClearFlag(TIM4, TIM_FLAG_Update);
}

