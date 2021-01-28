#include <stm32f4xx.h>

//��ȡ�������η�����̬ʱ��
void TIM7_Configuration(void)
{
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
	NVIC_InitTypeDef        NVIC_InitStructure;
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM7,ENABLE);  ///ʹ��TIM7ʱ��
	
  	TIM_TimeBaseInitStructure.TIM_Period = 20-1; 	//�Զ���װ��ֵ    10us
	TIM_TimeBaseInitStructure.TIM_Prescaler= 42-1;  //��ʱ����Ƶ  2MHz
	TIM_TimeBaseInitStructure.TIM_CounterMode=TIM_CounterMode_Up; //���ϼ���ģʽ
	TIM_TimeBaseInitStructure.TIM_ClockDivision=TIM_CKD_DIV1; 
	
	TIM_TimeBaseInit(TIM7,&TIM_TimeBaseInitStructure);
	
	TIM_ITConfig(TIM7,TIM_IT_Update,ENABLE); 	 
	
	NVIC_InitStructure.NVIC_IRQChannel=TIM7_IRQn; 
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=1; 
	NVIC_InitStructure.NVIC_IRQChannelSubPriority=1; 
	NVIC_InitStructure.NVIC_IRQChannelCmd=ENABLE;
	NVIC_Init(&NVIC_InitStructure);	
    TIM_Cmd(TIM7,ENABLE);
}


