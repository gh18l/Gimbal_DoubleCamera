#include <stm32f4xx.h>
#include "stm32f4xx_can.h"

//CAN2_TX-----PB13
//CAN2_RX-----PB12

/*************************************************************************
函数名：CAN2_Configuration
功能：初始化CAN2配置为1M波特率
*************************************************************************/

void CAN2_Configuration(void)
{
    CAN_InitTypeDef        CAN2_InitStructure;
    GPIO_InitTypeDef       GPIO_InitStructure;
    NVIC_InitTypeDef       NVIC_InitStructure;
    CAN_FilterInitTypeDef  CAN2_FilterInitStructure;

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN1, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN2, ENABLE);

    GPIO_PinAFConfig(GPIOB, GPIO_PinSource12, GPIO_AF_CAN2);
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource13, GPIO_AF_CAN2); 

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12 | GPIO_Pin_13 ;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    NVIC_InitStructure.NVIC_IRQChannel = CAN2_RX0_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    CAN_DeInit(CAN2);
    CAN_StructInit(&CAN2_InitStructure);

    CAN2_InitStructure.CAN_TTCM = DISABLE;
    CAN2_InitStructure.CAN_ABOM = DISABLE;    
    CAN2_InitStructure.CAN_AWUM = DISABLE;    
    CAN2_InitStructure.CAN_NART = DISABLE;    
    CAN2_InitStructure.CAN_RFLM = DISABLE;    
    CAN2_InitStructure.CAN_TXFP = ENABLE;     
    CAN2_InitStructure.CAN_Mode = CAN_Mode_Normal; 
    CAN2_InitStructure.CAN_SJW  = CAN_SJW_1tq;
    CAN2_InitStructure.CAN_BS1 = CAN_BS1_9tq;
    CAN2_InitStructure.CAN_BS2 = CAN_BS2_4tq;
    CAN2_InitStructure.CAN_Prescaler = 3;   //CAN2 BaudRate 42/(1+9+4)/3=1Mbps
    CAN_Init(CAN2, &CAN2_InitStructure);
    
    CAN2_FilterInitStructure.CAN_FilterNumber=14;
    CAN2_FilterInitStructure.CAN_FilterMode=CAN_FilterMode_IdMask;
    CAN2_FilterInitStructure.CAN_FilterScale=CAN_FilterScale_32bit;
    CAN2_FilterInitStructure.CAN_FilterIdHigh=0x0000;
    CAN2_FilterInitStructure.CAN_FilterIdLow=0x0000;
    CAN2_FilterInitStructure.CAN_FilterMaskIdHigh=0x0000;
    CAN2_FilterInitStructure.CAN_FilterMaskIdLow=0x0000;
    CAN2_FilterInitStructure.CAN_FilterFIFOAssignment=0;//the message which pass the filter save in fifo0
    CAN2_FilterInitStructure.CAN_FilterActivation=ENABLE;
    CAN_FilterInit(&CAN2_FilterInitStructure);
    
    CAN_ITConfig(CAN2,CAN_IT_FMP0,ENABLE);
}

