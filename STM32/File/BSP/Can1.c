#include <stm32f4xx.h>
#include "stm32f4xx_can.h"

//CAN_TX---PA12(CANTX) 
//CAN_RX---PA11(CANRX) 

/*************************************************************************
函数名：CAN1_Configuration
功能：初始化CAN1配置为1M波特率
*************************************************************************/

void CAN1_Configuration(void)
{
    CAN_InitTypeDef        CAN1_InitStructure;
    GPIO_InitTypeDef       GPIO_InitStructure;
    NVIC_InitTypeDef       NVIC_InitStructure;
    CAN_FilterInitTypeDef  CAN1_FilterInitStructure;

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN1, ENABLE);

    GPIO_PinAFConfig(GPIOA, GPIO_PinSource11, GPIO_AF_CAN1);
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource12, GPIO_AF_CAN1);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11 | GPIO_Pin_12;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
    
    NVIC_InitStructure.NVIC_IRQChannel = CAN1_RX0_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
    
    NVIC_InitStructure.NVIC_IRQChannel = CAN1_TX_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure); 
    
    CAN_DeInit(CAN1);
    CAN_StructInit(&CAN1_InitStructure);
    
    CAN1_InitStructure.CAN_TTCM = DISABLE;
    CAN1_InitStructure.CAN_ABOM = DISABLE;
    CAN1_InitStructure.CAN_AWUM = DISABLE;
    CAN1_InitStructure.CAN_NART = DISABLE;
    CAN1_InitStructure.CAN_RFLM = DISABLE;
    CAN1_InitStructure.CAN_TXFP = ENABLE;
    CAN1_InitStructure.CAN_Mode = CAN_Mode_Normal;
    CAN1_InitStructure.CAN_SJW  = CAN_SJW_1tq;
    CAN1_InitStructure.CAN_BS1 = CAN_BS1_9tq;
    CAN1_InitStructure.CAN_BS2 = CAN_BS2_4tq;
    CAN1_InitStructure.CAN_Prescaler = 3;   //CAN1_InitStructure BaudRate 42/(1+9+4)/3=1Mbps
    CAN_Init(CAN1, &CAN1_InitStructure);

    CAN1_FilterInitStructure.CAN_FilterNumber=0;
    CAN1_FilterInitStructure.CAN_FilterMode=CAN_FilterMode_IdMask;
    CAN1_FilterInitStructure.CAN_FilterScale=CAN_FilterScale_32bit;
    CAN1_FilterInitStructure.CAN_FilterIdHigh=0x0000;
    CAN1_FilterInitStructure.CAN_FilterIdLow=0x0000;
    CAN1_FilterInitStructure.CAN_FilterMaskIdHigh=0x0000;
    CAN1_FilterInitStructure.CAN_FilterMaskIdLow=0x0000;
    CAN1_FilterInitStructure.CAN_FilterFIFOAssignment=0;//the message which pass the filter save in fifo0
    CAN1_FilterInitStructure.CAN_FilterActivation=ENABLE;
    CAN_FilterInit(&CAN1_FilterInitStructure);
    
    CAN_ITConfig(CAN1,CAN_IT_FMP0,ENABLE);
    CAN_ITConfig(CAN1,CAN_IT_TME,ENABLE); 
}

