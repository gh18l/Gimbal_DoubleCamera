#include <stm32f4xx.h>
#include "main.h"
#include "Delay.h"
/*-----USART1_RX-----PB7----*/
/*-----DMA2 - Stream2 - Channel4*/
volatile UCHAR8 sbus_rx_buffer[18] = {0};

void USART1_Configuration(void)
{
    USART_InitTypeDef USART1_InitStructure;
	GPIO_InitTypeDef  GPIO_InitStructure;
    NVIC_InitTypeDef  NVIC_InitStructure;
    DMA_InitTypeDef   DMA_InitStructure;
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB | RCC_AHB1Periph_DMA2,ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1,ENABLE);
	
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource7 ,GPIO_AF_USART1);
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7 ;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOB,&GPIO_InitStructure);
      
    USART_DeInit(USART1);
	USART1_InitStructure.USART_BaudRate = 100000;   //SBUS 100K baudrate
	USART1_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART1_InitStructure.USART_StopBits = USART_StopBits_1;
	USART1_InitStructure.USART_Parity = USART_Parity_Even;
	USART1_InitStructure.USART_Mode = USART_Mode_Rx;
    USART1_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_Init(USART1,&USART1_InitStructure);
    
	USART_Cmd(USART1,ENABLE);
    USART_DMACmd(USART1,USART_DMAReq_Rx,ENABLE);
    
    NVIC_InitStructure.NVIC_IRQChannel = DMA2_Stream2_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
    
    DMA_DeInit(DMA2_Stream2);
    DMA_InitStructure.DMA_Channel= DMA_Channel_4;
		//外设地址
    DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&(USART1->DR);
		//内存地址
    DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)sbus_rx_buffer;//将串口2接收到的数据存在sbus_rx_buffer[]里
    DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;
		//设置DMA在传输区的长度
    DMA_InitStructure.DMA_BufferSize = 18;
		//DMA传输为单向
    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
    DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
    DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
    DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
    DMA_InitStructure.DMA_Priority = DMA_Priority_VeryHigh;
    DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;
    DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_1QuarterFull;
    DMA_InitStructure.DMA_MemoryBurst = DMA_Mode_Normal;
    DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
    DMA_Init(DMA2_Stream2,&DMA_InitStructure);

    DMA_ITConfig(DMA2_Stream2,DMA_IT_TC,ENABLE);
    DMA_Cmd(DMA2_Stream2,ENABLE);
}

