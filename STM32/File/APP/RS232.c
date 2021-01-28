#include "main.h"
#include <stm32f4xx.h>
#include "RS232.h"
#include "Gimbal_Task.h"


/***********************串口3*****************************/
UCHAR8 Comm3RxBuf[COMM3_RX_BUF_SIZE];//串口3读缓冲块，大小取决于帧长度
volatile UCHAR8 Comm3GetData = 0;//数据是否有效

static UCHAR8 Comm3TxBuf[COMM3_TX_BUF_MAX];

volatile USHORT16 Comm3TxHead = 0;//队列头-接收的时候移动
volatile SSHORT16 Comm3TxTail = 0;//队列尾-读取的时候移动

void Comm3Rx_IRQ(void)
{
	static UCHAR8 Comm3_Rx_Status = COMM3_RX_FREE;
	static UCHAR8 ucPit = 0;//坑
	static UCHAR8 ucDataLength = 0;
	UCHAR8 ucData;

	ucData = USART_ReceiveData(USART3);
	
	switch(Comm3_Rx_Status)
	{
		case COMM3_RX_FREE:
			if(ucData == 0x55)
			{																  
//				Comm3GetData = 0;//缓冲区里数据无效化，快来数据覆盖机制
				if(Comm3GetData == 0)//堵塞机制
				{
					Comm3_Rx_Status = COMM3_RX_START;//自由状态下接到0x55认为开始
					ucPit = 0;	
				}
			}
			break;
		case COMM3_RX_START:
			if(ucPit != 2)
			{
				*(Comm3RxBuf + ucPit) = ucData;
				ucPit++;
			}
			else
			{
				if(ucData <= 0x12 && ucData >= 0x02)//数据长度正确
				{
					ucDataLength = ucData;
					Comm3_Rx_Status = COMM3_RX_DATA; 
				}
				else
				{
					Comm3_Rx_Status = COMM3_RX_FREE;
				} 	
			}
			break;
		case COMM3_RX_DATA:
			{
				if(ucPit < ucDataLength)//如果没够数，存
				{
					*(Comm3RxBuf + ucPit) = ucData;
					ucPit++;
				}
				else//够数了判断0x00
				{
					if(ucData == 0x00)
					{
						Comm3_Rx_Status = COMM3_RX_END;
					}
					else
					{
						Comm3_Rx_Status = COMM3_RX_FREE;
					}
				}
			}
			break;
		case COMM3_RX_END:
			{
				if(ucData == 0xAA)//如果接到了0xAA，数据有效		
				{
					Comm3GetData = 1;//数据有效
				}
				Comm3_Rx_Status = COMM3_RX_FREE;

				g_stYaw.fpPosDes = (FP32)(Comm3RxBuf[3]<<24 | Comm3RxBuf[4]<<16 | Comm3RxBuf[5]<<8 | Comm3RxBuf[6]);
			}
			break;
		default:
			break;
	}
}

void Comm3Tx_IRQ(void)
{
	if(Comm3IsDataInTxBuf())
	{
	//判断发送队列是否为空，非空则发送
		USART_SendData(USART3, Comm3TxBuf[Comm3TxTail]);
		Comm3TxTail = (Comm3TxTail + 1) & COMM3_TX_BUF_MARK;
	}
	else
	{
		USART_ITConfig(USART3, USART_IT_TXE, DISABLE);	
	}
}

//发送中断函数，使能发送空中断
void Comm3Tx_Start(void)
{
	USART_ITConfig(USART3, USART_IT_TXE, ENABLE);
}

//将一个字节放入发送队列中
UCHAR8 Comm3PutDataToTxBuf(UCHAR8 Data)
{
	SSHORT16 tmpHead;
	tmpHead = (Comm3TxHead + 1) & COMM3_TX_BUF_MARK;
	if(tmpHead == Comm3TxTail)
	{
		return 0;
	}
	Comm3TxBuf[Comm3TxHead] = Data;
	Comm3TxHead = tmpHead;	
	Comm3Tx_Start();
	return 1;
}

//判断队列是否为空
UCHAR8 Comm3IsDataInTxBuf(void)
{
	if(Comm3TxHead == Comm3TxTail)
	{
		return 0;
	}
	else
	{
		return 1;
	}
}
