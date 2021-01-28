#include "main.h"
#include <stm32f4xx.h>
#include "RS232.h"
#include "Gimbal_Task.h"


/***********************����3*****************************/
UCHAR8 Comm3RxBuf[COMM3_RX_BUF_SIZE];//����3������飬��Сȡ����֡����
volatile UCHAR8 Comm3GetData = 0;//�����Ƿ���Ч

static UCHAR8 Comm3TxBuf[COMM3_TX_BUF_MAX];

volatile USHORT16 Comm3TxHead = 0;//����ͷ-���յ�ʱ���ƶ�
volatile SSHORT16 Comm3TxTail = 0;//����β-��ȡ��ʱ���ƶ�

void Comm3Rx_IRQ(void)
{
	static UCHAR8 Comm3_Rx_Status = COMM3_RX_FREE;
	static UCHAR8 ucPit = 0;//��
	static UCHAR8 ucDataLength = 0;
	UCHAR8 ucData;

	ucData = USART_ReceiveData(USART3);
	
	switch(Comm3_Rx_Status)
	{
		case COMM3_RX_FREE:
			if(ucData == 0x55)
			{																  
//				Comm3GetData = 0;//��������������Ч�����������ݸ��ǻ���
				if(Comm3GetData == 0)//��������
				{
					Comm3_Rx_Status = COMM3_RX_START;//����״̬�½ӵ�0x55��Ϊ��ʼ
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
				if(ucData <= 0x12 && ucData >= 0x02)//���ݳ�����ȷ
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
				if(ucPit < ucDataLength)//���û��������
				{
					*(Comm3RxBuf + ucPit) = ucData;
					ucPit++;
				}
				else//�������ж�0x00
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
				if(ucData == 0xAA)//����ӵ���0xAA��������Ч		
				{
					Comm3GetData = 1;//������Ч
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
	//�жϷ��Ͷ����Ƿ�Ϊ�գ��ǿ�����
		USART_SendData(USART3, Comm3TxBuf[Comm3TxTail]);
		Comm3TxTail = (Comm3TxTail + 1) & COMM3_TX_BUF_MARK;
	}
	else
	{
		USART_ITConfig(USART3, USART_IT_TXE, DISABLE);	
	}
}

//�����жϺ�����ʹ�ܷ��Ϳ��ж�
void Comm3Tx_Start(void)
{
	USART_ITConfig(USART3, USART_IT_TXE, ENABLE);
}

//��һ���ֽڷ��뷢�Ͷ�����
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

//�ж϶����Ƿ�Ϊ��
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
