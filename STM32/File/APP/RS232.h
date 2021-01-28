#ifndef _RS232_H
#define _RS232_H

#include "main.h"

/********************����3***************************/
#define COMM3_RX_BUF_SIZE           18//��������С

#define COMM3_RX_FREE            0
#define COMM3_RX_START           1
#define COMM3_RX_DATA            2
#define COMM3_RX_END             3

#define COMM3_TX_BUF_MAX   1024
#define COMM3_TX_BUF_MARK (COMM3_TX_BUF_MAX - 1)

void Comm3Rx_IRQ(void);//����3�����жϴ�����
void Comm3Tx_IRQ(void);//����3�����жϴ�����

void Comm3Tx_Start(void);//��������3���Ϳ��ж�
UCHAR8 Comm3IsDataInTxBuf(void);//�жϴ���3���Ͷ����Ƿ�������
UCHAR8 Comm3PutDataToTxBuf(UCHAR8 Data);//�����ݷŽ�����3���Ͷ���

#endif

