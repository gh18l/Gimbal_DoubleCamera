#ifndef _RS232_H
#define _RS232_H

#include "main.h"

/********************串口3***************************/
#define COMM3_RX_BUF_SIZE           18//读缓冲块大小

#define COMM3_RX_FREE            0
#define COMM3_RX_START           1
#define COMM3_RX_DATA            2
#define COMM3_RX_END             3

#define COMM3_TX_BUF_MAX   1024
#define COMM3_TX_BUF_MARK (COMM3_TX_BUF_MAX - 1)

void Comm3Rx_IRQ(void);//串口3接收中断处理函数
void Comm3Tx_IRQ(void);//串口3发送中断处理函数

void Comm3Tx_Start(void);//开启串口3发送空中断
UCHAR8 Comm3IsDataInTxBuf(void);//判断串口3发送队列是否有数据
UCHAR8 Comm3PutDataToTxBuf(UCHAR8 Data);//将数据放进串口3发送队列

#endif

