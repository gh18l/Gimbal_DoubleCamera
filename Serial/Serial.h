#pragma once
#include <windows.h>  
#include <opencv2/opencv.hpp>
class CSerial
{
public:
	CSerial(void);
	~CSerial(void);

	//打开串口  
	BOOL OpenSerialPort(TCHAR* port, UINT baud_rate, BYTE date_bits, BYTE stop_bit, BYTE parity = NOPARITY);

	//发送数据  
	BOOL SendData(unsigned char* data, int len);
public:
	HANDLE m_hComm;

//////////////////////new/////////////////////////
public:
	int Serial_Send_Yaw(int value, bool dir);
	int Serial_Send_Pitch(int value, bool dir);
};