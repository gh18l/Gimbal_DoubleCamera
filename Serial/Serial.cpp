#include"tchar.h"
#include"Serial.h"  
#include<process.h>  
typedef unsigned(__stdcall *PTHREEA_START) (void *);

CSerial::CSerial(void)
{
	m_hComm = INVALID_HANDLE_VALUE;
}

CSerial::~CSerial(void)
{
	if (m_hComm != INVALID_HANDLE_VALUE) {
		CloseHandle(m_hComm);
	}
}

/*********************************************************************************************
* ����    ��    �������̻߳ص�����
* ����       �� �յ����ݺ󣬼򵥵���ʾ����
********************************************************************************************/
DWORD WINAPI CommProc(LPVOID lpParam) {

	CSerial* pSerial = (CSerial*)lpParam;

	//��մ���  
	PurgeComm(pSerial->m_hComm, PURGE_RXCLEAR | PURGE_TXCLEAR);

	char buf[512];
	DWORD dwRead;
	while (pSerial->m_hComm != INVALID_HANDLE_VALUE) {
		BOOL bReadOK = ReadFile(pSerial->m_hComm, buf, 512, &dwRead, NULL);
		if (bReadOK && (dwRead > 0)) {
			buf[dwRead] = '\0';
			MessageBoxA(NULL, buf, "�����յ�����", MB_OK);
		}
	}
	return 0;
}

/*******************************************************************************************
* ����     ��  �򿪴���
* port     :   ���ں�, ��_T("COM1:")
* baud_rate:   ������
* date_bits:  ����λ����Ч��Χ4~8��
* stop_bit :    ֹͣλ
* parity   : ��żУ�顣Ĭ��Ϊ��У�顣NOPARITY 0�� ODDPARITY 1��EVENPARITY 2��MARKPARITY 3��SPACEPARITY 4
********************************************************************************************/
BOOL CSerial::OpenSerialPort(TCHAR* port, UINT baud_rate, BYTE date_bits, BYTE stop_bit, BYTE parity)
{
	//�򿪴���  
	m_hComm = CreateFile(port, GENERIC_READ | GENERIC_WRITE, 0, NULL, OPEN_EXISTING, 0, NULL);//��ռ��ʽ�򿪴���  

	TCHAR err[512];

	if (m_hComm == INVALID_HANDLE_VALUE) {
		_stprintf_s(err, _T("�򿪴���%s ʧ�ܣ���鿴�ô����Ƿ��ѱ�ռ��"), port);
		MessageBox(NULL, err, _T("��ʾ"), MB_OK);
		return FALSE;
	}

	//MessageBox(NULL,_T("�򿪳ɹ�"),_T("��ʾ"),MB_OK);  

	//��ȡ����Ĭ������  
	DCB dcb;
	if (!GetCommState(m_hComm, &dcb)) {
		MessageBox(NULL, _T("��ȡ���ڵ�ǰ���Բ���ʧ��"), _T("��ʾ"), MB_OK);
	}

	//���ô��ڲ���  
	dcb.BaudRate = baud_rate;  //������  
	dcb.fBinary = TRUE;            //������ģʽ������ΪTRUE  
	dcb.ByteSize = date_bits;  //����λ����Χ4-8  
	dcb.StopBits = ONESTOPBIT; //ֹͣλ  

	if (parity == NOPARITY) {
		dcb.fParity = FALSE;   //��żУ�顣����żУ��  
		dcb.Parity = parity;   //У��ģʽ������żУ��  
	}
	else {
		dcb.fParity = TRUE;        //��żУ�顣  
		dcb.Parity = parity;   //У��ģʽ������żУ��  
	}

	dcb.fOutxCtsFlow = FALSE;  //CTS���ϵ�Ӳ������  
	dcb.fOutxDsrFlow = FALSE;  //DST���ϵ�Ӳ������  
	dcb.fDtrControl = DTR_CONTROL_ENABLE;//DTR����  
	dcb.fDsrSensitivity = FALSE;
	dcb.fTXContinueOnXoff = FALSE;//  
	dcb.fOutX = FALSE;         //�Ƿ�ʹ��XON/XOFFЭ��  
	dcb.fInX = FALSE;          //�Ƿ�ʹ��XON/XOFFЭ��  
	dcb.fErrorChar = FALSE;        //�Ƿ�ʹ�÷��ʹ���Э��  
	dcb.fNull = FALSE;         //ͣ��null stripping  
	dcb.fRtsControl = RTS_CONTROL_ENABLE;//  
	dcb.fAbortOnError = FALSE; //���ڷ��ʹ��󣬲�����ֹ���ڶ�д  

							   //���ô��ڲ���  
	if (!SetCommState(m_hComm, &dcb)) {
		MessageBox(NULL, _T("���ô��ڲ���ʧ��"), _T("��ʾ"), MB_OK);
		return FALSE;
	}

	//���ô����¼�  
	SetCommMask(m_hComm, EV_RXCHAR);//�ڻ��������ַ�ʱ�����¼�  
	SetupComm(m_hComm, 16384, 16384);

	//���ô��ڶ�дʱ��  
	COMMTIMEOUTS CommTimeOuts;
	GetCommTimeouts(m_hComm, &CommTimeOuts);
	CommTimeOuts.ReadIntervalTimeout = MAXDWORD;
	CommTimeOuts.ReadTotalTimeoutMultiplier = 0;
	CommTimeOuts.ReadTotalTimeoutConstant = 0;
	CommTimeOuts.WriteTotalTimeoutMultiplier = 10;
	CommTimeOuts.WriteTotalTimeoutConstant = 1000;

	if (!SetCommTimeouts(m_hComm, &CommTimeOuts)) {
		MessageBox(NULL, _T("���ô���ʱ��ʧ��"), _T("��ʾ"), MB_OK);
		return FALSE;
	}

	//�����̣߳���ȡ����  
	HANDLE hReadCommThread = (HANDLE)_beginthreadex(NULL, 0, (PTHREEA_START)CommProc, (LPVOID) this, 0, NULL);
	return TRUE;
}

/********************************************************************************************
* ����    ��    ͨ�����ڷ���һ������
********************************************************************************************/
BOOL CSerial::SendData(unsigned char* data, int len) {
	if (m_hComm == INVALID_HANDLE_VALUE) {
		MessageBox(NULL, _T("����δ��"), _T("��ʾ"), MB_OK);
		return FALSE;
	}

	//��մ���  
	PurgeComm(m_hComm, PURGE_RXCLEAR | PURGE_TXCLEAR);

	//д����  
	DWORD dwWrite = 0;
	DWORD dwRet = WriteFile(m_hComm, data, len, &dwWrite, NULL);

	//��մ���  
	PurgeComm(m_hComm, PURGE_RXCLEAR | PURGE_TXCLEAR);

	if (!dwRet) {
		MessageBox(NULL, _T("��������ʧ��"), _T("��ʾ"), MB_OK);
		return FALSE;
	}
	return TRUE;
}




//////////////////////////////new////////////////////////////////////
//int CSerial::Serial_Send_Yaw(float value)
//{
//	unsigned char data[8];//
//	data[0] = 0x55;
//	data[1] = 0x04;
//	data[6] = 0x00;
//	data[7] = 0xAA;     //0xAA��yaw��0xBB��pitch
//	memcpy(&data[2], &value, 4);
//	for (int j = 0; j<9; j++)
//		SendData(data, 8);
//	return 0;
//}
//
//int CSerial::Serial_Send_Pitch(float value)
//{
//	unsigned char data[8];//
//	data[0] = 0x55;
//	data[1] = 0x04;
//	data[6] = 0x00;
//	data[7] = 0xBB;     //0xAA��yaw��0xBB��pitch
//	memcpy(&data[2], &value, 4);
//	for (int j = 0; j<9; j++)
//		SendData(data, 8);
//	return 0;
//}

//////////////////////////////new////////////////////////////////////
int CSerial::Serial_Send_Yaw(int value, bool dir)
{
	unsigned char data[8];//
	data[0] = 0xAA;
	if (dir == 1)    //����
	{
		data[1] = 0xAB;
	}
	else
	{
		data[1] = 0xCD;
	}
	memcpy(&data[2], &value, 4);
	data[6] = 0x0d;
	data[7] = 0x0a;
	//for (int j = 0; j<9; j++)
	SendData(data, 8);
	return 0;
}

int CSerial::Serial_Send_Pitch(int value, bool dir)
{
	unsigned char data[8];//
	data[0] = 0xBB;
	if (dir == 1)    //����
	{
		data[1] = 0xAB;
	}
	else
	{
		data[1] = 0xCD;
	}
	memcpy(&data[2], &value, 4);
	data[6] = 0x0d;
	data[7] = 0x0a;
	//for (int j = 0; j<9; j++)
	SendData(data, 8);
	return 0;
}


