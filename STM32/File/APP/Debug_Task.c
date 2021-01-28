#include "main.h"
#include "Rs232.h"
#include "Debug_Task.h"
#include "Led.h"
#include "flash.h"
#include "UnderPan_Task.h"


UN_SAVE unSave;
bool bo_Enable_Flag = TRUE;
UCHAR8 Comm3TxData[13] = {0};//Comm3TxData[12]Ϊ�������ݳ���
/*Debug*/
extern FP32 plot_x;
bool bo_Plot_Flag = FALSE;
void Debug(void)
{
	extern UCHAR8 Comm3GetData;
	extern UCHAR8 Comm3RxBuf[COMM3_RX_BUF_SIZE];
	UCHAR8 ucSavePit = 0;//�洢��λ
	FP32   fpTransition;//����������ת
	SINT32 i = 0;		
/******************************************************************************/
	
//	Send2(plot_x/1000.0f,g_stUndePan1.fpVeltFB);
//	Comm3TxData[8] = 10;
//	Comm3PutDataToTxBuf(0x55);
//	Comm3PutDataToTxBuf(0x00);
//	Comm3PutDataToTxBuf(0x15);
//	Comm3PutDataToTxBuf(Comm3TxData[8]);
//	for(i = 0;i < 8;i++)
//	{
//		Comm3PutDataToTxBuf(Comm3TxData[i]);
//	}
//	Comm3PutDataToTxBuf(0x00);
//	Comm3PutDataToTxBuf(0xAA);
	
/*******************************************************************************/

	if(Comm3GetData == 1)
	{
		switch(Comm3RxBuf[1])
		{
/***************************************************ͨ��***********************************************************************/
			case 0x00://�洢
				Save();
				SaveParaWord32((UINT32)ucSavePit, 0, (SINT32*)&unSave.siSave[0]);
				break;
			case 0x01://ʹ�����е��
				bo_Enable_Flag = TRUE;
				EnableAllMotor();
				break;
			case 0x02://ʧ�����е��
				bo_Enable_Flag = FALSE;
				DisableAllMotor();
				break;
			case 0x03://�����ٶȵ��̵��
				bo_Enable_Flag = TRUE;
				SendVeltToMotor();
				break;
			case 0x04://ʧ�ܵ��̵��
			
				break;
			case 0x05://ʹ��ִ�е��
				
				break;
			case 0x06://ʧ��ִ�е��
				
				break;
			case 0x15://��ѯPIDͼ��
				break;
			case 0x16://��ѯPID
				switch(Comm3RxBuf[2])
				{
				    case 0x17://��ѯ���1PID
						Send(g_stUndePan1.stVeltPID.KP, g_stUndePan1.stVeltPID.KI, g_stUndePan1.stVeltPID.KD);
			            break;
				    case 0x18://��ѯ���2PID
						Send(g_stUndePan2.stVeltPID.KP, g_stUndePan2.stVeltPID.KI, g_stUndePan2.stVeltPID.KD);
					    break;
				    case 0x19://��ѯ���3PID
						Send(g_stUndePan3.stVeltPID.KP, g_stUndePan3.stVeltPID.KI, g_stUndePan3.stVeltPID.KD);
					    break;			    
				    case 0x1A://��ѯ���4PID
						Send(g_stUndePan4.stVeltPID.KP, g_stUndePan4.stVeltPID.KI, g_stUndePan4.stVeltPID.KD);
					    break;
				    case 0x1B://��ѯPitch�ٶ�PID
					    break;
					case 0x1C://��ѯPitchλ��PID
					    break;
					case 0x1D://��ѯYaw�ٶ�PID
					    break;
					case 0x1E://��ѯYawλ��PID
					    break;
					default:
					    break;
				}
				Comm3TxData[12] = 14;//����ͳһ��������
				Comm3PutDataToTxBuf(0x55);
				Comm3PutDataToTxBuf(0x00);
				Comm3PutDataToTxBuf(Comm3RxBuf[2]);
				Comm3PutDataToTxBuf(Comm3TxData[12]);
				for(i = 0;i < (Comm3TxData[12] - 2);i++)
				{
					Comm3PutDataToTxBuf(Comm3TxData[i]);
				}
				Comm3PutDataToTxBuf(0x00);
				Comm3PutDataToTxBuf(0xAA);
			    break;
		    case 0x17://���õ��1PID
				Receive(g_stUndePan1.stVeltPID.KP, g_stUndePan1.stVeltPID.KI, g_stUndePan1.stVeltPID.KD);
			    break;
		    case 0x18://���õ��2PID
				Receive(g_stUndePan2.stVeltPID.KP, g_stUndePan2.stVeltPID.KI, g_stUndePan2.stVeltPID.KD);
			    break;
		    case 0x19://���õ��3PID
				Receive(g_stUndePan3.stVeltPID.KP, g_stUndePan3.stVeltPID.KI, g_stUndePan3.stVeltPID.KD);
			    break;			    
		    case 0x1A://���õ��4PID
				Receive(g_stUndePan4.stVeltPID.KP, g_stUndePan4.stVeltPID.KI, g_stUndePan4.stVeltPID.KD);
			    break;
		    case 0x1B://����Pitch�ٶ�PID
			    break;
			case 0x1C://����Pitchλ��PID
			    break;
			case 0x1D://����Yaw�ٶ�PID
			    break;
			case 0x1E://����Yawλ��PID
			    break;
			case 0x1F:
				Receive(g_stUndePan1.fpVeltDes, g_stUndePan2.fpVeltDes,g_stUndePan3.fpVeltDes);
				break;
			case 0x50:
				LED_RED_ON(); //������
				break;
		    case 0x51:
			    LED_RED_OFF();
				break;
			default:
				break;
	  	}
		Comm3GetData = 0;//���ݴ�������
	 }
} 

void EnableAllMotor(void)
{
    g_stUndePan1.fpVeltDes = 0;
	g_stUndePan2.fpVeltDes = 0;
	g_stUndePan3.fpVeltDes = 0;
	g_stUndePan4.fpVeltDes = 0;
}
void DisableAllMotor(void)
{
    g_stUndePan1.stVeltPID.U = 0;
	g_stUndePan2.stVeltPID.U = 0;
	g_stUndePan3.stVeltPID.U = 0;
	g_stUndePan4.stVeltPID.U = 0;
}
void SendVeltToMotor(void)
{
    g_stUndePan1.fpVeltDes = 100;
	g_stUndePan2.fpVeltDes = 100;
	g_stUndePan3.fpVeltDes = 100;
	g_stUndePan4.fpVeltDes = 100;
}
