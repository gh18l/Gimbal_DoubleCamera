#include "main.h"
#include "Rs232.h"
#include "Debug_Task.h"
#include "Led.h"
#include "flash.h"
#include "UnderPan_Task.h"


UN_SAVE unSave;
bool bo_Enable_Flag = TRUE;
UCHAR8 Comm3TxData[13] = {0};//Comm3TxData[12]为发送数据长度
/*Debug*/
extern FP32 plot_x;
bool bo_Plot_Flag = FALSE;
void Debug(void)
{
	extern UCHAR8 Comm3GetData;
	extern UCHAR8 Comm3RxBuf[COMM3_RX_BUF_SIZE];
	UCHAR8 ucSavePit = 0;//存储坑位
	FP32   fpTransition;//数据类型中转
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
/***************************************************通用***********************************************************************/
			case 0x00://存储
				Save();
				SaveParaWord32((UINT32)ucSavePit, 0, (SINT32*)&unSave.siSave[0]);
				break;
			case 0x01://使能所有电机
				bo_Enable_Flag = TRUE;
				EnableAllMotor();
				break;
			case 0x02://失能所有电机
				bo_Enable_Flag = FALSE;
				DisableAllMotor();
				break;
			case 0x03://发送速度底盘电机
				bo_Enable_Flag = TRUE;
				SendVeltToMotor();
				break;
			case 0x04://失能底盘电机
			
				break;
			case 0x05://使能执行电机
				
				break;
			case 0x06://失能执行电机
				
				break;
			case 0x15://查询PID图像
				break;
			case 0x16://查询PID
				switch(Comm3RxBuf[2])
				{
				    case 0x17://查询电机1PID
						Send(g_stUndePan1.stVeltPID.KP, g_stUndePan1.stVeltPID.KI, g_stUndePan1.stVeltPID.KD);
			            break;
				    case 0x18://查询电机2PID
						Send(g_stUndePan2.stVeltPID.KP, g_stUndePan2.stVeltPID.KI, g_stUndePan2.stVeltPID.KD);
					    break;
				    case 0x19://查询电机3PID
						Send(g_stUndePan3.stVeltPID.KP, g_stUndePan3.stVeltPID.KI, g_stUndePan3.stVeltPID.KD);
					    break;			    
				    case 0x1A://查询电机4PID
						Send(g_stUndePan4.stVeltPID.KP, g_stUndePan4.stVeltPID.KI, g_stUndePan4.stVeltPID.KD);
					    break;
				    case 0x1B://查询Pitch速度PID
					    break;
					case 0x1C://查询Pitch位置PID
					    break;
					case 0x1D://查询Yaw速度PID
					    break;
					case 0x1E://查询Yaw位置PID
					    break;
					default:
					    break;
				}
				Comm3TxData[12] = 14;//现在统一发三个数
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
		    case 0x17://设置电机1PID
				Receive(g_stUndePan1.stVeltPID.KP, g_stUndePan1.stVeltPID.KI, g_stUndePan1.stVeltPID.KD);
			    break;
		    case 0x18://设置电机2PID
				Receive(g_stUndePan2.stVeltPID.KP, g_stUndePan2.stVeltPID.KI, g_stUndePan2.stVeltPID.KD);
			    break;
		    case 0x19://设置电机3PID
				Receive(g_stUndePan3.stVeltPID.KP, g_stUndePan3.stVeltPID.KI, g_stUndePan3.stVeltPID.KD);
			    break;			    
		    case 0x1A://设置电机4PID
				Receive(g_stUndePan4.stVeltPID.KP, g_stUndePan4.stVeltPID.KI, g_stUndePan4.stVeltPID.KD);
			    break;
		    case 0x1B://设置Pitch速度PID
			    break;
			case 0x1C://设置Pitch位置PID
			    break;
			case 0x1D://设置Yaw速度PID
			    break;
			case 0x1E://设置Yaw位置PID
			    break;
			case 0x1F:
				Receive(g_stUndePan1.fpVeltDes, g_stUndePan2.fpVeltDes,g_stUndePan3.fpVeltDes);
				break;
			case 0x50:
				LED_RED_ON(); //测试用
				break;
		    case 0x51:
			    LED_RED_OFF();
				break;
			default:
				break;
	  	}
		Comm3GetData = 0;//数据处理完了
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
