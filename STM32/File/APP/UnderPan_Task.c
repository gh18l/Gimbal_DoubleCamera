#include <stm32f4xx.h>
#include "main.h"
#include "RM_Algorithm.h"
#include "USART1_Deal.h"
#include "UnderPan_Task.h"
#include "Delay.h"


/*�����ٶ�ģʽ����*/
FP32 Speed_Coe;                           
//ң�����ٶȷ���
ST_VELT_DISTRIBUTE g_stVeltDistribute = {0};
//���̵���ջ�����                                                        //VeltPID
ST_MOTOR_CTRL g_stUndePan1 = {0x201,0,RUN,ON,VELT_LOOP,UNLOCK,{0},{0},{0},{30,0,0,0,0,0,0,0,5000,-5000,0,0,0},{0},0,0,0,0,0,0,0,0,0,0,0,0};
ST_MOTOR_CTRL g_stUndePan2 = {0x202,0,RUN,ON,VELT_LOOP,UNLOCK,{0},{0},{0},{30,0,0,0,0,0,0,0,5000,-5000,0,0,0},{0},0,0,0,0,0,0,0,0,0,0,0,0};
ST_MOTOR_CTRL g_stUndePan3 = {0x203,0,RUN,ON,VELT_LOOP,UNLOCK,{0},{0},{0},{30,0,0,0,0,0,0,0,5000,-5000,0,0,0},{0},0,0,0,0,0,0,0,0,0,0,0,0};
ST_MOTOR_CTRL g_stUndePan4 = {0x204,0,RUN,ON,VELT_LOOP,UNLOCK,{0},{0},{0},{30,0,0,0,0,0,0,0,5000,-5000,0,0,0},{0},0,0,0,0,0,0,0,0,0,0,0,0};

extern SINT32 g_Coder_Follow;
extern EN_WORK_MODE en_WorkMode;
extern EN_OPERATION_MODE g_emOperationMode;
extern bool bo_Enable_Flag;
void Underpan_Control(void)
{
	if (g_emOperationMode == RC_Mode)
	{
	    /*��ದ�뿪��ȷ���ٶ�ϵ��:���е���*/
	    if(g_stDBUS.stRC.SW_L == RC_SW_UP)
	    	Speed_Coe = HighSpeedMode;
	    if(g_stDBUS.stRC.SW_L == RC_SW_MID)
	    	Speed_Coe = MidSpeedMode;
	    if(g_stDBUS.stRC.SW_L == RC_SW_DOWM) 
	    	Speed_Coe = LowSpeedMode;
	    /*�ٶ���������*/
	    if(abs(g_stDBUS.stRC.Ch0 - RC_CH_VALUE_OFFSET) < RC_CH_VALUE_DEAD)
	    	g_stDBUS.stRC.Ch0 = RC_CH_VALUE_OFFSET;
	    if(abs(g_stDBUS.stRC.Ch1 - RC_CH_VALUE_OFFSET) < RC_CH_VALUE_DEAD)
	    	g_stDBUS.stRC.Ch1 = RC_CH_VALUE_OFFSET;
	    
	    
	    /*�������Ŀ���ٶ�*/
	    g_stVeltDistribute.x = (g_stDBUS.stRC.Ch0 - RC_CH_VALUE_OFFSET)*1.0f/RC_CH_VALUE_RANGE*TankSpeedMax;
	    g_stVeltDistribute.y = (g_stDBUS.stRC.Ch1 - RC_CH_VALUE_OFFSET)*1.0f/RC_CH_VALUE_RANGE*TankSpeedMax;
	    g_stVeltDistribute.r = -(g_Coder_Follow-1080)/2.0f;
	    /*�ٶ�Ŀ��ֵ������ĸ�����*/
	    Velt_Distribute(g_stVeltDistribute.x,g_stVeltDistribute.y,g_stVeltDistribute.r,Speed_Coe);
	}
	if (bo_Enable_Flag)
	{
	    /*PID����*/
	    g_stUndePan1.stVeltPID.E = g_stUndePan1.fpVeltDes - g_stUndePan1.fpVeltFB;
	    CalPID(&g_stUndePan1.stVeltPID);
	    g_stUndePan2.stVeltPID.E = g_stUndePan2.fpVeltDes - g_stUndePan2.fpVeltFB;
	    CalPID(&g_stUndePan2.stVeltPID);
	    g_stUndePan3.stVeltPID.E = g_stUndePan3.fpVeltDes - g_stUndePan3.fpVeltFB;
	    CalPID(&g_stUndePan3.stVeltPID);
	    g_stUndePan4.stVeltPID.E = g_stUndePan4.fpVeltDes - g_stUndePan4.fpVeltFB;
	    CalPID(&g_stUndePan4.stVeltPID);
	}
    /*�����̷�������*/
    Send_Current_To_UnderPan(); 
}

void Velt_Distribute(FP32 Vx,FP32 Vy,FP32 Vr,FP32 SpeedCoefficient)
{
//    const USHORT16 usSpdIncrement = 100;
//    FP32 VeltTmp1 = g_stUndePan1.fpVeltFB;
//    FP32 VeltTmp2 = g_stUndePan2.fpVeltFB;
//    FP32 VeltTmp3 = g_stUndePan3.fpVeltFB;
//    FP32 VeltTmp4 = g_stUndePan4.fpVeltFB;

	/*�����ٶȷ���*/
    g_stUndePan1.fpVeltDes = -(-Vx*Mec_Weel_Coe + Vy*Mec_Weel_Coe - Vr*Mec_Weel_Row)*SpeedCoefficient;
    g_stUndePan2.fpVeltDes =  ( Vx*Mec_Weel_Coe + Vy*Mec_Weel_Coe + Vr*Mec_Weel_Row)*SpeedCoefficient;
	g_stUndePan3.fpVeltDes =  (-Vx*Mec_Weel_Coe + Vy*Mec_Weel_Coe + Vr*Mec_Weel_Row)*SpeedCoefficient;
	g_stUndePan4.fpVeltDes = -( Vx*Mec_Weel_Coe + Vy*Mec_Weel_Coe - Vr*Mec_Weel_Row)*SpeedCoefficient;
	
//    /*�Ƚ������õ����ٶ��뵱ǰ�ٶȵĲ�ֵ�Ƿ�����ٶ�ƽ������ʱ����������֤�ٶȻ����仯*/
//    //�����ٶȱȵ�ǰ�ٶȴ�
//	 if(g_stUndePan1.fpVeltDes > VeltTmp1)
//	 {
//	 	if(VeltTmp1 + usSpdIncrement < g_stUndePan1.fpVeltDes)//�仯����
//		{
//			g_stUndePan1.fpVeltDes = VeltTmp1 + usSpdIncrement;
//		}
//	 }
//	 else //�����ٶȱȵ�ǰ�ٶ�С
//	 {
//	 	if(VeltTmp1 - usSpdIncrement > g_stUndePan1.fpVeltDes)//�仯����
//		{
//			g_stUndePan1.fpVeltDes = VeltTmp1 - usSpdIncrement;
//		}
//	 }
//
//   if(g_stUndePan2.fpVeltDes > VeltTmp2)
//	 {
//	 	if(VeltTmp2 + usSpdIncrement < g_stUndePan2.fpVeltDes)//�仯����
//		{
//			g_stUndePan2.fpVeltDes = VeltTmp2 + usSpdIncrement;
//		}
//	 }
//	 else 
//	 {
//	 	if(VeltTmp2 - usSpdIncrement > g_stUndePan2.fpVeltDes)//�仯����
//		{
//			g_stUndePan2.fpVeltDes = VeltTmp2 - usSpdIncrement;
//		}
//	 }
//
//     if(g_stUndePan3.fpVeltDes > VeltTmp3)
//	 {
//	 	if(VeltTmp3 + usSpdIncrement < g_stUndePan3.fpVeltDes)//�仯����
//		{
//			g_stUndePan3.fpVeltDes = VeltTmp3 + usSpdIncrement;
//		}
//	 }
//	 else 
//	 {
//	 	if(VeltTmp3 - usSpdIncrement > g_stUndePan3.fpVeltDes)//�仯����
//		{
//			g_stUndePan3.fpVeltDes = VeltTmp3 - usSpdIncrement;
//		}
//	 }
//
//     if(g_stUndePan4.fpVeltDes > VeltTmp4)
//	 {
//	 	if(VeltTmp4 + usSpdIncrement < g_stUndePan4.fpVeltDes)//�仯����
//		{
//			g_stUndePan4.fpVeltDes = VeltTmp4 + usSpdIncrement;
//		}
//	 }
//	 else 
//	 {
//	 	if(VeltTmp4 - usSpdIncrement > g_stUndePan4.fpVeltDes)//�仯����
//		{
//			g_stUndePan4.fpVeltDes = VeltTmp4 - usSpdIncrement;
//		}
//	 }
//
//	/*�ٶ��������*/
//	if(g_stUndePan1.fpVeltDes < SpeedMin)
//	{
//		g_stUndePan1.fpVeltDes = SpeedMin;
//	}
//	if(g_stUndePan1.fpVeltDes > SpeedMax)
//	{
//		g_stUndePan1.fpVeltDes = SpeedMax;
//	}
//
//	if(g_stUndePan2.fpVeltDes < SpeedMin)
//	{
//		g_stUndePan2.fpVeltDes = SpeedMin;
//	}
//	if(g_stUndePan2.fpVeltDes > SpeedMax)
//	{
//		g_stUndePan2.fpVeltDes = SpeedMax;
//	}
//
//	if(g_stUndePan3.fpVeltDes < SpeedMin)
//	{
//		g_stUndePan3.fpVeltDes = SpeedMin;
//	}
//	if(g_stUndePan3.fpVeltDes > SpeedMax)
//	{
//		g_stUndePan3.fpVeltDes = SpeedMax;
//	}
//
//	if(g_stUndePan4.fpVeltDes < SpeedMin)
//	{
//		g_stUndePan4.fpVeltDes = SpeedMin;
//	}
//	if(g_stUndePan4.fpVeltDes > SpeedMax)
//	{
//		g_stUndePan4.fpVeltDes = SpeedMax;
//	}
}

void Send_Current_To_UnderPan(void)
{
    CanTxMsg Tx2_Underpan;
    SSHORT16 ssMotor1 = (SSHORT16)(Round(g_stUndePan1.stVeltPID.U));
    SSHORT16 ssMotor2 = (SSHORT16)(Round(g_stUndePan2.stVeltPID.U));
    SSHORT16 ssMotor3 = (SSHORT16)(Round(g_stUndePan3.stVeltPID.U));
    SSHORT16 ssMotor4 = (SSHORT16)(Round(g_stUndePan4.stVeltPID.U));

    /*���͵����ٶ�*/	
	Tx2_Underpan.StdId = 0x200;
	Tx2_Underpan.IDE = CAN_Id_Standard;
	Tx2_Underpan.RTR = CAN_RTR_Data;
	Tx2_Underpan.DLC = 0x08;
	Tx2_Underpan.Data[0] = ssMotor1>>8; //��ǰ��     201            
	Tx2_Underpan.Data[1] = ssMotor1;
	Tx2_Underpan.Data[2] = ssMotor2>>8;    //��ǰ��   202            
	Tx2_Underpan.Data[3] = ssMotor2;                                    
	Tx2_Underpan.Data[4] = ssMotor3>>8;    //�����     203            
	Tx2_Underpan.Data[5] = ssMotor3;
	Tx2_Underpan.Data[6] = ssMotor4>>8;    //�Һ���   204            
	Tx2_Underpan.Data[7] = ssMotor4;
	CAN_Transmit(CAN2,&Tx2_Underpan);
	delay_us(100);//����ʱ���ܻ����ڷ���Ƶ��̫��,����ĳ���������û����ȥ
}
