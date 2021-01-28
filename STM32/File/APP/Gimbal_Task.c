#include "Gimbal_Task.h"
#include "RM_Algorithm.h"
#include "RM_MotorCtrlTypes.h"
#include "USART1_Deal.h"

ST_MOTOR_CTRL g_stPitch = 
{
    0x206,0x1FF,RUN,ON,POS_LOOP,UNLOCK,{0},{0},	
    {2,0,0,0,0,0,0,0,600,-600,0,0,0},
    {10,0,0,0,0,0,0,0,5000,-5000,0,0,0},
    {0},3600,0,0,0,0,0,4000,3100,0,0,0,0
};
ST_MOTOR_CTRL g_stYaw = 
{
    0x206,0x1FF,RUN,ON,POS_LOOP,UNLOCK,	{0},{0},
    {10,0,0,0,0,0,0,0,3000,-3000,0,0,0},
	{10,0.5,0,0,0,0,0,0,5000,-5000,0,0,0},
	{0},0,0,0,0,0,0,90,-90,0,0,0,0           //Debug:+-90
};

extern SINT32 g_Coder_Follow;
int siCount = 0;

void Gimbal_Control(void)
{   
    FP32 YawAngle_Tmp = 0;

    /*计算Pitch轴位置*/
    if (DUBS_Flag != FALSE)
        g_stPitch.fpPosDes -= (g_stDBUS.stRC.Ch3 - RC_CH_VALUE_OFFSET)/ 50.0f;
	if (g_stPitch.fpPosDes >= g_stPitch.MaxPos)
	    g_stPitch.fpPosDes = g_stPitch.MaxPos;
	else if(g_stPitch.fpPosDes <= g_stPitch.MinPos)
	    g_stPitch.fpPosDes = g_stPitch.MinPos;
 
    /*计算Yaw  相对底盘目标位置：增量式*/
//    YawAngle_Tmp = (g_stDBUS.stRC.Ch2 - RC_CH_VALUE_OFFSET)/400.0f;//单位脉冲个
//    if (YawAngle_Tmp>=0 && g_Coder_Follow+500<YawAngle_Tmp)
//       g_stYaw.fpPosDes -= (g_Coder_Follow+500)*1.0f/8191*360;     
//    else if (YawAngle_Tmp<=0 && g_Coder_Follow-3000>YawAngle_Tmp)
//       g_stYaw.fpPosDes -= (g_Coder_Follow-3000)*1.0f/8191*360; 
//    else if ((YawAngle_Tmp>0 && g_Coder_Follow+500>=YawAngle_Tmp) || (YawAngle_Tmp<0 && g_Coder_Follow-3500<=YawAngle_Tmp))
//       g_stYaw.fpPosDes -= YawAngle_Tmp*1.0f/8191*360;
//	if(g_stYaw.fpPosDes>90)
//	{
//		g_stYaw.fpPosDes = 90;
//	}
//	if(g_stYaw.fpPosDes<-90)
//	{
//		g_stYaw.fpPosDes = -90;
//	}


    /*Pitch轴闭环控制*/
    g_stPitch.stPosPID.E = g_stPitch.fpPosDes - g_stPitch.fpPosFB;
	CalPID(&g_stPitch.stPosPID);
	g_stPitch.stVeltPID.E = g_stPitch.stPosPID.U - g_stPitch.fpVeltFB;
	CalPID(&g_stPitch.stVeltPID);

	/*Yaw轴闭环控制*/
	//SU   171124原始程序中所有速度反馈都是读取mpu的角速度，只有位置用的是码盘，自己用码盘计算。
	siCount++;
	if(siCount > 10)
	{g_stYaw.fpPosDes = g_stYaw.fpPosDes + 1;
	siCount = 0;}
	g_stYaw.stPosPID.E = g_stYaw.fpPosDes - g_Coder_Follow;
	CalPID_Su(&g_stYaw.stPosPID);

//	g_stYaw.stVeltPID.E = -g_stYaw.stPosPID.U;
//	CalPID(&g_stYaw.stVeltPID);

//	g_stYaw.stPosPID.E = g_stYaw.fpPosDes - g_stYaw.fpPosFB;
//	CalPID(&g_stYaw.stPosPID);
//	g_stYaw.stVeltPID.E = -g_stYaw.stPosPID.U - g_stYaw.fpVeltFB;
//	g_stYaw.stVeltPID.E = 200 - g_stYaw.fpVeltFB;
//	CalPID(&g_stYaw.stVeltPID);
    /*发送云台数据*/
    Send_Current_To_Gimbal();
}


void Send_Current_To_Gimbal(void)
{
    CanTxMsg Tx2_Message;
    SSHORT16 Crt_Pitch = (SSHORT16)(Round(g_stPitch.stVeltPID.U));    
    SSHORT16 Crt_Yaw   = (SSHORT16)(Round(g_stYaw.stPosPID.U));

    Tx2_Message.StdId = 0x1ff;
    Tx2_Message.IDE = CAN_Id_Standard;
    Tx2_Message.RTR = CAN_RTR_Data;
    Tx2_Message.DLC = 0x08;
    
    Tx2_Message.Data[0] = Crt_Yaw>>8;
    Tx2_Message.Data[1] = Crt_Yaw;
 // 	Tx2_Message.Data[0] = 0;
//	Tx2_Message.Data[1] = 0;
    Tx2_Message.Data[2] = Crt_Pitch>>8;
    Tx2_Message.Data[3] = Crt_Pitch;
//	Tx2_Message.Data[2] = 0;
  //  Tx2_Message.Data[3] = 0;
    Tx2_Message.Data[4] = 0x00;
    Tx2_Message.Data[5] = 0x00;
    Tx2_Message.Data[6] = 0x00;
    Tx2_Message.Data[7] = 0x00;
    CAN_Transmit(CAN2,&Tx2_Message);
}
