#include "stm32f4xx_it.h"
#include "main.h"
#include "mpu6050_driver.h"
#include "UnderPan_Task.h"
#include "Gimbal_Task.h"
#include "USART1_Deal.h"
#include "RS232.h"
#include "os.h"

/*************************************************************************
��������SysTick_Handler
���ܣ�  ϵͳʱ���ж�
*************************************************************************/

void SysTick_Handler( void )
{
   	CPU_SR         cpu_sr;	
	OS_CRITICAL_ENTER();                         
    OSIntEnter();
    OS_CRITICAL_EXIT();

    OSTimeTick();                                /* Call uC/OS-II's OSTimeTick()                       */
    OSIntExit();
}

/*************************************************************************
��������USART3_IRQHandler
����:   ����3�жϴ�����
*************************************************************************/
void USART3_IRQHandler(void)
{
	CPU_SR         cpu_sr;	
	OS_CRITICAL_ENTER();                         
    OSIntEnter();
    OS_CRITICAL_EXIT();

	if(USART_GetITStatus(USART3, USART_IT_RXNE) != RESET)
	{
		Comm3Rx_IRQ();
	}

	if(USART_GetITStatus(USART3, USART_IT_TXE) != RESET)
	{   
		Comm3Tx_IRQ();
	}

	OSIntExit(); 
}


/*************************************************************************
��������TIM4_IRQHandler
���ܣ�  TIM4��ʱ���ж�--0.5ms
*************************************************************************/
void TIM4_IRQHandler(void)
{
    if (TIM_GetITStatus(TIM4,TIM_IT_Update)!= RESET) 
	{
        TIM_ClearFlag(TIM4, TIM_FLAG_Update);
    }
}

/*************************************************************************
��������TIM7_IRQHandler
���ܣ�  TIM7��ʱ���ж�--10us
*************************************************************************/
UINT32 Flag_Motor1_10us = 0;
UINT32 Flag_Motor2_10us = 0;
UINT32 Flag_Motor3_10us = 0;
UINT32 Flag_Motor4_10us = 0;
void TIM7_IRQHandler(void)
{
	if(TIM_GetITStatus(TIM7,TIM_IT_Update)!= RESET) //����ж�
	{
        Flag_Motor1_10us++;  
        Flag_Motor2_10us++;
        Flag_Motor3_10us++;
        Flag_Motor4_10us++;            	
	}
    TIM_ClearITPendingBit(TIM7,TIM_IT_Update);  //����жϱ�־λ	
}

/*************************************************************************
��������CAN1_TX_IRQHandler
���ܣ�  CAN1�����ж�
*************************************************************************/
void CAN1_TX_IRQHandler(void) //CAN TX
{
	CPU_SR         cpu_sr;	
	OS_CRITICAL_ENTER();                         
    OSIntEnter();
    OS_CRITICAL_EXIT();

    if (CAN_GetITStatus(CAN1,CAN_IT_TME)!= RESET) 
	{
	   CAN_ClearITPendingBit(CAN1,CAN_IT_TME);
    }

	OSIntExit();
}

/*************************************************************************
��������CAN1_RX0_IRQHandler
���ܣ�  CAN1�����ж�
*************************************************************************/
void CAN1_RX0_IRQHandler(void)
{
	CanRxMsg CAN1_RxMsg;

    CPU_SR         cpu_sr;	
	OS_CRITICAL_ENTER();                         
    OSIntEnter();
    OS_CRITICAL_EXIT();		     
    
    if (CAN_GetITStatus(CAN1,CAN_IT_FMP0)!= RESET) 
	{
        CAN_ClearITPendingBit(CAN1, CAN_IT_FMP0);
        CAN_Receive(CAN1, CAN_FIFO0, &CAN1_RxMsg);   
        
        g_stYaw.stCoder.siCode = (SINT32)(CAN1_RxMsg.Data[0]<<24)|(SINT32)(CAN1_RxMsg.Data[1]<<16) 
                                 |(SINT32)(CAN1_RxMsg.Data[2]<<8) | (SINT32)(CAN1_RxMsg.Data[3]);
        g_stYaw.fpPosFB = g_stYaw.stCoder.siCode*0.01f;//�Ƕ�           
    }

	OSIntExit();
}

/*************************************************************************
��������CAN2_TX_IRQHandler
���ܣ�  CAN2�����ж�
*************************************************************************/
void CAN2_TX_IRQHandler(void) //CAN TX
{
    CPU_SR         cpu_sr;	
	OS_CRITICAL_ENTER();                         
    OSIntEnter();
    OS_CRITICAL_EXIT();

    if (CAN_GetITStatus(CAN2,CAN_IT_TME)!= RESET) 
	{
	   CAN_ClearITPendingBit(CAN2,CAN_IT_TME);
    }

	OSIntExit();
}

/*******************************************************************
**���̵���˲���װ����
*********************************************************************/
#define NUM 10

void UnderPan_DataFilter(SINT32 *siDiffCode, UINT32 *FlagTmp, UINT32 *Flag_10us,FP32* fpBuf, FP32* UndePan_VeltFB)
{
	SINT32 i = 0;
	FP32 sum = 0;

	if (*siDiffCode > 5000)
        *siDiffCode -= 8191;           
    else if(*siDiffCode < -5000)
        *siDiffCode += 8191;

	fpBuf[(*FlagTmp)++] = 1.0f*(*siDiffCode)*60000/8191/((*Flag_10us)*0.01f);
    *Flag_10us = 0;
					
	if (*FlagTmp == NUM)
	{
		for (i=1;i<NUM;i++)
        	sum += fpBuf[i];

		*UndePan_VeltFB = sum/9.0f;  
		*FlagTmp = 0;
	}
}

/********************************************************************
**CAN2�����жϣ� ���̵�� ��̨���
*********************************************************************/
/*�˲�����*/
FP32 Velt1_Tmp[NUM] = {0};
FP32 Velt2_Tmp[NUM] = {0};
FP32 Velt3_Tmp[NUM] = {0};
FP32 Velt4_Tmp[NUM] = {0};
UINT32 Flag1_Tmp  = 0;
UINT32 Flag2_Tmp  = 0;
UINT32 Flag3_Tmp  = 0;
UINT32 Flag4_Tmp  = 0;

UINT32 Angle_Pitch_Buf[NUM] = {0};
UINT32 pitch_cnt = 0;

SINT32 g_Coder_Follow = 0;
SINT32 FollowTmpPre = 0;

/*Debug*/
int i=0;
void CAN2_RX0_IRQHandler(void)
{
    CanRxMsg CAN2_RxMsg;
    UINT32 Pitch_Sum = 0;
	SINT32 FollowTmp = 0;
	SINT32 Error= 0;
   	
	CPU_SR         cpu_sr;	
	OS_CRITICAL_ENTER();                         
    OSIntEnter();
    OS_CRITICAL_EXIT();

    if (CAN_GetITStatus(CAN2,CAN_IT_FMP0)!= RESET) 
    {
        CAN_Receive(CAN2, CAN_FIFO0, &CAN2_RxMsg);

        switch (CAN2_RxMsg.StdId)
        {
            /*���̵��*/
            case 0x201:         
                g_stUndePan1.stCoder.siCode = CAN2_RxMsg.Data[0]<<8 | CAN2_RxMsg.Data[1];
                g_stUndePan1.siCrtFB        = CAN2_RxMsg.Data[2]<<8 | CAN2_RxMsg.Data[3];
                //��CAn�����ж��������е�ǲ�ֵ���ڶ�ʱ���ж������ٶȻ�
                g_stUndePan1.stCoder.siDiffCode = g_stUndePan1.stCoder.siCode - g_stUndePan1.stCoder.siPreCode;
                g_stUndePan1.stCoder.siPreCode  = g_stUndePan1.stCoder.siCode;
                /*ƽ��ֵ�˲�*/
                UnderPan_DataFilter(&g_stUndePan1.stCoder.siDiffCode, &Flag1_Tmp, &Flag_Motor1_10us,Velt1_Tmp, &g_stUndePan1.fpVeltFB);
              	if (abs(g_stUndePan1.fpVeltFB) > 5000)
				{
					i++;	
				}

                break;
            case 0x202:
                g_stUndePan2.stCoder.siCode = CAN2_RxMsg.Data[0]<<8 | CAN2_RxMsg.Data[1];
                g_stUndePan2.siCrtFB        = CAN2_RxMsg.Data[2]<<8 | CAN2_RxMsg.Data[3];

                g_stUndePan2.stCoder.siDiffCode = g_stUndePan2.stCoder.siCode - g_stUndePan2.stCoder.siPreCode;
                g_stUndePan2.stCoder.siPreCode  = g_stUndePan2.stCoder.siCode;
                 /*ƽ��ֵ�˲�*/
				UnderPan_DataFilter(&g_stUndePan2.stCoder.siDiffCode, &Flag2_Tmp, &Flag_Motor2_10us,Velt2_Tmp, &g_stUndePan2.fpVeltFB);
                break; 
            case 0x203:
                g_stUndePan3.stCoder.siCode = CAN2_RxMsg.Data[0]<<8 | CAN2_RxMsg.Data[1];
                g_stUndePan3.siCrtFB        = CAN2_RxMsg.Data[2]<<8 | CAN2_RxMsg.Data[3];

                g_stUndePan3.stCoder.siDiffCode = g_stUndePan3.stCoder.siCode - g_stUndePan3.stCoder.siPreCode;
                g_stUndePan3.stCoder.siPreCode  = g_stUndePan3.stCoder.siCode;
                 /*ƽ��ֵ�˲�*/
				UnderPan_DataFilter(&g_stUndePan3.stCoder.siDiffCode, &Flag3_Tmp, &Flag_Motor3_10us,Velt3_Tmp, &g_stUndePan3.fpVeltFB);
                break;
            case 0x204:
                g_stUndePan4.stCoder.siCode = CAN2_RxMsg.Data[0]<<8 | CAN2_RxMsg.Data[1];
                g_stUndePan4.siCrtFB        = CAN2_RxMsg.Data[2]<<8 | CAN2_RxMsg.Data[3];

                g_stUndePan4.stCoder.siDiffCode = g_stUndePan4.stCoder.siCode - g_stUndePan4.stCoder.siPreCode;
                g_stUndePan4.stCoder.siPreCode  = g_stUndePan4.stCoder.siCode;
                 /*ƽ��ֵ�˲�*/
				UnderPan_DataFilter(&g_stUndePan4.stCoder.siDiffCode, &Flag4_Tmp, &Flag_Motor4_10us,Velt4_Tmp, &g_stUndePan4.fpVeltFB);

                break;
            /*��̨Pitch�������е��*/
            case 0x206:
                Angle_Pitch_Buf[pitch_cnt++] = CAN2_RxMsg.Data[0]<<8 | CAN2_RxMsg.Data[1];
                g_stPitch.siCrtFB            = CAN2_RxMsg.Data[2]<<8 | CAN2_RxMsg.Data[3];

				if (pitch_cnt == NUM)
				{
                    for(pitch_cnt = 0;pitch_cnt < NUM;pitch_cnt++)
				    	Pitch_Sum += Angle_Pitch_Buf[pitch_cnt];  /*�˲�*/
				    pitch_cnt =0;
					g_stPitch.fpPosFB = 1.0f*Pitch_Sum/NUM;
				}

                break;
            /*��̨Yaw�������е��*/
            case 0x205:
                FollowTmp = CAN2_RxMsg.Data[0]<<8 | CAN2_RxMsg.Data[1];
				Error = FollowTmp - FollowTmpPre;
				if(Error < 5000 && Error > -5000)
				{g_Coder_Follow += Error;}
				FollowTmpPre = FollowTmp;
//				if(FollowTmp > 5000)
//				    
//					g_Coder_Follow = FollowTmp - 8192;
//				else 
//				    g_Coder_Follow = FollowTmp; 

                break;
            default:
                break;
        }   
    }
    //����жϱ�־λ
    CAN_ClearITPendingBit(CAN2, CAN_IT_FF0);

	OSIntExit();
}




/***************************************************************************************
��������EXTI4_IRQHandler
���ܣ�  MPU6050 �ⲿ�ж�
***************************************************************************************/
/*ƽ���˲���*/
#define N 12
SINT32 Flag_PY = 0;
FP32 Pitch_Velt_Buf[N];
FP32 Yaw_Velt_Buf[N];
extern bool InitComplete_Flag;
void EXTI4_IRQHandler(void)
{  
    /*�˲��ֲ�����*/
    FP32 Pitch_Velt_Sum = 0;
    FP32 Yaw_Velt_Sum   = 0;
    SINT32 i = 0; 

	CPU_SR         cpu_sr;	
	OS_CRITICAL_ENTER();                         
    OSIntEnter();
    OS_CRITICAL_EXIT();

    if(EXTI_GetITStatus(EXTI_Line4) != RESET)
    {        
        MPU6050_ReadData();
        Pitch_Velt_Buf[Flag_PY++] = MPU6050_Real_Data.Gyro_Y * 6;//r/min
        Yaw_Velt_Buf[Flag_PY++]   = MPU6050_Real_Data.Gyro_Z * 6;
        if (Flag_PY == N)
        {
            for (i=1;i<N;i++)
            {
                Pitch_Velt_Sum += Pitch_Velt_Buf[i];
                Yaw_Velt_Sum   += Yaw_Velt_Buf[i];
            }
            /*Pitch�����ٶ�����*/
            if (abs(Pitch_Velt_Sum/11.0f) >= 1.0f)
                g_stPitch.fpVeltFB = Pitch_Velt_Sum/11.0f;
            else
                g_stPitch.fpVeltFB = 0;
            /*Yaw�����ٶ�����*/
            if (abs(Yaw_Velt_Sum/11.0f) >= 1.0f)
                g_stYaw.fpVeltFB = -Yaw_Velt_Sum/11.0f;
            else
                g_stYaw.fpVeltFB = 0;

            Flag_PY = 0;         
        }
        EXTI_ClearFlag(EXTI_Line4);          //�����־λ
        EXTI_ClearITPendingBit(EXTI_Line4);
    }
	OSIntExit();
}
