#ifndef __RM_MOTORCTRLTYPES_H__
#define __RM_MOTORCTRLTYPES_H__
#include <stm32f4xx.h>
#include "RM_Types.h"

/*��������״̬*/
typedef enum
{
	FIRST,		    //�״�����
	RUN		    	//�Ѿ�����
}EM_RUN_STATE;      

/*������״̬*/
typedef enum
{
	OFF,
	ON
}EM_DRIVER_STATE;
 
/*����ģʽ*/ 
typedef enum
{
    OPEN_LOOP,     //��������
	POS_LOOP,      //λ�ÿ���
	VELT_LOOP,     //�ٶȿ���
	CURRENT_LOOP,  //��������
}EM_CTRL_MODE;

/*��/����*/
typedef enum
{
	UNLOCK,	    	//����
	LOCK		    //����		  
}EM_LOCK_STATE;	    

/*�����صĲ���*/
typedef struct
{
	FP32     Acc;       //���ٶ�,��λ: r/s^2
    FP32     I;         //���ٱ�
	USHORT16 NE_ABS;    //����ʽ��������
    USHORT16 NE_INC;    //����ʽ��������
	USHORT16 MaxI;      //������,��λmA
	USHORT16 CtnI;      //���������������
    USHORT16 MaxV;      //��ߵ�ѹ����
	USHORT16 MinV;      //��͵�ѹ����
}ST_MOTOR_PARA;

/*����PID�������ݽṹ*/
typedef struct 
{
	FP32 KP;		    //����ϵ��KP
	FP32 KI;		    //����ϵ��KI
	FP32 KD;		    //΢��ϵ��KD
	FP32 E;			  	//����ƫ��
	FP32 PreE;		  	//�ϴ�ƫ��
	FP32 PrePreE;		//���ϴ�ƫ��
    FP32 SumE;          //��ƫ��
	FP32 U;	  			//����PID������
    SINT32 UMax;        //PID����������
    SINT32 UMin;        //PID�����С����
	SINT32 ELimit;      //�����ַ�������ʱ�ļ���ƫ��
	SINT32 ULimit;		//����������ʱ������ֵ
	SINT32 EDead;			//�������
}ST_PID;     

/*���̽ṹ��*/
typedef struct
{
    SINT32 siCode;   	    //��ǰ���̶���
	SINT32 siPreCode;	    //��һ�����̶���
	SINT32 siDiffCode;  	//���̶�����ֵ
	SINT32 siCycleCode;     //ת����Ȧ��
}ST_CODER;

typedef struct
{
	UCHAR8 SendPosFlag		:1;
	UCHAR8 SendVeltFlag	    :1;
	UCHAR8 SendCrtFlag		:1;
}ST_SEND_FLAG;

/*���Ŀ��ƽṹ��*/
typedef struct
{
	UINT32          SlaveAddr;       //�Ӷ�����ַ
	UINT32          MasterAddr;      //��������ַ
	EM_RUN_STATE    emRunState;      //�����˶�״̬
	EM_DRIVER_STATE emDriverState;   //������״̬
	EM_CTRL_MODE    emCtrlMode;      //�������ģʽ
	EM_LOCK_STATE   emLockState;     //��������״̬

	ST_MOTOR_PARA   stMotorPara;     //�������
    ST_CODER        stCoder;         //���̽ṹ��
	ST_PID          stPosPID;        //λ��PID
	ST_PID          stVeltPID;       //�ٶ�PID
	ST_PID          stCrtPID;        //����PID

    FP32            fpPosDes;        //Ŀ��λ��
    FP32            fpPosFB;         //λ�÷���
    FP32            fpVeltDes;       //Ŀ���ٶ�
	FP32            fpVeltFB;        //�ٶȷ���
    SINT32          siCrtDes;        //Ŀ�����
	SINT32          siCrtFB;         //��������

    SINT32          MaxPos;          //λ�û��������λ������
	SINT32          MinPos;          //λ�û�������Сλ������
	SINT32          MaxVelt;         //�ٶȻ���������ٶ�����
	SINT32          MinVelt;         //�ٶȻ�������С�ٶ�����
    SINT32          MaxCrt;          //�ٶȻ���������ٶ�����
	SINT32          MinCrt;          //�ٶȻ�������С�ٶ�����
}ST_MOTOR_CTRL;

typedef enum 
{
	DEBUG_MODE,
	NORMAL_MODE
}EN_WORK_MODE;

#endif
