#include "main.h"
#include "Bsp_Init.h"
#include "Led.h"
#include "Delay.h"
#include "Debug_Task.h"
#include "flash.h"
#include "os.h"
#include "Gimbal_Task.h"
#include "UnderPan_Task.h"

//�������ȼ�
#define START_TASK_PRIO		3
//�����ջ��С	
#define START_STK_SIZE 		128
//������ƿ�
OS_TCB StartTaskTCB;
//�����ջ	
CPU_STK START_TASK_STK[START_STK_SIZE];
//������
void start_task(void *p_arg);

/***************LED����*****************/
//�������ȼ�
#define LED_TASK_PRIO		6
//�����ջ��С	
#define LED_STK_SIZE 		128
//������ƿ�
OS_TCB Led_TaskTCB;
//�����ջ	
CPU_STK LED_TASK_STK[LED_STK_SIZE];
//������
void led_task(void *p_arg);

/***************��̨����*****************/
//�������ȼ�
#define GIMBAL_TASK_PRIO		4
//�����ջ��С	
#define GIMBAL_STK_SIZE 		128
//������ƿ�
OS_TCB Gimbal_TaskTCB;
//�����ջ	
CPU_STK Gimbal_TASK_STK[LED_STK_SIZE];
//������
void gimbal_task(void *p_arg);

/***************��������*****************/
//�������ȼ�
#define UNDERPAN_TASK_PRIO		5
//�����ջ��С	
#define UNDERPAN_STK_SIZE 		128
//������ƿ�
OS_TCB UnderPan_TaskTCB;
//�����ջ	
CPU_STK UnderPan_TASK_STK[LED_STK_SIZE];
//������
void underpan_task(void *p_arg);

bool InitComplete_Flag = FALSE;

int main(void)
{
	OS_ERR err;
	CPU_SR_ALLOC();

    BSP_Configuration();    
	InitComplete_Flag = TRUE;
	
	OSInit(&err);		    //��ʼ��UCOSIII
	OS_CRITICAL_ENTER();	//�����ٽ���

	//������ʼ����
	OSTaskCreate((OS_TCB 	* )&StartTaskTCB,		//������ƿ�
				 (CPU_CHAR	* )"start task", 		//��������
                 (OS_TASK_PTR )start_task, 			//������
                 (void		* )0,					//���ݸ��������Ĳ���
                 (OS_PRIO	  )START_TASK_PRIO,     //�������ȼ�
                 (CPU_STK   * )&START_TASK_STK[0],	//�����ջ����ַ
                 (CPU_STK_SIZE)START_STK_SIZE/10,	//�����ջ�����λ
                 (CPU_STK_SIZE)START_STK_SIZE,		//�����ջ��С
                 (OS_MSG_QTY  )0,					//�����ڲ���Ϣ�����ܹ����յ������Ϣ��Ŀ,Ϊ0ʱ��ֹ������Ϣ
                 (OS_TICK	  )0,					//��ʹ��ʱ��Ƭ��תʱ��ʱ��Ƭ���ȣ�Ϊ0ʱΪĬ�ϳ���
                 (void   	* )0,					//�û�����Ĵ洢��
                 (OS_OPT      )OS_OPT_TASK_STK_CHK|OS_OPT_TASK_STK_CLR, //����ѡ��
                 (OS_ERR 	* )&err);				//��Ÿú�������ʱ�ķ���ֵ

	OS_CRITICAL_EXIT();	//�˳��ٽ���	 
	OSStart(&err);      //����UCOSIII
    while(1);	   
}

void start_task(void *p_arg)
{
	OS_ERR err;
	CPU_SR_ALLOC();
	p_arg = p_arg;

	CPU_Init();

	OS_CRITICAL_ENTER();	//�����ٽ���

	//����LED����
	OSTaskCreate((OS_TCB 	* )&Led_TaskTCB,		
				 (CPU_CHAR	* )"led task", 		
                 (OS_TASK_PTR )led_task, 			
                 (void		* )0,					
                 (OS_PRIO	  )LED_TASK_PRIO,     
                 (CPU_STK   * )&LED_TASK_STK[0],	
                 (CPU_STK_SIZE)LED_STK_SIZE/10,	
                 (CPU_STK_SIZE)LED_STK_SIZE,		
                 (OS_MSG_QTY  )0,					
                 (OS_TICK	  )0,					
                 (void   	* )0,					
                 (OS_OPT      )OS_OPT_TASK_STK_CHK|OS_OPT_TASK_STK_CLR,
                 (OS_ERR 	* )&err);
				 
	//������̨����
	OSTaskCreate((OS_TCB 	* )&Gimbal_TaskTCB,		
				 (CPU_CHAR	* )"gimbal task", 		
	             (OS_TASK_PTR )gimbal_task, 			
	             (void		* )0,					
	             (OS_PRIO	  )GIMBAL_TASK_PRIO,     
	             (CPU_STK   * )&Gimbal_TASK_STK[0],	
	             (CPU_STK_SIZE)GIMBAL_STK_SIZE/10,	
	             (CPU_STK_SIZE)GIMBAL_STK_SIZE,		
	             (OS_MSG_QTY  )0,					
	             (OS_TICK	  )0,					
	             (void   	* )0,					
	             (OS_OPT      )OS_OPT_TASK_STK_CHK|OS_OPT_TASK_STK_CLR,
	             (OS_ERR 	* )&err);				

	//������̨����
	OSTaskCreate((OS_TCB 	* )&UnderPan_TaskTCB,		
				 (CPU_CHAR	* )"underpan task", 		
	             (OS_TASK_PTR )underpan_task, 			
	             (void		* )0,					
	             (OS_PRIO	  )UNDERPAN_TASK_PRIO,     
	             (CPU_STK   * )&UnderPan_TASK_STK[0],	
	             (CPU_STK_SIZE)UNDERPAN_STK_SIZE/10,	
	             (CPU_STK_SIZE)UNDERPAN_STK_SIZE,		
	             (OS_MSG_QTY  )0,					
	             (OS_TICK	  )0,					
	             (void   	* )0,					
	             (OS_OPT      )OS_OPT_TASK_STK_CHK|OS_OPT_TASK_STK_CLR,
	             (OS_ERR 	* )&err);
	OS_CRITICAL_EXIT();	//�˳��ٽ���
	OSTaskDel((OS_TCB*)0,&err);	//ɾ��start_task��������
}

void led_task(void *p_arg)
{
  	OS_ERR err;
	p_arg = p_arg;
	while(1)
	{
	 	LED_GREEN_TOGGLE();
		OSTimeDlyHMSM(0,0,0,500,OS_OPT_TIME_HMSM_STRICT,&err); //��ʱ500ms�������������л�
	}
}

void gimbal_task(void *p_arg)
{
	OS_ERR err;
	p_arg = p_arg;
	while(1)
	{
		if (InitComplete_Flag)
	 	Gimbal_Control();
		OSTimeDlyHMSM(0,0,0,5,OS_OPT_TIME_HMSM_STRICT,&err); //��ʱ500ms�������������л�
	}
}

void underpan_task(void *p_arg)
{
	OS_ERR err;
	p_arg = p_arg;
	while(1)
	{
	 	Underpan_Control();
		OSTimeDlyHMSM(0,0,0,5,OS_OPT_TIME_HMSM_STRICT,&err); //��ʱ500ms�������������л�
	}
}



