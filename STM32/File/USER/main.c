#include "main.h"
#include "Bsp_Init.h"
#include "Led.h"
#include "Delay.h"
#include "Debug_Task.h"
#include "flash.h"
#include "os.h"
#include "Gimbal_Task.h"
#include "UnderPan_Task.h"

//任务优先级
#define START_TASK_PRIO		3
//任务堆栈大小	
#define START_STK_SIZE 		128
//任务控制块
OS_TCB StartTaskTCB;
//任务堆栈	
CPU_STK START_TASK_STK[START_STK_SIZE];
//任务函数
void start_task(void *p_arg);

/***************LED任务*****************/
//任务优先级
#define LED_TASK_PRIO		6
//任务堆栈大小	
#define LED_STK_SIZE 		128
//任务控制块
OS_TCB Led_TaskTCB;
//任务堆栈	
CPU_STK LED_TASK_STK[LED_STK_SIZE];
//任务函数
void led_task(void *p_arg);

/***************云台任务*****************/
//任务优先级
#define GIMBAL_TASK_PRIO		4
//任务堆栈大小	
#define GIMBAL_STK_SIZE 		128
//任务控制块
OS_TCB Gimbal_TaskTCB;
//任务堆栈	
CPU_STK Gimbal_TASK_STK[LED_STK_SIZE];
//任务函数
void gimbal_task(void *p_arg);

/***************底盘任务*****************/
//任务优先级
#define UNDERPAN_TASK_PRIO		5
//任务堆栈大小	
#define UNDERPAN_STK_SIZE 		128
//任务控制块
OS_TCB UnderPan_TaskTCB;
//任务堆栈	
CPU_STK UnderPan_TASK_STK[LED_STK_SIZE];
//任务函数
void underpan_task(void *p_arg);

bool InitComplete_Flag = FALSE;

int main(void)
{
	OS_ERR err;
	CPU_SR_ALLOC();

    BSP_Configuration();    
	InitComplete_Flag = TRUE;
	
	OSInit(&err);		    //初始化UCOSIII
	OS_CRITICAL_ENTER();	//进入临界区

	//创建开始任务
	OSTaskCreate((OS_TCB 	* )&StartTaskTCB,		//任务控制块
				 (CPU_CHAR	* )"start task", 		//任务名字
                 (OS_TASK_PTR )start_task, 			//任务函数
                 (void		* )0,					//传递给任务函数的参数
                 (OS_PRIO	  )START_TASK_PRIO,     //任务优先级
                 (CPU_STK   * )&START_TASK_STK[0],	//任务堆栈基地址
                 (CPU_STK_SIZE)START_STK_SIZE/10,	//任务堆栈深度限位
                 (CPU_STK_SIZE)START_STK_SIZE,		//任务堆栈大小
                 (OS_MSG_QTY  )0,					//任务内部消息队列能够接收的最大消息数目,为0时禁止接收消息
                 (OS_TICK	  )0,					//当使能时间片轮转时的时间片长度，为0时为默认长度
                 (void   	* )0,					//用户补充的存储区
                 (OS_OPT      )OS_OPT_TASK_STK_CHK|OS_OPT_TASK_STK_CLR, //任务选项
                 (OS_ERR 	* )&err);				//存放该函数错误时的返回值

	OS_CRITICAL_EXIT();	//退出临界区	 
	OSStart(&err);      //开启UCOSIII
    while(1);	   
}

void start_task(void *p_arg)
{
	OS_ERR err;
	CPU_SR_ALLOC();
	p_arg = p_arg;

	CPU_Init();

	OS_CRITICAL_ENTER();	//进入临界区

	//创建LED任务
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
				 
	//创建云台任务
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

	//创建云台任务
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
	OS_CRITICAL_EXIT();	//退出临界区
	OSTaskDel((OS_TCB*)0,&err);	//删除start_task任务自身
}

void led_task(void *p_arg)
{
  	OS_ERR err;
	p_arg = p_arg;
	while(1)
	{
	 	LED_GREEN_TOGGLE();
		OSTimeDlyHMSM(0,0,0,500,OS_OPT_TIME_HMSM_STRICT,&err); //延时500ms，并进行任务切换
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
		OSTimeDlyHMSM(0,0,0,5,OS_OPT_TIME_HMSM_STRICT,&err); //延时500ms，并进行任务切换
	}
}

void underpan_task(void *p_arg)
{
	OS_ERR err;
	p_arg = p_arg;
	while(1)
	{
	 	Underpan_Control();
		OSTimeDlyHMSM(0,0,0,5,OS_OPT_TIME_HMSM_STRICT,&err); //延时500ms，并进行任务切换
	}
}



