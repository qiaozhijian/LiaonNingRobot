#include  <includes.h>
#include  <app_cfg.h>
#include "misc.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_rcc.h"
#include "timer.h"
#include "stm32f4xx_it.h"
#include "gpio.h"
#include "usart.h"
#include "can.h"
#include "elmo.h"
#include "math.h"
#include "input.h"
#include "output.h"
#include "route_ctr.h"
#include "stm32f4xx_usart.h"
#include "stdio.h"
#include "arm_math.h"
//////////////////Area of defining semaphore////////////////////////
 OS_EVENT 		*PeriodSem;

static  OS_STK  App_ConfigStk[Config_TASK_START_STK_SIZE];
static  OS_STK  WalkTaskStk[Walk_TASK_STK_SIZE];
static 	void  ConfigTask(void);
static 	void  WalkTask(void);
void App_Task()
{
	CPU_INT08U  os_err;
	os_err = os_err; /* prevent warning... */
	
	/******************Create Semaphore***********************/
   PeriodSem				=	OSSemCreate(0);

  /******************Create Task**************************/	
	os_err = OSTaskCreate(	(void (*)(void *)) ConfigTask,					//Initial Task
	                      	(void          * ) 0,							
													(OS_STK        * )&App_ConfigStk[Config_TASK_START_STK_SIZE-1],		
													(INT8U           ) Config_TASK_START_PRIO  );	
						
													
	os_err = OSTaskCreate(	(void (*)(void *)) WalkTask,					
	                      	(void          * ) 0,							
													(OS_STK        * )&WalkTaskStk[Walk_TASK_STK_SIZE-1],		
													(INT8U           ) Walk_TASK_PRIO  );													
}
void ConfigTask(void)
{	
	CPU_INT08U  os_err;	
	os_err = os_err;
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);	
	  
	TIM_Init(TIM2,999,839,0,0);					//主周期定时10ms	
	
	USART3_Init(115200);
	UART5_Init(115200);		//调试用蓝牙
	
	KeyInit();//按住为1
	
//	CAN_Config(CAN1,500,GPIOB,GPIO_Pin_8, GPIO_Pin_9);
//	TIM_Delayms(TIM5,50);
//	elmo_Init();
////	
//	elmo_Disable(1);
//	elmo_Disable(2);
//	elmo_Disable(3);

////	
////	Vel_cfg(1,300000,300000);

////1	
//	Pos_cfg(1,300000,300000,10000);
//	Pos_cfg(2,300000,300000,10000);
//	Pos_cfg(3,300000,300000,10000);
//	
//	PosCrl(1,0,-500);
//	PosCrl(2,0,0000);
//	PosCrl(3,0,0000);

//	
//	Vel_cfg(10,300000,300000);	//后 发射 

//	TIM_Delayms(TIM5,50);
	OSTaskSuspend(OS_PRIO_SELF);
}


//static int pos[3]={0};
void WalkTask(void)
{
	CPU_INT08U  os_err;
	os_err=os_err;
  OSSemSet(PeriodSem,0,&os_err);

	while(1)
	{
		OSSemPend(PeriodSem,0,&os_err); 
		//	ReadActualVel(3);    
	//	ReadActualPos(1);
	
	//	ReadActualPos(3);
	 RouteInput();
	//ReadActualPos(1);
	//		ReadActualPos(2);
	//		ReadActualPos(3);
//		pos[0]=GetPos1();
//		pos[1]=GetPos2();
//		pos[2]=GetPos3();
	//	RouteControl();
		//	RouteOutput();

	} 
}	


