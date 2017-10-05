/**
******************************************************************************
* @file	    main.c
* @author	  Action
* @version  V1.0.0
* @date	    2017/07/24
* @brief	  Whirling Death(死亡旋风)...
******************************************************************************
* @attention
*			None
******************************************************************************/
#include "config.h"
static int Startflag=1;
Robot_t gRobot={0};
//#define TEST 
int main(void)
{
	//int OSCPUUsage=0; 
	
	int left=0;
	int right=0;
	robotInit();
	USART_OUT(UART5,"initsucess");
	GPIO_ResetBits(GPIOE,GPIO_Pin_7);
  while(LaserStart());
  while (1)
  {
    while (getTimeFlag())                              //10ms执行进入一次
    {	
			left=GPIO_ReadInputDataBit(GPIOE, GPIO_Pin_2);
			right=GPIO_ReadInputDataBit(GPIOC,GPIO_Pin_0);
			
			  USART_OUT(UART5,"s=%d\t",(int)gRobot.status);
				USART_OUT(UART5,"%d\t",(int)gRobot.abnormal);
				USART_OUT(UART5,"ai=%d\t",(int)gRobot.walk_t.pid.aimAngle);
//				USART_OUT(UART5,"%d\t",(int)getLeftAdc());
//	  		USART_OUT(UART5,"%d\t",(int)getRightAdc());
				USART_OUT(UART5,"%d\t",(int)gRobot.shoot_t.sReal.speed);
				USART_OUT(UART5,"%d\t",(int)gRobot.walk_t.pos.x);
				USART_OUT(UART5,"%d\t",(int)gRobot.walk_t.pos.y);
				USART_OUT(UART5,"%d\t",(int)gRobot.walk_t.circleChange.linenum);
				USART_OUT(UART5,"%d\t",(int)gRobot.walk_t.circleChange.turnTime);
		    USART_OUT(UART5,"%d\t",(int)gRobot.walk_t.laser.status);
			  USART_OUT(UART5,"%d\t",(int)gRobot.walk_t.circleChange.quadrant);
				USART_OUT(UART5,"%d\t",(int)gRobot.walk_t.circleChange.quadrantlast);
				USART_OUT(UART5,"%d\t",(int)gRobot.walk_t.circleChange.direction);
			  USART_OUT(UART5,"l=%d\t",(int)left);
				USART_OUT(UART5,"r=%d\t",(int)right);
			  USART_OUT(UART5,"r=%d\t",(int) gRobot.camera_t.camrBaseWalk_t.circleChange.circleNum);
			//USART_OUT(UART5,"os%d\t",(int)OSCPUUsage);
				USART_OUT(UART5,"%d\t\r\n",(int)gRobot.walk_t.circleChange.circleNum); 
			 #ifdef TEST 
				TestMode();
			 #else
			MotorRead();  
			if(gRobot.avoid_t.signal>6000||Startflag==1)
			{
				 Startflag=0;
				 gRobot.status|=STATUS_AVOID_JUDGE;
			}
			//1在处理异常  为0就判断
			if(gRobot.status&STATUS_AVOID_JUDGE)
			{
				AbnormityJudge();
			}
			if(gRobot.status&STATUS_AVOID_HANDLE)
			{
				AbnormityHandle();
			}else if (gRobot.status & STATUS_SWEEP)
			{
				Run();
			}
			else if (gRobot.status & STATUS_FIX)
			{
				FixTask();
			}
			else if (gRobot.status & STATUS_SHOOTER)
			{
				fireTask();
			}	 
			else if(gRobot.status & STATUS_CAMERA_WALK)
			{
				CameraBaseWalk2();
			}
			#endif
		}
		
	//OSCPUUsage=getTimeCount();
}
}

