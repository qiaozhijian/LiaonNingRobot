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
Robot_t gRobot={0};
//#define TEST 
extern float Yxpos,Yypos,Yangle,avel;
int main(void)
{
	//int OSCPUUsage=0; 
	static int openTime=0;
	int left=0;
	int right=0;
	robotInit();
	USART_OUT(UART5,"initsucess\r\n");
	GPIO_ResetBits(GPIOE,GPIO_Pin_7);
  while(LaserStart());
	gRobot.start=1;
	//driveGyro();
  while (1)
  {
    while (getTimeFlag())                              //10ms执行进入一次
    {
			  left=GPIO_ReadInputDataBit(GPIOE, GPIO_Pin_2);
			  right=GPIO_ReadInputDataBit(GPIOC,GPIO_Pin_0);
				USART_OUT(UART5,"h%d\t",(int)gRobot.avoid_t.handleEnd);
			  USART_OUT(UART5,"s=%d\t",(int)gRobot.status);
				USART_OUT(UART5,"%d\t",(int)gRobot.abnormal);
				USART_OUT(UART5,"ai=%d\t",(int)gRobot.walk_t.pid.aimAngle);
				USART_OUT(UART5,"%d\t",(int)getLeftAdc());
	  		USART_OUT(UART5,"%d\t",(int)getRightAdc());
				USART_OUT(UART5,"%d\t",(int)getRightAdc()+getLeftAdc());
				USART_OUT(UART5,"%d\t\r\n",(int)gRobot.shoot_t.sReal.speed);
//				USART_OUTF(gRobot.walk_t.pos.angle);
//				USART_OUTF(gRobot.walk_t.pos.x);
//				USART_OUTF(gRobot.walk_t.pos.y);
//				USART_OUT_CHAR("\r\n");
				USART_OUT(UART5,"gA=%d\t",(int)gRobot.walk_t.pos.angle);
				USART_OUT(UART5,"gX=%d\t",(int)gRobot.walk_t.pos.x);
		    USART_OUT(UART5,"gY=%d\t\r\n",(int)gRobot.walk_t.pos.y);
				USART_OUT(UART5,"%d\t",(int)gRobot.walk_t.circleChange.linenum);
				USART_OUT(UART5,"%d\t",(int)gRobot.walk_t.circleChange.turnTime);
		    USART_OUT(UART5,"%d\t",(int)gRobot.walk_t.laser.status);
			  USART_OUT(UART5,"%d\t" ,(int)gRobot.walk_t.circleChange.quadrant);
				USART_OUT(UART5,"%d\t",(int)gRobot.walk_t.circleChange.quadrantlast);
				USART_OUT(UART5,"%d\t",(int)gRobot.walk_t.circleChange.direction);
			  USART_OUT(UART5,"l=%d\t",(int)left);
				USART_OUT(UART5,"r=%d\t",(int)right);
				USART_OUT(UART5,"%d\t\r\n",(int)avel);
				USART_OUT(UART5,"x=%d\t",(int)Yxpos);
    		USART_OUT(UART5,"y=%d\t",(int)Yypos);
				USART_OUT(UART5,"%d\t",(int)gRobot.fix_t.toBorder);
				USART_OUT(UART5,"an=%d\t\r\n",(int)Yangle);
				USART_OUT(UART5,"%d\t",(int)gRobot.fix_t.wayChoose);
			  USART_OUT(UART5,"c=%d\t",(int) gRobot.camera_t.camrBaseWalk_t.circleChange.circleNum);
			//USART_OUT(UART5,"os%d\t",(int)OSCPUUsage);
				USART_OUT(UART5,"%d\t\r\n",(int)gRobot.walk_t.circleChange.circleNum); 
			 #ifdef TEST 
				TestMode();
				MotorRead(); 
			 #else
			MotorRead(); 
			CornerJammedJudge();
			if((gRobot.status&STATUS_AVOID_HANDLE)==0&&gRobot.avoid_t.handleEnd==1)
			{
				openTime++;
				if(openTime>100)
				{
					gRobot.status|=STATUS_AVOID_JUDGE;
					openTime=0;
					gRobot.avoid_t.handleEnd=0;
				}
			}else
			{
				openTime=0;
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
			}else if(gRobot.status & STATUS_PARKING)
			{
				//Pointparking(gRobot.ParkingPoint.x,gRobot.ParkingPoint.y);
				Pointparking2();
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
				Out2In();
				//CameraBaseWalk2();
			}
			#endif
 		}	
	//OSCPUUsage=getTimeCount();
}
}