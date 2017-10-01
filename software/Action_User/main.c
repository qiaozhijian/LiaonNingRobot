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
extern int tempcirclerem;
int a=0;
float left=0;
float right=0;
Robot_t gRobot={0};
int main(void)
{
	robotInit();
	while(LaserStart());
	while (1)
	{
		while (getTimeFlag())                              //10ms执行进入一次
		{	
			left=Get_Adc_Average(ADC_Channel_15, 200);
			right=Get_Adc_Average(ADC_Channel_14, 200);
			USART_OUT(UART5,"%d\t", (int)left);
			USART_OUT(UART5,"%d\r\n", (int)right);
//			a=getBallColor();
//			USART_OUT(UART5,"%d\t", (int)gRobot.status);
//			USART_OUT(UART5,"%d\t",(int)gRobot.walk_t.circleChange.turnTime);
//			USART_OUT(UART5,"%d\t",(int)gRobot.walk_t.pos.angle);
//		  USART_OUT(UART5,"%d\t",(int)gRobot.walk_t.pos.x);
//		  USART_OUT(UART5,"%d\t",(int)gRobot.walk_t.pos.y);
//			USART_OUT(UART5,"%d\t",(int)LineCheck(gRobot.walk_t.circleChange.direction));
//   		USART_OUT(UART5,"%d\t",(int)gRobot.collect_t.PhotoElectric.ballcount);
//		  USART_OUT(UART5,"%d\t",(int)gRobot.camera_t.camrBaseWalk_t.circlechange.turntime);
//		  USART_OUT(UART5,"%d\t",(int)gRobot.walk_t.circlechange.direction);
//		  USART_OUT(UART5,"%d\t",(int)gRobot.camera_t.camrBaseWalk_t.circlechange.turntime);
//			USART_OUT(UART5,"%d\t",(int)gRobot.walk_t.circlechange.linenum);
//			USART_OUT(UART5,"%d\t",(int)gRobot.walk_t.pid.aimAngle );
//			USART_OUT(UART5,"%d\t",(int)gRobot.walk_t.pid.angleError);
//		  USART_OUT(UART5,"%d\t",(int)gRobot.walk_t.pid.disError);
//			USART_OUT(UART5,"%d\t",(int)gRobot.walk_t.pid.distanceStraight);
//			USART_OUT(UART5,"%d\t",(int)tempcirclerem);
//			USART_OUT(UART5,"%d\t",(int)gRobot.shoot_t.sReal.speed);
//			USART_OUT(UART5,"%d\r\n",(int)gRobot.walk_t.circlechange.circlenum);
//			         MotorRead();       
//						if (gRobot.status & STATUS_SWEEP)
//						{
//							 Run();
//						}
//						else if (gRobot.status & STATUS_FIX)
//						{
//							FixTask();
//						}
//						else if (gRobot.status & STATUS_SHOOTER)
//						{
//							fireTask();
//						}	
//						else if(gRobot.status&STATUS_AVOID)
//						{
//							Escape();
//						}	
//						else if(gRobot.status & STATUS_CAMERA_WALK)
//						{
//							//CameraBaseWalk3();
//							CameraBaseWalk2();
//						}
////						else if (gRobot.status & STATUS_CAMERA)
////						{
////							Findball_5();
////						}
//					  if(gRobot.avoid_t.signal)
//						{
//							//CheckOutline3();
//							//CheckOutline();
//							CheckOutline2();	
//						}
/*****************************************临时测试*****************************************/
//USART_OUT(UART5,"%d\t",(int)gRobot.camera_t.camerapid.aimAngle);
//USART_OUT(UART5,"%d\t",(int)gRobot.walk_t.pos.angle);
//USART_OUT(UART5,"%d\t",(int)gRobot.walk_t.pos.x);
//USART_OUT(UART5,"%d\t",(int)gRobot.walk_t.pos.y);
//USART_OUT(UART5,"%d\t",(int)gRobot.collect_t.PhotoElectric.ballcount);						
//USART_OUT(UART5,"%d\r\n",(int)gRobot.avoid_t.signal);
//						
//USART_OUT(UART5,"%d\t",(int)gRobot.avoid_t.passflag);	
//USART_OUT(UART5,"%d\t",(int)angleErrorCount(gRobot.avoid_t.pid.aimAngle,gRobot.walk_t.pos.angle));
//						
//USART_OUT(UART5,"%d\t",(int)gRobot.walk_t.pos.angle);
//USART_OUT(UART5,"%d\t",(int)gRobot.walk_t.pos.x);
//USART_OUT(UART5,"%d\t",(int)gRobot.walk_t.pos.y);			
//						
//USART_OUT(UART5,"%d\t",(int)gRobot.avoid_t.pid.aimAngle);	
////USART_OUT(UART5,"%d\t%d\t%d\t%d\t",(int)gRobot.walk_t.right.real,gRobot.status,gRobot.avoid_t.signal,(int)gRobot.walk_t.right.aim);
//USART_OUT(UART5,"%d\r\n",(int)gRobot.walk_t.circleChange.turnTime);
//USART_OUT(UART5,"%d\r\n",(int)gRobot.walk_t.circleChange.turnTimerem);
//USART_OUT(UART5,"%d\r\n",(int)gRobot.status);
//if(LimitTurn(gRobot.walk_t.pos.x,gRobot.walk_t.pos.y))
//	{
//		change=1;
//	}
//if(change==1)
//	{
//		CameraBaseWalk3();
//		change=0;
//	}
//if(change==0)
//	{
//		Findball_5();
//	}
			//			WalkTask1();
			//			WalkTask2();
			//			逆时针旋转
			//       CirlceSweep();
			//			NiShiZhenCircleBiHuan(1200,1600,2400,2400);
			//			USART_OUTF(Key1);
			
			//			USART_OUTF(Key2);
			//			fireTask();
			//      Sub_Box();
			//			Findball_3();
			//			Findball_5();
      //			Debug();
		  //USART_OUT(UART5,"%d\r\n",(int)gRobot.shoot_t.sReal.speed);
/*****************************************临时测试*****************************************/			
		}
	}
}

