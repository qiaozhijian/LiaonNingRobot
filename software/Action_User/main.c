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
int main(void)
{
	robotInit();
	while(LaserStart());
	while (1)
	{
		while (getTimeFlag())                              //10ms执行进入一次
		{	
			JudgeStick();
//			      MotorRead();   
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
//							BackCar(gRobot.walk_t.pos.angle);
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
//USART_OUT(UART5,(uint8_t*)"%d\t",(int)gRobot.camera_t.camerapid.aimAngle);
//USART_OUT(UART5,(uint8_t*)"%d\t",(int)gRobot.walk_t.pos.angle);
//USART_OUT(UART5,(uint8_t*)"%d\t",(int)gRobot.walk_t.pos.x);
//USART_OUT(UART5,(uint8_t*)"%d\t",(int)gRobot.walk_t.pos.y);
//USART_OUT(UART5,(uint8_t*)"%d\t",(int)gRobot.collect_t.PhotoElectric.ballcount);						
//USART_OUT(UART5,(uint8_t*)"%d\r\n",(int)gRobot.avoid_t.signal);
//						
//USART_OUT(UART5,(uint8_t*)"%d\t",(int)gRobot.avoid_t.passflag);	
//USART_OUT(UART5,(uint8_t*)"%d\t",(int)angleErrorCount(gRobot.avoid_t.pid.aimAngle,gRobot.walk_t.pos.angle));
//						
//USART_OUT(UART5,(uint8_t*)"%d\t",(int)gRobot.walk_t.pos.angle);
//USART_OUT(UART5,(uint8_t*)"%d\t",(int)gRobot.walk_t.pos.x);
//USART_OUT(UART5,(uint8_t*)"%d\t",(int)gRobot.walk_t.pos.y);			
//						
//USART_OUT(UART5,(uint8_t*)"%d\t",(int)gRobot.avoid_t.pid.aimAngle);	
////USART_OUT(UART5,(uint8_t*)"%d\t%d\t%d\t%d\t",(int)gRobot.walk_t.right.real,gRobot.status,gRobot.avoid_t.signal,(int)gRobot.walk_t.right.aim);
//USART_OUT(UART5,(uint8_t*)"%d\r\n",(int)gRobot.walk_t.circlechange.turntime);
//USART_OUT(UART5,(uint8_t*)"%d\r\n",(int)gRobot.walk_t.circlechange.turntimerem);
//USART_OUT(UART5,(uint8_t*)"%d\r\n",(int)gRobot.status);
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
		  //USART_OUT(UART5,(uint8_t*)"%d\r\n",(int)gRobot.shoot_t.real.speed);
/*****************************************临时测试*****************************************/			
		}
	}
}

