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
	gRobot.status=25;
	robotInit();

	while (1)
	{
		while (getTimeFlag())                              //10ms执行进入一次
		{	
			      MotorRead();      
						if (gRobot.status & STATUS_SWEEP)
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
						else if(gRobot.status&STATUS_AVOID)
						{
							BackCar(gRobot.walk_t.pos.angle);
						}	
						else if(gRobot.status & STATUS_CAMERA_WALK)
						{
							CameraBaseWalk3();
						}
						else if (gRobot.status & STATUS_CAMERA)
						{
							Findball_5();
						}
					  if(gRobot.avoid_t.signal)
						{
							CheckOutline3();
							//CheckOutline();
						}
/*****************************************临时测试*****************************************/
USART_OUT(UART5,(uint8_t*)"%d\t",(int)gRobot.avoid_t.passflag);				
//USART_OUT(UART5,(uint8_t*)"ttt\t%d\t%d\t%d\t%d\r\n",(int)gRobot.walk_t.right.real,gRobot.status,gRobot.avoid_t.signal,(int)gRobot.walk_t.right.aim);
USART_OUT(UART5,(uint8_t*)"%d\r\n",(int)gRobot.walk_t.circlechange.turntime);
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
			USART_OUT(UART5,(uint8_t*)"%d\r\n",(int)gRobot.shoot_t.real.speed);
/*****************************************临时测试*****************************************/			
		}
	}
}

