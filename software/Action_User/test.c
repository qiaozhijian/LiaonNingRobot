#include "config.h"
#include "test.h"

extern Robot_t gRobot;

int a=0;
extern int testMode;
void TestMode(void)
{
	float laserRight=getRightAdc();
	float laserLeft=getLeftAdc();
  switch(testMode)
	{
		case 1://æ£€æŸ¥æ‘„åƒå¤´æ•°çš„çƒ
			
		break;
		
		case 2:
			WheelTest(laserRight,laserLeft);
			TravelSwitchTest();
		break;
	
		default:
			
		break;
	
}

	
	
	
	
	
//			a=getBallColor();
/*****************************************ÁÙÊ±²âÊÔ*****************************************/
	USART_OUT(UART5,"%d\t",(int)gRobot.camera_t.camerapid.aimAngle);
	USART_OUT(UART5,"%d\t",(int)gRobot.walk_t.pos.angle);
	USART_OUT(UART5,"%d\t",(int)gRobot.walk_t.pos.x);
	USART_OUT(UART5,"%d\t",(int)gRobot.walk_t.pos.y);
	USART_OUT(UART5,"%d\t",(int)gRobot.collect_t.PhotoElectric.ballcount);						
	USART_OUT(UART5,"%d\r\n",(int)gRobot.avoid_t.signal);
							
	USART_OUT(UART5,"%d\t",(int)gRobot.avoid_t.passflag);	
	USART_OUT(UART5,"%d\t",(int)angleErrorCount(gRobot.avoid_t.pid.aimAngle,gRobot.walk_t.pos.angle));
							
	USART_OUT(UART5,"%d\t",(int)gRobot.walk_t.pos.angle);
	USART_OUT(UART5,"%d\t",(int)gRobot.walk_t.pos.x);
	USART_OUT(UART5,"%d\t",(int)gRobot.walk_t.pos.y);			
							
	USART_OUT(UART5,"%d\t",(int)gRobot.avoid_t.pid.aimAngle);	
	//USART_OUT(UART5,"%d\t%d\t%d\t%d\t",(int)gRobot.walk_t.right.real,gRobot.status,gRobot.avoid_t.signal,(int)gRobot.walk_t.right.aim);
	USART_OUT(UART5,"%d\r\n",(int)gRobot.walk_t.circleChange.turnTime);
	USART_OUT(UART5,"%d\r\n",(int)gRobot.walk_t.circleChange.turnTimerem);
	USART_OUT(UART5,"%d\r\n",(int)gRobot.status);

				//			WalkTask1();
				//			WalkTask2();
				//			ÄæÊ±ÕëÐý×ª
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
/*****************************************ÁÙÊ±²âÊÔ*****************************************/			

}
void WheelTest(float laserRight,float laserLeft)
{
	if(laserRight>400||laserLeft>400)
	{
		VelCrl(CAN2,1,0);
		VelCrl(CAN2,2,0);
		CollectBallVelCtr(0);
	}
	if(laserRight<=400)
	{
		VelCrl(CAN2,1,laserRight*13);
		VelCrl(CAN2,2,0);
		CollectBallVelCtr(laserRight/8);
	}
	if(laserLeft<=400)
	{
		VelCrl(CAN2,1,0);
		VelCrl(CAN2,2,-laserLeft*13);
		CollectBallVelCtr(laserLeft/8);
	}
}	
void TravelSwitchTest(void)
{
	if(TRAVEL_SWITCH_LEFT==1&&TRAVEL_SWITCH_RIGHT==1)
	{
		GPIO_ResetBits(GPIOB,GPIO_Pin_14);
		Delay_ms(800);
		GPIO_SetBits(GPIOB,GPIO_Pin_14);
		Delay_ms(800);
	}else if(TRAVEL_SWITCH_LEFT==1&&TRAVEL_SWITCH_RIGHT==0)
	{
		GPIO_ResetBits(GPIOB,GPIO_Pin_14);
		Delay_ms(300);
		GPIO_SetBits(GPIOB,GPIO_Pin_14);
		Delay_ms(300);
	}else if(TRAVEL_SWITCH_LEFT==0&&TRAVEL_SWITCH_RIGHT==1)
	{
		GPIO_ResetBits(GPIOB,GPIO_Pin_14);
		Delay_ms(500);
		GPIO_SetBits(GPIOB,GPIO_Pin_14);
		Delay_ms(500);
	}else if(TRAVEL_SWITCH_LEFT==0&&TRAVEL_SWITCH_RIGHT==0)
	{
		GPIO_ResetBits(GPIOB,GPIO_Pin_14);
	}

}
void Debug(void){

			USART_OUT(UART5,"%d\t", (int)gRobot.status);
			USART_OUT(UART5,"%d\t",(int)gRobot.walk_t.circleChange.turnTime);
			USART_OUT(UART5,"%d\t",(int)gRobot.walk_t.pos.angle);
		  USART_OUT(UART5,"%d\t",(int)gRobot.walk_t.pos.x);
		  USART_OUT(UART5,"%d\t",(int)gRobot.walk_t.pos.y);
			USART_OUT(UART5,"%d\t",(int)LineCheck(gRobot.walk_t.circleChange.direction));
   		USART_OUT(UART5,"%d\t",(int)gRobot.collect_t.PhotoElectric.ballcount);
		  USART_OUT(UART5,"%d\t",(int)gRobot.camera_t.camrBaseWalk_t.circleChange.turnTime);
		  USART_OUT(UART5,"%d\t",(int)gRobot.walk_t.circleChange.direction);
		  USART_OUT(UART5,"%d\t",(int)gRobot.camera_t.camrBaseWalk_t.circleChange.turnTime);
			USART_OUT(UART5,"%d\t",(int)gRobot.walk_t.circleChange.linenum);
			USART_OUT(UART5,"%d\t",(int)gRobot.walk_t.pid.aimAngle );
			USART_OUT(UART5,"%d\t",(int)gRobot.walk_t.pid.angleError);
		  USART_OUT(UART5,"%d\t",(int)gRobot.walk_t.pid.disError);
			USART_OUT(UART5,"%d\t",(int)gRobot.walk_t.pid.distanceStraight);
			USART_OUT(UART5,"%d\t",(int)gRobot.shoot_t.sReal.speed);
			USART_OUT(UART5,"%d\r\n",(int)gRobot.walk_t.circleChange.circleNum);


}

