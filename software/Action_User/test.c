#include "config.h"
#include "test.h"

extern Robot_t gRobot;

int a=0;
extern int testMode;
void TestMode(void)
{
	int ballNum=0;
	float laserRight=getRightAdc();
	float laserLeft=getLeftAdc();
  switch(testMode)
	{
		case 1://检查摄像头数的球
			ballNum=getF_ball();
			if(ballNum<=2)
			{
				GPIO_ResetBits(GPIOE,GPIO_Pin_7);;
				Delay_ms(300);
				GPIO_SetBits(GPIOE,GPIO_Pin_7);;
				Delay_ms(300);
			}else if(ballNum<5&&ballNum>=3)
			{
				GPIO_ResetBits(GPIOE,GPIO_Pin_7);;
				Delay_ms(500);
				GPIO_SetBits(GPIOE,GPIO_Pin_7);;
				Delay_ms(500);
			}else if(ballNum>=5)
			{
				GPIO_ResetBits(GPIOE,GPIO_Pin_7);;
				Delay_ms(800);
				GPIO_SetBits(GPIOE,GPIO_Pin_7);;
				Delay_ms(800);
			}
		break;
		
		case 2:
			WheelTest(laserRight,laserLeft);
			TravelSwitchTest();
		break;
	
		case 3://发射自检
			ShootTest();
		break;
		
		default:
		break;
	
}

	
	
	
	

/*****************************************ʱӢ˔*****************************************/
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
				//			Ŧʱ֫ѽת
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
/*****************************************ʱӢ˔*****************************************/			

}
void WheelTest(float laserRight,float laserLeft)
{
	if(laserRight>600&&laserLeft>600)
	{
		VelCrl(CAN2,1,0);
		VelCrl(CAN2,2,0);
		CollectBallVelCtr(0);
	}
	if(laserRight<=600)
	{
		VelCrl(CAN2,1,laserRight*10);
		VelCrl(CAN2,2,0);
		CollectBallVelCtr(laserRight/8);
	}
	if(laserLeft<=600)
	{
		VelCrl(CAN2,1,0);
		VelCrl(CAN2,2,-laserLeft*10);
		CollectBallVelCtr(laserLeft/8);
	}
}	
void TravelSwitchTest(void)
{
	if(TRAVEL_SWITCH_LEFT==1&&TRAVEL_SWITCH_RIGHT==1)
	{
		GPIO_ResetBits(GPIOE,GPIO_Pin_7);
		Delay_ms(800);
		GPIO_SetBits(GPIOE,GPIO_Pin_7);
		Delay_ms(800);
	}else if(TRAVEL_SWITCH_LEFT==1&&TRAVEL_SWITCH_RIGHT==0)
	{
		GPIO_ResetBits(GPIOE,GPIO_Pin_7);
		Delay_ms(300);
		GPIO_SetBits(GPIOE,GPIO_Pin_7);
		Delay_ms(300);
	}else if(TRAVEL_SWITCH_LEFT==0&&TRAVEL_SWITCH_RIGHT==1)
	{
		GPIO_ResetBits(GPIOE,GPIO_Pin_7);
		Delay_ms(500);
		GPIO_SetBits(GPIOE,GPIO_Pin_7);
		Delay_ms(500);
	}else if(TRAVEL_SWITCH_LEFT==0&&TRAVEL_SWITCH_RIGHT==0)
	{
		GPIO_ResetBits(GPIOE,GPIO_Pin_7);
	}
}
void ShootTest(void)
{
	static int pullTime=0;
	YawAngleCtr(gRobot.shoot_t.sAim.angle);
	ShootCtr(gRobot.shoot_t.sAim.speed);
	if(pullTime<3)
	{
		PushBall();
	}else if(pullTime>200&&pullTime<203)
	{
		PushBallReset();
	}
  pullTime++;
	pullTime%=400;
}
