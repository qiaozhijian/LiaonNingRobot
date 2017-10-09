#include "config.h"
#include "test.h"

extern Robot_t gRobot;

int a=0;
extern int testMode;
float laserRight;
float laserLeft;
void TestMode(void)
{
	int ballNum=0;
	 laserRight=getRightAdc();
	 laserLeft=getLeftAdc();
  switch(testMode)
	{
		case 1:
			WheelTest(laserRight,laserLeft);
			TravelSwitchTest();
		break;
		
		case 2://检查摄像头数的球
			ballNum=getF_ball();
			if(ballNum<=2)
			{
				GPIO_ResetBits(GPIOE,GPIO_Pin_7);
				Delay_ms(300);
				GPIO_SetBits(GPIOE,GPIO_Pin_7);
				Delay_ms(300);
			}else if(ballNum<5&&ballNum>=3)
			{
				GPIO_ResetBits(GPIOE,GPIO_Pin_7);
				Delay_ms(500);
				GPIO_SetBits(GPIOE,GPIO_Pin_7);
				Delay_ms(500);
			}else if(ballNum>=5)
			{
				GPIO_ResetBits(GPIOE,GPIO_Pin_7);
				Delay_ms(800);
				GPIO_SetBits(GPIOE,GPIO_Pin_7);
				Delay_ms(800);
			}
		break;
		
		case 3://发射自检
			ShootTest();
		break;
		case 4:
			break;
		case 5:
			USART_OUT(UART5,"posX:%d\t",(int)getXpos());
			USART_OUT(UART5,"POSY:%d\t",(int)getYpos());
			USART_OUT(UART5,"POSY:%d\t\r\n",(int)getAngle());
		break;
		
		default:
		break;
	
}
}
void WheelTest(float laserRight,float laserLeft)
{
	static int rightvel;
	static int leftvel;
	if(laserRight>800&&laserLeft>800)
	{
		rightvel=0;
		leftvel=0;
		VelCrl(CAN2,1,rightvel);
		VelCrl(CAN2,2,leftvel);
		CollectBallVelCtr(0);
	}else if(laserRight<=800)
	{
		rightvel=laserRight*10;
		leftvel=-laserRight*10;
		VelCrl(CAN2,1,rightvel);
		VelCrl(CAN2,2,leftvel);
		if(laserRight/8>50)
		{
			laserRight=400;
		}
		CollectBallVelCtr(laserRight/8);
	}else if(laserLeft<=800)
	{
		rightvel=-laserLeft*10;
		leftvel=laserLeft*10;
		VelCrl(CAN2,1,rightvel);
		VelCrl(CAN2,2,leftvel);
		if(laserRight/8>50)
		{
			laserLeft=400;
		}
		CollectBallVelCtr(laserLeft/8);
	}
	USART_OUT(UART5,"AimrightVel:%d\t",(int)rightvel);
	USART_OUT(UART5,"AimleftVel:%d\t",(int)leftvel);
	USART_OUT(UART5,"RealrightVel:%d\t",(int)gRobot.walk_t.right.real);
	USART_OUT(UART5,"RealleftVel:%d\r\n",(int)gRobot.walk_t.left.real);
}	
void TravelSwitchTest(void)
{
	static int right=0;
	static int left=0;
	if(TRAVEL_SWITCH_LEFT==1&&TRAVEL_SWITCH_RIGHT==1)
	{
		GPIO_SetBits(GPIOE,GPIO_Pin_7);
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
	 left=(GPIO_ReadInputDataBit(GPIOE, GPIO_Pin_2));
	 right=(GPIO_ReadInputDataBit(GPIOC,GPIO_Pin_0));
	USART_OUT(UART5,"rightkey:%d\t",(int)right);
	USART_OUT(UART5,"leftkey:%d\r\n",(int)left);
}
void ShootTest(void)
{
	static int pullTime=0;
	YawAngleCtr(gRobot.shoot_t.sAim.angle);
	USART_OUT(UART5,"Shootspeed:%d\t",(int)gRobot.shoot_t.sReal.speed);
	ShootCtr(gRobot.shoot_t.sAim.speed);
	USART_OUT(UART5,"Shootangle:%d\t",(int)gRobot.shoot_t.sReal.angle);
	if(pullTime<3)
	{
		PushBall();
		USART_OUT(UART5,"PushBall:%d\t",(int)gRobot.shoot_t.pReal.pos);
	}else if(pullTime>200&&pullTime<203)
	{
		PushBallReset();
		USART_OUT(UART5,"PushBallRest%d\t",(int)gRobot.shoot_t.pReal.pos);
	}
  pullTime++;
	pullTime%=400;
	if(getBallColor()==1)
	{
		USART_OUT(UART5,"blacksucess:%d\r\n",(int)getBallColor());
	}
	else if(getBallColor()==100)
	{
		USART_OUT(UART5,"whitesucess:%d\r\n",(int)getBallColor());
	}
	else
	{
		USART_OUT(UART5,"CCDFault\r\n");
	}
}
