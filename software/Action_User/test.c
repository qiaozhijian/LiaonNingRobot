#include "config.h"
#include "test.h"

extern Robot_t gRobot;

int a=0;
extern int testMode;
float laserRight;
float laserLeft;
void TestMode(void)
{
	 laserRight=getRightAdc();
	 laserLeft=getLeftAdc();
  switch(testMode)
	{
		case 1:
			WheelTest(laserRight,laserLeft);
			TravelSwitchTest();
		break;
		
		case 2://检查摄像头数的球
		   fireTask2();
		break;
		
		case 3://发射自检
			 CollectBallVelCtr(60);        
			ShootTest();
		break;
		
		case 4:
			break;
		case 5:
				
		break;
		
		default:
		break;
	
}
}
void WheelTest(float laserRight,float laserLeft)
{
	static int rightvel;
	static int leftvel;
	if(laserRight>600&&laserLeft>600)
	{
		rightvel=0;
		leftvel=0;
		VelCrl(CAN2,1,rightvel);
		VelCrl(CAN2,2,leftvel);
		CollectBallVelCtr(0);
	}else if(laserRight<=600)
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
	}else if(laserLeft<=600)
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
	USART_OUT(UART5,"posX:%d\t",(int)getXpos());
	USART_OUT(UART5,"POSY:%d\t",(int)getYpos());
	USART_OUT(UART5,"POSY:%d\t",(int)getAngle());
	USART_OUT(UART5,"laserleft:%d\t",(int)laserLeft);
	USART_OUT(UART5,"laserright:%d\t",(int)laserRight);
	USART_OUT(UART5,"AimrightVel:%d\t",(int)rightvel);
	USART_OUT(UART5,"AimleftVel:%d\t",(int)leftvel);
	USART_OUT(UART5,"RealrightVel:%d\t",(int)gRobot.walk_t.right.real);
	USART_OUT(UART5,"RealleftVel:%d\t",(int)gRobot.walk_t.left.real);
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
void test2(void)
{
	static int time1=0;
	static int time2=0;
	static int time3=0;
	static int left=0;
	static int right=0;
	 left=(GPIO_ReadInputDataBit(GPIOE, GPIO_Pin_2));
	 right=(GPIO_ReadInputDataBit(GPIOC,GPIO_Pin_0));
	time1++;
	time2++;
	time3++;
	//轮子检测
	if(time1>0 && time1<300)
	{
		VelCrl(CAN2,1,5000);
		VelCrl(CAN2,2,-5000);
	}else if(time1>300 && time1<600)
	{
		VelCrl(CAN2,1,-5000);
		VelCrl(CAN2,2,5000);
	}else if(time1>600)
	{
		time1=0;
	}
	//推子机构检测
	if(time2>0 && time2<3)
	{
		ShootCtr(15);
		PushBall();
	}else if(time2>150 && time2<153)
	{
	  PushBallReset();
	}else if(time2>300)
	{
		time2=0;
	}
	//发射机构检测
	if(getBallColor()==1)
	{
	  YawAngleCtr(45);
	}else if(getBallColor()==100)
	{
		YawAngleCtr(-45);
	}
	//蜂鸣器检测
	if(left==1)
	{
		GPIO_ResetBits(GPIOE,GPIO_Pin_7);
	}else if(right==1)
	{
		GPIO_SetBits(GPIOE,GPIO_Pin_7);
	}
	//激光检测
	if(getLeftAdc()<600)
	{
		GPIO_ResetBits(GPIOE,GPIO_Pin_7);
	}else if(getRightAdc()<600)
	{
		GPIO_SetBits(GPIOE,GPIO_Pin_7);
	}
	//发数
	USART_OUT(UART5,"posX:%d\t",(int)getXpos());
	USART_OUT(UART5,"POSY:%d\t",(int)getYpos());
	USART_OUT(UART5,"POSY:%d\t",(int)getAngle());
	USART_OUT(UART5,"rightkey:%d\t",(int)right);
	USART_OUT(UART5,"leftkey:%d\t",(int)left);
	USART_OUT(UART5,"laserleft:%d\t",(int)laserLeft);
	USART_OUT(UART5,"laserright:%d\t",(int)laserRight);
}
