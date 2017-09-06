#include "math.h"
#include "stm32f4xx.h"
#include "math.h"
#include "gpio.h"
#include "usart.h"
#include "stdlib.h"
#include "timer.h"
#include "stm32f4xx_usart.h"
#include "includes.h"
#include "misc.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_rcc.h"
#include "can.h"
#include "elmo.h"
#include "stm32f4xx_it.h"
#include "camera.h"
#include "sweep2.h"
#include "task.h"
#include "sweep.h"
#include "shoot.h"
#define NAMUDA (0.5)
static float Aim_X,Aim_Y=0,Theta=0,Aim_Theta=0,input=0,Err=0;
static float Get_Xcoor=0;
static float Get_Ycoor=0;
static float Get_Angle=0;
static int	Flag_Start=1;
static int lineChangeSymbol=0;
int Kp=200;
//??PID
void Target_Angle_X(int Step)
	{
		if(Step==9&&(Flag_Start==1))
		{
			Angle_0();
		}
		else if(Step==11&&(Flag_Start==1))
		{
			Angle_180();	
		}
//		else if(Step==9&&(Flag_Start==2))
//		{
//			Angle_0_1();
//		}
//		else if(Step==11&&(Flag_Start==2))
//		{
//			Angle_180_1();
//		}
	}
	
	
void Target_Angle_Y(int Step)
	{
	if(Step==10&&(Flag_Start==1))
	{
		Angle_f90();
	}
	else if(Step==12&&(Flag_Start==1))
	{
		Angle_90();
	}
//	else if(Step==0&&(Flag_Start==2))
//	{
//		Angle_90_1();
//	}
//	else if(Step==2&&(Flag_Start==2))
//	{
//		Angle_f90_1();
//	}
	}
//????PID??
void PID()
{
	Err=Aim_Theta-Get_Angle;
	if(Err>180)Err-=360;
	else if(Err<-180)Err+=360;
	input=Kp*Err;
	VelCrl(CAN2, 1, PULSE+input);
	VelCrl(CAN2, 2, -PULSE+input);
}
//?T?????
void Angle_f90()
{
	Aim_X=(Get_Xcoor*NAMUDA+(400+ERR_X*lineChangeSymbol))/(NAMUDA+1);
	Aim_Y=1400-ERR_Y*lineChangeSymbol;
	Theta=atan((Get_Ycoor-Aim_Y)/(Get_Xcoor-Aim_X))*180/PI;
	Aim_Theta=-90+Theta;
	PID();
}
void Angle_0()
{
	Aim_X=600+ERR_X*lineChangeSymbol;
	Aim_Y=(Get_Ycoor*NAMUDA+(3200+ERR_Y*lineChangeSymbol))/(1+NAMUDA);
	Theta=atan((Get_Xcoor-Aim_X)/(Get_Ycoor-Aim_Y))*180/PI;
	Aim_Theta=0-Theta;
	PID();
}
void Angle_90()
{
	Aim_X=(Get_Xcoor*NAMUDA+(-400-ERR_X*lineChangeSymbol))/(1+NAMUDA);
	Aim_Y=3767+ERR_Y*lineChangeSymbol;
	Theta=atan((Get_Ycoor-Aim_Y)/(Get_Xcoor-Aim_X))*180/PI;
	Aim_Theta=90+Theta;
	PID();
}
void Angle_180()
{
	Aim_X=-600-ERR_X*lineChangeSymbol;
	Aim_Y=(Get_Ycoor*NAMUDA+(1600-ERR_Y*lineChangeSymbol))/(1+NAMUDA);
	Theta=atan((Get_Xcoor-Aim_X)/(Get_Ycoor-Aim_Y))*180/PI;
	if(Get_Xcoor-Aim_X>0)
		Aim_Theta=180-Theta;
	else if(Get_Xcoor-Aim_X<0)
		Aim_Theta=-180-Theta;
	PID();
}
extern Robot_t gRobot;
extern float posX;
extern float posY;
void Sweep2(void)//??????
{
	static int turntime=9;
		Get_Xcoor=gRobot.pos.x;
		Get_Ycoor=gRobot.pos.y;
		Get_Angle=gRobot.pos.angle;
		switch (turntime)
		{
		case 5:
			AgainstWall(0,Get_Angle);
		break;
		
		case 7:
			BackCar(Get_Angle);
		break;
			
		case 8:
			fireTask();
		break;
		case 9:
			if(Get_Xcoor>400+ERR_X*lineChangeSymbol)
			turntime++;
			Angle_0();
			break;
		case 10:
			if(Get_Ycoor>3200+ERR_Y*lineChangeSymbol)
				turntime++;
				Angle_90();
			break;
		case 11:
			if(Get_Xcoor<-400-ERR_X*lineChangeSymbol)
			turntime++;
			Angle_180();
			break;
		case 12:
			if(Get_Ycoor<1600-ERR_Y*lineChangeSymbol)
			{
				turntime=9;	
				lineChangeSymbol++;
			}
			Angle_f90();
			break;
		default:
		break;
		}
		DeBug();
		USART_OUT(UART5, (uint8_t *)"%d\t", (int)gRobot.pos.x);
		USART_OUT(UART5, (uint8_t *)"%d\t", (int)gRobot.pos.y);
		USART_OUT(UART5, (uint8_t *)"%d\t", (int)posX);
		USART_OUT(UART5, (uint8_t *)"%d\t", (int)posY);
//		
//		USART_OUT(UART5, (uint8_t *)"%d\t", (int)angle);//gRobot.pos.angle
//		USART_OUT(UART5, (uint8_t *)"%d\t", (int)angleError);
//		USART_OUT(UART5, (uint8_t *)"%d\t", (int)spacingError);
//		USART_OUT(UART5, (uint8_t *)"%d\t", (int)disError);
//		USART_OUT(UART5, (uint8_t *)"%d\t", (int)piddisShuchu);
//		USART_OUT(UART5, (uint8_t *)"%d\t", (int)pidZongShuchu);
		USART_OUT(UART5, (uint8_t *)"%d\t", (int)turntime);
		USART_OUT(UART5, (uint8_t *)"%d\t", (int)lineChangeSymbol);
////		USART_OUT(USART1, (uint8_t *)"%d\t", (int)stickError);
////		USART_OUT(UART5, (uint8_t *)"%d\t", (int)xStick);
////		USART_OUT(UART5, (uint8_t *)"%d\t", (int)yStick);
//		USART_OUT(UART5, (uint8_t *)"%d\r\n", (int)turnTimeRemember);
}

