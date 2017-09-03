#include "shoot.h"
#include "arm_math.h"
#include "stdint.h"
#include "arm_math.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_rcc.h"
#include "timer.h"
#include "gpio.h"
#include "usart.h"
#include "motor.h"
#include "fix.h"
#include "stm32f4xx_it.h"
float LauncherPidControl(float ERR)
{
	static float ERR_OLD=0;
	static float Kp=50;
	static float Ki=0;
	static float Kd=10;
	static float OUTPUT=0;
	OUTPUT=Kp*ERR + Kd*(ERR-ERR_OLD);
	ERR_OLD=ERR;
	return OUTPUT;
}
Launcher_t Launcher(float x,float y,float angle,int ballNum)
{
	static Launcher_t launcher;
	static float s = 0;//车到圆环中心的距离
	static float h = 474.6;//发射口到框的高度，垂直高度
	static float v = 0;//要求的速度
	//static float rev = 0;//转动速度
	static float x0=-150, y0=2400;//框的中心
	static float g = 9.9;//重力加速度
	//static float courceAngle = 0;//定义航向角度
	static float dx=0, dy=0;//定义坐标差值

	if (ballNum == 100)//加入球是白球
	{
		x0 = -150;
		y0 = 2400;
	}else if (ballNum==1)//假如球是黑球
	{
		x0 = 150;
		y0 = 2400;
	}

	s = sqrt((x - x0)*(x - x0) + (y - y0)*(y - y0));
	v = sqrt(s*s*g * 1000 / ((2 * s*tan(51 * PI / 180) - 2 * h)*cos(51 * PI / 180)*cos(51 * PI / 180)));
	launcher.rev = v  / PI / 66;
	dx = x0 - x;//建立以车为原点的坐标系
	dy = y0 - y;

	//以下的处理过程是为了两个角度0位置在同一条直线上
	launcher.courceAngle = atan2(dy, dx) / PI * 180;
	if(launcher.courceAngle<=180&& launcher.courceAngle>=-90)
	{
		launcher.courceAngle -= 90;
	}
	else if (launcher.courceAngle < -90&& launcher.courceAngle>=-180)
	{
		launcher.courceAngle = 270 + launcher.courceAngle;
	}
	
	//得出航向角的值
	launcher.courceAngle = angle-launcher.courceAngle;
	//防止航向角大于180
	
	if(launcher.courceAngle>180)
	{
		launcher.courceAngle -= 360;
	}
	else if (launcher.courceAngle < -180)
	{
		launcher.courceAngle += 360;
	}
	return launcher;
}
void fireTask(void)
{
	static int waitAdjust=0;//定义发射电机以及航向角调整等待他们调整完之后进行推送球
	static float x=0,y=0,angle=0;
	static int ballNum=1;
	static Launcher_t launcher;  
	CollectBallVelCtr(40);
	x=getXpos();//当前x坐标
	y=getYpos();//当前y坐标
	angle=getAngle();//当前角度
	ballNum=getBallColor();
	CollectBallVelCtr(35);
	launcher=Launcher(x,y,angle,ballNum);
	YawAngleCtr(launcher.courceAngle);
	ShootCtr(launcher.rev*2+13);
	waitAdjust++;
	if(waitAdjust<=200)
	{
		PushBall();
	}
	if(waitAdjust>200)
	{
		PushBallReset();
	}
	waitAdjust%=400;
	USART_OUT(UART5, (uint8_t *)"%d\t", (int)getXpos());
	USART_OUT(UART5, (uint8_t *)"%d\t", (int)getYpos());
	USART_OUT(UART5, (uint8_t *)"%d\t", (int)waitAdjust);
	USART_OUT(UART5, (uint8_t *)"%d\t", (int)angle);
	USART_OUT(UART5, (uint8_t *)"%d\t", (int)launcher.courceAngle);
	USART_OUT(UART5, (uint8_t *)"%d\t\r\n", (int)launcher.rev*3);
}



