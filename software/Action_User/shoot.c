#include "config.h"


float LauncherPidControl(float ERR)
{
	static float ERR_OLD=0;
	static float Kp=50;
//	static float Ki=0;
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
	static float h = 424.6;//发射口到框的高度，垂直高度
	static float v = 0;//要求的速度
	//static float rev = 0;//转动速度
	static float x0=-150, y0=2400;//框的中心
	//static float g = 9.9;//重力加速度
	//static float courceAngle = 0;//定义航向角度
	static float dx=0, dy=0;//定义坐标差值
	static float alpha = 0;
	//算出发射装置的坐标
	ballNum=getBallColor();
	if (ballNum == 100)//加入球是白球
	{
		x0 = -162.5;
		y0 = 2335.35;
	}else if (ballNum==1)//假如球是黑球
	{
		x0 = 162.5;
		y0 = 2335.35;
	}
	//计算航向角转轴的坐标
	x = x - 118.36f*sinf(angle);
	y = y + 118.36f*cosf(angle);
	//计算发射装置的速度
	s =21.16+ __sqrtf((x - x0)*(x - x0) + (y - y0)*(y - y0));
	v = 150.f / __sqrtf(2.0f) * s / __sqrtf(1.234f*s - h);
// v=1.59f*s*(__sqrtf(g*1000/(1.234f*s-h)));
	launcher.rev=0.01434f*v-6.086f;
	dx = x0 - x;
	dy = y0 - y;
	alpha = atan2(dy, dx) * 180 / PI ;
	alpha = alpha - 90;
	if (alpha>180)alpha -= 360;
	else if (alpha<-180)alpha += 360;
	launcher.courceAngle = angle - alpha;
	if (launcher.courceAngle>180)launcher.courceAngle -= 360;
	else if (launcher.courceAngle<-180)launcher.courceAngle += 360;
	return launcher;

}
extern int stopUSARTsignal;
extern int32_t nowShootVel;
extern Robot_t gRobot;
void fireTask(void)
{
	static int waitAdjust=0;//定义发射电机以及航向角调整等待他们调整完之后进行推送球
	static int waitAdjust2=0;
	static float x=0,y=0,angle=0;
	static int ballNum=1;
	static Launcher_t launcher;
	static int timeCounter=0;
	
	x=gRobot.pos.x;//当前x坐标
	y=gRobot.pos.y;//当前y坐标
	angle=gRobot.pos.angle;//当前角度
	ballNum=getBallColor();
	
	timeCounter++;
	timeCounter%=4;
	
	launcher=Launcher(x,y,angle,ballNum);
	if(timeCounter==3)
	{
		YawAngleCtr(launcher.courceAngle);
	}
//	if(stopUSARTsignal==0)
//	{
	ShootCtr(launcher.rev);
//	}
//	if(10*fabs(nowShootVel-launcher.rev)<1)
//	{
//		stopUSARTsignal=0;
//	}
		if(ballNum==0)
	{
			waitAdjust++;
			if(waitAdjust<=3&&waitAdjust>0)
			{
				PushBall();
			}
			if(waitAdjust<=103&&waitAdjust>100)
			{	
				PushBallReset();
			}
			waitAdjust%=200;
	}
	else if(ballNum!=0)
	{
			waitAdjust2++;
			if(waitAdjust2<=3&&waitAdjust2>0)
			{
				PushBall();
			}
			if(waitAdjust2<=203&&waitAdjust2>200)
			{	
				PushBallReset();
			}
			waitAdjust2%=400;
	}
	USART_OUT(UART5, (uint8_t *)"%d\t", (int)gRobot.laser.leftDistance);
	USART_OUT(UART5, (uint8_t *)"%d\t", (int)gRobot.pos.x);
	USART_OUT(UART5, (uint8_t *)"%d\t", (int)gRobot.pos.y);
	USART_OUT(UART5, (uint8_t *)"%d\t", (int)ballNum);
	USART_OUT(UART5, (uint8_t *)"%d\t", (int)waitAdjust);
	USART_OUT(UART5, (uint8_t *)"%d\t", (int)gRobot.pos.angle);
	USART_OUT(UART5, (uint8_t *)"%d\t", (int)launcher.courceAngle);
	USART_OUT(UART5, (uint8_t *)"%d\t\r\n", (int)launcher.rev);
	
}



