#include "config.h"

extern Robot_t gRobot;

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

//////////////////////////////////
//测试
float b_realvel=0;
float b_realangle=0;
float w_realvel=0;
float w_realangle=0;
 float zhuansu=0.5;
 float mubiaoangle=1 ;
extern int zhuan,mubiao;
/*******************/

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
	}

else if (ballNum==1)//假如球是黑球
	{
		x0 = 162.5;
		y0 = 2335.35;
	}
		//计算航向角转轴的坐标
		x = x - 118.36f*sinf(angle);
		y = y + 118.36f*cosf(angle);
		//计算发射装置的速度
		s =21.16f+ __sqrtf((x - x0)*(x - x0) + (y - y0)*(y - y0));
		v = 157.f / __sqrtf(2.0f) * s / __sqrtf(1.234f*s - h);
		//v=1.59f*s*(__sqrtf(g*1000/(1.234f*s-h)));
			
		launcher.rev=0.98f*(0.01434f*v-6.086f);
		launcher.rev=launcher.rev+zhuan*zhuansu;
		b_realvel=launcher.rev;
			
		//launcher.rev=v/(66*PI);
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
extern int d_flag;
int ballNum=1;
void fireTask(void)
{
	static int waitAdjust=0;									//定义发射电机以及航向角调整等待他们调整完之后进行推送球
	static float x=0,y=0,angle=0;
	static Launcher_t launcher;
	static int timeCounter=0;
	CollectBallVelCtr(0);
	x=gRobot.pos.x;														//当前x坐标
	y=gRobot.pos.y;														//当前y坐标
	angle=gRobot.pos.angle;										//当前角度
	ballNum=getBallColor();
	
	timeCounter++;
	timeCounter%=4;
	
	launcher=Launcher(x,y,angle,ballNum);
	if(timeCounter==3)
	{
		
		
		
//		if(gRobot.Yawangle!=launcher.courceAngle)
//		{
//			launcher.courceAngle+=launcher.courceAngle-gRobot.Yawangle;
//		}
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
//	if(ballNum==20)
//	{
//			waitAdjust++;
//			if(waitAdjust<=3&&waitAdjust>0)
//			{
//				PushBall();
//			}
//			if(waitAdjust<=53&&waitAdjust>50)
//			{	
//				PushBallReset();
//			}
//			waitAdjust%=100;
//	}
				waitAdjust++;
			if(waitAdjust<=3&&waitAdjust>0)
			{
				PushBall();
			}
			if(waitAdjust<=153&&waitAdjust>150)
			{	
				PushBallReset();
			}
			waitAdjust%=300;
			if(d_flag==1)
			{
				//d_fireTask(ballNum,waitAdjust,launcher.courceAngle,launcher.rev);	
				d_fireTask();
				d_flag=0;
			}
}



