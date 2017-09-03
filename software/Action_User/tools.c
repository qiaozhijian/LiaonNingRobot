#include "tools.h"
#include "stdint.h"
#include "arm_math.h"
#include "usart.h"
#include "fix.h"
static float Aimxfirst=0,Aimyfirst=0;
static float Aimxsecond=0,Aimysecond=0;
static float xRem=0,yRem=0; 
static float Bestangle=0;
static int f_ball=0;
static int ballColor=0;
/************************SET***********************/
void setF_ball(int val)
{
	f_ball=val;
}
void setBestangle(float val)
{
	Bestangle=val;
}
void setxRem(float val)
{
	xRem=val;
}
void setyRem(float val)
{
	yRem=val;
}
void setAimxfirst(float val)
{
	Aimxfirst=val;
}
void setAimyfirst(float val)
{
	Aimyfirst=val;
}
void setAimxsecond(float val)
{
	Aimxsecond=val;
}
void setAimysecond(float val)
{
	Aimysecond=val;
}
void setBallColor(int temp)
{
	ballColor=temp;
}


/************************GET***********************/
int getF_ball(void)
{
	if(f_ball>0)return 1;
	else return 0;
}
float getBestangle(void)
{
	return Bestangle;
}
float getxRem(void)
{
	return xRem;
}
float getyRem(void)
{
	return yRem;
}
float getAimxfirst(void)
{
	return Aimxfirst;
}
float getAimyfirst(void)
{
	return Aimyfirst;
}
float getAimxsecond(void)
{
	return Aimxsecond;
}
float getAimysecond(void)
{
	return Aimysecond;
}
int getBallColor(void)
{
	return ballColor;
}
/************************求点到点的距离***********************/
double Dis(float Xstart,float Ystart,float Xstop,float Ystop)
{
	return sqrt(pow(Xstart-Xstop,2)+pow(Ystart-Ystop,2));
}
/*******************将摄像头坐标变为世界坐标系****************/
float Xcoorchange(float x,float y,float angle)
{
	return getXpos()-Dis(x,y,0,0)*sinf((getAngle()+angle)*PI/180)-220*sin(getAngle()*PI/180);
}
float Ycoorchange(float x,float y,float angle)
{
	return getYpos()+Dis(x,y,0,0)*cosf((getAngle()+angle)*PI/180)+220*cos(getAngle()*PI/180);
}
float Anglechange(float angle)
{
	static float tmp1=0;
	static float tmp2=0;
	static float tmp3=0;
	tmp1=fabs(angle);
	tmp2=180/PI*asinf(140*sinf((tmp1)*PI/180)/sqrt(20384+7840*cosf((tmp1)*PI/180)));
	if(angle<0)tmp2=-tmp2;
	tmp3=getAngle()+tmp2;
	if(tmp3<-180)tmp3+=360;
	else if(tmp3>180)tmp3-=360;
	return tmp3;
}

