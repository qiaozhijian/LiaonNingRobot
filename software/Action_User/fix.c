#include "config.h"

static float angle=0,xpos=0,ypos=0;
static float errSingle=0; //errSingle = realSingle - nowSingle
static float errX0=0;
static float errY0=0;
void setAngle(float val)
{
	angle=val;
}
void setXpos(float val)
{ 
	xpos=val;
}
void setYpos(float val)
{
	ypos=val;
}
/****************************************************************************
* 名    称：setErrSingle() setErrX(float realX) setErrY(float realy)
* 功    能：摄像头的数据处理
* 入口参数：无
* 出口参数：无
* 说    明：无
* 调用方法：无 
****************************************************************************/

void setErrSingle(float reaAngle)
{
	errSingle = reaAngle - angle;
}


void setErrX(float realX)
{
	float temp;
	temp = xpos * cos(-errSingle * PI / 180) - ypos * sin(-errSingle * PI / 180);
	errX0 = realX - temp;
}
void setErrY(float realy)
{
	float temp;
	temp = xpos * sin(-errSingle * PI / 180) + ypos * cos(-errSingle * PI / 180);
	errY0 = realy - temp;
}


float getAngle(void)
{
	if (angle + errSingle > 180)
		return angle + errSingle - 360;
	else if (angle + errSingle < -180)
		return angle + errSingle + 360;
	else
		return angle + errSingle;
}
float getXpos(void)
{
	return xpos * cos(-errSingle * PI / 180) - ypos * sin(-errSingle * PI / 180) + errX0;
}
float getYpos(void)
{
	return xpos * sin(-errSingle * PI / 180) + ypos * cos(-errSingle * PI / 180) + errY0;
}
void setErr(float reaAngle,float realX,float realy)
{
	setErrSingle(reaAngle);
	setErrX(realX);
	setErrY(realy);
}
/**
*	参数 void
*	返回值 得到左侧激光运算后返回的距离
*/
int getLeftAdc()
{
	return 0.9389*Get_Adc_Average(ADC_Channel_15, 10)+428.6575;
}
/**
*	参数 void
*	返回值 得到右侧激光运算后返回的距离
*/
int getRightAdc()
{
	return 0.9403*Get_Adc_Average(ADC_Channel_14, 10)+435.445;
}
