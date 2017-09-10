#include "config.h"

extern Robot_t gRobot;

static int turnTimeRemember;												   //记住在卡死的时候是什么直线的状态，等倒车case结束后让重新填装
static float xStick, yStick;															   //卡住时存储的位置数据

 /****************************************************************************
* 名    称：void BackCarIn(float angle)
* 功    能：内环逃逸程序后退1.5s，外转45度
* 入口参数：angle//当前角度
* 出口参数：无
* 说    明：无
* 调用方法：无 
****************************************************************************/
void BackCarIn(float angle) //内环倒车程序
{
	static float aimAngle = 0;   //目标角度
	static float angleError = 0; //目标角度与当前角度的偏差
	static int i = 0;																  //目标角度变换标志位
	static int j = 0; 																//在此设立标志位在信号量10ms进入一次，达到延时的效果
	if (i == 0)																		    //使目标角度偏向右边45
	{
		aimAngle = angle - 45; //让车头目标角度右偏45度
		i = 1;
	}
	angleError = angleErrorCount(aimAngle,angle);
	j++;
	if (j < 150)
	{
		VelCrl(CAN2, 1, -6107); //pid中填入的是差值
		VelCrl(CAN2, 2,  6107);
	}else if (j >=150)
	{
		VelCrl(CAN2, 1, AnglePidControl(angleError)); //pid中填入的是差值
		VelCrl(CAN2, 2, AnglePidControl(angleError));
		if (fabs(angleError) < 5)
		{
			gRobot.turnTime = turnTimeRemember;
			i = 0;
			j = 0;//清空标志位
		} 
	}
//	pidZongShuchu = AnglePidControl(angleError);
}
 /****************************************************************************
* 名    称：void BackCarOut(float angle) 
* 功    能：外环逃逸程序后退1.5s，内转45度
* 入口参数：angle//当前角度
* 出口参数：无
* 说    明：无
* 调用方法：无 
****************************************************************************/
void BackCarOut(float angle) //外环倒车程序
{
	static float aimAngle = 0;   //目标角度
	static float angleError = 0; //目标角度与当前角度的偏差
	static int i = 0;																  //目标角度变换标志位
	static int j = 0; 																//在此设立标志位在信号量10ms进入一次，达到延时的效果
	if (i == 0)																		  //使目标角度偏向右边45
	{
		aimAngle = angle + 45; //让车头目标角度右偏45度
		i = 1;
	}
	angleError = angleErrorCount(aimAngle,angle);
	j++;
	if (j < 150)
	{
		VelCrl(CAN2, 1, -6107); //pid中填入的是差值
		VelCrl(CAN2, 2,  6107);
	}else if (j >=150)
	{
		VelCrl(CAN2, 1, AnglePidControl(angleError)); //pid中填入的是差值
		VelCrl(CAN2, 2, AnglePidControl(angleError));
		if (fabs(angleError) < 5)
		{
			gRobot.turnTime = turnTimeRemember;
			i = 0;
			j = 0;//清空标志位
		}
	}
//	pidZongShuchu = AnglePidControl(angleError);
}
 /****************************************************************************
* 名    称：void CheckOutline(void) 
* 功    能：检测是否卡死
* 入口参数：无
* 出口参数：无
* 说    明：当前是停在x=1000 y=0
* 调用方法：无 
****************************************************************************/
void CheckOutline(void)//检测是否卡死
{
	static int stickError = 0;													   //卡死错误积累值
	static float xError = 0, yError = 0;
	turnTimeRemember = gRobot.turnTime;
	xError = gRobot.pos.x - getxRem();
	yError = gRobot.pos.y - getyRem();
	//判断进程到哪一步（替换M）  --summer
	if (fabs(xError) < 1 && fabs(yError) < 1 && gRobot.M != 0)
	{
		stickError++;
	}
	else
	{
		stickError = 0;
	}
	/*
	200太大
	分情况：启动的时候，走直线的时候，过弯的时候，停下投球，矫正的时候。
	不一定是小于1  根据应该有的速度设定
	*/
	if (stickError > 200)
	{
		xStick = getxRem();//记住卡死的坐标
		yStick = getyRem();
		gRobot.turnTime = 7;
		stickError = 0;
	}
}
 /****************************************************************************
* 名    称：void BackCarOut(float angle) 
* 功    能：内外环逃逸程序合并
* 入口参数： xKRem,yKRem,angle卡住的位置的x，y坐标，和当前角度
* 出口参数：无
* 说    明：无
* 调用方法：无 
****************************************************************************/
void BackCar(float angle)
{
	angle=gRobot.pos.angle;
	if((xStick>-1400&&xStick<1400)&&(yStick>900&&yStick<3900))//内环
	{
		BackCarIn(angle);
	}
	else if((xStick<-1400||xStick>1400)||(yStick<900||yStick>3900))//外环
	{
		BackCarOut(angle);
	}
}	

