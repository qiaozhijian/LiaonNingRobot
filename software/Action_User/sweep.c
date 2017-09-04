#include "sweep.h"
/**
  ******************************************************************************
  * @file	  moveBase.c
  * @author	  Action
  * @version   V1.0.0
  * @date	  2017/07/24
  * @brief	 2017省赛底盘运动控制部分
  ******************************************************************************
  * @attention
  *			None
  ******************************************************************************
  */
/* Includes -------------------------------------------------------------------------------------------*/
#include "stm32f4xx.h"
#include "math.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_rcc.h"
#include "timer.h"
#include "gpio.h"
#include "usart.h"
#include "can.h"
#include "elmo.h"
#include "stm32f4xx_it.h"
#include "stm32f4xx_usart.h"
#include "stm32f4xx_adc.h"
#include "fix.h"           // Device header
#include "config.h"
#include "adc.h"
#include "arm_math.h"
#include "task.h"
#include "tools.h"
#include "shoot.h"
/* Private typedef ------------------------------------------------------------------------------------*/
/* Private define -------------------------------------------------------------------------------------*/
/* Private macro --------------------------------------------------------------------------------------*/
/* Private variables ----------------------------------------------------------------------------------*/
/* Private function prototypes ------------------------------------------------------------------------*/
/* Private functions ----------------------------------------------------------------------------------*/
/****************************************************************************
 
    宏定义区
 
****************************************************************************/

/****************************************************************************
 
    变量定义区
 
****************************************************************************/
//static float x = 0, y = 0, angle = 0;//当前的x,y坐标和angle
//static float aimAngle = 0;   //目标角度
//static float angleError = 0; //目标角度与当前角度的偏差
extern Robot_t gRobot;
static float xStick, yStick;															   //卡住时存储的位置数据
static float M;																   //速度脉冲
static int turnTime = 0;													   //切换转换方向
static int turnTimeRemember;												   //记住在卡死的时候是什么直线的状态，等倒车case结束后让重新填装
/****************************************************************************
* 名    称：pid函数1
* 功    能：计算偏差作为pid
* 入口参数：偏差
* 出口参数：调整量
* 说    明：无
* 调用方法：无 
****************************************************************************/
/********************************************/ //角度pid函数
float ParkingAnglePidControl(float ERR)
{
	static int ERR_OLD = 0;
	static float Kp = 70; //90
	static float Ki = 0;
	static float Kd = 10;
	static float OUTPUT;
	OUTPUT = Kp * ERR + Kd * (ERR - ERR_OLD) +Ki*0.0f;
	ERR_OLD = ERR;
	return OUTPUT;
}
/********************************************/ //角度pid函数
float AnglePidControl(float ERR)
{
	static int ERR_OLD = 0;
	static float Kp = 250.f; //200
	static float Ki = 0.f;
	static float Kd = 50.f;
	static float OUTPUT;
	OUTPUT = Kp * ERR + Kd * (ERR - ERR_OLD) +Ki*0.0f;
	ERR_OLD = ERR;
	return OUTPUT;
}
/**********************************************/ //距离pid函数
float distancePidControl(float ERR)
{
	static int ERR_OLD = 0;
	static float Kp = 0.0085; //0.02//0.03
	static float Ki = 0;
	static float Kd = 0;
	static float OUTPUT;
	OUTPUT = Kp * ERR + Kd * (ERR - ERR_OLD) +Ki*0.0f;
	ERR_OLD = ERR;
	return OUTPUT;
}
float onceDistancePidControl(float ERR)
{
	static int ERR_OLD = 0;
	static float Kp = 0.08; //0.1
	static float Ki = 0;
	static float Kd = 0;
	static float OUTPUT;
	OUTPUT = Kp * ERR + Kd * (ERR - ERR_OLD) +Ki*0.0f;
	ERR_OLD = ERR;
	return OUTPUT;
}
float spacingPidControl(float ERR)
{
	static int ERR_OLD = 0;
	static float Kp = 40; //0.1
	static float Ki = 0;
	static float Kd = 1;
	static float OUTPUT;
	OUTPUT = Kp * ERR + Kd * (ERR - ERR_OLD) +Ki*0.0f;
	ERR_OLD = ERR;
	return OUTPUT;
}

/****************************************************************************
* 名    称：float angleErrorCount(float aimAngle,float angle)
* 功    能：计算角度偏差作为pid
* 入口参数：aimAngle: 目标角度
						angle：   当前角度
* 出口参数：angleError：角度错误
* 说    明：无
* 调用方法：无 
****************************************************************************/
float angleErrorCount(float aimAngle,float angle)//计算角度偏差作为pid
{
	static float angleError=0;
	angleError=aimAngle-angle;
	if (angleError > 180) //防止出现乱转
	{
		angleError = angleError - 360;
	}
	else if (angleError < -180)
	{
		angleError = angleError + 360;
	}
	return angleError;
}

/****************************************************************************
* 名    称：void Parking(void)
* 功    能：走直线停车函数函数
* 入口参数：无
* 出口参数：无
* 说    明：当前是停在x=1000 y=0
* 调用方法：无 
****************************************************************************/
//void Parking(void)
//{
//	static float x = 0, y = 0, angle = 0;
//	static float distanceStraight = 0;//提前量
//	static float disError = 0;   //距离偏差
//	static float pidZongShuchu = 0, piddisShuchu = 0;
//	static float spacingError = 0;
//	static int i=0;
//	switch(i)
//	{
//		case 0:
//			disError = y - 100 ; //初始值50//小车距离与直线的偏差//不加绝对值是因为判断车在直线上还是直线下
//			aimAngle = -90;
//			angleError = angleErrorCount(aimAngle,angle);
//			distanceStraight = 1000  - x;
//			if (fabs(distanceStraight) > 200)
//			{
//				VelCrl(CAN1, 1, 5000 + ParkingAnglePidControl(angleError - distancePidControl(disError))); //角度误差pid和距离误差相结合
//				VelCrl(CAN1, 2, -5000 + ParkingAnglePidControl(angleError - distancePidControl(disError)));
//			}
//			if (fabs(distanceStraight) < 200)
//			{
//				distanceStraight = 0;
//				i = 1;
//			}
//			pidZongShuchu = ParkingAnglePidControl(angleError - distancePidControl(disError));
//			piddisShuchu = distancePidControl(disError);
//			CheckOutline();
//			break;

//		case 1:
//			disError = x - 1000; //小车距离与直线的偏差//不加绝对值是因为判断车在直线上还是直线下
//			aimAngle = 0;
//			angleError = angleErrorCount(aimAngle,angle);
//			if (fabs(disError) > 100)
//			{
//				VelCrl(CAN1, 1, 5000 + ParkingAnglePidControl(angleError + distancePidControl(disError))); //pid中填入的是差值
//				VelCrl(CAN1, 2, -5000 + ParkingAnglePidControl(angleError + distancePidControl(disError)));
//			}
//			if (fabs(disError) < 100)
//			{
//				i = 2;
//			}
//			pidZongShuchu = ParkingAnglePidControl(angleError + distancePidControl(disError));
//			piddisShuchu = distancePidControl(disError);
//			CheckOutline();
//			break;
//			
//		case 2:
//			spacingError=y;
//			aimAngle=0;
//			angleError=angleErrorCount(aimAngle,angle);
//			VelCrl(CAN1, 1, ParkingAnglePidControl(angleError));//pid中填入的是差值
//			VelCrl(CAN1, 2, ParkingAnglePidControl(angleError));
//			if(fabs(angleError)<5)
//			{
//				VelCrl(CAN1, 1,-spacingPidControl(spacingError));//pid中填入的是差值
//				VelCrl(CAN1, 2, spacingPidControl(spacingError));
//			}
//			pidZongShuchu=ParkingAnglePidControl(angleError);
//			piddisShuchu=spacingPidControl(spacingError);
//		break;
//		
//		default:
//		break;
//	}
//}

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
			turnTime = turnTimeRemember;
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
			turnTime = turnTimeRemember;
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
	turnTimeRemember = turnTime;
	xError = gRobot.pos.x - getxRem();
	yError = gRobot.pos.y - getyRem();
	if (fabs(xError) < 1 && fabs(yError) < 1 && M != 0)
	{
		stickError++;
	}
	else
	{
		stickError = 0;
	}
	if (stickError > 80)
	{
		xStick = getxRem();//记住卡死的坐标
		yStick = getyRem();
		turnTime = 7;
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
		BackCarOut(angle);
	}
	else if((xStick<-1400||xStick>1400)||(yStick<900||yStick>3900))//外环
	{
		BackCarOut(angle);
	}
}	

 /****************************************************************************
* 名    称：int CheckAgainstWall(void)
* 功    能：检查是否靠墙
* 入口参数：无
* 出口参数：无
* 说    明：无
* 调用方法：无 
****************************************************************************/
int CheckAgainstWall(void)
{
	static int againstTime=0;//靠在墙上的时间
	if (TRAVEL_SWITCH_LEFT==1&&TRAVEL_SWITCH_RIGHT==1)
	{
		againstTime++;
	}
	else
	{
		againstTime = 0;
	}
	if (againstTime > 80)
	{
		againstTime=0;
		return 1; //另外一种标志方案
	}
	else
	{
		return 0;
	}
}
 /****************************************************************************
* 名    称：void AgainstWall(float aimAngle,float angle)
* 功    能：倒车靠墙程序
* 入口参数：无
* 出口参数：无
* 说    明：当前是停在x=1000 y=0
* 调用方法：无 
****************************************************************************/
void AgainstWall(float aimAngle,float angle)
{
	static float angleError=0;
	angleError = angleErrorCount(aimAngle,angle);
	VelCrl(CAN1, 1, -5000 + AnglePidControl(angleError));
	VelCrl(CAN1, 2, 5000 + AnglePidControl(angleError));
//	if (fabs(angleError) < 8)
//	{
//		if (CheckAgainstWall())
//		{
//			setErr(0,-(2400-getLeftAdc()),0);
//			turnTime = 8;
//		}
//	}
}

 /****************************************************************************
* 名    称：void Vchange(int lineChangeSymbol)
* 功    能：通过lineChangeSymbol改变内外环的速度
* 入口参数：lineChangeSymbol
* 出口参数：M//给予轮子的脉冲
* 说    明：在函数内部修改vOut1,2等的值能够输出需要的速度
* 调用方法：无 
****************************************************************************/
int Vchange(int lineChangeSymbol)
{
	static float vOut1 = 1100; //外环速度
	static float vOut2 = 1100;//中环速度
	static float vIn = 1100;  //内环速度
	if (lineChangeSymbol < 1)
	{
		M = vIn / (PI * WHEEL_DIAMETER) * COUNT_PER_ROUND;
	}else if(lineChangeSymbol >=1&&lineChangeSymbol < 3)
	{
		M = vOut2 / (PI * WHEEL_DIAMETER) * COUNT_PER_ROUND;
	}
	else if (lineChangeSymbol >= 3)
	{
		M = vOut1 / (PI * WHEEL_DIAMETER) * COUNT_PER_ROUND;
	}
	return M;
}

int turnTimeLead(int lineChangeSymbol)
{
	static int lead=0;
	if (lineChangeSymbol < 1)
	{
		lead=700;
	}else if(lineChangeSymbol >=1&&lineChangeSymbol < 3)
	{
		lead=800;
	}
	else if (lineChangeSymbol >= 3)
	{
		lead=800;
	}
	return lead;
}

void Pointparking(float Pointx,float Pointy)
{
//	static float V=700,M;
	static float x=0,y=0,angle=0;
	static float aimAngle=0;//目标角度
	static float angleError=0;//目标角度与当前角度的偏差
//	static float disError=0;//距离偏差
	static float /*a=-1,b=1,c=0,*/k;//定义斜率
	static float 	spacingError;//定义两个点之间的距离
	static float kAngle;//直线角度（用actan发回的数据）
	static float dx,dy;
	x=gRobot.pos.x;//当前x坐标
	y=gRobot.pos.y;//当前y坐标
	angle=gRobot.pos.angle;//当前角度
	spacingError=sqrt(pow(x-Pointx,2)+pow(y-Pointy,2));
	dx=x-Pointx;
	dy=y-Pointy;
	k=dy/dx;
	kAngle=atan(k)*180/PI;
	if((1000*dx>=-1)&&(1000*dx<=1))//当k不存在的时候
	{
		if(dy>0)
		{
			aimAngle=-180;
		}else if(dy<0)
		{
			aimAngle=0;
		}
	}
	
	if((dx>0))
	{
		if(dy>0)//目标在以车为原点建立坐标系的左下角
		{
			aimAngle=kAngle+90;
		}else if(dy<0)//目标在以车为原点建立坐标系的左上角
		{
			aimAngle=90+kAngle;
		}
	}else if(dx<0)
	{
		if(dy>0)//目标在以车为原点建立坐标系的右下角
		{
			aimAngle=kAngle-90;
		}else if(dy<0)//目标在以车为原点建立坐标系的右上角
		{
			aimAngle=-(90-kAngle);
		}
	}
	angleError=angleErrorCount(aimAngle,angle);
	if(fabs(spacingError)>250)
	{
		VelCrl(CAN2, 1,6000+AnglePidControl(angleError));//pid中填入的是差值
		VelCrl(CAN2, 2,-6000+AnglePidControl(angleError));
	}
	if(fabs(spacingError)>200&&fabs(spacingError)<250)//设立减速环带
	{
		VelCrl(CAN2, 1,4000+AnglePidControl(angleError));//pid中填入的是差值
		VelCrl(CAN2, 2,-4000+AnglePidControl(angleError));
	}
	if(fabs(spacingError)<200&&fabs(spacingError)>150)
	{
		VelCrl(CAN2, 1,2000+AnglePidControl(angleError));//pid中填入的是差值
		VelCrl(CAN2, 2,-2000+AnglePidControl(angleError));
	}
	if(fabs(spacingError)>50&&fabs(spacingError)<150)
	{	
		VelCrl(CAN2, 1,AnglePidControl(angleError));//pid中填入的是差值
		VelCrl(CAN2, 2,-AnglePidControl(angleError));
	}
//		USART_OUT(USART1,(uint8_t*) "%d\t",(int)GetPosX());
//		USART_OUT(USART1,(uint8_t*) "%d\t",(int)GetPosY());
//		USART_OUT(USART1,(uint8_t*) "%d\t",(int)dx);
//		USART_OUT(USART1,(uint8_t*) "%d\t",(int)dy);
//		USART_OUT(USART1,(uint8_t*) "%d\t",(int)angleError);//角度偏差
//		USART_OUT(USART1,(uint8_t*) "%d\t",(int)kAngle);//
//		USART_OUT(USART1,(uint8_t*) "%d\t",(int)spacingError);//距离
//		USART_OUT(USART1,(uint8_t*) "%d\t",(int)out1);
//		USART_OUT(USART1,(uint8_t*) "%d\t",(int)v1);
//		USART_OUT(USART1,(uint8_t*) "%d\r\n",(int)v2);
}

void Sweep()//基础扫场程序
{
		static float x = 0, y = 0, angle = 0;
		static float aimAngle = 0;   //目标角度
		static float angleError = 0; //目标角度与当前角度的偏差
		static float distanceStraight = 0;//提前量
		static float disError = 0;   //距离偏差
		static float pidZongShuchu = 0, piddisShuchu = 0;
		static float spacingError = 0;
		static int lineChangeSymbol=0;
		x = gRobot.pos.x;			//矫正过的x坐标
		y = gRobot.pos.y;			//矫正过的y坐标
		angle = gRobot.pos.angle; //矫正过的角度角度
		M=Vchange(lineChangeSymbol);			//通过判定lineChangeSymbol给速度脉冲赋值
		switch (turnTime)
		{
			case 0:
				disError = x-(600+ lineChangeSymbol*500);
				aimAngle=0;
				angleError=angleErrorCount(aimAngle,angle);;
				distanceStraight=(3400+ lineChangeSymbol*350)-y;
				if(lineChangeSymbol<1)
				{
						if (fabs(distanceStraight) > turnTimeLead(lineChangeSymbol))
					{
						VelCrl(CAN2, 1, M + AnglePidControl(angleError + onceDistancePidControl(disError))); //pid中填入的是差值
						VelCrl(CAN2, 2, -M + AnglePidControl(angleError + onceDistancePidControl(disError)));
					}else if (fabs(distanceStraight) < turnTimeLead(lineChangeSymbol))
					{
						distanceStraight = 0;
						turnTime = 1;
					}
				}else if(lineChangeSymbol>=1)
				{		
					if (fabs(distanceStraight) > turnTimeLead(lineChangeSymbol))
					{
						VelCrl(CAN2, 1, M + AnglePidControl(angleError + distancePidControl(disError))); //pid中填入的是差值
						VelCrl(CAN2, 2, -M + AnglePidControl(angleError + distancePidControl(disError)));
					}
					if (fabs(distanceStraight) < turnTimeLead(lineChangeSymbol))
					{
						distanceStraight = 0;
						turnTime = 1;
					}
				}
				CheckOutline();
				pidZongShuchu = AnglePidControl(angleError + distancePidControl(disError));
				piddisShuchu = distancePidControl(disError);
			break;
				
		case 1:
			disError = y - (3400 +  lineChangeSymbol*350); //小车距离与直线的偏差//不加绝对值是因为判断车在直线上还是直线下//4100
			aimAngle = 90;
			angleError = angleErrorCount(aimAngle,angle);
			distanceStraight = -(600 +  lineChangeSymbol*500) - x;
			if (fabs(distanceStraight) > turnTimeLead(lineChangeSymbol))
			{
				VelCrl(CAN2, 1, M + AnglePidControl(angleError + distancePidControl(disError))); //pid中填入的是差值
				VelCrl(CAN2, 2, -M + AnglePidControl(angleError + distancePidControl(disError)));
			}
			if (fabs(distanceStraight) < turnTimeLead(lineChangeSymbol))
			{
				distanceStraight = 0;
				turnTime = 2;
			}
			CheckOutline();
			pidZongShuchu = AnglePidControl(angleError + distancePidControl(disError));
			piddisShuchu = distancePidControl(disError);
		break;

		case 2:
			disError = x + (600 +  lineChangeSymbol*500); //小车距离与直线的偏差//不加绝对值是因为判断车在直线上还是直线下
			aimAngle = 180;
			angleError = angleErrorCount(aimAngle,angle);
			distanceStraight = y - (1400 -  lineChangeSymbol*350);//100
			if (fabs(distanceStraight) > turnTimeLead(lineChangeSymbol))
			{
				VelCrl(CAN2, 1, M + AnglePidControl(angleError - distancePidControl(disError))); //pid中填入的是差值
				VelCrl(CAN2, 2, -M + AnglePidControl(angleError - distancePidControl(disError)));
			}
			if (fabs(distanceStraight) < turnTimeLead(lineChangeSymbol))
			{
				distanceStraight = 0;
				turnTime = 3;
			}
			CheckOutline();
			pidZongShuchu = AnglePidControl(angleError - distancePidControl(disError));
			piddisShuchu = distancePidControl(disError);
		break;

		case 3:
			disError = y - (1400 -  lineChangeSymbol*350); //初始值50//小车距离与直线的偏差//不加绝对值是因为判断车在直线上还是直线下
			aimAngle = -90;
			angleError = angleErrorCount(aimAngle,angle);
			distanceStraight = (600 +  lineChangeSymbol*500) - x;
			if (fabs(distanceStraight) > turnTimeLead(lineChangeSymbol))
			{
				VelCrl(CAN2, 1, M + AnglePidControl(angleError - distancePidControl(disError))); //角度误差pid和距离误差相结合
				VelCrl(CAN2, 2, -M + AnglePidControl(angleError - distancePidControl(disError)));
			}
			if (fabs(distanceStraight) < turnTimeLead(lineChangeSymbol))
			{
				distanceStraight = 0;
				turnTime = 0; //重新进入循环
//				if (lineChangeSymbol < 4)
//				{
//					lineChangeSymbol++;
//				}
				lineChangeSymbol++;
				if (lineChangeSymbol == 4)
				{
					lineChangeSymbol=0;
					turnTime = 5;
				}
			}
			CheckOutline();
			pidZongShuchu = AnglePidControl(angleError - distancePidControl(disError));
			piddisShuchu = distancePidControl(disError);
		break;
			

		case 5:
			AgainstWall(0,angle);
		break;
		
		case 7:
			BackCar(angle);
		break;
			
		case 8:
			fireTask();
		break;

		default:
		break;
		}

		USART_OUT(UART5, (uint8_t *)"%d\t", (int)gRobot.pos.x);
		USART_OUT(UART5, (uint8_t *)"%d\t", (int)gRobot.pos.y);
		USART_OUT(UART5, (uint8_t *)"%d\t", (int)angle);//gRobot.pos.angle
		USART_OUT(UART5, (uint8_t *)"%d\t", (int)angleError);
		USART_OUT(UART5, (uint8_t *)"%d\t", (int)spacingError);
		USART_OUT(UART5, (uint8_t *)"%d\t", (int)disError);
		USART_OUT(UART5, (uint8_t *)"%d\t", (int)piddisShuchu);
		USART_OUT(UART5, (uint8_t *)"%d\t", (int)pidZongShuchu);
		USART_OUT(UART5, (uint8_t *)"%d\t", (int)turnTime);
		USART_OUT(UART5, (uint8_t *)"%d\t", (int)lineChangeSymbol);
//		USART_OUT(USART1, (uint8_t *)"%d\t", (int)stickError);
		USART_OUT(UART5, (uint8_t *)"%d\t", (int)xStick);
		USART_OUT(UART5, (uint8_t *)"%d\t", (int)yStick);
		USART_OUT(UART5, (uint8_t *)"%d\r\n", (int)turnTimeRemember);
}

/********************* (C) COPYRIGHT NEU_ACTION_2017 ****************END OF FILE************************/
