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
#include "config.h"
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
static float x = 0, y = 0, angle = 0;
static float angleError = 0; //目标角度与当前角度的偏差
static float aimAngle = 0;   //目标角度
static float distanceStraight = 0;//提前量
static float disError = 0;   //距离偏差
static float pidZongShuchu = 0, piddisShuchu = 0;
static float spacingError = 0;
static int lineChangeSymbol=0;



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
	if(fabs(gRobot.pos.x-getxRem())<1&&fabs(gRobot.pos.y-getyRem())<1&&gRobot.M!=0)
	{
		againstTime++;
	}
//	if (TRAVEL_SWITCH_LEFT==1&&TRAVEL_SWITCH_RIGHT==1)
//	{
//		againstTime++;
//	}
	else
	{
		againstTime = 0;
	}
	if (againstTime > 200)
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
*
名    称：void AgainstWall(float aimAngle,float angle)
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
	VelCrl(CAN2, 1, -5000 + AnglePidControl(angleError));
	VelCrl(CAN2, 2, 5000 + AnglePidControl(angleError));
	if (fabs(angleError) < 8)
	{
		if (CheckAgainstWall())
		{
			VelCrl(CAN2, 1, 0);
			VelCrl(CAN2, 2, 0);
			setErr(0,-(2400-getLeftAdc()),0);
			gRobot.laser.leftDistance=getLeftAdc();
			gRobot.turnTime = 8;
		}
	}
}
 /****************************************************************************
* 名    称：void Vchange(int lineChangeSymbol)
* 功    能：通过lineChangeSymbol改变内外环的速度
* 入口参数：lineChangeSymbol
* 出口参数：gRobot.M//给予轮子的脉冲
* 说    明：在函数内部修改vOut1,2等的值能够输出需要的速度
* 调用方法：无 
****************************************************************************/
int Vchange(int lineChangeSymbol)
{
	static float vOut1 = 1300; 																//外环速度
	static float vOut2 = 1600;
	static float vIn = 1100;  																//内环速度
	if (lineChangeSymbol < 1)
	{
		gRobot.M = vIn / (3.14f * WHEEL_DIAMETER) * 4096.f;
	}else if(lineChangeSymbol >=1&&lineChangeSymbol < 3)
	{
		gRobot.M = vOut2 / (3.14f * WHEEL_DIAMETER) * 4096.f;
	}
	else if (lineChangeSymbol >= 3)
	{
		gRobot.M = vOut1 / (3.14f * WHEEL_DIAMETER) * 4096.f;
	}
	return gRobot.M;
}

int turnTimeLead(int lineChangeSymbol)
{
	static int lead=0;
	if (lineChangeSymbol < 1)
	{
		lead=400;
	}else if(lineChangeSymbol >=1&&lineChangeSymbol < 3)
	{
		lead=900;
	}
	else if (lineChangeSymbol >= 3)
	{
		lead=900;
	}
	return lead;
}

void Pointparking(float Pointx,float Pointy)
{
//	static float V=700,gRobot.M;
	static float x=0,y=0,angle=0;
	static float aimAngle=0;														//目标角度
	static float angleError=0;													//目标角度与当前角度的偏差
//	static float disError=0;													//距离偏差
	static float /*a=-1,b=1,c=0,*/k;										//定义斜率
	static float 	spacingError;													//定义两个点之间的距离
	static float kAngle;																//直线角度（用actan发回的数据）
	static float dx,dy;
	x=gRobot.pos.x;																			//当前x坐标
	y=gRobot.pos.y;																			//当前y坐标
	angle=gRobot.pos.angle;															//当前角度
	spacingError=sqrt(pow(x-Pointx,2)+pow(y-Pointy,2));
	dx=x-Pointx;
	dy=y-Pointy;
	k=dy/dx;
	kAngle=atan(k)*180/PI;
	if((1000*dx>=-1)&&(1000*dx<=1))											//当k不存在的时候
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
		if(dy>0)																				//目标在以车为原点建立坐标系的左下角
		{
			aimAngle=kAngle+90;
		}else if(dy<0)																	//目标在以车为原点建立坐标系的左上角
		{
			aimAngle=90+kAngle;
		}
	}else if(dx<0)
	{
		if(dy>0)																				//目标在以车为原点建立坐标系的右下角
		{
			aimAngle=kAngle-90;
		}else if(dy<0)																	//目标在以车为原点建立坐标系的右上角
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
	if(fabs(spacingError)>200&&fabs(spacingError)<250)	//设立减速环带
	{
		VelCrl(CAN2, 1,4000+AnglePidControl(angleError));	//pid中填入的是差值
		VelCrl(CAN2, 2,-4000+AnglePidControl(angleError));
	}
	if(fabs(spacingError)<200&&fabs(spacingError)>150)
	{
		VelCrl(CAN2, 1,2000+AnglePidControl(angleError));	//pid中填入的是差值
		VelCrl(CAN2, 2,-2000+AnglePidControl(angleError));
	}
	if(fabs(spacingError)>50&&fabs(spacingError)<150)
	{		
		VelCrl(CAN2, 1,AnglePidControl(angleError));			//pid中填入的是差值
		VelCrl(CAN2, 2,-AnglePidControl(angleError));
	}
	
}
extern float  angle;												//定义角度
extern float posX ;	 												//定位系统返回的X坐标
extern float posY ;	 												//定位系统返回的Y坐标
void In2Out()																//基础扫场程序
{
		x = gRobot.pos.x;												//矫正过的x坐标
		y = gRobot.pos.y;												//矫正过的y坐标
		angle = gRobot.pos.angle; 							//矫正过的角度角度
		gRobot.M=Vchange(lineChangeSymbol);			//通过判定lineChangeSymbol给速度脉冲赋值
		switch (gRobot.turnTime)
		{
			case 0:
				disError = x-(600+ lineChangeSymbol*470);
				aimAngle=0;
				angleError=angleErrorCount(aimAngle,angle);
				distanceStraight=(3400+ lineChangeSymbol*350)-y;
				if(lineChangeSymbol<1)
				{
						if (fabs(distanceStraight) > turnTimeLead(lineChangeSymbol))
					{
						VelCrl(CAN2, 1, gRobot.M + AnglePidControl(angleError + onceDistancePidControl(disError))); //pid中填入的是差值
						VelCrl(CAN2, 2, -gRobot.M + AnglePidControl(angleError + onceDistancePidControl(disError)));
					}else if (fabs(distanceStraight) < turnTimeLead(lineChangeSymbol))
					{
						distanceStraight = 0;
						gRobot.turnTime = 1;
					}
				}else if(lineChangeSymbol>=1)
				{		
					if (fabs(distanceStraight) > turnTimeLead(lineChangeSymbol))
					{
						VelCrl(CAN2, 1, gRobot.M + AnglePidControl(angleError + distancePidControl(disError))); //pid中填入的是差值
						VelCrl(CAN2, 2, -gRobot.M + AnglePidControl(angleError + distancePidControl(disError)));
					}
					if (fabs(distanceStraight) < turnTimeLead(lineChangeSymbol))
					{
						distanceStraight = 0;
						gRobot.turnTime = 1;
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
			distanceStraight = -(600 +  lineChangeSymbol*470) - x;
			if (fabs(distanceStraight) > turnTimeLead(lineChangeSymbol))
			{
				VelCrl(CAN2, 1, gRobot.M + AnglePidControl(angleError + distancePidControl(disError))); //pid中填入的是差值
				VelCrl(CAN2, 2, -gRobot.M + AnglePidControl(angleError + distancePidControl(disError)));
			}
			if (fabs(distanceStraight) < turnTimeLead(lineChangeSymbol))
			{
				distanceStraight = 0;
				gRobot.turnTime = 2;
			}
			CheckOutline();
			pidZongShuchu = AnglePidControl(angleError + distancePidControl(disError));
			piddisShuchu = distancePidControl(disError);
		break;

		case 2:
			disError = x + (600 +  lineChangeSymbol*470); //小车距离与直线的偏差//不加绝对值是因为判断车在直线上还是直线下
			aimAngle = 180;
			angleError = angleErrorCount(aimAngle,angle);
			distanceStraight = y - (1400 -  lineChangeSymbol*350);//100
			if (fabs(distanceStraight) > turnTimeLead(lineChangeSymbol))
			{
				VelCrl(CAN2, 1, gRobot.M + AnglePidControl(angleError - distancePidControl(disError))); //pid中填入的是差值
				VelCrl(CAN2, 2, -gRobot.M + AnglePidControl(angleError - distancePidControl(disError)));
			}
			if (fabs(distanceStraight) < turnTimeLead(lineChangeSymbol))
			{
				distanceStraight = 0;
				gRobot.turnTime = 3;
			}
			CheckOutline();
			pidZongShuchu = AnglePidControl(angleError - distancePidControl(disError));
			piddisShuchu = distancePidControl(disError);
		break;

		case 3:
			disError = y - (1400 -  lineChangeSymbol*350); //初始值50//小车距离与直线的偏差//不加绝对值是因为判断车在直线上还是直线下
			aimAngle = -90;
			angleError = angleErrorCount(aimAngle,angle);
			distanceStraight = (600 +  lineChangeSymbol*470) - x;
			if (fabs(distanceStraight) > turnTimeLead(lineChangeSymbol))
			{
				VelCrl(CAN2, 1, gRobot.M + AnglePidControl(angleError - distancePidControl(disError))); //角度误差pid和距离误差相结合
				VelCrl(CAN2, 2, -gRobot.M + AnglePidControl(angleError - distancePidControl(disError)));
			}
			if (fabs(distanceStraight) < turnTimeLead(lineChangeSymbol))
			{
				distanceStraight = 0;
				gRobot.turnTime = 0; //重新进入循环
//				if (lineChangeSymbol < 4)
//				{
//					lineChangeSymbol++;
//				}
				lineChangeSymbol++;
				if (lineChangeSymbol == 3)
				{
					lineChangeSymbol=0;
					gRobot.turnTime = 5;
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
}

void Debug(void)
{
	
#define DEBUG_SWEEP 1
	
#define DEBUG 1

	
#if DEBUG==1
		USART_OUTF(gRobot.pos.x);
		USART_OUTF(gRobot.pos.y);
		USART_OUTF(posX);
		USART_OUTF(posY);
		
		USART_OUTF(angle);//gRobot.pos.angle
		USART_OUTF(angleError);
		USART_OUTF(spacingError);
		USART_OUTF(disError);
		USART_OUTF(piddisShuchu);
		USART_OUTF(pidZongShuchu);
		USART_OUTF(gRobot.turnTime);
		USART_OUTF(lineChangeSymbol);
		USART_OUT_CHAR("\r\n");
//		USART_OUT(USART1, (uint8_t *)"%d\t", (int)stickError);
//#elif
#endif
}

int LineChange(void)			   //设立缩圈函数，symbol=0,1,2时为外圈，3,4为内圈返回缩圈距离
{
	if (lineChangeSymbol < 3)
	{
		return 350 * lineChangeSymbol;
	}
	else if (lineChangeSymbol >= 3)
	{
		return 225 * lineChangeSymbol;
	}
	return 0;
}
void WalkTask2(void)
{
	
	//边走边看坐标对不对
		x = gRobot.pos.x;			//矫正过的x坐标
		y = gRobot.pos.y;			//矫正过的y坐标
		angle = gRobot.pos.angle; //矫正过的角度角度

		gRobot.M=Vchange(lineChangeSymbol);			//通过判定lineChangeSymbol给速度脉冲赋值
	
		switch (gRobot.turnTime)
		{
		case 0:
			disError = y - (500 + LineChange()); //初始值50//小车距离与直线的偏差//不加绝对值是因为判断车在直线上还是直线下
			aimAngle = -90;
			angleError = angleErrorCount(aimAngle,angle);
			distanceStraight = (2000 - LineChange()) - x;
			if (fabs(distanceStraight) > 900)
			{
				VelCrl(CAN2, 1, gRobot.M + AnglePidControl(angleError - distancePidControl(disError))); //角度误差pid和距离误差相结合
				VelCrl(CAN2, 2, -gRobot.M + AnglePidControl(angleError - distancePidControl(disError)));
			}
			if (fabs(distanceStraight) < 900)
			{
				distanceStraight = 0;
				gRobot.turnTime = 1;
			}
			pidZongShuchu = AnglePidControl(angleError - distancePidControl(disError));
			piddisShuchu = distancePidControl(disError);
			CheckOutline();
		break;

		case 1:
			disError = x - (1900 - LineChange()); //小车距离与直线的偏差//不加绝对值是因为判断车在直线上还是直线下
			aimAngle = 0;
			angleError = angleErrorCount(aimAngle,angle);
			distanceStraight = (4400 - LineChange()) - y;
			if (fabs(distanceStraight) > 900)
			{
				VelCrl(CAN2, 1, gRobot.M + AnglePidControl(angleError + distancePidControl(disError))); //pid中填入的是差值
				VelCrl(CAN2, 2, -gRobot.M + AnglePidControl(angleError + distancePidControl(disError)));
			}
			if (fabs(distanceStraight) < 900)
			{
				distanceStraight = 0;
				gRobot.turnTime = 2;
			}
			pidZongShuchu = AnglePidControl(angleError + distancePidControl(disError));
			piddisShuchu = distancePidControl(disError);
			CheckOutline();
		break;

		case 2:
			disError = y - (4400 - LineChange()); //小车距离与直线的偏差//不加绝对值是因为判断车在直线上还是直线下//4100
			aimAngle = 90;
			angleError = angleErrorCount(aimAngle,angle);
			distanceStraight = -(2000 - LineChange()) - x;
			if (fabs(distanceStraight) > 900)
			{
				VelCrl(CAN2, 1, gRobot.M + AnglePidControl(angleError + distancePidControl(disError))); //pid中填入的是差值
				VelCrl(CAN2, 2, -gRobot.M + AnglePidControl(angleError + distancePidControl(disError)));
			}
			if (fabs(distanceStraight) < 900)
			{
				distanceStraight = 0;
				gRobot.turnTime = 3;
			}
			pidZongShuchu = AnglePidControl(angleError + distancePidControl(disError));
			piddisShuchu = distancePidControl(disError);
			CheckOutline();
		break;

		case 3:
			disError = x + (2000 - LineChange()); //小车距离与直线的偏差//不加绝对值是因为判断车在直线上还是直线下
			aimAngle = 180;
			angleError = angleErrorCount(aimAngle,angle);
			distanceStraight = y - (500 + LineChange());//100
			if (fabs(distanceStraight) > 900)
			{
				VelCrl(CAN2, 1, gRobot.M + AnglePidControl(angleError - distancePidControl(disError))); //pid中填入的是差值
				VelCrl(CAN2, 2, -gRobot.M + AnglePidControl(angleError - distancePidControl(disError)));
			}
			if (fabs(distanceStraight) < 900)
			{
				distanceStraight = 0;
				gRobot.turnTime = 0; //重新进入循环
				if (lineChangeSymbol < 3)
				{
					lineChangeSymbol++;
				}
				if (lineChangeSymbol == 3)
				{
					gRobot.turnTime = 5;
				}
			}
			CheckOutline();
			pidZongShuchu = AnglePidControl(angleError - distancePidControl(disError));
			piddisShuchu = distancePidControl(disError);
		break;

		case 5:
				//NiShiZhenCircleBiHuan(1500,1100,0,2400);
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
		
			if(gRobot.turnTime!=8)
		{
			Debug();
		}
}
/****************************************************************************
* 名    称：CirlceSweep(void)	
* 功    能：主扫场控制程序
* 入口参数：无
* 出口参数：无
* 说    明：无
* 调用方法：无 
****************************************************************************/
void CirlceSweep(void)											//基础扫场程序
{
		x = gRobot.pos.x;												//矫正过的x坐标
		y = gRobot.pos.y;												//矫正过的y坐标
		angle = gRobot.pos.angle; 							//矫正过的角度角度
		gRobot.M=Vchange(lineChangeSymbol);			//通过判定lineChangeSymbol给速度脉冲赋值
		switch (gRobot.turnTime)
		{
			case 0:
				Line(600,3400,0,0,1);
			break;
				
			case 1:
				Line(-600,3400,90,1,1);
			break;

			case 2:
				Line(-600,1400,180,0,-1);
			break;

			case 3:
				Line(600,1400,-90,1,-1);
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
		
		
			case 11:
				if(250<gRobot.pos.x&&gRobot.pos.x<300)
					gRobot.turnTime=12;
				break;
				
			case 12:
				if(-50<gRobot.pos.x && gRobot.pos.x<0 && gRobot.pos.y<1700)
				{
					gRobot.turnTime=13;
				}
				NiShiZhenCircleBiHuan(1800,1100,0,2400);
				break;
				
			case 13:
				if(-100<gRobot.pos.x && gRobot.pos.x<-50 && gRobot.pos.y<1700)
				{
					gRobot.turnTime=14;
				}
				NiShiZhenCircleBiHuan(1800,1600,0,2400);
				break;
				
			case 14:
				if(-150<gRobot.pos.x && gRobot.pos.x<-100 && gRobot.pos.y<1700)
				{
					gRobot.turnTime=5;
				}
				NiShiZhenCircleBiHuan(1800,2100,0,2400);
				break;
				
				default:
				break;
		}
		CheckOutline();
}

void WalkTask1(void)
{
		x = gRobot.pos.x;												//矫正过的x坐标
		y = gRobot.pos.y;												//矫正过的y坐标
		angle = gRobot.pos.angle; 							//矫正过的角度角度
		gRobot.M=Vchange(lineChangeSymbol);			//通过判定lineChangeSymbol给速度脉冲赋值
		switch (gRobot.turnTime)
		{
				case 0:
					disError = x - (600 + lineChangeSymbol*450); //小车距离与直线的偏差//不加绝对值是因为判断车在直线上还是直线下
					aimAngle = 0;
					angleError = angleErrorCount(aimAngle,angle);
					distanceStraight = (3400 + lineChangeSymbol*350) - y;
					if(lineChangeSymbol<1)
				{
						if (fabs(distanceStraight) > turnTimeLead(lineChangeSymbol))
					{
						VelCrl(CAN2, 1, gRobot.M + AnglePidControl(angleError + onceDistancePidControl(disError))); //pid中填入的是差值
						VelCrl(CAN2, 2, -gRobot.M + AnglePidControl(angleError + onceDistancePidControl(disError)));
					}
						if (fabs(distanceStraight) < turnTimeLead(lineChangeSymbol))
					{
						distanceStraight = 0;
						gRobot.turnTime = 1;
					}
				}else if(lineChangeSymbol>=1)
				{		
					if (fabs(distanceStraight) > turnTimeLead(lineChangeSymbol))
					{
						VelCrl(CAN2, 1, gRobot.M + AnglePidControl(angleError + distancePidControl(disError))); //pid中填入的是差值
						VelCrl(CAN2, 2, -gRobot.M + AnglePidControl(angleError + distancePidControl(disError)));
					}
					if (fabs(distanceStraight) < turnTimeLead(lineChangeSymbol))
					{
						distanceStraight = 0;
						gRobot.turnTime = 1;
					}
				}
					pidZongShuchu = AnglePidControl(angleError + distancePidControl(disError));
					piddisShuchu = distancePidControl(disError);
					CheckOutline();
			 break;
			
			case 1:
				disError = y - (3400 + lineChangeSymbol*350); //小车距离与直线的偏差//不加绝对值是因为判断车在直线上还是直线下//4100
				aimAngle = 90;
				angleError = angleErrorCount(aimAngle,angle);
				distanceStraight = -(600 + lineChangeSymbol*470) - x;
				if (fabs(distanceStraight) > turnTimeLead(lineChangeSymbol))
				{
					VelCrl(CAN2, 1, gRobot.M + AnglePidControl(angleError + distancePidControl(disError))); //pid中填入的是差值
					VelCrl(CAN2, 2, -gRobot.M + AnglePidControl(angleError + distancePidControl(disError)));
				}
				if (fabs(distanceStraight) < turnTimeLead(lineChangeSymbol))
				{
					distanceStraight = 0;
					gRobot.turnTime = 2;
				}
				pidZongShuchu = AnglePidControl(angleError + distancePidControl(disError));
				piddisShuchu = distancePidControl(disError);
				CheckOutline();
			break;
		
			case 2:
				disError = x + (600 + lineChangeSymbol*470); //小车距离与直线的偏差//不加绝对值是因为判断车在直线上还是直线下
				aimAngle = 180;
				angleError = angleErrorCount(aimAngle,angle);
				distanceStraight = y - (1400 + lineChangeSymbol*350);//100
				if (fabs(distanceStraight) > turnTimeLead(lineChangeSymbol))
				{
					VelCrl(CAN2, 1, gRobot.M + AnglePidControl(angleError - distancePidControl(disError))); //pid中填入的是差值
					VelCrl(CAN2, 2, -gRobot.M + AnglePidControl(angleError - distancePidControl(disError)));
				}
				if (fabs(distanceStraight) < turnTimeLead(lineChangeSymbol))
				{
					distanceStraight = 0;
					gRobot.turnTime = 3; //重新进入循环
				}
				CheckOutline();
				pidZongShuchu = AnglePidControl(angleError - distancePidControl(disError));
				piddisShuchu = distancePidControl(disError);
			break;

			case 3:
				disError = y - (1400 - lineChangeSymbol*350); //初始值50//小车距离与直线的偏差//不加绝对值是因为判断车在直线上还是直线下
				aimAngle = -90;
				angleError = angleErrorCount(aimAngle,angle);
				distanceStraight = (600 + lineChangeSymbol*470) - x;
				if (fabs(distanceStraight) > turnTimeLead(lineChangeSymbol))
				{
					VelCrl(CAN2, 1, gRobot.M + AnglePidControl(angleError - distancePidControl(disError))); //角度误差pid和距离误差相结合
					VelCrl(CAN2, 2, -gRobot.M + AnglePidControl(angleError - distancePidControl(disError)));
				}
				if (fabs(distanceStraight) < turnTimeLead(lineChangeSymbol))
				{
					distanceStraight = 0;
					gRobot.turnTime = 0;
					if (lineChangeSymbol < 3)
					{
						lineChangeSymbol++;
					}
					if (lineChangeSymbol == 3)
					{
						gRobot.turnTime = 5;
					}
				}
				pidZongShuchu = AnglePidControl(angleError - distancePidControl(disError));
				piddisShuchu = distancePidControl(disError);
				CheckOutline();
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
}
/********************* (C) COPYRIGHT NEU_ACTION_2017 ****************END OF FILE************************/

