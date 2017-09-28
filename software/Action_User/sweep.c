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
extern Robot_t gRobot;
static int lineChangeSymbol=0;
/*****有必要的全局变量可以****/


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
	//这两个要结合在一起，不能在矫正的时候卡死
//	if(fabs(gRobot.walk_t.pos.x-getxRem())<1&&fabs(gRobot.walk_t.pos.y-getyRem())<1&&gRobot.walk_t.left.base!=0)
//	{
//		againstTime++;
//	}
	if (TRAVEL_SWITCH_LEFT==1&&TRAVEL_SWITCH_RIGHT==1)
	{
		againstTime++;
	}
	else
	{
		againstTime = 0;
	}
	if (againstTime > 30)
	{
		againstTime=0;
		return 1; 	//另外一种标志方案
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
void AgainstWall(float aimAngle,float angle,float spacingError)
{
	gRobot.walk_t.pid.angleError = angleErrorCount(aimAngle,angle);
	VelCrl(CAN2, 1, -5000 + AnglePidControl(gRobot.walk_t.pid.angleError)-AgainstWallPidControl(spacingError));
	VelCrl(CAN2, 2, 5000 + AnglePidControl(gRobot.walk_t.pid.angleError)+AgainstWallPidControl(spacingError));
//	if (fabs(gRobot.walk_t.pid.angleError) < 8)
//	{
//		if (CheckAgainstWall())
//		{
//			VelCrl(CAN2, 1, 0);
//			VelCrl(CAN2, 2, 0);
//		}
//	}
}
 /****************************************************************************
* 名    称：void Vchange(int lineChangeSymbol)
* 功    能：通过lineChangeSymbol改变内外环的速度
* 入口参数：lineChangeSymbol
* 出口参数：gRobot.M//给予轮子的脉冲
* 说    明：在函数内部修改vOut1,2等的值能够输出需要的速度
* 调用方法：无 
****************************************************************************/
void Vchange(int lineChangeSymbol)
{
	//最外环速度
	const float vOut1 = 1300.0f; 	
	//中环速度
	const float vOut2 = 1600;
	//内环速度
	const float vIn = 1500;  																//内环速度
	if (lineChangeSymbol < 1)
	{
		gRobot.walk_t.left.base=gRobot.walk_t.right.base=vIn / (3.14f * WHEEL_DIAMETER) * 4096.f;
	}else if(lineChangeSymbol >=1&&lineChangeSymbol < 3)
	{
		gRobot.walk_t.left.base=gRobot.walk_t.right.base=vOut2 / (3.14f * WHEEL_DIAMETER) * 4096.f;
	}
	else if (lineChangeSymbol >= 3)
	{
		gRobot.walk_t.left.base=gRobot.walk_t.right.base=vOut1 / (3.14f * WHEEL_DIAMETER) * 4096.f;
	}
}
 /****************************************************************************
* 名    称：turnTimeLead(int lineChangeSymbol)
* 功    能：给定提前量
* 入口参数：lineChangeSymbol
* 出口参数：lead提前量
* 说    明：在函数内部修改vOut1,2等的值能够输出需要的速度
* 调用方法：无 
****************************************************************************/
int turnTimeLead(int lineChangeSymbol)
{
  int lead=0;
	if (lineChangeSymbol < 1)
	{
		lead=500;
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

/****************************************************************************
* 名    称：Pointparking
* 功    能：定点停车
* 入口参数：目标点(Pointx,Pointy)
* 出口参数：gRobot.//给予轮子的脉冲
* 说    明：
* 调用方法：无 
****************************************************************************/
int Pointparking(float Pointx,float Pointy)
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
	x=gRobot.walk_t.pos.x;															//当前x坐标
	y=gRobot.walk_t.pos.y;															//当前y坐标
	angle=gRobot.walk_t.pos.angle;											//当前角度
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
		VelCrl(CAN2, 1,6000+AnglePidControl(angleError));			//pid中填入的是差值
		VelCrl(CAN2, 2,-6000+AnglePidControl(angleError));
	}
	if(fabs(spacingError)>200&&fabs(spacingError)<250)			//设立减速环带
	{
		VelCrl(CAN2, 1,4000+AnglePidControl(angleError));		  //pid中填入的是差值
		VelCrl(CAN2, 2,-4000+AnglePidControl(angleError));
	}
	if(fabs(spacingError)<200&&fabs(spacingError)>80)
	{
		VelCrl(CAN2, 1,2000+AnglePidControl(angleError));			//pid中填入的是差值
		VelCrl(CAN2, 2,-2000+AnglePidControl(angleError));
	}
	if(fabs(spacingError)>0&&fabs(spacingError)<80)
	{		
		VelCrl(CAN2, 1,0);																		//pid中填入的是差值
		VelCrl(CAN2, 2,0);
		return 1;
	}
	return 0;
}

//void Debug(void)
//{
//	
//#define DEBUG_SWEEP 1
//	
//#define DEBUG 1

//	
//#if DEBUG==1
//		USART_OUTF(gRobot.walk_t.pos.x);
//		USART_OUTF(gRobot.walk_t.pos.y);
//		USART_OUTF(angle);//gRobot.walk_t.pos.angle
//		USART_OUTF(angleError);
//		USART_OUTF(spacingError);
//		USART_OUTF(disError);
//		USART_OUTF(piddisShuchu);
//		USART_OUTF(pidZongShuchu);
//		USART_OUTF(gRobot.turnTime);
//		USART_OUTF(lineChangeSymbol);
//		USART_OUT_CHAR("\r\n");
////		USART_OUT(USART1, (uint8_t *)"%d\t", (int)stickError);
////#elif
//#endif
//}
/****************************************************************************
* 名    称：LineChange()	
* 功    能：设立缩圈函数
* 入口参数：无
* 出口参数：返回缩圈距离
* 说    明：symbol=0,1,2时为外圈，3,4为内圈
* 调用方法：无 
****************************************************************************/
int LineChange(void)			   //，
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
/****************************************************************************
* 名    称：In2Out()	
* 功    能：主扫场控制程序
* 入口参数：lineChangeSymbol(改变第一圈的位置)
* 出口参数：无
* 说    明：无
* 调用方法：无 
* 注    意: 0:逆 1:顺
****************************************************************************/
void In2Out(int lineChangeSymbol,int direction)
{
	//条件判断
	In2OutChange();
	switch(direction)
	{
		case 0:
			ClockWise();
			break;
		case 1:
			AntiClockWise();
			break;
		default:
			break;
	}
}
/****************************************************************************
* 名    称：WalkOne()	
* 功    能：挡车程序
* 入口参数：无
* 出口参数：无
* 说    明：无
* 调用方法：无 
****************************************************************************/
void WalkOne()
{
	static int turntime=0;
	switch(turntime)
	{
		case 0:
			if(200<gRobot.walk_t.pos.x&&gRobot.walk_t.pos.x<300)
			  turntime=1;
		  ShunShiZhenCircleBiHuan(800,500,-100,600);
		break;
		case 1:
			Ygoal(300,1400,-90,-1,0);
		break;
		case 2:
			In2Out(0,1);
		break;
		default:
		  break;
	}
}
/****************************************************************************
* 名    称：LaserStart()	
* 功    能：激光启动程序
* 入口参数：无
* 出口参数：无
* 说    明：无
* 调用方法：无 
****************************************************************************/
int LaserStart(void)
{
	static int lasercount=0;       //让激光执行一段时间再生效
	if(lasercount<300)    
	 lasercount++;
	if(getLeftAdc()<600 && lasercount==300)
	{
		lasercount=302;
		gRobot.walk_t.laser.statue=1;
		gRobot.walk_t.circlechange.direction=0;
		return 0;
	}
	if(getRightAdc()<600 && lasercount==300)
	{
		lasercount=302;
		gRobot.walk_t.laser.statue=1;
		gRobot.walk_t.circlechange.direction=1;
		return 0;
	}
	if(getRightAdc()<600 && getLeftAdc()<600&&lasercount==300)
	{
		lasercount=302;
	  gRobot.walk_t.laser.statue=2;
		return 0;
	}
	//防止跳变
	if(lasercount>300)
	{
		lasercount=0;
	}
	return 1;
} 
/****************************************************************************
* 名    称：Xgoal()
* 功    能：走目标直线X
* 入口参数：无
* 出口参数：无
* 说    明：(0度)sign=1;(180度)sign=-1;lineChangeSymbol正常情况下为0
* 调用方法：无 
****************************************************************************/
void Xgoal(float aimX,float aimY,float aimAngle,int sign,int lineChangeSymbol)
{
	  static float x = 0, y = 0, angle = 0;				
		x=gRobot.walk_t.pos.x;											//赋值当前姿态
		y=gRobot.walk_t.pos.y;
		angle=gRobot.walk_t.pos.angle;
	
	  gRobot.walk_t.pid.disError = x-(aimX+ sign*lineChangeSymbol*470);
		gRobot.walk_t.pid.angleError=angleErrorCount(aimAngle,angle);;
		gRobot.walk_t.pid.distanceStraight=sign*(aimY+ sign*lineChangeSymbol*350)-sign*y;
	
		VelCrl(CAN2, 1, gRobot.walk_t.right.base + AnglePidControl(gRobot.walk_t.pid.angleError +sign* distancePidControl(gRobot.walk_t.pid.disError))); //pid中填入的是差值
		VelCrl(CAN2, 2, -gRobot.walk_t.right.base+ AnglePidControl(gRobot.walk_t.pid.angleError +sign* distancePidControl(gRobot.walk_t.pid.disError)));
}
/****************************************************************************
* 名    称：Ygoal()
* 功    能：走目标直线Y
* 入口参数：aimX:目标X aimY:目标y aimAngle:目标角度,sign:直线标志,
           lineChangeSymbol：缩圈量
* 出口参数：无
* 说    明：(90度)sign=1;(-90度)sign=-1;lineChangeSymbol正常情况下为0
* 调用方法：无 
****************************************************************************/
void Ygoal(float aimX,float aimY,float aimAngle,int sign,int lineChangeSymbol)
{
		static float x = 0, y = 0, angle = 0;				
		x=gRobot.walk_t.pos.x;											//赋值当前姿态
		y=gRobot.walk_t.pos.y;
		angle=gRobot.walk_t.pos.angle;
		
		gRobot.walk_t.pid.disError = y - (aimY +  sign*lineChangeSymbol*350); //小车距离与直线的偏差//不加绝对值是因为判断车在直线上还是直线下//4100
		gRobot.walk_t.pid.angleError = angleErrorCount(aimAngle,angle);
		gRobot.walk_t.pid.distanceStraight = (aimX -sign*lineChangeSymbol*470) - x;
	
		VelCrl(CAN2, 1, gRobot.walk_t.right.base + AnglePidControl(gRobot.walk_t.pid.angleError +sign* distancePidControl(gRobot.walk_t.pid.disError))); //pid中填入的是差值
		VelCrl(CAN2, 2, -gRobot.walk_t.right.base + AnglePidControl(gRobot.walk_t.pid.angleError +sign* distancePidControl(gRobot.walk_t.pid.disError)));
}
/****************************************************************************
* 名    称：Run()	
* 功    能：程序运行分类
* 入口参数：无
* 出口参数：无
* 说    明：无
* 调用方法：无 
****************************************************************************/
void Run(void)
{
	switch(gRobot.walk_t.laser.statue)
	{
		case 1:
			//逆时针
			//顺时针
			In2Out(1,gRobot.walk_t.circlechange.direction);
			break;
		case 2:
			WalkOne();
			break;
		case 3:
			break;
		default:
			break;
	}
}
/****************************************************************************
* 名    称：ClockWise()	
* 功    能：顺时针行驶
* 入口参数：无
* 出口参数：无
* 说    明：无
* 调用方法：无 
****************************************************************************/
void ClockWise(void)
{
	switch(gRobot.walk_t.circlechange.turntime)
	{
		//内圈
		  case 0:
				Line(-600,3400,0,0,1,1);
			break;
				
			case 1:
				Line(600,3400,-90,1,1,1);
			break;

			case 2:
				Line(600,1400,180,0,-1,1);
			break;

			case 3:
				Line(-600,1400,90,1,-1,1);
			break;
		
			case 4:
				ShunShiZhenCircleBiHuan(1800,1100,0,2400);
			break;
				
			case 5:
				ShunShiZhenCircleBiHuan(1800,1600,0,2400);
			break;
			
			case 6:
			  ShunShiZhenCircleBiHuan(1800,2100,0,2400);
			 break;	
		  default:
			break;
	}
}
/****************************************************************************
* 名    称：AntiClockWise ()	
* 功    能：逆时针行驶
* 入口参数：无
* 出口参数：无
* 说    明：无
* 调用方法：无 
****************************************************************************/
void AntiClockWise(void)
{
	switch(gRobot.walk_t.circlechange.turntime)
	{
		//内圈
		  case 0:
				Line(600,3400,0,0,1,1);
			break;
				
			case 1:
				Line(-600,3400,90,1,1,1);
			break;

			case 2:
				Line(-600,1400,180,0,-1,1);
			break;

			case 3:
				Line(600,1400,-90,1,-1,1);
			break;
		
			case 4:
				NiShiZhenCircleBiHuan(1800,1100,0,2400);
			break;
				
			case 5:
				NiShiZhenCircleBiHuan(1800,1600,0,2400);
			break;
			
			case 6:
			  NiShiZhenCircleBiHuan(1800,2100,0,2400);
			 break;	
		  default:
			break;
	}
}
/********************* (C) COPYRIGHT NEU_ACTION_2017 ****************END OF FILE************************/

