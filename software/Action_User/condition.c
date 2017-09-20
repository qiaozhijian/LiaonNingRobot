/**
  ******************************************************************************
  * @file	   moveBase.c
  * @author	 Action
  * @version V1.0.0
  * @date	   2017/07/24
  * @brief	 2017省赛底盘运动控制部分
  ******************************************************************************
  * @attention
  *			None
  ******************************************************************************
  */
/* Includes -------------------------------------------------------------------------------------------*/
#include "config.h"
extern Robot_t gRobot;
/****************************************************************************
* 名    称：In2OutChange()	
* 功    能：扫场条件改变
* 入口参数：无
* 出口参数：无
* 说    明：无
* 调用方法：无 
****************************************************************************/
void In2OutChange(void)
{
	//启动避障条件
  if(gRobot.walk_t.right.real>2000)
	{
		gRobot.avoid_t.signal=1;
	}
	//启动避障逆向
	if(gRobot.avoid_t.passflag==1)
	{
		gRobot.avoid_t.passflag=0;
		if(gRobot.walk_t.circlechange.turntime>=7 && gRobot.walk_t.circlechange.turntime!=9)
		{
			if(gRobot.walk_t.circlechange.circlenum<1)
			{
				gRobot.walk_t.circlechange.turntime=gRobot.walk_t.circlechange.turntime-3;
			}else if(gRobot.walk_t.circlechange.circlenum>1)
			{
				gRobot.walk_t.circlechange.turntime=gRobot.walk_t.circlechange.turntime-2;
			}
		}else if(gRobot.walk_t.circlechange.turntime==9)
		{
		   //gRobot.status=
				FixTask();
	  }else if(gRobot.walk_t.circlechange.turntime==6)
		{
			 //gRobot.status=
			  FixTask();
		}else if(gRobot.walk_t.circlechange.turntime<7 && gRobot.walk_t.circlechange.turntime!=6)
		{
			if(gRobot.walk_t.circlechange.circlenum<1)
			{
				gRobot.walk_t.circlechange.turntime=gRobot.walk_t.circlechange.turntime+3;
			}else if(gRobot.walk_t.circlechange.circlenum>1)
			{
					gRobot.walk_t.circlechange.turntime=gRobot.walk_t.circlechange.turntime+4;
			}
		}		
	}
	//路径切换
	if(gRobot.walk_t.circlechange.turntime==4||gRobot.walk_t.circlechange.turntime==5||(gRobot.walk_t.circlechange.turntime==7||gRobot.walk_t.circlechange.turntime==8))
	{
		gRobot.walk_t.circlechange.turntime=gRobot.walk_t.circlechange.turntime+circlechange();
	}
	//进入矫正
	if(gRobot.walk_t.circlechange.turntime==6)
	{
		if(2000<gRobot.walk_t.pos.y && gRobot.walk_t.pos.y<2100 && gRobot.walk_t.pos.x>1900)
				{
					gRobot.status&=~STATUS_SWEEP;
					gRobot.walk_t.circlechange.turntime=0;
				}
	}
	if(gRobot.walk_t.circlechange.turntime==9)
	{
		if(2000<gRobot.walk_t.pos.y && gRobot.walk_t.pos.y<2100 && gRobot.walk_t.pos.x>1900)
				{
					gRobot.status&=~STATUS_SWEEP;
					gRobot.walk_t.circlechange.turntime=0;
				}
	}
}
/****************************************************************************
* 名    称：circlechange()	
* 功    能：记录当前的圈数以及是优弧还是劣弧
* 入口参数：当前的坐标
* 出口参数：无
* 说    明：无
* 调用方法：无 
****************************************************************************/
int circlechange(void)
{
	static uint8_t Quadrant=1;//象限
	if(gRobot.walk_t.pos.x>0&&gRobot.walk_t.pos.y<2400&&Quadrant==1)
	{
			gRobot.walk_t.circlechange.circlenum++;
			Quadrant=2;
	}
	if(gRobot.walk_t.pos.x>0&&gRobot.walk_t.pos.y>2400&&Quadrant==2)
	{
			gRobot.walk_t.circlechange.circlenum++;
			Quadrant=3;
	}
	if(gRobot.walk_t.pos.x<0&&gRobot.walk_t.pos.y>2400&&Quadrant==3)
	{
		  gRobot.walk_t.circlechange.circlenum++;
			Quadrant=4;
	}
	if(gRobot.walk_t.pos.x<0&&gRobot.walk_t.pos.y<2400&&Quadrant==4)
	{
			gRobot.walk_t.circlechange.circlenum++;
			Quadrant=1;
	}
	if(gRobot.walk_t.circlechange.circlenum>3)
	{
		gRobot.walk_t.circlechange.circlenum=0;
		return 1;
	}
	else
	{
	return 0;
	}
}
/****************************************************************************
* 名    称：LimitTurn()	
* 功    能：极限拐弯
* 入口参数：当前的坐标
* 出口参数：无
* 说    明：无
* 调用方法：无 
****************************************************************************/
int LimitTurn(float x,float y)
{
//内环
	if(x<1850&&y<=550)//设立极限拐弯区域,优先级先为最大
	{
		return 1;
	}else if(x>=1850&&y<4250)
	{
		return 1;
	}else if(x>-1850&&y>=4250)
	{
		return 1;
	}else if(x<=-1850&&y>550)
	{
		return 1;
	}
	return 0;
}
