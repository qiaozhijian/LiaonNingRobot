/**
  ******************************************************************************
  * @file	   condition.c
  * @author	 Action
  * @version V1.0.0
  * @date	   2017/07/24
  * @brief	 2017省赛条件控制部分
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
  if(gRobot.walk_t.right.real>6107 && gRobot.avoid_t.passflag==0)
	{
		gRobot.avoid_t.signal=1;
	}
	//启动避障
	if(gRobot.avoid_t.passflag==1)
	{
		
	}
}
/****************************************************************************
* 名    称：circleChange()	
* 功    能：记录当前的圈数以及是优弧还是劣弧
* 入口参数：当前的坐标
* 出口参数：无
* 说    明：无
* 调用方法：无 
****************************************************************************/
int circleChange(void)
{
	//Quadrant//象限
	if(gRobot.walk_t.pos.x>0 && gRobot.walk_t.pos.y<2400 && gRobot.walk_t.circleChange.quadrant!=1)
	{
			gRobot.walk_t.circleChange.linenum++;
			gRobot.walk_t.circleChange.quadrant=1;
	}
	if(gRobot.walk_t.pos.x>0 && gRobot.walk_t.pos.y>2400 && gRobot.walk_t.circleChange.quadrant!=2)
	{
			gRobot.walk_t.circleChange.linenum++;
			gRobot.walk_t.circleChange.quadrant=2;
	}
	if(gRobot.walk_t.pos.x<0 && gRobot.walk_t.pos.y>2400 && gRobot.walk_t.circleChange.quadrant!=3)
	{
		  gRobot.walk_t.circleChange.linenum++;
			gRobot.walk_t.circleChange.quadrant=3;
	}
	if(gRobot.walk_t.pos.x<0 && gRobot.walk_t.pos.y<2400 && gRobot.walk_t.circleChange.quadrant!=4)
	{
			gRobot.walk_t.circleChange.linenum++;
			gRobot.walk_t.circleChange.quadrant=4;
	}
	
	
	if(gRobot.walk_t.circleChange.linenum==4)
	{
		gRobot.walk_t.circleChange.linenum=0;
		if(gRobot.status==25)
		{
			//记录扫场的圈数
		  gRobot.walk_t.circleChange.circleNum++;
		}else if(gRobot.status==6)
		{
			//记录摄像头扫场
			gRobot.camera_t.camrBaseWalk_t.circleChange.circleNum++;
		}
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
/****************************************************************************
* 名    称：void LineCheck() 
* 功    能：判断车的侧边是那一边
* 入口参数：当前的坐标
* 出口参数：无
* 说    明：0:顺时针 1:逆时针
* 调用方法：无 
****************************************************************************/
int LineCheck(int position) 
{
	switch(position)
	{
		case 0:
			if(160<gRobot.walk_t.pos.angle || gRobot.walk_t.pos.angle<-160)
			{
				return 3;
			}else if(-110<gRobot.walk_t.pos.angle && gRobot.walk_t.pos.angle<-70)
			{
				return 2;
			}else if(-20<gRobot.walk_t.pos.angle && gRobot.walk_t.pos.angle<20)
			{
				return 1;
			}else if(70<gRobot.walk_t.pos.angle && gRobot.walk_t.pos.angle<110)
			{
			  return 4;
			}
			
			break;
			
		case 1:
			if(-110<gRobot.walk_t.pos.angle && gRobot.walk_t.pos.angle<-70)
			{
				return 4;
			}else if(-20<gRobot.walk_t.pos.angle && gRobot.walk_t.pos.angle<20)
			{
				return 1;
			
			}else if(70<gRobot.walk_t.pos.angle && gRobot.walk_t.pos.angle<110)
			{
				return 2;
			}else if(160<gRobot.walk_t.pos.angle || gRobot.walk_t.pos.angle<-160)
			{
			  return 3;
			}
			break;
			
		default:
			break;	
  }
	return 0;
}
/********************* (C) COPYRIGHT NEU_ACTION_2017 ****************END OF FILE************************/
