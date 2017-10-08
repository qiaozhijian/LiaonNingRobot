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
	if(gRobot.walk_t.pos.x>100 && gRobot.walk_t.pos.y<2400 && gRobot.walk_t.circleChange.quadrant!=1)
	{
			gRobot.walk_t.circleChange.linenum++;
			gRobot.walk_t.circleChange.quadrant=1;
	}
	if(gRobot.walk_t.pos.x>100 && gRobot.walk_t.pos.y>2400 && gRobot.walk_t.circleChange.quadrant!=2)
	{
			gRobot.walk_t.circleChange.linenum++;
			gRobot.walk_t.circleChange.quadrant=2;
	}
	if(gRobot.walk_t.pos.x<100 && gRobot.walk_t.pos.y>2400 && gRobot.walk_t.circleChange.quadrant!=3)
	{
		  gRobot.walk_t.circleChange.linenum++;
			gRobot.walk_t.circleChange.quadrant=3;
	}
	if(gRobot.walk_t.pos.x<100 && gRobot.walk_t.pos.y<2400 && gRobot.walk_t.circleChange.quadrant!=4)
	{
			gRobot.walk_t.circleChange.linenum++;
			gRobot.walk_t.circleChange.quadrant=4;
	}
	
	
	if(gRobot.walk_t.circleChange.linenum==4)
	{
		gRobot.walk_t.circleChange.linenum=0;
		if(gRobot.status&STATUS_SWEEP)
		{
			//记录扫场的圈数
		  gRobot.walk_t.circleChange.circleNum++;
		}else if(gRobot.status&STATUS_CAMERA_WALK)
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
}/****************************************************************************
* 名    称：int CornerJammed(void)
* 功    能：判断车的是否在角落卡死
* 入口参数：无
* 出口参数：无
* 说    明：无
* 调用方法：无 
****************************************************************************/
void CornerJammedJudge(void)
{
	static int step=0;
	static int stoptime=0;
	static int Lasttime=0;
	static float Lastx=0.0f;
	static float Lasty=0.0f;
	static float Lastangle=0.0f;
	static int jumptime=0;
 if(gRobot.status & (STATUS_SWEEP | STATUS_FIX | STATUS_PARKING | STATUS_CAMERA_AND_WALK))
{	
	switch(step)
	{
		case 0:
			Lastx=gRobot.walk_t.pos.x;
			Lasty=gRobot.walk_t.pos.y;
			Lastangle=gRobot.walk_t.pos.angle;
			step=1;
		 USART_OUT(UART5,"case0\r\n");
			break;
		case 1://判断卡死前的过程,为试图逃逸与投球做好准备
			Lasttime++;
			if(fabs(gRobot.walk_t.pos.x-Lastx)<10 && fabs(gRobot.walk_t.pos.y-Lasty)<10 && fabs(gRobot.walk_t.pos.angle-Lastangle)<3)
			{
				stoptime++;
			}else{
				stoptime=0;
			}
			if(Lasttime>50)
			{
				Lasttime=0;
				Lastx=gRobot.walk_t.pos.x;
				Lasty=gRobot.walk_t.pos.y;
				Lastangle=gRobot.walk_t.pos.angle;
			}
			
			if(stoptime>=500)
			{
				step=2;
				stoptime=0;
			}
			 USART_OUT(UART5,"case1\t");
			 USART_OUT(UART5,"%d\t",(int)Lastangle);
			 USART_OUT(UART5,"%d\t",(int)Lastx);
			 USART_OUT(UART5,"%d\t",(int)Lasty);
			 USART_OUT(UART5,"%d\t",(int)stoptime);
			 USART_OUT(UART5,"%d\r\n",(int)step);
			break; 
		case 2://开始射球
			USART_OUT(UART5,"case2\r\n");
			if(fabs(gRobot.walk_t.pos.x-Lastx)>10 && fabs(gRobot.walk_t.pos.y-Lasty)>10 && fabs(gRobot.walk_t.pos.angle-Lastangle)>3)
			{
				jumptime++;
				step=0;
			}else
			{
				jumptime=0;
			}
			if(jumptime>10)
			{ 
				step=0;
				gRobot.status|=STATUS_FIX;
			} 
				gRobot.status&=~STATUS_FIX;
		    fireTask();
			break;
	}
}
}


/********************* (C) COPYRIGHT NEU_ACTION_2017 ****************END OF FILE************************/
