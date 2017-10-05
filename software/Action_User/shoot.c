/**
  ******************************************************************************
  * @file	  moveBase.c
  * @author	  Action
  * @version   V1.0.0
  * @date	  2017/07/24
  * @brief	 2017省赛掷球部分
  ******************************************************************************
  * @attention
  *			None
  ******************************************************************************/
#include "config.h"

extern Robot_t gRobot;



 /****************************************************************************
* 名    称：Launcher()
* 功    能：射球数据处理函数
* 入口参数：(x,y,angle):当前姿态，ballNum:当前球球的号码(黑1、白100)
* 出口参数：无
* 说    明：无
* 调用方法：无 
****************************************************************************/

ShootPara_t Launcher(float x,float y,float angle,int ballNum)
{
	static ShootPara_t launcher; 
	static float s = 0.0f;                    //车到圆环中心的距离
	static float h = 424.6f;                //发射口到框的高度，垂直高度
	static float v = 0.0f;                    //要求的速度
	//static float rev = 0;//转动速度
	static float x0=-150.0f, y0=2400.0f;         //框的中心
	//static float g = 9.9;//重力加速度
	//static float angle = 0;//定义航向角度
	static float dx=0.0f, dy=0.0f;               //定义坐标差值
	static float alpha = 0.0f;
	//算出发射装置的坐标
	ballNum=getBallColor();
if (ballNum == 100)                      //加入球是白球
	{
		x0 = -162.5f;
		y0 = 2335.35f;
	}
else if (ballNum==1)                     //假如球是黑球
	{
		x0 = 162.5f;
		y0 = 2335.35f;
	}
//		USART_OUT(UART5,"%d\r\n",ballNum);
		//计算航向角转轴的坐标
		x = x - 92.2f*sinf(angle);
		y = y + 92.2f*cosf(angle);
		//计算发射装置的速度
		s = __sqrtf((x - x0)*(x - x0) + (y - y0)*(y - y0));
	  v = __sqrtf(12372.3578f * s * s / (s * 1.2349f - h));
		launcher.speed=0.01364f*v-1.333f;
//	//彻底卡死
//	 if(gRobot.abnormal==)
//	 {
//		 //白球
//		 if (ballNum == 100)
//		 {
//				x0=0;
//				y0=2850;
//		 }
//		else if(ballNum==1)
//			{
//				x0=0;
//				y0=1950;
//			}
//		 s = __sqrtf((x - x0)*(x - x0) + (y - y0)*(y - y0));
//	   launcher.speed=0.01356f*s+31.01f;
//	 }
		dx = x0 - x;
		dy = y0 - y;
		alpha = atan2(dy, dx) * 180.0f/ PI ;
		alpha = alpha - 90.0f;
		if (alpha>180)alpha -= 360.0f;
		else if (alpha<-180)alpha += 360.0f;
		launcher.angle = angle - alpha;
		if (launcher.angle>180)launcher.angle -= 360.0f;
		else if (launcher.angle<-180)launcher.angle += 360.0f;
		return launcher;
}
 /****************************************************************************
* 名    称：fireTask()
* 功    能：掷球执行函数
* 入口参数：无
* 出口参数：无
* 说    明：无
* 调用方法：无 
****************************************************************************/
static int ballColor=1;
 void fireTask(void)
{
//	static int waitAdjust=0;									//定义发射电机以及航向角调整等待他们调整完之后进行推送球
	static float x=0,y=0,angle=0;
	static ShootPara_t launcher;
	static int yawCount=0;
	static int noBallCount=0;										//没球计时间
	static int noBall=0;
	static int step=0;
	static int Nostep=0;
	gRobot.shoot_t.startSignal=1;//打开发射球标志位告诉检查射球时是否被撞到函数记住此时的坐标
	x=gRobot.walk_t.pos.x;											 //当前x坐标
	y=gRobot.walk_t.pos.y;											 //当前y坐标
	angle=gRobot.walk_t.pos.angle;							 //当前角度
	//得到球的颜色
	ballColor=getBallColor();
	//计算角度速度
	launcher=Launcher(x,y,angle,ballColor);
	//实施角度
	yawCount++;
	yawCount%=4;
	if(yawCount==3)
	{	
		YawAngleCtr(launcher.angle);
	}
	//实施速度
	ShootCtr(launcher.speed);
	//实施推球	
  if(ballColor!=0)
	{
		noBall=0;
		noBallCount=0;
		if(fabs(gRobot.shoot_t.sReal.speed+launcher.speed)<2 && fabs(gRobot.shoot_t.sReal.angle-launcher.angle)<1)
		{
			switch(step)
			{
				case 0://判断投球的初始位置
					if(gRobot.shoot_t.pReal.pos<2000)
					{
						step=2;
						PushBallReset();
					}else if(gRobot.shoot_t.pReal.pos>=2000)
					{
						step=1;
						PushBall();
					}
					break;
					
				case 1:
					gRobot.shoot_t.pReal.error++;
							PushBallReset();
							if(fabs(gRobot.shoot_t.pReal.pos-PUSH_RESET_POSITION)<100)
							step=2;
				  break;
				case 2:
					gRobot.shoot_t.pReal.error++;
							PushBall();
					    if(fabs(gRobot.shoot_t.pReal.pos-PUSH_POSITION)<100)
							step=1;   
					break;
			}
	  }
		//推子被卡死
	}else if(ballColor==0)  //没球快推
	{
			noBallCount++;
		if(noBallCount>=200)
		{ 
			noBall++;
			noBallCount=0;
			if(noBall<4)
			{ 
				switch(Nostep)
				{
					case 0:
						PushBallReset();
					  if(fabs(gRobot.shoot_t.pReal.pos-PUSH_POSITION)<100)
					{
						Nostep=1;
					  step=2;
					}
						break;
					case 1:
						PushBall();
						if(fabs(gRobot.shoot_t.pReal.pos-PUSH_RESET_POSITION)<100)
						{
						Nostep=0;
						step=1;
						}
						break;
				}
			}
			}
		if(noBall>4)
			{
				gRobot.status&=~STATUS_SHOOTER;
				gRobot.status|=STATUS_CAMERA_AND_WALK;
				gRobot.status|=STATUS_AVOID_JUDGE;
				noBall=0;
				noBallCount=0;
				gRobot.shoot_t.startSignal=0;
			}
		}
	if(gRobot.shoot_t.pReal.error>300)
	{
		gRobot.shoot_t.pReal.error=0;
		GPIO_SetBits(GPIOE,GPIO_Pin_7);
	/*应急状态*/
	}
	//脱离状态 
	USART_OUT(UART5,"%d\t",(int)ballColor);
	USART_OUT(UART5,"%d\t",(int)step);
	USART_OUT(UART5,"%d\t",(int)ballColor);
	USART_OUT(UART5,"%d\t",(int)gRobot.walk_t.pos.angle);
	USART_OUT(UART5,"%d\t",(int)gRobot.walk_t.pos.x);
	USART_OUT(UART5,"%d\t",(int)gRobot.walk_t.pos.y);
	USART_OUT(UART5,"%d\t",(int)gRobot.shoot_t.pReal.pos);
	USART_OUT(UART5,"%d\t",(int)launcher.angle);
	USART_OUT(UART5,"%d\t",(int)gRobot.shoot_t.sReal.angle);
	USART_OUT(UART5,"%d\t",(int)launcher.speed);
	USART_OUT(UART5,"%d\r\n",(int)gRobot.shoot_t.sReal.speed);
//	USART_OUT(UART5,(uint8_t *)"%d\t\r\n",(int)gRobot.walk_t.pos.y);
}


