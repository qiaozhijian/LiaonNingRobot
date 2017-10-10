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
//		x0 = -162.5f;
//		y0 = 2335.35f;
		x0 = -150.0f;
		y0 = 2392.0f;
	}
else if (ballNum==1)                     //假如球是黑球
	{
		x0 = 150.0f;
		y0 = 2392.0f;
	}
//		USART_OUT(UART5,"%d\r\n",ballNum);
		//计算航向角转轴的坐标
		//x = x - 92.2f*sinf(angle);
		//y = y + 92.2f*cosf(angle);
		//计算发射装置的速度
		s = __sqrtf((x - x0)*(x - x0) + (y - y0)*(y - y0));
	  v = __sqrtf(12372.3578f * s * s / (s * 1.2349f - h));
//		launcher.speed=0.01364f*v-1.333f;
//		launcher.speed=0.01371f*v-3.413f;
	   launcher.speed=0.01517f*v-10.88f-4.0f;
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
	static int noballtime=0;                    //没球时间
	static int Stabletime=0;                    //转速稳定时间
	static int Stabletimelim=0;                 //黑白球交替
	static int balllast=0;
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
	//当行程开关触发时一直矫正
	if(TRAVEL_SWITCH_LEFT==1&&TRAVEL_SWITCH_RIGHT==1&&getLeftAdc()+getRightAdc()<4850&&getLeftAdc()+getRightAdc()>4700)
	{
		fixPosFirst(gRobot.fix_t.inBorder);
	}
	//实施速度
	ShootCtr(launcher.speed);
	//实施推球	
  if(ballColor!=0)
	{
		noBall=0;
		noBallCount=0;
		noballtime=0;
		if(balllast!=ballColor)
		{
			Stabletimelim=100;
			balllast=ballColor;
		}
		else{
			Stabletimelim=60;
		}
		if(fabs(gRobot.shoot_t.sReal.speed+launcher.speed)<2 && fabs(gRobot.shoot_t.sReal.angle-launcher.angle)<1)
		{
			Stabletime++;
			ShootCount();
			switch(step)
			{
				case 0://判断投球的初始位置
					if(gRobot.shoot_t.pReal.pos<2000)
					{
						if(Stabletime>Stabletimelim)
						{
							step=1;
							PushBallReset();
							Stabletime=0;
						}
					}else if(gRobot.shoot_t.pReal.pos>=2000)
					{
						if(Stabletime>Stabletimelim)
						{
							step=2;
							PushBall();
							Stabletime=0;
						}
					}
					break;
					
				case 1:
							PushBallReset();
				//判断为卡死
							if(fabs(gRobot.shoot_t.pReal.pos-PUSH_RESET_POSITION)<300)
							{
								step=2;
								gRobot.shoot_t.pReal.error=0;
							}else{
								gRobot.shoot_t.pReal.error++;
							}
				  break;
							
				case 2:
							PushBall();
				//判断为卡死
					    if(fabs(gRobot.shoot_t.pReal.pos-PUSH_POSITION)<300)
							{
								step=1;   
								gRobot.shoot_t.pReal.error=0;
							}else{
								gRobot.shoot_t.pReal.error++;
							}
					break;
			}
	  }
		//推子被卡死
	}else if(ballColor==0)  //没球快推
	{
			noBallCount++;
			noballtime++;
		if(noBallCount>=200)
		{ 
			noBall++;
			noBallCount=0;
		}
		//槽内没有球
		if(noballtime>150)
		{
			if(step==1||step==0)
			{
				PushBallReset();
				step=2;
			}else if(step==2)
			{
				PushBall();
				step=1;
			}
			noballtime=0;
		}
	}
	if(gRobot.shoot_t.pReal.error>500)
	{
		gRobot.shoot_t.pReal.error=0;
     noBall=5;
		GPIO_SetBits(GPIOE,GPIO_Pin_7);
	/*应急状态*/
	}
	if(noBall>4)
			{
				gRobot.status&=~STATUS_SHOOTER;
				gRobot.status|=STATUS_CAMERA_AND_WALK;
				gRobot.status|=STATUS_AVOID_JUDGE;
				gRobot.status&=~STATUS_FIX;
				noBall=0;
				step=0;
				noBallCount=0;
				gRobot.shoot_t.startSignal=0;
			}
	//脱离状态 
			//				gRobot.status&=~STATUS_AVOID_JUDGE;
//	USART_OUT(UART5,"%d\t",(int)noBall);
//	USART_OUT(UART5,"%d\t",(int)ballColor);
//	USART_OUT(UART5,"%d\t",(int)step);
//	USART_OUT(UART5,"%d\t",(int)noballtime);
//	USART_OUT(UART5,"%d\t",(int)ballColor);
			
////	USART_OUTF(gRobot.walk_t.pos.angle);
////	USART_OUTF(gRobot.walk_t.pos.x);
////	USART_OUTF(gRobot.walk_t.pos.y);
//////	USART_OUT(UART5,"%d\t",(int)gRobot.shoot_t.pReal.pos);
////	USART_OUTF(launcher.angle);
////	USART_OUTF(gRobot.shoot_t.sReal.angle);
////	USART_OUTF(launcher.speed);
////	USART_OUTF(gRobot.shoot_t.sReal.speed);
////	USART_OUT_CHAR("\r\n");
//	USART_OUT(UART5,(uint8_t *)"%d\t\r\n",(int)gRobot.walk_t.pos.y);

}


