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

float LauncherPidControl(float ERR)
{
	static float ERR_OLD=0;
	static float Kp=50;
//	static float Ki=0;
	static float Kd=10;
	static float OUTPUT=0;
	OUTPUT=Kp*ERR + Kd*(ERR-ERR_OLD);
	ERR_OLD=ERR;
	return OUTPUT;
}


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
	static float s = 0;                    //车到圆环中心的距离
	static float h = 424.6;                //发射口到框的高度，垂直高度
	static float v = 0;                    //要求的速度
	//static float rev = 0;//转动速度
	static float x0=-150, y0=2400;         //框的中心
	//static float g = 9.9;//重力加速度
	//static float angle = 0;//定义航向角度
	static float dx=0, dy=0;               //定义坐标差值
	static float alpha = 0;
	//算出发射装置的坐标
	ballNum=getBallColor();
if (ballNum == 100)                      //加入球是白球
	{
		x0 = -162.5;
		y0 = 2335.35;
	}

else if (ballNum==1)                     //假如球是黑球
	{
		x0 = 162.5;
		y0 = 2335.35;
	}
		//计算航向角转轴的坐标
		x = x - 97.2f*sinf(angle);
		y = y + 97.2f*cosf(angle);
		//计算发射装置的速度
		s = __sqrtf((x - x0)*(x - x0) + (y - y0)*(y - y0));
	  v = __sqrtf(12372.3578f * s * s / (s * 1.2349f - h));

		//v = 157.f / s / __sqrtf(1.234f*s - h);
		//v=1.59f*s*(__sqrtf(g*1000/(1.234f*s-h)));
		
	  //launcher.speed=0.01402f*v-5.457f+2.0f;
		launcher.speed=0.01587f*v-16.01f+4;
		//launcher.rev=(0.01434f*v-6.086f);
		//launcher.rev=launcher.rev+zhuan*zhuansu;
			
		//launcher.rev=v/(66*PI);
		dx = x0 - x;
		dy = y0 - y;
		alpha = atan2(dy, dx) * 180 / PI ;
		alpha = alpha - 90;
		if (alpha>180)alpha -= 360;
		else if (alpha<-180)alpha += 360;
		launcher.angle = angle - alpha;
		if (launcher.angle>180)launcher.angle -= 360;
		else if (launcher.angle<-180)launcher.angle += 360;
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
	static int YesBallCount=0;									//有球时推的时间
	gRobot.shoot_t.startSignal=1;//打开发射球标志位告诉检查射球时是否被撞到函数记住此时的坐标
	//CollectBallVelCtr(0);
	
	x=gRobot.walk_t.pos.x;											 //当前x坐标
	y=gRobot.walk_t.pos.y;											 //当前y坐标
	angle=gRobot.walk_t.pos.angle;							 //当前角度
	
	ballColor=getBallColor();
	
	yawCount++;
	yawCount%=4;
	
	launcher=Launcher(x,y,angle,ballColor);
	if(yawCount==3)
	{	
//		if(gRobot.shoot_t.sReal.Yawangle!=launcher.angle)
//		{
//			launcher.angle+=launcher.angle-gRobot.shoot_t.sReal.Yawangle;
//		}
		YawAngleCtr(launcher.angle);
	}
	//投球被打断
//	if(TRAVEL_SWITCH_LEFT!=1 && TRAVEL_SWITCH_RIGHT!=1)
//	{
//		FixTask();
//		YesBallCount=201;
//	}
//	else
//	{
	  ShootCtr(launcher.speed);
//	}

/**********************************测试版***************************/
    if(fabs(gRobot.shoot_t.pReal.pos-gRobot.shoot_t.pReal.posrem)<10)
		{
			//计算推球失败次数
			gRobot.shoot_t.pReal.turnTime++;
		}
		gRobot.shoot_t.pReal.posrem=gRobot.shoot_t.pReal.pos;
		if(YesBallCount<=3&&YesBallCount>0)
		{
			PushBall();
		}else if(YesBallCount<=203&&YesBallCount>200)
		{	
			PushBallReset();
		}
		YesBallCount++;
		YesBallCount%=400;
	
	if(ballColor==0)
	{
		noBallCount++;
		if(noBallCount>=200)
		{ 
			noBall++;
		}
		noBallCount%=200;
	}else if(ballColor!=0)
	{
		noBall=0;
		noBallCount=0;
	} 
	
	//脱离状态 
	if(noBall>5)
	{
		CollectBallVelCtr(60);
		Delay_ms(1000);
		gRobot.status=STATUS_CAMERA_AND_WALK;
		noBall=0;
		YesBallCount=0;
		noBallCount= 0;
		gRobot.shoot_t.startSignal=0;
	}
	
	if(gRobot.shoot_t.pReal.turnTime>30)
	{
		gRobot.shoot_t.pReal.turnTime=0;
	}
//	USART_OUT(UART5,(uint8_t *)"%d\t",noBall);
//	USART_OUT(UART5,(uint8_t *)"%d\t",(int)gRobot.walk_t.pos.x);
//	USART_OUT(UART5,(uint8_t *)"%d\t\r\n",(int)gRobot.walk_t.pos.y);

}
void LiuLeLiuLe(void)//此时应该躲避重新投球//放入异常判断处理
{
	 static int getAimWall=1;
	 static AimPos_t aimPos={0};																					//躲避的停车位
	 static int aimWall=0;
	 if(getAimWall)
	 {
		 switch(gRobot.abnormal)
		 {
			 case 8://证明车从左边来反向去下一面墙
				aimWall=gRobot.fix_t.inBorder-1;
			 if(aimWall<0)
			 {
				 aimWall=3;
			 }
			 break;
			 
			 
			 case 9://证明车从右边来
				 aimWall=gRobot.fix_t.inBorder+1;
				 if(aimWall>3)
				 {
						aimWall=0;//目标墙
				 }
			 break;
				 
			 case 10:
					aimWall=gRobot.fix_t.inBorder;
			 break;
			}
			aimPos=Go2NextWall(aimWall);
			getAimWall=0;
		}
		USART_OUT(UART5,"aimWall=%d\t\r\n",aimWall);
		if(Pointparking(aimPos.x,aimPos.y)==1)//停车完成
		{
			gRobot.status&=~STATUS_AVOID;//将异常处理关闭
			gRobot.status|=STATUS_FIX;
			getAimWall=1;
		}	 
}
