#include "config.h"

extern Robot_t gRobot;
Point_t boardPointIn[4]={  {X_LEFT,Y_UP},{X_LEFT,Y_DOWN },{X_RIGHT,Y_DOWN },{X_RIGHT,Y_UP}};//  //铁框点群
Point_t boardPointOut[4]={ {X_MIN,Y_MAX} , {X_MIN,Y_MIN} ,{X_MAX,Y_MIN},{X_MAX,Y_MAX}}; //外墙点群

//static int turnTimeRemember;												//记住在卡死的时候是什么直线的状态，等倒车case结束后让重新填装
static float xStick=0;
static float yStick=0;	//卡住时存储的位置数据
static int stickStatus=0;//判断现在卡死的状态
/****************************************************************************
* 名    称：void TransitionIn()
* 功    能：内环逃逸程序后退1.5s，外转45度
* 入口参数：angle//当前角度
* 出口参数：无
* 说    明：无
* 调用方法：无 
****************************************************************************/
void TransitionIn() 												//内环倒车程序
{
	static float LastX=0;
	static float LastY=0;
	static int flag=0;
	if(gRobot.walk_t.base==0)
	{
		LastX=gRobot.walk_t.pos.x;
		LastY=gRobot.walk_t.pos.y;
	}
	if(gRobot.avoid_t.passflag==1)
	{
		switch(gRobot.walk_t.circleChange.turnTime)
		{
			case 0:
				Line(1300.f,3400.f,0,0,1,1);
				break;
				
			case 1:
				Line(-600.f,4100.f,90,1,1,1);
				break;
				
			case 2:
				Line(-1300.f,1400,180,0,-1,1);
				break;
				
			case 3:
				Line(600.f,700,-90,1,-1,1);
				break;
		}
	}
	gRobot.avoid_t.passflag=1;
	if(Dis(LastX,LastY,gRobot.walk_t.pos.x,gRobot.walk_t.pos.y)>300 && flag==0)
	{
		flag=1;
	}
	if(flag==1)
	{
		gRobot.avoid_t.passflag=0;
		AngleRoute(gRobot.walk_t.pid.aimAngle);
	}
}
/****************************************************************************
* 名    称：void BackCarOut(float angle) 
* 功    能：外环逃逸程序后退1.5s，内转45度
* 入口参数：angle//当前角度
* 出口参数：无
* 说    明：无
* 调用方法：无 
****************************************************************************/
void TransitionOut(float angle) 											//外环倒车程序
{
  static float LastX=0;
	static float LastY=0;
	static int flag=0;
	if(gRobot.walk_t.base==0)
	{
		LastX=gRobot.walk_t.pos.x;
		LastY=gRobot.walk_t.pos.y;
	}
	if(gRobot.avoid_t.passflag==1)
	{
		switch(gRobot.walk_t.circleChange.turnTime)
		{
		case 6:
    Line(1400.f,3400.f,0,0,1,4);
    break;
    
    case 7:
    Line(-600.f,4100.f,90,1,1,4);
    break;
    
    case 8:
    Line(-1400.f,1400,180,0,-1,4);
    break;
    
    case 9:
    Line(600.f,700,-90,1,-1,4);
    break;
    
		}
	}
	gRobot.avoid_t.passflag=1;
	if(Dis(LastX,LastY,gRobot.walk_t.pos.x,gRobot.walk_t.pos.y)>300 && flag==0)
	{
		flag=1;
	}
	if(flag==1)
	{
		gRobot.avoid_t.passflag=0;
		AngleRoute(gRobot.walk_t.pid.aimAngle);
	}
}
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

	static float LastX=0;
	static float LastY=0;
	static int step=0;
	static float aimAngle = 0;   //目标角度
	static float angleError = 0; //目标角度与当前角度的偏差
	static int avoidtime=0;
	static int backtime=0;
   avoidtime++;
	switch(step)
	{
		case 0:
			avoidtime=0;
			LastX=gRobot.walk_t.pos.x;
			LastY=gRobot.walk_t.pos.y;
			if(gRobot.walk_t.circleChange.direction==1)
			{
				aimAngle = angle - 45;
			}else if(gRobot.walk_t.circleChange.direction==0)
			{
				aimAngle = angle + 45;
			}
			if (aimAngle > 180) 
			{
				aimAngle = aimAngle - 360;
			}
			else if (aimAngle < -180)
			{
				aimAngle = aimAngle + 360;
			}
		  step++;
			USART_OUT(UART5,"get%d\t",(int)aimAngle);
			USART_OUT(UART5,"get%d\t",(int)aimAngle);
			USART_OUT(UART5,"get%d\t",(int)aimAngle);
			break;
		case 1:
				avoidtime=0;
				backtime++;
			 VelCrl(CAN2, 1, -6000); //pid中填入的是差值
		   VelCrl(CAN2, 2,  6000);
		 if(fabs(angleError) < 5)
				{
					step=2;
					avoidtime=0;
				}
		 if(backtime>100)
			 {
				 backtime=0;
				 step++;
				 LastX=gRobot.walk_t.pos.x;
				 LastY=gRobot.walk_t.pos.y;
				 avoidtime=0;
			 }
			 USART_OUT(UART5,"get2%d\t",(int)aimAngle);
			 USART_OUT(UART5,"get2%d\t",(int)aimAngle);
			 USART_OUT(UART5,"get2%d\t",(int)aimAngle);
			break;
		case 2:
			 VelCrl(CAN2, 1, AnglePidControl(angleError)); //pid中填入的是差值
		   VelCrl(CAN2, 2, AnglePidControl(angleError));
		  if(fabs(angleError) < 5)
				{
					step++;
					avoidtime=0;
				}
				USART_OUT(UART5,"get3%d\t",(int)aimAngle);
				USART_OUT(UART5,"get3%d\t",(int)aimAngle);
				USART_OUT(UART5,"get3%d\t",(int)aimAngle);
			break;
		case 3:
			 VelCrl(CAN2, 1, 10000); //pid中填入的是差值
		   VelCrl(CAN2, 2,  -10000);
		   if(Dis(LastX,LastY,gRobot.walk_t.pos.x,gRobot.walk_t.pos.y)>300)
			 {
					gRobot.status&=~STATUS_AVOID_HANDLE;
//					gRobot.status|=STATUS_AVOID_JUDGE;
					step=0;
					avoidtime=0;
					gRobot.avoid_t.handleEnd=1;
			 }
			 USART_OUT(UART5,"get4\t");
			 USART_OUT(UART5,"get4\t");
			 USART_OUT(UART5,"get4\t");
			break;
	}
	//时间限制
	angleError = angleErrorCount(aimAngle,angle);
	USART_OUT(UART5,"ep=%d\t\r\n",step);
	if(avoidtime>250)
	{
	  gRobot.status&=~STATUS_AVOID_HANDLE;
//		gRobot.status|=STATUS_AVOID_JUDGE;
		step=0;
		avoidtime=0;
		gRobot.avoid_t.handleEnd=1;
	}
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

	static float LastX=0;
	static float LastY=0;
	static int step=0;
	static float aimAngle = 0;   //目标角度
	static float angleError = 0; //目标角度与当前角度的偏差
	static int avoidtime=0;
	static int backtime=0;
   avoidtime++;
	angleError = angleErrorCount(aimAngle,angle);
	switch(step)
	{
		case 0:
			LastX=gRobot.walk_t.pos.x;
			LastY=gRobot.walk_t.pos.y;
			if(gRobot.walk_t.circleChange.direction==1)
			{
				aimAngle = angle + 45;
			}else if(gRobot.walk_t.circleChange.direction==0)
			{
				aimAngle = angle - 45;
			}
			if (aimAngle > 180) 
			{
				aimAngle = aimAngle - 360;
			}
			else if (aimAngle < -180)
			{
				aimAngle = aimAngle + 360;
			}
		   step++;
			USART_OUT(UART5,"get%d\t",(int)aimAngle);
			USART_OUT(UART5,"get%d\t",(int)aimAngle);
			USART_OUT(UART5,"get%d\t",(int)aimAngle);
			break;
		case 1:
			backtime++;
			 VelCrl(CAN2, 1, -10000); //pid中填入的是差值
		   VelCrl(CAN2, 2,  10000);
		 if(fabs(angleError) < 5)
				{
					step=2;
					avoidtime=0;
				}
		 if(backtime>100)
			 {
					step++;
				 LastX=gRobot.walk_t.pos.x;
				 LastY=gRobot.walk_t.pos.y;
				 avoidtime=0;
				 backtime=0;
			 }
			 USART_OUT(UART5,"get2%d\t",(int)aimAngle);
			 USART_OUT(UART5,"get2%d\t",(int)aimAngle);
			 USART_OUT(UART5,"get2%d\t",(int)aimAngle);
			break;
		case 2:
			 VelCrl(CAN2, 1, AnglePidControl(angleError)); //pid中填入的是差值
		   VelCrl(CAN2, 2, AnglePidControl(angleError));
		  if(fabs(angleError) < 5)
				{
					step++;
					avoidtime=0;
				}
				USART_OUT(UART5,"get3%d\t",(int)aimAngle);
				USART_OUT(UART5,"get3%d\t",(int)aimAngle);
				USART_OUT(UART5,"get3%d\t",(int)aimAngle);
			break;
		case 3:
			 VelCrl(CAN2, 1, 10000); //pid中填入的是差值
		   VelCrl(CAN2, 2,  -10000);
		   if(Dis(LastX,LastY,gRobot.walk_t.pos.x,gRobot.walk_t.pos.y)>300)
			 {
					gRobot.status&=~STATUS_AVOID_HANDLE;
//					gRobot.status|=STATUS_AVOID_JUDGE;
				  step=0;
					avoidtime=0;
					gRobot.avoid_t.handleEnd=1;
			 }
			 USART_OUT(UART5,"get4\t");
			 USART_OUT(UART5,"get4\t");
			 USART_OUT(UART5,"get4\t");
			break;
	}
	//时间限制
	USART_OUT(UART5,"ep=%d\t\r\n",step);
	if(avoidtime>250)
	{
	  gRobot.status&=~STATUS_AVOID_HANDLE;
//		gRobot.status|=STATUS_AVOID_JUDGE;
		step=0;
		avoidtime=0;
		gRobot.avoid_t.handleEnd=1;
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
void BackCar(void)
{
	//顺时针
		if((xStick>-1400&&xStick<1400)&&(yStick>900&&yStick<3900))//内环
		{
			BackCarIn(gRobot.walk_t.pos.angle);
		}
		else if((xStick<-1400||xStick>1400)||(yStick<900||yStick>3900))//外环
		{
			BackCarOut(gRobot.walk_t.pos.angle);
		}
}	

/****************************************************************************
* 名    称：void CheckEnemy(void)	
* 功    能：靠墙时后面有车决绝方案
* 入口参数：无
* 出口参数：无
* 说    明：无
* 调用方法：无 
* 注    意: 车的方向顺:0 逆:1
****************************************************************************/
/*
用激光检测距离靠墙的距离与定位器的距离偏差是否很大，如果很大
不靠墙	
*/
int CheckEnemy(void)
{
#ifndef YES
#define NO 0
#define YES 1
#endif
  
  switch(LineCheck(gRobot.walk_t.circleChange.direction))
  {
  case 1://X=2400
    if(gRobot.walk_t.circleChange.direction==0)
    {
      if(fabs(2400-gRobot.walk_t.pos.x-getLeftAdc())>500)
      {
        return NO;
      }else{
        return YES;
      }
    }else if(gRobot.walk_t.circleChange.direction==1)
    {
      if(fabs(2400-gRobot.walk_t.pos.x-getRightAdc())>500)
      {
        return NO;
      }else
      {
        return YES;
      }
    }
    break;
  case 2://Y=4800
    if(gRobot.walk_t.circleChange.direction==0)
    {
      if(fabs(4800-gRobot.walk_t.pos.y-getLeftAdc())>500)
      {
        return NO;
      }else{
        return YES;
      }
    }else if(gRobot.walk_t.circleChange.direction==1)
    {
      if(fabs(4800-gRobot.walk_t.pos.y-getRightAdc())>500)
      {
        return NO;
      }else
      {
        return YES;
      }
    }
    break;
  case 3://X=-2400
    if(gRobot.walk_t.circleChange.direction==0)
    {
      if(fabs(2400+gRobot.walk_t.pos.x-getLeftAdc())>500)
      {
        return NO;
      }else{
        return YES;
      }
    }else if(gRobot.walk_t.circleChange.direction==1)
    {
      if(fabs(2400+gRobot.walk_t.pos.x-getRightAdc())>500)
      {
        return NO;
      }else
      {
        return YES;
      }
    }
    break;
  case 4://Y=0
    if(gRobot.walk_t.circleChange.direction==0)
    {
      if(fabs(gRobot.walk_t.pos.y-getLeftAdc())>500)
      {
        return NO;
      }else{
        return YES;
      }
    }else if(gRobot.walk_t.circleChange.direction==1)
    {
      if(fabs(gRobot.walk_t.pos.y-getRightAdc())>500)
      {
        return NO;
      }else
      {
        return YES;
      }
    }
    break;
  default:
    break;
  }
  return YES;
}
int JudgeStick(void);
void SweepJudge(void)
{
	//判断是否卡死
	if(JudgeStick()==1)
	{
		//判断
		gRobot.abnormal=CheckIntersect();              //判断卡死的区域;
		gRobot.status&=~STATUS_AVOID_JUDGE;
		gRobot.status|=STATUS_AVOID_HANDLE;
//		USART_OUT(UART5,"nnnnn%d\t\r\n");
//		USART_OUT(UART5,"gRobots%d\t\r\n",(int)gRobot.status);
	}
}
void PositionJudge(void)
{
	static int crashError=0;//受到冲击
	if(fabs(gRobot.shoot_t.shootPos.y-gRobot.walk_t.pos.y)>20||fabs(gRobot.shoot_t.shootPos.x-gRobot.walk_t.pos.x)>20)
	{
		crashError++;
	}
	else 
	{
		crashError=0;
	}
	
	if(crashError>3)
	{
		gRobot.abnormal=ABNOMAL_START_CRASHING;
		crashError=0;
		gRobot.status|=STATUS_AVOID_HANDLE;
		gRobot.status&=~STATUS_AVOID_JUDGE;
	}
	else 
	{
		gRobot.abnormal=0;
	}
}
int LaserJudge(void)
{
	static int count=0;//计算激光的变换时间来判断是否有车过来
  float laserLeft=getLeftAdc();
	float laserRight=getRightAdc();
//  static float lastlaserRight=0;//50ms前的右激光的值
//  static float lastlaserLeft=0;//50ms前左激光的值
  static int changeErrorRight=0;//右激光疯狂变换
	static int changeErrorLeft=0;//左激光疯狂变换

  count++;
	
	if(laserLeft<1350&&fabs(laserLeft-gRobot.shoot_t.lastLaser.left)>100)
    {
      changeErrorLeft++;
    }
    else
    {
      changeErrorLeft=0;
    }
		
	if(laserRight<1350&&fabs(laserRight-gRobot.shoot_t.lastLaser.right)>100)//右边有车来
	{
		changeErrorRight++;
		return 1;
	}else
	{
		changeErrorRight=0;
	}
	
	if(changeErrorRight>=2)
	{
		return 1;
	}
	if(changeErrorLeft>=2)//左边有车来
	{
		
		return 2;
	}
  
	USART_OUT(UART5,"Left=%d\t",(int)laserLeft);
	USART_OUT(UART5,"Right=%d\t",(int)laserRight);
	USART_OUT(UART5,"count=%d\t",(int)count);
	USART_OUT(UART5,"lastLeft=%d\t",(int)gRobot.shoot_t.lastLaser.left);
	USART_OUT(UART5,"lastRight%d\t",(int)gRobot.shoot_t.lastLaser.right);
	USART_OUT(UART5,"ErrorRight=%d\t",(int)changeErrorRight);
	USART_OUT(UART5,"ErrorLeft=%d\t\r\n",(int)changeErrorLeft);	
	
	return 0;
}
void ShootJudge(void)
{
//	PositionJudge();
	if(gRobot.abnormal!=10)
	{
		if(LaserJudge()==2)//发现左边有车来
		{
			gRobot.abnormal=8;
			gRobot.status|=STATUS_AVOID_HANDLE;
			gRobot.status&=~STATUS_AVOID_JUDGE;
		}else if(LaserJudge()==1)//发现右边有车来
		{
			gRobot.abnormal=9;
			gRobot.status|=STATUS_AVOID_HANDLE;
			gRobot.status&=~STATUS_AVOID_JUDGE;
		}
		USART_OUT(UART5,"LaserJudge=%d\t\r\n",(int)LaserJudge());	
		/*else if(LaserJudge()==0)
		{
			gRobot.abnormal=0;	//说明要嘛激光没发现要嘛真的没有车过来用射球坐标判断
			if(fabs(xShoot-gRobot.walk_t.pos.x)>20||fabs(yShoot-gRobot.walk_t.pos.y)>20)//受到冲击撞上发生位移
			{
				gRobot.abnormal=10;
				gRobot.status|=STATUS_AVOID_HANDLE;
				gRobot.status&=~STATUS_AVOID_JUDGE;
			}
		}*/
	}
}


void CWalkJudge(void)
{
	if(JudgeStick()==1)
	{
		//判断
		gRobot.abnormal=CheckIntersect();              //判断卡死的区域;
		gRobot.status&=~STATUS_AVOID_JUDGE;
		gRobot.status|=STATUS_AVOID_HANDLE;
	}
}
/****************************************************************************
* 名    称：void JudgeStick(void)
* 功    能：检测是否卡死
* 入口参数：无
* 出口参数：无
* 说    明：无
* 调用方法：无 
****************************************************************************/
//int JudgeStick(void)
//{
//	static int count=0;
//  static float velX=0;
//  static float velY=0;
//  static float aimVel=0.0f;
//  static float lastX=0;
//  static float lastY=0;
//  static int stickError=0;
//  count++;
//	if(count==5)
//	{
//    count=0;
//    velX=(gRobot.walk_t.pos.x-lastX)/5.0f*100.0f;
//    velY=(gRobot.walk_t.pos.y-lastY)/5.0f*100.0f;
//    //卡死速度比较值
//    aimVel=Pulse2Vel(gRobot.walk_t.base);
//		//目前实际速度
//    gRobot.walk_t.averageV=__sqrtf(velX*velX+velY*velY);
//    if(TwoNumCompare(gRobot.walk_t.averageV,aimVel*0.2f))
//    {
//      stickError++;
//    }
//    else
//    {
//      stickError=0;
//    }
//    if(stickError==5)
//    {
//      stickError=0;
//      //改变状态码
//			gRobot.status&=~STATUS_AVOID_JUDGE;
//			gRobot.status|=STATUS_AVOID_HANDLE;
//      gRobot.avoid_t.signal=0;             //清零
//			//记住卡死坐标
//			xStick=gRobot.walk_t.pos.x;
//			yStick=gRobot.walk_t.pos.y;
//			gRobot.avoid_t.continueTrigger++;
//		if(gRobot.avoid_t.continueTrigger>=5)//倘若这时候出现了扫场便把它储存起来
//		{
//			gRobot.status&=~STATUS_AVOID_HANDLE;
//			//对除了异常处理的情况进行状态储存
//			gRobot.avoid_t.statusRem|=gRobot.status;
//			//gRobot.st atus&=~STATUS_AVOID_JUDGE;
//			if(gRobot.status&STATUS_SWEEP)
//			{
//				gRobot.status&=~STATUS_SWEEP;//将sweep关闭
//			}  
////			if(gRobot.status&STATUS_PARKING)
////			{
////				gRobot.status&=~STATUS_PARKING;//将parking关闭
////			}  
//			gRobot.status|=STATUS_FIX;//连续触发直接关闭所有在fix前面状态handle，直接进入矫正
//			gRobot.avoid_t.continueTrigger=0;
//			gRobot.avoid_t.continueTriggerSignal=1;
//		}
//			return 1;
//		}
//    lastX=gRobot.walk_t.pos.x;
//    lastY=gRobot.walk_t.pos.y;
//		USART_OUT(UART5,"%d\t",(int)velX);
//		USART_OUT(UART5,"%d\t",(int)velY);
//		USART_OUT(UART5,"%d\t",(int)aimVel);
//		USART_OUT(UART5,"%d\t",(int)__sqrtf(velX*velX+velY*velY));
//		USART_OUT(UART5,"%d\t",(int)gRobot.walk_t.pos.x);
//		USART_OUT(UART5,"%d\t",(int)gRobot.walk_t.pos.y);	
//		USART_OUT(UART5,"%d\t",(int)stickError);	
//		USART_OUT(UART5,"%d\t",(int)gRobot.status);	
//		USART_OUT(UART5,"%d\r\n",(int)gRobot.walk_t.circleChange.turnTime);
//  }
//	return 0;
//}
int JudgeStick(void)
{
	static float xError = 0, yError = 0,angleError=0;
	static int stickError=0;
	xError = gRobot.walk_t.pos.x - getxRem();
	yError = gRobot.walk_t.pos.y - getyRem();
	angleError=gRobot.walk_t.pos.angle-getAngle();
	if (fabs(xError) < 3 && fabs(yError) < 3 && gRobot.walk_t.base!=0/*&&fabs(angleError)<3*/)
	{
		stickError++;
	}
	else
	{
		stickError = 0;
	}
	USART_OUT(UART5,"%d\t",(int)xError);
	USART_OUT(UART5,"%d\t",(int)getxRem());
	USART_OUT(UART5,"%d\t\r\n",(int)getyRem());
	if (stickError > 36)
	{
		stickError = 0;
		xStick=gRobot.walk_t.pos.x;
		yStick=gRobot.walk_t.pos.y;
		USART_OUT(UART5,"xStick=%d\t",(int)xStick);
		USART_OUT(UART5,"yStick=%d\t\r\n",(int)yStick);
      //改变状态码
		gRobot.status&=~STATUS_AVOID_JUDGE;
		gRobot.status|=STATUS_AVOID_HANDLE;
    gRobot.avoid_t.signal=0;             //清零
		gRobot.avoid_t.continueTrigger++;
		if(gRobot.avoid_t.continueTrigger>=5)//倘若这时候出现了扫场便把它储存起来
		{
			gRobot.status&=~STATUS_AVOID_HANDLE;
			//对除了异常处理的情况进行状态储存
			gRobot.avoid_t.statusRem|=gRobot.status;
			//gRobot.st atus&=~STATUS_AVOID_JUDGE;
			if(gRobot.status&STATUS_SWEEP)
			{
				gRobot.status&=~STATUS_SWEEP;//将sweep关闭
			}  
//			if(gRobot.status&STATUS_PARKING)
//			{
//				gRobot.status&=~STATUS_PARKING;//将parking关闭
//			}  
			gRobot.status|=STATUS_FIX;//连续触发直接关闭所有在fix前面状态handle，直接进入矫正
			gRobot.avoid_t.continueTrigger=0;
			gRobot.avoid_t.continueTriggerSignal=1;
		}
		return 1;
	}
	return 0;
}
/****************************************************************************
* 名    称：CheckIntersect(float x, float y, float angle, Point_t cP[4])
* 功    能：检查撞到的状态
* 入口参数：无
* 出口参数：无
* 说    明：无
* 调用方法：无 
****************************************************************************/
int CheckIntersect()//检查是否与边相交
{
	float x=gRobot.walk_t.pos.x,y=gRobot.walk_t.pos.y,angle=gRobot.walk_t.pos.angle;
  Point_t cP[4]={0};//车的四个顶点
  int StickOut[4] = {0};
  int StickIn[4] = {0};
  int intersect = 0;
  int stickCornor = 0;//卡角标志位
  //外框木板点群

  CarPointTrans(x,y,angle,cP);//得到车的4个顶点
  //////循环检查每一边
  for (int m = 0, n = 3; m <= 3; n = m++)
  {
    for (int i = 0, j = 3; i <= 3; j = i++)
    {
      if (CheckStraddle(cP[m], cP[n], boardPointOut[i], boardPointOut[j]))
      {
        StickOut[i] = 1;
      }
    }
  }
  
  for (int m = 0, n = 3; m <= 3; n = m++)
  {
    for (int i = 0, j = 3; i <= 3; j = i++)
    {
      if (CheckStraddle(cP[m], cP[n], boardPointIn[i], boardPointIn[j]))
      {
        StickIn[i] = 1;
      }
    }
  }
  /////////看看是在哪里卡住的
  if (StickOut[0] == 1 || StickOut[1] == 1 || StickOut[2] == 1 || StickOut[3] == 1)
  {
    intersect = 2;
  }
  else
  {
    intersect = 0;
  }
  
  if (intersect != 2)
  {
    if (StickIn[0] == 1 || StickIn[1] == 1 || StickIn[2] == 1 || StickIn[3] == 1)
    {
      intersect = 1;
    }
    else
    {
      intersect = 0;
    }
  }
  
  
  ///////////接下来判断是否卡在角
  if (intersect == 2)//判断是否卡在墙角
  {
			if ((StickOut[0] + StickOut[1] + StickOut[2] + StickOut[3]) == 2)
			{
				if (StickOut[0] + StickOut[1] == 2 || StickOut[1] + StickOut[2] == 2 || StickOut[2] + StickOut[3] == 2 || StickOut[3] + StickOut[0] == 2)//卡在墙角
				{
					stickCornor = 2;
				}
				else
				{
					stickCornor = 0;
				}
			}
  }
  else if (intersect == 1)//判断是否卡在铁框角
  {
    if ((StickIn[0] + StickIn[1] + StickIn[2] + StickIn[3]) == 2)
    {
      if (StickIn[0] + StickIn[1] == 2 || StickIn[1] + StickIn[2] == 2 || StickIn[2] + StickIn[3] == 2 || StickIn[3] + StickIn[0] == 2)//卡在墙角
      {
        stickCornor = 1;
      }
      else
      {
        stickCornor = 0;
      }
    }
  }
  else
  {
    stickCornor = 0;
  }
  
  if (intersect == 2)
  {
    stickStatus = ABNOMAL_BLOCK_OUT;//卡在外框边
  }
  else if (intersect == 1)
  {
    stickStatus = ABNOMAL_BLOCK_IN;//卡在内框边
  }
  else if (intersect == 0)
  {
    stickStatus = ABNOMAL_BLOCK_MIDDLE;//和车碰撞
  }		
	
  if(stickCornor==0)
  {
    stickStatus=stickStatus;
  }
  else if(stickCornor==1)
  {
    stickStatus=ABNOMAL_BLOCK_IN_CORNER;//卡在内圈角落
  }else if(stickCornor==2)
  {
    stickStatus=ABNOMAL_BLOCK_OUT_CORNER;//卡在外圈角落
  }
  USART_OUT(UART5,"stickCornor=%d\t",(int)stickCornor);
  USART_OUT(UART5,"intersect=%d\t",(int)intersect);
  USART_OUT(UART5,"stickStatus=%d\t",(int)stickStatus);
  //USART_OUT(UART5,"%d %d %d %d %d %d %d %d\t\r\n",(int)StickIn[0], StickIn[1], StickIn[2], StickIn[3], StickOut[0], StickOut[1], StickOut[2], StickOut[3]);	
  return stickStatus;
}
/****************************************************************************
* 名    称：CheckStraddle(Point_t c1, Point_t c2, Point_t b1, Point_t b2)
* 功    能：检查车的点是否出去
* 入口参数：无
* 出口参数：无
* 说    明：无
* 调用方法：无 
****************************************************************************/
int CheckStraddle(Point_t c1, Point_t c2, Point_t b1, Point_t b2)//分别传入车的两边的参数和边界点的参数
{
  //两个向量相乘的值
  float temp1 = ((b2.x - c2.x)*(c1.y - c2.y) - (c1.x - c2.x)*(b2.y - c2.y))*((b1.x - c2.x)*(c1.y - c2.y) - (c1.x - c2.x)*(b1.y - c2.y));
  //两个向量相乘的值
  float temp2 = ((c1.x - b2.x)*(b1.y - b2.y) - (b1.x - b2.x)*(c1.y - b2.y))*((c2.x - b2.x)*(b1.y - b2.y) - (b1.x - b2.x)*(c2.y - b2.y));
  if (Max(c1.x, c2.x) >= Min(b1.x, b2.x) && Max(b1.x, b2.x) >= Min(c1.x, c2.x) && Max(c1.y, c2.y) >= Min(b1.y, b2.y) && Max(b1.y, b2.y) >= Min(c1.y, c2.y))//快速排斥实验
  {
    if (temp1 <= 0 && temp2 <= 0)
    {
      return 1;
    }
  }
  return 0;
}
/****************************************************************************
* 名    称：CarPointTrans(float x, float y, float angle, Point_t cP[4])
* 功    能：转换车的四个角坐标
* 入口参数：无
* 出口参数：无
* 说    明：无
* 调用方法：无 
****************************************************************************/
void CarPointTrans(float x, float y, float angle, Point_t cP[4])//将定位系统转换为车的四个角落
{
  static float carLength = 500.f;
  static float wheelLength = 500.f;
  float xieBian = sqrt(wheelLength*wheelLength / 4 + carLength* carLength);
  float  angle1 = angle / 180 * PI;
  float  angle2 = angle + 90;
  float  angle3 = angle + 90;
  if (angle2 >= 180)
  {
    angle2 = angle2 - 360;
  }
  else if (angle2 <= -180)
  {
    angle2 = angle2 + 360;
  }
  
  if (angle3 >= 180)
  {
    angle3 = angle3 - 360;
  }
  else if (angle3 <= -180)
  {
    angle3 = angle3 + 360;
  }
  angle2 = angle2 / 180 * PI - atanf(wheelLength / 2 / carLength);
  angle3 = angle3 / 180 * PI + atanf(wheelLength / 2 / carLength);
  
  cP[1].x = x - wheelLength / 2 * cosf(angle1);
  cP[1].y = y - wheelLength / 2 * sinf(angle1);
  
  cP[2].x = x + wheelLength / 2 * cosf(angle1);
  cP[2].y = y + wheelLength / 2 * sinf(angle1);
  
  
  cP[0].x = x + xieBian*cosf(angle3);
  cP[0].y = y + xieBian*sinf(angle3);
  
  cP[3].x = x + xieBian*cosf(angle2);
  cP[3].y = y + xieBian*sinf(angle2);
  //USART_OUT(UART5,"angle2=%d\t angle3=%d\r\n",(int)angle2 / PI * 180,angle3 / PI * 180);
  // ("luX=%f luY=%f ldX=%f ldY=%f rdX=%f rdY=%f ruX=%f ruY=%f\n", cP[0].x, cP[0].y, cP[1].x, cP[1].y, cP[2].x, cP[2].y, cP[3].x, cP[3].y);
}

/****************************************************************************
* 名    称：SoundOut()
* 功    能：
* 入口参数：无
* 出口参数：无
* 说    明：无
* 调用方法：无 
****************************************************************************/
void SoundOut(void)//试探对方车是否能动
{
  static float forwardY=0;//y轴上移动的距离
  static int soundOutTime=0;//定义试探时间
  VelCrl(CAN2, 1, 20000);
  VelCrl(CAN2, 2, 20000);
  soundOutTime++;
  forwardY=gRobot.walk_t.pos.y-yStick;
  if(soundOutTime>300)//让其向前加速撞3秒
  {
    soundOutTime=0;
    if(forwardY<0)
    {
      //跳至反向跑
    }
  }
}
/****************************************************************************
* 名    称：void TransitionIn()
* 功    能：内环逃逸程序后退1.5s，外转45度
* 入口参数：
* 出口参数：无
* 说    明：无
* 调用方法：无 
****************************************************************************/
void AntiTransition(void) 												//内环倒车程序
{
	static float LastX=0;
	static float LastY=0;
	static int flag=0;
	static int Lasttime=0;
	//程序进入一次
	if(gRobot.avoid_t.passflag==0)
	{
		LastX=gRobot.walk_t.pos.x;
		LastY=gRobot.walk_t.pos.y;
		Lasttime=gRobot.walk_t.circleChange.turnTime;
	}else if(gRobot.avoid_t.passflag==1)
	{
		switch(gRobot.walk_t.circleChange.turnTime)
		{
			case 0:
				Line(1300.f,3400.f,0,0,1,1);
				break;
				
			case 1:
				Line(-600.f,4100.f,90,1,1,1);
				break;
				
			case 2:
				Line(-1300.f,1400,180,0,-1,1);
				break;
				
			case 3:
				Line(600.f,700,-90,1,-1,1);
				break;
		}
	}
	gRobot.avoid_t.passflag=1;
	if(Dis(LastX,LastY,gRobot.walk_t.pos.x,gRobot.walk_t.pos.y)>300 && flag==0)
	{
		flag=1;
	}
	if(flag==1)
	{
		gRobot.avoid_t.passflag=0;
		AngleRoute(gRobot.walk_t.pid.aimAngle);
	}
	
	
	if(gRobot.walk_t.circleChange.circleNum==0)   //内圈
{
	if(gRobot.walk_t.pos.x>(gRobot.walk_t.board)[0][0] && gRobot.walk_t.pos.y<(gRobot.walk_t.board)[0][1]){
		gRobot.walk_t.circleChange.turnTime=0;
	}else if(gRobot.walk_t.pos.x>(gRobot.walk_t.board)[0][2] && gRobot.walk_t.pos.y>(gRobot.walk_t.board)[0][1]){
		gRobot.walk_t.circleChange.turnTime=1;
	}else if(gRobot.walk_t.pos.x<(gRobot.walk_t.board)[0][2] && gRobot.walk_t.pos.y>(gRobot.walk_t.board)[0][3]){
		gRobot.walk_t.circleChange.turnTime=2;
	}else if(gRobot.walk_t.pos.x<(gRobot.walk_t.board)[0][0] && gRobot.walk_t.pos.y<(gRobot.walk_t.board)[0][3]){
		gRobot.walk_t.circleChange.turnTime=3;
	}
}else if(gRobot.walk_t.circleChange.circleNum!=0) //外圈
	{
			if(gRobot.walk_t.pos.x>(gRobot.walk_t.board)[1][0]&&gRobot.walk_t.pos.y<(gRobot.walk_t.board)[1][1]){
		gRobot.walk_t.circleChange.turnTime=6;
	}else if(gRobot.walk_t.pos.x>(gRobot.walk_t.board)[1][2]&&gRobot.walk_t.pos.y>(gRobot.walk_t.board)[1][1]){
		gRobot.walk_t.circleChange.turnTime=7;
	}else if(gRobot.walk_t.pos.x<(gRobot.walk_t.board)[1][2]&&gRobot.walk_t.pos.y>(gRobot.walk_t.board)[1][3]){
		gRobot.walk_t.circleChange.turnTime=8;
	}else if(gRobot.walk_t.pos.x<(gRobot.walk_t.board)[1][0]&&gRobot.walk_t.pos.y<(gRobot.walk_t.board)[1][3]){
		gRobot.walk_t.circleChange.turnTime=9;
	}
	}
	if(Lasttime!=gRobot.walk_t.circleChange.turnTime)
	{
		//结束避障
		//还原数据
		gRobot.status|=STATUS_AVOID_JUDGE;
		gRobot.status&=~STATUS_AVOID_HANDLE;
		flag=0;
		gRobot.avoid_t.passflag=0;
	}
		USART_OUT(UART5,"A%d\t",(int)flag);
		USART_OUT(UART5,"%d\t",(int)Dis(LastX,LastY,gRobot.walk_t.pos.x,gRobot.walk_t.pos.y));
		USART_OUT(UART5,"%d\t",(int)LastX);
		USART_OUT(UART5,"%d\r\n",(int)LastY);
}
/****************************************************************************
* 名    称：Transition(float angle) 
* 功    能：顺时针避障
* 入口参数：angle//当前角度
* 出口参数：无
* 说    明：无
* 调用方法：无 
****************************************************************************/
void Transition(void) 											//外环倒车程序
{
  static float LastX=0;
	static float LastY=0;
	static int flag=0;
	static int Lasttime=0;
	if(gRobot.avoid_t.passflag==0)
	{
		LastX=gRobot.walk_t.pos.x;
		LastY=gRobot.walk_t.pos.y;
		Lasttime=gRobot.walk_t.circleChange.turnTime;
	}else if(gRobot.avoid_t.passflag==1)
	{
		switch(gRobot.walk_t.circleChange.turnTime)
		{
			case 0:
				Line(-600.f,3400.f,0,0,1,1);
				break;
				
			case 1:
				Line(600.f,3400.f,-90,1,1,1);
				break;
				
			case 2:
				Line(600.f,1400,180,0,-1,1);
				break;
				
			case 3: 
				Line(-600.f,1400,90,1,-1,1);
				break;
		}
	}
	gRobot.avoid_t.passflag=1;
	if(Dis(LastX,LastY,gRobot.walk_t.pos.x,gRobot.walk_t.pos.y)>300 && flag==0)
	{
		flag=1;
	}
	if(flag==1)
	{
		gRobot.avoid_t.passflag=0;
		AngleRoute(gRobot.walk_t.pid.aimAngle);
	}
	if(gRobot.walk_t.circleChange.circleNum==0)   //内圈
{
		if(gRobot.walk_t.pos.x<(gRobot.walk_t.board)[1][0] && gRobot.walk_t.pos.y<(gRobot.walk_t.board)[1][1]){
		gRobot.walk_t.circleChange.turnTime=0;
	}else if(gRobot.walk_t.pos.x<(gRobot.walk_t.board)[1][2] && gRobot.walk_t.pos.y>(gRobot.walk_t.board)[1][1]){
		gRobot.walk_t.circleChange.turnTime=1;
	}else if(gRobot.walk_t.pos.x>(gRobot.walk_t.board)[1][2] && gRobot.walk_t.pos.y>(gRobot.walk_t.board)[1][3]){
		gRobot.walk_t.circleChange.turnTime=2;
	}else if(gRobot.walk_t.pos.x>(gRobot.walk_t.board)[1][0] && gRobot.walk_t.pos.y<(gRobot.walk_t.board)[1][3]){
		gRobot.walk_t.circleChange.turnTime=3;
}else if(gRobot.walk_t.circleChange.circleNum!=0) //外圈
	{
			if(gRobot.walk_t.pos.x<(gRobot.walk_t.board)[1][0] && gRobot.walk_t.pos.y<(gRobot.walk_t.board)[1][1]){
		gRobot.walk_t.circleChange.turnTime=6;
	}else if(gRobot.walk_t.pos.x<(gRobot.walk_t.board)[1][2] && gRobot.walk_t.pos.y>(gRobot.walk_t.board)[1][1]){
		gRobot.walk_t.circleChange.turnTime=7;
	}else if(gRobot.walk_t.pos.x>(gRobot.walk_t.board)[1][2] && gRobot.walk_t.pos.y>(gRobot.walk_t.board)[1][3]){
		gRobot.walk_t.circleChange.turnTime=8;
	}else if(gRobot.walk_t.pos.x>(gRobot.walk_t.board)[1][0] && gRobot.walk_t.pos.y<(gRobot.walk_t.board)[1][3]){
		gRobot.walk_t.circleChange.turnTime=9;
	}
	}
	if(Lasttime!=gRobot.walk_t.circleChange.turnTime)
	{
		//结束避障
		//还原数据
		gRobot.status|=STATUS_AVOID_JUDGE;
		gRobot.status&=~STATUS_AVOID_HANDLE;
		flag=0;
		gRobot.avoid_t.passflag=0;
	}
	
}
}
 void CornerIn(void) //内环倒车程序
{
	static float aimAngle = 0;   //目标角度
	static float angleError = 0; //目标角度与当前角度的偏差
	static int i = 0;																  //目标角度变换标志位
	static int j = 0; 																//在此设立标志位在信号量10ms进入一次，达到延时的效果
	static int avoidtime=0;
	if (i == 0)																		    //使目标角度偏向右边45
	{
		if(gRobot.walk_t.circleChange.direction==0)
		{
			aimAngle = gRobot.walk_t.pos.angle + 45; //让车头目标角度右偏45度
		}else if(gRobot.walk_t.circleChange.direction==1)
		{
			aimAngle = gRobot.walk_t.pos.angle - 45;
		}
		i = 1;
	}
	angleError = angleErrorCount(aimAngle,gRobot.walk_t.pos.angle);
	j++;
	if (j < 100)
	{
		VelCrl(CAN2, 1, -10000); //pid中填入的是差值
		VelCrl(CAN2, 2,  10000);
	}else if (j >=100)
	{
		VelCrl(CAN2, 1, AnglePidControl(angleError)); //pid中填入的是差值
		VelCrl(CAN2, 2, AnglePidControl(angleError));
	}
	
	if (fabs(angleError) < 5)
		{
			i = 0;
			j = 0;//清空标志位
//			gRobot.status|=STATUS_AVOID_JUDGE;
			gRobot.avoid_t.handleEnd=1;
			gRobot.status&=~STATUS_AVOID_HANDLE;
			avoidtime=0;
		} 
		
	if(avoidtime>250)
	{
		gRobot.status&=~STATUS_AVOID_HANDLE;
//		gRobot.status|=STATUS_AVOID_JUDGE;
		avoidtime=0;
		gRobot.avoid_t.handleEnd=1;
	}
}
void CornerOut(void) //内环倒车程序
{
	static float aimAngle = 0;   //目标角度
	static float angleError = 0; //目标角度与当前角度的偏差
	static int i = 0;																  //目标角度变换标志位
	static int j = 0; 																//在此设立标志位在信号量10ms进入一次，达到延时的效果
	static int avoidtime=0;
	if (i == 0)																		    //使目标角度偏向右边45
	{
		if(gRobot.walk_t.circleChange.direction==0)
		{
			aimAngle = gRobot.walk_t.pos.angle - 45; //让车头目标角度右偏45度
		}else if(gRobot.walk_t.circleChange.direction==1)
		{
			aimAngle = gRobot.walk_t.pos.angle + 45;
		}
		i = 1;
	}
	angleError = angleErrorCount(aimAngle,gRobot.walk_t.pos.angle);
	j++;
	if (j < 150)
	{
		VelCrl(CAN2, 1, -10000); //pid中填入的是差值
		VelCrl(CAN2, 2,  10000);
	}else if (j >=150)
	{
		VelCrl(CAN2, 1, AnglePidControl(angleError)); //pid中填入的是差值
		VelCrl(CAN2, 2, AnglePidControl(angleError));
	}
	
	if (fabs(angleError) < 5)
		{
			i = 0;
			j = 0;//清空标志位
//			gRobot.status|=STATUS_AVOID_JUDGE;
			gRobot.status&=~STATUS_AVOID_HANDLE;
			avoidtime=0;
			gRobot.avoid_t.handleEnd=1;
		} 
	if(avoidtime>250)
	{
		gRobot.status&=~STATUS_AVOID_HANDLE;
//		gRobot.status|=STATUS_AVOID_JUDGE;
		avoidtime=0;
		gRobot.avoid_t.handleEnd=1;
	}
}
void WalkHandle(void)
{
	switch(gRobot.abnormal)
	{
		case ABNOMAL_BLOCK_IN:
			//SquareTransition();
			CornerIn();
		break;
		
		case ABNOMAL_BLOCK_OUT :
			//Square2Transition();
		   	CornerOut();
		break;
		
		case ABNOMAL_BLOCK_MIDDLE :
			CircleTransition();
		 break;
		
		case ABNOMAL_BLOCK_IN_CORNER:
			CornerIn();
		break;
		
		case ABNOMAL_BLOCK_OUT_CORNER:
				CornerOut();
		break;
		
		default:
			USART_OUT(UART5,"SweepHandleErr");
		break;
	}
}
/****************************************************************************
* 名    称：SweepHandle
* 功    能：走形避障
* 入口参数：无
* 出口参数：无
* 说    明：无
* 调用方法：无 
****************************************************************************/
void SweepHandle(void)
{
	switch(gRobot.abnormal)
	{
		case ABNOMAL_BLOCK_IN:
			//SquareTransition();
			CornerIn();
		break;
		
		case ABNOMAL_BLOCK_OUT :
			//Square2Transition();
		  //LineBack();
			CornerOut();
		break;
		
		case ABNOMAL_BLOCK_MIDDLE :
			CircleTransition();
		 break;
		
		case ABNOMAL_BLOCK_IN_CORNER:
			CornerIn();
		break;
		
		case ABNOMAL_BLOCK_OUT_CORNER:
			//LineBack();
		CornerOut();
		break;
		
		default:
			USART_OUT(UART5,"SweepHandleErr");
		break;
	}
		
}
/****************************************************************************
* 名    称：Transition(float angle) 
* 功    能：顺时针避障
* 入口参数：angle//当前角度
* 出口参数：无
* 说    明：无
* 调用方法：无 
****************************************************************************/
void SquareTransition(void)
{
	if(gRobot.walk_t.circleChange.direction==0)     //顺时针
	{
		BackCar();
		//Transition();
	}else if(gRobot.walk_t.circleChange.direction==1) //逆时针
	{
		BackCar();
		//AntiTransition();
	}
	
}
/****************************************************************************
* 名    称：CircleTransition
* 功    能：圆避障过渡
* 入口参数：angle//当前角度
* 出口参数：无
* 说    明：无
* 调用方法：无 
****************************************************************************/
void CircleTransition(void)
{
	BackCar();
}
/****************************************************************************
* 名    称：Square2Transition
* 功    能：外圈正方形避障
* 入口参数：angle//当前角度
* 出口参数：无
* 说    明：无
* 调用方法：无 
****************************************************************************/
void Square2Transition(void)
{
	if(gRobot.walk_t.circleChange.direction==0)     //顺时针
	{
		BackCar();
		//Transition();
	}else if(gRobot.walk_t.circleChange.direction==1) //逆时针
	{
		BackCar();
		//AntiTransition();
	}
}
void ShootHandle(void)//此时应该躲避重新投球//放入异常判断处理
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
			 gRobot.avoid_t.direction=1;
			 break;
			 
			 case 9://证明车从右边来
				 aimWall=gRobot.fix_t.inBorder+1;
				 if(aimWall>3)
				 {
						aimWall=0;//目标墙
				 }
				 gRobot.avoid_t.direction=0;
			 break;
				 
			 case 10:
					aimWall=gRobot.fix_t.inBorder+1;
					if(aimWall>3)
					{
						aimWall=0;//目标墙
					}
			 break;
			}
		 gRobot.fix_t.wayChoose=1;
			aimPos=Go2NextWall(aimWall);
			getAimWall=0;
		}
		USART_OUT(UART5,"aimWall=%d\t\r\n",aimWall);
		gRobot.ParkingPoint.x=aimPos.x;
		gRobot.ParkingPoint.y=aimPos.y;
		gRobot.fix_t.toBorder=aimWall;
		getAimWall=1;
		gRobot.status|=STATUS_PARKING;
//		gRobot.status|=STATUS_FIX;
		gRobot.status|=STATUS_AVOID_JUDGE;
		gRobot.status&=~STATUS_AVOID_HANDLE;
		gRobot.avoid_t.handleEnd=1;
}
 /****************************************************************************
* 名    称：void BackCarOut(float angle) 
* 功    能：外环逃逸程序后退1.5s，内转45度
* 入口参数：angle//当前角度
* 出口参数：无
* 说    明：无
* 调用方法：无 
****************************************************************************/
void LineBack(void)//边界倒车程序
{
	static float angleError = 0; //目标角度与当前角度的偏差
	static int j = 0; 																//在此设立标志位在信号量10ms进入一次，达到延时的效果
	static int avoidtime=0;
	angleError = angleErrorCount(gRobot.walk_t.pid.aimAngle,gRobot.walk_t.pos.angle);
	j++;
	if (j < 150)
	{
		VelCrl(CAN2, 1, -10000); //pid中填入的是差值
		VelCrl(CAN2, 2,  10000);
	}else if (j >=150)
	{
		VelCrl(CAN2, 1, AnglePidControl(angleError)); //pid中填入的是差值
		VelCrl(CAN2, 2, AnglePidControl(angleError));
		if (fabs(angleError) < 5)
		{
			j = 0;//清空标志位
			gRobot.status|=STATUS_AVOID_JUDGE;
			gRobot.status&=~STATUS_AVOID_HANDLE;
			avoidtime=0;
		}
	}
	if(avoidtime>250)
	{
	  gRobot.status&=~STATUS_AVOID_HANDLE;
		gRobot.status|=STATUS_AVOID_JUDGE;
		avoidtime=0;
	}
}
void ParkingSmallHandle(void)
{
	static float aimAngle = 0;   //目标角度
	static float angleError = 0; //目标角度与当前角度的偏差
	static int i = 0;																  //目标角度变换标志位
	static int j = 0; 																//在此设立标志位在信号量10ms进入一次，达到延时的效果
	static int avoidtime=0;
	if (i == 0)																		    //使目标角度偏向右边45
	{
		aimAngle = gRobot.walk_t.pos.angle + 45; //让车头目标角度右偏45度
		i = 1;
	}
	angleError = angleErrorCount(aimAngle,gRobot.walk_t.pos.angle);
	j++;
	if (j < 120)
	{
		VelCrl(CAN2, 1, -10000); //pid中填入的是差值
		VelCrl(CAN2, 2,  10000);
	}else if (j >=120)
	{
		VelCrl(CAN2, 1, AnglePidControl(angleError)); //pid中填入的是差值
		VelCrl(CAN2, 2, AnglePidControl(angleError));
		if (fabs(angleError) < 5)
		{
			i = 0;
			j = 0;//清空标志位
			gRobot.status&=~STATUS_AVOID_JUDGE;
			gRobot.status&=~STATUS_AVOID_HANDLE;
			gRobot.status&=~STATUS_PARKING;//关闭停车
			gRobot.status|=STATUS_FIX;
		} 
	}
	
	if(avoidtime>400)
	{
		gRobot.status&=~STATUS_AVOID_HANDLE;
		gRobot.status|=STATUS_AVOID_JUDGE;
		avoidtime=0;
	}
}
void ParkingJudge(void)
{
	if(JudgeStick()==1)
	{
		//判断
		gRobot.abnormal=CheckIntersect();              //判断卡死的区域;
		gRobot.status&=~STATUS_AVOID_JUDGE;
		gRobot.status|=STATUS_AVOID_HANDLE;
	}
}
void ParkingLineBack(void)//边界倒车程序
{
	static float angleError = 0; //目标角度与当前角度的偏差
	static int j = 0; 																//在此设立标志位在信号量10ms进入一次，达到延时的效果
	static int avoidtime=0;
	angleError = angleErrorCount(gRobot.walk_t.pid.aimAngle,gRobot.walk_t.pos.angle);
	j++;
	if (j < 100)
	{
		VelCrl(CAN2, 1, -10000); //pid中填入的是差值
		VelCrl(CAN2, 2,  10000);
	}else if (j >=100)
	{
		VelCrl(CAN2, 1, AnglePidControl(angleError)); //pid中填入的是差值
		VelCrl(CAN2, 2, AnglePidControl(angleError));
		if (fabs(angleError) < 5)
		{
			j = 0;//清空标志位
			gRobot.status|=STATUS_AVOID_JUDGE;
			gRobot.status&=~STATUS_AVOID_HANDLE;
			avoidtime=0;
		}
	}
	if(avoidtime>250)
	{
	  gRobot.status&=~STATUS_AVOID_HANDLE;
		gRobot.status|=STATUS_AVOID_JUDGE;
		avoidtime=0;
	}
}

void ParkingHandle(void)
{
	switch(gRobot.abnormal)
	{
		case ABNOMAL_BLOCK_IN:
			CornerIn();
		break;
		
		case ABNOMAL_BLOCK_OUT :
		  CornerOut();
		break;
		
		case ABNOMAL_BLOCK_MIDDLE :
			CircleTransition();
		break;
		
		case ABNOMAL_BLOCK_IN_CORNER:
			CornerIn();
		break;
		
		case ABNOMAL_BLOCK_OUT_CORNER:
			CornerOut();
		break;
		
		default:
			USART_OUT(UART5,"SweepHandleErr");
		break;
	}
}
void FixJudge(void)
{


}
void Pointparking2(void)
{
	static int step=1;
	//circleChange();
	switch(gRobot.fix_t.wayChoose)
	{
	case 0:
		FixErr();
		break;
	case 1:
		AvoidCar(step);
		break;
	}
	
	switch(gRobot.fix_t.toBorder)
	{
		case 0:
			if(gRobot.walk_t.pos.y>1700&&gRobot.walk_t.pos.y<3100&&gRobot.walk_t.pos.x<-250)
			{
				step=!step;
				gRobot.status|=STATUS_FIX;
				gRobot.status&=~STATUS_PARKING;
				gRobot.status|=STATUS_SHOOTER;
			}
		break;
		
		case 1:
			if(gRobot.walk_t.pos.y<1700&&gRobot.walk_t.pos.x>-700&&gRobot.walk_t.pos.x<700)
			{
				step=!step;
				gRobot.status|=STATUS_FIX;
				gRobot.status&=~STATUS_PARKING;
				gRobot.status|=STATUS_SHOOTER;
			}
		break;
		
		case 2:
			if(gRobot.walk_t.pos.y>1700&&gRobot.walk_t.pos.y<3100&&gRobot.walk_t.pos.x>250)
			{
				step=!step;
				gRobot.status|=STATUS_FIX;
				gRobot.status&=~STATUS_PARKING;
				gRobot.status|=STATUS_SHOOTER;
			}
		break;
		
		case 3:
			if(gRobot.walk_t.pos.y>3100&&gRobot.walk_t.pos.x>-700&&gRobot.walk_t.pos.x<700)
			{
				step=!step;
				gRobot.status|=STATUS_FIX;
				gRobot.status&=~STATUS_PARKING;
				gRobot.status|=STATUS_SHOOTER;
			}
		break;
	}
//	if(gRobot.walk_t.circleChange.linenum>=2)
//		{
//			gRobot.walk_t.circleChange.linenum=0;
//			step++;
//			if(step>1)
//			{
//				step=0;
//			}
//			gRobot.status&=~STATUS_PARKING;
//			gRobot.status|=STATUS_FIX;
//			gRobot.status|=STATUS_SHOOTER;
//		}
}
	
void AvoidCar(int step)
{
	if(gRobot.avoid_t.direction==0) //顺时针
	{
		switch(step)
		{
			case 0:
				Square3();
				break;
			case 1:
				gRobot.walk_t.circleChange.turnTime=5;
			  Circle(2100,800);
				break;
		}
  }else if(gRobot.avoid_t.direction==1) //逆时针
	{
		switch(step)
		{
		  case 0:
				AntiSquare3();
				break;
			case 1:
				gRobot.walk_t.circleChange.turnTime=5;
			USART_OUT(UART5,"hhhhhh\r\n");
			USART_OUT(UART5,"hhhhhh\r\n");
			USART_OUT(UART5,"hhhhhh\r\n");
			  AntiCircle(2100,800);
				break;
		}
 }
}

void FixErr(void)
{
	if(gRobot.walk_t.circleChange.direction==0) //顺时针
	{
		gRobot.walk_t.circleChange.turnTime=5;
		Circle(2100,800);
	}else if(gRobot.walk_t.circleChange.direction==1) //逆时针
	{
		gRobot.walk_t.circleChange.turnTime=5;
		AntiCircle(2100,800);
  }
}

void FixHandle(void)
{
	gRobot.status&=~STATUS_AVOID_HANDLE;
}

void AbnormityJudge(void)
{
	if (gRobot.status & STATUS_SWEEP)
  {
			SweepJudge();
  }else if(gRobot.status & STATUS_PARKING)
	{
		  ParkingJudge();
	}
  else if (gRobot.status & STATUS_FIX)
  {
			FixJudge();
  }
  else if (gRobot.status & STATUS_SHOOTER)
  {
      ShootJudge();
  }	
  else if(gRobot.status & STATUS_CAMERA_WALK)
  {
      CWalkJudge();
	}
	USART_OUT(UART5,"statusRem=%d\t",(int)gRobot.avoid_t.statusRem);
	USART_OUT(UART5,"gRobot.status=%d\t",(int)gRobot.status);		
	USART_OUT(UART5,"Signal=%d\t",(int)gRobot.avoid_t.continueTriggerSignal);	
	USART_OUT(UART5,"continueTrigger=%d\t\r\n",(int)gRobot.avoid_t.continueTrigger);		
}
/****************************************************************************
* 名    称：void AbnormityHandle()
* 功    能：异常处理
* 入口参数：无
* 出口参数：无
* 说    明：无
* 调用方法：无 
****************************************************************************/
void AbnormityHandle(void)
{
	if (gRobot.status & STATUS_SWEEP)
  {
		SweepHandle();
  }
  else if (gRobot.status & STATUS_PARKING)
  {
		ParkingHandle();
  }else if(gRobot.status & STATUS_FIX)
	{
		FixHandle();
	}
  else if (gRobot.status & STATUS_SHOOTER)
  {
		ShootHandle();
  }	
  else if(gRobot.status & STATUS_CAMERA_WALK)
  {
    WalkHandle();
	}    	
}
/********************* (C) COPYRIGHT NEU_ACTION_2017 ****************END OF FILE************************/
