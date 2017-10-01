#include "config.h"

extern Robot_t gRobot;

//static int turnTimeRemember;												//记住在卡死的时候是什么直线的状态，等倒车case结束后让重新填装
static int statusRemember;                          //记住卡死时的状态码
static float xStick=0;
static float yStick=0;												//卡住时存储的位置数据

/****************************************************************************
* 名    称：void BackCarIn(float angle)
* 功    能：内环逃逸程序后退1.5s，外转45度
* 入口参数：angle//当前角度
* 出口参数：无
* 说    明：无
* 调用方法：无 
****************************************************************************/
void BackCarIn(float angle) 												//内环倒车程序
{
  static float aimAngle = 0;   											//目标角度
  static float angleError = 0; 											//目标角度与当前角度的偏差
  static int i = 0;																  //目标角度变换标志位
  static int j = 0; 																//在此设立标志位在信号量10ms进入一次，达到延时的效果
  if (i == 0)																		    //使目标角度偏向右边45
  {
    aimAngle = angle - 45;												//让车头目标角度右偏45度
    i = 1;
  }
  angleError = angleErrorCount(aimAngle,angle);
  j++;
  if (j < 150)
  {
    VelCrl(CAN2, 1, -6107); 											//pid中填入的是差值
    VelCrl(CAN2, 2,  6107);
  }else if(j >=150)
  {
    VelCrl(CAN2, 1, AnglePidControl(angleError)); 	//pid中填入的是差值
    VelCrl(CAN2, 2, AnglePidControl(angleError));
    if (fabs(angleError) < 5)
    {
      
      //gRobot.turnTime = turnTimeRemember;
      i = 0;	
      j = 0;																				//清空标志位
      //turnTimeRemember=0;
      
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
void BackCarOut(float angle) 											//外环倒车程序
{
  static float aimAngle = 0;  									  //目标角度
  static float angleError = 0; 										//目标角度与当前角度的偏差
  static int i = 0;																//目标角度变换标志位
  static int j = 0; 															//在此设立标志位在信号量10ms进入一次，达到延时的效果
  if (i == 0)																		  //使目标角度偏向右边45
  {
    aimAngle = angle + 60; 												//让车头目标角度右偏45度
    i = 1;
  }
  angleError = angleErrorCount(aimAngle,angle);
  j++;
  if (j < 100)
  {
    VelCrl(CAN2, 1, -6107); 											//pid中填入的是差值
    VelCrl(CAN2, 2,  6107);
  }else if (j >=100)
  {
    VelCrl(CAN2, 1, AnglePidControl(angleError)); //pid中填入的是差值
    VelCrl(CAN2, 2, AnglePidControl(angleError));
    if (fabs(angleError) < 5)
    {
      //gRobot.turnTime = turnTimeRemember;
      i = 0;
      j = 0;																		 //清空标志位
      //turnTimeRemember=0;
      
      gRobot.avoid_t.posRem.angle=gRobot.walk_t.pos.angle;
      gRobot.avoid_t.passflag=1;                    //检测是否执行过倒车
      gRobot.avoid_t.pid.aimAngle=gRobot.walk_t.pos.angle+180;
      
      gRobot.status=statusRemember;              //切换到进入避障前的大状态
      gRobot.walk_t.circleChange.turnTimerem=gRobot.walk_t.circleChange.turnTime;
    }
  }
  //	pidZongShuchu = AnglePidControl(angleError);
}

/****************************************************************************
* 名    称：BackCar
* 功    能：倒车
* 入口参数：当前角度
* 出口参数：无
* 说    明：无
* 调用方法：无 
****************************************************************************/
void BackCar(float angle) 											//外环倒车程序
{
  static float aimAngle = 0.f;  									  //目标角度
  static float angleError = 0.f; 										//目标角度与当前角度的偏差
  static int i = 0;																//目标角度变换标志位
  static int j = 0; 															//在此设立标志位在信号量10ms进入一次，达到延时的效果
  int sign=0;///通过判断在外在内查看它应该外拐内拐
  if((xStick>-1400.f&&xStick<1400.f)&&(yStick>900.f&&yStick<3900.f))			//内环
  {
    sign=-1;
  }
  else if((xStick<-1400.f||xStick>1400.f)||(yStick<900.f||yStick>3900.f))	//外环
  {
    sign=1;
  }
  //	if(direction=shunshizhen)
  //	{
  //		sign=-sign;
  //	}
  if (i == 0)																		  //使目标角度偏向右边45
  {
    aimAngle = angle +sign*60; 												//让车头目标角度右偏45度
    i = 1;
  }
  angleError = angleErrorCount(aimAngle,angle);
  j++;
  if (j < 100)
  {
    VelCrl(CAN2, 1, -6107); 											//pid中填入的是差值
    VelCrl(CAN2, 2,  6107);
  }else if (j >=100)
  {
    VelCrl(CAN2, 1, AnglePidControl(angleError)); //pid中填入的是差值
    VelCrl(CAN2, 2, AnglePidControl(angleError));
    if (fabs(angleError) < 5)
    {
      //gRobot.turnTime = turnTimeRemember;
      i = 0;
      j = 0;																		 //清空标志位
      gRobot.status=statusRemember; 
    }
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
/****************************************************************************
* 名    称：Turn180(void)	
* 功    能：逆时针转角函数
* 入口参数：无
* 出口参数：无
* 说    明：无
* 调用方法：无 
* 注    意: 
****************************************************************************/
int Turn180(void)
{
  //防止旋转过程中turnTime加
  gRobot.walk_t.circleChange.linenum=0;
  if(fabs(angleErrorCount(gRobot.avoid_t.pid.aimAngle,gRobot.walk_t.pos.angle)) >100)
  {
    if(gRobot.walk_t.circleChange.turnTime<=4 && gRobot.walk_t.circleChange.direction==0) 
    {	
      VelCrl(CAN2, 1, -1000);     //顺时针
      VelCrl(CAN2, 2, 10000);
    }else if(gRobot.walk_t.circleChange.turnTime>4 && gRobot.walk_t.circleChange.direction==0)
    {
      VelCrl(CAN2, 1,-10000);
      VelCrl(CAN2, 2,1000);
    }else if(gRobot.walk_t.circleChange.turnTime<=4 && gRobot.walk_t.circleChange.direction==1)
    {	
      VelCrl(CAN2, 1, -10000);    //逆时针
      VelCrl(CAN2, 2, 1000);
    }else if(gRobot.walk_t.circleChange.turnTime>4 && gRobot.walk_t.circleChange.direction==1)
    {
      VelCrl(CAN2, 1,-1000);
      VelCrl(CAN2, 2,10000);
    }
  }else if(fabs(angleErrorCount(gRobot.avoid_t.pid.aimAngle,gRobot.walk_t.pos.angle))<100)
  {
    if(gRobot.walk_t.circleChange.turnTime<=4 && gRobot.walk_t.circleChange.direction==0) 
    {	
      VelCrl(CAN2, 1, 10000);     //顺时针
      VelCrl(CAN2, 2, 0);
    }else if(gRobot.walk_t.circleChange.turnTime>4 && gRobot.walk_t.circleChange.direction==0)
    {
      VelCrl(CAN2, 1,0);
      VelCrl(CAN2, 2,-10000);
    }else if(gRobot.walk_t.circleChange.turnTime<=4 && gRobot.walk_t.circleChange.direction==1)
    {		
      VelCrl(CAN2, 1, 0);      //逆时针
      VelCrl(CAN2, 2, -10000);
    }else if(gRobot.walk_t.circleChange.turnTime>4 && gRobot.walk_t.circleChange.direction==1)
    {
      VelCrl(CAN2, 1,10000);
      VelCrl(CAN2, 2,0);
    }
  }
  //判断是否满足条件
  //	//时间条件
  //	gRobot.avoid_t.pid.pidtime++;
  //	if(gRobot.avoid_t.pid.pidtime>400)
  //	{
  //		gRobot.avoid_t.pid.pidtime=0;
  //		return 1;
  //	}
  //角度条件
  if(fabs(angleErrorCount(gRobot.avoid_t.pid.aimAngle,gRobot.walk_t.pos.angle)) <5)
  {
    return 1;
  }
  else
  {
    return 0;
  }
}
/****************************************************************************
* 名    称：CheckOutline2()
* 功    能：检测是否卡死
* 入口参数：无
* 出口参数：无
* 说    明：无
* 调用方法：无 
****************************************************************************/
void CheckOutline(void)
{
  static int count=0;
  static float velX=0;
  static float velY=0;
  static float aimVel=0.0f;
  static float lastX=0;
  static float lastY=0;
  static int stickError=0;
  //记录从哪个状态进来的
  statusRemember=gRobot.status;
  count++;
  if(count==5){
    count=0;
    velX=(gRobot.walk_t.pos.x-lastX)/5.0f*100.0f;
    velY=(gRobot.walk_t.pos.y-lastY)/5.0f*100.0f;
    //卡死速度比较值
    aimVel=Pulse2Vel(gRobot.walk_t.base);
		//目前实际速度
    gRobot.walk_t.averageV=__sqrtf(velX*velX+velY*velY);
		
    if(TwoNumCompare(gRobot.walk_t.averageV,aimVel*0.2f))
    {
      stickError++;
    }
    else
    {
      stickError=0;
    }
    if(stickError==5)
    {
      stickError=0;
      //改变状态码
      gRobot.status=STATUS_AVOID;          //进入backcar
      gRobot.avoid_t.signal=0;             //清零
      JudgeStick();                        //判断卡死的区域
    }
    lastX=gRobot.walk_t.pos.x;
    lastY=gRobot.walk_t.pos.y;
    USART_OUT(UART5,"%d\t",(int)velX);
    USART_OUT(UART5,"%d\t",(int)velY);
    USART_OUT(UART5,"%d\t",(int)aimVel);
    USART_OUT(UART5,"%d\t",(int)__sqrtf(velX*velX+velY*velY));
    USART_OUT(UART5,"%d\t",(int)gRobot.walk_t.pos.x);
    USART_OUT(UART5,"%d\t",(int)gRobot.walk_t.pos.y);	
    USART_OUT(UART5,"%d\t",(int)stickError);	
    USART_OUT(UART5,"%d\t",(int)gRobot.status);	
    USART_OUT(UART5,"%d\r\n",(int)gRobot.walk_t.circleChange.turnTime);
  }
}
/****************************************************************************
* 名    称：void Escape(void)
* 功    能：逃逸执行函数
* 入口参数：无
* 出口参数：无
* 说    明：无
* 调用方法：无 
****************************************************************************/
static int stickStatus=0;//判断现在卡死的状态
void Escape(void)
{
  USART_OUT(UART5,"stick:%d\r\n",(int)stickStatus);
  switch(stickStatus)
  {
  case 1://内圈撞墙
    USART_OUT(UART5,"case1");
    BackCar(gRobot.walk_t.pos.angle);
    break;
    
  case 2://外圈撞墙
    USART_OUT(UART5,"case2");
    BackCar(gRobot.walk_t.pos.angle);
    break;
    
  case 3://和车撞
    //SoundOut();
    USART_OUT(UART5,"case3");
    if(statusRemember==25)
    {
      gRobot.avoid_t.posRem.angle=gRobot.walk_t.pos.angle;
      gRobot.avoid_t.passflag=1;                    //检测是否执行过倒车
      gRobot.avoid_t.pid.aimAngle=gRobot.walk_t.pos.angle+180;
      
      gRobot.status=statusRemember;              //切换到进入避障前的大状态
      gRobot.walk_t.circleChange.turnTimerem=gRobot.walk_t.circleChange.turnTime;
    }else if(statusRemember==6)
    {
      BackCar(gRobot.walk_t.pos.angle);
    }
    break;
    
  case 4://和内圈角撞
    USART_OUT(UART5,"case4");
    BackCar(gRobot.walk_t.pos.angle);
    break;
    
  case 5://和外圈角撞
    USART_OUT(UART5,"case5");
    BackCar(gRobot.walk_t.pos.angle);
    break;
    
  default:
    break;
  }
}
/****************************************************************************
* 名    称：CheckIntersect(float x, float y, float angle, Point_t cP[4])
* 功    能：检查是否撞车
* 入口参数：无
* 出口参数：无
* 说    明：无
* 调用方法：无 
****************************************************************************/
void JudgeStick(void)
{
  float x = getXpos(), y = getYpos(), angle = getAngle();
  static Point_t cP[4];//车的四个顶点
  CarPointTrans(x, y, angle, cP);
  stickStatus = CheckIntersect(x, y, angle, cP);
  USART_OUT(UART5,"x=%d\t",(int)x);
  USART_OUT(UART5,"y=%d\t",(int)y);
  USART_OUT(UART5,"angle=%d\t\r\n",(int)angle);
}
//	cP[0] leftUp;
//	cP[1] letDown;
//	cP[2] rightDown;
//	cP[3] rightUp;
/****************************************************************************
* 名    称：CheckIntersect(float x, float y, float angle, Point_t cP[4])
* 功    能：检查撞到的状态
* 入口参数：无
* 出口参数：无
* 说    明：无
* 调用方法：无 
****************************************************************************/
int CheckIntersect(float x, float y, float angle, Point_t cP[4])//检查是否与边相交
{
  static Point_t boardPointOut[4];//boardPoint
  static Point_t boardPointIn[4];
  int StickOut[4] = {0};
  int StickIn[4] = {0};
  int intersect = 0;
  int stickCornor = 0;//卡角标志位
  //外框木板点群
  boardPointOut[0].x = -2400; 	boardPointOut[1].x = -2400;
  boardPointOut[0].y = 4800;    boardPointOut[1].y = 0;
  
  boardPointOut[2].x = 2400;    boardPointOut[3].x = 2400;
  boardPointOut[2].y = 0;				boardPointOut[3].y = 4800;
  
  //铁框点群
  boardPointIn[0].x = -250; 		boardPointIn[1].x = -250;
  boardPointIn[0].y = 3100;    	boardPointIn[1].y = 1700;
  
  boardPointIn[2].x = 250;      boardPointIn[3].x = 250;
  boardPointIn[2].y = 1700;			boardPointIn[3].y = 3100;
  
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
    stickStatus = 2;//卡在外框边
  }
  else if (intersect == 1)
  {
    stickStatus = 1;//卡在内框边
  }
  else if (intersect == 0)
  {
    stickStatus = 3;//和车碰撞
  }		
  
  if(stickCornor==0)
  {
    stickStatus=stickStatus;
  }
  else if(stickCornor==1)
  {
    stickStatus=4;//卡在内圈角落
  }else if(stickCornor==2)
  {
    stickStatus=5;//卡在外圈角落
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
/********************* (C) COPYRIGHT NEU_ACTION_2017 ****************END OF FILE************************/
