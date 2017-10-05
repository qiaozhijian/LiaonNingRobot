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
  static int againstError=0;
	static int totalTime=0;
  //这两个要结合在一起，不能在矫正的时候卡死
	totalTime++;
  if(fabs(gRobot.walk_t.pos.x-getxRem())<3&&fabs(gRobot.walk_t.pos.y-getyRem())<3&&gRobot.walk_t.base!=0&&(TRAVEL_SWITCH_LEFT==0||TRAVEL_SWITCH_LEFT==0))//卡住了
  {
  	againstError++;
		againstTime=0;
  }else
  {
		againstError=0;
  }
  
  if(fabs(gRobot.walk_t.pos.x-getxRem())<3&&fabs(gRobot.walk_t.pos.y-getyRem())<3&&gRobot.walk_t.base!=0&&(TRAVEL_SWITCH_LEFT==1&&TRAVEL_SWITCH_RIGHT==1))
  {
   againstTime++;
	 againstError=0;
  }
  else
  {
    againstTime = 0;
  }
  if (againstTime > 30)
  {
    againstTime=0;
		totalTime=0;
		return 1;
  }
	if(againstError>100)
  {
		againstError=0;
		totalTime=0;
		return 2;
  }
	if(totalTime>1000)
	{
		totalTime=0;
		return 1;
	}
  return 0;

}
/*****************************************************************************
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
  if(TRAVEL_SWITCH_LEFT==0&&TRAVEL_SWITCH_RIGHT==0)
	{
		VelCrl(CAN2, 1, -5000 + AnglePidControl(gRobot.walk_t.pid.angleError)-AgainstWallPidControl(spacingError));
		VelCrl(CAN2, 2, 5000 + AnglePidControl(gRobot.walk_t.pid.angleError)+AgainstWallPidControl(spacingError));
  }else if(TRAVEL_SWITCH_LEFT==1&&TRAVEL_SWITCH_RIGHT==0)
	{
		VelCrl(CAN2, 1, -12000);
		VelCrl(CAN2, 2,  4000);
	}else if(TRAVEL_SWITCH_LEFT==0 && TRAVEL_SWITCH_RIGHT==1)
	{
		VelCrl(CAN2, 1, -4000);
		VelCrl(CAN2, 2,  12000);
	}
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
  const float vOut2 = 1100.f;
  //内环速度
  const float vIn = 1300;  																//内环速度
  if (lineChangeSymbol==0)
  {
    gRobot.walk_t.base=gRobot.walk_t.base=vIn / (3.14f * WHEEL_DIAMETER) * 4096.f;
  }else if(lineChangeSymbol==1)
  {
    gRobot.walk_t.base=gRobot.walk_t.base=vOut2 / (3.14f * WHEEL_DIAMETER) * 4096.f;
  }
  else if (lineChangeSymbol>=2)
  {
   gRobot.walk_t.base=gRobot.walk_t.base=vOut1 / (3.14f * WHEEL_DIAMETER) * 4096.f;
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
	static int parkingTime=0;
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
	
	parkingTime++;//记录停车时间
	
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
	if(fabs(spacingError)>500)
  {
    VelCrl(CAN2, 1,13000.f+AnglePidControl(angleError));			//pid中填入的是差值
    VelCrl(CAN2, 2,-13000.f+AnglePidControl(angleError));
  }else if(fabs(spacingError)>300&&fabs(spacingError)<500)
	{
		VelCrl(CAN2, 1,8000.f+AnglePidControl(angleError));			//pid中填入的是差值
    VelCrl(CAN2, 2,-8000.f+AnglePidControl(angleError));
	}else if(fabs(spacingError)>200&&fabs(spacingError)<300)			//设立减速环带
  {
    VelCrl(CAN2, 1,2000+AnglePidControl(angleError));		  //pid中填入的是差值
    VelCrl(CAN2, 2,-2000+AnglePidControl(angleError));
  }else if(fabs(spacingError)<200&&fabs(spacingError)>0)
  {
    VelCrl(CAN2, 1,0);																		//pid中填入的是差值
    VelCrl(CAN2, 2,0);
		parkingTime=0;
    return 1;
  }
	if(parkingTime>700)//超出6秒没有到达目标点便判定为与车卡死里面进行矫正
	{
		parkingTime=0;
		gRobot.status|=STATUS_FIX;
		gRobot.status|=STATUS_AVOID_JUDGE;
		gRobot.status&=~STATUS_AVOID_HANDLE;
		gRobot.abnormal=ABNOMAL_PARKING_BLOCK;//停车异常
	}
	return 0;
}

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
  static int turnTime=0;
  switch(turnTime)
  {
  case 0:
    if(200<gRobot.walk_t.pos.x&&gRobot.walk_t.pos.x<300)
      turnTime=1;
    ShunShiZhenCircleBiHuan(800,500,-100,600.f);
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
#define SUCCESSTIMES 20
int LaserStart(void)
{
  static uint32_t successCount[6]={0};		//前三个代表左的不同状况 后三个代表右的不同状况
  float leftLaser=getLeftAdc()-ROBOT_WIDTH/2.f;
  float RightLaser=getRightAdc()-ROBOT_WIDTH/2.f;
	int status=0;
	//如果激光超出范围
  if(leftLaser>=600.f && RightLaser>=600.f)
    return 1;
  //如果左激光被触发
  if(leftLaser<900.f){
    if(leftLaser<300.f){
      successCount[0]++;
    }
    else if(leftLaser>=300.f&&leftLaser<=600.f){
      successCount[1]++;
    }
    else{
      successCount[2]++;
    }
  }
  //如果右激光被触发
  if(RightLaser<900.f){
    if(RightLaser<300.f){
      successCount[3]++;
    }
    else if(RightLaser>=300.f&&RightLaser<=600.f){
      successCount[4]++;
    }
    else{
      successCount[5]++;
    }
  }
	/*status 取 0,1,2,3,4,5*/
	for(int i=0;i<5;i++){
		if(successCount[i]>SUCCESSTIMES)
			status=i+1;
	}
  //处理状态
	switch(status){
		case 0:
			return 1;
		case 1:
			gRobot.walk_t.laser.status=1;
			gRobot.walk_t.circleChange.direction=0;
			return 0;
		case 2:
			gRobot.walk_t.laser.status=1;
			gRobot.walk_t.circleChange.direction=0;
			return 0;
		case 3:
			gRobot.walk_t.laser.status=1;
			gRobot.walk_t.circleChange.direction=0;
			return 0;
		case 4:
			gRobot.walk_t.laser.status=1;
			gRobot.walk_t.circleChange.direction=1;
			return 0;
		case 5:
			gRobot.walk_t.laser.status=1;
			gRobot.walk_t.circleChange.direction=1;
			return 0;
		case 6:
			gRobot.walk_t.laser.status=1;
			gRobot.walk_t.circleChange.direction=1;
			return 0;
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
  
  VelCrl(CAN2, 1, gRobot.walk_t.base + AnglePidControl(gRobot.walk_t.pid.angleError +sign* distancePidControl(gRobot.walk_t.pid.disError))); //pid中填入的是差值
  VelCrl(CAN2, 2, -gRobot.walk_t.base+ AnglePidControl(gRobot.walk_t.pid.angleError +sign* distancePidControl(gRobot.walk_t.pid.disError)));
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
  
  VelCrl(CAN2, 1, gRobot.walk_t.base + AnglePidControl(gRobot.walk_t.pid.angleError +sign* distancePidControl(gRobot.walk_t.pid.disError))); //pid中填入的是差值
  VelCrl(CAN2, 2, -gRobot.walk_t.base + AnglePidControl(gRobot.walk_t.pid.angleError +sign* distancePidControl(gRobot.walk_t.pid.disError)));
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
  switch(gRobot.walk_t.laser.status)
  {
  case 1:
    In2Out(1,gRobot.walk_t.circleChange.direction);
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
	circleChange();
	if(gRobot.walk_t.circleChange.circleNum==0)
	{
		Square();
	}else if(gRobot.walk_t.circleChange.circleNum==1)
  {
		gRobot.walk_t.circleChange.turnTime=4;
		Circle();
  }else if(gRobot.walk_t.circleChange.circleNum==2)
	{
		gRobot.walk_t.circleChange.turnTime=5;
		Circle();
	}
	else if(gRobot.walk_t.circleChange.circleNum>=3)
	{
		Square2();
	}
	if(	gRobot.walk_t.circleChange.quadrantlast<gRobot.walk_t.circleChange.quadrant && gRobot.walk_t.circleChange.quadrant!=4 && gRobot.walk_t.circleChange.quadrantlast!=1)
	{
		gRobot.walk_t.circleChange.linenum=gRobot.walk_t.circleChange.linenum-2;
	}
	gRobot.walk_t.circleChange.quadrantlast=gRobot.walk_t.circleChange.quadrant;
    //进入矫正	
    if(gRobot.walk_t.circleChange.circleNum==4)
    {
        gRobot.status&=~STATUS_SWEEP;
        gRobot.walk_t.circleChange.turnTime=0;
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
	circleChange();
	if(gRobot.walk_t.circleChange.circleNum==0)
	{
		AntiSquare();
	}else if(gRobot.walk_t.circleChange.circleNum==1)
  {
		gRobot.walk_t.circleChange.turnTime=4;
		AntiCircle();
  }else if(gRobot.walk_t.circleChange.circleNum==2)
	{
		gRobot.walk_t.circleChange.turnTime=5;
		AntiCircle();
	}
	else if(gRobot.walk_t.circleChange.circleNum>=3)
	{
		AntiSquare2();
	}
	if(gRobot.walk_t.circleChange.quadrant<gRobot.walk_t.circleChange.quadrantlast && gRobot.walk_t.circleChange.quadrant!=1&&gRobot.walk_t.circleChange.quadrantlast!=4)
	{
		gRobot.walk_t.circleChange.linenum=gRobot.walk_t.circleChange.linenum-2;
	}
	gRobot.walk_t.circleChange.quadrantlast=gRobot.walk_t.circleChange.quadrant;
    //进入矫正	
    if(gRobot.walk_t.circleChange.circleNum==4)
    {
        gRobot.status&=~STATUS_SWEEP;
        gRobot.walk_t.circleChange.turnTime=0;
    }
  }
/****************************************************************************
* 名    称：void ChangeBoard(void)	
* 功    能：区域限定与划分
* 入口参数：无
* 出口参数：无
* 说    明：无
* 调用方法：无 
****************************************************************************/
void ChangeBoard(void){
	if(gRobot.walk_t.circleChange.direction==0) //顺时针
	{
	if(gRobot.walk_t.circleChange.circleNum==0){
		gRobot.walk_t.board[0][0]=-100;
		gRobot.walk_t.board[0][1]=2900;
		gRobot.walk_t.board[0][2]=100;
		gRobot.walk_t.board[0][3]=1900;
	}else if(gRobot.walk_t.circleChange.circleNum!=0){
		gRobot.walk_t.board[1][0]=-1110;
		gRobot.walk_t.board[1][1]=3550;
		gRobot.walk_t.board[1][2]=1110;
		gRobot.walk_t.board[1][3]=1250;
	}
  }else if(gRobot.walk_t.circleChange.direction==1) //逆时针
	{
		if(gRobot.walk_t.circleChange.circleNum==0){
		gRobot.walk_t.board[0][0]=100;
		gRobot.walk_t.board[0][1]=2900;
		gRobot.walk_t.board[0][2]=-100;
		gRobot.walk_t.board[0][3]=1900;
	}else if(gRobot.walk_t.circleChange.circleNum!=0){
		gRobot.walk_t.board[1][0]=1110;
		gRobot.walk_t.board[1][1]=3550;
		gRobot.walk_t.board[1][2]=-1110;
		gRobot.walk_t.board[1][3]=1250;
	}

}
}
/****************************************************************************
* 名    称：void Square(void)	
* 功    能：顺时针正方形行驶
* 入口参数：无
* 出口参数：无
* 说    明：无
* 调用方法：无 
****************************************************************************/
int Square(void)
{
	//改变区域的值
	ChangeBoard();
	if(gRobot.walk_t.pos.x<(gRobot.walk_t.board)[0][0] && gRobot.walk_t.pos.y<(gRobot.walk_t.board)[0][1]){
		gRobot.walk_t.circleChange.turnTime=0;
	}else if(gRobot.walk_t.pos.x<(gRobot.walk_t.board)[0][2] && gRobot.walk_t.pos.y>(gRobot.walk_t.board)[0][1]){
		gRobot.walk_t.circleChange.turnTime=1;
	}else if(gRobot.walk_t.pos.x>(gRobot.walk_t.board)[0][2] && gRobot.walk_t.pos.y>(gRobot.walk_t.board)[0][3]){
		gRobot.walk_t.circleChange.turnTime=2;
	}else if(gRobot.walk_t.pos.x>(gRobot.walk_t.board)[0][0] && gRobot.walk_t.pos.y<(gRobot.walk_t.board)[0][3]){
		gRobot.walk_t.circleChange.turnTime=3;
	}else{
		//角度闭环
		if(gRobot.walk_t.pos.y>UPPER_FRAME){
		 AngleRoute(-90);
		}else if(gRobot.walk_t.pos.y<DOWN_FRAME){
		 AngleRoute(90);
		}else if(gRobot.walk_t.pos.x>RIGHT_FRAME){
		 AngleRoute(180);
		}else{
		 AngleRoute(0);
		}
	}
	
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
    
  default:
    break;
  }
  return 1;
}	
/****************************************************************************
* 名    称：void AntiSquare(void)	
* 功    能：逆时针正方行行驶
* 入口参数：无
* 出口参数：无
* 说    明：无
* 调用方法：无 
****************************************************************************/
int AntiSquare(void)
{
	//改变区域的值
	ChangeBoard();
	
	if(gRobot.walk_t.pos.x>(gRobot.walk_t.board)[0][0] && gRobot.walk_t.pos.y<(gRobot.walk_t.board)[0][1]){
		gRobot.walk_t.circleChange.turnTime=0;
	}else if(gRobot.walk_t.pos.x>(gRobot.walk_t.board)[0][2] && gRobot.walk_t.pos.y>(gRobot.walk_t.board)[0][1]){
		gRobot.walk_t.circleChange.turnTime=1;
	}else if(gRobot.walk_t.pos.x<(gRobot.walk_t.board)[0][2] && gRobot.walk_t.pos.y>(gRobot.walk_t.board)[0][3]){
		gRobot.walk_t.circleChange.turnTime=2;
	}else if(gRobot.walk_t.pos.x<(gRobot.walk_t.board)[0][0] && gRobot.walk_t.pos.y<(gRobot.walk_t.board)[0][3]){
		gRobot.walk_t.circleChange.turnTime=3;
	}else{
		//角度闭环
		if(gRobot.walk_t.pos.y>UPPER_FRAME){
		 AngleRoute(90);
		}else if(gRobot.walk_t.pos.y<DOWN_FRAME){
		 AngleRoute(-90);
		}else if(gRobot.walk_t.pos.x>RIGHT_FRAME){
		 AngleRoute(0);
		}else{
		 AngleRoute(180);
		}
	}
  switch(gRobot.walk_t.circleChange.turnTime)
  {
  case 0:
    Line(600.f,3300.f,0,0,1,1);
    break;
    
  case 1:
    Line(-500.f,3300.f,90,1,1,1);
    break;
    
  case 2:
    Line(-600.f,1400,180,0,-1,1);
    break;
    
  case 3:
    Line(600.f,1400,-90,1,-1,1);
    break;
    
  default:
    break;
  }
  return 1;
}
/****************************************************************************
* 名    称：void Circle(void)	
* 功    能：顺时针正方向画圆
* 入口参数：无
* 出口参数：无
* 说    明：无
* 调用方法：无 
****************************************************************************/
int Circle(void)
{
  switch(gRobot.walk_t.circleChange.turnTime)
  {
  case 4:
    ShunShiZhenCircleBiHuan(2200,900,0,2400);
    break;
    
  case 5:
    ShunShiZhenCircleBiHuan(2400,1400.f,0,2400);
    break;
    
  default:
    break;
  }
  return 1;
}
/****************************************************************************
* 名    称：void Circle(void)	
* 功    能：顺时针正方向画圆
* 入口参数：无
* 出口参数：无
* 说    明：无
* 调用方法：无 
****************************************************************************/
int AntiCircle(void)
{
  switch(gRobot.walk_t.circleChange.turnTime)
  {		
  case 4:
    NiShiZhenCircleBiHuan(2200,800,0,2400);
    break;
    
  case 5:
    NiShiZhenCircleBiHuan(2400,1200.f,0,2400);
    break;
    
  default:
    break;
  }
  return 1;
}
/****************************************************************************
* 名    称：void Circle(void)	
* 功    能：顺时针最外圈正方形
* 入口参数：无
* 出口参数：无
* 说    明：无
* 调用方法：无 
****************************************************************************/
int Square2(void)
{
		//改变区域的值
	ChangeBoard();
	
	if(gRobot.walk_t.pos.x<(gRobot.walk_t.board)[1][0] && gRobot.walk_t.pos.y<(gRobot.walk_t.board)[1][1]){
		gRobot.walk_t.circleChange.turnTime=6;
	}else if(gRobot.walk_t.pos.x<(gRobot.walk_t.board)[1][2] && gRobot.walk_t.pos.y>(gRobot.walk_t.board)[1][1]){
		gRobot.walk_t.circleChange.turnTime=7;
	}else if(gRobot.walk_t.pos.x>(gRobot.walk_t.board)[1][2] && gRobot.walk_t.pos.y>(gRobot.walk_t.board)[1][3]){
		gRobot.walk_t.circleChange.turnTime=8;
	}else if(gRobot.walk_t.pos.x>(gRobot.walk_t.board)[1][0] && gRobot.walk_t.pos.y<(gRobot.walk_t.board)[1][3]){
		gRobot.walk_t.circleChange.turnTime=9;
	}else{
		//角度闭环
		if(gRobot.walk_t.pos.y>UPPER_FRAME){
		 AngleRoute(-90);
		}else if(gRobot.walk_t.pos.y<DOWN_FRAME){
		 AngleRoute(90);
		}else if(gRobot.walk_t.pos.x>RIGHT_FRAME){
		 AngleRoute(180);
		}else{
		 AngleRoute(0);
		}
	}
	
  switch(gRobot.walk_t.circleChange.turnTime)
  {
  case 6:
    Line(-2100.f,3400.f,0,0,1,1);//x
    break;
    
  case 7:
    Line(600.f,4500.f,-90,1,1,1);//y
    break;
    
  case 8:
    Line(2100.f,1400,180,0,-1,1);//x
    break;
    
  case 9:
    Line(-600.f,300,90,1,-1,1);//y
    break;
    
  default:
    break;
  }
  return 1;
}
/****************************************************************************
* 名    称：int AntiSquare2(void)	
* 功    能：逆时针最外圈正方形
* 入口参数：无
* 出口参数：无
* 说    明：无
* 调用方法：无 
****************************************************************************/
int AntiSquare2(void)
{
		ChangeBoard();
	
	if(gRobot.walk_t.pos.x>(gRobot.walk_t.board)[1][0]&&gRobot.walk_t.pos.y<(gRobot.walk_t.board)[1][1]){
		gRobot.walk_t.circleChange.turnTime=6;
	}else if(gRobot.walk_t.pos.x>(gRobot.walk_t.board)[1][2]&&gRobot.walk_t.pos.y>(gRobot.walk_t.board)[1][1]){
		gRobot.walk_t.circleChange.turnTime=7;
	}else if(gRobot.walk_t.pos.x<(gRobot.walk_t.board)[1][2]&&gRobot.walk_t.pos.y>(gRobot.walk_t.board)[1][3]){
		gRobot.walk_t.circleChange.turnTime=8;
	}else if(gRobot.walk_t.pos.x<(gRobot.walk_t.board)[1][0]&&gRobot.walk_t.pos.y<(gRobot.walk_t.board)[1][3]){
		gRobot.walk_t.circleChange.turnTime=9;
	}else{
		//角度闭环
		if(gRobot.walk_t.pos.y>UPPER_FRAME){
		 AngleRoute(90);
		}else if(gRobot.walk_t.pos.y<DOWN_FRAME){
		 AngleRoute(-90);
		}else if(gRobot.walk_t.pos.x>RIGHT_FRAME){
		 AngleRoute(0);
		}else{
		 AngleRoute(180);
		}
	}
  switch(gRobot.walk_t.circleChange.turnTime)
  {
  case 6:
    Line(2100.f,3400.f,0,0,1,1);
    break;
    
  case 7:
    Line(-600.f,4500.f,90,1,1,1);
    break;
    
  case 8:
    Line(-2100.f,1400,180,0,-1,1);
    break;
    
  case 9:
    Line(600.f,300,-90,1,-1,1);
    break;
  default:
    break;
  }
  return 1;
}
/****************************************************************************
* 名    称：AngleRoute()	
* 功    能：走x定直线
* 入口参数：无
* 出口参数：无
* 说    明：无
* 调用方法：无 
****************************************************************************/
void AngleRoute(float aimangle)
{
  static float AngleErr=0;
	AngleErr=angleErrorCount(aimangle,gRobot.walk_t.pos.angle);
	VelCrl(CAN2, 1,7000+AnglePidControl(AngleErr)); //pid中填入的是差值
	VelCrl(CAN2, 2,-7000+AnglePidControl(AngleErr));
}
/********************* (C) COPYRIGHT NEU_ACTION_2017 ****************END OF FILE************************/

