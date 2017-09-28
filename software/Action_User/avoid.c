#include "config.h"

extern Robot_t gRobot;

//static int turnTimeRemember;												//记住在卡死的时候是什么直线的状态，等倒车case结束后让重新填装
static int statueRemember;                          //记住卡死时的状态码
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
			
			gRobot.status=statueRemember;              //切换到进入避障前的大状态
			gRobot.walk_t.circlechange.turntimerem=gRobot.walk_t.circlechange.turntime;
		}
	}
//	pidZongShuchu = AnglePidControl(angleError);
}
 /****************************************************************************
* 名    称：void CheckOutline(void) 
* 功    能：检测是否卡死
* 入口参数：无
* 出口参数：无
* 说    明：当前是停在x=1000 y=0
* 调用方法：无 
****************************************************************************/
void CheckOutline(void)																	//检测是否卡死
{
	static int stickError = 0;													  //卡死错误积累值
	static float xError = 0, yError = 0;
	statueRemember=gRobot.status;
	xError = gRobot.walk_t.pos.x - getxRem();
	yError = gRobot.walk_t.pos.y - getyRem();
																												//判断进程到哪一步（替换M）  --summer
	if (fabs(xError) < 1 && fabs(yError) < 1 && gRobot.walk_t.left.base >2000)
	{
		stickError++;
	}
	else
	{
		stickError = 0;
	}
	/*
	200太大
	分情况：启动的时候，走直线的时候，过弯的时候，停下投球，矫正的时候。
	不一定是小于1  根据应该有的速度设定
	*/
	if (stickError > 130)
	{
		xStick = getxRem();																	//记住卡死的坐标
		yStick = getyRem();
		stickError = 0;
		gRobot.avoid_t.signal=0;                            //清零
		gRobot.status=32;
		
	}
}
 /****************************************************************************
* 名    称：BackCar
* 功    能：倒车
* 入口参数：当前角度
* 出口参数：无
* 说    明：无
* 调用方法：无 
****************************************************************************/
void BackCar(float angle)
{
	
		gRobot.avoid_t.posRem.angle=gRobot.walk_t.pos.angle;
		gRobot.avoid_t.passflag=1;                    //检测是否执行过倒车
		gRobot.avoid_t.pid.aimAngle=gRobot.walk_t.pos.angle+180;	
		
		gRobot.status=statueRemember;                 //切换到进入避障前的大状态
//	angle=gRobot.walk_t.pos.angle;
//	if((xStick>-1400&&xStick<1400)&&(yStick>900&&yStick<3900))			//内环
//		{
//			//BackCarIn(angle);
//		}
//	else if((xStick<-1400||xStick>1400)||(yStick<900||yStick>3900))	//外环
//		{
//			BackCarOut(angle);
//		}
}	
 /****************************************************************************
* 名    称：CheckOutline3()
* 功    能：检测是否卡死
* 入口参数：无
* 出口参数：无
* 说    明：无
* 调用方法：无 
****************************************************************************/
void CheckOutline3(void)//检测是否卡死
{
	static int stickError = 0;							//卡死错误积累值
	//每100ms做为速度平均的一个周期
	statueRemember=gRobot.status;
	if(gRobot.walk_t.right.real<=gRobot.walk_t.right.aim*0.3f)
	{
	  stickError++;
	}else
	{
		stickError = 0;
	}
	if (stickError > 10)
	{
		xStick = getxRem();                  //记住卡死的坐标
		yStick = getyRem();
		//改变状态码
		stickError = 0;
		gRobot.avoid_t.signal=0;             //清零
		gRobot.status=32;
	}
	//USART_OUT(UART5,(uint8_t *)"%d\r\n",stickError);
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
	
	switch(LineCheck(gRobot.walk_t.circlechange.direction))
	{
		case 1://X=2400
			 if(gRobot.walk_t.circlechange.direction==0)
			   {
					 if(fabs(2400-gRobot.walk_t.pos.x-getLeftAdc())>500)
					 {
						 return NO;
					 }else{
						return YES;
				   }
			   }else if(gRobot.walk_t.circlechange.direction==1)
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
			   	if(gRobot.walk_t.circlechange.direction==0)
			   {
					 if(fabs(4800-gRobot.walk_t.pos.y-getLeftAdc())>500)
					 {
						 return NO;
					 }else{
						return YES;
				   }
			   }else if(gRobot.walk_t.circlechange.direction==1)
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
					if(gRobot.walk_t.circlechange.direction==0)
			   {
					 if(fabs(2400+gRobot.walk_t.pos.x-getLeftAdc())>500)
					 {
						 return NO;
					 }else{
						return YES;
				   }
			   }else if(gRobot.walk_t.circlechange.direction==1)
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
						if(gRobot.walk_t.circlechange.direction==0)
			   {
					 if(fabs(gRobot.walk_t.pos.y-getLeftAdc())>500)
					 {
						 return NO;
					 }else{
						return YES;
				   }
			   }else if(gRobot.walk_t.circlechange.direction==1)
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
	if(fabs(angleErrorCount(gRobot.avoid_t.pid.aimAngle,gRobot.walk_t.pos.angle)) >100)
	{
		if(gRobot.walk_t.circlechange.turntime<=4 && gRobot.walk_t.circlechange.direction==0) 
		{	
			USART_OUT(UART5,(uint8_t*)"ssss");
			VelCrl(CAN2, 1, -1000);     //顺时针
			VelCrl(CAN2, 2, 10000);
		}else if(gRobot.walk_t.circlechange.turntime>4 && gRobot.walk_t.circlechange.direction==0)
		{
			USART_OUT(UART5,(uint8_t*)"aaaaaa");
			VelCrl(CAN2, 1,-10000);
			VelCrl(CAN2, 2,1000);
		}else if(gRobot.walk_t.circlechange.turntime<=4 && gRobot.walk_t.circlechange.direction==1)
		{	
      USART_OUT(UART5,(uint8_t*)"bbbbb");
			VelCrl(CAN2, 1, -10000);    //逆时针
			VelCrl(CAN2, 2, 1000);
		}else if(gRobot.walk_t.circlechange.turntime>4 && gRobot.walk_t.circlechange.direction==1)
		{
			USART_OUT(UART5,(uint8_t*)"cccc");
			VelCrl(CAN2, 1,-1000);
			VelCrl(CAN2, 2,10000);
		}
  }else if(fabs(angleErrorCount(gRobot.avoid_t.pid.aimAngle,gRobot.walk_t.pos.angle))<100)
	{
		if(gRobot.walk_t.circlechange.turntime<=4 && gRobot.walk_t.circlechange.direction==0) 
		{
      USART_OUT(UART5,(uint8_t*)"ddddd");			
			VelCrl(CAN2, 1, 10000);     //顺时针
			VelCrl(CAN2, 2, 0);
		}else if(gRobot.walk_t.circlechange.turntime>4 && gRobot.walk_t.circlechange.direction==0)
		{
			USART_OUT(UART5,(uint8_t*)"eeeeee");
			VelCrl(CAN2, 1,0);
			VelCrl(CAN2, 2,-10000);
		}else if(gRobot.walk_t.circlechange.turntime<=4 && gRobot.walk_t.circlechange.direction==1)
		{		
			USART_OUT(UART5,(uint8_t*)"ffff");
			VelCrl(CAN2, 1, 0);      //逆时针
			VelCrl(CAN2, 2, -10000);
		}else if(gRobot.walk_t.circlechange.turntime>4 && gRobot.walk_t.circlechange.direction==1)
		{
			USART_OUT(UART5,(uint8_t*)"gggggg");
			VelCrl(CAN2, 1,10000);
			VelCrl(CAN2, 2,0);
		}
	}
 //判断是否满足条件
	//时间条件
	gRobot.avoid_t.pid.pidtime++;
	if(gRobot.avoid_t.pid.pidtime>400)
	{
		gRobot.avoid_t.pid.pidtime=0;
		return 1;
	}
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
void CheckOutline2(void)
{
	static int count=0;
	static float vx=0;
	static float vy=0;
	static float aimv=0.0f;
	static float lastx=0;
	static float lasty=0;
	static int stickError=0;
	statueRemember=gRobot.status;
	count++;
if(count==5){
	count=0;
	vx=(gRobot.walk_t.pos.x-lastx)/5.0f*100.0f;
	vy=(gRobot.walk_t.pos.y-lasty)/5.0f*100.0f;
	
	aimv=Pulse2Vel(gRobot.walk_t.right.base+gRobot.walk_t.left.base)*0.5f;
	gRobot.walk_t.displacement.averageV=__sqrtf(vx*vx+vy*vy);
	if(TwoNumCompare(gRobot.walk_t.displacement.averageV,aimv*0.2f))
	{
		stickError++;
	}
	else
	{
		stickError=0;
	}
	if(stickError==10)
	{
		stickError=0;
		xStick = getxRem();                  //记住卡死的坐标
		yStick = getyRem();
		//改变状态码
		gRobot.status=32;                    //进入backcar
		gRobot.avoid_t.signal=0;             //清零
	}
lastx=gRobot.walk_t.pos.x;
lasty=gRobot.walk_t.pos.y;
USART_OUT(UART5,(uint8_t*)"%d\t",(int)vx);
USART_OUT(UART5,(uint8_t*)"%d\t",(int)vy);
USART_OUT(UART5,(uint8_t*)"%d\t",(int)aimv);
USART_OUT(UART5,(uint8_t*)"%d\t",(int)__sqrtf(vx*vx+vy*vy));
USART_OUT(UART5,(uint8_t*)"%d\t",(int)gRobot.walk_t.pos.x);
USART_OUT(UART5,(uint8_t*)"%d\t",(int)gRobot.walk_t.pos.y);	
USART_OUT(UART5,(uint8_t*)"%d\t",(int)stickError);	
USART_OUT(UART5,(uint8_t*)"%d\t",(int)gRobot.status);	
USART_OUT(UART5,(uint8_t*)"%d\r\n",(int)gRobot.walk_t.circlechange.turntime);
	
}
}
/********************* (C) COPYRIGHT NEU_ACTION_2017 ****************END OF FILE************************/
