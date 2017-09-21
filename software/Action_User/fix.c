#include "config.h"
extern Robot_t gRobot;

static float angle=0,xpos=0,ypos=0;
static float errSingle=0;                //errSingle = realSingle - nowSingle
static float errX0=0;
static float errY0=0;

void setAngle(float val)
{
	angle=val;
}
void setXpos(float val)
{ 
	xpos=val;
}
void setYpos(float val)
{
	ypos=val;
}

/****************************************************************************
* 名    称：setErrSingle() setErrX() setErrY()
* 功    能：得到误差
* 入口参数：无
* 出口参数：激光与真实值的误差
* 说    明：无
* 调用方法：无 
****************************************************************************/
void setErrSingle(float reaAngle)
{
	errSingle = reaAngle - angle;
}
void setErrX(float realX)
{
	float temp;
	temp = xpos * cos(-errSingle * PI / 180) - ypos * sin(-errSingle * PI / 180);
	errX0 = realX - temp;
}
void setErrY(float realy)
{
	float temp;
	temp = xpos * sin(-errSingle * PI / 180) + ypos * cos(-errSingle * PI / 180);
	errY0 = realy - temp;
}
void setErr(float reaAngle,float realX,float realy)
{
	setErrSingle(reaAngle);
	setErrX(realX);
	setErrY(realy);
}
/****************************************************************************
* 名    称：getAngle() getXpos() getYpos()
* 功    能：得到正确的姿态
* 入口参数：无
* 出口参数：正确的姿态值
* 说    明：无
* 调用方法：无 
****************************************************************************/
float getAngle(void)
{
	if (angle + errSingle > 180)
		return angle + errSingle - 360;
	else if (angle + errSingle < -180)
		return angle + errSingle + 360;
	else
		return angle + errSingle;
}
float getXpos(void)
{
	return xpos * cos(-errSingle * PI / 180) - ypos * sin(-errSingle * PI / 180) + errX0;
}
float getYpos(void)
{
	return xpos * sin(-errSingle * PI / 180) + ypos * cos(-errSingle * PI / 180) + errY0;
}
/****************************************************************************
* 名    称：getLeftAdc()	getRightAdc()
* 功    能：减小激光误差
* 入口参数：无
* 出口参数：拟合后的激光值
* 说    明：无
* 调用方法：无 
****************************************************************************/
int getLeftAdc()
{
	return 0.9389*Get_Adc_Average(ADC_Channel_15, 200)+428.6575;
}
int getRightAdc()
{
	return 0.9403*Get_Adc_Average(ADC_Channel_14, 200)+435.445;
}

/****************************************************************************
* 名    称：getAimBorder()	
* 功    能：返回距离最小的边界，获取最佳的停车地点
* 入口参数：无
* 出口参数： 0 Left 1 Right 2 Up 3 Down
* 说    明：无
* 调用方法：无 
****************************************************************************/
static int map[4] = {0};
int getAimBorder(void) 
{
	static int i = 0;
	int min_num = 0;
	static int temp[4];
	static int min_dis = 0;

	//
	float x = gRobot.walk_t.pos.x;
	float y = gRobot.walk_t.pos.y;
	// 计算到四个边界的距离
	temp[0] = 2400 + x; //Left
	temp[1] = 2400 - x; //Right
	temp[2] = 4800 - y; //Up
	temp[3] = y;		    //Down
	min_dis = temp[0];
	for (i = 1; i < 4; i++)
	{
		if (min_dis > temp[i] && !map[i])
		{
			min_dis = temp[i];
			min_num = i;
		}
	}
	map[min_num] = 1;//消除靠墙那个边的比较
	return min_num;
}
#define LEFT_BORDER 0  //左边界
#define RIGHT_BORDER 1 //右边界
#define UP_BORDER 2	   //上边界
#define DOWN_BORDER 3  //下边界
//需要进行边界修改
//#define Y_MIN (320 + 200)
//#define X_MAX (2400 - 320 - 200 - 200)
//#define X_MIN (-2400 + 320 + 200)
//#define Y_MAX (4800 - 320 - 200)


/****************************************************************************
* 名    称：getFixAngle()	
* 功    能：得到修正角度
* 入口参数：无
* 出口参数：无
* 说    明：无
* 调用方法：无 
****************************************************************************/
static int fixSuccessFlag = 0; //修正成功标志位//到时把这个变量放入结构体当中
FixPara_t getFixPara(int aimBorder)
{
	FixPara_t fixPara={0};
	switch (aimBorder)
	{
		case LEFT_BORDER:
			fixPara.angle=-90;
			fixPara.spacingError=getXpos()+2400;
		break;
		
		case RIGHT_BORDER:
			fixPara.angle=90;
			fixPara.spacingError=2400-getXpos();
		break;
		
		case UP_BORDER:
			fixPara.angle=180;
			fixPara.spacingError=4800-getYpos();
		break;
		
		case DOWN_BORDER:
			fixPara.angle=0;
			fixPara.spacingError=getYpos();
		break;
	}
	return fixPara;
}

/**
	矫正的规划
	1.获取目标角度
	2.试图第一次矫正 
		旋转 靠墙 判断开关触发
		如果开关不能同时触发 进行下一次矫正
		如果开关触发 但是 左右激光距离误差过大 那么记录这次坐标 进行下次矫正 
		如果可以矫正 直接矫正坐标	
*
*/
static int fix_status=11;//需要矫正时赋值为11//矫正开始时赋值为11

#define WAIT_AIM_DIRECTION 1
#define TRY_FIRST_FIX 2
#define TRY_SEC_FIX 4
#define AGAINST_Wall 8
/****************************************************************************
* 名    称：CommitFix()	
* 功    能：
* 入口参数：无
* 出口参数：无
* 说    明：无
* 调用方法：无 
****************************************************************************/
int CommitFix(int laserLeftDistance,int laserRightDistance)//确定是否能进行修正激光被挡或者不在激光处理范围内
{
	static int commitFix=0;//靠下一面墙的标志位
	if(laserLeftDistance>4096+MOVEBASE_WIDTH/2||laserLeftDistance<40+MOVEBASE_WIDTH/2)
	{
		commitFix=0;//说明要靠下一面墙
	}
	else
	{
		commitFix=1;	//说明能够进行矫正
	}
	
	if(laserRightDistance>4096+MOVEBASE_WIDTH/2||laserRightDistance<40+MOVEBASE_WIDTH/2)
	{
		commitFix=0;//说明要靠下一面墙
	}
	else
	{
		commitFix=1;	//说明能够进行矫正
	}
	
//	if(laserLeftDistance+laserRightDistance<4800-50)
//	{
//		commitFix=0;//说明要靠下一面墙
//	}
//	else
//	{
//		commitFix=1;	//说明能够进行矫正
//	} 
//	USART_OUT(UART5, (uint8_t *)"%d\t\r\n", commitFix);
	return commitFix;
}
/****************************************************************************
* 名    称：fixPosFirst()	
* 功    能：
* 入口参数：无
* 出口参数：无
* 说    明：无
* 调用方法：无 
****************************************************************************/
void fixPosFirst(int aimBorder)
{
	static float x=0,y=0;
	x=gRobot.walk_t.pos.x;//定位系统返回的错误坐标
	y=gRobot.walk_t.pos.y;
	switch(aimBorder)
	{
		case LEFT_BORDER :
			if(fabs(getRightAdc()-y) <= fabs(4800-getLeftAdc()-y))
			{
				setErr(-90,X_MIN,getRightAdc());
			}
			else 
			{
				setErr(-90,X_MIN,4800-getLeftAdc());
			}
		break;
		
		case RIGHT_BORDER:
			if(fabs(getLeftAdc()-y) <= fabs(4800-getRightAdc()-y))
			{
				setErr(90,X_MAX,getLeftAdc());
			}else 
			{
				setErr(90,X_MAX,4800-getRightAdc());
			}
		break;
		
		case UP_BORDER:
			if(fabs((getRightAdc()-2400)-x) <= fabs((2400-getLeftAdc())-x))
			{
				setErr(180,getRightAdc()-2400,Y_MAX);
			}else 
			{
				setErr(180,2400-getLeftAdc(),Y_MAX);
			}
		break;
		
		case DOWN_BORDER :
			if(fabs((getLeftAdc()-2400)-x) <= fabs((2400-getRightAdc())-x))
			{
				setErr(0,getLeftAdc()-2400,Y_MIN);
			}
			else
			{
				setErr(0,2400-getRightAdc(),Y_MIN);
			}
		break;
	}
	USART_OUT(UART5, (uint8_t *)"%s\t\r\n", "hahahahahha1");
}
/****************************************************************************
* 名    称：fixPosSec()	
* 功    能：
* 入口参数：无
* 出口参数：无
* 说    明：无
* 调用方法：无 
****************************************************************************/
void fixPosSec(int aimBorder)//矫正当前墙的坐标
{
	static FixPara_t aimFixPara={0};
	aimFixPara=getFixPara(aimBorder);
	setErrSingle(aimFixPara.angle); //修正角度
	switch (aimBorder)
	{
		case LEFT_BORDER:
			setErrX(X_MIN);
		break;

		case RIGHT_BORDER:
			setErrX(X_MAX);
		break;

		case UP_BORDER:
			setErrY(Y_MAX);
		break;

		case DOWN_BORDER:
			setErrY(Y_MIN);
		break;
	}
	  USART_OUT(UART5, (uint8_t *)"%s\t\r\n", "hahahahahha4");
		USART_OUT(UART5, (uint8_t *)"%s\t\r\n", "hahahahahha4");
		USART_OUT(UART5, (uint8_t *)"%s\t\r\n", "hahahahahha4");
		USART_OUT(UART5, (uint8_t *)"%s\t\r\n", "hahahahahha4");
}
/****************************************************************************
* 名    称：Go2NextWall()	
* 功    能：靠下一面墙
* 入口参数：无
* 出口参数：无
* 说    明：无
* 调用方法：无 
****************************************************************************/
AimPos_t Go2NextWall(int aimBorder)//第一次矫正失败后到下一面墙的目标点
{
	AimPos_t aimPos;
	switch (aimBorder)
	{
		case LEFT_BORDER:
			aimPos.x=-1800;							//第二次靠墙判断最小距离墙面之后代入定点停车
			aimPos.y=2400;
		break;

		case RIGHT_BORDER:
			aimPos.x=1800;
			aimPos.y=2400;
		break;

		case UP_BORDER:
			aimPos.x=0;
			aimPos.y=4200;
		break;

		case DOWN_BORDER:
			aimPos.x=0;
			aimPos.y=600;
		break;
	}
	return aimPos;
}
/****************************************************************************
* 名    称：FixTask()	
* 功    能：坐标矫正
* 入口参数：无
* 出口参数：无
* 说    明：无
* 调用方法：无 
****************************************************************************/
void FixTask(void)
{
	//修正状态
	static int againstTime=0;																	//靠墙的次数
	static int aimBorder=0;																		//目标边界
	FixPara_t fixPara={0};																	//矫正角度和距离墙的距离
	int laserLeftDistance=getLeftAdc();												//左边激光
	int laserRightDistance=getRightAdc();											//右边激光
	static int commitFix=0;
	AimPos_t aimPos;																					//二次矫正的停车位
	
	ShootCtr(60);
	
	/***/
	gRobot.avoid_t.signal=0;																	//关闭checkoutline()
	commitFix=CommitFix(laserLeftDistance,laserRightDistance);//判断能否进行矫正
	
	
	if (fix_status & WAIT_AIM_DIRECTION)											//1011 & 0001 
	{
		aimBorder = getAimBorder();
		fix_status &= ~WAIT_AIM_DIRECTION;											//1011 & 1110 将此位滞空=1010
//		fixPara=getFixPara(aimBorder);												//矫正角度也是当前靠墙的角度
	} 
	else if (fix_status & TRY_FIRST_FIX)											//第一次矫正 1010 & 0010
	{
		if ((fix_status & AGAINST_Wall))												//靠墙 1010 & 1000
		{
			fixPara=getFixPara(aimBorder);											//矫正角度也是当前靠墙的角度
			AgainstWall(fixPara.angle,gRobot.walk_t.pos.angle,fixPara.spacingError);
			if (CheckAgainstWall())																//检查靠墙
			{
					VelCrl(CAN2, 1, 0);
					VelCrl(CAN2, 2, 0);
//				USART_OUT(UART5, (uint8_t *)"%s\t\r\n", "hahahahahha5");
//				USART_OUT(UART5, (uint8_t *)"%s\t\r\n", "hahahahahha5");
//				USART_OUT(UART5, (uint8_t *)"%s\t\r\n", "hahahahahha5");
//				USART_OUT(UART5, (uint8_t *)"%s\t\r\n", "hahahahahha5");
				if(commitFix)//当激光没问题
				{
					fixPosFirst(aimBorder);
					fixSuccessFlag=1;
						fix_status=0;																		//在这里改变状态码
				
				}
				else 																								//第二次矫正
				{
					fixPosSec(aimBorder);															//矫正当前的角度
					againstTime++;
					fixSuccessFlag=0;
					if(againstTime>=2)
					{
						fixSuccessFlag = 1;
						againstTime=0;
					}
					//状态码修改
					fix_status = 0;
					fix_status |= WAIT_AIM_DIRECTION;
					fix_status |= TRY_SEC_FIX;
				}
			}/*else if()*///靠墙失败
		}
	}
	else if (fix_status & TRY_SEC_FIX)
	{
		aimPos=Go2NextWall(aimBorder);
		if(Pointparking(aimPos.x,aimPos.y)==1)//停车完成
		{
			fix_status=0;
//			fix_status &=~TRY_SEC_FIX;
			fix_status |=TRY_FIRST_FIX;
			fix_status |=AGAINST_Wall;
		}
	}
	
	if(fixSuccessFlag==1)
	{
		gRobot.status&=~STATUS_FIX;
		fix_status=11;
		fixSuccessFlag= 0;
		againstTime=0;
		map[0]=0;
		map[1]=0;
		map[2]=0;
		map[3]=0;//之前在判断最小距离墙面的时候将原来靠上的那面墙排除比较，现在恢复让其重新比较
	}
	
	
//		USART_OUT(UART5, (uint8_t *)"%d\t", (int)fixSuccessFlag);
//		USART_OUT(UART5, (uint8_t *)"%d\t", (int)fix_status);
//		USART_OUT(UART5, (uint8_t *)"%d\t", (int)commitFix);
//		USART_OUT(UART5, (uint8_t *)"%d\t", (int)aimBorder);
//		USART_OUT(UART5, (uint8_t *)"%d\t", (int)fixAngle);
//		USART_OUT(UART5, (uint8_t *)"%d\t", (int)errSingle); //errSingle = realSingle - nowSingle
//		USART_OUT(UART5, (uint8_t *)"%d\t", (int)errX0);
//		USART_OUT(UART5, (uint8_t *)"%d\t", (int)errY0);
//		USART_OUT(UART5, (uint8_t *)"%d\t", (int)gRobot.walk_t.pos.x);
//		USART_OUT(UART5, (uint8_t *)"%d\t", (int)gRobot.walk_t.pos.y);
//		USART_OUT(UART5, (uint8_t *)"%d\t", (int)getxRem());
//		USART_OUT(UART5, (uint8_t *)"%d\t", (int)getyRem());
//		USART_OUT(UART5, (uint8_t *)"%d\t", (int)gRobot.walk_t.pos.angle);
//		USART_OUT(UART5, (uint8_t *)"%d\t", (int)againstTime);
//		USART_OUT(UART5, (uint8_t *)"%d\t", (int)laserLeftDistance);
//		USART_OUT(UART5, (uint8_t *)"%d\t", (int)laserRightDistance);
//		USART_OUT(UART5, (uint8_t *)"%d\t\r\n", (int)gRobot.turnTime);
	
		
}


