/**
  ******************************************************************************
  * @file	  moveBase.c
  * @author	  Action
  * @version   V1.0.0
  * @date	  2017/07/24
  * @brief	 2017省赛摄像头处理部分
  ******************************************************************************
  * @attention
  *			None
  ******************************************************************************/
#include "config.h"

extern Robot_t gRobot;
static int turnTimeChange = 0;//记住拐弯的次数
static int circleChangeSymbol=1;//切换圈的标志
extern Robot_t gRobot;

/****************************************************************************
* 名    称：CameraBaseWalk3(void)
* 功    能：摄像头基础走形
* 入口参数：无
* 出口参数：无
* 说    明：无
* 调用方法：无 
****************************************************************************/
void CameraBaseWalk3(void)										//摄像头基础走形
{
	static int M=12214;
	static float x = 0, y = 0, angle = 0;
	static float aimAngle = 0;   								//目标角度
	static float angleError = 0; 								//目标角度与当前角度的偏差
	static float disError = 0;   								//距离偏差
	x = gRobot.walk_t.pos.x;														//矫正过的x坐标
	y = gRobot.walk_t.pos.y;														//矫正过的y坐标
	angle = gRobot.walk_t.pos.angle; 									//矫正过的角度角度
	AreaCheck(x,y);
	if(gRobot.walk_t.right.real>2000)
	{
		gRobot.avoid_t.signal=1;
	}
  switch (gRobot.camera_t.camrBaseWalk_t.turnTime)
	{
	case 0:
			//初始值50//小车距离与直线的偏差//不加绝对值是因为判断车在直线上还是直线下
			disError = y - (1100 - circleChangeSymbol*500); 
			aimAngle = -90+Findball_5();
			angleError = angleErrorCount(aimAngle,angle);
		
			VelCrl(CAN2, 1, M + AnglePidControl(angleError - onceDistancePidControl(disError))); //角度误差pid和距离误差相结合
			VelCrl(CAN2, 2, -M + AnglePidControl(angleError - onceDistancePidControl(disError)));
			gRobot.walk_t.right.aim =M + AnglePidControl(angleError - onceDistancePidControl(disError));
//			piddisShuchu = distancePidControl(disError);
		
		break;

		case 1:
			disError = x-(850+circleChangeSymbol*1150);
			aimAngle=0+Findball_5();
			angleError=angleErrorCount(aimAngle,angle);
			VelCrl(CAN2, 1, M + AnglePidControl(angleError + onceDistancePidControl(disError))); //pid中填入的是差值
			VelCrl(CAN2, 2, -M + AnglePidControl(angleError + onceDistancePidControl(disError)));
			gRobot.walk_t.right.aim =M+AnglePidControl(angleError + onceDistancePidControl(disError));
//			piddisShuchu = distancePidControl(disError);
		break;
				
		case 2:
			//小车距离与直线的偏差//不加绝对值是因为判断车在直线上还是直线下//4100
			disError = y - (3700 + circleChangeSymbol*500); 
			aimAngle = 90+Findball_5();
			angleError = angleErrorCount(aimAngle,angle);
			VelCrl(CAN2, 1, M + AnglePidControl(angleError + onceDistancePidControl(disError))); //pid中填入的是差值
			VelCrl(CAN2, 2, -M + AnglePidControl(angleError + onceDistancePidControl(disError)));
			gRobot.walk_t.right.aim =M+ AnglePidControl(angleError + distancePidControl(disError));
//			piddisShuchu = distancePidControl(disError);
		break;

		case 3:
			//小车距离与直线的偏差//不加绝对值是因为判断车在直线上还是直线下
			disError = x + (850 + circleChangeSymbol*1150); 
			aimAngle = 180+Findball_5();
			angleError = angleErrorCount(aimAngle,angle);
			VelCrl(CAN2, 1, M + AnglePidControl(angleError - onceDistancePidControl(disError))); //pid中填入的是差值
			VelCrl(CAN2, 2, -M + AnglePidControl(angleError - onceDistancePidControl(disError)));
			gRobot.walk_t.right.aim =M+AnglePidControl(angleError - distancePidControl(disError));
//			piddisShuchu = distancePidControl(disError);
		break;
			
		default:
		break;
	}

		USART_OUT(UART5, (uint8_t *)"%d\t", (int)gRobot.walk_t.pos.x);
		USART_OUT(UART5, (uint8_t *)"%d\t", (int)gRobot.walk_t.pos.y);
		USART_OUT(UART5, (uint8_t *)"%d\t", (int)turnTimeChange);
		USART_OUT(UART5, (uint8_t *)"%d\t", (int)circleChangeSymbol);
		USART_OUT(UART5, (uint8_t *)"%d\t", (int)angle);//gRobot.walk_t.pos.angle
		USART_OUT(UART5, (uint8_t *)"%d\t", (int)angleError);
		USART_OUT(UART5, (uint8_t *)"%d\t", (int)disError);
		USART_OUT(UART5, (uint8_t *)"%d\t", (int)gRobot.walk_t.right.real);
		USART_OUT(UART5, (uint8_t *)"%d\t", (int)gRobot.walk_t.right.aim);
//		USART_OUT(UART5, (uint8_t *)"%d\t", (int)piddisShuchu);
//		USART_OUT(UART5, (uint8_t *)"%d\t", (int)pidZongShuchu);
		USART_OUT(UART5, (uint8_t *)"%d\t\r\n", (int)gRobot.camera_t.camrBaseWalk_t.turnTime);
}
/****************************************************************************
* 名    称：AreaCheck()
* 功    能：区域检查函数
* 入口参数：无
* 出口参数：无
* 说    明：无
* 调用方法：无 
****************************************************************************/
void AreaCheck(float x, float y)//全区域检查函数
{
//	static int turnTime = 0;//定义转弯直线
	static int turnTimeRem = 0;//当turnTime改变时通过Rem来使车知道它转弯了
	static int temp1=0,temp2=0;//temp1=300 提前量267 900 提前460 1800 
	static int changeFlag=0;//切换出摄像头状态又回来的时候给一个标志位
	static int circleNum=0;
	
	if (circleChangeSymbol == 0)
	{
		temp1 = 2000;//1870
		temp2 = 1450;//1450
	}
	else if (circleChangeSymbol == 1)
	{
		temp1 = 500;//500
		temp2 = 900;//900
	}

	//poitChange,用于改变点的值
	//区域一
	float peakX_Area1[5] = { 250 ,-250,-2400,-2400,2400 - temp1 };
	float peakY_Area1[5] = { 1700,1700,0 + temp2,0,0 };
	//区域二
	float peakX_Area2[5] = { 250 ,250 ,2400 - temp1,2400,2400 };
	float peakY_Area2[5] = { 3100,1700,0,0,4800 - temp2 };
	//区域三
	float peakX_Area3[5] = { -250,250 ,2400,2400,-2400 + temp1 };
	float peakY_Area3[5] = { 3100,3100,4800 - temp2,4800,4800 };
	////区域四
	float peakX_Area4[5] = { -250 ,-250,-2400 + temp1,-2400,-2400 };
	float peakY_Area4[5] = { 1700 ,3100,4800,4800,temp2 };
	
	
	turnTimeRem = gRobot.camera_t.camrBaseWalk_t.turnTime;
//检查在哪一个区域走相应的直线
	if (CheckIn(x, y, 5, peakX_Area1, peakY_Area1) == 1)
	{
		gRobot.camera_t.camrBaseWalk_t.turnTime = 0;
	}
	else if (CheckIn(x, y, 5, peakX_Area2, peakY_Area2) == 1)
	{
		gRobot.camera_t.camrBaseWalk_t.turnTime = 1;
	}
	else if (CheckIn(x, y, 5, peakX_Area3, peakY_Area3) == 1)
	{
		gRobot.camera_t.camrBaseWalk_t.turnTime = 2;
	}
	else if (CheckIn(x, y, 5, peakX_Area4, peakY_Area4) == 1)
	{
		gRobot.camera_t.camrBaseWalk_t.turnTime = 3;
	}

	if (gRobot.camera_t.camrBaseWalk_t.turnTime != turnTimeRem)
	{
		turnTimeChange++;//记住拐弯的次数
//		turnTimeChange = turnTimeChange % 6;//不让他超过5
		//SetTurnTimeChange(turnTimeChange);
	} 
   
	if(changeFlag==1)
	{
		if ((x > -1400 && x < 1400) && (y > 1400 && y < 3900))//内环
		{
			circleChangeSymbol = 0;
		}
		else if ((x < -1400 || x>1400) || (y < 1400 || y>3900))//外环
		{
			circleChangeSymbol = 1;
		}
	}
	
	if(turnTimeChange>=5)//转了5次弯道后
	{
		circleChangeSymbol=!circleChangeSymbol;
		turnTimeChange=1;
		circleNum++;
	}
	
	if(circleNum>=2)
	{
		circleNum=0;
		turnTimeChange=0;
		gRobot.status=24;
	}
//	if(getF_ball()!=0)//有球清空转弯次数
//	{
//		turnTimeChange=0;	
//	}
}
/****************************************************************************
* 名    称：CheckIn(float x, float y, int pointNum, float * peakX, float  * peakY)
* 功    能：单个区域检查函数
* 入口参数：无
* 出口参数：无
* 说    明：无
* 调用方法：无 
****************************************************************************/
int CheckIn(float x, float y, int pointNum, float * peakX, float  * peakY)
{
	int c = 0;//定义布尔值
	for (int i = 0, j = pointNum - 1; i < pointNum; j = i++)
	{
		if (((y<peakY[i]) != (y<peakY[j])) && (x<(y - peakY[i]) * (peakX[j] - peakX[i]) / (peakY[j] - peakY[i]) + peakX[i]))
		{
			c = !c;
		}
	}
	return c;
}
/****************************************************************************
* 名    称：Sub_Box()
* 功    能：摄像头的数据处理
* 入口参数：无
* 出口参数：无
* 说    明：无
* 调用方法：无 
****************************************************************************/

extern int Ball_counter;
void Sub_Box(void)
{
	USART_OUT(UART5,(uint8_t*)"getf:%d\r\n",getF_ball());
if(getF_ball())
	{
		int8_t leftAngLimit = -25;
		int8_t rightAngLimit = 4;
	
		int8_t maxFirstlayer = 0;
		int8_t max_Firstlayernum = 0;
	  float c_Aimxfirst=0.0f,c_Aimyfirst=72.75f;		
		float c_Firstbestangle=0;
		float Aimxfirst=0,Aimyfirst=0;
	
		int8_t maxSecondlayer =0;
		int8_t max_Secondlayernum=0;
		float c_Aimxsecond=0,c_Aimysecond=140.25;
		float c_Secondbestangle=0;
		float Aimxsecond=0,Aimysecond=0;
//第一层
		for (int i = 0; i < 4; i++)
		{
			int8_t counter1 = 0;
			for (int j = 0; j < Ball_counter; j++)
			{
				if(gRobot.camera_t.camrCatch_t.camrPara[j].dis<=106.5)
					{
						
						if (gRobot.camera_t.camrCatch_t.camrPara[j].angle < rightAngLimit && gRobot.camera_t.camrCatch_t.camrPara[j].angle > leftAngLimit)
							counter1++;
					}
			}
			if (counter1 > max_Firstlayernum)
			{
				max_Firstlayernum = counter1;
				maxFirstlayer = i;
			}
			rightAngLimit+=7;
			leftAngLimit+=7;
		}	
//第二层
		leftAngLimit = -25;
		rightAngLimit=-11;
		for (int i = 0; i < 8; i++)
		{
			int8_t counter2=0;
			for (int j = 0; j < Ball_counter; j++)
			{
				 if(gRobot.camera_t.camrCatch_t.camrPara[j].dis>106.5)
					{
						if (gRobot.camera_t.camrCatch_t.camrPara[j].angle < rightAngLimit && gRobot.camera_t.camrCatch_t.camrPara[j].angle > leftAngLimit)
							counter2++;
					}
			}
			if(counter2>max_Secondlayernum)
			{
				max_Secondlayernum=counter2;
				maxSecondlayer=i;
			}
			rightAngLimit+=5;
			leftAngLimit+=5;
		}	
//第一层球最多的坐标
		c_Aimxfirst =(float) 72.75 / tanf((65+29/2 +7* maxFirstlayer)*PI / 180.f);
		c_Firstbestangle = -25.f + 7*maxFirstlayer+14.5f;
		Aimxfirst = getXpos() - 10*Dis(c_Aimxfirst, c_Aimyfirst, 0, 0)*sinf((getAngle() + c_Firstbestangle)*PI / 180) - 220 * sin(getAngle()*PI / 180);
	  Aimyfirst = getYpos() + 10*Dis(c_Aimxfirst, c_Aimyfirst, 0, 0)*cosf((getAngle() + c_Firstbestangle)*PI / 180) + 220 * cos(getAngle()*PI / 180);
		setAimxfirst(Aimxfirst);
		setAimyfirst(Aimyfirst);
//第二层球最多的坐标
		c_Aimxsecond=(float)140.25/tanf((65+14/2+5*maxSecondlayer)*PI/180.f);
		c_Secondbestangle=-25+5*maxSecondlayer+7;
		Aimxsecond = getXpos() -10* Dis(c_Aimxsecond, c_Aimysecond, 0, 0)*sinf((getAngle() + c_Secondbestangle)*PI / 180) - 220 * sin(getAngle()*PI / 180);
	  Aimysecond = getYpos() +10* Dis(c_Aimxsecond, c_Aimysecond, 0, 0)*cosf((getAngle() + c_Secondbestangle)*PI / 180) + 220 * cos(getAngle()*PI / 180);
		setAimxsecond(Aimxsecond);
		setAimysecond(Aimysecond);
		if (maxFirstlayer == 0)
			{
				Aimxfirst = 0;
			}
		if (maxSecondlayer == 0)
			{
				Aimxsecond = 0;
			}
	d_Sub_Box(Ball_counter,maxFirstlayer,c_Aimxfirst, Aimxfirst,Aimyfirst,c_Aimxsecond,Aimxsecond,Aimysecond,maxSecondlayer);
				Ball_counter=0;
		}
	}
//void C_coorchange(int Ball_counter)
//{
//	Point_t Cardpoint[Ball_counter];
//	for(int i=0;i<Ball_counter;i++)
//	{
//		Cardpoint[i].x=gRobot.walk_t.pos.x-10* gRobot.camera_t.camrCatch_t.camrPara[i].dis*sinf((gRobot.walk_t.pos.angle + gRobot.camera_t.camrCatch_t.camrPara[i].angle)*PI / 180) - 220 * sin(gRobot.walk_t.pos.angle*PI / 180);
//		Cardpoint[i].y=gRobot.walk_t.pos.y+10* gRobot.camera_t.camrCatch_t.camrPara[i].dis*cosf((gRobot.walk_t.pos.angle + gRobot.camera_t.camrCatch_t.camrPara[i].angle)*PI / 180) + 220 * cos(gRobot.walk_t.pos.angle*PI / 180);
//	}
//}
//void C_route()
//{
//	
//}

