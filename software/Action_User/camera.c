#include "camera.h"
#include "stm32f4xx.h"
#include "arm_math.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_rcc.h"
#include "timer.h"
#include "gpio.h"
#include "usart.h"    
#include "can.h"
#include "elmo.h"
#include "stm32f4xx_it.h"
#include "stm32f4xx_usart.h"
#include "stm32f4xx_adc.h"
#include "fix.h"
#include "sweep.h"
#include "task.h"
#include "tools.h"
extern Robot_t gRobot;
static int turnTimeChange = 0;//记住拐弯的次数
static int circleChangeSymbol = 0;
int CheckIn(float x, float y, int pointNum, float * peakX, float  * peakY);
int AreaCheck(float x, float y);
void SetTurnTimeChange(int temp);
int GetTurnTimeChange(void);
void CameraBaseWalk3(void)//摄像头基础走形
{
	static int M=0;
	static float x = 0, y = 0, angle = 0;
	static float aimAngle = 0;   //目标角度
	static float angleError = 0; //目标角度与当前角度的偏差
	static int circleChangeSymbol=0;//相当于缩圈变量，lineChangeSymbol
	static float disError = 0;   //距离偏差
	static float pidZongShuchu = 0, piddisShuchu = 0;
	static int turnChangeTimes=0;//记住拐弯的次数
	static int turnTime=0;
	x = gRobot.pos.x;			//矫正过的x坐标
	y = gRobot.pos.y;			//矫正过的y坐标
	angle = gRobot.pos.angle; //矫正过的角度角度
	M=12214;
	turnTime=AreaCheck(x,y);
  switch (turnTime)
	{
		case 0:
			disError = y - (1100 - circleChangeSymbol*500); //初始值50//小车距离与直线的偏差//不加绝对值是因为判断车在直线上还是直线下
			aimAngle = -90;
			angleError = angleErrorCount(aimAngle,angle);
		
			VelCrl(CAN2, 1, M + AnglePidControl(angleError - distancePidControl(disError))); //角度误差pid和距离误差相结合
			VelCrl(CAN2, 2, -M + AnglePidControl(angleError - distancePidControl(disError)));
			CheckOutline();
			pidZongShuchu = AnglePidControl(angleError - distancePidControl(disError));
			piddisShuchu = distancePidControl(disError);
		break;

		case 1:
			disError = x-(850+circleChangeSymbol*950);
			aimAngle=0;
			angleError=angleErrorCount(aimAngle,angle);;
			VelCrl(CAN2, 1, M + AnglePidControl(angleError + distancePidControl(disError))); //pid中填入的是差值
			VelCrl(CAN2, 2, -M + AnglePidControl(angleError + distancePidControl(disError)));
			CheckOutline();
			pidZongShuchu = AnglePidControl(angleError + distancePidControl(disError));
			piddisShuchu = distancePidControl(disError);
		break;
				
		case 2:
			disError = y - (3700 + circleChangeSymbol*500); //小车距离与直线的偏差//不加绝对值是因为判断车在直线上还是直线下//4100
			aimAngle = 90;
			angleError = angleErrorCount(aimAngle,angle);
			VelCrl(CAN2, 1, M + AnglePidControl(angleError + distancePidControl(disError))); //pid中填入的是差值
			VelCrl(CAN2, 2, -M + AnglePidControl(angleError + distancePidControl(disError)));
			CheckOutline();
			pidZongShuchu = AnglePidControl(angleError + distancePidControl(disError));
			piddisShuchu = distancePidControl(disError);
		break;

		case 3:
			disError = x + (850 + circleChangeSymbol*950); //小车距离与直线的偏差//不加绝对值是因为判断车在直线上还是直线下
			aimAngle = 180;
			angleError = angleErrorCount(aimAngle,angle);
			VelCrl(CAN2, 1, M + AnglePidControl(angleError - distancePidControl(disError))); //pid中填入的是差值
			VelCrl(CAN2, 2, -M + AnglePidControl(angleError - distancePidControl(disError)));
			CheckOutline();
			pidZongShuchu = AnglePidControl(angleError - distancePidControl(disError));
			piddisShuchu = distancePidControl(disError);
		break;


		case 7://倒车程序
			BackCar(angle);
		break;
			
		default:
		break;
	}

		USART_OUT(UART5, (uint8_t *)"%d\t", (int)gRobot.pos.x);
		USART_OUT(UART5, (uint8_t *)"%d\t", (int)gRobot.pos.y);
		USART_OUT(UART5, (uint8_t *)"%d\t", (int)turnChangeTimes);
		USART_OUT(UART5, (uint8_t *)"%d\t", (int)circleChangeSymbol);
		USART_OUT(UART5, (uint8_t *)"%d\t", (int)angle);//gRobot.pos.angle
		USART_OUT(UART5, (uint8_t *)"%d\t", (int)angleError);
		USART_OUT(UART5, (uint8_t *)"%d\t", (int)disError);
		USART_OUT(UART5, (uint8_t *)"%d\t", (int)piddisShuchu);
		USART_OUT(UART5, (uint8_t *)"%d\t", (int)pidZongShuchu);
		USART_OUT(UART5, (uint8_t *)"%d\t", (int)circleChangeSymbol);
		USART_OUT(UART5, (uint8_t *)"%d\t\r\n", (int)turnTime);
}

int AreaCheck(float x, float y)//区域检查函数
{
	static int turnTime = 0;//定义转弯直线
	static int turnTimeRem = 0;//当turnTime改变时通过Rem来使车知道它转弯了
	static int circleChangeSymbolRem=0;//记住circleChangeSymbol当摄像头找球搞得换圈的时候的时候进行清空turnTimeChange
	//区域一
	float peakX_Area1[5] = { 250 ,-250,-2400,-2400,1900 };
	float peakY_Area1[5] = { 1700,1700,1450,0,0 };
	//区域二
	float peakX_Area2[5] = { 250 ,250,2400,2400,1900 };
	float peakY_Area2[5] = { 1700,3100,3350,0,0 };
	//区域三
	float peakX_Area3[5] = { 250 ,-250,2400,2400,-1900 };
	float peakY_Area3[5] = { 3100,3100,3350,4800,4800 };
	////区域四
	float peakX_Area4[5] = { -250 ,-250,-1900,-2400,-2400 };
	float peakY_Area4[5] = { 1700 ,3100,4800,4800,1450 };
	
	turnTimeRem = turnTime;
	circleChangeSymbolRem = circleChangeSymbol;

	if (CheckIn(x, y, 5, peakX_Area1, peakY_Area1) == 1)
	{
		turnTime = 0;
	}
	else if (CheckIn(x, y, 5, peakX_Area2, peakY_Area2) == 1)
	{
		turnTime = 1;
	}
	else if (CheckIn(x, y, 5, peakX_Area3, peakY_Area3) == 1)
	{
		turnTime = 2;
	}
	else if (CheckIn(x, y, 5, peakX_Area4, peakY_Area4) == 1)
	{
		turnTime = 3;
	}

	if (turnTime != turnTimeRem)
	{
		turnTimeChange++;//记住拐弯的次数
		SetTurnTimeChange(turnTimeChange);
	} 


	if (turnTimeChange < 5)
	{
		if ((x > -1400 && x < 1400) && (y > 900 && y < 3900))//内环
		{
			circleChangeSymbol = 0;
		}
		else if ((x < -1400 || x>1400) || (y < 900 || y>3900))//外环
		{
			circleChangeSymbol = 1;
		}
		
		if (circleChangeSymbolRem != circleChangeSymbol)//当发现其切出摄像头状态再回来的时候圈位置变了，使得自动拐弯的turnTimeChange变量自动清空重新计数
		{
			turnTimeChange = 0;
		}

	}else if (turnTimeChange == 5)//四条直线都走完了，还没发现球
	{
		circleChangeSymbol = !circleChangeSymbol;
	}else if (turnTimeChange == 6)
	{
		turnTimeChange = 2;
	}

	return turnTime;
}
int CheckIn(float x, float y, int pointNum, float * peakX, float  * peakY)//单个区域检查函数
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
void SetTurnTimeChange(int temp)
{
	turnTimeChange = temp;
}
int GetTurnTimeChange(void)
{
	return turnTimeChange;
}
extern Robot_t gRobot;
extern int Ball_counter;
float bestAngle=0,C_bestAngle=0,Cabs_bestAngle=0;
int arreaBestCounter=0;
void angleBest()
{
		int8_t leftAngLimit = -25;
		int8_t rightAngLimit = -9;
		int8_t max = 0;
		int8_t max_num = 0;
		for (int i = 0; i < 34; i++)
		{
			int8_t counter = 0;
			for (int j = 0; j < Ball_counter; j++)
			{
				if (gRobot.camera[j].angle < rightAngLimit && gRobot.camera[j].angle > leftAngLimit)
					counter++;
			}
			if (counter > max_num)
			{
				max_num = counter;
				max = i;
		arreaBestCounter=max;
		C_bestAngle=-17+arreaBestCounter;
		 bestAngle=Anglechange(C_bestAngle);
			}
			rightAngLimit++;
			leftAngLimit++;
		}
	Ball_counter=0;
}
/****************????????????????**************/
void Sub_Box(void)
{
	///????????2???,??????,?????????///
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
//???????
		for (int i = 0; i < 3; i++)
		{
			int8_t counter1 = 0;
			for (int j = 0; j < Ball_counter; j++)
			{
				if(gRobot.camera[j].dis<=106.5)
					{
						
						if (gRobot.camera[j].angle < rightAngLimit && gRobot.camera[j].angle > leftAngLimit)
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
//???
		leftAngLimit = -25;
		rightAngLimit=-11;
		for (int i = 0; i < 7; i++)
		{
			int8_t counter2=0;
			for (int j = 0; j < Ball_counter; j++)
			{
				 if(gRobot.camera[j].dis>106.5)
					{
						if (gRobot.camera[j].angle < rightAngLimit && gRobot.camera[j].angle > leftAngLimit)
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
//???
		c_Aimxfirst = 72.75 / tanf((65+29/2 +7* maxFirstlayer)*PI / 180);
		c_Firstbestangle = -25 + 7*maxFirstlayer+14.5f;
		Aimxfirst = getXpos() - 10*Dis(c_Aimxfirst, c_Aimyfirst, 0, 0)*sinf((getAngle() + c_Firstbestangle)*PI / 180) - 220 * sin(getAngle()*PI / 180);
	  Aimyfirst = getYpos() + 10*Dis(c_Aimxfirst, c_Aimyfirst, 0, 0)*cosf((getAngle() + c_Firstbestangle)*PI / 180) + 220 * cos(getAngle()*PI / 180);
		setAimxfirst(Aimxfirst);
		setAimyfirst(Aimyfirst);
//???
		c_Aimxsecond=140.25/tanf((65+14/2+5*maxSecondlayer)*PI/180);
		c_Secondbestangle=-25+5*maxSecondlayer+7;
		Aimxsecond = getXpos() -10* Dis(c_Aimxsecond, c_Aimysecond, 0, 0)*sinf((getAngle() + c_Secondbestangle)*PI / 180) - 220 * sin(getAngle()*PI / 180);
	  Aimysecond = getYpos() +10* Dis(c_Aimxsecond, c_Aimysecond, 0, 0)*cosf((getAngle() + c_Secondbestangle)*PI / 180) + 220 * cos(getAngle()*PI / 180);
		setAimxsecond(Aimxsecond);
		setAimysecond(Aimysecond);
		Ball_counter=0;
	}
	

