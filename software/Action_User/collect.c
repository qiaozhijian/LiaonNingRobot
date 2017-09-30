#include "config.h"
extern Robot_t gRobot;
 /****************************************************************************
* 名    称：maoCountBall
* 功    能：光电数球
* 入口参数：无
* 出口参数：Count(球的个数)
* 说    明：无
* 调用方法：无 
****************************************************************************/
int maoCountBall(void)
{
	static float blindTime = 0.0f;//被遮挡的时间

	static float Count = 0.0f;   //光电数到的球

	static float photoElectricity = 0;
	
	if(!ballVacant)
	{
		blindTime++;	
	}
	else
	{
		if(blindTime > 0)
		{
			photoElectricity = (blindTime * Pulse2Vel((gRobot.walk_t.right.aim+gRobot.walk_t.left.aim))*0.5f) / 38.f + 0.7f;/*这是利用坐标变化计算的车子的实际速度mm/s*/
			photoElectricity < 5 ? Count += (int)(photoElectricity) : Count++;
		}
		blindTime = 0;
	}
	return Count;
}
 /****************************************************************************
* 名    称：CountBall()
* 功    能：棍子数球函数
* 入口参数：无
* 出口参数：无
* 说    明：无
* 调用方法：无 
****************************************************************************/
void CountBall(void)
{
	static int ballN=0;
	static int sum=0;
	static int beginSum=0;
	static int pass=0;
	ReadActualVel(CAN1, COLLECT_BALL_ID);
	gRobot.collect_t.real.speed=gRobot.collect_t.real.speed/1000;
	if(pass==2)
	{
		if(gRobot.collect_t.real.speed>240)
		{
			pass=0;
			sum=0;
		}
		else 
		{
			pass=1;
		}
	}	
	
	if(gRobot.collect_t.real.speed<=235 && gRobot.collect_t.real.speed>100)
	{
		beginSum = 1;
		pass=2;
	}

	if(gRobot.collect_t.real.speed>=250)
	{
		beginSum = 0;
		if(sum>=50&&sum<=230)
		{
			ballN=1;
		}
		else if(sum>230&&sum<=1200)
		{
			ballN=2;
		}
		else if(sum>1200&&sum<=2000)
		{
			ballN=3;
		}
		else if(sum>2000)
		{
			ballN=4;
		}
		else 
		{
			ballN=0; 
			sum=0;
		}
	}
	if(beginSum&&pass)
	{
		sum += (250-gRobot.collect_t.real.speed/1000);
	}
	if(ballN)
	{
		gRobot.collect_t.PhotoElectric.ballcount +=ballN;
		ballN=0;
		sum=0;
	}

}


