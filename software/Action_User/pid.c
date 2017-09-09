#include "config.h"

extern Robot_t gRobot;
/****************************************************************************
* 名    称：angleErrorCount()
* 功    能：计算角度误差
* 入口参数：angle:角度,aimAngle:目标角度
* 出口参数：无
* 说    明：无
* 调用方法：无 
****************************************************************************/
float angleErrorCount(float aimAngle,float angle)
{
  static float angleError=0;
	angleError=aimAngle-angle;
	if (angleError > 180) 
	{
		angleError = angleError - 360;
	}
	else if (angleError < -180)
	{
		angleError = angleError + 360;
	}
	return angleError;
	
}

/****************************************************************************
* 名    称： ParkingAnglePidControl(float ERR)
* 功    能：定点停车专用PID
* 入口参数：angle:角度,aimAngle:目标角度
* 出口参数：无
* 说    明：无
* 调用方法：无 
****************************************************************************/
float ParkingAnglePidControl(float ERR)
{
	static int ERR_OLD = 0;
	static float Kp = 70; //90
	static float Kd = 10;
	static float OUTPUT;
	OUTPUT = Kp * ERR + Kd * (ERR - ERR_OLD);
	ERR_OLD = ERR;
	return OUTPUT;
}
/****************************************************************************
* 名    称：  AnglePidControl(float ERR)
* 功    能：角度PID
* 入口参数：ERR:角度误差
* 出口参数：无
* 说    明：无
* 调用方法：无 
****************************************************************************/
float AnglePidControl(float ERR)
{
	static int ERR_OLD = 0;
	static float Kp = 180;//165
	static float Kd = 20;//13
	static float OUTPUT;

	OUTPUT = Kp * ERR + Kd * (ERR - ERR_OLD);
	ERR_OLD = ERR;
	return OUTPUT;
}
/****************************************************************************
* 名    称：  distancePidControl(float ERR)
* 功    能：距离PID
* 入口参数：ERR:距离误差
* 出口参数：无
* 说    明：无
* 调用方法：无 
****************************************************************************/
float distancePidControl(float ERR)
{
	static int ERR_OLD = 0;
	static float Kp = 0.03; //0.02//0.03
	static float Kd = 0;
	static float OUTPUT;
	OUTPUT = Kp * ERR + Kd * (ERR - ERR_OLD);
	ERR_OLD = ERR;
	return OUTPUT;
}
/****************************************************************************
* 名    称：onceDistancePidControl(float ERR)
* 功    能：距离PID(只用1次)
* 入口参数：ERR:距离误差
* 出口参数：无
* 说    明：无
* 调用方法：无 
****************************************************************************/
float onceDistancePidControl(float ERR)
{
	static int ERR_OLD = 0;
	static float Kp = 0.08; //0.2
	static float Ki = 0;
	static float Kd = 0;
	static float OUTPUT;
	OUTPUT = Kp * ERR + Kd * (ERR - ERR_OLD) +Ki*0.0f;
	ERR_OLD = ERR;
	return OUTPUT;
}
float spacingPidControl(float ERR)
{
	static int ERR_OLD = 0;
	static float Kp = 0.05; //0.1
	static float Ki = 0;
	static float Kd = 1;
	static float OUTPUT;
	OUTPUT = Kp * ERR + Kd * (ERR - ERR_OLD) +Ki*0.0f;
	ERR_OLD = ERR;
	return OUTPUT;
}
/****************************************************************************
* 名    称：Line1()
* 功    能：主扫场PID
* 入口参数：aimX:目标x aimY:目标y,aimAngle:目标角度
line1:判断当前的目标直线是X=？(line1=0),还是Y=？(line1=1)
sign:符号位
* 出口参数：无
* 说    明：无
* 调用方法：无 
****************************************************************************/
void Line(float aimX,float aimY,float aimAngle,int line1,int sign)
{
		static float x = 0, y = 0, angle = 0;
		static float angleError = 0; 											//目标角度与当前角度的偏差
		static float distanceStraight = 0;								//提前量
		static float disError = 0;   											//距离偏差
		static int lineChangeSymbol=0;
	
		x=gRobot.pos.x;
		y=gRobot.pos.y;
		angle=gRobot.pos.angle;
	
	switch(line1)
	{
case 0:
		disError = x-(aimX+ sign*lineChangeSymbol*470);
		angleError=angleErrorCount(aimAngle,angle);;
		distanceStraight=sign*(aimY+ sign*lineChangeSymbol*350)-sign*y;
		if (fabs(distanceStraight) > turnTimeLead(lineChangeSymbol))
			{
					if(lineChangeSymbol<1)
					{
						VelCrl(CAN2, 1, gRobot.M + AnglePidControl(angleError +sign* onceDistancePidControl(disError))); //pid中填入的是差值
						VelCrl(CAN2, 2, -gRobot.M + AnglePidControl(angleError +sign* onceDistancePidControl(disError)));
					}
					else if(lineChangeSymbol>=1)
					{
						VelCrl(CAN2, 1, gRobot.M + AnglePidControl(angleError +sign* distancePidControl(disError))); //pid中填入的是差值
						VelCrl(CAN2, 2, -gRobot.M + AnglePidControl(angleError +sign* distancePidControl(disError)));
					}
			}
		else if (fabs(distanceStraight) < turnTimeLead(lineChangeSymbol))
			{
					distanceStraight = 0;
					gRobot.turnTime ++;
			}
				break;
case 1:
			disError = y - (aimY +  sign*lineChangeSymbol*350); //小车距离与直线的偏差//不加绝对值是因为判断车在直线上还是直线下//4100
			angleError = angleErrorCount(aimAngle,angle);
			distanceStraight = (aimX -sign*lineChangeSymbol*470) - x;
			if (fabs(distanceStraight) > turnTimeLead(lineChangeSymbol))
			{
				VelCrl(CAN2, 1, gRobot.M + AnglePidControl(angleError +sign* distancePidControl(disError))); //pid中填入的是差值
				VelCrl(CAN2, 2, -gRobot.M + AnglePidControl(angleError +sign* distancePidControl(disError)));
			}
			if (fabs(distanceStraight) < turnTimeLead(lineChangeSymbol))
			{
				distanceStraight = 0;
				gRobot.turnTime++;
			}
			break;
			default:
			break;
	}
		if(gRobot.turnTime==4)
		{
			lineChangeSymbol++;
			gRobot.turnTime=0;
		}
		if (lineChangeSymbol == 3)
		{
			lineChangeSymbol=0;
			gRobot.turnTime = 11 ;
		}
//检查是否卡死，若卡死则触发避障
		CheckOutline();
//调试程序
		d_Coor();
		d_Line(gRobot.turnTime,lineChangeSymbol,disError,angleError,distanceStraight,turnTimeLead(lineChangeSymbol));
}

