#include "config.h"


/****************************************************************************
* 名    称：float angleErrorCount(float aimAngle,float angle)
* 功    能：计算角度偏差作为pid
* 入口参数：aimAngle: 目标角度
						angle：   当前角度
* 出口参数：angleError：角度错误
* 说    明：无
* 调用方法：无 
****************************************************************************/
float angleErrorCount(float aimAngle,float angle)//计算角度偏差作为pid
{
  static float angleError=0;
	angleError=aimAngle-angle;
	if (angleError > 180) //防止出现乱转
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
* 名    称：pid函数1
* 功    能：计算偏差作为pid
* 入口参数：偏差
* 出口参数：调整量
* 说    明：无
* 调用方法：无 
****************************************************************************/
/********************************************/ //角度pid函数
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
/********************************************/ //角度pid函数
float AnglePidControl(float ERR)
{
	static int ERR_OLD = 0;
	static float Kp = 165;
	static float Kd = 13;
	static float OUTPUT;
	
	OUTPUT = Kp * ERR + Kd * (ERR - ERR_OLD);
	ERR_OLD = ERR;
	return OUTPUT;
}
/**********************************************/ //距离pid函数
float distancePidControl(float ERR)
{
	static int ERR_OLD = 0;
	static float Kp = 0.01; //0.02//0.03
	static float Ki = 0;
	static float Kd = 0;
	static float OUTPUT;
	OUTPUT = Kp * ERR + Kd * (ERR - ERR_OLD);
	ERR_OLD = ERR;
	return OUTPUT;
}
/**********************************************/ //距离pid函数
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
	static float Kp = 0.1; //0.1//40
	static float Ki = 0;
	static float Kd = 1;
	static float OUTPUT;
	OUTPUT = Kp * ERR + Kd * (ERR - ERR_OLD) +Ki*0.0f;
	ERR_OLD = ERR;
	return OUTPUT;
}

