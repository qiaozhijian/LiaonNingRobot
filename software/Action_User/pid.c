#include "config.h"


/****************************************************************************
* ��    �ƣ�float angleErrorCount(float aimAngle,float angle)
* ��    �ܣ�����Ƕ�ƫ����Ϊpid
* ��ڲ�����aimAngle: Ŀ��Ƕ�
						angle��   ��ǰ�Ƕ�
* ���ڲ�����angleError���Ƕȴ���
* ˵    ������
* ���÷������� 
****************************************************************************/
float angleErrorCount(float aimAngle,float angle)//����Ƕ�ƫ����Ϊpid
{
  static float angleError=0;
	angleError=aimAngle-angle;
	if (angleError > 180) //��ֹ������ת
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
* ��    �ƣ�pid����1
* ��    �ܣ�����ƫ����Ϊpid
* ��ڲ�����ƫ��
* ���ڲ�����������
* ˵    ������
* ���÷������� 
****************************************************************************/
/********************************************/ //�Ƕ�pid����
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
/********************************************/ //�Ƕ�pid����
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
/**********************************************/ //����pid����
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
/**********************************************/ //����pid����
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

