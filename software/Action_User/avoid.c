#include "config.h"

extern Robot_t gRobot;

static int turnTimeRemember;												   //��ס�ڿ�����ʱ����ʲôֱ�ߵ�״̬���ȵ���case��������������װ
static float xStick, yStick;															   //��סʱ�洢��λ������

 /****************************************************************************
* ��    �ƣ�void BackCarIn(float angle)
* ��    �ܣ��ڻ����ݳ������1.5s����ת45��
* ��ڲ�����angle//��ǰ�Ƕ�
* ���ڲ�������
* ˵    ������
* ���÷������� 
****************************************************************************/
void BackCarIn(float angle) //�ڻ���������
{
	static float aimAngle = 0;   //Ŀ��Ƕ�
	static float angleError = 0; //Ŀ��Ƕ��뵱ǰ�Ƕȵ�ƫ��
	static int i = 0;																  //Ŀ��Ƕȱ任��־λ
	static int j = 0; 																//�ڴ�������־λ���ź���10ms����һ�Σ��ﵽ��ʱ��Ч��
	if (i == 0)																		    //ʹĿ��Ƕ�ƫ���ұ�45
	{
		aimAngle = angle - 45; //�ó�ͷĿ��Ƕ���ƫ45��
		i = 1;
	}
	angleError = angleErrorCount(aimAngle,angle);
	j++;
	if (j < 150)
	{
		VelCrl(CAN2, 1, -6107); //pid��������ǲ�ֵ
		VelCrl(CAN2, 2,  6107);
	}else if (j >=150)
	{
		VelCrl(CAN2, 1, AnglePidControl(angleError)); //pid��������ǲ�ֵ
		VelCrl(CAN2, 2, AnglePidControl(angleError));
		if (fabs(angleError) < 5)
		{
			gRobot.turnTime = turnTimeRemember;
			i = 0;
			j = 0;//��ձ�־λ
		} 
	}
//	pidZongShuchu = AnglePidControl(angleError);
}
 /****************************************************************************
* ��    �ƣ�void BackCarOut(float angle) 
* ��    �ܣ��⻷���ݳ������1.5s����ת45��
* ��ڲ�����angle//��ǰ�Ƕ�
* ���ڲ�������
* ˵    ������
* ���÷������� 
****************************************************************************/
void BackCarOut(float angle) //�⻷��������
{
	static float aimAngle = 0;   //Ŀ��Ƕ�
	static float angleError = 0; //Ŀ��Ƕ��뵱ǰ�Ƕȵ�ƫ��
	static int i = 0;																  //Ŀ��Ƕȱ任��־λ
	static int j = 0; 																//�ڴ�������־λ���ź���10ms����һ�Σ��ﵽ��ʱ��Ч��
	if (i == 0)																		  //ʹĿ��Ƕ�ƫ���ұ�45
	{
		aimAngle = angle + 45; //�ó�ͷĿ��Ƕ���ƫ45��
		i = 1;
	}
	angleError = angleErrorCount(aimAngle,angle);
	j++;
	if (j < 150)
	{
		VelCrl(CAN2, 1, -6107); //pid��������ǲ�ֵ
		VelCrl(CAN2, 2,  6107);
	}else if (j >=150)
	{
		VelCrl(CAN2, 1, AnglePidControl(angleError)); //pid��������ǲ�ֵ
		VelCrl(CAN2, 2, AnglePidControl(angleError));
		if (fabs(angleError) < 5)
		{
			gRobot.turnTime = turnTimeRemember;
			i = 0;
			j = 0;//��ձ�־λ
		}
	}
//	pidZongShuchu = AnglePidControl(angleError);
}
 /****************************************************************************
* ��    �ƣ�void CheckOutline(void) 
* ��    �ܣ�����Ƿ���
* ��ڲ�������
* ���ڲ�������
* ˵    ������ǰ��ͣ��x=1000 y=0
* ���÷������� 
****************************************************************************/
void CheckOutline(void)//����Ƿ���
{
	static int stickError = 0;													   //�����������ֵ
	static float xError = 0, yError = 0;
	turnTimeRemember = gRobot.turnTime;
	xError = gRobot.pos.x - getxRem();
	yError = gRobot.pos.y - getyRem();
	//�жϽ��̵���һ�����滻M��  --summer
	if (fabs(xError) < 1 && fabs(yError) < 1 && gRobot.M != 0)
	{
		stickError++;
	}
	else
	{
		stickError = 0;
	}
	/*
	200̫��
	�������������ʱ����ֱ�ߵ�ʱ�򣬹����ʱ��ͣ��Ͷ�򣬽�����ʱ��
	��һ����С��1  ����Ӧ���е��ٶ��趨
	*/
	if (stickError > 200)
	{
		xStick = getxRem();//��ס����������
		yStick = getyRem();
		gRobot.turnTime = 7;
		stickError = 0;
	}
}
 /****************************************************************************
* ��    �ƣ�void BackCarOut(float angle) 
* ��    �ܣ����⻷���ݳ���ϲ�
* ��ڲ����� xKRem,yKRem,angle��ס��λ�õ�x��y���꣬�͵�ǰ�Ƕ�
* ���ڲ�������
* ˵    ������
* ���÷������� 
****************************************************************************/
void BackCar(float angle)
{
	angle=gRobot.pos.angle;
	if((xStick>-1400&&xStick<1400)&&(yStick>900&&yStick<3900))//�ڻ�
	{
		BackCarOut(angle);
	}
	else if((xStick<-1400||xStick>1400)||(yStick<900||yStick>3900))//�⻷
	{
		BackCarOut(angle);
	}
}	

