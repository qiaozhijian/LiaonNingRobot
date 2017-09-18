#include "config.h"


/**********************************************/
extern Robot_t gRobot;
/*******************逆时针画圆*****************/
float CircleAnglePidControl(float ERR)
{
	static int ERR_OLD=0;
	static float Kp=90;//原来是60
	static float Kd=8;
	static float OUTPUT;
	OUTPUT = Kp * ERR + Kd*(ERR- ERR_OLD);
	ERR_OLD=ERR;
	return OUTPUT;
}
void NiShiZhenCircleBiHuan(float V,float R,float X0,float Y0)//逆时针旋转
{
		int M1,M2,V1,V2;//定义速度，输入的脉冲
		static float x=0.f,y=0.f,angle=0.f;
		static float aimAngle=0;//目标角度
		static float angleError=0;//目标角度与当前角度的偏差
		static float distanceCenter=0;//当前坐标与圆心的差值
		static float k;//定义斜率
		static float 	spacingError;//定义两个点之间的距离
		static float kAngle;//当前点与圆相交点的切线的速度方向（用actan处理的角度制的数据）
		static float dx,dy;//当前坐标与圆心的差值
		//逆时针圆形闭环
		x=getXpos();//当前x坐标
		y=getYpos();//当前y坐标
		angle=getAngle();//当前角度
		distanceCenter=sqrt((x-X0)*(x-X0)+(y-Y0)*(y-Y0));
		spacingError=distanceCenter-R; 
		dx=x-X0;
		dy=y-Y0;
		k=dy/dx;
		kAngle=atan(-1/k)*180/PI;
		if(fabs(dy)<1)//判断特殊角度
		{
			if(dx>0)
			{
				aimAngle=0;
			}else if(dx<0)
			{
			
				aimAngle=180;
			}
		}
		if(fabs(dx)<1)
		{
			if(dy>0)
			{
				aimAngle=90;
			}else if(dy<0)
			{
				aimAngle=-90;
			}
		}
		
		if((dx>0))
		{
			if(dy>0)//在第一象限
			{
				aimAngle=kAngle+90;
			}else if(dy<0)//在第四象限
			{
				aimAngle=-(90-kAngle);
			}
		}else if(dx<0)
		{
			if(dy>0)//在第二象限
			{
				aimAngle=kAngle+90;
			}else if(dy<0)//在第三象限
			{
				aimAngle=-(90-kAngle);
			}
		}
		angleError=angleErrorCount(aimAngle,angle);
		V1=((R+(WHEEL_TREAD/2))/R)*V;	//可以得到外轮的速度
		V2=((R-(WHEEL_TREAD/2))/R)*V;
		M1=V1/(3.14f*WHEEL_DIAMETER)*4096.f;
		M2=V2/(3.14f*WHEEL_DIAMETER)*4096.f;
		VelCrl(CAN2, 1, M1+CircleAnglePidControl(angleError+spacingPidControl(spacingError)));      
		VelCrl(CAN2, 2, -M2+CircleAnglePidControl(angleError+spacingPidControl(spacingError)));
		
//		USART_OUTF(x);
//		USART_OUTF(y);
//		USART_OUTF(dx);
//		USART_OUTF(dy);
//		USART_OUTF(kAngle);
//		USART_OUTF(angle);
//		USART_OUTF(angleError);//角度偏差
//		USART_OUTF(spacingError);//距离
//		USART_OUTF(aimAngle);
//		USART_OUTF(M1+CircleAnglePidControl(angleError+spacingPidControl(spacingError)));
//		USART_OUTF(M2+CircleAnglePidControl(angleError+spacingPidControl(spacingError)));
//		USART_OUT_CHAR("\r\n");
}
/***********************************************************/
void ShunShiZhenCircleBiHuan(float V,float R,float X0,float Y0)//顺时针旋转
{
		int M1,M2,V1,V2;//定义速度，输入的脉冲
		static float x=0.f,y=0.f,angle=0.f;
		static float aimAngle=0;//目标角度
		static float angleError=0;//目标角度与当前角度的偏差
		static float distanceCenter=0;//当前坐标与圆心的差值
		static float /*a=-1,b=1,c=0,*/k;//定义斜率
		static float 	spacingError;//定义两个点之间的距离
		static float kAngle;//当前点与圆相交点的切线的速度方向（用actan处理的角度制的数据）
		static float dx,dy;
		x=getXpos();//当前x坐标
		y=getYpos();//当前y坐标
		angle=getAngle();//当前角度
		distanceCenter=sqrt((x-X0)*(x-X0)+(y-Y0)*(y-Y0));
		spacingError=distanceCenter-R;  
		dx=x-X0;
		dy=y-Y0;
		k=dy/dx;
		kAngle=atan(-1/k)*180/PI;
		if(fabs(dy)<0.0001)
			{
				if(dx>0)
				{
					aimAngle=-180.f;
				}else if(dx<0)
				{
					aimAngle=0.f;
				}
			}else if(fabs(dx)<0.0001)
			{
				if(dy>0)
				{
					aimAngle=-90.f;
				}else if(dy<0)
				{
					aimAngle=90.f;
				}
			}
			
			if(dx>0)
			{
				if(dy>0)//在第一象限
				{
					aimAngle=kAngle-90.f;
				}else if(dy<0)//在第四象限
				{
					aimAngle=90.f+kAngle;
				}
			}else if(dx<0)
			{
				if(dy>0)//在第二象限
				{
					aimAngle=kAngle-90.f;
				}else if(dy<0)//在第三象限
				{
					aimAngle=90.f+kAngle;
				}
			}
			angleError=angleErrorCount(aimAngle,angle);
			V1=((R+(WHEEL_TREAD/2))/R)*V;	//可以得到外轮的速度
			V2=((R-(WHEEL_TREAD/2))/R)*V;
			M1=V1/(3.14f*WHEEL_DIAMETER)*4096.f;
			M2=V2/(3.14f*WHEEL_DIAMETER)*4096.f;
			VelCrl(CAN2, 1, M2+CircleAnglePidControl(angleError-spacingPidControl(spacingError)));      
			VelCrl(CAN2, 2, -(M1-CircleAnglePidControl(angleError-spacingPidControl(spacingError))));
}
/**********************************吃球路线1*************************/
CircleCenter_t countEatBallWay1(float xBall, float yBall, float xStart, float yStart, float angle)//吃球方案1，得出圆形的圆心
{
//	static float x1 = 0, y1 = 0, x2 = 0, y2 = 0;
	static float a = 0, b = 0, c = 0, d = 0;    //两条直线的参数前两个为车所在的直线的垂线，
																							//后两个为车的起始位和球区域中点的形成的弦的垂直平分线
	static float xMid = 0, yMid = 0;            //垂直平分线的中点
	static float dx = 0, dy = 0;//
	static int step = 0;
	static CircleCenter_t circleCenter;
	xMid = (xBall + xStart) / 2;
	yMid = (yBall + yStart) / 2;
	dx = xBall - xStart;
	dy = yBall - yStart;
	//第一条直线
	if ((angle>89&&angle<90) || (angle>-90&&angle<-89))//angle是车体的角度。在此时车速度的垂线不存在（与x轴垂直）
	{
//		x1 = xStart;
		step = 1;
	}else if ((1000*angle>-1&&1000*angle<1) || (angle>179) || (angle<-179))//车速度的垂线与x轴平行
	{
		a = 0;
		b = yStart;
//		y1 = b;
		step = 5;
	}
	else//注意此时的angle是之前12秒初始化确立的正前方为0度，然而k却是以x轴为起始坐标确定的
	{
		if (angle > 0 && angle < 90)
		{
			angle = 90 - angle;
			a = 1 / tan(angle*PI / 180);//a =- 1 / -tan(angle*PI / 180)
		}
		else if (angle > -180 && angle < -90)
		{
			angle = 90 + angle;
			a = -1 / tan(angle*PI / 180);
		}
		else if (angle > -90 && angle < 0)
		{
			angle = 90 + angle;
			a = -1 / tan(angle*PI / 180);//
		}
		else if (angle > 90 && angle < 180)
		{
			angle = angle - 90;
			a = -1 / tan(angle*PI / 180);//
		}
		b = yStart - a*xStart;
//		y1 = a*x1 + b;
		step = 9;
	}


	//第二条直线
	if (1000*dy<1&&1000*dy>-1)					//当垂直平分线k不存在的时候(与x轴垂直)
	{
//		x2 = xMid;
		step +=1;
	}
	else if (1000*dx<1&&1000*dx>-1)			//垂直平分线与x轴平行
	{
		c = 0;
		d = yMid;
//		y2 = d;
		step += 2;
	}
	else//另外的情况
	{
		c = -dx / dy;
		d = yMid - c*xMid;
//		y2 = c*x2 + d;
		step += 3;
	}

	switch (step)
	{
		case 2:													//两条直线平行直接爆炸返回中点
			circleCenter.x = xMid;
			circleCenter.y = yMid;
			step = 0;
		break;

		case 3:
			circleCenter.x = xStart;
			circleCenter.y = yMid;
			step = 0;
		break;

		case 4:
			circleCenter.x = xStart;
			circleCenter.y = c*xStart + d;
			step = 0;
		break;

		
		case 6:
			circleCenter.x = xMid;
			circleCenter.y = yStart;
			step = 0;
		break;

		case 7://两条直线平行直接爆炸,返回中点
			circleCenter.x = xMid;
			circleCenter.y = yMid;
			step = 0;
		break;

		case 8:
			circleCenter.x = (b - d) / c;
			circleCenter.y = yStart;
			step = 0;
		break;

		
		
		case 10:
			circleCenter.x = xMid;
			circleCenter.y = a*xMid + b;
			step = 0;
		break;

		case 11:
			circleCenter.x = (yMid - b) / a;
			circleCenter.y = yMid;
			step = 0;
		break;

		case 12:
			if (1000*(a - c) < 1 && 1000*(a - c) > -1)
			{
				circleCenter.x = xMid;
				circleCenter.y = yMid;
			}
			else
			{
				circleCenter.x = (b - d) / (c - a);
				circleCenter.y = a*(b - d) / (c - a) + b;
			}
			step = 0;
		break;

		default:
			step = 0;
		break;
	}
	circleCenter.R = sqrt((circleCenter.x - xStart)*(circleCenter.x - xStart) + (circleCenter.y - yStart)*(circleCenter.y - yStart));
	return circleCenter;
}
/********************************吃球路线2***************************/
CircleCenter2_t countCircleCenter2(Point_t p1, Point_t p2, Point_t p3)
{
	static float a = 0.f, b = 0.f, c = 0.f, d = 0.f;////p1,p2个点的坐标差值	//p2,p3点的坐标差值
	static float e = 0.f, f = 0.f;//计算公式中后面的定值
	static CircleCenter2_t circleCenter2;
	//p1,p2个点的坐标差值
	a = p1.x - p2.x;
	b = p1.y - p2.y;
	//p2,p3点的坐标差值
	c = p1.x - p3.x;
	d = p1.y - p3.y;
	e = ((p1.x*p1.x - p2.x*p2.x) + (p1.y*p1.y - p2.y*p2.y)) / 2;
	f = ((p1.x*p1.x - p3.x*p3.x) + (p1.y*p1.y - p3.y*p3.y)) / 2;
	if ((a / b)!=(c / d))//三个点不在同一条直线上
	{
		circleCenter2.checkOnSameLine = 0;
		circleCenter2.x = (b*f - d*e) / (b*c - a*d);
		circleCenter2.y = (c*e - a*f) / (b*c - a*d);
		circleCenter2.R = sqrt((circleCenter2.x - p1.x)*(circleCenter2.x - p1.x) + (circleCenter2.y - p1.y)*(circleCenter2.y - p1.y));
		return circleCenter2;
	}
	else if(1000*((a / b) - (c / d))<1&& 1000*((a / b) - (c / d))>-1)
	{
		circleCenter2.checkOnSameLine = 1;
		circleCenter2.x = 0;
		circleCenter2.y = 0;
		circleCenter2.R = 0;
		return circleCenter2;
	}
	return circleCenter2;
}
/***************************吃球路线3***************************/

/*************************找球程序******************************/
/*************************结构体数据缓存容器**************************/
Container_t Container(void)
{
	Container_t tmp;
	tmp.anglestart=getAngle();
	tmp.xstart=getXpos();
	tmp.ystart=getYpos();
	tmp.tmpgetAimxfirst=getAimxfirst();
	tmp.tmpgetAimyfirst=getAimyfirst();
	tmp.tmpgetAimxsecond=getAimxsecond();
	tmp.tmpgetAimysecond=getAimysecond();
	tmp.dis_start2first=Dis(gRobot.walk_t.pos.x,gRobot.walk_t.pos.y,getAimxfirst(),getAimyfirst());
	tmp.dis_first2second=Dis(getAimxfirst(),getAimyfirst(),getAimxsecond(),getAimysecond());
	return tmp;
}

static int flag=0;
/************************方案1对应找球1*************************/
extern int t_FindBall;
void Findball_1(void)
{
	static CircleCenter_t tmpFirst;
	static CircleCenter_t tmpSecond;
	static float anglerem=0;
	static Container_t tmp;
	if(Dis(tmp.tmpgetAimxfirst,tmp.tmpgetAimyfirst,getXpos(),getYpos())<20)
	{
		flag=2;
	}
	else if(Dis(tmp.tmpgetAimxsecond,tmp.tmpgetAimysecond,getXpos(),getYpos())<20)
	{
		flag=4;
	}
	switch(flag)
	{
		case 0:
			tmp=Container();
			anglerem=getAngle();
			flag++;
			break;
		case 1:
			tmpFirst=countEatBallWay1(tmp.tmpgetAimxfirst,tmp.tmpgetAimyfirst,tmp.xstart,tmp.ystart,tmp.anglestart);
		if(anglerem>0)ShunShiZhenCircleBiHuan(500,tmpFirst.R,tmpFirst.x,tmpFirst.y);
		else NiShiZhenCircleBiHuan(500,tmpFirst.R,tmpFirst.x,tmpFirst.y);
			break;
		case 2:
			anglerem=getAngle();
			flag++;
			break;
		case 3:
			tmpSecond=countEatBallWay1(tmp.tmpgetAimxsecond,tmp.tmpgetAimysecond ,tmp.xstart,tmp.ystart,tmp.anglestart);
		if(anglerem<0)ShunShiZhenCircleBiHuan(500,tmpSecond.R,tmpSecond.x,tmpSecond.y);
		else NiShiZhenCircleBiHuan(500,tmpSecond.R,tmpSecond.x,tmpSecond.y);
			break;
		case 4:
			flag=0;
			break;
		default: break;
	}
		t_FindBall=0;
}
/************************方案2对应找球2*************************/
void Findball_2()
{
	static Point_t Pointstart;
	static Point_t Pointfirst;
	static Point_t Pointsecond;
	static CircleCenter2_t circle;
	static Container_t tmp;
	static float anglerem=0;
	switch(flag)	
	{
		case 0:
		tmp=Container();
		Pointstart.x=tmp.xstart;
		Pointstart.y=tmp.ystart;
		Pointfirst.x=tmp.tmpgetAimxfirst;
		Pointfirst.y=tmp.tmpgetAimyfirst;
		Pointsecond.x=tmp.tmpgetAimxsecond;
		Pointsecond.y=tmp.tmpgetAimysecond;
		anglerem=getAngle();
		circle=countCircleCenter2(Pointstart,Pointfirst,Pointsecond);
		flag++;
			break;
		case 1:
		if(circle.checkOnSameLine==0)
			{
				flag=2;
			}
		else if(circle.checkOnSameLine==1)
			{
				flag=3;
			}
			break;
		case 2:
			if(anglerem>0)ShunShiZhenCircleBiHuan(5000,circle.R,circle.x,circle.y);
			else NiShiZhenCircleBiHuan(5000,circle.R,circle.x,circle.y);
			break;
		case 3:
			Pointparking(Pointsecond.x,Pointsecond.y);
			break;
		default://USART_OUT();
			break;
	}
	if(Dis(Pointsecond.x,Pointsecond.y,getXpos(),getYpos())<20)
	{
		flag=0;
	}
		t_FindBall=0;
}
/************************找球方案3************************/
void Findball_3(void)
{
	static int flagcount=0;
	static Container_t tmp;
	switch(flag)
	{
		case 0:
			tmp=Container();
			flagcount=flagcount+4;
		if(flagcount>200)
			flag=1;
		  break;
		case 1:
			USART_OUT(UART5,(uint8_t*)"ttt%d\t%d\t%d\r\n",(int)tmp.tmpgetAimxfirst,(int)tmp.tmpgetAimyfirst,(int)Dis(tmp.tmpgetAimxfirst,tmp.tmpgetAimyfirst,getXpos(),getYpos()));
				Pointparking(tmp.tmpgetAimxfirst,tmp.tmpgetAimyfirst);
 			if(Dis(tmp.tmpgetAimxfirst,tmp.tmpgetAimyfirst,getXpos(),getYpos())<150)
				{
					 flag=2;
				}
			break;
		case 2:
			Pointparking(tmp.tmpgetAimxsecond,tmp.tmpgetAimysecond);
			if(Dis(tmp.tmpgetAimxsecond,tmp.tmpgetAimysecond,getXpos(),getYpos())<150)
				{
					 flag=3;
				}
			break;
		case 3:
			flag=0;
		  break;
		default://USRAT_OUT("");
			break;
	}
		t_FindBall=0;
	
}
/************************找球方案4************************/
/*********************此方案用球最多的角度****************/
void Findball_4(void)
{
	static float Xstart=0;
	static float Ystart=0;
	static float aimangle=0;
	static float Angleerr=0;
	switch(flag)
	{
		case 0:
			Xstart=getXpos();
			Ystart=getYpos();
			aimangle=Anglechange(getBestangle());
			flag++;
			break;
		case 1:
			Angleerr=angleErrorCount(aimangle,getAngle());
			VelCrl(CAN2, 1,AnglePidControl(Angleerr));//pid中填入的是差值
			VelCrl(CAN2, 2,-AnglePidControl(Angleerr));
			break;
		case 2:
			VelCrl(CAN2, 1,5000);
			VelCrl(CAN2, 2,-5000);
		  break;
		case 3:
			flag=0;
		default://USART_OUT();
			break;
	}	
	if(fabs(Angleerr)<2)flag=2;
	if(fabs(Dis(Xstart,Ystart,getXpos(),getYpos()))>1500)flag=3;
	t_FindBall=0;
	}
/************************找球方案5************************/
float Findball_5(void)
{
	return getBestangle()*20;
}
/*******************找球函数*******************/
void CameraFindball(int cmodel)
{
	if(t_FindBall>300)
	{
		flag=0;
	}
  switch(cmodel)
	{
		case 1:Findball_1();
		break;
		case 2:Findball_2();
		break;
		case 3:Findball_3();
		break;
		case 4:Findball_4();
		break;
		case 5:Findball_5();
		break;
		default:cmodel=0;
		//USART_OUT();
		break;
	}
}
