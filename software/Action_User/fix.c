#include "config.h"

static float angle=0,xpos=0,ypos=0;
static float errSingle=0; //errSingle = realSingle - nowSingle
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


void setErrSingle(float reaAngle)
{
	errSingle = reaAngle - angle;
	USART_OUT(USART1,(uint8_t*) "setSingle %d\r\n", (int)errSingle);
}


void setErrX(float realX)
{
	float temp;
	temp = xpos * cos(-errSingle * PI / 180) - ypos * sin(-errSingle * PI / 180);
	errX0 = realX - temp;
	USART_OUT(USART1,(uint8_t*)"setErrX %d\r\n", (int)errX0);
}
void setErrY(float realy)
{
	float temp;
	temp = xpos * sin(-errSingle * PI / 180) + ypos * cos(-errSingle * PI / 180);
	errY0 = realy - temp;
	USART_OUT(USART1,(uint8_t*)"setErrY %d\r\n", (int)errY0);
}


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
void setErr(float reaAngle,float realX,float realy)
{
	setErrSingle(reaAngle);
	setErrX(realX);
	setErrY(realy);
}
/**
*	参数 void
*	返回值 得到左侧激光运算后返回的距离
*/
int getLeftAdc()
{
	return 0.9389*Get_Adc_Average(ADC_Channel_15, 10)+428.6575;
}
/**
*	参数 void
*	返回值 得到右侧激光运算后返回的距离
*/
int getRightAdc()
{
	return 0.9403*Get_Adc_Average(ADC_Channel_14, 10)+435.445;
}

/**
* @author Haoan Feng
* 参数
* 获取最佳的停车地点
* 返回  0 Left 1 Right 2 Up 3 Down
* 可重入
*/
extern Robot_t gRobot;
static int map[4] = {0};
uint8_t getAimBorder() //返回距离最小的边界
{
	static int i = 0;
	static int min_num = 0;
	static int temp[4];
	static int min_dis = 0;

	//
	float x = gRobot.pos.x;
	float y = gRobot.pos.y;
	// 计算到四个边界的距离
	temp[0] = 2400 + x; //Left
	temp[1] = 2400 - x; //Right
	temp[2] = 4800 - y; //Up
	temp[3] = y;		//Down
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
#define UP_BORDER 2	//上边界
#define DOWN_BORDER 3  //下边界
//需要进行边界修改
//#define Y_MIN (320 + 200)
//#define X_MAX (2400 - 320 - 200 - 200)
//#define X_MIN (-2400 + 320 + 200)
//#define Y_MAX (4800 - 320 - 200)

static uint8_t fixSuccessFlag = 0; //修正成功标志位//到时把这个变量放入结构体当中

float getFixAngle(int aimBorder)//得到修正角度
{
	switch (aimBorder)
	{
		case LEFT_BORDER:
			return -90;
		case RIGHT_BORDER:
			return 90;
		case UP_BORDER:
			return 180;
		case DOWN_BORDER:
			return 0;
	}
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
typedef struct{//矫正失败拿到新的墙的目标角度
	float x;
	float y;
}AimPos_t;

int CommitFix(int laserLeftDistance,int laserRightDistance)//确定是否能进行修正激光被挡或者不在激光处理范围内
{
	static int commitFix=0;//靠下一面墙的标志位
	if(laserLeftDistance+laserRightDistance<4800-42)
	{
		commitFix=0;//说明要靠下一面墙
	}
	else
	{
		commitFix=1;	//说明能够进行矫正
	} 
	
	if(gRobot.laser.leftDistance>4000||gRobot.laser.leftDistance<40)
	{
		commitFix=0;//说明要靠下一面墙
	}
	else
	{
		commitFix=1;	//说明能够进行矫正
	}
	
	if(gRobot.laser.rightDistance>4000||gRobot.laser.rightDistance<40)
	{
		commitFix=0;//说明要靠下一面墙
	}
	else
	{
		commitFix=1;	//说明能够进行矫正
	}
	return commitFix;
}

void fixPosFirst(int aimBorder)
{
	switch(aimBorder)
	{
		case LEFT_BORDER :
		{
			setErr(-90,X_MIN,getRightAdc());
		}
		break;
		
		case RIGHT_BORDER:
		{
			setErr(90,X_MAX,getLeftAdc());
		}
		break;
		
		case UP_BORDER:
		{
			setErr(180,getRightAdc()-2400,Y_MAX);
		}
		break;
		
		case DOWN_BORDER :
		{
			setErr(0,getLeftAdc()-2400,Y_MIN);
		}
		break;
	}
}

void fixPosSec(int aimBorder)//矫正当前墙的坐标
{
	static float aimFixAngle=0;
	aimFixAngle=getFixAngle(aimBorder);
	setErrSingle(aimFixAngle); //修正角度
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
}

AimPos_t Go2NextWall(int aimBorder)//第一次矫正失败后到下一面墙的目标点
{
	AimPos_t aimPos;
	switch (aimBorder)
	{
		case LEFT_BORDER:
			aimPos.x=-1800;//第二次靠墙判断最小距离墙面之后代入定点停车
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

int FixTask()
{
	//修正状态
	static int againstTime=0;//靠墙的次数
	static int aimBorder=0;//目标边界
	static int aimFixCounter;//矫正时间计算
	static float fixAngle=0;//矫正角度
	int laserLeftDistance=getLeftAdc();//左边激光
	int laserRightDistance=getRightAdc();//右边激光
	int commitFix=0;
	AimPos_t aimPos;//二次矫正的停车位
	commitFix=CommitFix(laserLeftDistance,laserRightDistance);//判断能否进行矫正
	if (fix_status & WAIT_AIM_DIRECTION)//1011 & 0001 
	{
		aimBorder = getAimBorder();
		fix_status &= ~WAIT_AIM_DIRECTION;//1011 & 1110 将此位滞空=1010
		fixAngle=getFixAngle(aimBorder);//矫正角度也是当前靠墙的角度
	} 
	else if (fix_status & TRY_FIRST_FIX)//第一次矫正 1010 & 0010
	{
		if ((fix_status & AGAINST_Wall))//靠墙 1010 & 1000
		{
			AgainstWall(fixAngle,gRobot.pos.angle);
			if (CheckAgainstWall())//检查靠墙
			{
		
				if(commitFix)//当激光没问题
				{
					fixPosFirst(aimBorder);
					map[0]=0;map[1]=0;map[2]=0;map[3]=0;//之前在判断最小距离墙面的时候将原来靠上的那面墙排除比较，现在恢复让其重新比较
					fixSuccessFlag=1;
					//在这里改变状态码
					fix_status=0;
				}else //第二次矫正
				{
					fixPosSec(aimBorder);//矫正当前的角度
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
		gRobot.turnTime=8;
	}
}