#ifndef  __TASK_H
#define  __TASK_H
#include "config.h"
#include "MotionCard.h"
/******************矫正结构体***********************/
void driveGyro(void);
void DisDriveGyro(void);
//激光距离
typedef struct {
	int left;
	int right;
	//激光启动
	int status;
	
}Laser_t;

typedef struct{
	//激光的左右距离
	Laser_t laser;
	//知道矫正完投球在哪个边界
	int inBorder;
	//去哪一个边界
	int toBorder;
	
	int wayChoose;
	
}Fix_t;
/*************************PID调解***********************/
//PID调解的相关参数
typedef struct{
		float aimAngle;
		float disError;
		float angleError;
		float distanceStraight;
		int pidtime;
}Pid_t;
/******************走行进程结构体***********************/
//机器人缩圈标志
typedef struct{
	//记录当前姿态
	int turnTime;
	//记录上一次的姿态
	int turnTimerem;
	//车再那一边
	int linenum;
	//记录圈数
	int circleNum;
	int quadrantlast;
	//车所再象限
	int quadrant;
	//车的顺逆方向
	int direction;
}CircleChange_t;
//轮子状态 速度和调节量  
typedef struct{
	float aim;
	float real;
	float adjust;
}MovePara_t;

//位置
typedef struct {
	float angle;
	float x;
	float y;
}Position_t;
typedef struct{
	
	//左轮状态
	MovePara_t left;
	//右轮状态
	MovePara_t right;
	float base;
	//车的位置
	Position_t pos;
	//Pid调解
	Pid_t pid;
	//走形的切换
	CircleChange_t circleChange;
	//激光启动
	Laser_t laser;
	//正方形区域  x1 y1 x2 y2的顺序
	float board[4][4];
	//位移计算量 
	float averageV;
}Walk_t;


/******************射球进程***********************/

/******************推球电机结构体***********************/
typedef struct{
	float pos;
	float posrem;
	float error;
}PushPara_t;

//发射电机参数
typedef struct {
	//航向角
	float angle;
	//航向角度达到所给速度标志位
	int angleAchieve;
	//发射速度
	float speed;
	//电机速度达到所给速度标志位
	int velAchieve;
	//射了的球的个数
	int shootNum;
}ShootPara_t;

typedef struct{
	ShootPara_t sAim;
	//实际射球参数
	ShootPara_t sReal;
	
	PushPara_t pReal;
	
	PushPara_t pAim;
	
	int startSignal;
	
	Point_t shootPos;
	
	Laser_t lastLaser;
	
}Shoot_t;

/******************摄像头结构体***********************/
//摄像头
typedef struct {
	int8_t angle;                //摄像头下球的角度()
	uint8_t dis;                 //摄像头下球到摄像头的距离(cm)
}CamrBallPara_t;


typedef struct {
 //摄像头
	CamrBallPara_t camrPara[10];  //摄像头数据
}CamrCatch_t;

typedef struct {
	int findturn;
	CircleChange_t circleChange;
}CamrBaseWalk_t;

typedef struct{
	CamrCatch_t camrCatch_t;
	CamrBaseWalk_t camrBaseWalk_t;
	Pid_t camerapid;
}Camera_t;

/******************避障结构体***********************/
typedef struct{
	float x;
	float y;
	float angle;
}PosRem_t;

typedef struct{
	int signal;				//开启逃逸与关闭逃逸检测函数Chechoutline()
	int passflag;     //每经过backcare一次被置1
	Pid_t pid;				//判断逃逸倒车后的角度与目标角度的差值
	PosRem_t posRem;  //记录进入backcare时的姿态
	int continueTriggerSignal;
	int continueTrigger;
	int statusRem;//连续触发瞬间进入矫正进程让它记住当前的大状态码
	int direction;
	int handleEnd;
}Avoid_t;

/******************辊子收球结构体***********************/
//光电门数球程序
typedef struct{
  int ballcount;
}PhotoElectric_t;
//收球电机参数
typedef struct {
	float angle;
	float speed;     //辊子的速度
	int velAchieve;  //电机速度达到所给速度标志位
}CollectPara_t;

typedef struct{
	CollectPara_t aim;  //辊子收球目标
	CollectPara_t real; //辊子当前姿态
	PhotoElectric_t PhotoElectric;
}Collect_t;

/******************机器人结构体*********************/
typedef struct {
	/******************进程选择***********************/
	uint8_t status;
	/******************射球进程***********************/	
	Shoot_t shoot_t;
	/******************走行进程***********************/
	Walk_t walk_t;
	/******************矫正进程***********************/
	Fix_t fix_t;
	/******************摄像头进程***********************/	
	Camera_t camera_t;
	/******************避障进程***********************/		
	Avoid_t avoid_t;
	/******************收球进程***********************/
	Collect_t collect_t;
	/*******************定点停车进程********************/
	Point_t ParkingPoint;
	/********************等待定位系统的反馈**************/
	int gpsSignal;
	/********************等待彻底启动**************/
	int start;
	/******************初始化检查硬件链接***************/
	int check;
	
	int correctSide;
	
	int angle;
	
	int speed;
//	/*场地区域数组*/
//  uint32_t area[];
	uint32_t abnormal;
}Robot_t;



void robotInit(void);

#endif
