#ifndef  __TASK_H
#define  __TASK_H
#include "config.h"

/*************************PID调解***********************/
typedef struct{
		float disError;
		float angleError;
		float distanceStraight;
}Pid_t;
/******************走行进程结构体***********************/
//轮子状态 速度和调节量  
typedef struct{
	float aim;
	float base;
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
	//车的位置
	Position_t pos;
	//Pid调解
	Pid_t pid;
	//走形的切换
	int turntime;	
}Walk_t;


/******************走行进程结构体***********************/

//发射电机参数
typedef struct {
	float angle;
	float speed;
//电机速度达到所给速度标志位
	int VelAchieve;
}ShootPara_t;

typedef struct{
	ShootPara_t aim;
	//实际射球参数
	ShootPara_t real;
}Shoot_t;

/******************走行进程结构体***********************/
//激光距离
typedef struct {
	int left;
	int right;
}Laser_t;

typedef struct{
	//激光的左右距离
	Laser_t laser;
}Fix_t;

/******************走行进程结构体***********************/
//摄像头
typedef struct {
	int8_t angle;
	uint8_t dis;
}CamrBallPara_t;


typedef struct {
 //摄像头
	CamrBallPara_t camrPara[10];
}CamrCatch_t;

typedef struct {
	int a;
	
}CamrBaseWalk_t;

typedef struct{
	CamrCatch_t camrCatch_t;
	CamrBaseWalk_t camrBaseWalk_t;
	Pid_t camerapid;
}Camera_t;

/******************走行进程结构体***********************/
typedef struct{
		int a;
		Pid_t pid;
}Avoid_t;

/******************走行进程结构体***********************/
//收球电机参数
typedef struct {
	float angle;
	float speed;
//电机速度达到所给速度标志位
	int VelAchieve;
}CollectPara_t;

typedef struct{
	CollectPara_t aim;
	//实际射球参数
	CollectPara_t real;
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
	
	int turnTime; 			//	分开

}Robot_t;



void robotInit(void);

#endif
