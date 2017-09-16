#ifndef  __TASK_H
#define  __TASK_H
#include "config.h"
//摄像头
typedef struct {
	int8_t angle;
	uint8_t dis;
}Camera_t;

//激光距离
typedef struct {
	int leftDistance;
	int rightDistance;
}Laser_t;

//发射电机参数
typedef struct {
	float angle;
	float speed;
//电机速度达到所给速度标志位
	int VelAchieve;
}Shooter_t;

//位置
typedef struct {
	float angle;
	float x;
	float y;
}Position_t;

//轮子状态 速度和调节量  
typedef struct{
	float speed;
	float adjust;
}Move_t;

//机器人结构体
typedef struct {
	//机器人的状态
	uint8_t status;
	//车的方向
	uint8_t direction;
	//左轮状态
	Move_t left;
	//右轮状态
	Move_t right;
	//激光的左右距离
	Laser_t laser;
	//白球射球参数
	Shooter_t Wshooter;
	//黑球射球参数
	Shooter_t Bshooter;
	//实际射球参数
	Shooter_t RealShooter;
	 //摄像头
	Camera_t camera[10];
	//车的位置
	Position_t pos;
	
	float M;
	
	int turnTime;
	//定义射球装置角度与距离
	int Yawangle;
	int Yawvel;

}Robot_t;

void robotInit(void);

#endif
