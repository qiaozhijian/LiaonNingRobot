#ifndef  __TASK_H
#define  __TASK_H
#include "stm32f4xx.h"
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
	//射球的参数
	Shooter_t shooter;
	 //摄像头
	Camera_t camera[10];
	//车的位置
	Position_t pos;

}Robot_t;

#endif
