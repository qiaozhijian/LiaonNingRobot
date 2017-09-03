#include "stm32f4xx.h"
#include "usart.h"
#include "misc.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_rcc.h"
#include "timer.h"
#include "stm32f4xx_it.h"
#include "gpio.h"
#include "usart.h"
#include "can.h"
#include "math.h"
#include "stm32f4xx_usart.h"
#include "arm_math.h"
#include "config.h"

//摄像头
typedef struct Camera_t{
	uint8_t mode;
}Camera;

//激光距离
typedef struct Laser_t{
	int leftDistance;
	int rightDistance;
}Laser;

//发射电机参数
typedef struct Shooter_t{
	float angle;
	float speed;
}Shooter;

//位置
typedef struct Position_t{
	float angle;
	float x;
	float y;
}Pos;

//轮子状态 速度和调节量  
typedef struct Move_t{
	float speed;
	float adjust;
}Move;

//机器人结构体
typedef struct Robot_t{
	//机器人的状态
	uint8_t status;
	//车的方向
	uint8_t direction;
	//左轮状态
	Move left;	
	//右轮状态
	Move right;
	//激光的左右距离
	Laser laser;
	//射球的参数
	Shooter shooter;
	 //摄像头
	Camera camera;
	//车的位置
	Pos pos;

}Robot;

//globle 变量
static Robot robot;

void robotInit()
{

}

void setRobotStatus(uint8_t status)
{

}

void setDirection(uint8_t direction)
{

}

void setLeftMove()
{

}

void setRightMove()
{

}