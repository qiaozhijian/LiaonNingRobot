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
#include "task.h"
#include "usart.h"
#include "elmo.h"
#include "motor.h"

extern Robot_t gRobot;


void HardWare(void){
	TIM_Init(TIM2, 99, 83, 0, 0); 
	//10ms定时器TIM3用于控制WalkTask周期
	TIM_Init(TIM3, 999, 839, 0, 1);

	//CAN初始化
	CAN_Config(CAN1, 500, GPIOB, GPIO_Pin_8, GPIO_Pin_9);
	CAN_Config(CAN2, 500, GPIOB, GPIO_Pin_5, GPIO_Pin_6);
	
	//激光测距初始化
	Adc_Init();
	//行程开关初始化
	TravelSwitch_Init();
	//投球串口
	ShootUSART1_Init(115200);
	//摄像头串口
	CameraUSART2_Init(115200);
	//定位系统	
	PostionUSART3_Init(115200);
	//蓝牙串口
	TestUART5_Init(115200);
}

void elmoInit(void){
	
	elmo_Init(CAN2);
	elmo_Enable(CAN2, 1);
	elmo_Enable(CAN2, 2);
	
	//收球电机初始化

	Vel_cfg(CAN2, 1, 50000, 50000); //can通信，50000脉冲加速度
	Vel_cfg(CAN2, 2, 50000, 50000);

	
}
void robotInit(void)
{
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
	
	HardWare();
	
	elmoInit();
	
	Delay_ms(6000);
	Delay_ms(6000);
	Vel_cfg(CAN1, COLLECT_BALL_ID, 50000, 50000);
	CollectBallVelCtr(35);
	//Delay_ms(6000);
}

/**
*设置机器人的状态
* 入口参数 status :
*   STATUS_SWEEP 
*   STATUS_CAMERA_WALK 
*   STATUS_CAMERA
*   STATUS_SHOOTER 
*   STATUS_FIX 
*   STATUS_AVOID
*   如果同时设置多个状态，那么各个状态之间按位或
*/
void setRobotStatus(uint8_t status)
{
    gRobot.status = status;
}
/**
*   入口参数    direction 车的方向 
*   GO_STRAIGHT
*   GO_LEFT
*   GO_RIGHT
*   GO_BACK
*/
void setDirection(uint8_t direction)
{
    gRobot.direction = direction;
}
//
void setPosition(Position_t *pos)
{
    gRobot.pos.x = pos->x;
    gRobot.pos.y = pos->y;
    gRobot.pos.angle = pos->angle;
}

void setLeftMove()
{
}

void setRightMove()
{
}
