/**
  ******************************************************************************
  * @file	    task.c
  * @author	  Action
  * @version  V1.0.0
  * @date	    2017/07/24
  * @brief	  2017省赛初始化
  ******************************************************************************
  * @attention
  *			None
  ******************************************************************************/
	
	
#include "config.h"
extern Robot_t gRobot;

 /****************************************************************************
* 名    称：HardWare(void)
* 功    能：外设初始化
* 入口参数：无
* 出口参数：无
* 说    明：无
* 调用方法：无 
****************************************************************************/
void HardWare(void){
	TIM_Init(TIM2, 99, 83, 0, 0); 
	//10ms定时器TIM3用于控制WalkTask周期
	TIM_Init(TIM3, 999, 839, 0, 1);
	//1ms定时器用于调用光电门计数函数
	TIM_Init(TIM4, 99, 839, 0, 1);
	//CAN初始化
	CAN_Config(CAN1, 500, GPIOB, GPIO_Pin_8, GPIO_Pin_9);
	CAN_Config(CAN2, 500, GPIOB, GPIO_Pin_5, GPIO_Pin_6);	
	//激光测距初始化
	Adc_Init();
	//蜂鸣器初始化
  BeepInit();
	//光电开关初始化
	PhotoelectricityInit();
	//行程开关初始化
	TravelSwitch_Init();
	//树莓派拉电平
  PullLevel();
	//投球串口
	ShootUSART1_Init(115200);
	//摄像头串口
	CameraUSART2_Init(115200);
	//定位系统	
	PostionUSART3_Init(115200);
	//蓝牙串口
	UART5DMAInit(921600);
}
 /****************************************************************************
* 名    称：elmoInit()
* 功    能：电机初始化
* 入口参数：无
* 出口参数：无
* 说    明：无
* 调用方法：无 
* 注意：
****************************************************************************/
void elmoInit(void){
	
	elmo_Init(CAN2);
	elmo_Enable(CAN2, 1);
	elmo_Enable(CAN2, 2);
	
	Vel_cfg(CAN2, 1, 50000, 50000); //can通信，50000脉冲加速度
	Vel_cfg(CAN2, 2, 50000, 50000);

	
}
/****************************************************************************
* 名    称：variableInit)
* 功    能：gRobot初始化
* 入口参数：无
* 出口参数：无
* 说    明：为某些特定要附初值的变量提供便利与统一
* 调用方法：无 
* 注    意：每个变量名要分好类，不要混淆
****************************************************************************/
void variableInit(void)
{
	//大状态初始值为run
	gRobot.status=25; 
	//车的初始方向默认为逆时针
	gRobot.walk_t.circleChange.direction=0;
	gRobot.walk_t.circleChange.linenum=-1;
	//??赋值
	gRobot.walk_t.board[0][0]=0.f;
}
 /****************************************************************************
* 名    称：robotInit()
* 功    能：机器人初始化
* 入口参数：无
* 出口参数：无
* 说    明：无
* 调用方法：无 
****************************************************************************/
void robotInit(void)
{
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
	
	HardWare();
	
	elmoInit();
	
	variableInit();

	Delay_ms(8000);
  Delay_ms(7000);
  Vel_cfg(CAN1, COLLECT_BALL_ID, 50000, 50000);
   CollectBallVelCtr(60);                                       //让辊子转起来
}
