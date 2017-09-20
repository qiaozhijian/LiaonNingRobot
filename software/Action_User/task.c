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

	//CAN初始化
	CAN_Config(CAN1, 500, GPIOB, GPIO_Pin_8, GPIO_Pin_9);
	CAN_Config(CAN2, 500, GPIOB, GPIO_Pin_5, GPIO_Pin_6);
//	
	//激光测距初始化
	Adc_Init();
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
	UART5DMAInit(115200);
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
* 名    称：robotInit
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
	
	Delay_ms(6000);
	Delay_ms(6000);
//	Vel_cfg(CAN1, COLLECT_BALL_ID, 50000, 50000);
//	CollectBallVelCtr(55);
//	Delay_ms(6000);                                         //让辊子转起来
	
}
