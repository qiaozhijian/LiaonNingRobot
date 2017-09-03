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
#include "elmo.h"
#include "sweep.h"
#include "task.h"
#include "circle.h"
#include "camera.h"
#include "shoot.h"
#include "tools.h"

void init(void)
{
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
	
	TIM_Init(TIM2, 999, 83, 0, 0); //��ʱ1ms
	ShootUSART1_Init(115200);
	CameraUSART2_Init(115200);//串口2
	PostionUSART3_Init(115200);//串口3
	TestUART5_Init(115200);

	//1ms定时器用于控制WalkTask周期
	TIM_Init(TIM2, 99, 839, 0, 0);
	//10ms定时器TIM3用于控制WalkTask周期
	TIM_Init(TIM3, 999, 839, 0, 0);

	//adc初始化
	Adc_Init();
	TravelSwitch_Init();
	//CAN初始化
	CAN_Config(CAN1, 500, GPIOB, GPIO_Pin_8, GPIO_Pin_9);
	CAN_Config(CAN2, 500, GPIOB, GPIO_Pin_5, GPIO_Pin_6);

	//驱动器初始化
//	elmo_Init(CAN1);
//	elmo_Enable(CAN1, 1);
//	elmo_Enable(CAN1, 2);
	
		elmo_Init(CAN2);
		elmo_Enable(CAN2, 1);
		elmo_Enable(CAN2, 2);

	//配置速度环
//	Vel_cfg(CAN1, 1, 50000, 50000); //can通信，50000脉冲加速度
//	Vel_cfg(CAN1, 2, 50000, 50000);
		Vel_cfg(CAN2, 1, 50000, 50000); //can通信，50000脉冲加速度
		Vel_cfg(CAN2, 2, 50000, 50000);
		Vel_cfg(CAN1, COLLECT_BALL_ID, 50000,50000);
		Delay_ms(12000);
//	VelCrl(CAN1, 1, 5000);//can通信，电机，转速，4096一秒一转，
//	VelCrl(CAN1, 2, 5000);//can通信，电机，转速，顺时针为正
}
//globle 变量
Robot_t gRobot;
int main(void)
{
	init();

	while (1)
	{
		while (getTimeFlag())//10ms执行进入一次
		{
	//		switch (gRobot.status)
	//		{
	//			case STATUS_SWEEP:
	//				Sweep();
	//			break;
	//			
	//			case STATUS_CAMERA_WALK:
	//				switch(getF_ball())
	//					{
	//						case 0:
	//							Pointparking(1200,2400);
	//							CameraBaseWalk3();
	//						break;
	//						
	//						case 1:
	//							CameraFindball(4);
	//						break;
	//						
	//						default://USART_OUT();
	//						break;
	//					}

	//			break;
	//					
	//			case STATUS_FIX :
	//				
	//			break;
	//				
	//			case STATUS_AVOID:
	//				
	//			break;
	//			
	//			case STATUS_SHOOTER:
	//				fireTask();
	//			break;
	//			
	//			default:
	//				
	//			break;
	//		}			
			Sweep();
		}
	}
}
