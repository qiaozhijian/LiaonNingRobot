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
#include "motor.h"

//globle 变量
Robot_t gRobot;
int main(void)
{
	robotInit();
	while (1)
	{
		while (getTimeFlag()) //10ms执行进入一次
		{
			ReadActualVel(CAN2,1);
			ReadActualVel(CAN2,2);
			//			if (gRobot.status & STATUS_SWEEP)
			//			{
			//				//执行扫场
			//			}
			//			else if(gRobot.status& STATUS_CAMERA_WALK)
			//			{

			//			}
			//			else if (gRobot.status & STATUS_CAMERA)
			//			{
			//			}
			//			else if (gRobot.status & STATUS_FIX)
			//			{

			//			}
			//			else if(gRobot.status&STATUS_AVOID)
			//			{

			//			}

			//			if (gRobot.status & STATUS_SHOOTER)
			//			{
			//				//执行射球
			//			}
			//				WalkTask1();
		//	WalkTask2();
				CirlceSweep();

//			CameraBaseWalk3();
//			NiShiZhenCircleBiHuan(1200,1600,2400,2400);
			//				USART_OUT(UART5, (uint8_t *)"%d\t", (int)Key1);
			//				USART_OUT(UART5, (uint8_t *)"%d\t", (int)Key2);
			//		fireTask();
			
//			Debug();
		}
	}
}

