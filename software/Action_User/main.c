#include "config.h"
//globle全局变量
Robot_t gRobot;
int rightadc,leftadc;
int main(void)
{
	robotInit();
	//ReadActualPos(CAN1,GUN_YAW_ID);
	while (1)
	{
		while (getTimeFlag()) //10ms执行进入一次
		{
					//USART_OUT(UART5,(uint8_t*)"%d\t%d\r\n",rightadc,leftadc);

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
			
			/**************临时测试********/
			//			WalkTask1();
			//			WalkTask2();
			//			逆时针旋转
			       CirlceSweep();
			//			NiShiZhenCircleBiHuan(1200,1600,2400,2400);
			//			USART_OUTF(Key1);
			//			USART_OUTF(Key2);
			//			fireTask();
			//      Sub_Box();
			//			Findball_3();
			//			Findball_5();
      //			Debug();
		}
	}
}

