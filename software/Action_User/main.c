#include "config.h"
//globle全局变量
Robot_t gRobot={0};
int rightadc,leftadc;
int main(void)
{
	robotInit();
	ReadActualVel(CAN1,COLLECT_BALL_ID);
//ReadActualPos(CAN1);

	gRobot.status=25;
	//ReadActualPos(CAN1,GUN_YAW_ID);
	while (1)
	{
		while (getTimeFlag()) //10ms执行进入一次
		{
						if (gRobot.status & STATUS_SWEEP)
						{
							In2Out();
						}
						else if (gRobot.status & STATUS_FIX)
						{
								FixTask();
								ShootCtr(60);
						}
						else if (gRobot.status & STATUS_SHOOTER)
						{
							fireTask();
						}
					
//						else if(gRobot.status& STATUS_CAMERA_WALK)
//						{

//						}
//						else if (gRobot.status & STATUS_CAMERA)
//						{
//			
//						}
//		
			//			else if(gRobot.status&STATUS_AVOID)
			//			{

			//			}

			
			/**************临时测试********/
			//			WalkTask1();
			//			WalkTask2();
			//			逆时针旋转
			//       CirlceSweep();
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

