/**
******************************************************************************
* @file	    main.c
* @author	  Action
* @version  V1.0.0
* @date	    2017/07/24
* @brief	  Whirling Death(死亡旋风)...
******************************************************************************
* @attention
*			None
******************************************************************************/
#include "config.h"
Robot_t gRobot;
//#define TEST 
int main(void)
{
  robotInit();
 // while(LaserStart());
  while (1)
  {
    while (getTimeFlag())                              //10ms执行进入一次
    {	
			USART_OUT(UART5,"%d",(int)gRobot.walk_t.circleChange.linenum);
			USART_OUT(UART5,"%d\t",(int)gRobot.walk_t.pos.x);
			USART_OUT(UART5,"%d\t",(int)gRobot.walk_t.pos.y);
			USART_OUT(UART5,"%d\t",(int)gRobot.walk_t.right.aim);
			USART_OUT(UART5,"%d\t",(int)gRobot.walk_t.left.aim);
			USART_OUT(UART5,"%d\t",(int)gRobot.walk_t.circleChange.circleNum);
			USART_OUT(UART5,"%d\r\n",(int)gRobot.walk_t.circleChange.turnTime);
			AntiClockWise();
//			#ifdef TEST 
//				TestMode();
//			#else
//      MotorRead();  
//			//1在处理异常  为0就判断
//			if(!(gRobot.status&STATUS_AVOID))
//      {
//				if()
//        //judge()   jin cheng  进程的异常情况  得出结论
//			}
			
			if(gRobot.status&STATUS_AVOID){
//				Escape();//进程  异常情况  处理
//			}	else	
//			if (gRobot.status & STATUS_SWEEP)
//      {
//        Run();
//      }
//      else if (gRobot.status & STATUS_FIX)
//      {
//        FixTask();
//      }
//      else if (gRobot.status & STATUS_SHOOTER)
//      {
//        fireTask();
//      }	
//      else if(gRobot.status & STATUS_CAMERA_WALK)
//      {
//        CameraBaseWalk2();
//      }
//      

//			
////      if(gRobot.avoid_t.signal)
////      {
////        CheckOutline();	
////      }
//      Debug();
//			#endif
    }
  }
}
}
