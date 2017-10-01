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
#define TEST
int main(void)
{
  robotInit();
  while(LaserStart());
  while (1)
  {
    while (getTimeFlag())                              //10ms执行进入一次
    {	
			#ifdef TEST
				TestMode();
			#else
      MotorRead();       
      if (gRobot.status & STATUS_SWEEP)
      {
        Run();
      }
      else if (gRobot.status & STATUS_FIX)
      {
        FixTask();
      }
      else if (gRobot.status & STATUS_SHOOTER)
      {
        fireTask();
      }	
      else if(gRobot.status & STATUS_CAMERA_WALK)
      {
        CameraBaseWalk2();
      }
      else if(gRobot.status&STATUS_AVOID)
      {
        Escape();
      }
			
      if(gRobot.avoid_t.signal)
      {
        CheckOutline();	
      }
      Debug();
			#endif
    }
  }
}

