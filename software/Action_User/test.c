#include "config.h"
#include "test.h"

extern Robot_t gRobot;

int a=0;
void TestMode(void){

//			a=getBallColor();
/*****************************************¡Ÿ ±≤‚ ‘*****************************************/
	USART_OUT(UART5,"%d\t",(int)gRobot.camera_t.camerapid.aimAngle);
	USART_OUT(UART5,"%d\t",(int)gRobot.walk_t.pos.angle);
	USART_OUT(UART5,"%d\t",(int)gRobot.walk_t.pos.x);
	USART_OUT(UART5,"%d\t",(int)gRobot.walk_t.pos.y);
	USART_OUT(UART5,"%d\t",(int)gRobot.collect_t.PhotoElectric.ballcount);						
	USART_OUT(UART5,"%d\r\n",(int)gRobot.avoid_t.signal);
							
	USART_OUT(UART5,"%d\t",(int)gRobot.avoid_t.passflag);	
	USART_OUT(UART5,"%d\t",(int)angleErrorCount(gRobot.avoid_t.pid.aimAngle,gRobot.walk_t.pos.angle));
							
	USART_OUT(UART5,"%d\t",(int)gRobot.walk_t.pos.angle);
	USART_OUT(UART5,"%d\t",(int)gRobot.walk_t.pos.x);
	USART_OUT(UART5,"%d\t",(int)gRobot.walk_t.pos.y);			
							
	USART_OUT(UART5,"%d\t",(int)gRobot.avoid_t.pid.aimAngle);	
	//USART_OUT(UART5,"%d\t%d\t%d\t%d\t",(int)gRobot.walk_t.right.real,gRobot.status,gRobot.avoid_t.signal,(int)gRobot.walk_t.right.aim);
	USART_OUT(UART5,"%d\r\n",(int)gRobot.walk_t.circleChange.turnTime);
	USART_OUT(UART5,"%d\r\n",(int)gRobot.walk_t.circleChange.turnTimerem);
	USART_OUT(UART5,"%d\r\n",(int)gRobot.status);

				//			WalkTask1();
				//			WalkTask2();
				//			ƒÊ ±’Î–˝◊™
				//       CirlceSweep();
				//			NiShiZhenCircleBiHuan(1200,1600,2400,2400);
				//			USART_OUTF(Key1);
				
				//			USART_OUTF(Key2);
				//			fireTask();
				//      Sub_Box();
				//			Findball_3();
				//			Findball_5();
				//			Debug();
				//USART_OUT(UART5,"%d\r\n",(int)gRobot.shoot_t.sReal.speed);
/*****************************************¡Ÿ ±≤‚ ‘*****************************************/			

}

void Debug(void){

			USART_OUT(UART5,"%d\t", (int)gRobot.status);
			USART_OUT(UART5,"%d\t",(int)gRobot.walk_t.circleChange.turnTime);
			USART_OUT(UART5,"%d\t",(int)gRobot.walk_t.pos.angle);
		  USART_OUT(UART5,"%d\t",(int)gRobot.walk_t.pos.x);
		  USART_OUT(UART5,"%d\t",(int)gRobot.walk_t.pos.y);
			USART_OUT(UART5,"%d\t",(int)LineCheck(gRobot.walk_t.circleChange.direction));
   		USART_OUT(UART5,"%d\t",(int)gRobot.collect_t.PhotoElectric.ballcount);
		  USART_OUT(UART5,"%d\t",(int)gRobot.camera_t.camrBaseWalk_t.circleChange.turnTime);
		  USART_OUT(UART5,"%d\t",(int)gRobot.walk_t.circleChange.direction);
		  USART_OUT(UART5,"%d\t",(int)gRobot.camera_t.camrBaseWalk_t.circleChange.turnTime);
			USART_OUT(UART5,"%d\t",(int)gRobot.walk_t.circleChange.linenum);
			USART_OUT(UART5,"%d\t",(int)gRobot.walk_t.pid.aimAngle );
			USART_OUT(UART5,"%d\t",(int)gRobot.walk_t.pid.angleError);
		  USART_OUT(UART5,"%d\t",(int)gRobot.walk_t.pid.disError);
			USART_OUT(UART5,"%d\t",(int)gRobot.walk_t.pid.distanceStraight);
			USART_OUT(UART5,"%d\t",(int)gRobot.shoot_t.sReal.speed);
			USART_OUT(UART5,"%d\r\n",(int)gRobot.walk_t.circleChange.circlenum);


}

