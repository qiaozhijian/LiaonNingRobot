#include "debug.h"
#include "config.h"
extern Robot_t gRobot;
void d_getVel(void)
{
	//开启航向电机接收
	ReadActualPos(CAN1,GUN_YAW_ID);
	ReadActualVel(CAN1,GUN_YAW_ID);
}
void d_Coor(void)
{
		USART_OUTF(gRobot.pos.angle);
		USART_OUTF(gRobot.pos.x);
		USART_OUTF(gRobot.pos.y);
		USART_OUT_CHAR("\r\n");
}
extern float rightadc,leftadc;
extern float b_realvel;
extern float b_realangle;
extern float w_realvel;
extern float w_realangle;
//void d_fireTask(int ballNum,int waitAdjust,float courceAngle,float rev)
//{
////		USART_OUTF(ballNum);
////		USART_OUTF(waitAdjust);
//		USART_OUTF(gRobot.pos.angle);
//		USART_OUTF(gRobot.pos.x);
//		USART_OUTF(gRobot.pos.y);
//		USART_OUTF(leftadc);
//		USART_OUTF(rightadc);
//		USART_OUTF(courceAngle);
//		USART_OUTF(rev);
//		USART_OUT_CHAR("\r\n");
//}
void d_Line(int turnTime,int lineChangeSymbol,float disError,float angleError,float distanceStraight,float turnTimeLead)
{
		USART_OUTF(gRobot.turnTime);
		USART_OUTF(lineChangeSymbol);
		USART_OUTF(disError);
		USART_OUTF(angleError);
		USART_OUTF(distanceStraight);
		USART_OUTF(turnTimeLead);
		USART_OUT_CHAR("\r\n");
}
void d_Sub_Box(int Ball_counter,int maxFirstlayer,int c_Aimxfirst,int Aimxfirst,int Aimyfirst,int c_Aimxsecond,int Aimxsecond,int Aimysecond,int maxSecondlayer)
{
		USART_OUTF(Ball_counter);
		USART_OUTF(maxFirstlayer);
		USART_OUTF(c_Aimxfirst);
		USART_OUTF(Aimxfirst);
		USART_OUTF(Aimyfirst);
		USART_OUTF(maxSecondlayer);
		USART_OUTF(c_Aimxsecond);
		USART_OUTF(Aimxsecond);
		USART_OUTF(Aimysecond);
		USART_OUT_CHAR("\r\n");
}
extern int ballNum;
void d_fireTask()
{
		USART_OUTF(gRobot.pos.angle);
//		USART_OUTF(gRobot.pos.x);
//		USART_OUTF(gRobot.pos.y);
		USART_OUTF(leftadc);
		USART_OUTF(rightadc);
		USART_OUT_INT(ballNum);
		USART_OUTF(b_realvel);
		USART_OUTF(b_realangle);
		USART_OUTF(w_realvel);
		USART_OUTF(w_realangle);
	  
		USART_OUT_CHAR("\r\n");
}
