#include "debug.h"
#include "config.h"
extern Robot_t gRobot;
void d_Coor(void)
{
		USART_OUTF(gRobot.pos.angle);
		USART_OUTF(gRobot.pos.x);
		USART_OUTF(gRobot.pos.y);
		USART_OUT_CHAR("\r\n");
}
void d_fireTask(int ballNum,int waitAdjust,float courceAngle,float rev)
{
		USART_OUTF(ballNum);
		USART_OUTF(waitAdjust);
		USART_OUTF(courceAngle);
		USART_OUTF(rev);
		USART_OUT_CHAR("\r\n");
}
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
void d_Sub_Box(int Ball_counter,int maxFirstlayer,int c_Aimxfirst,int Aimxfirst,int Aimyfirst,int Aimxsecond,int Aimysecond,int maxSecondlayer)
{
		USART_OUTF(Ball_counter);
		USART_OUTF(maxFirstlayer);
		USART_OUTF(c_Aimxfirst);
		USART_OUTF(Aimyfirst);
		USART_OUTF(Aimyfirst);
		USART_OUTF(Aimxsecond);
		USART_OUTF(Aimysecond);
		USART_OUTF(maxSecondlayer);
		USART_OUT_CHAR("\r\n");
}
