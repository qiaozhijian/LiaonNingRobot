#include "debug.h"
#include "config.h"
extern Robot_t gRobot;
void d_Coor(void)
{
		USART_OUT(UART5, (uint8_t *)"%d\t", (int)gRobot.pos.angle);
		USART_OUT(UART5, (uint8_t *)"%d\t", (int)gRobot.pos.x);
		USART_OUT(UART5, (uint8_t *)"%d\t\r\n", (int)gRobot.pos.y);
}
void d_fireTask(int ballNum,int waitAdjust,float courceAngle,float rev)
{
		USART_OUT(UART5, (uint8_t *)"%d\t", (int)ballNum);
		USART_OUT(UART5, (uint8_t *)"%d\t", (int)waitAdjust);
		USART_OUT(UART5, (uint8_t *)"%d\t", (int)courceAngle);
		USART_OUT(UART5, (uint8_t *)"%d\t\r\n", (int)rev);
}
void d_Line(int turnTime,int lineChangeSymbol,float disError,float angleError,float distanceStraight,float turnTimeLead)
{
		USART_OUT(UART5, (uint8_t *)"%d\t", (int)gRobot.turnTime);
		USART_OUT(UART5, (uint8_t *)"%d\t", (int)lineChangeSymbol);
		USART_OUT(UART5, (uint8_t *)"%d\t", (int)disError);
		USART_OUT(UART5, (uint8_t *)"%d\t", (int)angleError);
		USART_OUT(UART5, (uint8_t *)"%d\t", (int)distanceStraight);
		USART_OUT(UART5, (uint8_t *)"%d\t\r\n", (int)turnTimeLead);
}
void d_Sub_Box(int Ball_counter,int maxFirstlayer,int c_Aimxfirst,int Aimxfirst,int Aimyfirst,int Aimxsecond,int Aimysecond,int maxSecondlayer)
{
		USART_OUT(UART5, (uint8_t *)"%d\t", (int)Ball_counter);
		USART_OUT(UART5, (uint8_t *)"%d\t", (int)maxFirstlayer);
		USART_OUT(UART5, (uint8_t *)"%d\t", (int)c_Aimxfirst);
		USART_OUT(UART5, (uint8_t *)"%d\t", (int)Aimyfirst);
		USART_OUT(UART5, (uint8_t *)"%d\t", (int)Aimyfirst);
		USART_OUT(UART5, (uint8_t *)"%d\t", (int)Aimxsecond);
		USART_OUT(UART5, (uint8_t *)"%d\t", (int)Aimysecond);
		USART_OUT(UART5, (uint8_t *)"%d\t\r\n", (int)maxSecondlayer);
}
