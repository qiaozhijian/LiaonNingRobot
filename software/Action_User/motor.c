#include "motor.h"
#include "stm32f4xx.h"
#include "arm_math.h"
#include "gpio.h"
#include "usart.h"
#include "can.h"
#include "stm32f4xx_it.h"
#include "stm32f4xx_usart.h"
#include "elmo.h"
#include "config.h"
//发射航向角转换函数 由度转换为脉冲
float YawTransform(float yawAngle)
{
	return (yawAngle * YAW_REDUCTION_RATIO * COUNT_PER_DEGREE);
}

//发射航向角控制函数 单位：度（枪顺时针转为正，逆时针为负）
void YawAngleCtr(float yawAngle)
{

	PosCrl(CAN1, GUN_YAW_ID, POS_ABS, YawTransform(yawAngle));
}


//送弹推球函数
void PushBall(void)
{
	PosCrl(CAN1, PUSH_BALL_ID, POS_ABS, PUSH_POSITION);
}

//送弹推球收回函数
void PushBallReset(void)
{
	PosCrl(CAN1, PUSH_BALL_ID, POS_ABS, PUSH_RESET_POSITION);
}

//收球电机速度转换函数 由转每秒转换为脉冲
float CollectBallVelTrans(float round)
{
	return round * COUNT_PER_ROUND;
}

//收球电机速度控制函数 单位：转每秒
void CollectBallVelCtr(float round)
{
	VelCrl(CAN1,COLLECT_BALL_ID,CollectBallVelTrans(round));
}


//发射电机速度转换函数 由转每秒转换为脉冲
int32_t shootVelTrans(float roundPerS)
{
	return (int32_t)-roundPerS * COUNT_PER_ROUND;
}

//发射电机速度控制函数 单位：转每秒
void ShootCtr(float rps)
{
    shootPara_t shootPara;
	
	shootPara.velInt32 = shootVelTrans(rps);

    //起始位
    USART_SendData(USART1, 'A');
    //通过串口1发数
    USART_SendData(USART1, shootPara.velUint8[0]);
    USART_SendData(USART1, shootPara.velUint8[1]);
    USART_SendData(USART1, shootPara.velUint8[2]);
    USART_SendData(USART1, shootPara.velUint8[3]);
    //终止位
    USART_SendData(USART1, 'J');
}

