#ifndef __MOTOR_H
#define  __MOTOR_H

#include "stm32f4xx.h"

typedef union
{
    //这个32位整型数是给电机发送的速度（脉冲/s）
    int32_t velInt32;
    //通过串口发送数据每次只能发8位
    uint8_t velUint8[4];

}shootPara_t;
//发射航向角转换函数 由度转换为脉冲
float YawTransform(float yawAngle);
//发射航向角控制函数 单位：度（枪顺时针转为正，逆时针为负）
void YawAngleCtr(float yawAngle);
//送弹推球函数
void PushBall(void);

void PushBallReset(void);

//收球电机速度转换函数 由转每秒转换为脉冲
float CollectBallVelTrans(float round);


//收球电机速度控制函数 单位：转每秒
void CollectBallVelCtr(float round);

//发射电机速度转换函数 由转每秒转换为脉冲
int32_t shootVelTrans(float roundPerS);
//发射电机速度控制函数 单位：转每秒
void ShootCtr(float rps);

#endif
