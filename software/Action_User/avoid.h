#ifndef AVOID_H
#define AVOID_H
#include "MotionCard.h"

void CheckOutline(void);  			 //避障检测
void CheckOutline2(void);
void BackCarIn(float angle);     //内环倒车
void BackCarOut(float angle);    //外环倒车
void BackCar(float angle);       //倒车程序
int CheckEnemy(void);            //fixtask前检测是否靠墙
int Turn180(void);
int CheckStraddle(Point_t c1, Point_t c2, Point_t b1, Point_t b2);//分别传入车的两边的参数和边界点的参数
int CheckIntersect(float x, float y, float angle, Point_t cP[4]);
void CarPointTrans(float x, float y, float angle, Point_t cP[4]);
float Max(float a, float b);
float Min(float a, float b);
void JudgeStick(void);
#endif
