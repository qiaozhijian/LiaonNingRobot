#ifndef AVOID_H
#define AVOID_H


void CheckOutline(void);  			 //避障检测
void CheckOutline2(void);
void BackCarIn(float angle);     //内环倒车
void BackCarOut(float angle);    //外环倒车
void BackCar(float angle);       //倒车程序
int CheckEnemy(void);            //fixtask前检测是否靠墙
int Turn180(void);
#endif
