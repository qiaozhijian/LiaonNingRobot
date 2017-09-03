#ifndef SWEEP_H
#define SWEEP_H

float distancePidControl(float ERR);
float AnglePidControl(float ERR);
float spacingPidControl(float ERR);
float ParkingAnglePidControl(float ERR);
float angleErrorCount(float aimAngle,float angle);//计算角度偏差作为pid

//int LineChange(int lineChangeSymbol);
int Vchange(int lineChangeSymbol);
void CheckOutline(void);//检测是否卡死
void BackCarIn(float angle);//内环倒车程序
void BackCarOut(float angle); //外环倒车程序
void BackCar(float angle);
int CheckAgainstWall(void);
void Pointparking(float Pointx,float Pointy);
void AgainstWall(float aimAngle,float angle);
void Sweep(void);
#endif
