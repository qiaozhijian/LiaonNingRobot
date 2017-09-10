#ifndef SWEEP_H
#define SWEEP_H

float angleErrorCount(float aimAngle,float angle);//计算角度偏差作为pid

int Vchange(int lineChangeSymbol);
int turnTimeLead(int lineChangeSymbol);

int CheckAgainstWall(void);
int Pointparking(float Pointx,float Pointy);
void AgainstWall(float aimAngle,float angle);
void Sweep(void);
void Debug(void);
void CirlceSweep(void);
void WalkTask1(void);
void WalkTask2(void);
#endif
