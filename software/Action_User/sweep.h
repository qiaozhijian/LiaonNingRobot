#ifndef SWEEP_H
#define SWEEP_H

float angleErrorCount(float aimAngle,float angle);//计算角度偏差作为pid

int turnTimeLead(int lineChangeSymbol);           //计算提前量

int CheckAgainstWall(void);                       //检测是否靠墙
int Pointparking(float Pointx,float Pointy);      //定点停车
void AgainstWall(float aimAngle,float angle);     //靠墙
void In2Out(int lineChangeSymbol);                //刚开始的基础扫场程序
void Debug(void);                                 //debug                        
void Vchange(int lineChangeSymbol);               //速度与脉冲的转换
int LaserStart(void);                             //激光启动
void WalkOne(void);
void Run(void);
#endif
