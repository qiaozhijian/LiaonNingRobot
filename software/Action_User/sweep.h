#ifndef SWEEP_H
#define SWEEP_H

float angleErrorCount(float aimAngle,float angle);//计算角度偏差作为pid

int turnTimeLead(int lineChangeSymbol);           //计算提前量

int CheckAgainstWall(void);                       //检测是否靠墙
int Pointparking(float Pointx,float Pointy);      //定点停车
void AgainstWall(float aimAngle,float angle,float spacingError);    //靠墙
void In2Out(void);  //刚开始的基础扫场程序
void In2Out2(void);
void In2Out3(void);
void Out2In(void);
int AntiSquare3(void);
int Square3(void);
int Square4(void);
int AntiSquare4(void);
void Debug(void);                                 //debug                        
void Vchange(int lineChangeSymbol);               //速度与脉冲的转换
int LaserStart(void);                             //激光启动
void WalkOne(void);
void Run(void);
void ClockWise(void);                             //顺时针行驶
void AntiClockWise(void);                         //逆时针行驶
void Xgoal(float aimX,float aimY,float aimAngle,int sign,int lineChangeSymbol);//走x=?
void Ygoal(float aimX,float aimY,float aimAngle,int sign,int lineChangeSymbol);//走y=?

int Square(void);
int AntiSquare(void);
int Circle(float vel,float r); 
int AntiCircle(float vel,float r);
int Square2(void);
int AntiSquare2(void);

void AngleRoute(float aimangle);

#endif
