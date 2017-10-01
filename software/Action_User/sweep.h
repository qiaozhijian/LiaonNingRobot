#ifndef SWEEP_H
#define SWEEP_H

float angleErrorCount(float aimAngle,float angle);//计算角度偏差作为pid

int turnTimeLead(int lineChangeSymbol);           //计算提前量

int CheckAgainstWall(void);                       //检测是否靠墙
int Pointparking(float Pointx,float Pointy);      //定点停车
void AgainstWall(float aimAngle,float angle,float spacingError);    //靠墙
void In2Out(int lineChangeSymbol,int direction);  //刚开始的基础扫场程序
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
int Circle(void); 
int AntiCircle(void);
int Square2(void);
int AntiSquare2(void);

void AngleRoute(float aimangle);

#endif
