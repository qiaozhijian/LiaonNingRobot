#ifndef __DEBUG_H
#define __DEBUG_H
void d_Coor(void);
void d_fireTask(int ballNum,int waitAdjust,float courceAngle,float rev);
void d_Line(int turnTime,int lineChangeSymbol,float disError,float angleError,float distanceStraight,float turnTimeLead);
void d_Sub_Box(int Ball_counter,int maxFirstlayer,int c_Aimxfirst,int Aimxfirst,int Aimyfirst,int Aimxsecond,int Aimysecond,int maxSecondlayer);
#endif
