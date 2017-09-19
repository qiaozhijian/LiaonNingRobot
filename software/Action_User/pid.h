#ifndef __PID_H
#define __PID_H


float angleErrorCount(float aimAngle,float angle);
float distancePidControl(float ERR);
float AnglePidControl(float ERR);
float spacingPidControl(float ERR);
float ParkingAnglePidControl(float ERR);
float onceDistancePidControl(float ERR);
void Line(float aimX,float aimY,float aimAngle,int line1,int sign,int lineChangeSymbol);

#endif

