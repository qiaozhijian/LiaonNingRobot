#ifndef __SWEEP2_H
#define __SWEEP2_H
#define ERR_X 500

#define ERR_Y 367 

#define PULSE 12306
void Target_Angle_X(int Step);
void Target_Angle_Y(int Step);
void PID();
void Angle_f90();
void Angle_0();
void Angle_90();
void Angle_180();
void Sweep2(void);

#endif