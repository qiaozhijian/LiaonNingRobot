#ifndef FIX_H
#define FIX_H

#include "stdint.h"


void setAngle(float val);
void setXpos(float val);
void setYpos(float val);
void setErrSingle(float reaAngle);
void setErrX(float realX);
void setErrY(float realy);
float getAngle(void);
float getXpos(void);
float getYpos(void);
void setErr(float reaAngle,float realX,float realy);
int getLeftAdc(void);
int getRightAdc(void);


typedef struct{
	float x;
	float y;
}AimPos_t;

int getAimBorder(void);

typedef struct{
		float angle;
		float spacingError;
}FixPara_t;
FixPara_t getFixPara(int aimBorder);

int CommitFix(int laserLeftDistance,int laserRightDistance);

void fixPosFirst(int aimBorder);

void fixPosSec(int aimBorder);

AimPos_t Go2NextWall(int aimBorder);

void FixTask(void);

#endif
