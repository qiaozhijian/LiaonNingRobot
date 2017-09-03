#ifndef FIX_H
#define FIX_H




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

int getLeftAdc();
int getRightAdc();

#endif
