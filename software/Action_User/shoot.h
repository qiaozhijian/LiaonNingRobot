#ifndef SHOOT_H
#define SHOOT_H

typedef struct {
	float rev;
	float courceAngle;
	float revrem;
}Launcher_t;
float LauncherPidControl(float ERR);
Launcher_t Launcher(float x,float y,float angle,int ballNum);
void fireTask(void);



#endif


