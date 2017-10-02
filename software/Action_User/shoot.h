#ifndef SHOOT_H
#define SHOOT_H

float LauncherPidControl(float ERR);
void fireTask(void);
void CheckComingCar(float leftLaser,float rightLaser);
void LiuLeLiuLe(void);//此时应该躲避重新投球//放入异常判断处理
#endif


