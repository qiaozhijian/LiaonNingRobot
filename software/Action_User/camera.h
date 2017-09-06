#ifndef CAMERA_H
#define CAMERA_H

#include "stm32f4xx.h"
#include "usart.h"
#include "misc.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_rcc.h"
#include "timer.h"
#include "stm32f4xx_it.h"
#include "gpio.h"
#include "usart.h"
#include "can.h"
#include "math.h"
#include "stm32f4xx_usart.h"
#include "arm_math.h"
#include "config.h"

int CheckIn(float x, float y, int pointNum, float * peakX, float  * peakY);
int AreaCheck(float x, float y);
void SetTurnTimeChange(int temp);
int GetTurnTimeChange(void);
void CameraBaseWalk3(void);
int CheckIn(float x, float y, int pointNum, float * peakX, float  * peakY);
int AreaCheck(float x, float y);
void SetTurnTimeChange(int temp);
int GetTurnTimeChange(void);


#endif
