/**
  ******************************************************************************
  * @file    .h
  * @author  ACTION_2017
  * @version V0.0.0._alpha
  * @date    2017//
  * @brief   This file contains all the functions prototypes for 
  *          
  ******************************************************************************
  * @attention
  *
  *
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __TOOLS_H
#define __TOOLS_H



/* Includes ------------------------------------------------------------------*/

#include "config.h"
/* Exported types ------------------------------------------------------------*/

/** 
  * @brief  
  */


 
/* Exported constants --------------------------------------------------------*/



/** @defgroup 
  * @{
  */


/**
  * @}
  */

/* Exported macro ------------------------------------------------------------*/
/* Exported functions --------------------------------------------------------*/

/***************************SET***********************/
void setF_ball(int val);
void setBestangle(int8_t val);
void setxRem(float val);
void setyRem(float val);
void setAngle(float val);
void setXposition(float val);
void setYposition(float val);
void setAimxfirst(float val);
void setAimyfirst(float val);
void setAimxsecond(float val);
void setAimysecond(float val);
void setBallColor(int temp);
/***************************GET**********************/
int getF_ball(void);
float getBestangle(void);
float getxRem(void);
float getyRem(void);
float getAimxfirst(void);
float getAimyfirst(void);
float getAimxsecond(void);
float getAimysecond(void);
int getBallColor(void);
/**********************坐标变换**********************/
float Xcoorchange(float x,float y,float angle);
float Ycoorchange(float x,float y,float angle);
float Anglechange(float angle);
/******************读电机速度与位置******************/
void MotorRead(void);
/**********************脉冲转速度********************/
float Pulse2Vel(float Pulse);
/*****************比较两个数的大小*******************/
int TwoNumCompare(float num1,float num2);
/*********************计算两点间的距离***************/
double Dis(float Xstart,float Ystart,float Xstop,float Ystop);
/************************求最大值******************/
float Max(float a,float b);
/************************求最小值******************/
float Min(float a,float b);
/**********************oher tools********************/


#endif /* ___H */



/************************ (C) COPYRIGHT NEU_ACTION_2017 *****END OF FILE****/

