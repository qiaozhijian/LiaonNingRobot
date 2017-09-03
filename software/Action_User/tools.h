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
void setBasket(int val);
void setF_ball(int val);
void setBestangle(float val);
void setxRem(float val);
void setyRem(float val);
void setAngle(float val);
void setXposition(float val);
void setYposition(float val);
void setAimxfirst(float val);
void setAimyfirst(float val);
void setAimxsecond(float val);
void setAimysecond(float val);

/***************************GET**********************/
int getBasket(void);
int getF_ball(void);
float getBestangle(void);
float getxRem(void);
float getyRem(void);
float getAimxfirst(void);
float getAimyfirst(void);
float getAimxsecond(void);
float getAimysecond(void);
/**********************坐标变换**********************/
float Xcoorchange(float x,float y,float angle);
float Ycoorchange(float x,float y,float angle);
float Anglechange(float angle);
/**********************oher tools********************/
double Dis(float Xstart,float Ystart,float Xstop,float Ystop);
#endif /* ___H */



/************************ (C) COPYRIGHT NEU_ACTION_2017 *****END OF FILE****/

