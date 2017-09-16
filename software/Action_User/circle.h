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
#ifndef __CIRCLE_H
#define __CIRCLE_H



/* Includes ------------------------------------------------------------------*/



/* Exported types ------------------------------------------------------------*/
typedef struct {
	float x;
	float y;
	float R;
}CircleCenter_t;
//定义圆心结构体
typedef struct {
	int checkOnSameLine;//定义一个变量判断是否三个点在同一条直线上
	float x;
	float y;
	float R;
}CircleCenter2_t;
//定义结构体来存储要改变数据的瞬间值
typedef struct {
	 float xstart,ystart,anglestart;
	 float tmpgetAimxfirst,tmpgetAimyfirst;
	 float tmpgetAimxsecond,tmpgetAimysecond;
	 float dis_first2second,dis_start2first;
}Container_t;
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
void NiShiZhenCircleBiHuan(float V,float R,float X0,float Y0);
void ShunShiZhenCircleBiHuan(float V,float R,float X0,float Y0) ;
CircleCenter_t countEatBallWay1(float xBall, float yBall, float xStart, float yStart, float angle);
void Findball_1(void);
void Findball_2(void);
void Findball_3(void);
void Findball_4(void);
void Findball_5(void);
void CameraFindball(int cmodel);
#endif /* ___H */



/************************ (C) COPYRIGHT NEU_ACTION_2017 *****END OF FILE****/

