#ifndef __GPIO_H
#define __GPIO_H

#include "stm32f4xx_gpio.h"

#define RESET_SWITCH  			(GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_2))
//#define KEYSWITCH		    	(GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_0))
#define PHOTOSENSORLEFT 		(GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_0))
#define PHOTOSENSORRIGHT 		(GPIO_ReadInputDataBit(GPIOE, GPIO_Pin_8))
#define PHOTOSENSORUPGUN 		(GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_6))
#define PHOTOSENSORLEFTGUN 		(GPIO_ReadInputDataBit(GPIOE, GPIO_Pin_12))
#define PHOTOSENSORRIGHTGUN 	(GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_8))
#define BEEP_ON          		 GPIO_SetBits(GPIOE, GPIO_Pin_7)
#define BEEP_OFF         		 GPIO_ResetBits(GPIOE, GPIO_Pin_7)
#define BLUE_LED_ON				GPIO_ResetBits(GPIOE, GPIO_Pin_6)
#define RED_LED_ON				GPIO_ResetBits(GPIOC, GPIO_Pin_0)
#define BLUE_LED_OFF			GPIO_SetBits(GPIOE, GPIO_Pin_6)
#define RED_LED_OFF				GPIO_SetBits(GPIOC, GPIO_Pin_0)

#define ballVacant                  (GPIO_ReadInputDataBit(GPIOE, GPIO_Pin_8)) //没被挡住是1，被挡住是0
#define TRAVEL_SWITCH_LEFT		    	(GPIO_ReadInputDataBit(GPIOE, GPIO_Pin_2))
#define TRAVEL_SWITCH_RIGHT         (GPIO_ReadInputDataBit(GPIOC,GPIO_Pin_0))

#define LEVEL 4
void GPIO_Init_Pins(GPIO_TypeDef * GPIOx,
					uint16_t GPIO_Pin,
					GPIOMode_TypeDef GPIO_Mode);

void KeyInit(void);
void LEDInit(void);
void PhotoelectricityInit(void);
void BeepInit(void);
void PhotoelectricityInit(void);
void  Adc_Init(void);
void TravelSwitch_Init(void);
void PullLevel(void);
//
#endif
