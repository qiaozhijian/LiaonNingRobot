/**
  ******************************************************************************
  * @file    timer.c
  * @author  Calcus Lee
  * @version V1.0.1
  * @date    9-August-2013
  * @brief   functions of time
  ******************************************************************************
  * @attention
  * three are some great and accurency time delay function in timer.h
  *
**/

#include "timer.h"
#include "misc.h"
#include "stm32f4xx_rcc.h"
#include "stm32f4xx_gpio.h"



/**
  * @brief  Configures the TIMx's interrupt time.
  * @param  TIMx: where x can be 1-14. 
			@note TIM1 and TIM 9,10,11;TIM2 and TIM12,13,14 have some sharing IRQ handle functions.
  * @param  arr: the period value to be loaded into the active
                 Auto-Reload Register at the next update event, range from 1 to 65535
  * @param  psr: the prescaler value used to divide the TIM clock, range from 1 to 65535
  * @note   1.the TIMx's PreemptionPriority and SubPriority has been predefined in this function,
			change them according to your need.
			2.Tout= ((arr+1)*(psc+1))/Tclk s, 
			  for TIM2-7 and TIM12-14, Tclk=84M
			  for TIM1,8,9,10,11 Tclk=168M
			  exp: for TIM2, if arr=999, psc=83, then Tout=(1000*84)/84M=1ms
  * @retval None
  * @author Calcus Lee
  */
void TIM_Init(TIM_TypeDef * TIMx, uint16_t arr, uint16_t psr,uint16_t prepri,uint16_t subpri)
{
	TIM_TimeBaseInitTypeDef TIMx_TimeBaseStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
		
	switch((uint32_t)TIMx)
	{
		//APB2 TIM
		case TIM1_BASE:
		{
			RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);
			NVIC_InitStructure.NVIC_IRQChannel=TIM1_UP_TIM10_IRQn ;     
			
			break;
		}
		case TIM8_BASE:
		{
			RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM8, ENABLE);
			NVIC_InitStructure.NVIC_IRQChannel=TIM8_UP_TIM13_IRQn ;    
			
			break;
		}
		case TIM9_BASE:
		{
			RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM9, ENABLE);
			NVIC_InitStructure.NVIC_IRQChannel=TIM1_BRK_TIM9_IRQn ;    
			
			break;
		}
		case TIM10_BASE:
		{
			RCC_APB2PeriphClockCmd(TIM1_UP_TIM10_IRQn, ENABLE);
			NVIC_InitStructure.NVIC_IRQChannel=TIM8_UP_TIM13_IRQn ;    
			
			break;
		}
		case TIM11_BASE:
		{
			RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM11, ENABLE);
			NVIC_InitStructure.NVIC_IRQChannel=TIM1_TRG_COM_TIM11_IRQn ;   
			
			break;
		}
		//APB1 TIM
		case TIM2_BASE:
		{
			RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
			NVIC_InitStructure.NVIC_IRQChannel=TIM2_IRQn ;     

			break;
		}
		case TIM3_BASE:
		{
			RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
			NVIC_InitStructure.NVIC_IRQChannel=TIM3_IRQn ;    

			break;
		}
		case TIM4_BASE:
		{
			RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);
			NVIC_InitStructure.NVIC_IRQChannel=TIM4_IRQn ;    

			break;
		}
		case TIM5_BASE:
		{
			RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM5, ENABLE);
			NVIC_InitStructure.NVIC_IRQChannel=TIM5_IRQn ;     

			break;
		}
		case TIM6_BASE:
		{
			RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM6, ENABLE);
			NVIC_InitStructure.NVIC_IRQChannel=TIM6_DAC_IRQn ;     

			break;
		}
		case TIM7_BASE:
		{
			RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM7, ENABLE);
			NVIC_InitStructure.NVIC_IRQChannel=TIM7_IRQn ;    

			break;
		}
		case TIM12_BASE:
		{
			RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM12, ENABLE);
			NVIC_InitStructure.NVIC_IRQChannel=TIM8_BRK_TIM12_IRQn ;    

			break;
		}
		case TIM13_BASE:
		{
			RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM13, ENABLE);
			NVIC_InitStructure.NVIC_IRQChannel=TIM8_UP_TIM13_IRQn ;   

			break;
		}
		case TIM14_BASE:
		{
			RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM14, ENABLE);
			NVIC_InitStructure.NVIC_IRQChannel=TIM8_TRG_COM_TIM14_IRQn ;     

			break;
		}
				
		default: break;
	}
	
	//定时器TIMx初始化
	TIMx_TimeBaseStructure.TIM_Period=arr;						//设置自动重转载寄存器周期的值
	TIMx_TimeBaseStructure.TIM_Prescaler=psr;        		    //设置时钟分频除数的预分频值
	TIMx_TimeBaseStructure.TIM_ClockDivision=TIM_CKD_DIV1;      //设置时钟分割
	TIMx_TimeBaseStructure.TIM_CounterMode=TIM_CounterMode_Up;  //TIM向上计数
	TIM_TimeBaseInit(TIMx, &TIMx_TimeBaseStructure);            //初始化TIMx
	TIM_ClearITPendingBit(TIMx, TIM_IT_Update);                 //初始化时必须将溢出中断清0,必须在开溢出中断之前
	TIM_ITConfig(TIMx,TIM_IT_Update,ENABLE);                    //允许溢出中断
	//中断优先级NVIC设置
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=prepri;		//抢占优先级
	NVIC_InitStructure.NVIC_IRQChannelSubPriority=subpri;            //从优先级
	NVIC_InitStructure.NVIC_IRQChannelCmd=ENABLE;               //使能IRQ通道
	NVIC_Init(&NVIC_InitStructure);                             //初始化NVIC寄存器
	
	TIM_Cmd(TIMx,ENABLE);                                       //使能TIMx
}

void Delay_us(uint32_t nTime)
{
	SysTick->LOAD=168*nTime;				//装载计数值，因为时钟168M,168次为1us
	SysTick->CTRL=0x00000005;			//时钟来源为HCLK(168M),打开定时器
	while(!(SysTick->CTRL&0x00010000));	//等待计数到0
	SysTick->CTRL=0x00000004;			//关闭定时器
}

void Delay_ms(uint32_t nTime)
{
	for(;nTime>0;nTime--)
		Delay_us(1000);
}

