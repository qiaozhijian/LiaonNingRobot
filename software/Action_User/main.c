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
void init(void)
{
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);	
	  
	TIM_Init(TIM2,999,83,0,0);					//¶¨Ê±1ms	
	
	
}

int main(void)
{
	init();
	
	while(1)
	{
	while(getTimeFlag())
	{

		
	}
	}
}

