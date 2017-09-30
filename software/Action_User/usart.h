#ifndef __USART_H
#define __USART_H

#include "stdint.h"
#include "stm32f4xx_usart.h"

void  ShootUSART1_Init(uint32_t BaudRate);//串口1
void CameraUSART2_Init(uint32_t BaudRate);//串口2
void PostionUSART3_Init(uint32_t BaudRate);//串口3
void TestUART5_Init(uint32_t BaudRate);//串口5
void UART5_Init(uint32_t BaudRate);
void USART_OUT(USART_TypeDef* USARTx,const char *Data,...);
char *itoa(int value, char *string, int radix);
void USART_OUT_F(float value);

#endif

