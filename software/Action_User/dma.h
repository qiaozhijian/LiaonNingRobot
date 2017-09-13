/**
  ******************************************************************************
  * @file    dma.h
  * @author  ACTION_2017
  * @version V0.0.0._alpha
  * @date    2017/04/29
  * @brief   This file contains all the functions prototypes for DMA
  *          
  ******************************************************************************
  * @attention
  *
  *
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __DMA_H
#define __DMA_H


void USART1_DMA_INIT(void);
void USART_OUT_INT(const int32_t value) ;
void USART_OUT_UINT(const uint32_t value) ;
void USART_OUTF(const float value) ;
void USART_OUT_CHAR(const char* value);
void UART5DMAInit(uint32_t BaudRate);
#endif /* __DMA_H */

