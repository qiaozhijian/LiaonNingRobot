/**
  ******************************************************************************
  * @file    usart.cpp
  * @author  Luo Xiaoyi 
  * @version V1.0
  * @date    2016.10.26
  * @brief   ���ڿ��ƴ���
  ******************************************************************************
  * @attention
  *
  *
  *
  * 
  ******************************************************************************
  */ 
/* Includes -------------------------------------------------------------------*/
#include "usart.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_usart.h"
#include "string.h"
#include "stdio.h"
#include "stm32f4xx_rcc.h"
#include "stm32f4xx_dma.h"
#include "misc.h"
#include "arm_math.h"
#include "string.h"
/* Private  typedef -----------------------------------------------------------*/
/* Private  define ------------------------------------------------------------*/
#define DMA_SEND_SIZE   40
/* Private  macro -------------------------------------------------------------*/
/* Private  variables ---------------------------------------------------------*/


static uint8_t dmaSendBuffer[DMA_SEND_SIZE];

/* Extern   variables ---------------------------------------------------------*/
/* Extern   function prototypes -----------------------------------------------*/
/* Private  function prototypes -----------------------------------------------*/
/* Private  functions ---------------------------------------------------------*/

void USART_SendDataToDMA(uint8_t data)
{
	static uint8_t tempBuffer[DMA_SEND_SIZE];
	static uint32_t count=0;
	tempBuffer[count]=data;
	count++;
	
	if(count>=DMA_SEND_SIZE)
	{
		while(DMA_GetITStatus(DMA1_Stream7,DMA_IT_TCIF7) == RESET	&&	DMA_GetCmdStatus(DMA1_Stream7)	==	ENABLE	);    
		DMA_ClearFlag(DMA1_Stream7,DMA_IT_TCIF7);  
		DMA_Cmd(DMA1_Stream7,DISABLE);  
		count=0;
		memcpy(dmaSendBuffer,tempBuffer,DMA_SEND_SIZE);
		DMA_SetCurrDataCounter(DMA1_Stream7,DMA_SEND_SIZE);
		DMA_Cmd(DMA1_Stream7,ENABLE);
	}
	
}

void UART5_Init(uint32_t BaudRate)
{
	NVIC_InitTypeDef NVIC_InitStructure;
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	/* USARTx configured as follow:
	- BaudRate = 57600 baud  
	- Word Length = 8 Bits
	- One Stop Bit
	- No parity
	- Hardware flow control disabled (RTS and CTS signals)
	- Receive and transmit enabled
	*/
	USART_InitStructure.USART_BaudRate = BaudRate;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
//------------------------------------------------------------
	 
  /* Enable GPIO clock */
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC | RCC_AHB1Periph_GPIOD, ENABLE);

  RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART5, ENABLE);

  //?????????
  USART_DeInit(UART5);  //????5
	
  /* Connect PXx to USARTx_Tx*/
  GPIO_PinAFConfig(GPIOC, GPIO_PinSource12, GPIO_AF_UART5);
  /* Connect PXx to USARTx_Rx*/
  GPIO_PinAFConfig(GPIOD, GPIO_PinSource2,  GPIO_AF_UART5);

  /* Configure USART Tx as alternate function  */
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;

  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOC, &GPIO_InitStructure);

  /* Configure USART Rx as alternate function  */
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
  GPIO_Init(GPIOD, &GPIO_InitStructure);

  /* USART configuration */
  USART_Init(UART5, & USART_InitStructure);
  
	//////////   ??UART5??       ///////////////
	NVIC_InitStructure.NVIC_IRQChannel = UART5_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
	NVIC_Init(&NVIC_InitStructure);

	USART_ITConfig(UART5, USART_IT_RXNE, ENABLE);
	/* Enable USART */
	USART_Cmd(UART5, ENABLE);
}

void UART5DMAInit(uint32_t BaudRate)
{
	DMA_InitTypeDef DMA_InitStructure;
	NVIC_InitTypeDef 	NVIC_InitStructure;

	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA1, ENABLE);

	UART5_Init(BaudRate);

	DMA_DeInit(DMA1_Stream7);
	while (DMA_GetCmdStatus(DMA1_Stream7) != DISABLE){}

	DMA_InitStructure.DMA_Channel = DMA_Channel_4;
	DMA_InitStructure.DMA_PeripheralBaseAddr =(uint32_t)&(UART5->DR); // peripheral address, = & USART5->DR;
	DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)dmaSendBuffer;	// memory address to save DMA data
	DMA_InitStructure.DMA_DIR = DMA_DIR_MemoryToPeripheral;				// data dirction:  memory to peripheral
	DMA_InitStructure.DMA_BufferSize = 0;					//the buffer size, in data unit
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;//8 bit data
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;		//8 bit data  32??MCU?1?half-word?16 bits
	DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
	DMA_InitStructure.DMA_Priority = DMA_Priority_High;
	DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;
	DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_HalfFull;
	DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;
	DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
	DMA_Init(DMA1_Stream7, &DMA_InitStructure);

	USART_DMACmd(UART5,USART_DMAReq_Tx,ENABLE);
	DMA_ITConfig(DMA1_Stream7,DMA_IT_TC,ENABLE);

	NVIC_InitStructure.NVIC_IRQChannel = DMA1_Stream7_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=2;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority =2;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
}

void DMA1_Stream7_IRQHandler(void)
{
	if(DMA_GetITStatus(DMA1_Stream7, DMA_IT_TCIF7) != RESET)
	{
		DMA_Cmd(DMA1_Stream7, DISABLE);
		DMA_ClearITPendingBit(DMA1_Stream7, DMA_IT_TCIF7);
	}
}
/* Exported function prototypes -----------------------------------------------*/
/* Exported functions ---------------------------------------------------------*/
/**
  * @brief   ��ʼ������������
  * @none    none
  * @retval  none
  */
void USART1_DMA_INIT(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	DMA_InitTypeDef DMA_InitStructure;
	
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA,ENABLE); //ʹ��GPIOAʱ��
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1,ENABLE);//ʹ��USART1ʱ��
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2, ENABLE);  
	
	DMA_DeInit(DMA2_Stream7);  
	while (DMA_GetCmdStatus(DMA2_Stream7) != DISABLE){}
		
	DMA_InitStructure.DMA_Channel = DMA_Channel_4;   
	DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)(&USART1->DR);  
	DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)dmaSendBuffer;  
	DMA_InitStructure.DMA_DIR = DMA_DIR_MemoryToPeripheral;   
	DMA_InitStructure.DMA_BufferSize = DMA_SEND_SIZE;  
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;  
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;  
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;  
	DMA_InitStructure.DMA_MemoryDataSize = DMA_PeripheralDataSize_Byte;  
	DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;  
	DMA_InitStructure.DMA_Priority = DMA_Priority_High;  
		 
	DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;      
	DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_HalfFull;          
	DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;         
	DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;   	 
	DMA_Init(DMA2_Stream7, &DMA_InitStructure);    
//	DMA_ITConfig(DMA2_Stream7,DMA_IT_TC,ENABLE);  	

	DMA_ClearFlag(DMA2_Stream7,DMA_IT_TCIF7);  
	DMA_Cmd(DMA2_Stream7,DISABLE);
	
	//����1��Ӧ���Ÿ���ӳ��
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource10,GPIO_AF_USART1); //GPIOA9����ΪUSART1
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource9, GPIO_AF_USART1); //GPIOA10����ΪUSART1
	
	//USART1�˿�����
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10 | GPIO_Pin_9; //GPIOA9��GPIOA10
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;//���ù���
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	//�ٶ�50MHz
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; //���츴�����
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP; //����
	GPIO_Init(GPIOA,&GPIO_InitStructure); //��ʼ��PA9��PA10

   //USART1 ��ʼ������
	USART_InitStructure.USART_BaudRate = 115200;//����������
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;//�ֳ�Ϊ8λ���ݸ�ʽ
	USART_InitStructure.USART_StopBits = USART_StopBits_1;//һ��ֹͣλ
	USART_InitStructure.USART_Parity = USART_Parity_No;//����żУ��λ
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//��Ӳ������������
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//�շ�ģʽ
  USART_Init(USART1, &USART_InitStructure); //��ʼ������1

	//Usart1 NVIC ����
  NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;//����1�ж�ͨ��
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=1;//��ռ���ȼ�3
	NVIC_InitStructure.NVIC_IRQChannelSubPriority =0;		//�����ȼ�3
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQͨ��ʹ��
	NVIC_Init(&NVIC_InitStructure);	//����ָ���Ĳ�����ʼ��VIC�Ĵ�����
	USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);//��������ж�
	USART_ClearFlag(USART1, USART_FLAG_TC);
	USART_ClearFlag(USART1, USART_FLAG_TXE);
	
	
	USART_DMACmd(USART1,USART_DMAReq_Tx,ENABLE);
	USART_Cmd(USART1, ENABLE);  //ʹ�ܴ���3
}

/**
	* @brief   ����<<�������ַ����������䷢�͵�dma
	* @param   value: ��Ҫ�������ַ���
	* @retval  usart_io
  */
void USART_OUT_CHAR(const char* value) 
{
	uint8_t len=0;
	while(1)
	{
		if(value[len]!='\0')
		  USART_SendDataToDMA(value[len]);
		else
			break;
		len++;
		
		if(len>100)
			break;
	}
}
/**
	* @brief   ����<<�������������ݣ������䷢�͵�dma
	* @param   value: ��Ҫ��������������
	* @retval  usart_io
  */
void USART_OUT_INT(const int32_t value) 
{
	char buf[10];
	uint8_t len;
	len=sprintf(buf,"%d",value);
	for(uint8_t i=0;i<len;i++)
	{
		USART_SendDataToDMA(buf[i]);
	}
	USART_OUT_CHAR("\t");
}
void USART_OUT_UINT(const uint32_t value) 
{
	char buf[10];
	uint8_t len;
	len=sprintf(buf,"%u",value);
	for(uint8_t i=0;i<len;i++)
	{
		USART_SendDataToDMA(buf[i]);
	}
	USART_OUT_CHAR("\t");
}
/**
	* @brief   ����<<���������������ݣ������䷢�͵�dma
	* @param   value: ��Ҫ�����ĸ���������
	* @retval  usart_io
  */
void USART_OUTF(const float value) 
{
	char buf[20];
	uint8_t len;
	len=sprintf(buf,"%f",value);
	for(uint8_t i=0;i<len;i++)
	{
		USART_SendDataToDMA(buf[i]);
	}
	USART_OUT_CHAR("\t");
}


/************************ (C) COPYRIGHT 2016 ACTION *****END OF FILE****/
