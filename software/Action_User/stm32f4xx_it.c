/**
  ******************************************************************************
  * @file    Project/STM32F4xx_StdPeriph_Template/stm32f4xx_it.c 
  * @author  MCD Application Team
  * @version V1.0.1
  * @date    13-April-2012
  * @brief   Main Interrupt Service Routines.
  *          This file provides template for all exceptions handler and 
  *          peripherals interrupt service routine.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT 2012 STMicroelectronics</center></h2>
  *
  * Licensed under MCD-ST Liberty SW License Agreement V2, (the "License");
  * You may not use this file except in compliance with the License.
  * You may obtain a copy of the License at:
  *
  *        http://www.st.com/software_license_agreement_liberty_v2
  *
  * Unless required by applicable law or agreed to in writing, software 
  * distributed under the License is distributed on an "AS IS" BASIS, 
  * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
  * See the License for the specific language governing permissions and
  * limitations under the License.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_it.h"
#include "stm32f4xx.h"
#include "can.h"
#include "config.h"
#include "fix.h"
#include "task.h"
#include "sweep.h"
#include "tools.h"
/******************************************************************************/
/*            Cortex-M4 Processor Exceptions Handlers                         */
/******************************************************************************/

/************************************************************/
extern Robot_t gRobot;
float angle;				//定义角度
float posX   = 0;	 	//定位系统返回的X坐标
float posY   = 0;	 	//定位系统返回的Y坐标
/****************CAn***start******************/
typedef union{
	uint8_t buffer[8];
	int32_t receivebuff[2];
}Msg_t;
void CAN1_RX0_IRQHandler(void)
{
	static Msg_t Msg; 
	static uint8_t length=8;
	static uint32_t StdId;
	CAN_RxMsg(CAN1,&StdId,Msg.buffer,length);
	
	if(StdId==0x30)
	{
		setBallColor(Msg.buffer[0]);
	}
	else if(StdId==0x280+GUN_YAW_ID)
	{
		if(Msg.receivebuff[0]==0x00005856)
		{
			gRobot.Yawvel=Msg.receivebuff[1];
		}
		else if(Msg.receivebuff[0]==0x00005850)
		{
			//得到位置信息
			gRobot.Yawangle=(Msg.receivebuff[1]);
			USART_OUT(UART5,(uint8_t*)"%d",Msg.receivebuff[1]);
		}
	}
	//USART_OUT(UART5,(uint8_t*)"%d\r\n",buffer[0]);
	CAN_ClearFlag(CAN1, CAN_FLAG_EWG);
	CAN_ClearFlag(CAN1, CAN_FLAG_EPV);
	CAN_ClearFlag(CAN1, CAN_FLAG_BOF);
	CAN_ClearFlag(CAN1, CAN_FLAG_LEC);
	CAN_ClearFlag(CAN1, CAN_FLAG_FMP0);
	CAN_ClearFlag(CAN1, CAN_FLAG_FF0);
	CAN_ClearFlag(CAN1, CAN_FLAG_FOV0);
	CAN_ClearFlag(CAN1, CAN_FLAG_FMP1);
	CAN_ClearFlag(CAN1, CAN_FLAG_FF1);
	CAN_ClearFlag(CAN1, CAN_FLAG_FOV1);
}

/**
  * @brief  CAN2 receive FIFO0 interrupt request handler
  * @note
  * @param  None
  * @retval None
  */
void CAN2_RX0_IRQHandler(void)
{
	static uint32_t StdId;
	static uint8_t canReceice[8];
	static uint8_t len=8;
	CAN_RxMsg(CAN2,&StdId,canReceice,len);

	CAN_ClearFlag(CAN2, CAN_FLAG_EWG);
	CAN_ClearFlag(CAN2, CAN_FLAG_EPV);
	CAN_ClearFlag(CAN2, CAN_FLAG_BOF);
	CAN_ClearFlag(CAN2, CAN_FLAG_LEC);
	CAN_ClearFlag(CAN2, CAN_FLAG_FMP0);
	CAN_ClearFlag(CAN2, CAN_FLAG_FF0);
	CAN_ClearFlag(CAN2, CAN_FLAG_FOV0);
	CAN_ClearFlag(CAN2, CAN_FLAG_FMP1);
	CAN_ClearFlag(CAN2, CAN_FLAG_FF1);
	CAN_ClearFlag(CAN2, CAN_FLAG_FOV1);
}

/****************������CAN1�ӿ�ģ��****end******************/
/************************************************************/

/*************��ʱ��2******start************/

void UART5_IRQHandler(void)
{
//	uint8_t data = 0;
	if(USART_GetFlagStatus(UART5,USART_FLAG_ORE)!=RESET){
//		data=USART_ReceiveData(UART5);
	}
	else if(USART_GetITStatus(UART5, USART_IT_RXNE)==SET)   
	{
		USART_ClearITPendingBit( UART5,USART_IT_RXNE);
//		data=USART_ReceiveData(UART5);
	}
	 
}
typedef union
{
    //这个32位整型数是给电机发送的速度（脉冲/s）
    int32_t velInt32;
    //通过串口发送数据每次只能发8位
    uint8_t velUint8[4];

}BackShootTest_t;
int stopUSARTsignal=0;
int32_t nowShootVel=0;
void USART1_IRQHandler(void)
{
	static int i=0,j=0;
	uint8_t data = 0;
	BackShootTest_t backShootTest ;
	if(USART_GetFlagStatus(USART1,USART_FLAG_ORE)!=RESET){
		data=USART_ReceiveData(USART1);
		if(data=='V')
		{
			i=0;
			nowShootVel=backShootTest.velInt32;
		}else if(data=='R')
		{
			i=0;
			stopUSARTsignal=1;
		}
		switch (i)
		{
			case 0:
				if(data=='A')
				i=1;
				break;
				
			case 1:
				backShootTest.velUint8[j]=data;
				j++;
			  if(j>4)
				{
					j=0;
					i=0;
				}
			break;
			
			default://USART_OUT();
			break;
		}
	}
	else if(USART_GetITStatus(USART1, USART_IT_RXNE)==SET)   
	{
		USART_ClearITPendingBit( USART1,USART_IT_RXNE);
		data=USART_ReceiveData(USART1);
	}
	
	//USART_OUT(UART5,(uint8_t*)"%d\r\n",(int)backShootTest.velInt32);
}


void USART3_IRQHandler(void) //更新频率200Hz
{
	static uint8_t ch;
	static union {
		uint8_t data[24];
		float ActVal[6];
	} posture;
	static uint8_t count = 0;
	static uint8_t i = 0;
	if (USART_GetITStatus(USART3, USART_IT_RXNE) == SET)
	{
		USART_ClearITPendingBit(USART3, USART_IT_RXNE);
		ch = USART_ReceiveData(USART3);
		switch (count)
		{
		case 0:
			if (ch == 0x0d)
				count++;
			else
				count = 0;
			break;

		case 1:
			if (ch == 0x0a)
			{
				i = 0;
				count++;
			}
			else if (ch == 0x0d)
				;
			else
				count = 0;
			break;

		case 2:
			posture.data[i] = ch;
			i++;
			if (i >= 24)
			{
				i = 0;
				count++;
			}
			break;

		case 3:
			if (ch == 0x0a)
				count++;
			else
				count = 0;
			break;

		case 4:
			if (ch == 0x0d)
			{

				angle = -posture.ActVal[0] ;//角度
//				USART_OUT(UART5,(uint8_t*)"%d\r\n",(int)posture.ActVal[0]);
				posture.ActVal[1] = posture.ActVal[1];
				posture.ActVal[2] = posture.ActVal[2];
				posX = posture.ActVal[3];//x
				posY = posture.ActVal[4];//y
				posture.ActVal[5] = posture.ActVal[5];
				setXpos(posX);
				setYpos(posY);
				setAngle(angle);
			}
			count = 0;
			
//			if(CheckAgainstWall())
//			{
//				
//			}
				
			gRobot.pos.x=getXpos();
			gRobot.pos.y=getYpos();
			gRobot.pos.angle=getAngle();
			break;

		default:
			count = 0;
			break;
		}
	}
	else
	{
		USART_ClearITPendingBit(USART3, USART_IT_PE);
		USART_ClearITPendingBit(USART3, USART_IT_TXE);
		USART_ClearITPendingBit(USART3, USART_IT_TC);
		USART_ClearITPendingBit(USART3, USART_IT_ORE_RX);
		USART_ClearITPendingBit(USART3, USART_IT_IDLE);
		USART_ClearITPendingBit(USART3, USART_IT_LBD);
		USART_ClearITPendingBit(USART3, USART_IT_CTS);
		USART_ClearITPendingBit(USART3, USART_IT_ERR);
		USART_ClearITPendingBit(USART3, USART_IT_ORE_ER);
		USART_ClearITPendingBit(USART3, USART_IT_NE);
		USART_ClearITPendingBit(USART3, USART_IT_FE);
		USART_ReceiveData(USART3);
	}
}
//树莓派接收图片帧程序程序
int Ball_counter=0;
uint8_t tmp;
extern Robot_t gRobot;
void USART2_IRQHandler(void)
{
	static int Ball_tmpcounter=0;
	static int i=0,	j=0;
	static int flag=0;
	if (USART_GetITStatus(USART2, USART_IT_RXNE) == SET)
	{
	USART_ClearITPendingBit(USART2, USART_IT_RXNE);
		tmp=USART_ReceiveData(USART2);
	  USART_SendData(UART5,tmp);
/****************球最多的角度****************/
if(LEVEL==3)
{
		if(tmp==0xDA||flag)
		{
			flag++;
			if(flag==2)
			{
				setBestangle(tmp); 
				flag=0;
			}
//			else USART_OUT();
		}
	}
/***************所有球的角度和距离***********/
else if(LEVEL==4)
{
			if(tmp==CAMERA_STATUS_5_END)
		{
			i=0;
			j=0;
			//当Ball_tmpcounter为0时表明已经没球了
			Ball_tmpcounter=Ball_counter;
			setF_ball(Ball_tmpcounter);
		}
		switch (i)
		{
			case 0:
				if(tmp==CAMERA_STATUS_4_START)
				i++;
				break;
			case 1:
				gRobot.camera[j].angle=tmp;
			  i++;
				break;
			case 2:  
				gRobot.camera[j].dis=tmp;
 				i=1;
				j++;
			Ball_counter++;
				break;
			default://USART_OUT();
				break;
		}
	}
}
}

/**
  * @brief   This function handles NMI exception.
  * @param  None
  * @retval None
  */
void NMI_Handler(void)
{
   while (1)
   {
   }
}

/**
  * @brief  This function handles Hard Fault exception.
  * @param  None
  * @retval None
  */
void HardFault_Handler(void)
{

  /* Go to infinite loop when Hard Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Memory Manage exception.
  * @param  None
  * @retval None
  */
void MemManage_Handler(void)
{
  /* Go to infinite loop when Memory Manage exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Bus Fault exception.
  * @param  None
  * @retval None
  */
void BusFault_Handler(void)
{

  /* Go to infinite loop when Bus Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Usage Fault exception.
  * @param  None
  * @retval None
  */
void UsageFault_Handler(void)
{
 
  /* Go to infinite loop when Usage Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles SVCall exception.
  * @param  None
  * @retval None
  */
void SVC_Handler(void)
{
}

/**
  * @brief  This function handles Debug Monitor exception.
  * @param  None
  * @retval None
  */
void DebugMon_Handler(void)
{
}

