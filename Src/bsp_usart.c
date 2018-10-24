#include "bsp_usart.h"
#include "stdarg.h"


UART_HandleTypeDef uart2_hand_typedef;

uint8_t aRxbuf[1] = {0};


TYPEDEF_UART_RX pcUsart2RX = {{0},0,0,0};

void ResetUartrRx(TYPEDEF_UART_RX *puartrx)
{
	 memset(puartrx->pcRxbuf,0,200);
	 puartrx->rofset = 0;
	 puartrx->lastTick = 0;
	 puartrx->rflag = 0;
}


void USART2_Init(void)
{
		uart2_hand_typedef.Instance=USART2;					    //USART2
		uart2_hand_typedef.Init.BaudRate=115200;				    
		uart2_hand_typedef.Init.WordLength=UART_WORDLENGTH_8B;   
		uart2_hand_typedef.Init.StopBits=UART_STOPBITS_1;	    
		uart2_hand_typedef.Init.Parity=UART_PARITY_NONE;		    
		uart2_hand_typedef.Init.HwFlowCtl=UART_HWCONTROL_NONE;   
		uart2_hand_typedef.Init.Mode=UART_MODE_TX_RX;		    
		HAL_UART_Init(&uart2_hand_typedef);					    

		HAL_UART_Receive_IT(&uart2_hand_typedef, (uint8_t *)aRxbuf, 1);
}

void HAL_UART_MspInit(UART_HandleTypeDef *huart)
{
	GPIO_InitTypeDef GPIO_Initure;
	
	if(huart->Instance==USART2)
	{
		__HAL_RCC_GPIOA_CLK_ENABLE();			
		__HAL_RCC_USART2_CLK_ENABLE();			
	
		GPIO_Initure.Pin=GPIO_PIN_2;			
		GPIO_Initure.Mode=GPIO_MODE_AF_PP;		
		GPIO_Initure.Pull=GPIO_PULLUP;			
		GPIO_Initure.Speed=GPIO_SPEED_FAST;		
		GPIO_Initure.Alternate=GPIO_AF7_USART2;	
		HAL_GPIO_Init(GPIOA,&GPIO_Initure);	   	

		GPIO_Initure.Pin=GPIO_PIN_3;			
		HAL_GPIO_Init(GPIOA,&GPIO_Initure);	   	
		
		HAL_NVIC_EnableIRQ(USART2_IRQn);		
		HAL_NVIC_SetPriority(USART2_IRQn,3,3);	
	}

}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if(huart->Instance==USART2)
	{
		    if(pcUsart2RX.rflag == 0)
				{
					pcUsart2RX.pcRxbuf[pcUsart2RX.rofset++] = aRxbuf[0];
					pcUsart2RX.lastTick = HAL_GetTick();
				}
	}
}
 

void USART2_IRQHandler(void)                	
{
		uint32_t timeout=0;
		uint32_t maxDelay=0x1FFFF;	
	
		HAL_UART_IRQHandler(&uart2_hand_typedef);	
		timeout=0;
		while (HAL_UART_GetState(&uart2_hand_typedef) != HAL_UART_STATE_READY)
		{
				timeout++;
				if(timeout>maxDelay) break;		
		}
		timeout=0;
		while(HAL_UART_Receive_IT(&uart2_hand_typedef, (uint8_t *)aRxbuf, 1) != HAL_OK)
		{
				timeout++; 
				if(timeout>maxDelay) break;	
		}
}

void HAL_SYSTICK_Callback(void)
{
		if((HAL_GetTick() - pcUsart2RX.lastTick) > 30  && pcUsart2RX.rflag == 0 && pcUsart2RX.lastTick!=0)
		{
				pcUsart2RX.rflag = 1;
		}
}

/*************************************************
Functions that printf must implement
arm-none-eabi-gcc    _write
mdk                  fputc
**************************************************/
#define ARM_NONE_EABI
#ifdef ARM_NONE_EABI
	int _write (int fd, char *pBuffer, int size)
	{
		for (int i = 0; i < size; i++)
		{
			while((USART2->SR&0X40)==0);   
			USART2->DR = pBuffer[i];
		}
		return size;
	}
#else
	int fputc(int ch, FILE *f)
	{ 	
		while((USART2->SR&0X40)==0);   
		USART2->DR = (uint8_t) ch;
		return ch;
	}
#endif

