#ifndef _BSP_USART_H_
#define _BSP_USART_H_
#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_uart.h"
#include "string.h"

typedef struct TYPEDEF_UART_RX
{
		uint8_t pcRxbuf[200];
		uint8_t rofset;
		uint32_t lastTick;
		uint8_t rflag;
}TYPEDEF_UART_RX;

extern TYPEDEF_UART_RX pcUsart2RX;
extern UART_HandleTypeDef uart2_hand_typedef;

void USART2_Init(void);
void ResetUartrRx(TYPEDEF_UART_RX *puartrx);


#define __DEBUG__

#ifdef __DEBUG__
    #define DEBUG(format,...) printf("%s:"format"",__func__, ##__VA_ARGS__)
#else
    #define DEBUG(format,...)
#endif

#define LOG(format,...) printf(""format"", ##__VA_ARGS__)

#define ERROR(format,...) printf("ERROR:%s %s %d: "format"",__FILE__, __func__,__LINE__, ##__VA_ARGS__)
#endif
