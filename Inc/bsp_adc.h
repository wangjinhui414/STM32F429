#ifndef _BSP_ADC_H_
#define _BSP_ADC_H_
#include "stm32f4xx.h"
#include "stm32f4xx_hal.h"

void ADC_Init(void);

uint16_t Get_ADC_Value(uint8_t ch);
uint16_t Get_ADC_AverageValue(uint8_t ch,uint16_t times);










#endif