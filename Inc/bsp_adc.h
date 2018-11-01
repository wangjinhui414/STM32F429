#ifndef _BSP_ADC_H_
#define _BSP_ADC_H_
#include "stm32f4xx.h"
#include "stm32f4xx_hal.h"
// #include "stm32f4xx_hal_adc.h"
void ADC_Init(void);

uint32_t Get_ADC_Value(uint32_t ch);
uint32_t Get_ADC_AverageValue(uint32_t ch,uint16_t times);

double Get_Temprate(void);










#endif