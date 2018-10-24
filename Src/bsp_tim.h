#ifndef  _BSP_TIM_H_
#define  _BSP_TIM_H_
#include "stm32f4xx.h"
//#include "stm32f4xx_hal_tim.h"

extern TIM_HandleTypeDef tim3_handle_typedef;

void TIM3_Init_Timer(void);


void TIM3_Init_Pwm_CH4(void);
void TIM3_Set_Compare_CH4(uint32_t count);

#endif

