#include "bsp_tim.h"

TIM_HandleTypeDef tim3_handle_typedef;
TIM_OC_InitTypeDef pwm1_init_typedef;

void TIM3_Init_Timer(void)
{
		TIM_Base_InitTypeDef tim3_init_typedef;
			
		tim3_init_typedef.ClockDivision = TIM_CLOCKDIVISION_DIV1;
		tim3_init_typedef.CounterMode = TIM_COUNTERMODE_DOWN;
		tim3_init_typedef.Prescaler = 8999;
		tim3_init_typedef.Period = 4999;
		tim3_handle_typedef.Instance = TIM3;
		tim3_handle_typedef.Init = tim3_init_typedef;
	
		HAL_TIM_Base_Init(&tim3_handle_typedef);
		HAL_TIM_Base_Start_IT(&tim3_handle_typedef);
}

void TIM3_Init_Pwm_CH4(void)
{
		TIM_Base_InitTypeDef tim3_init_typedef;
			
		tim3_init_typedef.ClockDivision = TIM_CLOCKDIVISION_DIV1;
		tim3_init_typedef.CounterMode = TIM_COUNTERMODE_UP;
		tim3_init_typedef.Prescaler = 89;
		tim3_init_typedef.Period = 499;
		tim3_handle_typedef.Instance = TIM3;
		tim3_handle_typedef.Init = tim3_init_typedef;	
		HAL_TIM_PWM_Init(&tim3_handle_typedef);
	
		pwm1_init_typedef.OCMode = 	TIM_OCMODE_PWM1;
		pwm1_init_typedef.Pulse = 499/2;
		pwm1_init_typedef.OCPolarity = TIM_OCPOLARITY_LOW;
	
		HAL_TIM_PWM_ConfigChannel(&tim3_handle_typedef ,&pwm1_init_typedef ,TIM_CHANNEL_4);
		HAL_TIM_PWM_Start(&tim3_handle_typedef ,TIM_CHANNEL_4);
}


void HAL_TIM_Base_MspInit(TIM_HandleTypeDef *htim)
{
		if(htim->Instance == TIM3)
		{
				__HAL_RCC_TIM3_CLK_ENABLE();
				HAL_NVIC_SetPriority(TIM3_IRQn,1,4);
				HAL_NVIC_EnableIRQ(TIM3_IRQn);
		}
}

void HAL_TIM_PWM_MspInit(TIM_HandleTypeDef *htim)
{
		if(htim->Instance == TIM3)
		{
				GPIO_InitTypeDef gpio_init_struct;
				__HAL_RCC_TIM3_CLK_ENABLE();
				__HAL_RCC_GPIOB_CLK_ENABLE();
				gpio_init_struct.Pin = GPIO_PIN_1;
				gpio_init_struct.Mode = GPIO_MODE_AF_PP;
				gpio_init_struct.Pull = GPIO_PULLUP;
				gpio_init_struct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
				gpio_init_struct.Alternate = GPIO_AF2_TIM3;
				HAL_GPIO_Init(GPIOB,&gpio_init_struct);
		}
	
}

void TIM3_Set_Compare_CH4(uint32_t count)
{
	TIM3->CCR4=count; 
		//pwm1_init_typedef.Pulse = count;
		//HAL_TIM_PWM_ConfigChannel(&tim3_handle_typedef,&pwm1_init_typedef,TIM_CHANNEL_4);
}

void TIM3_IRQHandler(void )
{
		HAL_TIM_IRQHandler(&tim3_handle_typedef);
}


void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
		if(htim->Instance == TIM3)
		{
				if(GPIO_PIN_SET == HAL_GPIO_ReadPin(GPIOB ,GPIO_PIN_0))
				{
						HAL_GPIO_WritePin(GPIOB ,GPIO_PIN_0 , GPIO_PIN_RESET);
				}
				else
				{
						HAL_GPIO_WritePin(GPIOB ,GPIO_PIN_0 , GPIO_PIN_SET);
				}
		}
}

