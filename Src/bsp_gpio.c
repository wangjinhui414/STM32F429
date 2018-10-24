#include "bsp_gpio.h"

IWDG_HandleTypeDef hiwdg;


void Delay(__IO uint32_t nCount);

void Delay(__IO uint32_t nCount)
{
  while(nCount--){}
}

void LED_Init(void)
{
		GPIO_InitTypeDef gpio_init_struct;
	
		__HAL_RCC_GPIOB_CLK_ENABLE();
	
		gpio_init_struct.Pin = GPIO_PIN_0 | GPIO_PIN_1;
		gpio_init_struct.Mode = GPIO_MODE_OUTPUT_PP;
		gpio_init_struct.Pull = GPIO_PULLUP;
		gpio_init_struct.Speed = GPIO_SPEED_FREQ_HIGH;
		HAL_GPIO_Init(GPIOB, &gpio_init_struct);
		HAL_GPIO_WritePin(GPIOB ,GPIO_PIN_0 | GPIO_PIN_1, GPIO_PIN_SET);
}
void LED_Lignt1(void)
{
		HAL_GPIO_WritePin(GPIOB ,GPIO_PIN_0 , GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOB ,GPIO_PIN_1 , GPIO_PIN_RESET);
}
void LED_Lignt2(void)
{
		HAL_GPIO_WritePin(GPIOB ,GPIO_PIN_0 , GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOB ,GPIO_PIN_1 , GPIO_PIN_RESET);
}
void LED_Lignt3(void)
{
		HAL_GPIO_WritePin(GPIOB ,GPIO_PIN_0, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOB ,GPIO_PIN_1, GPIO_PIN_SET);
}
void LED_Lignt4(void)
{
		HAL_GPIO_WritePin(GPIOB ,GPIO_PIN_0, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOB ,GPIO_PIN_1, GPIO_PIN_SET);
}


void KEY_Init(void)
{
		GPIO_InitTypeDef gpio_init_struct;
	
		__HAL_RCC_GPIOA_CLK_ENABLE();
		__HAL_RCC_GPIOC_CLK_ENABLE();
		__HAL_RCC_GPIOH_CLK_ENABLE();

		gpio_init_struct.Pin = GPIO_PIN_0;
		gpio_init_struct.Mode = GPIO_MODE_IT_RISING;
		gpio_init_struct.Pull = GPIO_PULLDOWN;
		gpio_init_struct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
		HAL_GPIO_Init(GPIOA, &gpio_init_struct);
		
		gpio_init_struct.Pin = GPIO_PIN_13;
		gpio_init_struct.Mode = GPIO_MODE_IT_FALLING;
		gpio_init_struct.Pull = GPIO_PULLUP;
		gpio_init_struct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
		HAL_GPIO_Init(GPIOC, &gpio_init_struct);
	
		gpio_init_struct.Pin = GPIO_PIN_2 | GPIO_PIN_3;
		gpio_init_struct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
		HAL_GPIO_Init(GPIOH, &gpio_init_struct);
	
		HAL_NVIC_SetPriority(EXTI0_IRQn,2,0);
		HAL_NVIC_EnableIRQ(EXTI0_IRQn);

		HAL_NVIC_SetPriority(EXTI2_IRQn,2,1);
		HAL_NVIC_EnableIRQ(EXTI2_IRQn);
		
		HAL_NVIC_SetPriority(EXTI3_IRQn,2,2);
		HAL_NVIC_EnableIRQ(EXTI3_IRQn);
		
		HAL_NVIC_SetPriority(EXTI15_10_IRQn,2,3);
		HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);
}

uint8_t KEY_Scan(void)
{
		uint8_t ret = 0;
		if(HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_0))
		{
				ret |= KEY_UP_ON;
		}
		if(!HAL_GPIO_ReadPin(GPIOC,GPIO_PIN_13))
		{	
				ret |= KEY_2_ON;
		}
		if(!HAL_GPIO_ReadPin(GPIOH,GPIO_PIN_2))
		{
				ret |= KEY_1_ON;
		}
		if(!HAL_GPIO_ReadPin(GPIOH,GPIO_PIN_3))
		{
				ret |= KEY_0_ON;
		}
		return ret;
}

//�жϷ�����
void EXTI0_IRQHandler(void)
{
    HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_0);
}

void EXTI2_IRQHandler(void)
{
    HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_2);
}

void EXTI3_IRQHandler(void)
{
    HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_3);
}

void EXTI15_10_IRQHandler(void)
{
    HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_13);
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
		HAL_Delay(100);
		if(GPIO_Pin == GPIO_PIN_0  && KEY_Scan() == KEY_UP_ON)
		{
				LED_Lignt4();
				//HAL_IWDG_Refresh(&hiwdg);
		}
		if(GPIO_Pin == GPIO_PIN_3  && KEY_Scan() == KEY_0_ON)
		{
				LED_Lignt1();
		}
		if(GPIO_Pin == GPIO_PIN_2  && KEY_Scan() == KEY_1_ON)
		{
				LED_Lignt2();
		}
		if(GPIO_Pin == GPIO_PIN_13  && KEY_Scan() == KEY_2_ON)
		{
				LED_Lignt3();
		}
}



void IWDG_Init(void)
{

  hiwdg.Instance = IWDG;
  hiwdg.Init.Prescaler = IWDG_PRESCALER_64;
  hiwdg.Init.Reload = 500;
  if (HAL_IWDG_Init(&hiwdg) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}


