/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
  ******************************************************************************
  *
  * COPYRIGHT(c) 2016 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/* USER CODE BEGIN Includes */
#include "bsp_gpio.h"
#include "bsp_usart.h"
#include "bsp_tim.h"
#include "bsp_tpad.h"
#include "bsp_tftlcd.h"
#include "bsp_sdram.h"

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
//static void MX_GPIO_Init(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

int main(void)
{

  /* USER CODE BEGIN 1 */
//	uint8_t dir = 0;
//	uint32_t cout = 300;
  char sbuf[20]={0};
  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();

  /* Initialize all configured peripherals */

  /* USER CODE BEGIN 2 */
  LED_Init();
  KEY_Init();
  //LED_Lignt1();
  USART2_Init();
  printf("***********SYSTEM INIT***********\n");
  TPAD_Init();
  LCD_Init();
  SDRAM_Init();
  RNG_Init();
  //TIM3_Init_Timer();
  //TIM3_Init_Pwm_CH4();
  //IWDG_Init();
  sprintf((char*)sbuf,"LCD ID:%04X",lcddev.id);//将LCD ID打印到lcd_id数组。
  LCD_ShowString(10,40,240,32,32,"Apollo STM32F4/F7"); 	
  LCD_ShowString(10,80,240,24,24,"TFTLCD TEST");
  LCD_ShowString(10,110,240,16,16,"ATOM@ALIENTEK");
  LCD_ShowString(10,130,240,16,16,sbuf);		//显示LCD ID	      					 
  LCD_ShowString(10,150,240,12,12,"2018/10/6");

  /* USER CODE END 2 */
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	/* USER CODE END WHILE */
			
	/* USER CODE BEGIN 3 */
    if(pcUsart2RX.rflag == 1)
    {
      printf("%s (OK)\n",pcUsart2RX.pcRxbuf);
      if( strncmp((char*)pcUsart2RX.pcRxbuf,"LED_CHANGE",strlen("LED_CHANGE"))==0)
      {
        if(GPIO_PIN_SET == HAL_GPIO_ReadPin(GPIOB ,GPIO_PIN_0))
        {
          HAL_GPIO_WritePin(GPIOB ,GPIO_PIN_0 , GPIO_PIN_RESET);
          printf("LED_ON\n");
        }
        else
        {
            HAL_GPIO_WritePin(GPIOB ,GPIO_PIN_0 , GPIO_PIN_SET);
            printf("LED_OFF\n");
        }			
      }
      else if( strncmp((char*)pcUsart2RX.pcRxbuf,"SDRAM_TEST",strlen("SDRAM_TEST"))==0)
      {
          fsmc_sdram_test(30,170);//测试SRAM容量
      }
      else if(strncmp((char*)pcUsart2RX.pcRxbuf,"SDRAM_PRINT",strlen("SDRAM_PRINT"))==0)
      {
           char buf[100]="SDRAM TEST STRING!!";
           printf("Write:%s\n",buf);
           FMC_SDRAM_WriteBuffer((uint8_t*)buf,0,strlen(buf));
           memset(buf,0,100);
           FMC_SDRAM_ReadBuffer((uint8_t*)buf,0,100);
           printf("Read:%s\n",buf);    
      }
      ResetUartrRx(&pcUsart2RX);
    }

    if(TPAD_Scan(0))
    {
      char snumbuf[20];
      sprintf(snumbuf,"rondom:%03d",RNG_Get_RandomRange(0,100));
      LCD_ShowString(10,170,240,32,32,snumbuf);
      if(GPIO_PIN_SET == HAL_GPIO_ReadPin(GPIOB ,GPIO_PIN_0))
      {
        HAL_GPIO_WritePin(GPIOB ,GPIO_PIN_0 , GPIO_PIN_RESET);
      }
      else
      {
          HAL_GPIO_WritePin(GPIOB ,GPIO_PIN_0 , GPIO_PIN_SET);
      }		        	
    }
    HAL_Delay(10);
//			if(!dir)	cout--;
//			else cout++;
//			if(cout == 0)	dir =1;
//			if(cout >= 300) dir = 0;
//			TIM3_Set_Compare_CH4(cout);
//			HAL_Delay(10);
			
  }
  /* USER CODE END 3 */

}

/** System Clock Configuration
*/
void SystemClock_Config(void)
{

	RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

    /**Configure the main internal regulator output voltage 
    */
  __HAL_RCC_PWR_CLK_ENABLE();

  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 25;
  RCC_OscInitStruct.PLL.PLLN = 360;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Activate the Over-Drive mode 
    */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  file: The file name as string.
  * @param  line: The line in file as a number.
  * @retval None
  */
void _Error_Handler(char *file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef USE_FULL_ASSERT

/**
   * @brief Reports the name of the source file and the source line number
   * where the assert_param error has occurred.
   * @param file: pointer to the source file name
   * @param line: assert_param error line source number
   * @retval None
   */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */

}

#endif

/**
  * @}
  */ 

/**
  * @}
*/ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
