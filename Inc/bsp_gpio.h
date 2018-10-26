#ifndef _BSP_GPIO_H_
#define _BSP_GPIO_H_
#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_iwdg.h"

extern IWDG_HandleTypeDef hiwdg;


void LED_Init(void);
void LED_Lignt1(void);
void LED_Lignt2(void);
void LED_Lignt3(void);
void LED_Lignt4(void);

void KEY_Init(void);
#define KEY_0_ON   0x01
#define KEY_1_ON   0x02
#define KEY_2_ON   0x04
#define KEY_UP_ON  0x08

uint8_t KEY_Scan(void);

void IWDG_Init(void);


void RNG_Init(void);
uint32_t RNG_Get_RandomNum(void);
int RNG_Get_RandomRange(int min,int max);
#endif

