#ifndef _BSP_TFT_LCD_H_
#define _BSP_TFT_LCD_H_
//#include "stm32f4xx.h"
#include "stm32f4xx_hal.h"


//扫描方向定义
#define L2R_U2D  0 		//从左到右,从上到下
#define L2R_D2U  1 		//从左到右,从下到上
#define R2L_U2D  2 		//从右到左,从上到下
#define R2L_D2U  3 		//从右到左,从下到上

#define U2D_L2R  4 		//从上到下,从左到右
#define U2D_R2L  5 		//从上到下,从右到左
#define D2U_L2R  6 		//从下到上,从左到右
#define D2U_R2L  7		//从下到上,从右到左	 

#define DFT_SCAN_DIR  L2R_U2D  //默认的扫描方向
//LCD背光
#define LCD_LED(n)   ( n>0 ? HAL_GPIO_WritePin(GPIOB ,GPIO_PIN_5 , GPIO_PIN_SET) : HAL_GPIO_WritePin(GPIOB ,GPIO_PIN_5 , GPIO_PIN_RESET))
//画笔颜色
#define WHITE			0xFFFF
#define BLACK			0x0000	  
#define BLUE			0x001F  
#define BRED			0XF81F
#define GRED			0XFFE0
#define GBLUE			0X07FF
#define RED				0xF800
#define MAGENTA			0xF81F
#define GREEN			0x07E0
#define CYAN			0x7FFF
#define YELLOW			0xFFE0
#define BROWN			0XBC40 //棕色
#define BRRED			0XFC07 //棕红色
#define GRAY			0X8430 //灰色
//GUI颜色

#define DARKBLUE		0X01CF	//深蓝色
#define LIGHTBLUE		0X7D7C	//浅蓝色  
#define GRAYBLUE		0X5458 //灰蓝色



//LCD重要参数集
typedef struct  
{		 	 
	uint16_t width;			//LCD 宽度
	uint16_t height;			//LCD 高度
	uint16_t id;				//LCD ID
	uint8_t  dir;			//横屏还是竖屏控制：0，竖屏；1，横屏。	
	uint16_t	wramcmd;		//开始写gram指令
	uint16_t setxcmd;		//设置x坐标指令
	uint16_t setycmd;		//设置y坐标指令 
}_lcd_dev; 	  

//LCD参数
extern SRAM_HandleTypeDef SRAM_Handler; 
extern _lcd_dev lcddev;	//管理LCD重要参数
void LCD_Init(void);
void LCD_ShowString(uint16_t x,uint16_t y,uint16_t width,uint16_t height,uint8_t size,char *p);
void LCD_ShowxNum(uint16_t x,uint16_t y,uint32_t num,uint8_t len,uint8_t size,uint8_t mode);
#endif