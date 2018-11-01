#include "bsp_adc.h"

ADC_HandleTypeDef adc_handle_typedef;

void ADC_Init(void)
{
    adc_handle_typedef.Instance=ADC1;
    adc_handle_typedef.Init.ClockPrescaler=ADC_CLOCK_SYNC_PCLK_DIV4;   //4分频，ADCCLK=PCLK2/4=90/4=22.5MHZ
    adc_handle_typedef.Init.Resolution=ADC_RESOLUTION_12B;             //12位模式
    adc_handle_typedef.Init.DataAlign=ADC_DATAALIGN_RIGHT;             //右对齐
    adc_handle_typedef.Init.ScanConvMode=DISABLE;                      //非扫描模式
    adc_handle_typedef.Init.EOCSelection=DISABLE;                      //关闭EOC中断
    adc_handle_typedef.Init.ContinuousConvMode=DISABLE;                //关闭连续转换
    adc_handle_typedef.Init.NbrOfConversion=1;                         //1个转换在规则序列中 也就是只转换规则序列1 
    adc_handle_typedef.Init.DiscontinuousConvMode=DISABLE;             //禁止不连续采样模式
    adc_handle_typedef.Init.NbrOfDiscConversion=0;                     //不连续采样通道数为0
    adc_handle_typedef.Init.ExternalTrigConv=ADC_SOFTWARE_START;       //软件触发
    adc_handle_typedef.Init.ExternalTrigConvEdge=ADC_EXTERNALTRIGCONVEDGE_NONE;//使用软件触发
    adc_handle_typedef.Init.DMAContinuousRequests=DISABLE;             //关闭DMA请求
    HAL_ADC_Init(&adc_handle_typedef);
}

void HAL_ADC_MspInit(ADC_HandleTypeDef* hadc)
{
    GPIO_InitTypeDef GPIO_Initure;
    __HAL_RCC_ADC1_CLK_ENABLE();            //使能ADC1时钟
    __HAL_RCC_GPIOA_CLK_ENABLE();			//开启GPIOA时钟
	
    GPIO_Initure.Pin=GPIO_PIN_5;            //PA5
    GPIO_Initure.Mode=GPIO_MODE_ANALOG;     //模拟
    GPIO_Initure.Pull=GPIO_NOPULL;          //不带上下拉
    HAL_GPIO_Init(GPIOA,&GPIO_Initure);
}

uint32_t Get_ADC_Value(uint8_t ch)
{
    ADC_ChannelConfTypeDef adc1_chanconf;
    adc1_chanconf.Channel = ch;
    adc1_chanconf.Rank = 1;
    adc1_chanconf.SamplingTime = ADC_SAMPLETIME_480CYCLES;
    adc1_chanconf.Offset= 0;
    HAL_ADC_ConfigChannel(&adc_handle_typedef, &adc1_chanconf);
    HAL_ADC_Start(&adc_handle_typedef);
    HAL_ADC_PollForConversion(&adc_handle_typedef,10);
    return HAL_ADC_GetValue(&adc_handle_typedef);
}
uint32_t Get_ADC_AverageValue(uint8_t ch,uint16_t times)
{
    uint32_t sum;
    uint8_t i=times;
    while(i--)
        sum+=Get_ADC_Value(ch);
    return sum/times;
}