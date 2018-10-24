#include "bsp_tpad.h"

uint32_t  g_default_value;

TIM_HandleTypeDef tim2_handle_typedef;

void TIM2_Capture_CH1_Init(void)
{
		TIM_IC_InitTypeDef tim2_ch1_config;  
	
		tim2_handle_typedef.Instance = TIM2;
		tim2_handle_typedef.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
		tim2_handle_typedef.Init.CounterMode = TIM_COUNTERMODE_UP;
		tim2_handle_typedef.Init.Prescaler = 1;
		tim2_handle_typedef.Init.Period = 0xFFFFFFFF;
		HAL_TIM_IC_Init(&tim2_handle_typedef);
		
		tim2_ch1_config.ICPolarity=TIM_ICPOLARITY_RISING;  
    tim2_ch1_config.ICSelection=TIM_ICSELECTION_DIRECTTI;
    tim2_ch1_config.ICPrescaler=TIM_ICPSC_DIV1;        
    tim2_ch1_config.ICFilter=0;                          
	
    HAL_TIM_IC_ConfigChannel(&tim2_handle_typedef , &tim2_ch1_config , TIM_CHANNEL_1);
    HAL_TIM_IC_Start(&tim2_handle_typedef,TIM_CHANNEL_1);
}

void HAL_TIM_IC_MspInit(TIM_HandleTypeDef *htim)
{
    GPIO_InitTypeDef GPIO_Initure;
    __HAL_RCC_TIM2_CLK_ENABLE();          
    __HAL_RCC_GPIOA_CLK_ENABLE();			
	
    GPIO_Initure.Pin=GPIO_PIN_5;            
    GPIO_Initure.Mode=GPIO_MODE_AF_PP;      
    GPIO_Initure.Pull=GPIO_NOPULL;          
    GPIO_Initure.Speed=GPIO_SPEED_HIGH;     
    GPIO_Initure.Alternate=GPIO_AF1_TIM2;   
    HAL_GPIO_Init(GPIOA,&GPIO_Initure);
}

void TPAD_Reset(void)
{
    GPIO_InitTypeDef GPIO_Initure;
	
    GPIO_Initure.Pin=GPIO_PIN_5;            
    GPIO_Initure.Mode=GPIO_MODE_OUTPUT_PP;  
    GPIO_Initure.Pull=GPIO_PULLDOWN;        
    GPIO_Initure.Speed=GPIO_SPEED_HIGH;     
    HAL_GPIO_Init(GPIOA,&GPIO_Initure);
    
    HAL_GPIO_WritePin(GPIOA,GPIO_PIN_5,GPIO_PIN_RESET);	
    HAL_Delay(5);
    __HAL_TIM_CLEAR_FLAG(&tim2_handle_typedef, TIM_FLAG_CC1 | TIM_FLAG_UPDATE);   
    __HAL_TIM_SET_COUNTER(&tim2_handle_typedef,0); 
    
    GPIO_Initure.Mode=GPIO_MODE_AF_PP;      
    GPIO_Initure.Pull=GPIO_NOPULL;          
    GPIO_Initure.Alternate=GPIO_AF1_TIM2;   
    HAL_GPIO_Init(GPIOA,&GPIO_Initure);         
}

uint16_t TPAD_GetValue(void)
{
		TPAD_Reset();
    while(__HAL_TIM_GET_FLAG(&tim2_handle_typedef , TIM_FLAG_CC1)==RESET) 
    {
        if(__HAL_TIM_GET_COUNTER(&tim2_handle_typedef)>0XFFFFFFFF-500) 
					return __HAL_TIM_GET_COUNTER(&tim2_handle_typedef);
    }
    return HAL_TIM_ReadCapturedValue(&tim2_handle_typedef, TIM_CHANNEL_1);

}

uint16_t TPAD_Get_MaxVal(uint8_t n)
{ 
	uint16_t temp=0; 
	uint16_t res=0; 
	uint8_t lcntnum=n*2/3;
	uint8_t okcnt=0;
	while(n--)
	{
		temp = TPAD_GetValue();
		if(temp >(g_default_value*5/4))
				okcnt++;
		if(temp>res)res=temp;
	}
	if(okcnt>=lcntnum)
			return res;
	else return 0;
}  

uint8_t TPAD_Init(void)
{
	uint16_t buf[10];
	uint16_t temp;
	uint8_t j,i;
	TIM2_Capture_CH1_Init();
	for(i=0;i<10;i++)
	{				 
		buf[i]=TPAD_GetValue();
		HAL_Delay(10);	    
	}				    
	for(i=0;i<9;i++)
	{
		for(j=i+1;j<10;j++)
		{
			if(buf[i]>buf[j])
			{
				temp=buf[i];
				buf[i]=buf[j];
				buf[j]=temp;
			}
		}
	}
	temp=0;
	for(i=2;i<8;i++)temp+=buf[i];
	g_default_value=temp/6;
	printf("tpad_default_val:%d\r\n",(int)g_default_value);	
	if(g_default_value > 0xFFFFFFFF/2)
			return 1;
	return 0;				
}

uint8_t TPAD_Scan(uint8_t mode)
{

	static uint8_t keyen=0;	 
	uint8_t res=0;
	uint8_t sample=3;	
	uint16_t rval;
	if(mode)
	{
		sample=6;	
		keyen=0;	
	}
	rval=TPAD_Get_MaxVal(sample); 
	if( rval>(g_default_value*4/3) &&
		  rval<(10*g_default_value))
	{							 
		if(keyen==0)
				res=1;	
		printf("r:%d\r\n",rval);		     	    					   
				keyen=3;				
	} 
	if(keyen)
			keyen--;		   							   		     	    					   
	return res;
}
