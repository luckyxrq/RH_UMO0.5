#include "bsp.h"


/*
*********************************************************************************************************
*	函 数 名: bsp_InitOffSiteSW
*	功能说明: 初始化离地开关使能引脚
*	形    参: 无
*	返 回 值: 无
*********************************************************************************************************
*/
void bsp_InitCliffSW(void)
{
	ADC_InitTypeDef ADC_InitStructure; 
	GPIO_InitTypeDef GPIO_InitStructure;
	
	/*跳崖1*/
	{
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA |RCC_APB2Periph_ADC2, ENABLE );
 
		/*设置ADC分频因子6 72M/6=12,ADC最大时间不能超过14M*/
		RCC_ADCCLKConfig(RCC_PCLK2_Div6);
                  
		/*配置引脚为模拟输入模式*/
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7;
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
		GPIO_Init(GPIOA, &GPIO_InitStructure);	

		/*复位ADC*/
		ADC_DeInit(ADC2);

		ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;
		ADC_InitStructure.ADC_ScanConvMode = DISABLE;
		ADC_InitStructure.ADC_ContinuousConvMode = DISABLE;
		ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;
		ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
		ADC_InitStructure.ADC_NbrOfChannel = 1;
		ADC_Init(ADC2, &ADC_InitStructure);  

		/*使能指定的ADC*/
		ADC_Cmd(ADC2, ENABLE);
		
		/*使能复位校准 */
		ADC_ResetCalibration(ADC2);
		 
		/*等待复位校准结束*/
		while(ADC_GetResetCalibrationStatus(ADC2));
		
		/*开启AD校准*/
		ADC_StartCalibration(ADC2);
	 
		/*等待校准结束*/
		while(ADC_GetCalibrationStatus(ADC2));
		
		/*ADC,ADC通道,采样时间为239.5周期*/
		ADC_RegularChannelConfig(ADC2, ADC_Channel_7, 1, ADC_SampleTime_239Cycles5 );
	}
	
	
	/*跳崖2*/
	{
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA |RCC_APB2Periph_ADC2, ENABLE );
 
		/*设置ADC分频因子6 72M/6=12,ADC最大时间不能超过14M*/
		RCC_ADCCLKConfig(RCC_PCLK2_Div6);
                  
		/*配置引脚为模拟输入模式*/
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
		GPIO_Init(GPIOA, &GPIO_InitStructure);	

		/*复位ADC*/
		ADC_DeInit(ADC2);

		ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;
		ADC_InitStructure.ADC_ScanConvMode = DISABLE;
		ADC_InitStructure.ADC_ContinuousConvMode = DISABLE;
		ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;
		ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
		ADC_InitStructure.ADC_NbrOfChannel = 1;
		ADC_Init(ADC2, &ADC_InitStructure);  

		/*使能指定的ADC*/
		ADC_Cmd(ADC2, ENABLE);
		
		/*使能复位校准 */
		ADC_ResetCalibration(ADC2);
		 
		/*等待复位校准结束*/
		while(ADC_GetResetCalibrationStatus(ADC2));
		
		/*开启AD校准*/
		ADC_StartCalibration(ADC2);
	 
		/*等待校准结束*/
		while(ADC_GetCalibrationStatus(ADC2));
		
		/*ADC,ADC通道,采样时间为239.5周期*/
		ADC_RegularChannelConfig(ADC2, ADC_Channel_6, 1, ADC_SampleTime_239Cycles5 );
	}
	
	/*跳崖3*/
	{
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOF |RCC_APB2Periph_ADC3, ENABLE );
 
		/*设置ADC分频因子6 72M/6=12,ADC最大时间不能超过14M*/
		RCC_ADCCLKConfig(RCC_PCLK2_Div6);
                  
		/*配置引脚为模拟输入模式*/
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
		GPIO_Init(GPIOF, &GPIO_InitStructure);	

		/*复位ADC*/
		ADC_DeInit(ADC3);

		ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;
		ADC_InitStructure.ADC_ScanConvMode = DISABLE;
		ADC_InitStructure.ADC_ContinuousConvMode = DISABLE;
		ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;
		ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
		ADC_InitStructure.ADC_NbrOfChannel = 1;
		ADC_Init(ADC3, &ADC_InitStructure);  

		/*使能指定的ADC*/
		ADC_Cmd(ADC3, ENABLE);
		
		/*使能复位校准 */
		ADC_ResetCalibration(ADC3);
		 
		/*等待复位校准结束*/
		while(ADC_GetResetCalibrationStatus(ADC3));
		
		/*开启AD校准*/
		ADC_StartCalibration(ADC3);
	 
		/*等待校准结束*/
		while(ADC_GetCalibrationStatus(ADC3));
		
		/*ADC,ADC通道,采样时间为239.5周期*/
		ADC_RegularChannelConfig(ADC3, ADC_Channel_7, 1, ADC_SampleTime_239Cycles5 );
	}	

}

