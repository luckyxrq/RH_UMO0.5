#include "bsp.h"



/*
*********************************************************************************************************
*	函 数 名: bsp_InitCurrentFeedbackADC
*	功能说明: 配置电机电流反馈
*	形    参：无
*	返 回 值: 无
*********************************************************************************************************
*/
void bsp_InitCurrentFeedbackADC(void)
{
	ADC_InitTypeDef ADC_InitStructure; 
	GPIO_InitTypeDef GPIO_InitStructure;

	/*左轮机*/
	{
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB |RCC_APB2Periph_ADC2, ENABLE );
 
		/*设置ADC分频因子6 72M/6=12,ADC最大时间不能超过14M*/
		RCC_ADCCLKConfig(RCC_PCLK2_Div6);
                  
		/*配置引脚为模拟输入模式*/
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
		GPIO_Init(GPIOB, &GPIO_InitStructure);	

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
		ADC_RegularChannelConfig(ADC2, ADC_Channel_9, 1, ADC_SampleTime_239Cycles5 );
	}
	
	/*右轮机*/
	{
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOF |RCC_APB2Periph_ADC3, ENABLE );
 
		/*设置ADC分频因子6 72M/6=12,ADC最大时间不能超过14M*/
		RCC_ADCCLKConfig(RCC_PCLK2_Div6);
                  
		/*配置引脚为模拟输入模式*/
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
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
		ADC_RegularChannelConfig(ADC3, ADC_Channel_8, 1, ADC_SampleTime_239Cycles5 );
	}	
	
	
	/*吸尘*/
	{
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC |RCC_APB2Periph_ADC2, ENABLE );
 
		/*设置ADC分频因子6 72M/6=12,ADC最大时间不能超过14M*/
		RCC_ADCCLKConfig(RCC_PCLK2_Div6);
                  
		/*配置引脚为模拟输入模式*/
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
		GPIO_Init(GPIOC, &GPIO_InitStructure);	

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
		ADC_RegularChannelConfig(ADC2, ADC_Channel_12, 1, ADC_SampleTime_239Cycles5 );
	}
	
	/*滚刷*/
	{
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC |RCC_APB2Periph_ADC2, ENABLE );
 
		/*设置ADC分频因子6 72M/6=12,ADC最大时间不能超过14M*/
		RCC_ADCCLKConfig(RCC_PCLK2_Div6);
                  
		/*配置引脚为模拟输入模式*/
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
		GPIO_Init(GPIOC, &GPIO_InitStructure);	

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
		ADC_RegularChannelConfig(ADC2, ADC_Channel_10, 1, ADC_SampleTime_239Cycles5 );
	}
 
	/*边刷*/
	{
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC |RCC_APB2Periph_ADC2, ENABLE );
 
		/*设置ADC分频因子6 72M/6=12,ADC最大时间不能超过14M*/
		RCC_ADCCLKConfig(RCC_PCLK2_Div6);
                  
		/*配置引脚为模拟输入模式*/
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
		GPIO_Init(GPIOC, &GPIO_InitStructure);	

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
		ADC_RegularChannelConfig(ADC2, ADC_Channel_11, 1, ADC_SampleTime_239Cycles5 );
	}
	
	
	/*电池电压*/
	{
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB |RCC_APB2Periph_ADC2, ENABLE );
 
		/*设置ADC分频因子6 72M/6=12,ADC最大时间不能超过14M*/
		RCC_ADCCLKConfig(RCC_PCLK2_Div6);
                  
		/*配置引脚为模拟输入模式*/
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
		GPIO_Init(GPIOB, &GPIO_InitStructure);	

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
		ADC_RegularChannelConfig(ADC2, ADC_Channel_8, 1, ADC_SampleTime_239Cycles5 );
	}
	
	/*充电电流*/
	{
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOF |RCC_APB2Periph_ADC3, ENABLE );
 
		/*设置ADC分频因子6 72M/6=12,ADC最大时间不能超过14M*/
		RCC_ADCCLKConfig(RCC_PCLK2_Div6);
                  
		/*配置引脚为模拟输入模式*/
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;
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
		ADC_RegularChannelConfig(ADC3, ADC_Channel_4, 1, ADC_SampleTime_239Cycles5 );
	}
	
}


/*
*********************************************************************************************************
*	函 数 名: bsp_GetFeedbackVoltage
*	功能说明: 返回反馈的电压
*	形    参：无
*	返 回 值: 无
*********************************************************************************************************
*/
float bsp_GetFeedbackVoltage(FeedbackSN sn)
{
	float ret = 0;
	
	switch(sn)
	{
		case eMotorLeft:
		{
			ADC_RegularChannelConfig(ADC2, ADC_Channel_9, 1, ADC_SampleTime_239Cycles5 );
			ADC_SoftwareStartConvCmd(ADC2, ENABLE);	
			while(!ADC_GetFlagStatus(ADC2, ADC_FLAG_EOC ));
			ret = ADC_GetConversionValue(ADC2) * 3.3F / 4096;
		}break;
		
		case eMotorRight:
		{
			ADC_RegularChannelConfig(ADC3, ADC_Channel_8, 1, ADC_SampleTime_239Cycles5 );
			ADC_SoftwareStartConvCmd(ADC3, ENABLE);	
			while(!ADC_GetFlagStatus(ADC3, ADC_FLAG_EOC ));
			ret = ADC_GetConversionValue(ADC3) * 3.3F / 4096;
		}break;
		
		case eVacuum:
		{
			ADC_RegularChannelConfig(ADC2, ADC_Channel_12, 1, ADC_SampleTime_239Cycles5 );
			ADC_SoftwareStartConvCmd(ADC2, ENABLE);	
			while(!ADC_GetFlagStatus(ADC2, ADC_FLAG_EOC ));
			ret = ADC_GetConversionValue(ADC2) * 3.3F / 4096;
		}break;
		
		case eRollingBrush:
		{
			ADC_RegularChannelConfig(ADC2, ADC_Channel_10, 1, ADC_SampleTime_239Cycles5 );
			ADC_SoftwareStartConvCmd(ADC2, ENABLE);	
			while(!ADC_GetFlagStatus(ADC2, ADC_FLAG_EOC ));
			ret = ADC_GetConversionValue(ADC2) * 3.3F / 4096;
		}break;
		
		case eSideBrush:
		{
			ADC_RegularChannelConfig(ADC2, ADC_Channel_11, 1, ADC_SampleTime_239Cycles5 );
			ADC_SoftwareStartConvCmd(ADC2, ENABLE);	
			while(!ADC_GetFlagStatus(ADC2, ADC_FLAG_EOC ));
			ret = ADC_GetConversionValue(ADC2) * 3.3F / 4096;
		}break;
		
		case eBatteryVoltage:
		{
			ADC_RegularChannelConfig(ADC2, ADC_Channel_8, 1, ADC_SampleTime_239Cycles5 );
			ADC_SoftwareStartConvCmd(ADC2, ENABLE);	
			while(!ADC_GetFlagStatus(ADC2, ADC_FLAG_EOC ));
			ret = ADC_GetConversionValue(ADC2) * 3.3F / 4096;
		}break;
		
		case eBatteryCurrent:
		{
			ADC_RegularChannelConfig(ADC3, ADC_Channel_4, 1, ADC_SampleTime_239Cycles5 );
			ADC_SoftwareStartConvCmd(ADC3, ENABLE);	
			while(!ADC_GetFlagStatus(ADC3, ADC_FLAG_EOC ));
			ret = ADC_GetConversionValue(ADC3) * 3.3F / 4096;
		}break;
	}
	
	return ret ;
}

 

void bsp_PrintAllVoltage(void)
{
	float batteryVoltage = bsp_GetFeedbackVoltage(eBatteryVoltage);
	float batteryCurrent = bsp_GetFeedbackVoltage(eBatteryCurrent);
	float wheelL = bsp_GetFeedbackVoltage(eMotorLeft);
	float wheelR = bsp_GetFeedbackVoltage(eMotorRight);
	float roll = bsp_GetFeedbackVoltage(eRollingBrush);
	float vacuum = bsp_GetFeedbackVoltage(eVacuum);
	float sideBrush = bsp_GetFeedbackVoltage(eSideBrush);
	
	/*430  66.5是电阻分压  0.2是根据实际情况补偿电压*/
	batteryVoltage = (batteryVoltage * 430 / 66.5) + batteryVoltage + 0.2F; 
	batteryCurrent = batteryCurrent*1000.0F * 1000.0F / 10.0F / 50.0F; 
	wheelL = wheelL * 1000.0F * 1000.0F / 33.0F / 50.0F;
	wheelR = wheelR * 1000.0F * 1000.0F / 33.0F / 50.0F;
	roll = roll * 1000.0F * 1000.0F / 33.0F / 50.0F;
	vacuum = vacuum * 1000.0F * 1000.0F / 33.0F / 50.0F;
	sideBrush = sideBrush * 1000.0F * 1000.0F / 100.0F / 50.0F;
	
	
	DEBUG("左轮:%.2fmA  右轮:%.2fmA  风机:%.2fmA  滚刷:%.2fmA  边刷:%.2fmA  电池电压:%.2fV  电池电流:%.2fmA\r\n",
	wheelL,
	wheelR,
	vacuum,
	roll,
	sideBrush,
	batteryVoltage,
	batteryCurrent);


}





/*******************************************滤波专用变量**************************************************/

#define ARR_SIZE  16

#define FILTER_ARR            ARR_SIZE
#define ARR_FILTER_START      2
#define ARR_FILTER_END        ARR_SIZE - 2 

static float g_vMotorLeft        = 0.0F;
static float g_vMotorRight       = 0.0F;
static float g_vVacuum           = 0.0F;
static float g_vRollingBrush     = 0.0F;
static float g_vSideBrush        = 0.0F;
static float g_vBatteryVoltage   = 0.0F;
static float g_vBatteryCurrent   = 0.0F;

static float arrMotorLeft[ARR_SIZE] = {0};
static float arrMotorRight[ARR_SIZE] = {0};
static float arrVacuum[ARR_SIZE] = {0};
static float arrRollingBrush[ARR_SIZE] = {0};
static float arrSideBrush[ARR_SIZE] = {0};
static float arrBatteryVoltage[ARR_SIZE] = {0};
static float arrBatteryCurrent[ARR_SIZE] = {0};

static uint8_t arr_index = 0 ;

/*
*********************************************************************************************************
*	函 数 名: bsp_GetVoltageFilterProc
*	功能说明: 采用滤波的方式获取电压
*	形    参：无
*	返 回 值: 无
*********************************************************************************************************
*/
void bsp_GetVoltageFilterProc(void)
{
	uint32_t i = 0 ;
	float sum = 0.0F;

	/*左轮*/
	sum = 0.0F;
	arrMotorLeft[arr_index] = bsp_GetFeedbackVoltage(eMotorLeft);
	sort_float(arrMotorLeft,FILTER_ARR);
	for(i=ARR_FILTER_START;i<ARR_FILTER_END;++i)
	{
		sum += arrMotorLeft[i];
	}
	g_vMotorLeft = sum / (float)(ARR_FILTER_END-ARR_FILTER_START);
	g_vMotorLeft = g_vMotorLeft * 1000.0F * 1000.0F / 33.0F / 50.0F;
	
	/*右轮*/
	sum = 0.0F;
	arrMotorRight[arr_index] = bsp_GetFeedbackVoltage(eMotorRight);
	sort_float(arrMotorRight,FILTER_ARR);
	for(i=ARR_FILTER_START;i<ARR_FILTER_END;++i)
	{
		sum += arrMotorRight[i];
	}
	g_vMotorRight = sum / (float)(ARR_FILTER_END-ARR_FILTER_START);
	g_vMotorRight = g_vMotorRight * 1000.0F * 1000.0F / 33.0F / 50.0F;
	
	/*风机*/
	sum = 0.0F;
	arrVacuum[arr_index] = bsp_GetFeedbackVoltage(eVacuum);
	sort_float(arrVacuum,FILTER_ARR);
	for(i=ARR_FILTER_START;i<ARR_FILTER_END;++i)
	{
		sum += arrVacuum[i];
	}
	g_vVacuum = sum / (float)(ARR_FILTER_END-ARR_FILTER_START);
	g_vVacuum = g_vVacuum * 1000.0F * 1000.0F / 33.0F / 50.0F;
	
	/*滚刷*/
	sum = 0.0F;
	arrRollingBrush[arr_index] = bsp_GetFeedbackVoltage(eRollingBrush);
	sort_float(arrRollingBrush,FILTER_ARR);
	for(i=ARR_FILTER_START;i<ARR_FILTER_END;++i)
	{
		sum += arrRollingBrush[i];
	}
	g_vRollingBrush = sum / (float)(ARR_FILTER_END-ARR_FILTER_START);
	g_vRollingBrush = g_vRollingBrush * 1000.0F * 1000.0F / 33.0F / 50.0F;
	
	/*边刷*/
	sum = 0.0F;
	arrSideBrush[arr_index] = bsp_GetFeedbackVoltage(eSideBrush);
	sort_float(arrSideBrush,FILTER_ARR);
	for(i=ARR_FILTER_START;i<ARR_FILTER_END;++i)
	{
		sum += arrSideBrush[i];
	}
	g_vSideBrush = sum / (float)(ARR_FILTER_END-ARR_FILTER_START);
	g_vSideBrush = g_vSideBrush * 1000.0F * 1000.0F / 100.0F / 50.0F;

	/*电池电压*/
	sum = 0.0F;
	arrBatteryVoltage[arr_index] = bsp_GetFeedbackVoltage(eBatteryVoltage);
	sort_float(arrBatteryVoltage,FILTER_ARR);
	for(i=ARR_FILTER_START;i<ARR_FILTER_END;++i)
	{
		sum += arrBatteryVoltage[i];
	}
	g_vBatteryVoltage = sum / (float)(ARR_FILTER_END-ARR_FILTER_START);	
	g_vBatteryVoltage = (g_vBatteryVoltage * 430 / 66.5) + g_vBatteryVoltage + 0.2F;
	g_vBatteryVoltage = g_vBatteryVoltage*1000;
	
	/*电池电流*/
	sum = 0.0F;
	arrBatteryCurrent[arr_index] = bsp_GetFeedbackVoltage(eBatteryCurrent);
	sort_float(arrBatteryCurrent,FILTER_ARR);
	for(i=ARR_FILTER_START;i<ARR_FILTER_END;++i)
	{
		sum += arrBatteryCurrent[i];
	}
	g_vBatteryCurrent = sum / (float)(ARR_FILTER_END-ARR_FILTER_START);	
	g_vBatteryCurrent = g_vBatteryCurrent*1000.0F * 1000.0F / 10.0F / 50.0F;
	
	++arr_index;
	if(arr_index == ARR_SIZE)
	{
		arr_index = 0 ;
	}
}



/*
*********************************************************************************************************
*	函 数 名: bsp_GetVoltageAfterFilter
*	功能说明: 返回反馈的电压
*	形    参：无
*	返 回 值: 无
*********************************************************************************************************
*/
float bsp_GetVoltageAfterFilter(FeedbackSN sn)
{
	float ret = 0;
	
	switch(sn)
	{
		case eMotorLeft:
		{
			ret = g_vMotorLeft;
		}break;
		
		case eMotorRight:
		{
			ret = g_vMotorRight;
		}break;
		
		case eVacuum:
		{
			ret = g_vVacuum;
		}break;
		
		case eRollingBrush:
		{
			ret = g_vRollingBrush;
		}break;
		
		case eSideBrush:
		{
			ret = g_vSideBrush;
		}break;
		
		case eBatteryVoltage:
		{
			ret = g_vBatteryVoltage;
		}break;
		
		case eBatteryCurrent:
		{
			ret = g_vBatteryCurrent;
		}break;
	}
	
	return ret ;
}
