#include "bsp.h"



/*
*********************************************************************************************************
*	�� �� ��: bsp_InitCurrentFeedbackADC
*	����˵��: ���õ����������
*	��    �Σ���
*	�� �� ֵ: ��
*********************************************************************************************************
*/
void bsp_InitCurrentFeedbackADC(void)
{
	ADC_InitTypeDef ADC_InitStructure; 
	GPIO_InitTypeDef GPIO_InitStructure;

	/*���ֻ�*/
	{
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB |RCC_APB2Periph_ADC2, ENABLE );
 
		/*����ADC��Ƶ����6 72M/6=12,ADC���ʱ�䲻�ܳ���14M*/
		RCC_ADCCLKConfig(RCC_PCLK2_Div6);
                  
		/*��������Ϊģ������ģʽ*/
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
		GPIO_Init(GPIOB, &GPIO_InitStructure);	

		/*��λADC*/
		ADC_DeInit(ADC2);

		ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;
		ADC_InitStructure.ADC_ScanConvMode = DISABLE;
		ADC_InitStructure.ADC_ContinuousConvMode = DISABLE;
		ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;
		ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
		ADC_InitStructure.ADC_NbrOfChannel = 1;
		ADC_Init(ADC2, &ADC_InitStructure);  

		/*ʹ��ָ����ADC*/
		ADC_Cmd(ADC2, ENABLE);
		
		/*ʹ�ܸ�λУ׼ */
		ADC_ResetCalibration(ADC2);
		 
		/*�ȴ���λУ׼����*/
		while(ADC_GetResetCalibrationStatus(ADC2));
		
		/*����ADУ׼*/
		ADC_StartCalibration(ADC2);
	 
		/*�ȴ�У׼����*/
		while(ADC_GetCalibrationStatus(ADC2));
		
		/*ADC,ADCͨ��,����ʱ��Ϊ239.5����*/
		ADC_RegularChannelConfig(ADC2, ADC_Channel_9, 1, ADC_SampleTime_239Cycles5 );
	}
	
	/*���ֻ�*/
	{
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOF |RCC_APB2Periph_ADC3, ENABLE );
 
		/*����ADC��Ƶ����6 72M/6=12,ADC���ʱ�䲻�ܳ���14M*/
		RCC_ADCCLKConfig(RCC_PCLK2_Div6);
                  
		/*��������Ϊģ������ģʽ*/
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
		GPIO_Init(GPIOF, &GPIO_InitStructure);	

		/*��λADC*/
		ADC_DeInit(ADC3);

		ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;
		ADC_InitStructure.ADC_ScanConvMode = DISABLE;
		ADC_InitStructure.ADC_ContinuousConvMode = DISABLE;
		ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;
		ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
		ADC_InitStructure.ADC_NbrOfChannel = 1;
		ADC_Init(ADC3, &ADC_InitStructure);  

		/*ʹ��ָ����ADC*/
		ADC_Cmd(ADC3, ENABLE);
		
		/*ʹ�ܸ�λУ׼ */
		ADC_ResetCalibration(ADC3);
		 
		/*�ȴ���λУ׼����*/
		while(ADC_GetResetCalibrationStatus(ADC3));
		
		/*����ADУ׼*/
		ADC_StartCalibration(ADC3);
	 
		/*�ȴ�У׼����*/
		while(ADC_GetCalibrationStatus(ADC3));
		
		/*ADC,ADCͨ��,����ʱ��Ϊ239.5����*/
		ADC_RegularChannelConfig(ADC3, ADC_Channel_8, 1, ADC_SampleTime_239Cycles5 );
	}	
	
	
	/*����*/
	{
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC |RCC_APB2Periph_ADC2, ENABLE );
 
		/*����ADC��Ƶ����6 72M/6=12,ADC���ʱ�䲻�ܳ���14M*/
		RCC_ADCCLKConfig(RCC_PCLK2_Div6);
                  
		/*��������Ϊģ������ģʽ*/
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
		GPIO_Init(GPIOC, &GPIO_InitStructure);	

		/*��λADC*/
		ADC_DeInit(ADC2);

		ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;
		ADC_InitStructure.ADC_ScanConvMode = DISABLE;
		ADC_InitStructure.ADC_ContinuousConvMode = DISABLE;
		ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;
		ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
		ADC_InitStructure.ADC_NbrOfChannel = 1;
		ADC_Init(ADC2, &ADC_InitStructure);  

		/*ʹ��ָ����ADC*/
		ADC_Cmd(ADC2, ENABLE);
		
		/*ʹ�ܸ�λУ׼ */
		ADC_ResetCalibration(ADC2);
		 
		/*�ȴ���λУ׼����*/
		while(ADC_GetResetCalibrationStatus(ADC2));
		
		/*����ADУ׼*/
		ADC_StartCalibration(ADC2);
	 
		/*�ȴ�У׼����*/
		while(ADC_GetCalibrationStatus(ADC2));
		
		/*ADC,ADCͨ��,����ʱ��Ϊ239.5����*/
		ADC_RegularChannelConfig(ADC2, ADC_Channel_12, 1, ADC_SampleTime_239Cycles5 );
	}
	
	/*��ˢ*/
	{
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC |RCC_APB2Periph_ADC2, ENABLE );
 
		/*����ADC��Ƶ����6 72M/6=12,ADC���ʱ�䲻�ܳ���14M*/
		RCC_ADCCLKConfig(RCC_PCLK2_Div6);
                  
		/*��������Ϊģ������ģʽ*/
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
		GPIO_Init(GPIOC, &GPIO_InitStructure);	

		/*��λADC*/
		ADC_DeInit(ADC2);

		ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;
		ADC_InitStructure.ADC_ScanConvMode = DISABLE;
		ADC_InitStructure.ADC_ContinuousConvMode = DISABLE;
		ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;
		ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
		ADC_InitStructure.ADC_NbrOfChannel = 1;
		ADC_Init(ADC2, &ADC_InitStructure);  

		/*ʹ��ָ����ADC*/
		ADC_Cmd(ADC2, ENABLE);
		
		/*ʹ�ܸ�λУ׼ */
		ADC_ResetCalibration(ADC2);
		 
		/*�ȴ���λУ׼����*/
		while(ADC_GetResetCalibrationStatus(ADC2));
		
		/*����ADУ׼*/
		ADC_StartCalibration(ADC2);
	 
		/*�ȴ�У׼����*/
		while(ADC_GetCalibrationStatus(ADC2));
		
		/*ADC,ADCͨ��,����ʱ��Ϊ239.5����*/
		ADC_RegularChannelConfig(ADC2, ADC_Channel_10, 1, ADC_SampleTime_239Cycles5 );
	}
 
	/*��ˢ*/
	{
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC |RCC_APB2Periph_ADC2, ENABLE );
 
		/*����ADC��Ƶ����6 72M/6=12,ADC���ʱ�䲻�ܳ���14M*/
		RCC_ADCCLKConfig(RCC_PCLK2_Div6);
                  
		/*��������Ϊģ������ģʽ*/
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
		GPIO_Init(GPIOC, &GPIO_InitStructure);	

		/*��λADC*/
		ADC_DeInit(ADC2);

		ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;
		ADC_InitStructure.ADC_ScanConvMode = DISABLE;
		ADC_InitStructure.ADC_ContinuousConvMode = DISABLE;
		ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;
		ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
		ADC_InitStructure.ADC_NbrOfChannel = 1;
		ADC_Init(ADC2, &ADC_InitStructure);  

		/*ʹ��ָ����ADC*/
		ADC_Cmd(ADC2, ENABLE);
		
		/*ʹ�ܸ�λУ׼ */
		ADC_ResetCalibration(ADC2);
		 
		/*�ȴ���λУ׼����*/
		while(ADC_GetResetCalibrationStatus(ADC2));
		
		/*����ADУ׼*/
		ADC_StartCalibration(ADC2);
	 
		/*�ȴ�У׼����*/
		while(ADC_GetCalibrationStatus(ADC2));
		
		/*ADC,ADCͨ��,����ʱ��Ϊ239.5����*/
		ADC_RegularChannelConfig(ADC2, ADC_Channel_11, 1, ADC_SampleTime_239Cycles5 );
	}
	
	
	/*��ص�ѹ*/
	{
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB |RCC_APB2Periph_ADC2, ENABLE );
 
		/*����ADC��Ƶ����6 72M/6=12,ADC���ʱ�䲻�ܳ���14M*/
		RCC_ADCCLKConfig(RCC_PCLK2_Div6);
                  
		/*��������Ϊģ������ģʽ*/
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
		GPIO_Init(GPIOB, &GPIO_InitStructure);	

		/*��λADC*/
		ADC_DeInit(ADC2);

		ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;
		ADC_InitStructure.ADC_ScanConvMode = DISABLE;
		ADC_InitStructure.ADC_ContinuousConvMode = DISABLE;
		ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;
		ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
		ADC_InitStructure.ADC_NbrOfChannel = 1;
		ADC_Init(ADC2, &ADC_InitStructure);  

		/*ʹ��ָ����ADC*/
		ADC_Cmd(ADC2, ENABLE);
		
		/*ʹ�ܸ�λУ׼ */
		ADC_ResetCalibration(ADC2);
		 
		/*�ȴ���λУ׼����*/
		while(ADC_GetResetCalibrationStatus(ADC2));
		
		/*����ADУ׼*/
		ADC_StartCalibration(ADC2);
	 
		/*�ȴ�У׼����*/
		while(ADC_GetCalibrationStatus(ADC2));
		
		/*ADC,ADCͨ��,����ʱ��Ϊ239.5����*/
		ADC_RegularChannelConfig(ADC2, ADC_Channel_8, 1, ADC_SampleTime_239Cycles5 );
	}
	
	/*������*/
	{
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOF |RCC_APB2Periph_ADC3, ENABLE );
 
		/*����ADC��Ƶ����6 72M/6=12,ADC���ʱ�䲻�ܳ���14M*/
		RCC_ADCCLKConfig(RCC_PCLK2_Div6);
                  
		/*��������Ϊģ������ģʽ*/
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
		GPIO_Init(GPIOF, &GPIO_InitStructure);	

		/*��λADC*/
		ADC_DeInit(ADC3);

		ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;
		ADC_InitStructure.ADC_ScanConvMode = DISABLE;
		ADC_InitStructure.ADC_ContinuousConvMode = DISABLE;
		ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;
		ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
		ADC_InitStructure.ADC_NbrOfChannel = 1;
		ADC_Init(ADC3, &ADC_InitStructure);  

		/*ʹ��ָ����ADC*/
		ADC_Cmd(ADC3, ENABLE);
		
		/*ʹ�ܸ�λУ׼ */
		ADC_ResetCalibration(ADC3);
		 
		/*�ȴ���λУ׼����*/
		while(ADC_GetResetCalibrationStatus(ADC3));
		
		/*����ADУ׼*/
		ADC_StartCalibration(ADC3);
	 
		/*�ȴ�У׼����*/
		while(ADC_GetCalibrationStatus(ADC3));
		
		/*ADC,ADCͨ��,����ʱ��Ϊ239.5����*/
		ADC_RegularChannelConfig(ADC3, ADC_Channel_4, 1, ADC_SampleTime_239Cycles5 );
	}
	
}


/*
*********************************************************************************************************
*	�� �� ��: bsp_GetFeedbackVoltage
*	����˵��: ���ط����ĵ�ѹ
*	��    �Σ���
*	�� �� ֵ: ��
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
	
	/*430  66.5�ǵ����ѹ  0.2�Ǹ���ʵ�����������ѹ*/
	batteryVoltage = (batteryVoltage * 430 / 66.5) + batteryVoltage + 0.2F; 
	batteryCurrent = batteryCurrent*1000.0F * 1000.0F / 10.0F / 50.0F; 
	wheelL = wheelL * 1000.0F * 1000.0F / 33.0F / 50.0F;
	wheelR = wheelR * 1000.0F * 1000.0F / 33.0F / 50.0F;
	roll = roll * 1000.0F * 1000.0F / 33.0F / 50.0F;
	vacuum = vacuum * 1000.0F * 1000.0F / 33.0F / 50.0F;
	sideBrush = sideBrush * 1000.0F * 1000.0F / 100.0F / 50.0F;
	
	
	DEBUG("����:%.2fmA  ����:%.2fmA  ���:%.2fmA  ��ˢ:%.2fmA  ��ˢ:%.2fmA  ��ص�ѹ:%.2fV  ��ص���:%.2fmA\r\n",
	wheelL,
	wheelR,
	vacuum,
	roll,
	sideBrush,
	batteryVoltage,
	batteryCurrent);


}





/*******************************************�˲�ר�ñ���**************************************************/

#define FILTER_ARR            512
#define ARR_FILTER_START      156
#define ARR_FILTER_END        356

float g_vMotorLeft        = 0.0F;
float g_vMotorRight       = 0.0F;
float g_vVacuum           = 0.0F;
float g_vRollingBrush     = 0.0F;
float g_vSideBrush        = 0.0F;
float g_vBatteryVoltage   = 0.0F;
float g_vBatteryCurrent   = 0.0F;

float vArrForFilter[FILTER_ARR] = {0};

/*
*********************************************************************************************************
*	�� �� ��: bsp_GetVoltageFilterProc
*	����˵��: �����˲��ķ�ʽ��ȡ��ѹ
*	��    �Σ���
*	�� �� ֵ: ��
*********************************************************************************************************
*/
void bsp_GetVoltageFilterProc(void)
{
	uint32_t i = 0 ;
	float sum = 0.0F;
	memset(vArrForFilter,0,FILTER_ARR);
	
	/*����*/
	sum = 0.0F;
	memset(vArrForFilter,0,FILTER_ARR);
	for(i=0;i<FILTER_ARR;++i)
	{
		vArrForFilter[i] = bsp_GetFeedbackVoltage(eMotorLeft);
	}
	sort_float(vArrForFilter,FILTER_ARR);
	for(i=ARR_FILTER_START;i<ARR_FILTER_END;++i)
	{
		sum += vArrForFilter[i];
	}
	g_vMotorLeft = sum / (float)(ARR_FILTER_END-ARR_FILTER_START);
	
	
	
	/*����*/
	sum = 0.0F;
	memset(vArrForFilter,0,FILTER_ARR);
	for(i=0;i<FILTER_ARR;++i)
	{
		vArrForFilter[i] = bsp_GetFeedbackVoltage(eMotorRight);
	}
	sort_float(vArrForFilter,FILTER_ARR);
	for(i=ARR_FILTER_START;i<ARR_FILTER_END;++i)
	{
		sum += vArrForFilter[i];
	}
	g_vMotorRight = sum / (float)(ARR_FILTER_END-ARR_FILTER_START);
	
	/*���*/
	sum = 0.0F;
	memset(vArrForFilter,0,FILTER_ARR);
	for(i=0;i<FILTER_ARR;++i)
	{
		vArrForFilter[i] = bsp_GetFeedbackVoltage(eVacuum);
	}
	sort_float(vArrForFilter,FILTER_ARR);
	for(i=ARR_FILTER_START;i<ARR_FILTER_END;++i)
	{
		sum += vArrForFilter[i];
	}
	g_vVacuum = sum / (float)(ARR_FILTER_END-ARR_FILTER_START);
	
	/*��ˢ*/
	sum = 0.0F;
	memset(vArrForFilter,0,FILTER_ARR);
	for(i=0;i<FILTER_ARR;++i)
	{
		vArrForFilter[i] = bsp_GetFeedbackVoltage(eRollingBrush);
	}
	sort_float(vArrForFilter,FILTER_ARR);
	for(i=ARR_FILTER_START;i<ARR_FILTER_END;++i)
	{
		sum += vArrForFilter[i];
	}
	g_vRollingBrush = sum / (float)(ARR_FILTER_END-ARR_FILTER_START);
	
	/*��ˢ*/
	sum = 0.0F;
	memset(vArrForFilter,0,FILTER_ARR);
	for(i=0;i<FILTER_ARR;++i)
	{
		vArrForFilter[i] = bsp_GetFeedbackVoltage(eSideBrush);
	}
	sort_float(vArrForFilter,FILTER_ARR);
	for(i=ARR_FILTER_START;i<ARR_FILTER_END;++i)
	{
		sum += vArrForFilter[i];
	}
	g_vSideBrush = sum / (float)(ARR_FILTER_END-ARR_FILTER_START);
	
	/*��ص�ѹ*/
	sum = 0.0F;
	memset(vArrForFilter,0,FILTER_ARR);
	for(i=0;i<FILTER_ARR;++i)
	{
		vArrForFilter[i] = bsp_GetFeedbackVoltage(eBatteryVoltage);
	}
	sort_float(vArrForFilter,FILTER_ARR);
	for(i=ARR_FILTER_START;i<ARR_FILTER_END;++i)
	{
		sum += vArrForFilter[i];
	}
	g_vBatteryVoltage = sum / (float)(ARR_FILTER_END-ARR_FILTER_START);	
	
	/*��ص���*/
	sum = 0.0F;
	memset(vArrForFilter,0,FILTER_ARR);
	for(i=0;i<FILTER_ARR;++i)
	{
		vArrForFilter[i] = bsp_GetFeedbackVoltage(eBatteryCurrent);
	}
	sort_float(vArrForFilter,FILTER_ARR);
	for(i=ARR_FILTER_START;i<ARR_FILTER_END;++i)
	{
		sum += vArrForFilter[i];
	}
	g_vBatteryCurrent = sum / (float)(ARR_FILTER_END-ARR_FILTER_START);	
}

