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
	uint32_t stopTickRTOS = 0;
	static bool isClose = false;

	
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
	
	
	DEBUG("����:%.2fmA  ����:%.2fmA  ���:%.2fmA  ��ˢ:%.2fmA  ��ˢ:%.2fmA  ��ص�ѹ:%.2fV  ��ص���:%.2fmA ������������:%d ����У�%s ������%s\r\n",
	wheelL,
	wheelR,
	vacuum,
	roll,
	sideBrush,
	batteryVoltage,
	batteryCurrent,
	stopTickRTOS,
	bsp_IsCharging()?"true":"fasle",
	bsp_IsChargeDone()?"true":"fasle");



	if(isClose)
		return;

	if(batteryVoltage <= 12.0F)
	{
		/*�رո���״̬��*/
		bsp_StopSearchChargePile();
		bsp_StopCliffTest();
		bsp_StopUpdateCleanStrategyB();
		bsp_StopVacuum();
		bsp_MotorCleanSetPWM(MotorRollingBrush, CCW , 0);
		bsp_MotorCleanSetPWM(MotorSideBrush, CW , 0);
		
		isClose = true;
		
		stopTickRTOS = xTaskGetTickCount();
		
		/*��������*/
		bsp_SperkerPlay(Song6);
	}
}



float bsp_PowerOn_DetectVoltage(void)
{
	float batteryVoltage = bsp_GetFeedbackVoltage(eBatteryVoltage);
	batteryVoltage = (batteryVoltage * 430 / 66.5) + batteryVoltage + 0.2F; 
	
	if(batteryVoltage <= 13.0F)
	{
		bsp_SperkerPlay(Song6); /*���س��*/
			
		while(bsp_SpeakerIsBusy()){}
			
		bsp_PutKey(KEY_LONG_POWER);
			
	}
	else
	{
		bsp_SperkerPlay(Song1); /*����*/
			
		while(bsp_SpeakerIsBusy()){}
	}
	
}







