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

	/*����*/
	sum = 0.0F;
	arrMotorLeft[arr_index] = bsp_GetFeedbackVoltage(eMotorLeft);
	sort_float(arrMotorLeft,FILTER_ARR);
	for(i=ARR_FILTER_START;i<ARR_FILTER_END;++i)
	{
		sum += arrMotorLeft[i];
	}
	g_vMotorLeft = sum / (float)(ARR_FILTER_END-ARR_FILTER_START);
	g_vMotorLeft = g_vMotorLeft * 1000.0F * 1000.0F / 33.0F / 50.0F;
	
	/*����*/
	sum = 0.0F;
	arrMotorRight[arr_index] = bsp_GetFeedbackVoltage(eMotorRight);
	sort_float(arrMotorRight,FILTER_ARR);
	for(i=ARR_FILTER_START;i<ARR_FILTER_END;++i)
	{
		sum += arrMotorRight[i];
	}
	g_vMotorRight = sum / (float)(ARR_FILTER_END-ARR_FILTER_START);
	g_vMotorRight = g_vMotorRight * 1000.0F * 1000.0F / 33.0F / 50.0F;
	
	/*���*/
	sum = 0.0F;
	arrVacuum[arr_index] = bsp_GetFeedbackVoltage(eVacuum);
	sort_float(arrVacuum,FILTER_ARR);
	for(i=ARR_FILTER_START;i<ARR_FILTER_END;++i)
	{
		sum += arrVacuum[i];
	}
	g_vVacuum = sum / (float)(ARR_FILTER_END-ARR_FILTER_START);
	g_vVacuum = g_vVacuum * 1000.0F * 1000.0F / 33.0F / 50.0F;
	
	/*��ˢ*/
	sum = 0.0F;
	arrRollingBrush[arr_index] = bsp_GetFeedbackVoltage(eRollingBrush);
	sort_float(arrRollingBrush,FILTER_ARR);
	for(i=ARR_FILTER_START;i<ARR_FILTER_END;++i)
	{
		sum += arrRollingBrush[i];
	}
	g_vRollingBrush = sum / (float)(ARR_FILTER_END-ARR_FILTER_START);
	g_vRollingBrush = g_vRollingBrush * 1000.0F * 1000.0F / 33.0F / 50.0F;
	
	/*��ˢ*/
	sum = 0.0F;
	arrSideBrush[arr_index] = bsp_GetFeedbackVoltage(eSideBrush);
	sort_float(arrSideBrush,FILTER_ARR);
	for(i=ARR_FILTER_START;i<ARR_FILTER_END;++i)
	{
		sum += arrSideBrush[i];
	}
	g_vSideBrush = sum / (float)(ARR_FILTER_END-ARR_FILTER_START);
	g_vSideBrush = g_vSideBrush * 1000.0F * 1000.0F / 100.0F / 50.0F;

	/*��ص�ѹ*/
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
	
	/*��ص���*/
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
*	�� �� ��: bsp_GetVoltageAfterFilter
*	����˵��: ���ط����ĵ�ѹ
*	��    �Σ���
*	�� �� ֵ: ��
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
