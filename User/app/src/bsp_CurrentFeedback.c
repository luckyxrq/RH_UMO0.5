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
