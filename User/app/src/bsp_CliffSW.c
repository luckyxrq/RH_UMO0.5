#include "bsp.h"

#define DELAY_FOR_READ_CLIFF_US            800
#define IS_OBSTACLE_CLIFF_MV               30   //�ϰ����ֵ��ѹ������

#define RCC_ALL_CLIFF_EMIT 	(RCC_APB2Periph_GPIOC)

#define GPIO_PORT_EMIT  GPIOC
#define GPIO_PIN_EMIT	GPIO_Pin_3

/*������ƣ����µĺ��ⷢ�䲢��һ������*/
#define CLIFF_EMIT_ENABLE()       GPIO_SetBits(GPIO_PORT_EMIT,GPIO_PIN_EMIT)
#define CLIFF_EMIT_DISABLE()      GPIO_ResetBits(GPIO_PORT_EMIT,GPIO_PIN_EMIT)

static void bsp_InitCliffEmit_GPIO(void);




/*
*********************************************************************************************************
*	�� �� ��: bsp_InitOffSiteSW
*	����˵��: ��ʼ�����´�����
*	��    ��: ��
*	�� �� ֵ: ��
*********************************************************************************************************
*/
void bsp_InitCliffSW(void)
{
	ADC_InitTypeDef ADC_InitStructure; 
	GPIO_InitTypeDef GPIO_InitStructure;
	
	/*��ʼ�����ⷢ������*/
	bsp_InitCliffEmit_GPIO();
	
	/*����1*/
	{
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA |RCC_APB2Periph_ADC2, ENABLE );
 
		/*����ADC��Ƶ����6 72M/6=12,ADC���ʱ�䲻�ܳ���14M*/
		RCC_ADCCLKConfig(RCC_PCLK2_Div6);
                  
		/*��������Ϊģ������ģʽ*/
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7;
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
		GPIO_Init(GPIOA, &GPIO_InitStructure);	

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
		ADC_RegularChannelConfig(ADC2, ADC_Channel_7, 1, ADC_SampleTime_239Cycles5 );
	}
	
	
	/*����2*/
	{
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA |RCC_APB2Periph_ADC2, ENABLE );
 
		/*����ADC��Ƶ����6 72M/6=12,ADC���ʱ�䲻�ܳ���14M*/
		RCC_ADCCLKConfig(RCC_PCLK2_Div6);
                  
		/*��������Ϊģ������ģʽ*/
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
		GPIO_Init(GPIOA, &GPIO_InitStructure);	

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
		ADC_RegularChannelConfig(ADC2, ADC_Channel_6, 1, ADC_SampleTime_239Cycles5 );
	}
	
	/*����3*/
	{
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOF |RCC_APB2Periph_ADC3, ENABLE );
 
		/*����ADC��Ƶ����6 72M/6=12,ADC���ʱ�䲻�ܳ���14M*/
		RCC_ADCCLKConfig(RCC_PCLK2_Div6);
                  
		/*��������Ϊģ������ģʽ*/
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
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
		ADC_RegularChannelConfig(ADC3, ADC_Channel_7, 1, ADC_SampleTime_239Cycles5 );
	}


}

/*
*********************************************************************************************************
*	�� �� ��: bsp_InitCliffEmit_GPIO
*	����˵��: ���ⷢ������
*	��    �Σ���
*	�� �� ֵ: ��
*********************************************************************************************************
*/
static void bsp_InitCliffEmit_GPIO(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;

	/* ��GPIOʱ�� */
	RCC_APB2PeriphClockCmd(RCC_ALL_CLIFF_EMIT, ENABLE);

	/*Ĭ�ϵ�ƽ*/
	CLIFF_EMIT_DISABLE();
	
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;	/* �������ģʽ */
	
	GPIO_InitStructure.GPIO_Pin = GPIO_PIN_EMIT;
	GPIO_Init(GPIO_PORT_EMIT, &GPIO_InitStructure);
	
	/*�����ٴθ���Ĭ�ϵ�ƽ������Ϊ������HAL��ӹ����±�׼����ͬ��������*/
	CLIFF_EMIT_DISABLE();
}


#define FILTER_ARR            32
#define ARR_FILTER_START      14
#define ARR_FILTER_END        18

static float vArrForFilter[FILTER_ARR] = {0};


/*
*********************************************************************************************************
*	�� �� ��: bsp_GetFeedbackVoltage
*	����˵��: ���ط����ĵ�ѹ
*	��    �Σ���
*	�� �� ֵ: ��
*********************************************************************************************************
*/
float bsp_GetCliffVoltage(CliffSWSN sn)
{
	float sum = 0.0F;
	float ret = 0.0F;
	float adc = 0 ;
	uint32_t i = 0 ;
	
	
	memset(vArrForFilter,0,FILTER_ARR);
	
	switch(sn)
	{
		case CliffLeft:
		{
			for(i = 0;i<FILTER_ARR;++i)
			{
				ADC_RegularChannelConfig(ADC2, ADC_Channel_7, 1, ADC_SampleTime_239Cycles5 );
				ADC_SoftwareStartConvCmd(ADC2, ENABLE);	
				while(!ADC_GetFlagStatus(ADC2, ADC_FLAG_EOC ));
				adc = ADC_GetConversionValue(ADC2) * 3.3F / 4096;
				
				vArrForFilter[i] = adc;
			}
			
			sort_float(vArrForFilter,FILTER_ARR);
			for(i=ARR_FILTER_START;i<ARR_FILTER_END;++i)
			{
				sum += vArrForFilter[i];
			}
			
			ret = sum / (float)(ARR_FILTER_END-ARR_FILTER_START);
		}break;
		
		case CliffMiddle:
		{
			for(i = 0;i<FILTER_ARR;++i)
			{
				ADC_RegularChannelConfig(ADC2, ADC_Channel_6, 1, ADC_SampleTime_239Cycles5 );
				ADC_SoftwareStartConvCmd(ADC2, ENABLE);	
				while(!ADC_GetFlagStatus(ADC2, ADC_FLAG_EOC ));
				adc = ADC_GetConversionValue(ADC2) * 3.3F / 4096;
				
				vArrForFilter[i] =  adc;
			}
			
			sort_float(vArrForFilter,FILTER_ARR);
			for(i=ARR_FILTER_START;i<ARR_FILTER_END;++i)
			{
				sum += vArrForFilter[i];
			}
			
			ret = sum / (float)(ARR_FILTER_END-ARR_FILTER_START);
		}break;
		
		case CliffRight:
		{
			for(i = 0;i<FILTER_ARR;++i)
			{
				ADC_RegularChannelConfig(ADC3, ADC_Channel_7, 1, ADC_SampleTime_239Cycles5 );
				ADC_SoftwareStartConvCmd(ADC3, ENABLE);	
				while(!ADC_GetFlagStatus(ADC3, ADC_FLAG_EOC ));
				adc = ADC_GetConversionValue(ADC3) * 3.3F / 4096;
				
				vArrForFilter[i] =  adc;
			}
			
			sort_float(vArrForFilter,FILTER_ARR);
			for(i=ARR_FILTER_START;i<ARR_FILTER_END;++i)
			{
				sum += vArrForFilter[i];
			}
			
			ret = sum / (float)(ARR_FILTER_END-ARR_FILTER_START);

		}break;
	}
	
	return ret;
}

/*�������´�������ÿ��������*/
static float cliffTwiceRead[3][2];
static uint8_t cliffStates = 0x00;
static float cliffSub[3] = {0};


/*
*********************************************************************************************************
*	�� �� ��: bsp_CliffIsDangerous
*	����˵��: �ж������Ƿ�Σ��
*	��    �Σ���
*	�� �� ֵ: ��
*********************************************************************************************************
*/
uint8_t bsp_GetCliffStates(void)
{
	uint8_t data = 0 ;
	
	/*�������*/
	CLIFF_EMIT_ENABLE();
	bsp_DelayUS(DELAY_FOR_READ_CLIFF_US);
	/*һ�ζ�����*/
	cliffTwiceRead [0][0] = bsp_GetCliffVoltage(CliffLeft);
	cliffTwiceRead [1][0] = bsp_GetCliffVoltage(CliffMiddle);
	cliffTwiceRead [2][0] = bsp_GetCliffVoltage(CliffRight);
	
	
	/*�ط����*/
	CLIFF_EMIT_DISABLE();
	bsp_DelayUS(DELAY_FOR_READ_CLIFF_US);
	/*һ�ζ�����*/
	cliffTwiceRead [0][1] = bsp_GetCliffVoltage(CliffLeft);
	cliffTwiceRead [1][1] = bsp_GetCliffVoltage(CliffMiddle);
	cliffTwiceRead [2][1] = bsp_GetCliffVoltage(CliffRight);
	
	
	cliffSub[0] = abs((cliffTwiceRead [0][1] - cliffTwiceRead [0][0])*1000);
	cliffSub[1] = abs((cliffTwiceRead [1][1] - cliffTwiceRead [1][0])*1000);
	cliffSub[2] = abs((cliffTwiceRead [2][1] - cliffTwiceRead [2][0])*1000);
	
	if( cliffSub[0] <= IS_OBSTACLE_CLIFF_MV )
	{
		data |= 1<< 0;
	}
	
	if( cliffSub[1] <= IS_OBSTACLE_CLIFF_MV )
	{
		data |= 1<< 1;
	}
	
	if( cliffSub[2] <= IS_OBSTACLE_CLIFF_MV )
	{
		data |= 1<< 2;
	}
	
	RTT("cliffSub[0]:%d\r\n",(int)cliffSub[0]);
	RTT("cliffSub[1]:%d\r\n",(int)cliffSub[1]);
	RTT("cliffSub[2]:%d\r\n",(int)cliffSub[2]);
	
	
	
	cliffStates = data;
	
	return data;
}

void bsp_GetCliffSub(float arr[])
{
	arr[0] = cliffSub[0];
	arr[1] = cliffSub[1];
	arr[2] = cliffSub[2];
}




bool bsp_CliffIsDangerous(CliffSWSN sn)
{
	
	if(sn == CliffLeft && (cliffStates&(1<<0)))
	{
		return true;
	}
	else if(sn == CliffMiddle && (cliffStates&(1<<1)))
	{
		return true;
	}
	else if(sn == CliffRight && (cliffStates&(1<<2)))
	{
		return true;
	}
	
	
	return false;
}

float bsp_GetCliffRealVal(CliffSWSN sn)
{
	return cliffSub[sn % 3];
}


void bsp_PrintCliff(void)
{
	DEBUG("CLIFF:%d %d %d\r\n",
	bsp_CliffIsDangerous(CliffLeft),
	bsp_CliffIsDangerous(CliffMiddle),
	bsp_CliffIsDangerous(CliffRight));
}





#define GO_BACK_PULSE                  (10/(3.14F*70)*1024)
#define ROTATE_CCW_SPEED_L             -5
#define ROTATE_CCW_SPEED_R             5

static void bsp_RotateCCW(void);
static float myabs(float val);


typedef struct
{
	bool isRunning;
	uint32_t delay;
	
	volatile uint8_t action  ;
	volatile uint32_t pulse  ;
	volatile float angle ;
	
}CliffTest;

static CliffTest cliffTest;


void bsp_StartCliffTest(void)
{
	cliffTest.delay = 0 ;
	cliffTest.action = 0;
	cliffTest.pulse = 0 ;
	cliffTest.angle = 0 ;
	
	cliffTest.isRunning = true;
}

void bsp_StopCliffTest(void)
{
	cliffTest.isRunning = false;
	
	cliffTest.delay = 0 ;
	cliffTest.action = 0;
	cliffTest.pulse = 0 ;
	cliffTest.angle = 0 ;
	
	bsp_SetMotorSpeed(MotorLeft, 0);
	bsp_SetMotorSpeed(MotorRight,0);
	
}


static bool isErlangGod = false;

void bsp_CliffTest(void)
{
//	if(!cliffTest.isRunning)
//	{
//		isErlangGod = false;
//		return;
//	}
	
	switch(cliffTest.action)
	{
		case 0: /*ֱ��*/
		{

			if(bsp_GetInfraRedAdcVoltage(IR7) >= 1.0F
				||bsp_GetInfraRedAdcVoltage(IR2) >= 1.0F
				||bsp_GetInfraRedAdcVoltage(IR3) >= 1.0F
				||bsp_GetInfraRedAdcVoltage(IR4) >= 1.0F)
			{
				isErlangGod = true;
			}
			
			
			cliffTest.action++;
		}break;
		
		case 1: /*����Ƿ������´�����*/
		{
//			if(bsp_CliffIsDangerous(CliffLeft) ||
//				bsp_CliffIsDangerous(CliffMiddle) ||
//			    bsp_CliffIsDangerous(CliffRight) ||
//			    bsp_CollisionScan() != CollisionNone)
			if(bsp_CliffIsDangerous(CliffLeft) ||
				bsp_CliffIsDangerous(CliffMiddle) ||
			    bsp_CliffIsDangerous(CliffRight))
			{
				DEBUG("���´���\r\n");
				bsp_SetMotorSpeed(MotorLeft, 0);
			    bsp_SetMotorSpeed(MotorRight,0);
				cliffTest.action++;
				
				isErlangGod = false;
			}
			else
				
			{
				if(isErlangGod)
				{
					bsp_SetMotorSpeed(MotorLeft, 6);
					bsp_SetMotorSpeed(MotorRight,6);
				}
				else
				{
					bsp_SetMotorSpeed(MotorLeft, 12);
					bsp_SetMotorSpeed(MotorRight,12);
				}
				cliffTest.action = 0 ;
			}
		}break;
		
		case 2:
		{
			cliffTest.pulse = bsp_GetCurrentBothPulse();
			bsp_SetMotorSpeed(MotorLeft, -6);
			bsp_SetMotorSpeed(MotorRight,-6);
			cliffTest.action++;
		}break;
		
		case 3:
		{
			if(bsp_GetCurrentBothPulse()-cliffTest.pulse >= GO_BACK_PULSE*5)
			{
				cliffTest.angle = bsp_AngleRead();
				bsp_RotateCCW();
				cliffTest.action++;
			}
		}break;
		
		case 4:
		{
			if(myabs(bsp_AngleAdd(cliffTest.angle ,20) - (bsp_AngleRead())) <= 2.0F)
			{
				bsp_SetMotorSpeed(MotorLeft, 6);
				bsp_SetMotorSpeed(MotorRight,6);
				cliffTest.action = 0 ;
			}
		}break;
	}
}


/*
*********************************************************************************************************
*	�� �� ��: bsp_RotateCCW
*	����˵��: ԭ����ת�������ֶ�������ʱ��
*	��    ��: ��
*	�� �� ֵ: ��
*********************************************************************************************************
*/
static void bsp_RotateCCW(void)
{
	bsp_SetMotorSpeed(MotorLeft, ROTATE_CCW_SPEED_L);
	bsp_SetMotorSpeed(MotorRight,ROTATE_CCW_SPEED_R);
}


static float myabs(float val)
{
	if(val < 0)
	{
		val = - val;
	}
	
	return val;
}
