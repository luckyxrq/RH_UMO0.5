#include "bsp.h"

#define VOLTAGE_FILTERING_COUNT      50


/*
*********************************************************************************************************
*	函 数 名: bsp_InitOffSiteSW
*	功能说明: 初始化跳崖传感器
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

	/*开机初始化ADC的时候，校准一次悬崖传感器校准值*/
	bsp_CliffCalibration();

}



/*
*********************************************************************************************************
*	函 数 名: bsp_GetFeedbackVoltage
*	功能说明: 返回反馈的电压
*	形    参：无
*	返 回 值: 无
*********************************************************************************************************
*/
float bsp_GetCliffVoltage(CliffSWSN sn)
{
	float ret = 0 ;
	
	switch(sn)
	{
		case CliffLeft:
		{
			ADC_RegularChannelConfig(ADC2, ADC_Channel_7, 1, ADC_SampleTime_239Cycles5 );
			ADC_SoftwareStartConvCmd(ADC2, ENABLE);	
			while(!ADC_GetFlagStatus(ADC2, ADC_FLAG_EOC ));
			ret = ADC_GetConversionValue(ADC2) * 3.3F / 4096;
		}break;
		
		case CliffMiddle:
		{
			ADC_RegularChannelConfig(ADC2, ADC_Channel_6, 1, ADC_SampleTime_239Cycles5 );
			ADC_SoftwareStartConvCmd(ADC2, ENABLE);	
			while(!ADC_GetFlagStatus(ADC2, ADC_FLAG_EOC ));
			ret = ADC_GetConversionValue(ADC2) * 3.3F / 4096;
		}break;
		
		case CliffRight:
		{
			ADC_RegularChannelConfig(ADC3, ADC_Channel_7, 1, ADC_SampleTime_239Cycles5 );
			ADC_SoftwareStartConvCmd(ADC3, ENABLE);	
			while(!ADC_GetFlagStatus(ADC3, ADC_FLAG_EOC ));
			ret = ADC_GetConversionValue(ADC3) * 3.3F / 4096;
		}break;
	}
	
	return ret;
}


#define DEFAULT_DANGEROUS_THRESHOLD    1.2F        /*默认的跳崖阈值*/
#define CLIFF_COUNT                    3           /*跳崖传感器个数*/   

typedef struct
{
	float threshold;          /*跳崖阈值，每个通道的阈值独立*/
	float initializeVoltage;  /*初始化电压，适应新环境的电压*/
	float currentVoltage;     /*当前电压，实时检测的电压*/
}Cliff;


static Cliff cliff[CLIFF_COUNT];

/*
*********************************************************************************************************
*	函 数 名: bsp_CliffCalibration
*	功能说明: 开机校准悬崖初始值
*	形    参：无
*	返 回 值: 无
*********************************************************************************************************
*/
void bsp_CliffCalibration(void)
{
	/*初始值*/
#if 1
	uint8_t i = 0 ;
	double sum = 0 ;
	
	/*左边初始值*/
	sum = 0 ;
	for(i=0;i<VOLTAGE_FILTERING_COUNT;i++)
	{
		sum += bsp_GetCliffVoltage(CliffLeft);
	}
	cliff[CliffLeft].initializeVoltage = sum / VOLTAGE_FILTERING_COUNT;
	
	/*中间初始值*/
	sum = 0 ;
	for(i=0;i<VOLTAGE_FILTERING_COUNT;i++)
	{
		sum += bsp_GetCliffVoltage(CliffMiddle);
	}
	cliff[CliffMiddle].initializeVoltage = sum / VOLTAGE_FILTERING_COUNT;
	
	/*右边初始值*/
	sum = 0 ;
	for(i=0;i<VOLTAGE_FILTERING_COUNT;i++)
	{
		sum += bsp_GetCliffVoltage(CliffRight);
	}
	cliff[CliffRight].initializeVoltage = sum / VOLTAGE_FILTERING_COUNT;
	
	/*阈值*/
	cliff[CliffLeft].threshold =   1.6F;
	cliff[CliffMiddle].threshold = 1.6F;
	cliff[CliffRight].threshold =  1.6F;
#else
	cliff[CliffLeft].initializeVoltage =   3.3F;
	cliff[CliffMiddle].initializeVoltage = 3.3F;
	cliff[CliffRight].initializeVoltage =  3.3F;
	
	cliff[CliffLeft].threshold =   1.6F;
	cliff[CliffMiddle].threshold = 1.6F;
	cliff[CliffRight].threshold =  1.6F;
#endif
	
	UNUSED(cliff);
}

/*
*********************************************************************************************************
*	函 数 名: bsp_CliffIsDangerous
*	功能说明: 判断跳崖是否危险
*	形    参：无
*	返 回 值: 无
*********************************************************************************************************
*/
bool bsp_CliffIsDangerous(CliffSWSN sn)
{

	cliff[sn].currentVoltage = bsp_GetCliffVoltage(sn);
	if(cliff[sn].initializeVoltage - cliff[sn].currentVoltage >= cliff[sn].threshold)
	{
		return true;
	}
	
	return false;
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
	if(!cliffTest.isRunning)
	{
		isErlangGod = false;
		return;
	}
	
	switch(cliffTest.action)
	{
		case 0: /*直行*/
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
		
		case 1: /*检测是否有悬崖触发了*/
		{
			if(bsp_CliffIsDangerous(CliffLeft) ||
				bsp_CliffIsDangerous(CliffMiddle) ||
			    bsp_CliffIsDangerous(CliffRight) ||
			    bsp_CollisionScan() != CollisionNone)
			{
				DEBUG("悬崖触发\r\n");
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
*	函 数 名: bsp_RotateCCW
*	功能说明: 原地旋转，左右轮都动，逆时针
*	形    参: 无
*	返 回 值: 无
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
