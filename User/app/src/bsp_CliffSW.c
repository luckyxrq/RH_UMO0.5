#include "bsp.h"

#define DELAY_FOR_READ_CLIFF_US            800
#define IS_OBSTACLE_CLIFF_MV               30   //障碍物差值电压，毫伏

#define RCC_ALL_CLIFF_EMIT 	(RCC_APB2Periph_GPIOC)

#define GPIO_PORT_EMIT  GPIOC
#define GPIO_PIN_EMIT	GPIO_Pin_3

/*发射控制，悬崖的红外发射并在一根线上*/
#define CLIFF_EMIT_ENABLE()       GPIO_SetBits(GPIO_PORT_EMIT,GPIO_PIN_EMIT)
#define CLIFF_EMIT_DISABLE()      GPIO_ResetBits(GPIO_PORT_EMIT,GPIO_PIN_EMIT)

static void bsp_InitCliffEmit_GPIO(void);




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
	
	/*初始化红外发射引脚*/
	bsp_InitCliffEmit_GPIO();
	
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

/*
*********************************************************************************************************
*	函 数 名: bsp_InitCliffEmit_GPIO
*	功能说明: 红外发射引脚
*	形    参：无
*	返 回 值: 无
*********************************************************************************************************
*/
static void bsp_InitCliffEmit_GPIO(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;

	/* 打开GPIO时钟 */
	RCC_APB2PeriphClockCmd(RCC_ALL_CLIFF_EMIT, ENABLE);

	/*默认电平*/
	CLIFF_EMIT_DISABLE();
	
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;	/* 推挽输出模式 */
	
	GPIO_InitStructure.GPIO_Pin = GPIO_PIN_EMIT;
	GPIO_Init(GPIO_PORT_EMIT, &GPIO_InitStructure);
	
	/*这里再次给予默认电平，是因为曾经被HAL库坑过，怕标准库有同样的问题*/
	CLIFF_EMIT_DISABLE();
}


#define FILTER_ARR            32
#define ARR_FILTER_START      14
#define ARR_FILTER_END        18

static float vArrForFilter[FILTER_ARR] = {0};


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

/*三个跳崖传感器，每个读两次*/
static float cliffTwiceRead[3][2];
static uint8_t cliffStates = 0x00;
static float cliffSub[3] = {0};


/*
*********************************************************************************************************
*	函 数 名: bsp_CliffIsDangerous
*	功能说明: 判断跳崖是否危险
*	形    参：无
*	返 回 值: 无
*********************************************************************************************************
*/
uint8_t bsp_GetCliffStates(void)
{
	uint8_t data = 0 ;
	
	/*开发射读*/
	CLIFF_EMIT_ENABLE();
	bsp_DelayUS(DELAY_FOR_READ_CLIFF_US);
	/*一次读三个*/
	cliffTwiceRead [0][0] = bsp_GetCliffVoltage(CliffLeft);
	cliffTwiceRead [1][0] = bsp_GetCliffVoltage(CliffMiddle);
	cliffTwiceRead [2][0] = bsp_GetCliffVoltage(CliffRight);
	
	
	/*关发射读*/
	CLIFF_EMIT_DISABLE();
	bsp_DelayUS(DELAY_FOR_READ_CLIFF_US);
	/*一次读三个*/
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
//			if(bsp_CliffIsDangerous(CliffLeft) ||
//				bsp_CliffIsDangerous(CliffMiddle) ||
//			    bsp_CliffIsDangerous(CliffRight) ||
//			    bsp_CollisionScan() != CollisionNone)
			if(bsp_CliffIsDangerous(CliffLeft) ||
				bsp_CliffIsDangerous(CliffMiddle) ||
			    bsp_CliffIsDangerous(CliffRight))
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
