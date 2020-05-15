#include "bsp.h"



#define PIN_MAP_MAX		10  //引脚对数
#define IntervalTime	2   //每对管子扫描间隔时间

#define DELAY_FOR_READ_US      200
#define IS_OBSTACLE_MV         60  //障碍物差值电压，毫伏

#define IR_OBSTACLE_0_6      60
#define IR_OBSTACLE_7        60  /*二郎神*/
#define IR_OBSTACLE_8        100 /*左沿边*/
#define IR_OBSTACLE_9        100 /*右沿边*/

AW_PIN PinMap[PIN_MAP_MAX][2]=
{
	{awP1_5,awP0_7},   //1
	{awP1_5,awP0_5},   //2
	{awP1_7,awP0_4},   //3
	{awP1_7,awP0_3},   //4
	{awP1_0,awP0_2},   //5
	{awP1_1,awP0_1},   //6
	{awP1_2,awP0_0},   //7
	{awP1_6,awP0_6},   //8中
	{awP1_5,awP1_4},   //Left
	{awP1_2,awP1_3},   //Right
};



static DetectAct detectAct;
static float adcContrast[PIN_MAP_MAX][2]; //开发射前后的电压值
static float adcRealTime[PIN_MAP_MAX];    //判断是否有障碍物
static uint8_t adcIsSunlight[PIN_MAP_MAX];//是否是太阳光

static void bsp_ADCConfig(void);


/*
*********************************************************************************************************
*	函 数 名: bsp_InitDetectAct
*	功能说明: 配置碰撞检测扫描状态机。
*	形    参: 无
*	返 回 值: 无
*********************************************************************************************************
*/
void bsp_InitDetectAct(void)
{
	UNUSED(adcRealTime);
	UNUSED(adcIsSunlight);
	
	bsp_ADCConfig();
	
	detectAct.isRunning = 0 ;
	detectAct.action = 0 ;
	detectAct.delay = 0 ;
	detectAct.adcVal = 0 ;
	detectAct.pinMapIndex = 0 ;
	
}

/*
*********************************************************************************************************
*	函 数 名: bsp_DetectStart
*	功能说明: 开启碰撞检测扫描状态机。
*	形    参: 无
*	返 回 值: 无
*********************************************************************************************************
*/
void bsp_DetectStart(void)
{
	detectAct.action = 0 ;
	detectAct.delay = 0 ;
	detectAct.adcVal = 0 ;
	detectAct.pinMapIndex = 0 ;
	detectAct.isRunning = 1 ;
}

/*
*********************************************************************************************************
*	函 数 名: bsp_DetectStart
*	功能说明: 关闭碰撞检测扫描状态机。
*	形    参: 无
*	返 回 值: 无
*********************************************************************************************************
*/
void bsp_DetectStop(void)
{
	detectAct.isRunning = 0 ;
	detectAct.action = 0 ;
	detectAct.delay = 0 ;
	detectAct.adcVal = 0 ;
	detectAct.pinMapIndex = 0 ;
}


void bsp_DetectActTest(uint8_t pinMapIndex)
{

}


/*
*********************************************************************************************************
*	函 数 名: bsp_DetectAct
*	功能说明: 按顺序扫描。
*	形    参: 无
*	返 回 值: 无
*********************************************************************************************************
*/
void bsp_DetectAct(void)
{
	if( !detectAct.isRunning )
		return ;
	
	
	switch(detectAct.action)
	{
		case 0:
		{
			//if(detectAct.pinMapIndex == 3)
			if(true)
			{
				/*开 读*/
				bsp_AWSetPinVal(PinMap[detectAct.pinMapIndex%PIN_MAP_MAX][1], AW_0); //先开接收，充电
				bsp_AWSetPinVal(PinMap[detectAct.pinMapIndex%PIN_MAP_MAX][0], AW_0); //再开发送，读AD
				bsp_DelayUS(DELAY_FOR_READ_US);
				adcContrast[detectAct.pinMapIndex%PIN_MAP_MAX][0] = bsp_GetAdScanValue();//开灯读
				/*关 读*/
				bsp_AWSetPinVal(PinMap[detectAct.pinMapIndex%PIN_MAP_MAX][0], AW_1);//关发射
				bsp_DelayUS(DELAY_FOR_READ_US);
				adcContrast[detectAct.pinMapIndex%PIN_MAP_MAX][1] = bsp_GetAdScanValue();//关灯读
				bsp_AWSetPinVal(PinMap[detectAct.pinMapIndex%PIN_MAP_MAX][1], AW_1);//关接收
				/*判断,adcRealTime 1.0F表示障碍物，0.0F表示无障碍物，为了兼容以前的框架，没有使用BOOL类型，但是算法那边判断切勿使用==1.0F  ==0.0F之类的，浮点数据不能这么判断*/
				//adcRealTime[detectAct.pinMapIndex%PIN_MAP_MAX] = (abs((adcContrast[detectAct.pinMapIndex%PIN_MAP_MAX][1] - adcContrast[detectAct.pinMapIndex%PIN_MAP_MAX][0])*1000) >= IS_OBSTACLE_MV) ? 1.0F : 0.0F;
				adcRealTime[detectAct.pinMapIndex%PIN_MAP_MAX] = abs((adcContrast[detectAct.pinMapIndex%PIN_MAP_MAX][1] - adcContrast[detectAct.pinMapIndex%PIN_MAP_MAX][0])*1000);			
				if(adcContrast[detectAct.pinMapIndex%PIN_MAP_MAX][1] < adcContrast[detectAct.pinMapIndex%PIN_MAP_MAX][0])
				{
					adcRealTime[detectAct.pinMapIndex%PIN_MAP_MAX] = 0;
				}
			}
			else
			{
				bsp_DelayUS(DELAY_FOR_READ_US);
				bsp_DelayUS(DELAY_FOR_READ_US);
				bsp_DelayUS(DELAY_FOR_READ_US);
			}
			
			detectAct.delay = xTaskGetTickCount();
			detectAct.action++;
			
		}break;
		
		case 1:
		{
			if(xTaskGetTickCount() - detectAct.delay >= IntervalTime)
			{
				detectAct.action = 0 ;
				
				++detectAct.pinMapIndex;
				if(detectAct.pinMapIndex >= PIN_MAP_MAX)
				{
					detectAct.pinMapIndex = 0 ;
				}
			}
		}break;
	
	}
}


/*
*********************************************************************************************************
*	函 数 名: ADC_Configuration
*	功能说明: 配置ADC, PC4作为ADC通道输入
*	形    参：无
*	返 回 值: 无
*********************************************************************************************************
*/
static void bsp_ADCConfig(void)
{
	ADC_InitTypeDef ADC_InitStructure; 
	GPIO_InitTypeDef GPIO_InitStructure;

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC |RCC_APB2Periph_ADC2	, ENABLE );	  //使能ADC2通道时钟
 

	RCC_ADCCLKConfig(RCC_PCLK2_Div6);   //设置ADC分频因子6 72M/6=12,ADC最大时间不能超过14M

	//PA1 作为模拟通道输入引脚                         
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;		//模拟输入引脚
	GPIO_Init(GPIOC, &GPIO_InitStructure);	

	ADC_DeInit(ADC2);  //复位ADC2 

	ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;	//ADC工作模式:ADC2和ADC2工作在独立模式
	ADC_InitStructure.ADC_ScanConvMode = DISABLE;	//模数转换工作在单通道模式
	ADC_InitStructure.ADC_ContinuousConvMode = DISABLE;	//模数转换工作在单次转换模式
	ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;	//转换由软件而不是外部触发启动
	ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;	//ADC数据右对齐
	ADC_InitStructure.ADC_NbrOfChannel = 1;	//顺序进行规则转换的ADC通道的数目
	ADC_Init(ADC2, &ADC_InitStructure);	//根据ADC_InitStruct中指定的参数初始化外设ADCx的寄存器   

  
	ADC_Cmd(ADC2, ENABLE);	//使能指定的ADC2
	
	ADC_ResetCalibration(ADC2);	//使能复位校准  
	 
	while(ADC_GetResetCalibrationStatus(ADC2));	//等待复位校准结束
	
	ADC_StartCalibration(ADC2);	 //开启AD校准
 
	while(ADC_GetCalibrationStatus(ADC2));	 //等待校准结束
 
}



/*
*********************************************************************************************************
*	函 数 名: AdcPro
*	功能说明: ADC采样处理，插入1ms systick 中断进行调用
*	形    参：无
*	返 回 值: 无
*********************************************************************************************************
*/
float bsp_GetAdScanValue(void)
{
	uint16_t ret;
	uint8_t i = 0 ;
	uint8_t sampleCount = 3 ; 
	uint32_t sum = 0 ;
	float voltage;
	
	
	for(i = 0;i<sampleCount;i++)
	{
		//设置指定ADC的规则组通道，一个序列，采样时间
		ADC_RegularChannelConfig(ADC2, ADC_Channel_14, 1, ADC_SampleTime_239Cycles5 );	//ADC2,ADC通道,采样时间为239.5周期	  			    
	  
		ADC_SoftwareStartConvCmd(ADC2, ENABLE);		//使能指定的ADC2的软件转换启动功能	
		 
		while(!ADC_GetFlagStatus(ADC2, ADC_FLAG_EOC ));//等待转换结束
		ret = ADC_GetConversionValue(ADC2);	//返回最近一次ADC2规则组的转换结果
		
		sum += ret;
	}
	
	voltage = (sum / (float)sampleCount)*3.3F/4096;

	return voltage;
}


/*
*********************************************************************************************************
*	函 数 名: bsp_GetInfraredVoltageLeft
*	功能说明: 获取左边的红外电压值
*	形    参: 无
*	返 回 值: 无
*********************************************************************************************************
*/
float bsp_GetInfraredVoltageLeft(void)
{
	return adcRealTime[8];
}

/*
*********************************************************************************************************
*	函 数 名: bsp_GetInfraredVoltageRight
*	功能说明: 获取右边的红外电压值
*	形    参: 无
*	返 回 值: 无
*********************************************************************************************************
*/
float bsp_GetInfraredVoltageRight(void)
{
	return adcRealTime[9];
}

/*
*********************************************************************************************************
*	函 数 名: bsp_GetInfraRedAdcVoltage
*	功能说明: 获取红外扫描管的电压
*	形    参: 无
*	返 回 值: 无
*********************************************************************************************************
*/
float bsp_GetInfraRedAdcVoltage(IR_SN sn)
{
	return adcRealTime[sn];
}


/*
*********************************************************************************************************
*	函 数 名: bsp_GetAllIrIsObstacle
*	功能说明: 获取所有红外障碍物信息
*	形    参: 传入bool数组  切记数组为10个元素
*	返 回 值: 无
*********************************************************************************************************
*/
void bsp_GetAllIrIsObstacle(uint8_t ret[])
{
	uint8_t i = 0 ;
	
	/*周边红外*/
	for(i=0;i<=6;i++)
	{
		if(adcRealTime[i] >= IR_OBSTACLE_0_6)
		{
			ret[i] = 1;
		}
		else
		{
			ret[i] = 0;
		}
	}
	
	/*二郎神*/
	if(adcRealTime[7] >= IR_OBSTACLE_7)
		ret[7] = 1;
	else
		ret[7] = 0;
	
	/*沿边 左*/
	if(adcRealTime[8] >= IR_OBSTACLE_8)
		ret[8] = 1;
	else
		ret[8] = 0;
	
	/*沿边 右*/
	if(adcRealTime[9] >= IR_OBSTACLE_9)
		ret[9] = 1;
	else
		ret[9] = 0;
	
}

#define PAUSE_V      30 /*暂停的阈值*/

void bsp_DetectDeal(void)
{
	uint8_t i = 0 ;
	static bool isObstacle = false;
	static uint32_t noObstacleTickCnt = 0 ;
	
	UNUSED(i);
	UNUSED(isObstacle);
	UNUSED(noObstacleTickCnt);
	
	
#if 0	
	for(i=0;i<10;i++)
	{
		printf("[%d]:%4d",i,(uint32_t)adcRealTime[i]);
	}
	printf("\r\n");
#endif
	
#if 0	
	if( adcRealTime[0] >= PAUSE_V || 
		adcRealTime[1] >= PAUSE_V || 
		adcRealTime[2] >= PAUSE_V || 
		adcRealTime[3] >= PAUSE_V || 
		adcRealTime[4] >= PAUSE_V || 
		adcRealTime[5] >= PAUSE_V || 
		adcRealTime[6] >= PAUSE_V)
	{
		bsp_SetMotorSpeed(MotorLeft,bsp_MotorSpeedMM2Pulse(0));
		bsp_SetMotorSpeed(MotorRight,bsp_MotorSpeedMM2Pulse(0));
	}
	else
	{
		bsp_SetMotorSpeed(MotorLeft,bsp_MotorSpeedMM2Pulse(250));
		bsp_SetMotorSpeed(MotorRight,bsp_MotorSpeedMM2Pulse(250));
	}
#endif	
	
	


#if 0	
	/*检测障碍物之前，先认为无障碍物*/
	isObstacle = false;
	/*只要一个管子认为有障碍物就是有障碍物*/
	for(i=0;i<=7;i++)
	{
		if(adcRealTime[i] >=1.0F)
		{
			/*前进 且 有障碍物的同时才减速，减速是有时间限制的，开启定时器回调函数，时间到了就恢复正常速度*/
			if(bsp_MotorGetTargetSpeed(MotorLeft)>=0 && bsp_MotorGetTargetSpeed(MotorRight)>=0)
			{
				bsp_SetMotorSpeed(MotorLeft,3);
				bsp_SetMotorSpeed(MotorRight,3);
				
				/*有障碍物*/
				isObstacle = true;
				noObstacleTickCnt = 0 ;
			}
		}
	}
	
	/*无障碍物计时，标志必须是前进*/
	if(isObstacle == false && bsp_MotorGetTargetSpeed(MotorLeft)>=0 && bsp_MotorGetTargetSpeed(MotorRight)>=0)
	{
		if(++noObstacleTickCnt >= 1500)
		{
			bsp_SetMotorSpeed(MotorLeft,6);
			bsp_SetMotorSpeed(MotorRight,6);
		}
	}
	
#endif	
}


/*
*********************************************************************************************************
*	函 数 名: bsp_DetectMeasureTest
*	功能说明: 检测到障碍物就停下来，直到障碍物离开
*	形    参: 无
*	返 回 值: 无
*********************************************************************************************************
*/
void bsp_DetectMeasureTest(void)
{
	if(bsp_GetInfraRedAdcVoltage(IR0)>=1.0F ||
	bsp_GetInfraRedAdcVoltage(IR1)>=1.0F ||
	bsp_GetInfraRedAdcVoltage(IR2)>=1.0F ||
	bsp_GetInfraRedAdcVoltage(IR3)>=1.0F ||
	bsp_GetInfraRedAdcVoltage(IR4)>=1.0F ||
	bsp_GetInfraRedAdcVoltage(IR5)>=1.0F ||
	bsp_GetInfraRedAdcVoltage(IR6)>=1.0F ||
	bsp_GetInfraRedAdcVoltage(IR7)>=1.0F)
	{
		bsp_SetMotorSpeed(MotorLeft, 0);
		bsp_SetMotorSpeed(MotorRight,0);
	}
	else
	{
		bsp_SetMotorSpeed(MotorLeft, 12);
		bsp_SetMotorSpeed(MotorRight,12);
	}
}


