#include "bsp.h"



#define PIN_MAP_MAX		10  //引脚对数
#define SAMP_COUNT      10  //采集AD值的个数
#define IntervalTime	4  //没对管子扫描间隔时间
#define ChargeTime	    3   //没对管子扫描间隔时间
#define TimeAfterOpen   120 //开发射后延时
#define TimeAfterClose  20  //关发射，延时读，判断太阳光
#define Sunlight        1.0F//如果关闭发射管也读取到了太阳光的阈值，则认为是太阳光的影响

AW_PIN PinMap[PIN_MAP_MAX][2]=
{
	{awP1_5,awP0_7},   //1
	{awP1_5,awP0_5},   //2
	{awP1_7,awP0_4},   //3
	{awP1_7,awP0_3},   //4
	{awP1_0,awP0_2},   //5
	{awP1_1,awP0_1},   //6
	{awP1_2,awP0_0},   //7
	{awP1_6,awP0_6},   //8
	{awP1_4,awP1_4},   //Left
	{awP1_3,awP1_3},   //Right
};


static DetectAct detectAct;
static float adcContrast[PIN_MAP_MAX][2]; //开发射前后的电压值
static float adcRealTime[PIN_MAP_MAX];    //对比开发射前后，权衡太阳光之后的电压值
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
	static uint8_t action = 0 ;
	static uint32_t delay = 0 ;
	
	switch(action)
	{
		case 0:
		{
			bsp_AWSetPinVal(PinMap[pinMapIndex%PIN_MAP_MAX][1], AW_0); //先开接收，充电
			delay = xTaskGetTickCount();
			action++;
			
		}break;
		
		case 1:
		{
			if(xTaskGetTickCount() - delay >= ChargeTime)
			{
				bsp_AWSetPinVal(PinMap[pinMapIndex%PIN_MAP_MAX][0], AW_0); //再开发送，读AD
				bsp_DelayUS(TimeAfterOpen);
				bsp_GetAdScanValue();
				bsp_AWSetPinVal(PinMap[pinMapIndex%PIN_MAP_MAX][0], AW_1);//关发射
				bsp_DelayUS(TimeAfterClose);
				bsp_GetAdScanValue();
				bsp_AWSetPinVal(PinMap[pinMapIndex%PIN_MAP_MAX][1], AW_1);//关接收
				//pinMapIndex++;
				
				delay = xTaskGetTickCount();
				action++;
			}
		}break;
		
		case 2:
		{
			if(xTaskGetTickCount() - delay >= (IntervalTime-ChargeTime)) 
			{
				action = 0 ;
			}
		}break;
	}
	
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
		//----------管子对--1-----------------------------------------------------------------------------
		case 0:
		{
			bsp_AWSetPinVal(PinMap[detectAct.pinMapIndex%PIN_MAP_MAX][1], AW_0); //先开接收，充电
			detectAct.delay = xTaskGetTickCount();
			detectAct.action++;
			
		}break;
		
		case 1:
		{
			if(xTaskGetTickCount() - detectAct.delay >= ChargeTime)
			{
				bsp_AWSetPinVal(PinMap[detectAct.pinMapIndex%PIN_MAP_MAX][0], AW_0); //再开发送，读AD
				bsp_DelayUS(TimeAfterOpen);
				adcContrast[detectAct.pinMapIndex%PIN_MAP_MAX][0] = bsp_GetAdScanValue();//开灯读
				bsp_AWSetPinVal(PinMap[detectAct.pinMapIndex%PIN_MAP_MAX][0], AW_1);//关发射
				bsp_DelayUS(TimeAfterClose);
				adcContrast[detectAct.pinMapIndex%PIN_MAP_MAX][1] = bsp_GetAdScanValue();//关灯读
				bsp_AWSetPinVal(PinMap[detectAct.pinMapIndex%PIN_MAP_MAX][1], AW_1);//关接收
				
				adcRealTime[detectAct.pinMapIndex%PIN_MAP_MAX] = adcContrast[detectAct.action/2][1] >= Sunlight ? 0 : adcContrast[detectAct.action/2][0];//如果读到太阳光了就当没有检测到障碍物
				adcIsSunlight[detectAct.pinMapIndex%PIN_MAP_MAX] = adcContrast[detectAct.action/2][1] >= Sunlight ? 1 : 0;//记录是否被当阳光挡住了
				detectAct.pinMapIndex++;
				
				detectAct.delay = xTaskGetTickCount();
				detectAct.action++;
			}
		}break;
		
		//----------管子对--2-----------------------------------------------------------------------------
		case 2:
		{
			if(xTaskGetTickCount() - detectAct.delay >= (IntervalTime-ChargeTime)) 
			{
				bsp_AWSetPinVal(PinMap[detectAct.pinMapIndex%PIN_MAP_MAX][1], AW_0); //先开接收，充电
				detectAct.delay = xTaskGetTickCount();
				detectAct.action++;
			}
		}break;
		
		case 3:
		{
			if(xTaskGetTickCount() - detectAct.delay >= ChargeTime)
			{
				bsp_AWSetPinVal(PinMap[detectAct.pinMapIndex%PIN_MAP_MAX][0], AW_0); //再开发送，读AD
				bsp_DelayUS(TimeAfterOpen);
				adcContrast[detectAct.pinMapIndex%PIN_MAP_MAX][0] = bsp_GetAdScanValue();//开灯读
				bsp_AWSetPinVal(PinMap[detectAct.pinMapIndex%PIN_MAP_MAX][0], AW_1);//关发射
				bsp_DelayUS(TimeAfterClose);
				adcContrast[detectAct.pinMapIndex%PIN_MAP_MAX][1] = bsp_GetAdScanValue();//关灯读
				bsp_AWSetPinVal(PinMap[detectAct.pinMapIndex%PIN_MAP_MAX][1], AW_1);//关接收
				
				adcRealTime[detectAct.pinMapIndex%PIN_MAP_MAX] = adcContrast[detectAct.action/2][1] >= Sunlight ? 0 : adcContrast[detectAct.action/2][0];//如果读到太阳光了就当没有检测到障碍物
				adcIsSunlight[detectAct.pinMapIndex%PIN_MAP_MAX] = adcContrast[detectAct.action/2][1] >= Sunlight ? 1 : 0;//记录是否被当阳光挡住了
				detectAct.pinMapIndex++;
				
				detectAct.delay = xTaskGetTickCount();
				detectAct.action++;
			}
		}break;
		
		//----------管子对--3-----------------------------------------------------------------------------
		case 4:
		{
			if(xTaskGetTickCount() - detectAct.delay >= (IntervalTime-ChargeTime)) 
			{
				bsp_AWSetPinVal(PinMap[detectAct.pinMapIndex%PIN_MAP_MAX][1], AW_0); //先开接收，充电
				detectAct.delay = xTaskGetTickCount();
				detectAct.action++;
			}
		}break;
		
		case 5:
		{
			if(xTaskGetTickCount() - detectAct.delay >= ChargeTime)
			{
				bsp_AWSetPinVal(PinMap[detectAct.pinMapIndex%PIN_MAP_MAX][0], AW_0); //再开发送，读AD
				bsp_DelayUS(TimeAfterOpen);
				adcContrast[detectAct.pinMapIndex%PIN_MAP_MAX][0] = bsp_GetAdScanValue();//开灯读
				bsp_AWSetPinVal(PinMap[detectAct.pinMapIndex%PIN_MAP_MAX][0], AW_1);//关发射
				bsp_DelayUS(TimeAfterClose);
				adcContrast[detectAct.pinMapIndex%PIN_MAP_MAX][1] = bsp_GetAdScanValue();//关灯读
				bsp_AWSetPinVal(PinMap[detectAct.pinMapIndex%PIN_MAP_MAX][1], AW_1);//关接收
				
				adcRealTime[detectAct.pinMapIndex%PIN_MAP_MAX] = adcContrast[detectAct.action/2][1] >= Sunlight ? 0 : adcContrast[detectAct.action/2][0];//如果读到太阳光了就当没有检测到障碍物
				adcIsSunlight[detectAct.pinMapIndex%PIN_MAP_MAX] = adcContrast[detectAct.action/2][1] >= Sunlight ? 1 : 0;//记录是否被当阳光挡住了
				detectAct.pinMapIndex++;
				
				detectAct.delay = xTaskGetTickCount();
				detectAct.action++;
			}
		}break;
		
		//----------管子对--4-----------------------------------------------------------------------------
		case 6:
		{
			if(xTaskGetTickCount() - detectAct.delay >= (IntervalTime-ChargeTime)) 
			{
				bsp_AWSetPinVal(PinMap[detectAct.pinMapIndex%PIN_MAP_MAX][1], AW_0); //先开接收，充电
				detectAct.delay = xTaskGetTickCount();
				detectAct.action++;
			}
		}break;
		
		case 7:
		{
			if(xTaskGetTickCount() - detectAct.delay >= ChargeTime)
			{
				bsp_AWSetPinVal(PinMap[detectAct.pinMapIndex%PIN_MAP_MAX][0], AW_0); //再开发送，读AD
				bsp_DelayUS(TimeAfterOpen);
				adcContrast[detectAct.pinMapIndex%PIN_MAP_MAX][0] = bsp_GetAdScanValue();//开灯读
				bsp_AWSetPinVal(PinMap[detectAct.pinMapIndex%PIN_MAP_MAX][0], AW_1);//关发射
				bsp_DelayUS(TimeAfterClose);
				adcContrast[detectAct.pinMapIndex%PIN_MAP_MAX][1] = bsp_GetAdScanValue();//关灯读
				bsp_AWSetPinVal(PinMap[detectAct.pinMapIndex%PIN_MAP_MAX][1], AW_1);//关接收
				
				adcRealTime[detectAct.pinMapIndex%PIN_MAP_MAX] = adcContrast[detectAct.action/2][1] >= Sunlight ? 0 : adcContrast[detectAct.action/2][0];//如果读到太阳光了就当没有检测到障碍物
				adcIsSunlight[detectAct.pinMapIndex%PIN_MAP_MAX] = adcContrast[detectAct.action/2][1] >= Sunlight ? 1 : 0;//记录是否被当阳光挡住了
				detectAct.pinMapIndex++;
				
				detectAct.delay = xTaskGetTickCount();
				detectAct.action++;
			}
		}break;
		
		//----------管子对--5-----------------------------------------------------------------------------
		case 8:
		{
			if(xTaskGetTickCount() - detectAct.delay >= (IntervalTime-ChargeTime)) 
			{
				bsp_AWSetPinVal(PinMap[detectAct.pinMapIndex%PIN_MAP_MAX][1], AW_0); //先开接收，充电
				detectAct.delay = xTaskGetTickCount();
				detectAct.action++;
			}
		}break;
		
		case 9:
		{
			if(xTaskGetTickCount() - detectAct.delay >= ChargeTime)
			{
				bsp_AWSetPinVal(PinMap[detectAct.pinMapIndex%PIN_MAP_MAX][0], AW_0); //再开发送，读AD
				bsp_DelayUS(TimeAfterOpen);
				adcContrast[detectAct.pinMapIndex%PIN_MAP_MAX][0] = bsp_GetAdScanValue();//开灯读
				bsp_AWSetPinVal(PinMap[detectAct.pinMapIndex%PIN_MAP_MAX][0], AW_1);//关发射
				bsp_DelayUS(TimeAfterClose);
				adcContrast[detectAct.pinMapIndex%PIN_MAP_MAX][1] = bsp_GetAdScanValue();//关灯读
				bsp_AWSetPinVal(PinMap[detectAct.pinMapIndex%PIN_MAP_MAX][1], AW_1);//关接收
				
				adcRealTime[detectAct.pinMapIndex%PIN_MAP_MAX] = adcContrast[detectAct.action/2][1] >= Sunlight ? 0 : adcContrast[detectAct.action/2][0];//如果读到太阳光了就当没有检测到障碍物
				adcIsSunlight[detectAct.pinMapIndex%PIN_MAP_MAX] = adcContrast[detectAct.action/2][1] >= Sunlight ? 1 : 0;//记录是否被当阳光挡住了
				detectAct.pinMapIndex++;
				
				detectAct.delay = xTaskGetTickCount();
				detectAct.action++;
			}
		}break;
		
		//----------管子对--6-----------------------------------------------------------------------------
		case 10:
		{
			if(xTaskGetTickCount() - detectAct.delay >= (IntervalTime-ChargeTime)) 
			{
				bsp_AWSetPinVal(PinMap[detectAct.pinMapIndex%PIN_MAP_MAX][1], AW_0); //先开接收，充电
				detectAct.delay = xTaskGetTickCount();
				detectAct.action++;
			}
		}break;
		
		case 11:
		{
			if(xTaskGetTickCount() - detectAct.delay >= ChargeTime)
			{
				bsp_AWSetPinVal(PinMap[detectAct.pinMapIndex%PIN_MAP_MAX][0], AW_0); //再开发送，读AD
				bsp_DelayUS(TimeAfterOpen);
				adcContrast[detectAct.pinMapIndex%PIN_MAP_MAX][0] = bsp_GetAdScanValue();//开灯读
				bsp_AWSetPinVal(PinMap[detectAct.pinMapIndex%PIN_MAP_MAX][0], AW_1);//关发射
				bsp_DelayUS(TimeAfterClose);
				adcContrast[detectAct.pinMapIndex%PIN_MAP_MAX][1] = bsp_GetAdScanValue();//关灯读
				bsp_AWSetPinVal(PinMap[detectAct.pinMapIndex%PIN_MAP_MAX][1], AW_1);//关接收
				
				adcRealTime[detectAct.pinMapIndex%PIN_MAP_MAX] = adcContrast[detectAct.action/2][1] >= Sunlight ? 0 : adcContrast[detectAct.action/2][0];//如果读到太阳光了就当没有检测到障碍物
				adcIsSunlight[detectAct.pinMapIndex%PIN_MAP_MAX] = adcContrast[detectAct.action/2][1] >= Sunlight ? 1 : 0;//记录是否被当阳光挡住了
				detectAct.pinMapIndex++;
				
				detectAct.delay = xTaskGetTickCount();
				detectAct.action++;
			}
		}break;
		
		//----------管子对--7-----------------------------------------------------------------------------
		case 12:
		{
			if(xTaskGetTickCount() - detectAct.delay >= (IntervalTime-ChargeTime)) 
			{
				bsp_AWSetPinVal(PinMap[detectAct.pinMapIndex%PIN_MAP_MAX][1], AW_0); //先开接收，充电
				detectAct.delay = xTaskGetTickCount();
				detectAct.action++;
			}
		}break;
		
		case 13:
		{
			if(xTaskGetTickCount() - detectAct.delay >= ChargeTime)
			{
				bsp_AWSetPinVal(PinMap[detectAct.pinMapIndex%PIN_MAP_MAX][0], AW_0); //再开发送，读AD
				bsp_DelayUS(TimeAfterOpen);
				adcContrast[detectAct.pinMapIndex%PIN_MAP_MAX][0] = bsp_GetAdScanValue();//开灯读
				bsp_AWSetPinVal(PinMap[detectAct.pinMapIndex%PIN_MAP_MAX][0], AW_1);//关发射
				bsp_DelayUS(TimeAfterClose);
				adcContrast[detectAct.pinMapIndex%PIN_MAP_MAX][1] = bsp_GetAdScanValue();//关灯读
				bsp_AWSetPinVal(PinMap[detectAct.pinMapIndex%PIN_MAP_MAX][1], AW_1);//关接收
				
				adcRealTime[detectAct.pinMapIndex%PIN_MAP_MAX] = adcContrast[detectAct.action/2][1] >= Sunlight ? 0 : adcContrast[detectAct.action/2][0];//如果读到太阳光了就当没有检测到障碍物
				adcIsSunlight[detectAct.pinMapIndex%PIN_MAP_MAX] = adcContrast[detectAct.action/2][1] >= Sunlight ? 1 : 0;//记录是否被当阳光挡住了
				detectAct.pinMapIndex++;
				
				detectAct.delay = xTaskGetTickCount();
				detectAct.action++;
			}
		}break;
		
		//----------管子对--8-----------------------------------------------------------------------------
		case 14:
		{
			if(xTaskGetTickCount() - detectAct.delay >= (IntervalTime-ChargeTime)) 
			{
				bsp_AWSetPinVal(PinMap[detectAct.pinMapIndex%PIN_MAP_MAX][1], AW_0); //先开接收，充电
				detectAct.delay = xTaskGetTickCount();
				detectAct.action++;
			}
		}break;
		
		case 15:
		{
			if(xTaskGetTickCount() - detectAct.delay >= ChargeTime)
			{
				bsp_AWSetPinVal(PinMap[detectAct.pinMapIndex%PIN_MAP_MAX][0], AW_0); //再开发送，读AD
				bsp_DelayUS(TimeAfterOpen);
				adcContrast[detectAct.pinMapIndex%PIN_MAP_MAX][0] = bsp_GetAdScanValue();//开灯读
				bsp_AWSetPinVal(PinMap[detectAct.pinMapIndex%PIN_MAP_MAX][0], AW_1);//关发射
				bsp_DelayUS(TimeAfterClose);
				adcContrast[detectAct.pinMapIndex%PIN_MAP_MAX][1] = bsp_GetAdScanValue();//关灯读
				bsp_AWSetPinVal(PinMap[detectAct.pinMapIndex%PIN_MAP_MAX][1], AW_1);//关接收
				
				adcRealTime[detectAct.pinMapIndex%PIN_MAP_MAX] = adcContrast[detectAct.action/2][1] >= Sunlight ? 0 : adcContrast[detectAct.action/2][0];//如果读到太阳光了就当没有检测到障碍物
				adcIsSunlight[detectAct.pinMapIndex%PIN_MAP_MAX] = adcContrast[detectAct.action/2][1] >= Sunlight ? 1 : 0;//记录是否被当阳光挡住了
				detectAct.pinMapIndex++;
				
				detectAct.delay = xTaskGetTickCount();
				detectAct.action++;
			}
		}break;
		
		//----------管子对--9-----------------------------------------------------------------------------
		case 16:
		{
			if(xTaskGetTickCount() - detectAct.delay >= (IntervalTime-ChargeTime)) 
			{
				bsp_AWSetPinVal(PinMap[detectAct.pinMapIndex%PIN_MAP_MAX][1], AW_0); //先开接收，充电
				detectAct.delay = xTaskGetTickCount();
				detectAct.action++;
			}
		}break;
		
		case 17:
		{
			if(xTaskGetTickCount() - detectAct.delay >= ChargeTime)
			{
				//bsp_AWSetPinVal(PinMap[detectAct.pinMapIndex%PIN_MAP_MAX][0], AW_0); //再开发送，读AD
				bsp_DelayUS(TimeAfterOpen);
				adcContrast[detectAct.pinMapIndex%PIN_MAP_MAX][0] = bsp_GetAdScanValue();//开灯读
				bsp_AWSetPinVal(PinMap[detectAct.pinMapIndex%PIN_MAP_MAX][0], AW_1);//关发射
				bsp_DelayUS(TimeAfterClose);
				adcContrast[detectAct.pinMapIndex%PIN_MAP_MAX][1] = bsp_GetAdScanValue();//关灯读
				bsp_AWSetPinVal(PinMap[detectAct.pinMapIndex%PIN_MAP_MAX][1], AW_1);//关接收
				
				adcRealTime[detectAct.pinMapIndex%PIN_MAP_MAX] = adcContrast[detectAct.action/2][1] >= Sunlight ? 0 : adcContrast[detectAct.action/2][0];//如果读到太阳光了就当没有检测到障碍物
				adcIsSunlight[detectAct.pinMapIndex%PIN_MAP_MAX] = adcContrast[detectAct.action/2][1] >= Sunlight ? 1 : 0;//记录是否被当阳光挡住了
				detectAct.pinMapIndex++;
				
				detectAct.delay = xTaskGetTickCount();
				detectAct.action++;
			}
		}break;
		

		//----------管子对--10-----------------------------------------------------------------------------
		case 18:
		{
			if(xTaskGetTickCount() - detectAct.delay >= (IntervalTime-ChargeTime)) 
			{
				bsp_AWSetPinVal(PinMap[detectAct.pinMapIndex%PIN_MAP_MAX][1], AW_0); //先开接收，充电
				detectAct.delay = xTaskGetTickCount();
				detectAct.action++;
			}
		}break;
		
		case 19:
		{
			if(xTaskGetTickCount() - detectAct.delay >= ChargeTime)
			{
				//bsp_AWSetPinVal(PinMap[detectAct.pinMapIndex%PIN_MAP_MAX][0], AW_0); //再开发送，读AD
				bsp_DelayUS(TimeAfterOpen);
				adcContrast[detectAct.pinMapIndex%PIN_MAP_MAX][0] = bsp_GetAdScanValue();//开灯读
				bsp_AWSetPinVal(PinMap[detectAct.pinMapIndex%PIN_MAP_MAX][0], AW_1);//关发射
				bsp_DelayUS(TimeAfterClose);
				adcContrast[detectAct.pinMapIndex%PIN_MAP_MAX][1] = bsp_GetAdScanValue();//关灯读
				bsp_AWSetPinVal(PinMap[detectAct.pinMapIndex%PIN_MAP_MAX][1], AW_1);//关接收
				
				adcRealTime[detectAct.pinMapIndex%PIN_MAP_MAX] = adcContrast[detectAct.action/2][1] >= Sunlight ? 0 : adcContrast[detectAct.action/2][0];//如果读到太阳光了就当没有检测到障碍物
				adcIsSunlight[detectAct.pinMapIndex%PIN_MAP_MAX] = adcContrast[detectAct.action/2][1] >= Sunlight ? 1 : 0;//记录是否被当阳光挡住了
				detectAct.pinMapIndex++;
				
				detectAct.delay = xTaskGetTickCount();
				detectAct.action++;
			}
		}break;
		
		case 20:
		{
			if(xTaskGetTickCount() - detectAct.delay >= (IntervalTime-ChargeTime)) 
			{
				detectAct.action = 0 ;
				detectAct.pinMapIndex = 0 ;
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



uint8_t flgdec = 0 ;

void bsp_DetectDeal(void)
{
	uint8_t i = 0 ;

	
	UNUSED(i);


	
	//如果是太阳光射到了，开启全闪烁灯
	for(i=0;i<=7;i++)
	{
		if(adcRealTime[i] >=1.0F)
		{
			if(flgdec == 0)
			{
				bsp_SetMotorPWM(MotorLeft,Forward, 6000);
				bsp_SetMotorPWM(MotorRight,Forward,6000);
				flgdec = 1 ;
			}
			
		}
	}
	
}



