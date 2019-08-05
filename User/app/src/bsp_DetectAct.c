#include "bsp.h"



#define PIN_MAP_MAX		10  //���Ŷ���
#define SAMP_COUNT      10  //�ɼ�ADֵ�ĸ���
#define IntervalTime	4  //û�Թ���ɨ����ʱ��
#define ChargeTime	    3   //û�Թ���ɨ����ʱ��
#define TimeAfterOpen   120 //���������ʱ
#define TimeAfterClose  20  //�ط��䣬��ʱ�����ж�̫����
#define Sunlight        1.0F//����رշ����Ҳ��ȡ����̫�������ֵ������Ϊ��̫�����Ӱ��

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
static float adcContrast[PIN_MAP_MAX][2]; //������ǰ��ĵ�ѹֵ
static float adcRealTime[PIN_MAP_MAX];    //�Աȿ�����ǰ��Ȩ��̫����֮��ĵ�ѹֵ
static uint8_t adcIsSunlight[PIN_MAP_MAX];//�Ƿ���̫����

static void bsp_ADCConfig(void);


/*
*********************************************************************************************************
*	�� �� ��: bsp_InitDetectAct
*	����˵��: ������ײ���ɨ��״̬����
*	��    ��: ��
*	�� �� ֵ: ��
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
*	�� �� ��: bsp_DetectStart
*	����˵��: ������ײ���ɨ��״̬����
*	��    ��: ��
*	�� �� ֵ: ��
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
*	�� �� ��: bsp_DetectStart
*	����˵��: �ر���ײ���ɨ��״̬����
*	��    ��: ��
*	�� �� ֵ: ��
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
			bsp_AWSetPinVal(PinMap[pinMapIndex%PIN_MAP_MAX][1], AW_0); //�ȿ����գ����
			delay = xTaskGetTickCount();
			action++;
			
		}break;
		
		case 1:
		{
			if(xTaskGetTickCount() - delay >= ChargeTime)
			{
				bsp_AWSetPinVal(PinMap[pinMapIndex%PIN_MAP_MAX][0], AW_0); //�ٿ����ͣ���AD
				bsp_DelayUS(TimeAfterOpen);
				bsp_GetAdScanValue();
				bsp_AWSetPinVal(PinMap[pinMapIndex%PIN_MAP_MAX][0], AW_1);//�ط���
				bsp_DelayUS(TimeAfterClose);
				bsp_GetAdScanValue();
				bsp_AWSetPinVal(PinMap[pinMapIndex%PIN_MAP_MAX][1], AW_1);//�ؽ���
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
*	�� �� ��: bsp_DetectAct
*	����˵��: ��˳��ɨ�衣
*	��    ��: ��
*	�� �� ֵ: ��
*********************************************************************************************************
*/
void bsp_DetectAct(void)
{
	if( !detectAct.isRunning )
		return ;
	
	switch(detectAct.action)
	{
		//----------���Ӷ�--1-----------------------------------------------------------------------------
		case 0:
		{
			bsp_AWSetPinVal(PinMap[detectAct.pinMapIndex%PIN_MAP_MAX][1], AW_0); //�ȿ����գ����
			detectAct.delay = xTaskGetTickCount();
			detectAct.action++;
			
		}break;
		
		case 1:
		{
			if(xTaskGetTickCount() - detectAct.delay >= ChargeTime)
			{
				bsp_AWSetPinVal(PinMap[detectAct.pinMapIndex%PIN_MAP_MAX][0], AW_0); //�ٿ����ͣ���AD
				bsp_DelayUS(TimeAfterOpen);
				adcContrast[detectAct.pinMapIndex%PIN_MAP_MAX][0] = bsp_GetAdScanValue();//���ƶ�
				bsp_AWSetPinVal(PinMap[detectAct.pinMapIndex%PIN_MAP_MAX][0], AW_1);//�ط���
				bsp_DelayUS(TimeAfterClose);
				adcContrast[detectAct.pinMapIndex%PIN_MAP_MAX][1] = bsp_GetAdScanValue();//�صƶ�
				bsp_AWSetPinVal(PinMap[detectAct.pinMapIndex%PIN_MAP_MAX][1], AW_1);//�ؽ���
				
				adcRealTime[detectAct.pinMapIndex%PIN_MAP_MAX] = adcContrast[detectAct.action/2][1] >= Sunlight ? 0 : adcContrast[detectAct.action/2][0];//�������̫�����˾͵�û�м�⵽�ϰ���
				adcIsSunlight[detectAct.pinMapIndex%PIN_MAP_MAX] = adcContrast[detectAct.action/2][1] >= Sunlight ? 1 : 0;//��¼�Ƿ񱻵����⵲ס��
				detectAct.pinMapIndex++;
				
				detectAct.delay = xTaskGetTickCount();
				detectAct.action++;
			}
		}break;
		
		//----------���Ӷ�--2-----------------------------------------------------------------------------
		case 2:
		{
			if(xTaskGetTickCount() - detectAct.delay >= (IntervalTime-ChargeTime)) 
			{
				bsp_AWSetPinVal(PinMap[detectAct.pinMapIndex%PIN_MAP_MAX][1], AW_0); //�ȿ����գ����
				detectAct.delay = xTaskGetTickCount();
				detectAct.action++;
			}
		}break;
		
		case 3:
		{
			if(xTaskGetTickCount() - detectAct.delay >= ChargeTime)
			{
				bsp_AWSetPinVal(PinMap[detectAct.pinMapIndex%PIN_MAP_MAX][0], AW_0); //�ٿ����ͣ���AD
				bsp_DelayUS(TimeAfterOpen);
				adcContrast[detectAct.pinMapIndex%PIN_MAP_MAX][0] = bsp_GetAdScanValue();//���ƶ�
				bsp_AWSetPinVal(PinMap[detectAct.pinMapIndex%PIN_MAP_MAX][0], AW_1);//�ط���
				bsp_DelayUS(TimeAfterClose);
				adcContrast[detectAct.pinMapIndex%PIN_MAP_MAX][1] = bsp_GetAdScanValue();//�صƶ�
				bsp_AWSetPinVal(PinMap[detectAct.pinMapIndex%PIN_MAP_MAX][1], AW_1);//�ؽ���
				
				adcRealTime[detectAct.pinMapIndex%PIN_MAP_MAX] = adcContrast[detectAct.action/2][1] >= Sunlight ? 0 : adcContrast[detectAct.action/2][0];//�������̫�����˾͵�û�м�⵽�ϰ���
				adcIsSunlight[detectAct.pinMapIndex%PIN_MAP_MAX] = adcContrast[detectAct.action/2][1] >= Sunlight ? 1 : 0;//��¼�Ƿ񱻵����⵲ס��
				detectAct.pinMapIndex++;
				
				detectAct.delay = xTaskGetTickCount();
				detectAct.action++;
			}
		}break;
		
		//----------���Ӷ�--3-----------------------------------------------------------------------------
		case 4:
		{
			if(xTaskGetTickCount() - detectAct.delay >= (IntervalTime-ChargeTime)) 
			{
				bsp_AWSetPinVal(PinMap[detectAct.pinMapIndex%PIN_MAP_MAX][1], AW_0); //�ȿ����գ����
				detectAct.delay = xTaskGetTickCount();
				detectAct.action++;
			}
		}break;
		
		case 5:
		{
			if(xTaskGetTickCount() - detectAct.delay >= ChargeTime)
			{
				bsp_AWSetPinVal(PinMap[detectAct.pinMapIndex%PIN_MAP_MAX][0], AW_0); //�ٿ����ͣ���AD
				bsp_DelayUS(TimeAfterOpen);
				adcContrast[detectAct.pinMapIndex%PIN_MAP_MAX][0] = bsp_GetAdScanValue();//���ƶ�
				bsp_AWSetPinVal(PinMap[detectAct.pinMapIndex%PIN_MAP_MAX][0], AW_1);//�ط���
				bsp_DelayUS(TimeAfterClose);
				adcContrast[detectAct.pinMapIndex%PIN_MAP_MAX][1] = bsp_GetAdScanValue();//�صƶ�
				bsp_AWSetPinVal(PinMap[detectAct.pinMapIndex%PIN_MAP_MAX][1], AW_1);//�ؽ���
				
				adcRealTime[detectAct.pinMapIndex%PIN_MAP_MAX] = adcContrast[detectAct.action/2][1] >= Sunlight ? 0 : adcContrast[detectAct.action/2][0];//�������̫�����˾͵�û�м�⵽�ϰ���
				adcIsSunlight[detectAct.pinMapIndex%PIN_MAP_MAX] = adcContrast[detectAct.action/2][1] >= Sunlight ? 1 : 0;//��¼�Ƿ񱻵����⵲ס��
				detectAct.pinMapIndex++;
				
				detectAct.delay = xTaskGetTickCount();
				detectAct.action++;
			}
		}break;
		
		//----------���Ӷ�--4-----------------------------------------------------------------------------
		case 6:
		{
			if(xTaskGetTickCount() - detectAct.delay >= (IntervalTime-ChargeTime)) 
			{
				bsp_AWSetPinVal(PinMap[detectAct.pinMapIndex%PIN_MAP_MAX][1], AW_0); //�ȿ����գ����
				detectAct.delay = xTaskGetTickCount();
				detectAct.action++;
			}
		}break;
		
		case 7:
		{
			if(xTaskGetTickCount() - detectAct.delay >= ChargeTime)
			{
				bsp_AWSetPinVal(PinMap[detectAct.pinMapIndex%PIN_MAP_MAX][0], AW_0); //�ٿ����ͣ���AD
				bsp_DelayUS(TimeAfterOpen);
				adcContrast[detectAct.pinMapIndex%PIN_MAP_MAX][0] = bsp_GetAdScanValue();//���ƶ�
				bsp_AWSetPinVal(PinMap[detectAct.pinMapIndex%PIN_MAP_MAX][0], AW_1);//�ط���
				bsp_DelayUS(TimeAfterClose);
				adcContrast[detectAct.pinMapIndex%PIN_MAP_MAX][1] = bsp_GetAdScanValue();//�صƶ�
				bsp_AWSetPinVal(PinMap[detectAct.pinMapIndex%PIN_MAP_MAX][1], AW_1);//�ؽ���
				
				adcRealTime[detectAct.pinMapIndex%PIN_MAP_MAX] = adcContrast[detectAct.action/2][1] >= Sunlight ? 0 : adcContrast[detectAct.action/2][0];//�������̫�����˾͵�û�м�⵽�ϰ���
				adcIsSunlight[detectAct.pinMapIndex%PIN_MAP_MAX] = adcContrast[detectAct.action/2][1] >= Sunlight ? 1 : 0;//��¼�Ƿ񱻵����⵲ס��
				detectAct.pinMapIndex++;
				
				detectAct.delay = xTaskGetTickCount();
				detectAct.action++;
			}
		}break;
		
		//----------���Ӷ�--5-----------------------------------------------------------------------------
		case 8:
		{
			if(xTaskGetTickCount() - detectAct.delay >= (IntervalTime-ChargeTime)) 
			{
				bsp_AWSetPinVal(PinMap[detectAct.pinMapIndex%PIN_MAP_MAX][1], AW_0); //�ȿ����գ����
				detectAct.delay = xTaskGetTickCount();
				detectAct.action++;
			}
		}break;
		
		case 9:
		{
			if(xTaskGetTickCount() - detectAct.delay >= ChargeTime)
			{
				bsp_AWSetPinVal(PinMap[detectAct.pinMapIndex%PIN_MAP_MAX][0], AW_0); //�ٿ����ͣ���AD
				bsp_DelayUS(TimeAfterOpen);
				adcContrast[detectAct.pinMapIndex%PIN_MAP_MAX][0] = bsp_GetAdScanValue();//���ƶ�
				bsp_AWSetPinVal(PinMap[detectAct.pinMapIndex%PIN_MAP_MAX][0], AW_1);//�ط���
				bsp_DelayUS(TimeAfterClose);
				adcContrast[detectAct.pinMapIndex%PIN_MAP_MAX][1] = bsp_GetAdScanValue();//�صƶ�
				bsp_AWSetPinVal(PinMap[detectAct.pinMapIndex%PIN_MAP_MAX][1], AW_1);//�ؽ���
				
				adcRealTime[detectAct.pinMapIndex%PIN_MAP_MAX] = adcContrast[detectAct.action/2][1] >= Sunlight ? 0 : adcContrast[detectAct.action/2][0];//�������̫�����˾͵�û�м�⵽�ϰ���
				adcIsSunlight[detectAct.pinMapIndex%PIN_MAP_MAX] = adcContrast[detectAct.action/2][1] >= Sunlight ? 1 : 0;//��¼�Ƿ񱻵����⵲ס��
				detectAct.pinMapIndex++;
				
				detectAct.delay = xTaskGetTickCount();
				detectAct.action++;
			}
		}break;
		
		//----------���Ӷ�--6-----------------------------------------------------------------------------
		case 10:
		{
			if(xTaskGetTickCount() - detectAct.delay >= (IntervalTime-ChargeTime)) 
			{
				bsp_AWSetPinVal(PinMap[detectAct.pinMapIndex%PIN_MAP_MAX][1], AW_0); //�ȿ����գ����
				detectAct.delay = xTaskGetTickCount();
				detectAct.action++;
			}
		}break;
		
		case 11:
		{
			if(xTaskGetTickCount() - detectAct.delay >= ChargeTime)
			{
				bsp_AWSetPinVal(PinMap[detectAct.pinMapIndex%PIN_MAP_MAX][0], AW_0); //�ٿ����ͣ���AD
				bsp_DelayUS(TimeAfterOpen);
				adcContrast[detectAct.pinMapIndex%PIN_MAP_MAX][0] = bsp_GetAdScanValue();//���ƶ�
				bsp_AWSetPinVal(PinMap[detectAct.pinMapIndex%PIN_MAP_MAX][0], AW_1);//�ط���
				bsp_DelayUS(TimeAfterClose);
				adcContrast[detectAct.pinMapIndex%PIN_MAP_MAX][1] = bsp_GetAdScanValue();//�صƶ�
				bsp_AWSetPinVal(PinMap[detectAct.pinMapIndex%PIN_MAP_MAX][1], AW_1);//�ؽ���
				
				adcRealTime[detectAct.pinMapIndex%PIN_MAP_MAX] = adcContrast[detectAct.action/2][1] >= Sunlight ? 0 : adcContrast[detectAct.action/2][0];//�������̫�����˾͵�û�м�⵽�ϰ���
				adcIsSunlight[detectAct.pinMapIndex%PIN_MAP_MAX] = adcContrast[detectAct.action/2][1] >= Sunlight ? 1 : 0;//��¼�Ƿ񱻵����⵲ס��
				detectAct.pinMapIndex++;
				
				detectAct.delay = xTaskGetTickCount();
				detectAct.action++;
			}
		}break;
		
		//----------���Ӷ�--7-----------------------------------------------------------------------------
		case 12:
		{
			if(xTaskGetTickCount() - detectAct.delay >= (IntervalTime-ChargeTime)) 
			{
				bsp_AWSetPinVal(PinMap[detectAct.pinMapIndex%PIN_MAP_MAX][1], AW_0); //�ȿ����գ����
				detectAct.delay = xTaskGetTickCount();
				detectAct.action++;
			}
		}break;
		
		case 13:
		{
			if(xTaskGetTickCount() - detectAct.delay >= ChargeTime)
			{
				bsp_AWSetPinVal(PinMap[detectAct.pinMapIndex%PIN_MAP_MAX][0], AW_0); //�ٿ����ͣ���AD
				bsp_DelayUS(TimeAfterOpen);
				adcContrast[detectAct.pinMapIndex%PIN_MAP_MAX][0] = bsp_GetAdScanValue();//���ƶ�
				bsp_AWSetPinVal(PinMap[detectAct.pinMapIndex%PIN_MAP_MAX][0], AW_1);//�ط���
				bsp_DelayUS(TimeAfterClose);
				adcContrast[detectAct.pinMapIndex%PIN_MAP_MAX][1] = bsp_GetAdScanValue();//�صƶ�
				bsp_AWSetPinVal(PinMap[detectAct.pinMapIndex%PIN_MAP_MAX][1], AW_1);//�ؽ���
				
				adcRealTime[detectAct.pinMapIndex%PIN_MAP_MAX] = adcContrast[detectAct.action/2][1] >= Sunlight ? 0 : adcContrast[detectAct.action/2][0];//�������̫�����˾͵�û�м�⵽�ϰ���
				adcIsSunlight[detectAct.pinMapIndex%PIN_MAP_MAX] = adcContrast[detectAct.action/2][1] >= Sunlight ? 1 : 0;//��¼�Ƿ񱻵����⵲ס��
				detectAct.pinMapIndex++;
				
				detectAct.delay = xTaskGetTickCount();
				detectAct.action++;
			}
		}break;
		
		//----------���Ӷ�--8-----------------------------------------------------------------------------
		case 14:
		{
			if(xTaskGetTickCount() - detectAct.delay >= (IntervalTime-ChargeTime)) 
			{
				bsp_AWSetPinVal(PinMap[detectAct.pinMapIndex%PIN_MAP_MAX][1], AW_0); //�ȿ����գ����
				detectAct.delay = xTaskGetTickCount();
				detectAct.action++;
			}
		}break;
		
		case 15:
		{
			if(xTaskGetTickCount() - detectAct.delay >= ChargeTime)
			{
				bsp_AWSetPinVal(PinMap[detectAct.pinMapIndex%PIN_MAP_MAX][0], AW_0); //�ٿ����ͣ���AD
				bsp_DelayUS(TimeAfterOpen);
				adcContrast[detectAct.pinMapIndex%PIN_MAP_MAX][0] = bsp_GetAdScanValue();//���ƶ�
				bsp_AWSetPinVal(PinMap[detectAct.pinMapIndex%PIN_MAP_MAX][0], AW_1);//�ط���
				bsp_DelayUS(TimeAfterClose);
				adcContrast[detectAct.pinMapIndex%PIN_MAP_MAX][1] = bsp_GetAdScanValue();//�صƶ�
				bsp_AWSetPinVal(PinMap[detectAct.pinMapIndex%PIN_MAP_MAX][1], AW_1);//�ؽ���
				
				adcRealTime[detectAct.pinMapIndex%PIN_MAP_MAX] = adcContrast[detectAct.action/2][1] >= Sunlight ? 0 : adcContrast[detectAct.action/2][0];//�������̫�����˾͵�û�м�⵽�ϰ���
				adcIsSunlight[detectAct.pinMapIndex%PIN_MAP_MAX] = adcContrast[detectAct.action/2][1] >= Sunlight ? 1 : 0;//��¼�Ƿ񱻵����⵲ס��
				detectAct.pinMapIndex++;
				
				detectAct.delay = xTaskGetTickCount();
				detectAct.action++;
			}
		}break;
		
		//----------���Ӷ�--9-----------------------------------------------------------------------------
		case 16:
		{
			if(xTaskGetTickCount() - detectAct.delay >= (IntervalTime-ChargeTime)) 
			{
				bsp_AWSetPinVal(PinMap[detectAct.pinMapIndex%PIN_MAP_MAX][1], AW_0); //�ȿ����գ����
				detectAct.delay = xTaskGetTickCount();
				detectAct.action++;
			}
		}break;
		
		case 17:
		{
			if(xTaskGetTickCount() - detectAct.delay >= ChargeTime)
			{
				//bsp_AWSetPinVal(PinMap[detectAct.pinMapIndex%PIN_MAP_MAX][0], AW_0); //�ٿ����ͣ���AD
				bsp_DelayUS(TimeAfterOpen);
				adcContrast[detectAct.pinMapIndex%PIN_MAP_MAX][0] = bsp_GetAdScanValue();//���ƶ�
				bsp_AWSetPinVal(PinMap[detectAct.pinMapIndex%PIN_MAP_MAX][0], AW_1);//�ط���
				bsp_DelayUS(TimeAfterClose);
				adcContrast[detectAct.pinMapIndex%PIN_MAP_MAX][1] = bsp_GetAdScanValue();//�صƶ�
				bsp_AWSetPinVal(PinMap[detectAct.pinMapIndex%PIN_MAP_MAX][1], AW_1);//�ؽ���
				
				adcRealTime[detectAct.pinMapIndex%PIN_MAP_MAX] = adcContrast[detectAct.action/2][1] >= Sunlight ? 0 : adcContrast[detectAct.action/2][0];//�������̫�����˾͵�û�м�⵽�ϰ���
				adcIsSunlight[detectAct.pinMapIndex%PIN_MAP_MAX] = adcContrast[detectAct.action/2][1] >= Sunlight ? 1 : 0;//��¼�Ƿ񱻵����⵲ס��
				detectAct.pinMapIndex++;
				
				detectAct.delay = xTaskGetTickCount();
				detectAct.action++;
			}
		}break;
		

		//----------���Ӷ�--10-----------------------------------------------------------------------------
		case 18:
		{
			if(xTaskGetTickCount() - detectAct.delay >= (IntervalTime-ChargeTime)) 
			{
				bsp_AWSetPinVal(PinMap[detectAct.pinMapIndex%PIN_MAP_MAX][1], AW_0); //�ȿ����գ����
				detectAct.delay = xTaskGetTickCount();
				detectAct.action++;
			}
		}break;
		
		case 19:
		{
			if(xTaskGetTickCount() - detectAct.delay >= ChargeTime)
			{
				//bsp_AWSetPinVal(PinMap[detectAct.pinMapIndex%PIN_MAP_MAX][0], AW_0); //�ٿ����ͣ���AD
				bsp_DelayUS(TimeAfterOpen);
				adcContrast[detectAct.pinMapIndex%PIN_MAP_MAX][0] = bsp_GetAdScanValue();//���ƶ�
				bsp_AWSetPinVal(PinMap[detectAct.pinMapIndex%PIN_MAP_MAX][0], AW_1);//�ط���
				bsp_DelayUS(TimeAfterClose);
				adcContrast[detectAct.pinMapIndex%PIN_MAP_MAX][1] = bsp_GetAdScanValue();//�صƶ�
				bsp_AWSetPinVal(PinMap[detectAct.pinMapIndex%PIN_MAP_MAX][1], AW_1);//�ؽ���
				
				adcRealTime[detectAct.pinMapIndex%PIN_MAP_MAX] = adcContrast[detectAct.action/2][1] >= Sunlight ? 0 : adcContrast[detectAct.action/2][0];//�������̫�����˾͵�û�м�⵽�ϰ���
				adcIsSunlight[detectAct.pinMapIndex%PIN_MAP_MAX] = adcContrast[detectAct.action/2][1] >= Sunlight ? 1 : 0;//��¼�Ƿ񱻵����⵲ס��
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
*	�� �� ��: ADC_Configuration
*	����˵��: ����ADC, PC4��ΪADCͨ������
*	��    �Σ���
*	�� �� ֵ: ��
*********************************************************************************************************
*/
static void bsp_ADCConfig(void)
{
	ADC_InitTypeDef ADC_InitStructure; 
	GPIO_InitTypeDef GPIO_InitStructure;

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC |RCC_APB2Periph_ADC2	, ENABLE );	  //ʹ��ADC2ͨ��ʱ��
 

	RCC_ADCCLKConfig(RCC_PCLK2_Div6);   //����ADC��Ƶ����6 72M/6=12,ADC���ʱ�䲻�ܳ���14M

	//PA1 ��Ϊģ��ͨ����������                         
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;		//ģ����������
	GPIO_Init(GPIOC, &GPIO_InitStructure);	

	ADC_DeInit(ADC2);  //��λADC2 

	ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;	//ADC����ģʽ:ADC2��ADC2�����ڶ���ģʽ
	ADC_InitStructure.ADC_ScanConvMode = DISABLE;	//ģ��ת�������ڵ�ͨ��ģʽ
	ADC_InitStructure.ADC_ContinuousConvMode = DISABLE;	//ģ��ת�������ڵ���ת��ģʽ
	ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;	//ת��������������ⲿ��������
	ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;	//ADC�����Ҷ���
	ADC_InitStructure.ADC_NbrOfChannel = 1;	//˳����й���ת����ADCͨ������Ŀ
	ADC_Init(ADC2, &ADC_InitStructure);	//����ADC_InitStruct��ָ���Ĳ�����ʼ������ADCx�ļĴ���   

  
	ADC_Cmd(ADC2, ENABLE);	//ʹ��ָ����ADC2
	
	ADC_ResetCalibration(ADC2);	//ʹ�ܸ�λУ׼  
	 
	while(ADC_GetResetCalibrationStatus(ADC2));	//�ȴ���λУ׼����
	
	ADC_StartCalibration(ADC2);	 //����ADУ׼
 
	while(ADC_GetCalibrationStatus(ADC2));	 //�ȴ�У׼����
 
}



/*
*********************************************************************************************************
*	�� �� ��: AdcPro
*	����˵��: ADC������������1ms systick �жϽ��е���
*	��    �Σ���
*	�� �� ֵ: ��
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
		//����ָ��ADC�Ĺ�����ͨ����һ�����У�����ʱ��
		ADC_RegularChannelConfig(ADC2, ADC_Channel_14, 1, ADC_SampleTime_239Cycles5 );	//ADC2,ADCͨ��,����ʱ��Ϊ239.5����	  			    
	  
		ADC_SoftwareStartConvCmd(ADC2, ENABLE);		//ʹ��ָ����ADC2�����ת����������	
		 
		while(!ADC_GetFlagStatus(ADC2, ADC_FLAG_EOC ));//�ȴ�ת������
		ret = ADC_GetConversionValue(ADC2);	//�������һ��ADC2�������ת�����
		
		sum += ret;
	}
	
	voltage = (sum / (float)sampleCount)*3.3F/4096;

	return voltage;
}


/*
*********************************************************************************************************
*	�� �� ��: bsp_GetInfraredVoltageLeft
*	����˵��: ��ȡ��ߵĺ����ѹֵ
*	��    ��: ��
*	�� �� ֵ: ��
*********************************************************************************************************
*/
float bsp_GetInfraredVoltageLeft(void)
{
	return adcRealTime[8];
}

/*
*********************************************************************************************************
*	�� �� ��: bsp_GetInfraredVoltageRight
*	����˵��: ��ȡ�ұߵĺ����ѹֵ
*	��    ��: ��
*	�� �� ֵ: ��
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


	
	//�����̫�����䵽�ˣ�����ȫ��˸��
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



