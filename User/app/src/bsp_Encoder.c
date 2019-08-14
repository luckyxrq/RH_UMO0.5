#include "bsp.h"

/* �������ڶ�Ӧ��RCCʱ�� */
#define RCC_ALL_ENCODER 	(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOE)

#define GPIO_PORT_LEFT   GPIOA
#define GPIO_PIN_LEFT	 GPIO_Pin_15

#define GPIO_PORT_RIGHT  GPIOE
#define GPIO_PIN_RIGHT	 GPIO_Pin_3



static Encoder encoder;
static uint32_t interruptCount = 0 ;

static void bsp_InitEncoderIO(void);
static void bsp_InitEncoderTick(void);
static void bsp_EncoderCalcSpeed(EncoderSN sn);

/*
*********************************************************************************************************
*	�� �� ��: bsp_InitEncoder
*	����˵��:��ʼ����������Ҫ�Ķ�ʱ����IO�ڡ�
*	��    ��:  ��
*	�� �� ֵ: ��
*********************************************************************************************************
*/
void bsp_InitEncoder(void)
{
	bsp_InitEncoderIO();
	bsp_InitEncoderTick();
	
}

/*
*********************************************************************************************************
*	�� �� ��: bsp_InitEncoderIO
*	����˵��:��ʼ��������IO�ڡ�
*	��    ��:  ��
*	�� �� ֵ: ��
*********************************************************************************************************
*/
static void bsp_InitEncoderIO(void)
{
	GPIO_InitTypeDef  GPIO_InitStructure;
	
	RCC_APB2PeriphClockCmd(RCC_ALL_ENCODER, ENABLE);
				  
	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_IPU;			
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	
	GPIO_InitStructure.GPIO_Pin   = GPIO_PIN_LEFT;
	GPIO_Init(GPIO_PORT_LEFT, &GPIO_InitStructure); 
	
	GPIO_InitStructure.GPIO_Pin   = GPIO_PIN_RIGHT;
	GPIO_Init(GPIO_PORT_RIGHT, &GPIO_InitStructure); 
}


/*
*********************************************************************************************************
*	�� �� ��: bsp_InitEncoderTick
*	����˵��:��ʼ����������ʱ����
*	��    ��:  ��
*	�� �� ֵ: ��
*********************************************************************************************************
*/
static void bsp_InitEncoderTick(void)
{
	bsp_SetTIMforInt(TIM7, ENCODER_INTERRUPT_FREQUENCY, 1, 0);
}


/*
*********************************************************************************************************
*	�� �� ��: bsp_EncoderCalcSpeed
*	����˵��: ͨ�������������ٶȡ�
*	��    ��: ��
*	�� �� ֵ: ��
*********************************************************************************************************
*/
static void bsp_EncoderCalcSpeed(EncoderSN sn)
{
	encoder.speed[sn] = (float)encoder.risingCount[sn] / (float)Ratio *  (float)PERIMETER / 0.3F;
	encoder.risingCount[sn] = 0 ;
	
}


/*
*********************************************************************************************************
*	�� �� ��: bsp_EncoderGetSpeed
*	����˵��: ���ر����������ĵ�����е��ٶ�
*	��    ��: ���������
*	�� �� ֵ: ��
*********************************************************************************************************
*/
float bsp_EncoderGetSpeed(EncoderSN sn)
{
	return encoder.speed[sn];
}


/*
*********************************************************************************************************
*	�� �� ��: TIM7_IRQHandler
*	����˵��:ÿ100us����һ���жϣ��������һ�������أ������������1
*	��    ��: ��
*	�� �� ֵ: ��
*********************************************************************************************************
*/
void TIM7_IRQHandler(void)
{
	if(RESET != TIM_GetITStatus(TIM7, TIM_IT_Update)) 
	{
		TIM_ClearITPendingBit(TIM7, TIM_IT_Update); 
		++interruptCount;
		
		
		/*���������*/
		if(GPIO_ReadInputDataBit(GPIO_PORT_LEFT,  GPIO_PIN_LEFT) == 0)
		{
			encoder.isReadyRising[EncoderLeft] = true;
		}
		else
		{
			if(encoder.isReadyRising[EncoderLeft])
			{
				encoder.isReadyRising[EncoderLeft] = false;
				++encoder.risingCount[EncoderLeft];
				encoder.odometer[0] += (bsp_MotorGetDir(MotorLeft)==Forward ? 1 : -1);/*��̼�*/
			}
		}
		
		/*���������*/
		if(GPIO_ReadInputDataBit(GPIO_PORT_RIGHT,  GPIO_PIN_RIGHT) == 0)
		{
			encoder.isReadyRising[EncoderRight] = true;
		}
		else
		{
			if(encoder.isReadyRising[EncoderRight])
			{
				encoder.isReadyRising[EncoderRight] = false;
				++encoder.risingCount[EncoderRight];
				encoder.odometer[1] += (bsp_MotorGetDir(MotorRight)==Forward ? 1 : -1);/*��̼�*/
			}
		}

		/*�����ٶ�*/
		if(interruptCount >= CALC_T)
		{
			interruptCount = 0 ;
			
			bsp_EncoderCalcSpeed(EncoderLeft);
			bsp_EncoderCalcSpeed(EncoderRight);
		}

	}
}



int32_t bsp_encoderGetOdometer(MotorSN sn)
{
	int32_t odometer = 0 ;
	switch(sn)
	{
		case MotorLeft:
		{
			odometer = encoder.odometer[0];
		}break;
		
		case MotorRight:
		{
			odometer = encoder.odometer[1];
		}break;
	}
	
	return  odometer;
	
}







/////////////////////////////////////////////////////////////////


/* ���������������̷������� */
#define EXAMPLE_NAME	"V4-004_EXIT�ⲿ�ж�����"
#define EXAMPLE_DATE	"2015-08-30"
#define DEMO_VER		"1.0"





/*
*********************************************************************************************************
*	�� �� ��: EXTI0_Config
*	����˵��: �����ⲿ�жϡ�K1��K2��K3 �ж�.
*				K1�� PC13 ���½��ش���
*				K2�� PA0  : �����ش���
*				K3�� PG8  : �½��ش���
*	��    �Σ���
*	�� �� ֵ: ��
*********************************************************************************************************
*/
void EXTI_Config(void)
{
	/* ����PA15 */
	{
		GPIO_InitTypeDef GPIO_InitStructure;
		EXTI_InitTypeDef   EXTI_InitStructure;
		NVIC_InitTypeDef   NVIC_InitStructure;		
		
		/* ʹ�� GPIO ʱ�� */
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);

		/* ���� PA0 Ϊ���븡��ģʽ */
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_15;
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
		GPIO_Init(GPIOA, &GPIO_InitStructure);

		/* ʹ�� AFIO ʱ�� */
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);

		/* Connect EXTI0 Line to PA.00 pin */
		GPIO_EXTILineConfig(GPIO_PortSourceGPIOA, GPIO_PinSource15);

		/* Configure EXTI0 line */
		EXTI_InitStructure.EXTI_Line = EXTI_Line15;
		EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
		EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising_Falling;  	
		EXTI_InitStructure.EXTI_LineCmd = ENABLE;
		EXTI_Init(&EXTI_InitStructure);

		/* Enable and set EXTI0 Interrupt to the lowest priority */
		NVIC_InitStructure.NVIC_IRQChannel = EXTI15_10_IRQn;
		NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x0F;
		NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x0F;
		NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
		NVIC_Init(&NVIC_InitStructure);
	}
	
	/* ���� PE3 �ⲿ�ж� */
	{
		GPIO_InitTypeDef GPIO_InitStructure;
		EXTI_InitTypeDef   EXTI_InitStructure;
		NVIC_InitTypeDef   NVIC_InitStructure;			
		
		/* ʹ�� GPIO ʱ�� */
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOE, ENABLE);

		/* ���� PC13 Ϊ���븡��ģʽ */
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
		GPIO_Init(GPIOE, &GPIO_InitStructure);

		/* ʹ�� AFIO ʱ�� */
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);

		/* Connect EXTI13 Line to PA.00 pin */
		GPIO_EXTILineConfig(GPIO_PortSourceGPIOE, GPIO_PinSource3);

		/* Configure EXTI13 line */
		EXTI_InitStructure.EXTI_Line = EXTI_Line3;
		EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
		EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising_Falling;  	/* �����ش��� */
		EXTI_InitStructure.EXTI_LineCmd = ENABLE;
		EXTI_Init(&EXTI_InitStructure);

		/* Enable and set EXTI13 Interrupt to the lowest priority */
		NVIC_InitStructure.NVIC_IRQChannel = EXTI3_IRQn;
		NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x0F;
		NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x0F;
		NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
		NVIC_Init(&NVIC_InitStructure);
	}			
}

/*
*********************************************************************************************************
*	�� �� ��: EXTI9_5_IRQHandler
*	����˵��: �ⲿ�жϷ������
*	��    �Σ���
*	�� �� ֵ: ��
*********************************************************************************************************
*/
void EXTI15_10_IRQHandler(void)
{

	if (EXTI_GetITStatus(EXTI_Line15) != RESET)
	{
		EXTI_ClearITPendingBit(EXTI_Line15);

		DEBUG("EXTI_Line15");
	}
}


/*
*********************************************************************************************************
*	�� �� ��: EXTI9_5_IRQHandler
*	����˵��: �ⲿ�жϷ������
*	��    �Σ���
*	�� �� ֵ: ��
*********************************************************************************************************
*/
void EXTI3_IRQHandler(void)
{

	if (EXTI_GetITStatus(EXTI_Line3) != RESET)
	{
		EXTI_ClearITPendingBit(EXTI_Line3);
		
		DEBUG("EXTI_Line3");
	}
}


/***************************** ���������� www.armfly.com (END OF FILE) *********************************/



