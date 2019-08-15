#include "bsp.h"

/*
**********************************************************************************************************
											��������
**********************************************************************************************************
*/
static Encoder encoder[ENCODER_NUM];

/*
**********************************************************************************************************
											��������
**********************************************************************************************************
*/
static void bsp_InitGPIO(void);
static void bsp_EXTI_Config(void);

/*
*********************************************************************************************************
*	�� �� ��: bsp_InitEncoder
*	����˵��: ��ʼ��������
*	��    �Σ���
*	�� �� ֵ: ��
*********************************************************************************************************
*/
void bsp_InitEncoder(void)
{
	UNUSED(encoder);
	
	bsp_InitGPIO();
}


/*
*********************************************************************************************************
*	�� �� ��: bsp_EncoderGetTotalMileage
*	����˵��: ��ȡ�������������������
*	��    �Σ���
*	�� �� ֵ: ��
*********************************************************************************************************
*/
uint32_t bsp_EncoderGetTotalMileage(EncoderSN sn)
{
	return encoder[sn].totalMileage;
}


/*
*********************************************************************************************************
*	�� �� ��: bsp_EncoderGetPulseT
*	����˵��: ������M���ٷ��������Եĵ��ã�5MS��10MS����һ�ξ��ɣ����۴�ֵ������������ֻ��һ��������Ӳ������
*             �޷��ж����� 
*	��    �Σ���
*	�� �� ֵ: ��
*********************************************************************************************************
*/
int32_t bsp_EncoderGetPulseT(EncoderSN sn)
{
	int32_t pulseCnt = 0 ;
	
	pulseCnt = encoder[sn].pulseT;
	encoder[sn].pulseT = 0 ;
	
	return pulseCnt;
}


/*
*********************************************************************************************************
*	�� �� ��: bsp_InitGPIO
*	����˵��: ��ʼ��������IO�ۣ�ɨ�ػ�ÿ�����ӱ�����ֻ��һ�������������û������
*	��    �Σ���
*	�� �� ֵ: ��
*********************************************************************************************************
*/
static void bsp_InitGPIO(void)
{
	bsp_EXTI_Config();
}


/*
*********************************************************************************************************
*	�� �� ��: bsp_EXTI_Config
*	����˵��: �����ⲿ�жϡ��������Ѿ������������裬��Ƭ������Ϊ�������룬2��Ƶ�������壬���½��ش���
*	��    �Σ���
*	�� �� ֵ: ��
*********************************************************************************************************
*/
static void bsp_EXTI_Config(void)
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
		NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x00;
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
		NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x00;
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
		
		++encoder[EncoderLeft].totalMileage; /*�������������1��������*/
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
		
		++encoder[EncoderRight].totalMileage; /*�������������1��������*/
	}
}


/***************************** ���������� www.armfly.com (END OF FILE) *********************************/



