#include "bsp.h"

#define RCC_ALL_RCC 	(RCC_APB2Periph_GPIOE|RCC_APB2Periph_AFIO)

/*���ڻ����뿪stopģʽ�İ���*/
#define GPIO_PORT_K    GPIOE
#define GPIO_PIN_K     GPIO_Pin_7

/*
*********************************************************************************************************
*	�� �� ��: bsp_InitKeyStopMODE
*	����˵��: �뿪stopģʽ�İ���
*	��    ��: ��
*	�� �� ֵ: ��
*********************************************************************************************************
*/
void bsp_InitKeyStopMODE(void)
{
    GPIO_InitTypeDef  GPIO_InitStructure;
	EXTI_InitTypeDef  EXTI_InitStructure;
	NVIC_InitTypeDef  NVIC_InitStructure;

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_PWR, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_ALL_RCC, ENABLE);

	/* �������� */
	GPIO_InitStructure.GPIO_Pin = GPIO_PIN_K;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
	GPIO_Init(GPIO_PORT_K, &GPIO_InitStructure);

	GPIO_EXTILineConfig(GPIO_PortSourceGPIOE, GPIO_PinSource7);

    /* �����ⲿ�ж��¼� */
    EXTI_InitStructure.EXTI_Line = EXTI_Line7;
    EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;
	EXTI_InitStructure.EXTI_LineCmd = ENABLE;
    EXTI_Init(&EXTI_InitStructure);

	/* 16����ռʽ���ȼ���0����Ӧʽ���ȼ� */
    NVIC_InitStructure.NVIC_IRQChannel = EXTI9_5_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
}

/*
*********************************************************************************************************
*	�� �� ��: bsp_EnterStopMODE
*	����˵��: ����stopģʽ
*	��    ��: ��
*	�� �� ֵ: ��
*********************************************************************************************************
*/
void bsp_EnterStopMODE(void)
{
	/*
		1. ֹͣģʽ����Cortex-M3����˯��ģʽ�����Ͻ���������ʱ�ӿ��ƻ��ƣ���ֹͣģʽ�µ�ѹ
		��������������������͹���ģʽ����ʱ��1.8V��������ĵ�����ʱ�Ӷ���ֹͣ�� PLL�� HSI��
		HSE��RC�����Ĺ��ܱ���ֹ�� SRAM�ͼĴ������ݱ�����������
		2. ��ֹͣģʽ�£����е�I/O���Ŷ���������������ģʽʱ��״̬��
		3. һ��Ҫ�رյδ�ʱ����ʵ�ʲ��Է��ֵδ�ʱ���ж�Ҳ�ܻ���ͣ��ģʽ��
	*/
	SysTick->CTRL &= ~SysTick_CTRL_ENABLE_Msk;  /* �رյδ�ʱ�� */  
	PWR_EnterSTOPMode(PWR_Regulator_LowPower, PWR_STOPEntry_WFE);				
	portENTER_CRITICAL();

	/* 
		1����һ���жϻ����¼������˳�ֹͣģʽʱ�� HSI RC������ѡΪϵͳʱ�ӡ�
		2���˳��͹��ĵ�ͣ��ģʽ����Ҫ��������ʹ��HSE��		
	*/
	RCC_HSEConfig(RCC_HSE_ON);
	while (RCC_GetFlagStatus(RCC_FLAG_HSERDY) == RESET){}

	RCC_PLLCmd(ENABLE);
	while (RCC_GetFlagStatus(RCC_FLAG_PLLRDY) == RESET){}
	RCC_SYSCLKConfig(RCC_SYSCLKSource_PLLCLK);

	while (RCC_GetSYSCLKSource() != 0x08){}
	SysTick->CTRL |= SysTick_CTRL_ENABLE_Msk; /* ʹ�ܵδ�ʱ�� */  
	portEXIT_CRITICAL();
}


void EXTI9_5_IRQHandler(void)
{
	if(EXTI_GetITStatus(EXTI_Line7) == SET)
	{	
		bsp_InitEncoder();
		bsp_SwOn(SW_IR_POWER);
		bsp_SwOn(SW_MOTOR_POWER);
		
		EXTI_ClearITPendingBit(EXTI_Line7); /* ����жϱ�־λ */
	}
}

/*
*********************************************************************************************************
*	�� �� ��: bsp_DISABLE_ALL_EXIT
*	����˵��: �ر����е��ⲿ�ж�
*	��    ��: ��
*	�� �� ֵ: ��
*********************************************************************************************************
*/
void bsp_DISABLE_ALL_EXIT(void)
{
	NVIC_InitTypeDef  NVIC_InitStructure;
	
    NVIC_InitStructure.NVIC_IRQChannel = EXTI0_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = DISABLE;
    NVIC_Init(&NVIC_InitStructure);
	
	NVIC_InitStructure.NVIC_IRQChannel = EXTI0_IRQn;
	NVIC_Init(&NVIC_InitStructure);
	
	NVIC_InitStructure.NVIC_IRQChannel = EXTI1_IRQn;
	NVIC_Init(&NVIC_InitStructure);
	
	NVIC_InitStructure.NVIC_IRQChannel = EXTI2_IRQn;
	NVIC_Init(&NVIC_InitStructure);
	
	NVIC_InitStructure.NVIC_IRQChannel = EXTI3_IRQn;
	NVIC_Init(&NVIC_InitStructure);
	
	NVIC_InitStructure.NVIC_IRQChannel = EXTI4_IRQn;
	NVIC_Init(&NVIC_InitStructure);
	
	NVIC_InitStructure.NVIC_IRQChannel = EXTI9_5_IRQn;
	NVIC_Init(&NVIC_InitStructure);
	
	NVIC_InitStructure.NVIC_IRQChannel = EXTI15_10_IRQn;
	NVIC_Init(&NVIC_InitStructure);
}

