#include "bsp.h"

#define RCC_ALL_RCC 	(RCC_APB2Periph_GPIOE|RCC_APB2Periph_AFIO)

/*���ڻ����뿪stopģʽ�İ���*/
#define GPIO_PORT_K    GPIOE
#define GPIO_PIN_K     GPIO_Pin_7

static void bsp_InitKeyStopMODE(void);
static void bsp_DISABLE_ALL_EXIT(void);
static void bsp_SetAllPinLowPower(void);

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
	/*�ϵ����������Դ*/
	bsp_DISABLE_ALL_EXIT();
	//bsp_SetAllPinLowPower();
	bsp_SwOff(SW_IR_POWER);
	bsp_SwOff(SW_MOTOR_POWER);
	bsp_SwOff(SW_VSLAM_POWER);
	bsp_SwOff(SW_WIFI_POWER);
	bsp_SwOff(SW_5V_EN_CTRL);
	bsp_SwOff(SW_3V3_EN_CTRL);
	
	
	/*��ʼ���ⲿ�ж����ţ�ר����������MCU*/
	bsp_InitKeyStopMODE();
	
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
		/*�����˺����³�ʼ������*/
		bsp_InitFormAwaken();
		
		/*������*/
		__disable_fault_irq();
		NVIC_SystemReset();
		
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
static void bsp_DISABLE_ALL_EXIT(void)
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


/*
*********************************************************************************************************
*	�� �� ��: bsp_SetAllPinLowPower
*	����˵��: ��������ȫ������Ϊ�͹���ģʽ
*	��    ��: ��
*	�� �� ֵ: ��
*********************************************************************************************************
*/
static void bsp_SetAllPinLowPower(void)
{
	/*
		#define RCC_ALL_SW 	(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOE | RCC_APB2Periph_GPIOF)

		#define GPIO_PORT_5V_EN_CTR  GPIOE
		#define GPIO_PIN_5V_EN_CTR   GPIO_Pin_15


		#define GPIO_PORT_IR_POWER  GPIOF
		#define GPIO_PIN_IR_POWER   GPIO_Pin_0


		#define GPIO_PORT_MOTOR_POWER  GPIOA
		#define GPIO_PIN_MOTOR_POWER   GPIO_Pin_9


		#define GPIO_PORT_ENCODER_POWER  GPIOA
		#define GPIO_PIN_ENCODER_POWER   GPIO_Pin_8
	*/
	
	GPIO_InitTypeDef GPIO_InitStructure;

	/* ��GPIOʱ�� */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB | RCC_APB2Periph_GPIOC | 
	RCC_APB2Periph_GPIOD | RCC_APB2Periph_GPIOE | RCC_APB2Periph_GPIOF | RCC_APB2Periph_GPIOG | RCC_APB2Periph_AFIO, ENABLE);
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, DISABLE);


	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;	/* �������ģʽ */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_All;
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_All & ~(GPIO_PIN_MOTOR_POWER | GPIO_PIN_VSLAM_POWER);
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_All;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_All;
	GPIO_Init(GPIOC, &GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_All;
	GPIO_Init(GPIOD, &GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_All & (~GPIO_PIN_5V_EN_CTR);
	GPIO_Init(GPIOE, &GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_All & (~GPIO_PIN_IR_POWER);
	GPIO_Init(GPIOF, &GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_All;
	GPIO_Init(GPIOG, &GPIO_InitStructure);
	
	ADC_Cmd(ADC1,DISABLE);
	ADC_Cmd(ADC2,DISABLE);
	ADC_Cmd(ADC3,DISABLE);
}

/*
*********************************************************************************************************
*	�� �� ��: bsp_InitKeyStopMODE
*	����˵��: �뿪stopģʽ�İ���
*	��    ��: ��
*	�� �� ֵ: ��
*********************************************************************************************************
*/
static void bsp_InitKeyStopMODE(void)
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
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
}
