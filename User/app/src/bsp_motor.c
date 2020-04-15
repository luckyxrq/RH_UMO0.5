#include "bsp.h"


/*
	�ʼǣ�
	Ӳ��ǰ�᣺bsp_InitTimer1(7199,0);10KHz
	�����Ϊ0��TIM_SetCompare1(TIM1,0);
	�����Ϊ1��TIM_SetCompare1(TIM1,7200);!!!ǧ���д��TIM_SetCompare1(TIM1,7199);;
*/

/* ����ײ�������ض�Ӧ��RCCʱ�� */
#define RCC_ALL_VACUUM 	(RCC_APB2Periph_GPIOA)

#define GPIO_PORT_VACUUM   GPIOA
#define GPIO_PIN_VACUUM	   GPIO_Pin_0




static Vacuum vacuum;
static void bsp_InitTimer1(uint16_t arr,uint16_t psc);
static void bsp_InitTimer4(uint16_t arr,uint16_t psc);
static void bsp_InitVacuum(void);

/*
*********************************************************************************************************
*	�� �� ��: bsp_InitMotor
*	����˵��: ��ʼ�����
*	��    ��: ��
*	�� �� ֵ: ��
*********************************************************************************************************
*/
void bsp_InitMotor(void)
{
	/*��ʼ��PWM 20KHZ���������������10KHZ��������*/
	bsp_InitTimer1(3599,0); 
	bsp_InitTimer4(3599,0);
	bsp_InitVacuum();
	
}

/*
*********************************************************************************************************
*	�� �� ��: bsp_MotorSetPWM
*	����˵��: ͨ��PWMֱ�ӿ��Ƶ����PWM��С��ʱ�򣬵���޷�ת��
*	��    ��: ��
*	�� �� ֵ: ��
*********************************************************************************************************
*/
void bsp_MotorSetPWM(MotorSN sn ,MotorDir dir,uint16_t pwm)
{
	switch(sn)
	{
		case MotorLeft:
		{
			if(dir == Forward)
			{
				TIM_SetCompare3(TIM1,0);
				TIM_SetCompare4(TIM1,pwm);
			}
			else
			{
				TIM_SetCompare3(TIM1,pwm);
				TIM_SetCompare4(TIM1,0);
			}
		}break;
		
		case MotorRight:
		{
			if(dir == Forward)
			{
				TIM_SetCompare1(TIM1,0);
				TIM_SetCompare2(TIM1,pwm);
			}
			else
			{
				TIM_SetCompare1(TIM1,pwm);
				TIM_SetCompare2(TIM1,0);
			}
		}break;
	}
}

/*
*********************************************************************************************************
*	�� �� ��: bsp_MotorCleanSetPWM
*	����˵��: ͨ��PWMֱ�ӿ��Ƶ����PWM��С��ʱ�򣬵���޷�ת����ר��������ɨ�ĵ��
*	��    ��: ��
*	�� �� ֵ: ��
*********************************************************************************************************
*/
void bsp_MotorCleanSetPWM(MotorCleanSN sn, MotorCleanDir dir , uint16_t pwm)
{
	switch(sn)
	{
		case MotorRollingBrush:
		{
			if(dir == CW)
			{
				TIM_SetCompare1(TIM4,0);
				TIM_SetCompare2(TIM4,pwm);
			}
			else
			{
				TIM_SetCompare1(TIM4,pwm);
				TIM_SetCompare2(TIM4,0);
			}
		}break;
		
		case MotorSideBrush:
		{
			if(dir == CW)
			{
				TIM_SetCompare3(TIM4,0);
				TIM_SetCompare4(TIM4,pwm);
			}
			else
			{
				TIM_SetCompare3(TIM4,pwm);
				TIM_SetCompare4(TIM4,0);
			}
		}break;

	}
}


/*
*********************************************************************************************************
*	�� �� ��: bsp_InitVacuum
*	����˵��: ��ʼ��������IO�ڣ�ʹ��IO��ģ��PWM
*	��    ��: ��
*	�� �� ֵ: ��
*********************************************************************************************************
*/
static void bsp_InitVacuum(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;

	/* ��GPIOʱ�� */
	RCC_APB2PeriphClockCmd(RCC_ALL_VACUUM, ENABLE);

	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;	
	
	GPIO_InitStructure.GPIO_Pin = GPIO_PIN_VACUUM;
	GPIO_Init(GPIO_PORT_VACUUM, &GPIO_InitStructure);
	
	GPIO_ResetBits(GPIO_PORT_VACUUM,GPIO_PIN_VACUUM);
}

/*
*********************************************************************************************************
*	�� �� ��: bsp_VacuumClean
*	����˵��: �����Ե��ã�����������
*	��    ��: ��
*	�� �� ֵ: ��
*********************************************************************************************************
*/
void bsp_VacuumClean(void)
{	
	const uint32_t max_tick = 10;
	const uint32_t changeTick = 9;
	
	if(!vacuum.isRunning)
		return ;
	
	
	GPIO_SetBits(GPIOA,GPIO_Pin_0);
	
//	++vacuum.tick;
//	
//	if(vacuum.tick <= changeTick)
//	{
//		GPIO_SetBits(GPIOA,GPIO_Pin_0);
//	}
//	else if(vacuum.tick > changeTick && vacuum.tick <=max_tick)
//	{
//		GPIO_ResetBits(GPIOA,GPIO_Pin_0);
//	}
//	else
//	{
//		vacuum.tick = 0 ;
//	}
}

/*
*********************************************************************************************************
*	�� �� ��: bsp_StartVacuum
*	����˵��: ����������
*	��    ��: ��
*	�� �� ֵ: ��
*********************************************************************************************************
*/
void bsp_StartVacuum(void)
{
	vacuum.tick = 0 ;
	vacuum.isRunning = true;
}

/*
*********************************************************************************************************
*	�� �� ��: bsp_StopVacuum
*	����˵��: �ر�������
*	��    ��: ��
*	�� �� ֵ: ��
*********************************************************************************************************
*/
void bsp_StopVacuum(void)
{
	vacuum.isRunning = false;
	vacuum.tick = 0 ;
	
	GPIO_ResetBits(GPIO_PORT_VACUUM,GPIO_PIN_VACUUM);
}


/*
*********************************************************************************************************
*	�� �� ��: bsp_InitTimer1
*	����˵��: ��ʼ��PWM��ע�ⶨʱ���Ƿ���Ҫ��ӳ��
*	��    ��: ��
*	�� �� ֵ: ��
*********************************************************************************************************
*/
static void bsp_InitTimer1(uint16_t arr,uint16_t psc)
{  
	
	GPIO_InitTypeDef   GPIO_InitStructure;
	TIM_OCInitTypeDef  TIM_OCInitStructure;
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);
 	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOE|RCC_APB2Periph_AFIO, ENABLE);
	GPIO_PinRemapConfig(GPIO_FullRemap_TIM1, ENABLE); /*Timer1��ӳ��*/     	

	/*GPIO��ʼ��*/
	GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_9|GPIO_Pin_11|GPIO_Pin_13|GPIO_Pin_14;
	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF_PP;  //�����������
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOE, &GPIO_InitStructure);
	
	/*��ʱ����ʼ��*/
	TIM_TimeBaseStructure.TIM_Period 		= arr; 					
	TIM_TimeBaseStructure.TIM_Prescaler     = psc; 					
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1; 		
	TIM_TimeBaseStructure.TIM_CounterMode 	= TIM_CounterMode_Up;  	
	TIM_TimeBaseInit(TIM1, &TIM_TimeBaseStructure); 
	
	/*
	PWMһ��������ģʽ��PWM1ģʽ��CNT<CRRxΪ��Ч��ƽ�����������ڣ���CNT>=CRRxΪ��Ч��ƽ��PWM2ģʽ�෴��
	��ô��������ʲôΪ��Ч��ƽ��������ôȷ����
	������TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;������������ġ�
	��Ч��ƽ��������Ծ͵���ʲôʱ������ߵ�
	*/ 
	TIM_OCInitStructure.TIM_OCMode 		= TIM_OCMode_PWM1; 			
 	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable; 	
	TIM_OCInitStructure.TIM_OCPolarity  = TIM_OCPolarity_High;	

	/*TIM1��TIM8����ʹ�ã�������RTOS���޷�ʹ�ã���ʱ���˽�ԭ��*/
	TIM_OCInitStructure.TIM_OutputNState = TIM_OutputNState_Disable;	/* only for TIM1 and TIM8. */	
	TIM_OCInitStructure.TIM_OCNPolarity = TIM_OCNPolarity_High;			/* only for TIM1 and TIM8. */		
	TIM_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Reset;		/* only for TIM1 and TIM8. */
	TIM_OCInitStructure.TIM_OCNIdleState = TIM_OCNIdleState_Reset;		/* only for TIM1 and TIM8. */
	
	/*CH1*/
	TIM_OC1Init(TIM1, &TIM_OCInitStructure);	 
	TIM_OC1PreloadConfig(TIM1, TIM_OCPreload_Enable);
	
	/*CH2*/
	TIM_OC2Init(TIM1, &TIM_OCInitStructure); 
	TIM_OC2PreloadConfig(TIM1, TIM_OCPreload_Enable);
	
	/*CH3*/
	TIM_OC3Init(TIM1, &TIM_OCInitStructure);	 
	TIM_OC3PreloadConfig(TIM1, TIM_OCPreload_Enable);
	
	/*CH4*/
	TIM_OC4Init(TIM1, &TIM_OCInitStructure);	 
	TIM_OC4PreloadConfig(TIM1, TIM_OCPreload_Enable);
  
	/*ʹ��TIM1*/
	TIM_Cmd(TIM1, ENABLE); 
	
	/*TIM1��TIM8����ʹ�ã�������ʱ����ʹ�û�ʹ��*/
	TIM_CtrlPWMOutputs(TIM1,ENABLE);
	
	/*4��ͨ��ȫ������ߵ�ƽ*/
	TIM_SetCompare1(TIM1,CONSTANT_HIGH_PWM);
	TIM_SetCompare2(TIM1,CONSTANT_HIGH_PWM);
	TIM_SetCompare3(TIM1,CONSTANT_HIGH_PWM);
	TIM_SetCompare4(TIM1,CONSTANT_HIGH_PWM);
	
}


/*
*********************************************************************************************************
*	�� �� ��: bsp_InitTimer4
*	����˵��: ��ʼ��PWM��ע�ⶨʱ���Ƿ���Ҫ��ӳ��
*	��    ��: ��
*	�� �� ֵ: ��
*********************************************************************************************************
*/
static void bsp_InitTimer4(uint16_t arr,uint16_t psc)
{  
	
	GPIO_InitTypeDef   GPIO_InitStructure;
	TIM_OCInitTypeDef  TIM_OCInitStructure;
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);
 	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD|RCC_APB2Periph_AFIO, ENABLE);
	GPIO_PinRemapConfig(GPIO_Remap_TIM4, ENABLE); /*Timer1��ӳ��*/     	

	/*GPIO��ʼ��*/
	GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_12 | GPIO_Pin_13 |GPIO_Pin_14 | GPIO_Pin_15;
	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF_PP;  //�����������
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOD, &GPIO_InitStructure);
	
	/*��ʱ����ʼ��*/
	TIM_TimeBaseStructure.TIM_Period 		= arr; 					
	TIM_TimeBaseStructure.TIM_Prescaler     = psc; 					
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1; 		
	TIM_TimeBaseStructure.TIM_CounterMode 	= TIM_CounterMode_Up;  	
	TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure); 
	
	/*
	PWMһ��������ģʽ��PWM1ģʽ��CNT<CRRxΪ��Ч��ƽ�����������ڣ���CNT>=CRRxΪ��Ч��ƽ��PWM2ģʽ�෴��
	��ô��������ʲôΪ��Ч��ƽ��������ôȷ����
	������TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;������������ġ�
	��Ч��ƽ��������Ծ͵���ʲôʱ������ߵ�
	*/ 
	TIM_OCInitStructure.TIM_OCMode 		= TIM_OCMode_PWM1; 			
 	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable; 	
	TIM_OCInitStructure.TIM_OCPolarity  = TIM_OCPolarity_High;	

	/*TIM1��TIM8����ʹ�ã�������RTOS���޷�ʹ�ã���ʱ���˽�ԭ��*/
	#if 0
	TIM_OCInitStructure.TIM_OutputNState = TIM_OutputNState_Disable;	/* only for TIM1 and TIM8. */	
	TIM_OCInitStructure.TIM_OCNPolarity = TIM_OCNPolarity_High;			/* only for TIM1 and TIM8. */		
	TIM_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Reset;		/* only for TIM1 and TIM8. */
	TIM_OCInitStructure.TIM_OCNIdleState = TIM_OCNIdleState_Reset;		/* only for TIM1 and TIM8. */
	#endif
	
	/*CH1*/
	TIM_OC1Init(TIM4, &TIM_OCInitStructure);	 
	TIM_OC1PreloadConfig(TIM4, TIM_OCPreload_Enable);
	
	/*CH2*/
	TIM_OC2Init(TIM4, &TIM_OCInitStructure); 
	TIM_OC2PreloadConfig(TIM4, TIM_OCPreload_Enable);
	
	/*CH3*/
	TIM_OC3Init(TIM4, &TIM_OCInitStructure);	 
	TIM_OC3PreloadConfig(TIM4, TIM_OCPreload_Enable);
	
	/*CH4*/
	TIM_OC4Init(TIM4, &TIM_OCInitStructure);	 
	TIM_OC4PreloadConfig(TIM4, TIM_OCPreload_Enable);
  
	/*ʹ��TIM*/
	TIM_Cmd(TIM4, ENABLE); 
	
	/*TIM1��TIM8����ʹ�ã�������ʱ����ʹ�û�ʹ��*/
	TIM_CtrlPWMOutputs(TIM4,ENABLE);
	
	/*4��ͨ��ȫ������ߵ�ƽ*/
	TIM_SetCompare1(TIM4,CONSTANT_HIGH_PWM);
	TIM_SetCompare2(TIM4,CONSTANT_HIGH_PWM);
	TIM_SetCompare3(TIM4,CONSTANT_HIGH_PWM);
	TIM_SetCompare4(TIM4,CONSTANT_HIGH_PWM);
	
}



