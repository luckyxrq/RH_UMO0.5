#include "bsp.h"


/*
	�ʼǣ�
	Ӳ��ǰ�᣺bsp_InitTimer1(7199,0);10KHz
	�����Ϊ0��TIM_SetCompare1(TIM1,0);
	�����Ϊ1��TIM_SetCompare1(TIM1,7200);!!!ǧ���д��TIM_SetCompare1(TIM1,7199);;
*/





static void bsp_InitTimer1(uint16_t arr,uint16_t psc);

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
	/*��ʼ��PWM 10KHZ�������������*/
	bsp_InitTimer1(7199,0); 
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
			if(dir != Forward)
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




