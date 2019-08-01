#include "bsp.h"





typedef struct
{
	bool isRunning;     /*����Ƿ�������*/
	uint32_t encodeCnt; /*����������ֵ*/
	
}Motor;


/*�������ṹ������*/
Motor motor[MOTOR_NUM];

static void bspInitPWM(void);


/*
*********************************************************************************************************
*	�� �� ��: bsp_InitDC_Motor
*	����˵��: ��ʼ��DC�����
*			 ȫ�ֱ�����
*	��    ��: ��
*	�� �� ֵ: ��
*********************************************************************************************************
*/
void bsp_InitDC_Motor(void)
{
	bspInitPWM();
}

/*
*********************************************************************************************************
*	�� �� ��: bspInitPWM
*	����˵��: ��ʼ��DC�����Ҫ�Ķ�ʱ����PWM�����
*			 ȫ�ֱ�����
*	��    ��: ��
*	�� �� ֵ: ��
*********************************************************************************************************
*/
static void bspInitPWM(void)
{
	GPIO_PinRemapConfig(GPIO_FullRemap_TIM1, ENABLE);           //Timer1��ӳ��     
	bsp_SetTIMOutPWM(GPIOE, GPIO_Pin_9,  TIM1, 1,0, MAXPWM);     //��Ƶ��Ϊ0��ռ�ձ�Ϊ100%ʱ��GPIO���1
	bsp_SetTIMOutPWM(GPIOE, GPIO_Pin_11, TIM1, 2,0, MAXPWM);     //��Ƶ��Ϊ0��ռ�ձ�Ϊ100%ʱ��GPIO���1
	bsp_SetTIMOutPWM(GPIOE, GPIO_Pin_13, TIM1, 3,0, MAXPWM);     //��Ƶ��Ϊ0��ռ�ձ�Ϊ100%ʱ��GPIO���1
	bsp_SetTIMOutPWM(GPIOE, GPIO_Pin_14, TIM1, 4,0, MAXPWM);     //��Ƶ��Ϊ0��ռ�ձ�Ϊ100%ʱ��GPIO���1
}

/*
*********************************************************************************************************
*	�� �� ��: bsp_MotorBrake
*	����˵��: �����ͣ
*	��    ��: ��
*	�� �� ֵ: ��
*********************************************************************************************************
*/
void bsp_MotorBrake(MotorSN sn)
{
	switch(sn)
	{
		case MotorLeft:
		{
			motor[MotorLeft].isRunning = false ;
			bsp_SetTIMOutPWM(GPIOE, GPIO_Pin_13, TIM1, 3,0,MAXPWM); 
			bsp_SetTIMOutPWM(GPIOE, GPIO_Pin_14, TIM1, 4,0,MAXPWM);
		}break;
		
		case MotorRight:
		{
			motor[MotorRight].isRunning = false ;
			bsp_SetTIMOutPWM(GPIOE, GPIO_Pin_9,  TIM1, 1,0,MAXPWM);
			bsp_SetTIMOutPWM(GPIOE, GPIO_Pin_11, TIM1, 2,0,MAXPWM);
		}break;
	}
	
}

/*
*********************************************************************************************************
*	�� �� ��: bsp_MotorCoast
*	����˵��: �������ͣ
*	��    ��: ��
*	�� �� ֵ: ��
*********************************************************************************************************
*/
void bsp_MotorCoast(MotorSN sn)
{
	switch(sn)
	{
		case MotorLeft:
		{
			motor[MotorLeft].isRunning = false ;
			bsp_SetTIMOutPWM(GPIOE, GPIO_Pin_13, TIM1, 3,0,0); 
			bsp_SetTIMOutPWM(GPIOE, GPIO_Pin_14, TIM1, 4,0,0);
		}break;
		
		case MotorRight:
		{
			motor[MotorRight].isRunning = false ;
			bsp_SetTIMOutPWM(GPIOE, GPIO_Pin_9,  TIM1, 1,0,0);
			bsp_SetTIMOutPWM(GPIOE, GPIO_Pin_11, TIM1, 2,0,0);
		}break;
	}
	
}

/*
*********************************************************************************************************
*	�� �� ��: bsp_SetMotorPWM
*	����˵��: �����ţ�����ռ�ձȣ������ƣ�������ͷ�ļ���
*	��    ��: ��
*	�� �� ֵ: ��
*********************************************************************************************************
*/
void bsp_SetMotorPWM(MotorSN sn, MotorDir dir, uint16_t pwm)
{	
	//���������PWM�Ѿ��������ֵ����ֱ���˳�����
	if(pwm > MAXPWM)
	{
		WARNING("PWMֵ������MAXPWM\r\n");
		return ;
	}
	
	switch(sn)
	{
		case MotorLeft:
		{
			motor[MotorLeft].isRunning = true ;
			if(dir == Forward)
			{
				bsp_SetTIMOutPWM(GPIOE, GPIO_Pin_13, TIM1, 3,0, 0); 
				bsp_SetTIMOutPWM(GPIOE, GPIO_Pin_14, TIM1, 4,DC_PWM_T, pwm);
			}
			else
			{
				bsp_SetTIMOutPWM(GPIOE, GPIO_Pin_14, TIM1, 4,0, 0);
				bsp_SetTIMOutPWM(GPIOE, GPIO_Pin_13, TIM1, 3, DC_PWM_T,pwm); 
			}
		}break;
		
		case MotorRight:
		{
			motor[MotorRight].isRunning = true ;	
			if(dir == Forward)
			{
				bsp_SetTIMOutPWM(GPIOE, GPIO_Pin_9,  TIM1, 1,0, 0);
				bsp_SetTIMOutPWM(GPIOE, GPIO_Pin_11, TIM1, 2,DC_PWM_T, pwm);
			}
			else
			{
				bsp_SetTIMOutPWM(GPIOE, GPIO_Pin_11, TIM1, 2,0, 0);
				bsp_SetTIMOutPWM(GPIOE, GPIO_Pin_9,  TIM1, 1,DC_PWM_T, pwm);
			}
		}break;
	}
}






