#include "bsp.h"


typedef struct
{
	volatile bool isRunning;     /*����Ƿ�������*/
	volatile uint32_t encodeCnt; /*����������ֵ*/
	
}Motor;


typedef struct
{
	bool isRunning;
	uint32_t tick;
}Vacuum;


/*�������ṹ������*/
Motor motor[MOTOR_NUM];
static PID pid[2];
static Vacuum vacuum;

static void bspInitPWM(void);
static float pidabs(float val);
void bsp_InitMotorPid(MotorSN sn);
static void bsp_InitVacuum(void);


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
	bsp_InitMotorPid(MotorLeft);
	bsp_InitMotorPid(MotorRight);
	bsp_InitVacuum();
}


static void bsp_InitVacuum(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;

	/* ��GPIOʱ�� */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);

	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;	
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	
	GPIO_ResetBits(GPIOA,GPIO_Pin_0);
}

void bsp_VacuumClean(void)
{	
	if(!vacuum.isRunning)
		return ;
	
	++vacuum.tick;
	
	if(vacuum.tick <= 20)
	{
		GPIO_SetBits(GPIOA,GPIO_Pin_0);
	}
	else if(vacuum.tick > 20 && vacuum.tick <=40)
	{
		GPIO_ResetBits(GPIOA,GPIO_Pin_0);
	}
	else
	{
		vacuum.tick = 0 ;
	}
}


void bsp_StartVacuum(void)
{
	vacuum.tick = 0 ;
	vacuum.isRunning = true;
}

void bsp_StopVacuum(void)
{
	vacuum.isRunning = false;
	vacuum.tick = 0 ;
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
	/*�ֻ�*/
	GPIO_PinRemapConfig(GPIO_FullRemap_TIM1, ENABLE);           //Timer1��ӳ��     
	bsp_SetTIMOutPWM(GPIOE, GPIO_Pin_9,  TIM1, 1,0, MAXPWM);     //��Ƶ��Ϊ0��ռ�ձ�Ϊ100%ʱ��GPIO���1
	bsp_SetTIMOutPWM(GPIOE, GPIO_Pin_11, TIM1, 2,0, MAXPWM);     //��Ƶ��Ϊ0��ռ�ձ�Ϊ100%ʱ��GPIO���1
	bsp_SetTIMOutPWM(GPIOE, GPIO_Pin_13, TIM1, 3,0, MAXPWM);     //��Ƶ��Ϊ0��ռ�ձ�Ϊ100%ʱ��GPIO���1
	bsp_SetTIMOutPWM(GPIOE, GPIO_Pin_14, TIM1, 4,0, MAXPWM);     //��Ƶ��Ϊ0��ռ�ձ�Ϊ100%ʱ��GPIO���1
	
	/*��ˢ����ˢ*/
	GPIO_PinRemapConfig(GPIO_Remap_TIM4, ENABLE);           //Timer4��ӳ��    
	bsp_SetTIMOutPWM(GPIOD, GPIO_Pin_12, TIM4, 1,0, MAXPWM);     //��Ƶ��Ϊ0��ռ�ձ�Ϊ100%ʱ��GPIO���1
	bsp_SetTIMOutPWM(GPIOD, GPIO_Pin_13, TIM4, 2,0, MAXPWM);     //��Ƶ��Ϊ0��ռ�ձ�Ϊ100%ʱ��GPIO���1
	bsp_SetTIMOutPWM(GPIOD, GPIO_Pin_14, TIM4, 3,0, MAXPWM);     //��Ƶ��Ϊ0��ռ�ձ�Ϊ100%ʱ��GPIO���1
	bsp_SetTIMOutPWM(GPIOD, GPIO_Pin_15, TIM4, 4,0, MAXPWM);     //��Ƶ��Ϊ0��ռ�ձ�Ϊ100%ʱ��GPIO���1
	
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
			if(dir == Backward)
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
		
		case MotorRoller:
		{
			if(dir == Backward)
			{
				bsp_SetTIMOutPWM(GPIOD, GPIO_Pin_12, TIM4, 1,0, 0);
				bsp_SetTIMOutPWM(GPIOD, GPIO_Pin_13, TIM4, 2,DC_PWM_T, pwm);
			}
			else
			{
				bsp_SetTIMOutPWM(GPIOD, GPIO_Pin_13, TIM4, 2,0, 0);
				bsp_SetTIMOutPWM(GPIOD, GPIO_Pin_12, TIM4, 1,DC_PWM_T, pwm);
			}
		}break;
		
		case MotorBrush:
		{
			if(dir == Backward)
			{
				bsp_SetTIMOutPWM(GPIOD, GPIO_Pin_14, TIM4, 3,0, 0);
				bsp_SetTIMOutPWM(GPIOD, GPIO_Pin_15, TIM4, 4,DC_PWM_T, pwm);
			}
			else
			{
				bsp_SetTIMOutPWM(GPIOD, GPIO_Pin_15, TIM4, 4,0, 0);
				bsp_SetTIMOutPWM(GPIOD, GPIO_Pin_14, TIM4, 3,DC_PWM_T, pwm);
			}
		}break;
	}
}




void bsp_SetMotorTargetSpeed(MotorSN sn, float targetSpeed)
{	
	//���������PWM�Ѿ��������ֵ����ֱ���˳�����
	if(sn!=MotorLeft && sn!=MotorRight)
	{
		WARNING("����ĵ��\r\n");
		return ;
	}
	
	
	bsp_MotorCoast(sn);
	
	
	
	switch(sn)
	{
		case MotorLeft:
		{
			pid[MotorLeft].target = targetSpeed;
			motor[MotorLeft].isRunning = true ;
		}break;
		
		case MotorRight:
		{
			pid[MotorRight].target = targetSpeed;
			motor[MotorRight].isRunning = true ;
		}break;
	}
}


/*
*********************************************************************************************************
*	�� �� ��: bsp_InitMotorPid
*	����˵��: ��ʼ��ĳ�����PID
*	��    ��: ��
*	�� �� ֵ: ��
*********************************************************************************************************
*/
void bsp_InitMotorPid(MotorSN sn)
{
	switch(sn)
	{
		case MotorLeft:
		{
			/************************PID ���ֻ�**********12 0.5 1 0.8 5500*******/
			pid[MotorLeft].target = 250;
			
			pid[MotorLeft].kp = 8;
			pid[MotorLeft].ki = 0.5;
			pid[MotorLeft].kd = 1;
			
			pid[MotorLeft].bias = 0 ;
			pid[MotorLeft].lastBias = 0 ;
			pid[MotorLeft].biasSum = 0 ;
			
			pid[MotorLeft].pwm = 0 ;
			pid[MotorLeft].pwmMax = MAXPWM ;
			
			pid[MotorLeft].kiLimit = 20000 ;
			pid[MotorLeft].fitK = 0.8 ;
			pid[MotorLeft].fitD = 5600 ;
		}break;
		
		case MotorRight:
		{
			/************************PID ���ֻ�************************/
			pid[MotorRight].target = 250;
			
			pid[MotorRight].kp = 8;
			pid[MotorRight].ki = 0.5;
			pid[MotorRight].kd = 1;
			
			pid[MotorRight].bias = 0 ;
			pid[MotorRight].lastBias = 0 ;
			pid[MotorRight].biasSum = 0 ;
			
			pid[MotorRight].pwm = 0 ;
			pid[MotorRight].pwmMax = MAXPWM ;
			
			pid[MotorRight].kiLimit = 20000 ;
			pid[MotorRight].fitK = 0.8 ;
			pid[MotorRight].fitD = 5600 ;
		}break;
	}
}



/*
*********************************************************************************************************
*	�� �� ��: bsp_PidControlAct
*	����˵��: ʵʱ���ڵ��PID��100MS����һ��
*	��    ��: ��
*	�� �� ֵ: ��
*********************************************************************************************************
*/
void bsp_PidControlAct(void)
{
	int32_t speed = 0 ;
	float S_Out[4] = {0};
	
	UNUSED(S_Out);
	
	/************************��ʾ�ٶȲ���************************/
	#if 0 
	S_Out[0] = bsp_EncoderGetSpeed(EncoderLeft);
	S_Out[1] = bsp_EncoderGetSpeed(EncoderRight);
	SDS_OutPut_Data(S_Out);
	#endif
	
	/************************PID  ����************************/
	if(motor[MotorLeft].isRunning)
	{
		speed = bsp_EncoderGetSpeed(EncoderLeft);//��ǰ�ٶȺ���/��
		pid[MotorLeft].bias = pidabs(pid[MotorLeft].target) - speed;//��ǰ���ֵ
		pid[MotorLeft].biasSum += pid[MotorLeft].bias;//����ۻ�
		
		//��������
		if(pid[MotorLeft].biasSum >= pid[MotorLeft].kiLimit)
		{
			pid[MotorLeft].biasSum = pid[MotorLeft].kiLimit;
		}
			
		pid[MotorLeft].pwm = (pid[MotorLeft].fitK*pidabs(pid[MotorLeft].target) + pid[MotorLeft].fitD) + pid[MotorLeft].kp*pid[MotorLeft].bias + pid[MotorLeft].ki*pid[MotorLeft].biasSum + pid[MotorLeft].kd*(pid[MotorLeft].bias-pid[MotorLeft].lastBias); 
		pid[MotorLeft].lastBias = pid[MotorLeft].bias;//�ϴ����
		
		//PWM����
		if(pid[MotorLeft].pwm >= pid[MotorLeft].pwmMax)
		{
			pid[MotorLeft].pwm = pid[MotorLeft].pwmMax;
		}
		else if(pid[MotorLeft].pwm <= 0)
		{
			pid[MotorLeft].pwm = 0 ;
		}
		#if 0
		printf("left:%dmm/s  ",bsp_EncoderGetSpeed(EncoderLeft));
		printf("pid[MotorLeft].bias:%f  ",pid[MotorLeft].bias);
		printf("pid[MotorLeft].biasSum:%f  ",pid[MotorLeft].biasSum);
		printf("pid[MotorLeft].lastBias:%f  ",pid[MotorLeft].lastBias);
		printf("pid[MotorLeft].pwm:%f  ",pid[MotorLeft].pwm);
		#endif
		bsp_SetMotorPWM(MotorLeft, pid[MotorLeft].target>0 ? Forward:Backward, pid[MotorLeft].pwm);
	}
	
	
	/************************PID  ����************************/
	if(motor[MotorRight].isRunning)
	{
		speed = bsp_EncoderGetSpeed(EncoderRight);//��ǰ�ٶȺ���/��
		pid[MotorRight].bias = pidabs(pid[MotorRight].target) - speed;//��ǰ���ֵ
		pid[MotorRight].biasSum += pid[MotorRight].bias;//����ۻ�
		
		//��������
		if(pid[MotorRight].biasSum >= pid[MotorRight].kiLimit)
		{
			pid[MotorRight].biasSum = pid[MotorRight].kiLimit;
		}
			
		pid[MotorRight].pwm = (pid[MotorRight].fitK*pidabs(pid[MotorRight].target) + pid[MotorRight].fitD) + pid[MotorRight].kp*pid[MotorRight].bias + pid[MotorRight].ki*pid[MotorRight].biasSum + pid[MotorRight].kd*(pid[MotorRight].bias-pid[MotorRight].lastBias); 
		pid[MotorRight].lastBias = pid[MotorRight].bias;//�ϴ����
		
		//PWM����
		if(pid[MotorRight].pwm >= pid[MotorRight].pwmMax)
		{
			pid[MotorRight].pwm = pid[MotorRight].pwmMax;
		}
		else if(pid[MotorRight].pwm <= 0)
		{
			pid[MotorRight].pwm = 0 ;
		}
		#if 0
		printf("right:%dmm/s  ",bsp_EncoderGetSpeed(EncoderRight));
		printf("pid[MotorRight].bias:%f  ",pid[MotorRight].bias);
		printf("pid[MotorRight].biasSum:%f  ",pid[MotorRight].biasSum);
		printf("pid[MotorRight].lastBias:%f  ",pid[MotorRight].lastBias);
		printf("pid[MotorRight].pwm:%f\r\n",pid[MotorRight].pwm);
		#endif
		bsp_SetMotorPWM(MotorRight, pid[MotorRight].target>0 ? Forward:Backward, pid[MotorRight].pwm);
	}
	
}

static float pidabs(float val)
{
	return val>0 ? val : -val;
}









