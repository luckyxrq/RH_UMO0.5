#include "bsp.h"

#define MAX_PWM               (3600) //���PWM����
#define SPEED_SAMP_COUNT      (10)   //�ٶ��˲�����FIFO���

typedef struct
{
	volatile float kp; /*�ٶ�KP*/
	volatile float ki; /*�ٶ�KI*/
	
	volatile float bias;	    /*ƫ��*/
	volatile float lastBias;    /*�ϴ�ƫ��*/
	volatile float pwm;         /*ռ�ձ�*/ 

	volatile int32_t target;             /*Ŀ���ٶ�*/
	volatile int32_t lastTarget;         /*��һ��Ŀ���ٶ�*/ 
}PID;


/*�����ֻ���Ҫʹ��PID����*/ 
static PID pid[2];
/*�ٶȣ�MS/S , 10MS����һ��*/ 
static int32_t speed[2][SPEED_SAMP_COUNT];
/*����ָʾ����FIFO���*/
static uint8_t sampleIndex[2] = {0,0} ;

static void bsp_PidClear(MotorSN sn);
static void bsp_MotorBrake(MotorSN sn);
static void bsp_PidExec(MotorSN sn , int32_t Encoder, int32_t Target);
static int32_t bsp_PwmLimit(int32_t pwm);
static int32_t myabs(int32_t val);

/*
*********************************************************************************************************
*	�� �� ��: bsp_InitPid
*	����˵��: ��ʼ�����PID����
*	��    �Σ���
*	�� �� ֵ: ��
*********************************************************************************************************
*/
void bsp_InitPid(MotorSN sn)
{
	if(sn == MotorLeft)
	{
		/*����KP KI����*/
		pid[0].kp = 5;
		pid[0].ki = 10;
		/*���֮ǰ�����*/
		pid[0].bias = 0 ;
		pid[0].lastBias = 0 ;
		pid[0].pwm = 0 ;
		/*Ŀ���ٶ�*/
		pid[0].target = 0 ;
		/*��һ��Ŀ���ٶ�*/
		pid[0].lastTarget = 0 ;
	}
	else if(sn == MotorRight)
	{
		/*����KP KI����*/
		pid[1].kp = 5;
		pid[1].ki = 10;
		/*���֮ǰ�����*/
		pid[1].bias = 0 ;
		pid[1].lastBias = 0 ;
		pid[1].pwm = 0 ;
		/*Ŀ���ٶ�*/
		pid[1].target = 0 ;
		/*��һ���ٶ�*/
		pid[1].lastTarget = 0 ;
	}
}

/*
*********************************************************************************************************
*	�� �� ��: bsp_SetMotorSpeed
*	����˵��: ���õ��Ŀ���ٶȣ�������
*	��    �Σ���
*	�� �� ֵ: ��
*********************************************************************************************************
*/
void bsp_SetMotorSpeed(MotorSN sn , int32_t speed)
{
	if(sn == MotorLeft)
	{
		pid[0].target = speed;
	}
	else if(sn == MotorRight)
	{
		pid[1].target = speed;
	}
}

/*
*********************************************************************************************************
*	�� �� ��: bsp_MotorGetTargetSpeed
*	����˵��: ��ȡ�����Ŀ���ٶ�
*	��    �Σ���
*	�� �� ֵ: ��
*********************************************************************************************************
*/
int32_t bsp_MotorGetTargetSpeed(MotorSN sn)
{
	return pid[sn].target;
}

/*
*********************************************************************************************************
*	�� �� ��: bsp_PidSched
*	����˵��: PID���Ⱥ����������Ե��ã�10MSһ��
*	��    �Σ���
*	�� �� ֵ: ��
*********************************************************************************************************
*/
void bsp_PidSched(void)
{
	bsp_PidExec(MotorLeft, bsp_EncoderGetPulseT(EncoderLeft), pid[0].target);
	bsp_PidExec(MotorRight,bsp_EncoderGetPulseT(EncoderRight),pid[1].target);
}


/*
*********************************************************************************************************
*	�� �� ��: bsp_MotorGetSpeed
*	����˵��: ���ص��ʵʱ�ٶȣ���λMM/S
*	��    �Σ���
*	�� �� ֵ: ��
*********************************************************************************************************
*/
int32_t bsp_MotorGetSpeed(MotorSN sn)
{
	uint8_t i = 0 ;
	int32_t sum = 0 ;
	int32_t ret = 0 ;
	
	for(i=0; i< SPEED_SAMP_COUNT ; i++)
	{
		sum += speed[sn][i];
	}
	
	ret = (float)sum / (float)SPEED_SAMP_COUNT;
	
	return ret;
}


/*
*********************************************************************************************************
*	�� �� ��: bsp_PidClear
*	����˵��: ��յ��PID�ۻ�����
*	��    �Σ���
*	�� �� ֵ: ��
*********************************************************************************************************
*/
static void bsp_PidClear(MotorSN sn)
{
	if(sn == MotorLeft)
	{
		/*���֮ǰ�����*/
		pid[0].bias = 0 ;
		pid[0].lastBias = 0 ;
		pid[0].pwm = 0 ;

		/*��һ��Ŀ���ٶ�*/
		pid[0].lastTarget = 0 ;
	}
	else if(sn == MotorRight)
	{
		/*���֮ǰ�����*/
		pid[1].bias = 0 ;
		pid[1].lastBias = 0 ;
		pid[1].pwm = 0 ;

		/*��һ��Ŀ���ٶ�*/
		pid[1].lastTarget = 0 ;
	}
}

/*
*********************************************************************************************************
*	�� �� ��: bsp_MotorBrake
*	����˵��: �����ͣ
*	��    �Σ���
*	�� �� ֵ: ��
*********************************************************************************************************
*/
static void bsp_MotorBrake(MotorSN sn)
{
	if(sn == MotorLeft)
	{
		TIM_SetCompare3(TIM1,CONSTANT_HIGH_PWM);
		TIM_SetCompare4(TIM1,CONSTANT_HIGH_PWM);
	}
	else if(sn == MotorRight)
	{
		TIM_SetCompare1(TIM1,CONSTANT_HIGH_PWM);
		TIM_SetCompare2(TIM1,CONSTANT_HIGH_PWM);
	}

}

/*
*********************************************************************************************************
*	�� �� ��: bsp_PidExec
*	����˵��: PIDִ�к���������ռ�ձ�
*	��    �Σ���
*	�� �� ֵ: ��
*********************************************************************************************************
*/
static void bsp_PidExec(MotorSN sn , int32_t Encoder, int32_t Target)
{
	/*���Ŀ���ٶ�Ϊ0*/
	if(Target == 0)
	{
		bsp_MotorBrake(sn);
		bsp_PidClear(sn);
	}
	
	
	/*���Ŀ���ٶ�С��0����ô�������ķ����ٶ�ҲӦ��Ϊ������������ֻ�ܼ�����û������*/
	if(Target < 0 ) 
	{
		Encoder = -Encoder;
	}
	
	/*�����ٶȣ�250MM/Sʱ��10MS��12������*/
	speed[sn][sampleIndex[sn]++] = Encoder / 12.0F * 250;
	if(sampleIndex[sn] >= SPEED_SAMP_COUNT)
	{
		sampleIndex[sn] = 0 ;
	}
		
	/*����PWMֵ������ʽPID*/
	if(sn == MotorLeft)
	{
		if(pid[0].lastTarget == 0) /*��0������*/
		{
			pid[0].lastTarget = Target;
		}
		else if(pid[0].lastTarget / (float)Target < 0) /*2��Ŀ���ٶȷ����෴*/
		{
			bsp_MotorBrake(sn);
			bsp_PidClear(sn);
			return ;
		}
		
		pid[0].bias = Encoder-Target;                                  
		pid[0].pwm += pid[0].kp*(pid[0].bias-pid[0].lastBias)+pid[0].ki*pid[0].bias;
		pid[0].lastBias=pid[0].bias;
		/*�޷�*/
		pid[0].pwm = bsp_PwmLimit(pid[0].pwm);	
		/*����PWM*/
		bsp_MotorSetPWM(MotorLeft,pid[0].pwm >0 ? Backward : Forward ,myabs(pid[0].pwm));
	}
	else if(sn == MotorRight)
	{
		if(pid[1].lastTarget == 0) /*��0������*/
		{
			pid[1].lastTarget = Target;
		}
		else if(pid[1].lastTarget / (float)Target < 0) /*2��Ŀ���ٶȷ����෴*/
		{
			bsp_MotorBrake(sn);
			bsp_PidClear(sn);
			return ;
		}
		
		pid[1].bias = Encoder-Target;                                
		pid[1].pwm += pid[1].kp*(pid[1].bias-pid[1].lastBias)+pid[1].ki*pid[1].bias;
		pid[1].lastBias=pid[1].bias;
		/*�޷�*/
		pid[1].pwm = bsp_PwmLimit(pid[1].pwm);
		/*����PWM*/
		bsp_MotorSetPWM(MotorRight,pid[1].pwm >0 ? Backward : Forward ,myabs(pid[1].pwm));
	}
	
}


/*
*********************************************************************************************************
*	�� �� ��: bsp_PwmLimit
*	����˵��: PID�޷�
*	��    �Σ���
*	�� �� ֵ: ��
*********************************************************************************************************
*/
static int32_t bsp_PwmLimit(int32_t pwm)
{
	int32_t ret = 0 ;
	
	if(pwm >= MAX_PWM)
	{
		ret = MAX_PWM;
	}
	else if(pwm <= -MAX_PWM)
	{
		ret = -MAX_PWM;
	}
	else
	{
		ret = pwm;
	}
	
	return ret;
}	


/*
*********************************************************************************************************
*	�� �� ��: myabs
*	����˵��: �����ֵ
*	��    �Σ���
*	�� �� ֵ: ��
*********************************************************************************************************
*/
static int32_t myabs(int32_t val)
{ 		   
    int32_t temp;
	
    if(val < 0)  
		temp = -val;  
    else 
		temp = val;
	
    return temp;
}

