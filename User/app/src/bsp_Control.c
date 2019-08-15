#include "bsp.h"

#define MAX_PWM      (3600) //���PWM����

typedef struct
{
	volatile float kp; /*�ٶ�KP*/
	volatile float ki; /*�ٶ�KI*/
	
	float bias;	       /*ƫ��*/
	float lastBias;    /*�ϴ�ƫ��*/
	float pwm;         /*ռ�ձ�*/ 

	

}PID;


/*�����ֻ���Ҫʹ��PID����*/ 
PID pid[2];

static int32_t bsp_PwmLimit(int32_t pwm);

void bsp_InitPid(MotorSN sn)
{
	if(sn == MotorLeft)
	{
		/*����KP KI����*/
		pid[0].kp = 20;
		pid[0].ki = 30;
		/*���֮ǰ�����*/
		pid[0].bias = 0 ;
		pid[0].lastBias = 0 ;
		pid[0].pwm = 0 ;
	}
	else if(sn == MotorRight)
	{
		/*����KP KI����*/
		pid[1].kp = 20;
		pid[1].ki = 30;
		/*���֮ǰ�����*/
		pid[1].bias = 0 ;
		pid[1].lastBias = 0 ;
		pid[1].pwm = 0 ;
	}
}


void bsp_PidClear(MotorSN sn)
{
	if(sn == MotorLeft)
	{
		/*���֮ǰ�����*/
		pid[0].bias = 0 ;
		pid[0].lastBias = 0 ;
		pid[0].pwm = 0 ;
	}
	else if(sn == MotorRight)
	{
		/*���֮ǰ�����*/
		pid[1].bias = 0 ;
		pid[1].lastBias = 0 ;
		pid[1].pwm = 0 ;
	}
}


void bsp_PidExec(MotorSN sn , int32_t Encoder, int32_t Target)
{

	/*���Ŀ���ٶ�С��0����ô�������ķ����ٶ�ҲӦ��Ϊ������������ֻ�ܼ�����û������*/
	if(Target < 0 ) 
	{
		Encoder = -Encoder;
	}
		
	/*����PWMֵ������ʽPID*/
	if(sn == MotorLeft)
	{
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
		pid[1].bias = Encoder-Target;                                
		pid[1].pwm += pid[1].kp*(pid[1].bias-pid[1].lastBias)+pid[1].ki*pid[1].bias;
		pid[1].lastBias=pid[1].bias;
		/*�޷�*/
		pid[1].pwm = bsp_PwmLimit(pid[1].pwm);
		/*����PWM*/
		bsp_MotorSetPWM(MotorRight,pid[1].pwm >0 ? Backward : Forward ,myabs(pid[1].pwm));
	}
	
}



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
int32_t myabs(int32_t val)
{ 		   
    int32_t temp;
	
    if(val < 0)  
		temp = -val;  
    else 
		temp = val;
	
    return temp;
}

