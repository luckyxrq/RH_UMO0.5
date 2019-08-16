#include "bsp.h"

#define MAX_PWM      (3600) //最大PWM绝对

typedef struct
{
	volatile float kp; /*速度KP*/
	volatile float ki; /*速度KI*/
	
	volatile float bias;	    /*偏差*/
	volatile float lastBias;    /*上次偏差*/
	volatile float pwm;         /*占空比*/ 

	volatile int32_t target;             /*目的速度*/
	volatile int32_t lastTarget;         /*上一次目的速度*/ 
}PID;


/*左右轮机需要使用PID控制*/ 
PID pid[2];

static void bsp_PidClear(MotorSN sn);
static void bsp_MotorBrake(MotorSN sn);
static void bsp_PidExec(MotorSN sn , int32_t Encoder, int32_t Target);
static int32_t bsp_PwmLimit(int32_t pwm);
static int32_t myabs(int32_t val);

void bsp_InitPid(MotorSN sn)
{
	if(sn == MotorLeft)
	{
		/*给定KP KI参数*/
		pid[0].kp = 5;
		pid[0].ki = 10;
		/*清除之前的误差*/
		pid[0].bias = 0 ;
		pid[0].lastBias = 0 ;
		pid[0].pwm = 0 ;
		/*目的速度*/
		pid[0].target = 0 ;
		/*上一次目的速度*/
		pid[0].lastTarget = 0 ;
	}
	else if(sn == MotorRight)
	{
		/*给定KP KI参数*/
		pid[1].kp = 5;
		pid[1].ki = 10;
		/*清除之前的误差*/
		pid[1].bias = 0 ;
		pid[1].lastBias = 0 ;
		pid[1].pwm = 0 ;
		/*目的速度*/
		pid[1].target = 0 ;
		/*上一次速度*/
		pid[1].lastTarget = 0 ;
	}
}


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


void bsp_PidSched(void)
{
	bsp_PidExec(MotorLeft, bsp_EncoderGetPulseT(EncoderLeft), pid[0].target);
	bsp_PidExec(MotorRight,bsp_EncoderGetPulseT(EncoderRight),pid[1].target);
}


static void bsp_PidClear(MotorSN sn)
{
	if(sn == MotorLeft)
	{
		/*清除之前的误差*/
		pid[0].bias = 0 ;
		pid[0].lastBias = 0 ;
		pid[0].pwm = 0 ;

		/*上一次目的速度*/
		pid[0].lastTarget = 0 ;
	}
	else if(sn == MotorRight)
	{
		/*清除之前的误差*/
		pid[1].bias = 0 ;
		pid[1].lastBias = 0 ;
		pid[1].pwm = 0 ;

		/*上一次目的速度*/
		pid[1].lastTarget = 0 ;
	}
}


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





static void bsp_PidExec(MotorSN sn , int32_t Encoder, int32_t Target)
{
	/*如果目标速度为0*/
	if(Target == 0)
	{
		bsp_MotorBrake(sn);
		bsp_PidClear(sn);
	}
	
	/*如果目标速度小于0，那么编码器的反馈速度也应该为负，单个霍尔只能计数，没有正负*/
	if(Target < 0 ) 
	{
		Encoder = -Encoder;
	}
	
	
		
	/*计算PWM值，增量式PID*/
	if(sn == MotorLeft)
	{
		if(pid[0].lastTarget == 0) /*从0速启动*/
		{
			pid[0].lastTarget = Target;
		}
		else if(pid[0].lastTarget / (float)Target < 0) /*2次目标速度方向相反*/
		{
			bsp_MotorBrake(sn);
			bsp_PidClear(sn);
			return ;
		}
		
		pid[0].bias = Encoder-Target;                                  
		pid[0].pwm += pid[0].kp*(pid[0].bias-pid[0].lastBias)+pid[0].ki*pid[0].bias;
		pid[0].lastBias=pid[0].bias;
		/*限幅*/
		pid[0].pwm = bsp_PwmLimit(pid[0].pwm);	
		/*设置PWM*/
		bsp_MotorSetPWM(MotorLeft,pid[0].pwm >0 ? Backward : Forward ,myabs(pid[0].pwm));
	}
	else if(sn == MotorRight)
	{
		if(pid[1].lastTarget == 0) /*从0速启动*/
		{
			pid[1].lastTarget = Target;
		}
		else if(pid[1].lastTarget / (float)Target < 0) /*2次目标速度方向相反*/
		{
			bsp_MotorBrake(sn);
			bsp_PidClear(sn);
			return ;
		}
		
		pid[1].bias = Encoder-Target;                                
		pid[1].pwm += pid[1].kp*(pid[1].bias-pid[1].lastBias)+pid[1].ki*pid[1].bias;
		pid[1].lastBias=pid[1].bias;
		/*限幅*/
		pid[1].pwm = bsp_PwmLimit(pid[1].pwm);
		/*设置PWM*/
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
*	函 数 名: myabs
*	功能说明: 求绝对值
*	形    参：无
*	返 回 值: 无
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

