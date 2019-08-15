#include "bsp.h"

typedef struct
{
	volatile float Velocity_KP; /*速度KP*/
	volatile float Velocity_KI; /*速度KI*/
	
	float Bias;	     /*偏差*/
	float Last_bias; /*上次偏差*/
	float Pwm;       /*占空比*/   

}PID_SPEED;



PID_SPEED pid_speed[2];


/*
*********************************************************************************************************
*	函 数 名: myabs
*	功能说明: 求绝对值
*	形    参：无
*	返 回 值: 无
*********************************************************************************************************
*/
int32_t Incremental_PI (MotorSN sn ,int32_t Encoder, int32_t Target)
{ 	
	/* 
		函数功能：增量PI控制器
		入口参数：编码器测量值，目标速度
		返回  值：电机PWM
		根据增量式离散PID公式 
		pwm+=Kp[e（k）-e(k-1)]+Ki*e(k)+Kd[e(k)-2e(k-1)+e(k-2)]
		e(k)代表本次偏差 
		e(k-1)代表上一次的偏差  以此类推 
		pwm代表增量输出
		在我们的速度控制闭环系统里面，只使用PI控制
		pwm+=Kp[e（k）-e(k-1)]+Ki*e(k)
	*/
	
	float Velocity_KP=20,Velocity_KI=30;      //PID系数
	static uint8_t flg = 0 ;
	
	static float Bias,Pwm,Last_bias;
	if(flg==0 && Target<0)
	{
		Bias = 0 ;
		Last_bias = 0 ;
		Pwm = 0 ;
		flg = 1 ;
	}
	
	
	
	
	
    Bias=Encoder-Target;                                  //计算偏差
    Pwm+=Velocity_KP*(Bias-Last_bias)+Velocity_KI*Bias;   //增量式PI控制器
    Last_bias=Bias;	                                     //保存上一次偏差 
    return Pwm;  

}


/*
*********************************************************************************************************
*	函 数 名: myabs
*	功能说明: 求绝对值
*	形    参：无
*	返 回 值: 无
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

