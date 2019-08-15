#include "bsp.h"

static int myabs(int a);


/*
*********************************************************************************************************
*	函 数 名: myabs
*	功能说明: 求绝对值
*	形    参：无
*	返 回 值: 无
*********************************************************************************************************
*/
int32_t Incremental_PI (int32_t Encoder,int32_t Target)
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
    static float Bias,Pwm,Last_bias;
	
	UNUSED(Bias);
	UNUSED(Pwm);
	UNUSED(Last_bias);
	
    Bias=Encoder-Target;                                  //计算偏差
    Pwm+=20*(Bias-Last_bias)+30*Bias;   //增量式PI控制器
    Last_bias=Bias;	                                     //保存上一次偏差 
    return Pwm;                                           //增量输出
}

/*
*********************************************************************************************************
*	函 数 名: myabs
*	功能说明: 求绝对值
*	形    参：无
*	返 回 值: 无
*********************************************************************************************************
*/
int32_t Position_PID (int32_t Encoder,int32_t Target)
{ 	
	/* 
		函数功能：位置式PID控制器
		入口参数：编码器测量位置信息，目标位置
		返回  值：电机PWM
		根据位置式离散PID公式 
		pwm=Kp*e(k)+Ki*∑e(k)+Kd[e（k）-e(k-1)]
		e(k)代表本次偏差 
		e(k-1)代表上一次的偏差  
		∑e(k)代表e(k)以及之前的偏差的累积和;其中k为1,2,,k;
		pwm代表输出
	*/
	static float Bias,Pwm,Integral_bias,Last_Bias;
	
	UNUSED(Bias);
	UNUSED(Pwm);
	UNUSED(Integral_bias);
	UNUSED(Last_Bias);
	
//	Bias=Encoder-Target;                                  //计算偏差
//	Integral_bias+=Bias;	                                 //求出偏差的积分
//	Pwm=Position_KP*Bias+Position_KI*Integral_bias+Position_KD*(Bias-Last_Bias);       //位置式PID控制器
//	Last_Bias=Bias;                                       //保存上一次偏差 
	return Pwm;                                           //增量输出
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

