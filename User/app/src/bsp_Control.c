#include "bsp.h"

typedef struct
{
	volatile float Velocity_KP; /*�ٶ�KP*/
	volatile float Velocity_KI; /*�ٶ�KI*/
	
	float Bias;	     /*ƫ��*/
	float Last_bias; /*�ϴ�ƫ��*/
	float Pwm;       /*ռ�ձ�*/   

}PID_SPEED;



PID_SPEED pid_speed[2];


/*
*********************************************************************************************************
*	�� �� ��: myabs
*	����˵��: �����ֵ
*	��    �Σ���
*	�� �� ֵ: ��
*********************************************************************************************************
*/
int32_t Incremental_PI (MotorSN sn ,int32_t Encoder, int32_t Target)
{ 	
	/* 
		�������ܣ�����PI������
		��ڲ���������������ֵ��Ŀ���ٶ�
		����  ֵ�����PWM
		��������ʽ��ɢPID��ʽ 
		pwm+=Kp[e��k��-e(k-1)]+Ki*e(k)+Kd[e(k)-2e(k-1)+e(k-2)]
		e(k)������ƫ�� 
		e(k-1)������һ�ε�ƫ��  �Դ����� 
		pwm�����������
		�����ǵ��ٶȿ��Ʊջ�ϵͳ���棬ֻʹ��PI����
		pwm+=Kp[e��k��-e(k-1)]+Ki*e(k)
	*/
	
	float Velocity_KP=20,Velocity_KI=30;      //PIDϵ��
	static uint8_t flg = 0 ;
	
	static float Bias,Pwm,Last_bias;
	if(flg==0 && Target<0)
	{
		Bias = 0 ;
		Last_bias = 0 ;
		Pwm = 0 ;
		flg = 1 ;
	}
	
	
	
	
	
    Bias=Encoder-Target;                                  //����ƫ��
    Pwm+=Velocity_KP*(Bias-Last_bias)+Velocity_KI*Bias;   //����ʽPI������
    Last_bias=Bias;	                                     //������һ��ƫ�� 
    return Pwm;  

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

