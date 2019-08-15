#include "bsp.h"

static int myabs(int a);


/*
*********************************************************************************************************
*	�� �� ��: myabs
*	����˵��: �����ֵ
*	��    �Σ���
*	�� �� ֵ: ��
*********************************************************************************************************
*/
int32_t Incremental_PI (int32_t Encoder,int32_t Target)
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
    static float Bias,Pwm,Last_bias;
	
	UNUSED(Bias);
	UNUSED(Pwm);
	UNUSED(Last_bias);
	
    Bias=Encoder-Target;                                  //����ƫ��
    Pwm+=20*(Bias-Last_bias)+30*Bias;   //����ʽPI������
    Last_bias=Bias;	                                     //������һ��ƫ�� 
    return Pwm;                                           //�������
}

/*
*********************************************************************************************************
*	�� �� ��: myabs
*	����˵��: �����ֵ
*	��    �Σ���
*	�� �� ֵ: ��
*********************************************************************************************************
*/
int32_t Position_PID (int32_t Encoder,int32_t Target)
{ 	
	/* 
		�������ܣ�λ��ʽPID������
		��ڲ���������������λ����Ϣ��Ŀ��λ��
		����  ֵ�����PWM
		����λ��ʽ��ɢPID��ʽ 
		pwm=Kp*e(k)+Ki*��e(k)+Kd[e��k��-e(k-1)]
		e(k)������ƫ�� 
		e(k-1)������һ�ε�ƫ��  
		��e(k)����e(k)�Լ�֮ǰ��ƫ����ۻ���;����kΪ1,2,,k;
		pwm�������
	*/
	static float Bias,Pwm,Integral_bias,Last_Bias;
	
	UNUSED(Bias);
	UNUSED(Pwm);
	UNUSED(Integral_bias);
	UNUSED(Last_Bias);
	
//	Bias=Encoder-Target;                                  //����ƫ��
//	Integral_bias+=Bias;	                                 //���ƫ��Ļ���
//	Pwm=Position_KP*Bias+Position_KI*Integral_bias+Position_KD*(Bias-Last_Bias);       //λ��ʽPID������
//	Last_Bias=Bias;                                       //������һ��ƫ�� 
	return Pwm;                                           //�������
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

