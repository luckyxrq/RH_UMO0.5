#ifndef __BSP_DC_MOTOR_H
#define __BSP_DC_MOTOR_H


#define MOTOR_NUM    2       /*�������*/
#define MAXPWM       10000   /*���PWMֵ*/
#define DC_PWM_T     30000   /*DC�������оƬռ�ձ�����*/


typedef enum
{
	MotorLeft = 0 ,/*��ߵ�����*/
	MotorRight     /*�ұߵ�����*/ 
}MotorSN;

typedef enum
{
	Forward  = 0 ,/*ǰ��*/
	Backward      /*����*/ 
}MotorDir;

typedef struct
{
	volatile float target; //Ŀ��ֵ
	
	volatile float kp;  //����
	volatile float ki;  //����
	volatile float kd;  //΢��
	
	volatile float bias;     //�������
	volatile float lastBias; //�ϴ����
	volatile float biasSum;  //�ۻ����
	
	volatile float pwm;     //�������PWMֵ
	volatile float pwmMax;  //PWM�ܹ�ȡ�����ֵ
	
	volatile float kiLimit; //��������
	volatile float fitK;    //��ϲ���k  
	volatile float fitD;    //��ϲ���d
}PID;

void bsp_InitDC_Motor(void);      /* ��ʼ��ֱ�������PWM��ʱ�� */
void bsp_MotorBrake(MotorSN sn);  /* ��ͣ */
void bsp_MotorCoast(MotorSN sn);  /* ����ֹͣ */
void bsp_SetMotorPWM(MotorSN sn, MotorDir dir, uint16_t pwm);  /* ֱ��ʹ��ռ�ձȿ��ƣ�6000����������ٶȣ���С�Ͳ�ת�� */
void bsp_PidControlAct(void);
void bsp_SetMotorTargetSpeed(MotorSN sn, float targetSpeed);
MotorDir bsp_MotorGetDir(MotorSN sn);
void bsp_MotorRunR(void);
void bsp_MotorRunL(void);
void bsp_MotorRun(void);
#endif

