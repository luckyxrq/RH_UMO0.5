#ifndef __ENCODER_H
#define __ENCODER_H

#define ENCODER_COUNT                 2         /*����������*/
#define ENCODER_INTERRUPT_FREQUENCY	  100000    /*10us�ж�һ��*/
#define CALC_T					      30000     /*1000���жϣ�300MS������һ���ٶ�*/

#define PulseCntPerCircle	16                                      /*���תһȦ�������������*/
#define CircleCntPerCircle	32                                     /*����תһȦ�����ʵ��ת��Ȧ��*/
#define Ratio		        (PulseCntPerCircle*CircleCntPerCircle) /*����תһȦ����������������*/
#define PI	3.14159F			  /*��*/
#define DIAMETER	70            /*ֱ��70MM*/
#define PERIMETER   (PI*DIAMETER) /*�����ܳ�*/

typedef enum
{
	EncoderLeft  = 0 ,
	EncoderRight
}EncoderSN;

typedef struct
{
	bool isReadyRising[ENCODER_COUNT];   /*��⵽һ���͵�ƽ�ˣ�����׼�����������*/
	uint32_t risingCount[ENCODER_COUNT]; /*�����ظ���*/
	float speed[ENCODER_COUNT];          /*��λ����ÿ��*/
	int32_t odometer[ENCODER_COUNT];
}Encoder;


void bsp_InitEncoder(void);               /*��ʼ�����������ţ�����ͳ������Ķ�ʱ��*/
float bsp_EncoderGetSpeed(EncoderSN sn);  /*���ر������������ٶȣ���λ������ÿ��*/
int32_t bsp_encoderGetOdometer(MotorSN sn);

#endif
