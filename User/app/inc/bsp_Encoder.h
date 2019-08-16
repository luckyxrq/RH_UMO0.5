#ifndef __ENCODER_H
#define __ENCODER_H

#define ENCODER_NUM      (2)     /*����������*/
#define RPM_PULSE_RATIO  (1024)  /*����ת1Ȧ���������*/
#define ENCODER_PI		 (3.14F) /*��*/

typedef enum
{
	EncoderLeft = 0 ,
	EncoderRight
}EncoderSN;

typedef struct
{
	uint32_t totalMileage; /*����̣�û��������ǰ�����˶��ӣ��������½��ض���*/
	int32_t pulseT;        /*�����ԵĶ�ȡ�������������������������Ŀǰɨ�ػ�ֻ��1���������޷�Ӳ������õ�����*/
}Encoder;

void bsp_InitEncoder(void);
uint32_t bsp_EncoderGetTotalMileage(EncoderSN sn);
int32_t bsp_EncoderGetPulseT(EncoderSN sn);

#endif
