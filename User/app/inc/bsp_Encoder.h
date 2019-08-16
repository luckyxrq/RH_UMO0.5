#ifndef __ENCODER_H
#define __ENCODER_H

#define ENCODER_NUM      (2)     /*编码器个数*/
#define RPM_PULSE_RATIO  (1024)  /*轮子转1圈，脉冲个数*/
#define ENCODER_PI		 (3.14F) /*π*/

typedef enum
{
	EncoderLeft = 0 ,
	EncoderRight
}EncoderSN;

typedef struct
{
	uint32_t totalMileage; /*总里程，没有正负，前进后退都加，上升沿下降沿都加*/
	int32_t pulseT;        /*周期性的读取脉冲个数，理论上有正负，但目前扫地机只有1个霍尔，无法硬件层面得到正负*/
}Encoder;

void bsp_InitEncoder(void);
uint32_t bsp_EncoderGetTotalMileage(EncoderSN sn);
int32_t bsp_EncoderGetPulseT(EncoderSN sn);

#endif
