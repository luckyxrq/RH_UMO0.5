#ifndef __ENCODER_H
#define __ENCODER_H

#define ENCODER_COUNT                 2         /*编码器个数*/
#define ENCODER_INTERRUPT_FREQUENCY	  100000    /*10us中断一次*/
#define CALC_T					      30000     /*1000次中断（300MS）计算一次速度*/

#define PulseCntPerCircle	16                                      /*电机转一圈编码器脉冲个数*/
#define CircleCntPerCircle	32                                     /*轮子转一圈，电机实际转的圈数*/
#define Ratio		        (PulseCntPerCircle*CircleCntPerCircle) /*轮子转一圈，编码器的脉冲数*/
#define PI	3.14159F			  /*π*/
#define DIAMETER	70            /*直径70MM*/
#define PERIMETER   (PI*DIAMETER) /*轮子周长*/

typedef enum
{
	EncoderLeft  = 0 ,
	EncoderRight
}EncoderSN;

typedef struct
{
	bool isReadyRising[ENCODER_COUNT];   /*检测到一个低电平了，现在准备检测上升沿*/
	uint32_t risingCount[ENCODER_COUNT]; /*上升沿个数*/
	float speed[ENCODER_COUNT];          /*单位毫米每秒*/
	int32_t odometer[ENCODER_COUNT];
}Encoder;


void bsp_InitEncoder(void);               /*初始化编码器引脚，用于统计脉冲的定时器*/
float bsp_EncoderGetSpeed(EncoderSN sn);  /*返回编码器反馈的速度，单位：毫米每秒*/
int32_t bsp_encoderGetOdometer(MotorSN sn);

#endif
