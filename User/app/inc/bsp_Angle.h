#ifndef __BSP_ANGLE_H
#define __BSP_ANGLE_H

#define RX_BUF_SIZE   13
#define RX_BAUD       115200


typedef struct
{
	volatile bool timeout ;
	volatile uint16_t rxCount;
	volatile uint8_t  buf[RX_BUF_SIZE] ;
	volatile float angle;
}Angle;


typedef struct
{
	volatile int16_t HEAD;
	volatile int16_t RATE;
	volatile int16_t ANGLE;
	volatile int16_t SUM1;
	volatile int16_t RESERVE;
	volatile int8_t SUM2;
	volatile int16_t END;
}AngleFrame;


void bsp_AngleRevByte(uint8_t byte);
float bsp_AngleRead(void);
float bsp_AngleAdd(float angle1 , float angle2);
void bsp_InitAngle(void);
void bsp_AngleRst(void);


#endif

