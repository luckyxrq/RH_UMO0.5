#ifndef __BSP_ANGLE_H
#define __BSP_ANGLE_H

#define RX_BUF_SIZE   13
#define RX_BAUD       UART3_BAUD

void bsp_AngleRevByte(uint8_t byte);
float bsp_AngleRead(void);
float bsp_AngleAdd(float angle1 , float angle2);

#endif

