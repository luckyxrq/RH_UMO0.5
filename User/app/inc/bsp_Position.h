#ifndef __BSP_POSITION_H
#define __BSP_POSITION_H

#define UPDATE_POS_T      50 /*更新坐标的时间周期，单位MM*/

typedef struct
{
	/*上一时刻位置信息*/
	volatile int32_t lastX ;
	volatile int32_t lastY ;
	
	/*当前时刻位置信息*/
	int32_t currentX ;
	int32_t currentY ;
	
	/*上一时刻和当前速度*/
	volatile double lastSpeed;
	volatile double currentSpeed;
	
	/*上一时刻和当前时刻时间戳*/
	volatile uint32_t lastTimestamp;
	volatile uint32_t currentTimestamp;
	
	/*上一时刻和当前时刻角度*/
	volatile double lastOrientation ;
	volatile double currentOrientation ;
	
	/*状态机*/
	volatile uint8_t action ;
	volatile bool isRunning ;
	volatile uint32_t delay ;
}Position;

void bsp_StartUpdatePos(void);
void bsp_StopUpdatePos(void);
void bsp_PositionUpdate(void);
int32_t bsp_GetCurrentPosX(void);
int32_t bsp_GetCurrentPosY(void);

#endif

