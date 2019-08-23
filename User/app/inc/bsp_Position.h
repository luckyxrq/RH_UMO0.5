#ifndef __BSP_POSITION_H
#define __BSP_POSITION_H



typedef struct
{
	/*上一时刻位置信息*/
	volatile double lastX ;
	volatile double lastY ;
	
	/*当前时刻位置信息*/
	volatile double currentX ;
	volatile double currentY ;
	
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

