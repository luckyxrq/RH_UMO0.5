#ifndef __BSP_POSITION_H
#define __BSP_POSITION_H

#define UPDATE_POS_T      50 /*���������ʱ�����ڣ���λMM*/

typedef struct
{
	/*��һʱ��λ����Ϣ*/
	volatile int32_t lastX ;
	volatile int32_t lastY ;
	
	/*��ǰʱ��λ����Ϣ*/
	int32_t currentX ;
	int32_t currentY ;
	
	/*��һʱ�̺͵�ǰ�ٶ�*/
	volatile double lastSpeed;
	volatile double currentSpeed;
	
	/*��һʱ�̺͵�ǰʱ��ʱ���*/
	volatile uint32_t lastTimestamp;
	volatile uint32_t currentTimestamp;
	
	/*��һʱ�̺͵�ǰʱ�̽Ƕ�*/
	volatile double lastOrientation ;
	volatile double currentOrientation ;
	
	/*״̬��*/
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

