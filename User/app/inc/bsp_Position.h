#ifndef __BSP_POSITION_H
#define __BSP_POSITION_H



typedef struct
{
	/*��һʱ��λ����Ϣ*/
	volatile double lastX ;
	volatile double lastY ;
	
	/*��ǰʱ��λ����Ϣ*/
	volatile double currentX ;
	volatile double currentY ;
	
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

