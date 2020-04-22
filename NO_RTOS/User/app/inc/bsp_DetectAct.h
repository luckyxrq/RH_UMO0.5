#ifndef __BSP_DETECTACT_H
#define __BSP_DETECTACT_H

typedef struct
{
	uint8_t isRunning;
	uint8_t action;
	uint32_t delay;
	
	uint8_t pinMapIndex;
	uint16_t adcVal;
}DetectAct;

void bsp_InitDetectAct(void);
void bsp_DetectStart(void);
void bsp_DetectStop(void);
void bsp_DetectAct(void);
void bsp_DetectActTest(uint8_t pinMapIndex);

#endif

