#ifndef __BSP_REMOTE_H
#define __BSP_REMOTE_H

#define CH_CNT    4

typedef enum
{
    CapCH1 = 0 ,
    CapCH2 = 1 ,
    CapCH3 = 2 ,
    CapCH4 = 3 ,
}CapCH;

typedef struct
{
    volatile uint8_t  capState[CH_CNT];  //输入捕获状态		    				
    volatile uint16_t capValue[CH_CNT];	 //输入捕获值
	volatile uint16_t capStart[CH_CNT];	 //输入捕获触发时定时器的值
}ChargingPile;

uint32_t bsp_GetCapCnt(CapCH capCH);
void bsp_InitChargingPile(void);
void bsp_SearchChargingPileAct(void);
void bsp_StartSearchChargingPile(void);
void bsp_StopSearchChargingPile(void);

#endif
