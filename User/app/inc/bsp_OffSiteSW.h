#ifndef __BSP_OFFSITESW_H
#define __BSP_OFFSITESW_H

#define RCC_ALL_OFFSITE_SW 	(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOE)

#define GPIO_PORT_OFFSITE_SW_L  GPIOA
#define GPIO_PIN_OFFSITE_SW_L   GPIO_Pin_12


#define GPIO_PORT_OFFSITE_SW_R  GPIOE
#define GPIO_PIN_OFFSITE_SW_R   GPIO_Pin_12

typedef enum
{
	OffSiteNone = 0 ,
	OffSiteLeft,
	OffSiteRight,
	OffSiteBoth
}OffSiteState;


void bsp_InitOffSiteSW(void);
OffSiteState bsp_OffSiteGetState(void);
void bsp_OffSiteProc(void);
void bsp_StartOffSiteProc(void);
void bsp_StopOffSiteProc(void);

#endif

