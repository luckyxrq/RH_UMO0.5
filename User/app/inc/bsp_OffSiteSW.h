#ifndef __BSP_OFFSITESW_H
#define __BSP_OFFSITESW_H

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

