#ifndef __BSP_CURRENTFEEDBACK_H
#define __BSP_CURRENTFEEDBACK_H


typedef enum
{
	eMotorLeft = 0 , /*左轮机*/
	eMotorRight,     /*右轮机*/
	eVacuum,		 /*吸尘器*/
	eRollingBrush,   /*滚刷*/
	eSideBrush,      /*边刷*/
	eBatteryVoltage, /*电池电压*/
	eBatteryCurrent  /*充电电流*/
}FeedbackSN;


void bsp_InitCurrentFeedbackADC(void);
float bsp_GetFeedbackVoltage(FeedbackSN sn);
void bsp_PrintAllVoltage(void);



#endif


