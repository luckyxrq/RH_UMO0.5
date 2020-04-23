#ifndef __BSP_CURRENTFEEDBACK_H
#define __BSP_CURRENTFEEDBACK_H


typedef enum
{
	eMotorLeft = 0 , /*���ֻ�*/
	eMotorRight,     /*���ֻ�*/
	eVacuum,		 /*������*/
	eRollingBrush,   /*��ˢ*/
	eSideBrush,      /*��ˢ*/
	eBatteryVoltage, /*��ص�ѹ*/
	eBatteryCurrent  /*������*/
}FeedbackSN;


void bsp_InitCurrentFeedbackADC(void);
float bsp_GetFeedbackVoltage(FeedbackSN sn);
void bsp_PrintAllVoltage(void);



#endif


