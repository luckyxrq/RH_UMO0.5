/*
*********************************************************************************************************
*
*	ģ������ : ��ʱ��ģ��
*	�ļ����� : bsp_timer.h
*	��    �� : V1.3
*	˵    �� : ͷ�ļ�
*
*	Copyright (C), 2015-2016, ���������� www.armfly.com
*
*********************************************************************************************************
*/

#ifndef __BSP_TIMER_H
#define __BSP_TIMER_H



/* �ṩ������C�ļ����õĺ��� */
void bsp_InitTimer(void);
void bsp_DelayMS(uint32_t n);
void bsp_DelayUS(uint32_t n);

uint32_t bsp_GetRunTime(void);


void bsp_InitHardTimer(void);
void bsp_StartHardTimer(uint8_t _CC, uint32_t _uiTimeOut, void * _pCallBack);

#endif

/***************************** ���������� www.armfly.com (END OF FILE) *********************************/
