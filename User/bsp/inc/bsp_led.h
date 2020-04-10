/*
*********************************************************************************************************
*
*	ģ������ : LEDָʾ������ģ��
*	�ļ����� : bsp_led.h
*	��    �� : V1.0
*	˵    �� : ͷ�ļ�
*
*	Copyright (C), 2013-2014, ���������� www.armfly.com
*
*********************************************************************************************************
*/

#ifndef __BSP_LED_H
#define __BSP_LED_H

/*ʹ��ö��������LED1 LED2 ... ����֪��*/
typedef enum
{
	LED_LOGO_CLEAN   = 4,
	LED_LOGO_POWER   = 5,
	LED_LOGO_CHARGE  = 6,
	LED_COLOR_YELLOW = 1,
	LED_COLOR_GREEN  = 2,
	LED_COLOR_RED    = 3,
	
	
	LED_WIFI_LINK = 10
}LED_SN;

/* ���ⲿ���õĺ������� */
void bsp_InitLed(void);
void bsp_LedOn(LED_SN _no);
void bsp_LedOff(LED_SN _no);
void bsp_LedToggle(LED_SN _no);
uint8_t bsp_IsLedOn(LED_SN _no);

#endif

/***************************** ���������� www.armfly.com (END OF FILE) *********************************/
