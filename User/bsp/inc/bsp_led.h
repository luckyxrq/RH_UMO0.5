/*
*********************************************************************************************************
*
*	模块名称 : LED指示灯驱动模块
*	文件名称 : bsp_led.h
*	版    本 : V1.0
*	说    明 : 头文件
*
*	Copyright (C), 2013-2014, 安富莱电子 www.armfly.com
*
*********************************************************************************************************
*/

#ifndef __BSP_LED_H
#define __BSP_LED_H

/*使用枚举来代替LED1 LED2 ... 见名知意*/
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

/* 供外部调用的函数声明 */
void bsp_InitLed(void);
void bsp_LedOn(LED_SN _no);
void bsp_LedOff(LED_SN _no);
void bsp_LedToggle(LED_SN _no);
uint8_t bsp_IsLedOn(LED_SN _no);

#endif

/***************************** 安富莱电子 www.armfly.com (END OF FILE) *********************************/
