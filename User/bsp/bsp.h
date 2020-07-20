/*
*********************************************************************************************************
*
*	ģ������ : BSPģ��
*	�ļ����� : bsp.h
*	˵    �� : ���ǵײ�����ģ�����е�h�ļ��Ļ����ļ��� Ӧ�ó���ֻ�� #include bsp.h ���ɣ�
*			  ����Ҫ#include ÿ��ģ��� h �ļ�
*
*	Copyright (C), 2013-2014, ���������� www.armfly.com
*
*********************************************************************************************************
*/

#ifndef __BSP_H
#define __BSP_H

#define STM32_V4
//#define STM32_X2

/*Ӳ��ƽ̨�汾*/
#define OLD_BOARD      1 /*������*/
#define NEW_BOARD      2 /*������*/
#define BOARD_VER      NEW_BOARD




/* ����Ƿ����˿������ͺ� */
#if !defined (STM32_V4) && !defined (STM32_X2)
	#error "Please define the board model : STM32_X2 or STM32_V4"
#endif

/* ���� BSP �汾�� */
#define __STM32F1_BSP_VERSION		"1.1"

/* CPU����ʱִ�еĺ��� */
//#define CPU_IDLE()		bsp_Idle()

#define  USE_FreeRTOS      1

#if USE_FreeRTOS == 1
	#include "FreeRTOS.h"
	#include "task.h"
	
	#if 0
	#define DISABLE_INT()    taskENTER_CRITICAL() /* ʹ������ģʽ�����ڲ������ж���ʹ�� */
	#define ENABLE_INT()     taskEXIT_CRITICAL()  /* ʹ������ģʽ�����ڲ������ж���ʹ�� */
	#else
	#define ENABLE_INT()	__set_PRIMASK(0)	/* ʹ��ȫ���ж� */
	#define DISABLE_INT()	__set_PRIMASK(1)	/* ��ֹȫ���ж� */
	#endif
	
#else
	/* ����ȫ���жϵĺ� */
	#define ENABLE_INT()	__set_PRIMASK(0)	/* ʹ��ȫ���ж� */
	#define DISABLE_INT()	__set_PRIMASK(1)	/* ��ֹȫ���ж� */
#endif

/* ���������ڵ��Խ׶��Ŵ� */
#define BSP_Printf		printf
//#define BSP_Printf(...)


#include "stm32f10x.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <stdbool.h>
#include "SEGGER_RTT.h"

#ifndef TRUE
	#define TRUE  1
#endif

#ifndef FALSE
	#define FALSE 0
#endif

#define UINT32_T_MAX      0xFFFFFFFF

#define ABS(val) ( (val)>0?(val):-(val) )

/*
	EXTI9_5_IRQHandler ���жϷ�������ɢ�ڼ��������� bsp�ļ��С�
	��Ҫ���ϵ� stm32f4xx_it.c �С�

	���������б�ʾEXTI9_5_IRQHandler��ں������зŵ� stm32f4xx_it.c��
*/
#define EXTI9_5_ISR_MOVE_OUT

#define DEBUG_GPS_TO_COM1	/* ��ӡGPS���ݵ�����1 */


#define UNUSED(x)    (void)(x) /*��ֹ����δʹ�õľ���*/

#define DEBUG_EN            1 
#define LOG_EN              0 
#define WARNING_EN          0 
#define STRATEGY_DEBUG      0
#define WIFI_DEBUG_EN       0
#define RTT_EN              1


#if DEBUG_EN
#define DEBUG(format, ...) printf (format, ##__VA_ARGS__)
#else
#define DEBUG(format, ...) {}
#endif
	
#if LOG_EN
#define LOG(format, ...) printf (format, ##__VA_ARGS__)
#else
#define LOG(format, ...)   {}
#endif
	
#if WARNING_EN
#define WARNING(format, ...) printf (format, ##__VA_ARGS__)
#else
#define WARNING(format, ...) {}
#endif

#if WIFI_DEBUG_EN
#define WIFI_DEBUG(format, ...) printf (format, ##__VA_ARGS__)
#else
#define WIFI_DEBUG(format, ...) {}
#endif
	
	
#if RTT_EN
#define RTT(format, ...) SEGGER_RTT_printf (0,format, ##__VA_ARGS__)
#else
#define RTT(format, ...) {}
#endif	
	
	
	
#if STRATEGY_DEBUG	
#define gridmap_debug(format, ...) printf (format, ##__VA_ARGS__)
#define main_debug(format, ...)    printf (format, ##__VA_ARGS__)
#define strategy_debug(format, ...)     printf (format, ##__VA_ARGS__)
#else
#define gridmap_debug(format, ...)  {}
#define main_debug(format, ...)  {}
#define strategy_debug(format, ...)  {}
#endif


/* ͨ��ȡ��ע�ͻ������ע�͵ķ�ʽ�����Ƿ�����ײ�����ģ�� */
#include "main.h"
#include "bsp_uart_fifo.h"
#include "bsp_led.h"
#include "bsp_timer.h"
#include "bsp_key.h"
#include "bsp_dwt.h"
#include "bsp_tim_pwm.h"
#include "bsp_i2c_gpio.h"
#include "bsp_ir_decode.h"
#include "bsp_user_lib.h"
#include "bsp_PowerSwitch.h"
#include "bsp_Collision.h"
#include "bsp_Angle.h"
#include "bsp_Action.h"
#include "bsp_aw9523b.h"
#include "bsp_DetectAct.h"
#include "bsp_PulseMark.h"
#include "bsp_IWDG.h"
#include "bsp_Edgewise.h"
#include "bsp_communication.h"
#include "bsp_Encoder.h"
#include "bsp_motor.h"
#include "bsp_Control.h"
#include "bsp_Position.h"
#include "bsp_stflash.h"
#include "bsp_param.h"
#include "bsp_OffSiteSW.h"
#include "bsp_CurrentFeedback.h"
#include "bsp_CliffSW.h"
#include "bsp_DustBox.h"
#include "bsp_speaker.h"
#include "bsp_RunControl.h"
#include "bsp_assistJudgeDirection.h"
#include "bsp_StopMode.h"
#include "bsp_Gridmap.h"
#include "bsp_CleanStrategyB.h"
#include "bsp_Gridmap.h"
#include "DX8_API.h"
#include "dx8_engineer.h"
#include "wifi.h"
#include "bsp_wifi_app.h"
#include "bsp_passwd.h"
#include "bsp_UploadMap.h"
#include "bsp_CleanStrategyRandom.h"
#include "bsp_SearchChargePile.h"
#include "bsp_searchpilesubproc.h"
#include "bsp_communication_bot3.h"
#include "bsp_selfcheck.h"
#include "bsp_functiontest.h"
#include "bsp_bed.h"
#include "bsp_electrolyticwater.h"
#include "bsp_pump.h"

typedef enum
{
	CLEAN_CAR_NORMAL = 0 ,            /*��ͨ��������ģʽ*/
	CLEAN_CAR_MAIN_BOARD_UPLOAD_DATA, /*���幤װ�ϱ�����ģʽ*/
}CleanCarRunMode;


/* �ṩ������C�ļ����õĺ��� */
void bsp_Init(void);
void bsp_Idle(void);
void BSP_Tick_Init (void);
void bsp_InitFormAwaken(void);
void bsp_CloseAllStateRun(void);
void bsp_SetAppRunMode(CleanCarRunMode mode);
CleanCarRunMode bsp_GetAppRunMode(void);

/* ����ͷ�ļ������ĺ��װ */
#define REAL_ANGLE()      (bsp_AngleReadRaw()*0.01F)


#endif

/***************************** ���������� www.armfly.com (END OF FILE) *********************************/
