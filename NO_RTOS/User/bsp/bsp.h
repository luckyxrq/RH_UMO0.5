/*
*********************************************************************************************************
*
*	模块名称 : BSP模块
*	文件名称 : bsp.h
*	说    明 : 这是底层驱动模块所有的h文件的汇总文件。 应用程序只需 #include bsp.h 即可，
*			  不需要#include 每个模块的 h 文件
*
*	Copyright (C), 2013-2014, 安富莱电子 www.armfly.com
*
*********************************************************************************************************
*/

#ifndef _BSP_H_
#define _BSP_H

#define STM32_V4
//#define STM32_X2

/* 检查是否定义了开发板型号 */
#if !defined (STM32_V4) && !defined (STM32_X2)
	#error "Please define the board model : STM32_X2 or STM32_V4"
#endif

/* 定义 BSP 版本号 */
#define __STM32F1_BSP_VERSION		"1.1"

/* CPU空闲时执行的函数 */
//#define CPU_IDLE()		bsp_Idle()

/* 开关全局中断的宏 */
#define ENABLE_INT()	__set_PRIMASK(0)	/* 使能全局中断 */
#define DISABLE_INT()	__set_PRIMASK(1)	/* 禁止全局中断 */

/* 这个宏仅用于调试阶段排错 */
#define BSP_Printf		printf
//#define BSP_Printf(...)

#include "stm32f10x.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <stdbool.h>
#include <string.h>
#ifndef TRUE
	#define TRUE  1
#endif

#ifndef FALSE
	#define FALSE 0
#endif

#define UNUSED(x)    (void)(x) /*防止出现未使用的警告*/

#define DEBUG_EN      1 
#define LOG_EN        1 
#define WARNING_EN    1 


#if DEBUG_EN
#define DEBUG(format, ...) printf (format, ##__VA_ARGS__)
#else
#define DEBUG(format, ...)
#endif
	
#if LOG_EN
#define LOG(format, ...) printf (format, ##__VA_ARGS__)
#else
#define LOG(format, ...)
#endif
	
#if WARNING_EN
#define WARNING(format, ...) printf (format, ##__VA_ARGS__)
#else
#define WARNING(format, ...)
#endif

#define M_PI 					    3.14F
#define Deg2Rad(deg) (M_PI * deg / 180.0F)
#define Rad2Deg(rad) (180.0F * rad / M_PI)

#include "DX8_API.h"
#include "bsp_user_lib.h"
#include "bsp_led.h"
#include "bsp_timer.h"
#include "bsp_key.h"
#include "bsp_uart_fifo.h"
#include "bsp_i2c_gpio.h"
#include "bsp_PowerSwitch.h"
#include "bsp_Collision.h"
#include "bsp_Angle.h"
#include "bsp_aw9523b.h"
#include "bsp_DetectAct.h"
#include "bsp_Edgewise.h"
#include "bsp_Encoder.h"
#include "bsp_motor.h"
#include "bsp_Control.h"
#include "bsp_Position.h"
#include "bsp_CliffSW.h"
#include "bsp_CurrentFeedback.h"
#include "bsp_OffSiteSW.h"
#include "bsp_DustBox.h"
#include "bsp_speaker.h"
#include "bsp_ir_decode.h"
#include "bsp_LedApp.h"


/* 提供给其他C文件调用的函数 */
void bsp_Init(void);
void bsp_Idle(void);

#endif

/***************************** 安富莱电子 www.armfly.com (END OF FILE) *********************************/
