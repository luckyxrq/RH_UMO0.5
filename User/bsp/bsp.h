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

/*硬件平台版本*/
#define OLD_BOARD      1 /*旧主板*/
#define NEW_BOARD      2 /*新主板*/
#define BOARD_VER      NEW_BOARD

/* 检查是否定义了开发板型号 */
#if !defined (STM32_V4) && !defined (STM32_X2)
	#error "Please define the board model : STM32_X2 or STM32_V4"
#endif

/* 定义 BSP 版本号 */
#define __STM32F1_BSP_VERSION		"1.1"

/* CPU空闲时执行的函数 */
//#define CPU_IDLE()		bsp_Idle()

#define  USE_FreeRTOS      1

#if USE_FreeRTOS == 1
	#include "FreeRTOS.h"
	#include "task.h"
	
	#if 0
	#define DISABLE_INT()    taskENTER_CRITICAL() /* 使用这种模式，串口不能在中断中使用 */
	#define ENABLE_INT()     taskEXIT_CRITICAL()  /* 使用这种模式，串口不能在中断中使用 */
	#else
	#define ENABLE_INT()	__set_PRIMASK(0)	/* 使能全局中断 */
	#define DISABLE_INT()	__set_PRIMASK(1)	/* 禁止全局中断 */
	#endif
	
#else
	/* 开关全局中断的宏 */
	#define ENABLE_INT()	__set_PRIMASK(0)	/* 使能全局中断 */
	#define DISABLE_INT()	__set_PRIMASK(1)	/* 禁止全局中断 */
#endif

/* 这个宏仅用于调试阶段排错 */
#define BSP_Printf		printf
//#define BSP_Printf(...)

#include "stm32f10x.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <stdbool.h>

#ifndef TRUE
	#define TRUE  1
#endif

#ifndef FALSE
	#define FALSE 0
#endif

/*
	EXTI9_5_IRQHandler 的中断服务程序分散在几个独立的 bsp文件中。
	需要整合到 stm32f4xx_it.c 中。

	定义下面行表示EXTI9_5_IRQHandler入口函数集中放到 stm32f4xx_it.c。
*/
#define EXTI9_5_ISR_MOVE_OUT

#define DEBUG_GPS_TO_COM1	/* 打印GPS数据到串口1 */


#define UNUSED(x)    (void)(x) /*防止出现未使用的警告*/

#define DEBUG_EN      1 
#define LOG_EN        1 
#define WARNING_EN    1 


#if DEBUG_EN
#define DEBUG(format, ...) printf (format, ##__VA_ARGS__)
#else
#define DEBUG(format, ...) {}
#endif
	
#if LOG_EN
#define LOG(format, ...) printf (format, ##__VA_ARGS__)
#else
#define LOG(format, ...) {}
#endif
	
#if WARNING_EN
#define WARNING(format, ...) printf (format, ##__VA_ARGS__)
#else
#define WARNING(format, ...) {}
#endif


/* 通过取消注释或者添加注释的方式控制是否包含底层驱动模块 */
#include "main.h"
#include "bsp_uart_fifo.h"
#include "bsp_led.h"
#include "bsp_timer.h"
#include "bsp_key.h"
#include "bsp_dwt.h"

//#include "bsp_msg.h"

//#include "bsp_beep.h"

#include "bsp_tim_pwm.h"

//#include "bsp_cpu_flash.h"
//#include "bsp_sdio_sd.h"
#include "bsp_i2c_gpio.h"
//#include "bsp_eeprom_24xx.h"
//#include "bsp_si4730.h"
//#include "bsp_hmc5883l.h"
//#include "bsp_mpu6050.h"
//#include "bsp_bh1750.h"
//#include "bsp_bmp180.h"
//#include "bsp_wm8978.h"
//#include "bsp_gt811.h"

//#include "bsp_fsmc_sram.h"
//#include "bsp_nand_flash.h"
//#include "bsp_nor_flash.h"

//#include "LCD_RA8875.h"
//#include "LCD_SPFD5420.h"
//#include "LCD_ILI9488.h"
//#include "bsp_ra8875_port.h"
//#include "bsp_tft_lcd.h"

//#include "bsp_touch.h"


//#include "bsp_oled.h"
//#include "bsp_sim800.h"
//#include "bsp_ra8875_flash.h"

//#include "bsp_spi_bus.h"
//#include "bsp_spi_flash.h"
//#include "bsp_tm7705.h"
//#include "bsp_vs1053b.h"
//#include "bsp_tsc2046.h"

//#include "bsp_ds18b20.h"
//#include "bsp_dac8501.h"
//#include "bsp_dht11.h"

#include "bsp_ir_decode.h"
//#include "bsp_ps2.h"

//#include "bsp_modbus.h"
//#include "bsp_rs485_led.h"
#include "bsp_user_lib.h"

//#include "bsp_dac8501.h"
//#include "bsp_dac8562.h"

//#include "bsp_esp8266.h"
//#include "bsp_step_moto.h"


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
#include "bsp_Scope.h"
#include "bsp_Position.h"
#include "bsp_stflash.h"
#include "bsp_param.h"
#include "bsp_OffSiteSW.h"
#include "bsp_CurrentFeedback.h"
#include "bsp_CliffSW.h"
#include "bsp_DustBox.h"
#include "bsp_speaker.h"
#include "bsp_RunControl.h"
#include "bsp_SearchChargePile.h"
#include "bsp_assistJudgeDirection.h"
#include "bsp_StopMode.h"
#include "bsp_Gridmap.h"

/* 提供给其他C文件调用的函数 */
void bsp_Init(void);
void bsp_Idle(void);
void BSP_Tick_Init (void);
void bsp_InitFormAwaken(void);

#endif

/***************************** 安富莱电子 www.armfly.com (END OF FILE) *********************************/
