/*
*********************************************************************************************************
*
*	模块名称 : 定时器时基
*	文件名称 : SysInfoTest.c
*	版    本 : V1.0
*	说    明 : 为了获取FreeRTOS的任务信息，需要创建一个定时器，这个定时器的时间基准精度要高于
*              系统时钟节拍。这样得到的任务信息才准确。
*              本文件提供的函数仅用于测试目的，切不可将其用于实际项目，原因有两点：
*               1. FreeRTOS的系统内核没有对总的计数时间做溢出保护。
*               2. 定时器中断是50us进入一次，比较影响系统性能。
*              --------------------------------------------------------------------------------------
*              本文件使用的是32位变量来保存50us一次的计数值，最大支持计数时间：
*              2^32 * 50us / 3600s = 59.6分钟。使用中测试的任务运行计数和任务占用率超过了59.6分钟将不准确。
*
*	修改记录 :
*		版本号    日期        作者     说明
*		V1.0    2015-08-19  Eric2013   首发
*
*	Copyright (C), 2015-2020, 安富莱电子 www.armfly.com
*
*********************************************************************************************************
*/
#include "bsp.h"


/* 定时器频率，50us一次中断 */
#define  timerINTERRUPT_FREQUENCY	100

/* 中断优先级 */
#define  timerHIGHEST_PRIORITY		1

/* 被系统调用 */
volatile uint32_t ulHighFrequencyTimerTicks = 0UL;

/*
*********************************************************************************************************
*	函 数 名: vSetupTimerTest
*	功能说明: 创建定时器
*	形    参: 无
*	返 回 值: 无
*********************************************************************************************************
*/
void vSetupSysInfoTest(void)
{
	bsp_SetTIMforInt(TIM6, timerINTERRUPT_FREQUENCY, timerHIGHEST_PRIORITY, 2);
}

/*
*********************************************************************************************************
*	函 数 名: TIM6_IRQHandler
*	功能说明: TIM6中断服务程序。
*	形    参: 无
*	返 回 值: 无
*********************************************************************************************************
*/
void TIM6_IRQHandler( void )
{
	
	
	if(TIM_GetITStatus(TIM6, TIM_IT_Update) != RESET)
	{
		bsp_DetectIsTouchChargePile();

		if(bsp_GetIsStartKeyProc())
			bsp_KeyScan();
		
		bsp_PidSched(); /*10MS调用一次，这里面进行PWM计算，占空比设置，速度（脉冲为单位；MM为单位）计算*/
		
		ulHighFrequencyTimerTicks++;
		TIM_ClearITPendingBit(TIM6, TIM_IT_Update);
	}
}

/***************************** 安富莱电子 www.armfly.com (END OF FILE) *********************************/
