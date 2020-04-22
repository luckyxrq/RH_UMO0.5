#include "bsp.h"


/*
*********************************************************************************************************
*	函 数 名: main
*	功能说明: c程序入口
*	形    参：无
*	返 回 值: 错误代码(无需处理)
*********************************************************************************************************
*/
int main(void)
{
	/*
		ST固件库中的启动文件已经执行了 SystemInit() 函数，该函数在 system_stm32f4xx.c 文件，主要功能是
	配置CPU系统的时钟，内部Flash访问时序，配置FSMC用于外部SRAM
	*/
	bsp_Init();		/* 硬件初始化 */
	
//	bsp_SetMotorSpeed(MotorLeft,bsp_MotorSpeedMM2Pulse(20));
//	bsp_SetMotorSpeed(MotorRight,bsp_MotorSpeedMM2Pulse(20));
	
	/* 主程序大循环 */
	while (1)
	{
		bsp_Idle();		/* CPU空闲时执行的函数，在 bsp.c */
	

		bsp_DelayMS(1);
	}
}


