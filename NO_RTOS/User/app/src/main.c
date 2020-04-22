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
	uint32_t tick = 0 ;
	/*
		ST固件库中的启动文件已经执行了 SystemInit() 函数，该函数在 system_stm32f4xx.c 文件，主要功能是
	配置CPU系统的时钟，内部Flash访问时序，配置FSMC用于外部SRAM
	*/
	bsp_Init();		/* 硬件初始化 */
	
	
	
//	bsp_SetMotorSpeed(MotorLeft,bsp_MotorSpeedMM2Pulse(20));
//	bsp_SetMotorSpeed(MotorRight,bsp_MotorSpeedMM2Pulse(20));

//	bsp_IRD_StartWork();
	
	bsp_LedOn(LED_LOGO_CLEAN);
	bsp_LedOn(LED_LOGO_POWER);
	bsp_LedOn(LED_LOGO_CHARGE);
	bsp_LedOff(LED_COLOR_YELLOW);
	bsp_LedOff(LED_COLOR_GREEN);
	bsp_LedOff(LED_COLOR_RED);
	
	
	/* 主程序大循环 */
	while (1)
	{
	
		if(tick % 500 == 0)
		{
			bsp_PrintIR_Rev();
		}
		
		if(tick % 10 == 0)
		{
			bsp_PositionUpdate();
		}
		
		/*下面内容不修改*/
		++tick;
		bsp_DelayMS(1);
	}
}


