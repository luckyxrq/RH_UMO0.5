#include "bsp.h"


static void bsp_KeyProc(void);

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
	
	
	
	bsp_SetMotorSpeed(MotorLeft,bsp_MotorSpeedMM2Pulse(100));
	bsp_SetMotorSpeed(MotorRight,bsp_MotorSpeedMM2Pulse(100));

//	bsp_IRD_StartWork();
	
	bsp_SetLedState(THREE_WHITE_TOOGLE);
	
	
	/* 主程序大循环 */
	while (1)
	{
	
		bsp_PositionUpdate();
		bsp_LedAppProc();
		bsp_KeyProc();
		bsp_SearchChargePile();
		
		/*下面内容不修改*/
		++tick;
		bsp_DelayMS(1);
	}
}



/*
*********************************************************************************************************
*	函 数 名: bsp_KeyProc
*	功能说明: 按键处理函数	  			  
*	形    参: 无
*	返 回 值: 无
*********************************************************************************************************
*/
static void bsp_KeyProc(void)
{
	uint8_t ucKeyCode;	
	
	ucKeyCode = bsp_GetKey();
	if (ucKeyCode > 0)
	{
		/* 有键按下 */
		switch (ucKeyCode)
		{
			case KEY_DOWN_POWER:
			{
				DEBUG("电源按键按下\r\n");

			}break;
				
			case KEY_DOWN_CHARGE:
			{
				DEBUG("充电按键按下\r\n");

			}break;
				
			case KEY_DOWN_CLEAN:	
			{
				DEBUG("清扫按键按下\r\n");

			}break;

			case KEY_LONG_POWER:
			{
				DEBUG("电源按键长按\r\n");
				
				
			}break;
			
			case KEY_LONG_CHARGE:
			{
				DEBUG("充电按键长按\r\n");
				
				
			}break;
			
			case KEY_LONG_CLEAN:
			{
				DEBUG("清扫按键长按\r\n");
				
				
			}break;
			
			case KEY_9_DOWN:
			{
				
				
			}break;
			
			
			case KEY_10_DOWN:
			{
				DEBUG("重新配网：同时按充电和清扫\r\n");

			}break;
		}   
	}
}


