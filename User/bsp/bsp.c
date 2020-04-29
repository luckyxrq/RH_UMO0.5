/*
*********************************************************************************************************
*
*	模块名称 : BSP模块(For STM32F1XX)
*	文件名称 : bsp.c
*	版    本 : V1.0
*	说    明 : 这是硬件底层驱动程序模块的主文件。主要提供 bsp_Init()函数供主程序调用。主程序的每个c文件可以在开
*			  头	添加 #include "bsp.h" 来包含所有的外设驱动模块。
*
*	修改记录 :
*		版本号  日期        作者     说明
*		V1.0    2013-03-01 armfly   正式发布
*
*
*	Copyright (C), 2015-2020, 安富莱电子 www.armfly.com
*
*********************************************************************************************************
*/
#include "bsp.h"


extern void vSetupSysInfoTest(void);



/*
*********************************************************************************************************
*	函 数 名: bsp_Init
*	功能说明: 初始化硬件设备。只需要调用一次。该函数配置CPU寄存器和外设的寄存器并初始化一些全局变量。
*			 全局变量。
*	形    参: 无
*	返 回 值: 无
*********************************************************************************************************
*/

void bsp_Init(void)
{
	uint8_t ret;
	
	UNUSED(ret);
	
	/*
		由于ST固件库的启动文件已经执行了CPU系统时钟的初始化，所以不必再次重复配置系统时钟。
		启动文件配置了CPU主时钟频率、内部Flash访问速度和可选的外部SRAM FSMC初始化。

		系统时钟缺省配置为72MHz，如果需要更改，可以修改 system_stm32f10x.c 文件
	*/
	
	/* 保证睡眠模式下调试器继续可以连接使用 */
	DBGMCU_Config(DBGMCU_SLEEP, ENABLE);
	
	/* 优先级分组设置为4，可配置0-15级抢占式优先级，0级子优先级，即不存在子优先级。*/
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);
	bsp_InitDWT();
	
	bsp_InitAngle();         /* 初始化陀螺仪，只是复位引脚初始化，里面没有用到串口打印。在串口初始化前面复位有助于陀螺仪第一帧数据不出错 */
	bsp_InitPinPulse();      /* 初始化脉冲指示引脚，脉冲指示没有使用串口打印，在串口之前初始化 */
	M_CLIFF_PULSE_LOW();     /* 默认电平低*/
	bsp_InitUart(); 	     /* 初始化串口 */
	bsp_InitLed();           /* 初始化LED */
	
	bsp_InitSW();		     /* 开机打开其他外设电源使能引脚 */
	
	bsp_AngleRst();
	bsp_SwOn(SW_5V_EN_CTRL);
	bsp_DelayMS(1000);
	bsp_SwOn(SW_3V3_EN_CTRL);
	bsp_SwOn(SW_IR_POWER);
	bsp_SwOn(SW_MOTOR_POWER);
	bsp_SwOn(SW_VSLAM_POWER);
	bsp_SwOn(SW_WIFI_POWER);
	
	bsp_InitKey();           /* 初始化按键 */
	bsp_InitHardTimer();     /* 初始化硬件定时器 */
	
	bsp_InitEncoder();
	bsp_InitMotor();
	bsp_InitPid(MotorLeft);
	bsp_InitPid(MotorRight);
	
	bsp_InitCollision();     /*初始化碰撞检测，触动开关*/
	
	bsp_InitSpeaker();		 /*初始化扬声器*/
	bsp_InitCurrentFeedbackADC();
	bsp_InitDustBox();
	
	
#if 0
	bsp_InitIWDG();     /*初始化看门狗，一旦开启，就不能停止*/
#endif

#if 0
	/* 初始化IO拓展芯片 */	
	do{
		ret = bsp_InitAW9523B();		
		if(!ret) 
		{
			WARNING("AW9523B Init Error\r\n");
			bsp_DelayMS(100);
		}
	}while(!ret);
#else

//	ret = bsp_InitAW9523B();
//	if(!ret)
//	{
//		WARNING("AW9523B Init Error\r\n");
//	}

#endif
	
	bsp_InitDetectAct();/* IO拓展芯片初始化成功了之后再初始化红外轮询扫描 */	
	
	
	bsp_IRD_StopWork();
	
		/*播放开机音乐*/
#if 1
	if(IsInitFromSleep())
	{
		SetIsInitFromSleep(false) ;
		bsp_SperkerPlay(Song32);
	}
	else
	{
		bsp_SperkerPlay(Song1);
		bsp_PowerOnLedProc();
	}
	
#endif
	
	
	
	wifi_protocol_init();/* 初始化WIFI协议栈 */	
	
	
	
	/*打印初始化完毕，还可以检测是否被看门狗重启了*/
	DEBUG("初始化完毕\r\n");
	
	bsp_ClearKey();
	
	vSetupSysInfoTest();
}



/***************************** 安富莱电子 www.armfly.com (END OF FILE) *********************************/
