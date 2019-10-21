#include "includes.h"

/*
**********************************************************************************************************
                                            宏定义
**********************************************************************************************************
*/
#define PAUSE_INTERVAL_RESPONSE_TIME      400

/*
**********************************************************************************************************
                                            函数声明
**********************************************************************************************************
*/
/*决策 整机软件控制流程*/
static void vTaskDecision(void *pvParameters);
/*控制 根据决策控制电机*/
static void vTaskControl(void *pvParameters);
/*感知 获取传感器数据 红外对管、跳崖、碰撞、离地、电机电流、尘盒霍尔、编码器、航向角*/
static void vTaskPerception(void *pvParameters);

static void AppTaskCreate (void);
static void AppObjCreate (void);
void  App_Printf(char *format, ...);
static void bsp_KeySuspend(void);
static void bsp_KeyProc(void);
/*
**********************************************************************************************************
                                            变量声明
**********************************************************************************************************
*/
static TaskHandle_t xHandleTaskDecision      = NULL;
static TaskHandle_t xHandleTaskControl       = NULL;
static TaskHandle_t xHandleTaskPerception    = NULL;

static SemaphoreHandle_t  xMutex = NULL;
static KeyProc keyProc;
/*
*********************************************************************************************************
*	函 数 名: main
*	功能说明: 标准c程序入口。
*	形    参：无
*	返 回 值: 无
*********************************************************************************************************
*/
int main(void)
{
    /* 
      在启动调度前，为了防止初始化STM32外设时有中断服务程序执行，这里禁止全局中断(除了NMI和HardFault)。
      这样做的好处是：
      1. 防止执行的中断服务程序中有FreeRTOS的API函数。
      2. 保证系统正常启动，不受别的中断影响。
      3. 关于是否关闭全局中断，大家根据自己的实际情况设置即可。
      在移植文件port.c中的函数prvStartFirstTask中会重新开启全局中断。通过指令cpsie i开启，__set_PRIMASK(1)
      和cpsie i是等效的。
     */
    __set_PRIMASK(1);  
    
    /* 硬件初始化 */
    bsp_Init(); 
    
    /* 1. 初始化一个定时器中断，精度高于滴答定时器中断，这样才可以获得准确的系统信息 仅供调试目的，实际项
          目中不要使用，因为这个功能比较影响系统实时性。
       2. 为了正确获取FreeRTOS的调试信息，可以考虑将上面的关闭中断指令__set_PRIMASK(1); 注释掉。 
    */
    vSetupSysInfoTest();
    
    /* 创建任务 */
    AppTaskCreate();
    
    /* 创建任务通信机制 */
    AppObjCreate();
    
    /* 启动调度，开始执行任务 */
    vTaskStartScheduler();
    
    /* 
      如果系统正常启动是不会运行到这里的，运行到这里极有可能是用于定时器任务或者空闲任务的
      heap空间不足造成创建失败，此要加大FreeRTOSConfig.h文件中定义的heap大小：
      #define configTOTAL_HEAP_SIZE	      ( ( size_t ) ( 17 * 1024 ) )
    */
    while(1);
}


/*
*********************************************************************************************************
*	函 数 名: vTaskStart
*	功能说明: 启动任务，也就是最高优先级任务，这里用作按键扫描。
*	形    参: pvParameters 是在创建该任务时传递的形参
*	返 回 值: 无
*   优 先 级: 4  
*********************************************************************************************************
*/
static void vTaskDecision(void *pvParameters)      //决策 整机软件控制流程
{
	bsp_SetMotorSpeed(MotorLeft ,  12*1.5);
	bsp_SetMotorSpeed(MotorRight , 12*1.5);
	
    while(1)
    {
        vTaskDelay(50);	
    }
}

/*
*********************************************************************************************************
*	函 数 名: vTaskStart
*	功能说明: 启动任务，也就是最高优先级任务，这里用作按键扫描。
*	形    参: pvParameters 是在创建该任务时传递的形参
*	返 回 值: 无
*   优 先 级: 4  
*********************************************************************************************************
*/
static void vTaskControl(void *pvParameters)       //控制 根据决策控制电机
{

	
    while(1)
    {

        		
#if 1		
        DEBUG("L %d MM/S  ",bsp_MotorGetSpeed(MotorLeft));
        DEBUG("R %d MM/S\r\n",bsp_MotorGetSpeed(MotorRight));
#endif		
		
		bsp_PidSched(); /*10MS调用一次，这里面进行PWM计算，占空比设置，速度（脉冲为单位；MM为单位）计算*/
		
        vTaskDelay(10);
    }
    
}


/*
*********************************************************************************************************
*	函 数 名: vTaskStart
*	功能说明: 感知 获取传感器数据 红外对管、跳崖、碰撞、离地、电机电流、尘盒霍尔、编码器、航向角。
*	形    参: pvParameters 是在创建该任务时传递的形参
*	返 回 值: 无
*   优 先 级: 4  
*********************************************************************************************************
*/
static void vTaskPerception(void *pvParameters)
{

	
    while(1)
    {
		bsp_KeyScan();

		
        vTaskDelay(1);	
    }		
    
}


/*
*********************************************************************************************************
*	函 数 名: AppTaskCreate
*	功能说明: 创建应用任务
*	形    参：无
*	返 回 值: 无
*********************************************************************************************************
*/
static void AppTaskCreate (void)
{
    xTaskCreate( vTaskDecision,     		    /* 任务函数  */
                 "vTaskDecision",   		    /* 任务名    */
                 1024,            		        /* 任务栈大小，单位word，也就是4字节 */
                 NULL,           		        /* 任务参数  */
                 1,              		        /* 任务优先级*/
                 &xHandleTaskDecision );        /* 任务句柄  */
    xTaskCreate( vTaskControl,     		        /* 任务函数  */
                 "vTaskControl",   		        /* 任务名    */
                 1024,            		        /* 任务栈大小，单位word，也就是4字节 */
                 NULL,           		        /* 任务参数  */
                 2,              		        /* 任务优先级*/
                 &xHandleTaskControl );         /* 任务句柄  */	
    xTaskCreate( vTaskPerception,     		    /* 任务函数  */
                 "vTaskPerception",   		    /* 任务名    */
                 1024,            		        /* 任务栈大小，单位word，也就是4字节 */
                 NULL,           		        /* 任务参数  */
                 3,              		        /* 任务优先级*/
                 &xHandleTaskPerception );      /* 任务句柄  */	
    
}

/*
*********************************************************************************************************
*	函 数 名: AppObjCreate
*	功能说明: 创建任务通信机制
*	形    参: 无
*	返 回 值: 无
*********************************************************************************************************
*/
static void AppObjCreate (void)
{
    /* 创建互斥信号量 */
    xMutex = xSemaphoreCreateMutex();
    
    if(xMutex == NULL)
    {
        /* 没有创建成功，用户可以在这里加入创建失败的处理机制 */
    }
}

/*
*********************************************************************************************************
*	函 数 名: App_Printf
*	功能说明: 线程安全的printf方式		  			  
*	形    参: 同printf的参数。
*             在C中，当无法列出传递函数的所有实参的类型和数目时,可以用省略号指定参数表
*	返 回 值: 无
*********************************************************************************************************
*/
void  App_Printf(char *format, ...)
{
    char  buf_str[200 + 1];
    va_list   v_args;
    
    
    va_start(v_args, format);
    (void)vsnprintf((char       *)&buf_str[0],
            (size_t      ) sizeof(buf_str),
            (char const *) format,
            v_args);
    va_end(v_args);
    
    /* 互斥信号量 */
    xSemaphoreTake(xMutex, portMAX_DELAY);
    
    printf("%s", buf_str);
    
    xSemaphoreGive(xMutex);
}

/*
*********************************************************************************************************
*	函 数 名: bsp_KeySuspend
*	功能说明: 不同状态暂停键的效果不同	  			  
*	形    参: 无
*	返 回 值: 无
*********************************************************************************************************
*/
static void bsp_KeySuspend(void)
{
	RunState lastRunState = bsp_GetKeyRunLastState();
	
	if(lastRunState != RUN_STATE_DEFAULT)
	{
		/*记录下短按的时间*/
		bsp_SetLastKeyTick(xTaskGetTickCount());
		
		if(lastRunState == RUN_STATE_CHARGE)
		{
			bsp_StopRunToggleLED();
			bsp_StopVacuum();
		}
		else if(lastRunState == RUN_STATE_CLEAN)
		{
			bsp_SperkerPlay(Song4);
			bsp_StopRunToggleLED();
			bsp_StopVacuum();
		}
		else if(lastRunState == RUN_STATE_SHUTDOWN)
		{
			bsp_LedOn(LED_LOGO_CLEAN);
			bsp_LedOn(LED_LOGO_POWER);
			bsp_LedOn(LED_LOGO_CHARGE);
			bsp_LedOff(LED_COLOR_YELLOW);
			bsp_LedOff(LED_COLOR_GREEN);
			bsp_LedOff(LED_COLOR_RED);
		}
		
		bsp_StopSearchChargePile();
		bsp_StopCliffTest();
		bsp_SetKeyRunLastState(RUN_STATE_DEFAULT);
	}
}


/*
*********************************************************************************************************
*	函 数 名: bsp_SetKeyRunLastState
*	功能说明: 设置上次的按键状态	  			  
*	形    参: 无
*	返 回 值: 无
*********************************************************************************************************
*/
void bsp_SetKeyRunLastState(RunState state)
{
	keyProc.lastRunState = state;
}

/*
*********************************************************************************************************
*	函 数 名: bsp_GetKeyRunLastState
*	功能说明: 获取上次的按键状态	  			  
*	形    参: 无
*	返 回 值: 无
*********************************************************************************************************
*/
RunState bsp_GetKeyRunLastState(void)
{
	return keyProc.lastRunState;
}

/*
*********************************************************************************************************
*	函 数 名: bsp_SetLastKeyTick
*	功能说明: 记录上次按键的时间 			  
*	形    参: 无
*	返 回 值: 无
*********************************************************************************************************
*/
void bsp_SetLastKeyTick(uint32_t tick)
{
	keyProc.lastKeyTick = tick;
}

/*
*********************************************************************************************************
*	函 数 名: bsp_GetLastKeyTick
*	功能说明: 获取上次按键的时间 			  
*	形    参: 无
*	返 回 值: 无
*********************************************************************************************************
*/
uint32_t bsp_GetLastKeyTick(void)
{
	return keyProc.lastKeyTick;
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
	if (ucKeyCode > 0 && bsp_IsSelfCheckingReady())
	{
		/* 有键按下 */
		switch (ucKeyCode)
		{
			case KEY_1_DOWN:
			{
				bsp_KeySuspend();
			}break;
				
			case KEY_2_DOWN:
			{
				bsp_KeySuspend();
			}break;
				
			case KEY_3_DOWN:	
			{
				bsp_KeySuspend();
			}break;
			
			case KEY_1_UP:
			{
				
			}break;
				
			case KEY_2_UP:
			{

			}break;
				
			case KEY_3_UP:
			{

			}break;
			
			case KEY_1_LONG: /*关机*/
			{
				if(xTaskGetTickCount() - bsp_GetLastKeyTick() >= PAUSE_INTERVAL_RESPONSE_TIME)
				{
					bsp_SetKeyRunLastState(RUN_STATE_SHUTDOWN);
					bsp_SperkerPlay(Song2);
					
					bsp_LedOff(LED_LOGO_CLEAN);
					bsp_LedOff(LED_LOGO_POWER);
					bsp_LedOff(LED_LOGO_CHARGE);
					bsp_LedOff(LED_COLOR_YELLOW);
					bsp_LedOff(LED_COLOR_GREEN);
					bsp_LedOff(LED_COLOR_RED);
					
					vTaskDelay(100);	
					while(bsp_SpeakerIsBusy()){}
					bsp_ClearKey();
				}
				
			}break;
			
			case KEY_2_LONG: /*充电*/	
			{
				if(xTaskGetTickCount() - bsp_GetLastKeyTick() >= PAUSE_INTERVAL_RESPONSE_TIME)
				{
					bsp_SetKeyRunLastState(RUN_STATE_CHARGE);
					bsp_SperkerPlay(Song5);
					bsp_StartRunToggleLED(LED_LOGO_CHARGE);
					//bsp_StartCliffTest();
					bsp_StartSearchChargePile();
					
					vTaskDelay(200);	
					while(bsp_SpeakerIsBusy()){}
					bsp_ClearKey();
				}
				
			}break;
			
			case KEY_3_LONG: /*清扫*/
			{
				if(xTaskGetTickCount() - bsp_GetLastKeyTick() >= PAUSE_INTERVAL_RESPONSE_TIME)
				{
					bsp_SetKeyRunLastState(RUN_STATE_CLEAN);
					bsp_SperkerPlay(Song3);
					bsp_StartRunToggleLED(LED_LOGO_CLEAN);
					bsp_StartCliffTest();
					bsp_StartVacuum();
					
					vTaskDelay(200);	
					while(bsp_SpeakerIsBusy()){}
					bsp_ClearKey();
				}
				
			}break;
			
			case KEY_9_DOWN:
			{
				bsp_StopRunToggleLED();
				
				/*复位上一次的按键状态*/
				bsp_SetKeyRunLastState(RUN_STATE_DEFAULT);
				
				/*关闭各种状态机*/
				bsp_StopCliffTest();
				bsp_StopVacuum();
				/*关闭电机*/
				bsp_SetMotorSpeed(MotorLeft, 0);
				bsp_SetMotorSpeed(MotorRight,0);
				bsp_StartEdgewiseRun();
				
			}break;
		}   
	}
}


/***************************** 安富莱电子 www.armfly.com (END OF FILE) *********************************/
