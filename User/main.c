#include "includes.h"

/*
**********************************************************************************************************
                                            宏定义
**********************************************************************************************************
*/
#define PAUSE_INTERVAL_RESPONSE_TIME         1
#define AT_POWER_ON_OPEN_ALL_MODULE_EN       0     /*在开机的时候直接打开所有的电机轮子...，用于调试的时候使用*/

#define DEBUG_CLOSE_CLEAN_MOTOR              0 //1 关闭清扫电机

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

static void vTaskMapping(void *pvParameters);

static void AppTaskCreate (void);
static void AppObjCreate (void);
void  App_Printf(char *format, ...);
static void bsp_KeyProc(void);
static void bsp_KeySuspend(void);
static void bsp_UploadBatteryInfo(void);
/*
**********************************************************************************************************
                                            变量声明
**********************************************************************************************************
*/
static TaskHandle_t xHandleTaskDecision      = NULL;
static TaskHandle_t xHandleTaskControl       = NULL;
static TaskHandle_t xHandleTaskPerception    = NULL;
static TaskHandle_t xHandleTaskMapping       = NULL;


static SemaphoreHandle_t  xMutex = NULL;

bool isSearchCharge = false;
bool isODDStart  = true;

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
static void vTaskMapping(void *pvParameters)
{
	uint32_t count = 0 ;
	
	
    while(1)
    {
     		
#if 1 /*更新地图*/
		
		if(isSearchCharge == false)
		{		
			bsp_GridMapUpdate(bsp_GetCurrentPosX(),bsp_GetCurrentPosY(),bsp_GetCurrentOrientation(),bsp_CollisionScan(),bsp_GetIRSensorData(),bsp_GetCliffSensorData());
		}
#endif
		
		//bsp_UploadMap();
		if(count++ % 100 == 0)
		{	
			bsp_UploadBatteryInfo();
		}
		
		
		//bsp_PrintCollision();
		
        vTaskDelay(100);
    }

}


/*
*********************************************************************************************************
*	函 数 名: vTaskDecision
*	功能说明: 决策 整机软件控制流程
*	形    参: pvParameters 是在创建该任务时传递的形参
*	返 回 值: 无
*   优 先 级: 4  
*********************************************************************************************************
*/
static void vTaskDecision(void *pvParameters)
{
    
    uint32_t count = 0 ;
	vTaskDelay(2000);
    while(1)
    {
        /* 处理按键事件 */
        bsp_KeyProc();
		
        if(count++ % 10 == 0)
        {
			bsp_ChangeWifi2SmartConfigStateProc();
			
			/*下面是打印开关，酌情注释*/
			bsp_WifiStateProc();
        }
		
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
	uint32_t count = 0 ;
	
    while(1)
    {
#if 0
        bsp_IWDG_Feed(); /* 喂狗 */
#endif
        		
#if 0		
        DEBUG("L %d MM/S\r\n",bsp_MotorGetSpeed(MotorLeft));
        DEBUG("R %d MM/S\r\n",bsp_MotorGetSpeed(MotorRight));
#endif		
#if 0
		if(count%100 ==0)
		{
			bsp_CleanZeroYaw();
		}
#endif
		
		if(isSearchCharge == false)
		{	
			bsp_UpdateCleanStrategyB(bsp_GetCurrentPosX(),bsp_GetCurrentPosY(),bsp_GetCurrentOrientation(), bsp_CollisionScan(), \
			bsp_MotorGetPulseVector(MotorLeft), bsp_MotorGetPulseVector(MotorRight), bsp_GetIRSensorData(),bsp_GetCliffSensorData());
			
		}
		if(GetReturnChargeStationStatus())
		{
			bsp_StopUpdateCleanStrategyB();
			ResetReturnChargeStationStatus();
			bsp_PutKey(KEY_LONG_CHARGE);
		}
		
		bsp_PumpProc();
		
		count++;
        vTaskDelay(20);
    }
    
}


bool isNeedRun = false;

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
	uint32_t count = 0 ;
	
    /*开启红外对管轮询扫描*/
    bsp_DetectStart(); 
	
	/*检测主机悬空*/
	bsp_StartOffSiteProc();
	
	/*打开检测尘盒*/
	bsp_StartDustBoxProc();
	
	/*开启位置坐标更新*/
    bsp_StartUpdatePos();
	
	/*开启栅格地图跟新*/
	bsp_StartUpdateGridMap();

	/*空闲休眠模式检测*/
	bsp_StartSleepProc();
	
	/*初始化跳崖传感器*/
	bsp_InitCliffSW();
	
	
	
#if AT_POWER_ON_OPEN_ALL_MODULE_EN /*在开机的时候直接打开所有的电机轮子...，用于调试的时候使用*/
	bsp_StartVacuum(VACUUM_DEFAULT_PER);
	bsp_MotorCleanSetPWM(MotorRollingBrush, CCW , CONSTANT_HIGH_PWM*0.9F);
	bsp_MotorCleanSetPWM(MotorSideBrush, CW , CONSTANT_HIGH_PWM*0.7F);
	bsp_SetMotorSpeed(MotorLeft,bsp_MotorSpeedMM2Pulse(250));
	bsp_SetMotorSpeed(MotorRight,bsp_MotorSpeedMM2Pulse(250));
#endif
	
    while(1)
    {
		bsp_ComAnalysis();
		
#if 0
		if(bsp_IsInitAW9523B_OK())
		{
			bsp_DetectAct();  /*红外对管轮询扫描*/
			bsp_DetectDeal(); /*红外对管扫描结果处理*/
		}
#endif
       
		/*随机策略*/
		bsp_StrategyRandomProc();
		
		/*测试床程序*/
		bsp_FunctionTestUpdate();
		
		/*检测主机悬空*/
		if(!GetCmdStartUpload())
		{
			bsp_OffSiteProc();
		}
		
		/*检测尘盒*/
//		if(!GetCmdStartUpload())
//		{
//			bsp_DustBoxProc();
//		}
		
        /*寻找充电桩*/
		bsp_SearchChargePile();
		/*沿边行走*/
		bsp_EdgewiseRun();
        /*更新坐标*/
        bsp_PositionUpdate();
		bsp_LedAppProc();
		wifi_uart_service();
		
		/*更新跳崖传感器信息*/
		bsp_GetCliffStates();
		
		/*自检程序*/
		bsp_SelfCheckProc();
		
		/*空闲休眠模式检测*/
		bsp_SleepProc();
		
		/*上传开关和时间间隔同时限制*/
		if(GetCmdStartUpload() && count % 50 == 0)
		{
			bsp_SendReportFrameWithCRC16();
		}
		
		count++;
        vTaskDelay(5);	
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
	
	xTaskCreate( vTaskMapping,     		        /* 任务函数  */
                 "vTaskMapping",   		        /* 任务名    */
                 1024*2,            		    /* 任务栈大小，单位word，也就是4字节 */
                 NULL,           		        /* 任务参数  */
                 1,              		        /* 任务优先级*/
                 &xHandleTaskMapping );         /* 任务句柄  */
    xTaskCreate( vTaskDecision,     		    /* 任务函数  */
                 "vTaskDecision",   		    /* 任务名    */
                 512,            		        /* 任务栈大小，单位word，也就是4字节 */
                 NULL,           		        /* 任务参数  */
                 2,              		        /* 任务优先级*/
                 &xHandleTaskDecision );        /* 任务句柄  */
    xTaskCreate( vTaskControl,     		        /* 任务函数  */
                 "vTaskControl",   		        /* 任务名    */
                 1024,            		        /* 任务栈大小，单位word，也就是4字节 */
                 NULL,           		        /* 任务参数  */
                 3,              		        /* 任务优先级*/
                 &xHandleTaskControl );         /* 任务句柄  */	
    xTaskCreate( vTaskPerception,     		    /* 任务函数  */
                 "vTaskPerception",   		    /* 任务名    */
                 1024,            		        /* 任务栈大小，单位word，也就是4字节 */
                 NULL,           		        /* 任务参数  */
                 4,              		        /* 任务优先级*/
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
    
    DEBUG("%s", buf_str);
    
    xSemaphoreGive(xMutex);
}

/*
*********************************************************************************************************
*	函 数 名: bsp_UploadBatteryInfo
*	功能说明: 无		  			  
*	形    参: 上传电池信息到APP
*	返 回 值: 无
*********************************************************************************************************
*/
static void bsp_UploadBatteryInfo(void)
{
	float battery_adc_value = 0;
	uint32_t battery_precent = 0 ;

	battery_adc_value  = bsp_GetFeedbackVoltage(eBatteryVoltage);
	battery_adc_value  = ((battery_adc_value * 430.0f / 66.5f) + battery_adc_value + 0.2F);
	battery_precent = ((battery_adc_value - 11.9f) / 4.8f)*100; 
	mcu_dp_value_update(DPID_RESIDUAL_ELECTRICITY, battery_precent);
	mcu_dp_value_update(DPID_CLEAN_TIME, RealWorkTime/1000/60);
}

static void bsp_StopAllMotor(void)
{
	bsp_SetMotorSpeed(MotorLeft, 0);
	bsp_SetMotorSpeed(MotorRight,0);
	bsp_MotorCleanSetPWM(MotorSideBrush, CW , 0);
	bsp_MotorCleanSetPWM(MotorRollingBrush, CW , 0);
	bsp_StopVacuum();
}


typedef enum
{
	eKEY_NONE = 0,
	eKEY_CLEAN
}KEY_STATE;

static KEY_STATE key_state = eKEY_NONE;

void bsp_SetLastKeyState(KEY_STATE state)
{
	key_state = state;
}

KEY_STATE bsp_GetLastKeyState(void)
{
	return key_state;
}


void bsp_CloseAllStateRun(void)
{
	bsp_StopSearchChargePile();
	bsp_StopCliffTest();
	bsp_StopUpdateCleanStrategyB();
	bsp_StopStrategyRandom();
	bsp_StopEdgewiseRun();
}

void bsp_OffsiteSuspend(void)
{
	/*灯光亮3颗白色灯*/
	bsp_OpenThreeWhileLed();
	bsp_SetLedState(LED_DEFAULT_STATE);
	
	/*关闭所有状态机*/
	bsp_CloseAllStateRun();
	
	/*关闭所有电机*/
	bsp_StopAllMotor();

	
	/*设置上一次按键值*/
	bsp_SetLastKeyState(eKEY_NONE);
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
	/*灯光亮3颗白色灯*/
	bsp_OpenThreeWhileLed();
	bsp_SetLedState(LED_DEFAULT_STATE);
	
	/*关闭所有电机*/
	bsp_StopAllMotor();
	
	/*关闭水泵*/
	bsp_StopPump();
	
	/*关闭所有状态机*/
	bsp_CloseAllStateRun();

	/*上一次是清扫，本次就播放暂停清扫*/
	if(bsp_GetLastKeyState() == eKEY_CLEAN)
	{
		bsp_SperkerPlay(Song4);
	}
	
	/*设置上一次按键值*/
	bsp_SetLastKeyState(eKEY_NONE);
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
				DEBUG("KEY_DOWN_POWER\r\n");
				bsp_KeySuspend();
				isNeedRun = true;
			}break;
				
			case KEY_DOWN_CHARGE:
			{
				DEBUG("KEY_DOWN_CHARGE\r\n");
				bsp_KeySuspend();
			}break;
				
			case KEY_DOWN_CLEAN:	
			{
				DEBUG("KEY_DOWN_CLEAN\r\n");
				bsp_KeySuspend();
			}break;
			

			
			case KEY_LONG_POWER: /*关机*/
			{
				DEBUG("电源按键长按\r\n");
				
				/*灯光亮3颗白色灯*/
				bsp_CloseAllLed();
				bsp_SetLedState(LED_DEFAULT_STATE);
				
				/*关闭所有电机*/
				bsp_StopAllMotor();
				
				/*关闭所有状态机*/
				bsp_CloseAllStateRun();

				/*设置上一次按键值*/
				bsp_SetLastKeyState(eKEY_NONE);
				/*进入休眠模式*/
				bsp_SperkerPlay(Song31);
				vTaskDelay(10);	
				while(bsp_SpeakerIsBusy()){}
				
				bsp_ClearKey();
				bsp_EnterStopMODE();
			}break;
			
			case KEY_LONG_CHARGE: /*充电*/	
			{
				DEBUG("充电按键长按\r\n");

				/*首先判断是否主机悬空*/
				if(!GetCmdStartUpload() && bsp_OffSiteGetState() != OffSiteNone) /*前提不处于上传状态*/
				{
					bsp_SperkerPlay(Song16);
					return;
				}
				
				/*首先判断尘盒*/
				if(!GetCmdStartUpload() && bsp_DustBoxGetState() == DustBoxOutside) /*前提不处于上传状态*/
				{
					bsp_SperkerPlay(Song9);
					return;
				}
				
				bsp_SperkerPlay(Song5);
				bsp_StartSearchChargePile();
				bsp_MotorCleanSetPWM(MotorSideBrush, CCW , CONSTANT_HIGH_PWM*0.6F);
				/*设置上一次按键值*/
				bsp_SetLastKeyState(eKEY_NONE);
				/*设置LED状态*/
				bsp_SetLedState(AT_SEARCH_CHARGE);
				isSearchCharge = true;
				bsp_ClearKey();
			}break;
			
			case KEY_LONG_CLEAN: /*清扫*/
			{
				DEBUG("清扫按键长按\r\n");
				
				/*首先判断是否主机悬空*/
				if(!GetCmdStartUpload() && bsp_OffSiteGetState() == OffSiteBoth)   /*前提不处于上传状态*/
				{
					bsp_SperkerPlay(Song16);
					return;
				}

				bsp_MotorCleanSetPWM(MotorRollingBrush, CCW , CONSTANT_HIGH_PWM*0.8F);
				bsp_StartPump();
				bsp_StartStrategyRandom();
				
				
				/*设置上一次按键值*/
				bsp_SetLastKeyState(eKEY_CLEAN);
				/*设置LED状态*/
				bsp_SetLedState(AT_CLEAN);
				isSearchCharge = false;
				bsp_ClearKey();
				
			}break;
			
			case KEY_9_DOWN:
			{
				DEBUG("重新配网：同时按充电和清扫\r\n");
				bsp_SperkerPlay(Song29);
				bsp_StartChangeWifi2SmartConfigState();
				bsp_SetLedState(AT_LINK);
			}break;
			
			
			case KEY_10_LONG:
			{
				bsp_StartFunctionTest();
			}break;
			
			
			case KEY_WIFI_OPEN_CLEAN_CAR:
			{
				bsp_SperkerPlay(Song1);
			}break;
			
			case KEY_WIFI_CLOSE_CLEAN_CAR:
			{
				bsp_SperkerPlay(Song2);
				
				/*灯光亮3颗白色灯*/
				bsp_OpenThreeWhileLed();
				bsp_SetLedState(LED_DEFAULT_STATE);
				
				/*关闭所有电机*/
				bsp_StopAllMotor();
				
				/*关闭所有状态机*/
				bsp_CloseAllStateRun();
				
				/*设置上一次按键值*/
				bsp_SetLastKeyState(eKEY_NONE);
			}break;
			
			case KEY_WIFI_DIR_FRONT:
			{
				bsp_KeySuspend();
				bsp_SetMotorSpeed(MotorLeft, bsp_MotorSpeedMM2Pulse(250));
				bsp_SetMotorSpeed(MotorRight,bsp_MotorSpeedMM2Pulse(250));
				vTaskDelay(1000);	
				bsp_SetMotorSpeed(MotorLeft, bsp_MotorSpeedMM2Pulse(0));
				bsp_SetMotorSpeed(MotorRight,bsp_MotorSpeedMM2Pulse(0));
			}break;
			
			case KEY_WIFI_DIR_BACK:
			{
				bsp_KeySuspend();
				bsp_SetMotorSpeed(MotorLeft, bsp_MotorSpeedMM2Pulse(-250));
				bsp_SetMotorSpeed(MotorRight,bsp_MotorSpeedMM2Pulse(-250));
				vTaskDelay(1000);	
				bsp_SetMotorSpeed(MotorLeft, bsp_MotorSpeedMM2Pulse(0));
				bsp_SetMotorSpeed(MotorRight,bsp_MotorSpeedMM2Pulse(0));
			}break;
			
			case KEY_WIFI_DIR_LEFT:
			{
				bsp_KeySuspend();
				bsp_SetMotorSpeed(MotorLeft, bsp_MotorSpeedMM2Pulse(-150));
				bsp_SetMotorSpeed(MotorRight,bsp_MotorSpeedMM2Pulse(+150));
				vTaskDelay(1500);	
				bsp_SetMotorSpeed(MotorLeft, bsp_MotorSpeedMM2Pulse(0));
				bsp_SetMotorSpeed(MotorRight,bsp_MotorSpeedMM2Pulse(0));
			}break;
			
			case KEY_WIFI_DIR_RIGHT:
			{
				bsp_KeySuspend();
				bsp_SetMotorSpeed(MotorLeft, bsp_MotorSpeedMM2Pulse(+150));
				bsp_SetMotorSpeed(MotorRight,bsp_MotorSpeedMM2Pulse(-150));
				vTaskDelay(1500);	
				bsp_SetMotorSpeed(MotorLeft, bsp_MotorSpeedMM2Pulse(0));
				bsp_SetMotorSpeed(MotorRight,bsp_MotorSpeedMM2Pulse(0));
			}break;
			
			case KEY_WIFI_EDGE:
			{
				bsp_KeySuspend();
				bsp_SperkerPlay(Song34);
				bsp_StartEdgewiseRun();
			}break;
		}   
	}
}





/***************************** 安富莱电子 www.armfly.com (END OF FILE) *********************************/
