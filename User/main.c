#include "includes.h"

/*
**********************************************************************************************************
                                            宏定义
**********************************************************************************************************
*/
#define PAUSE_INTERVAL_RESPONSE_TIME         1
#define AT_POWER_ON_OPEN_ALL_MODULE_EN       0     /*在开机的时候直接打开所有的电机轮子...，用于调试的时候使用*/
#define DEBUG_CLOSE_CLEAN_MOTOR              0 //1 关闭清扫电机
#define DEBUG_STRATEGY_SHOW                  0
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
static void vTaskKey(void *pvParameters);

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
static TaskHandle_t xHandleTaskKey           = NULL;
static TaskHandle_t xHandleTaskMappingUpload = NULL;

static SemaphoreHandle_t  xMutex = NULL;

bool isSearchCharge = false;
bool isODDStart  = true;

/*用于任务计数，每次切换过去时都会自增，用于判断任务是否死机*/
static uint32_t cnt_task_1 = 0;
static uint32_t cnt_task_2 = 0;
static uint32_t cnt_task_3 = 0;
static uint32_t cnt_task_4 = 0;
static uint32_t cnt_task_5 = 0;
static uint32_t cnt_task_6 = 0;


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

unsigned int vTaskMapping_cnt = 0;
unsigned int vTaskMappingUpload_cnt = 0;

static void vTaskMappingUpload(void *pvParameters)
{
	uint32_t count = 0 ;
    while(1)
    {
		if(!GetCmdStartUpload())
		{
			bsp_UploadMap();
		}
		if(count % 5 == 0)
        {
			bsp_ChangeWifi2SmartConfigStateProc();
			
			/*下面是打印开关，酌情注释*/
			bsp_WifiStateProc();
        }
		
		if(count % 100 == 0)
		{	
			bsp_UploadBatteryInfo();
		}
		if(count % 10 == 0)
		{	
			mcu_dp_value_update(DPID_CLEAN_TIME, RealWorkTime/1000/60);
			mcu_dp_value_update(DPID_CLEAN_AREA,bsp_Get_GridMapArea()); 
			mcu_dp_bool_update(DPID_SWITCH_GO,work_switch_go); //当前清扫开关;
			mcu_dp_enum_update(DPID_MODE,work_mode); //当前工作模式;
			mcu_dp_enum_update(DPID_STATUS,work_status); //当前设备状态
			//mcu_dp_fault_update(DPID_FAULT,当前故障告警); //当前故障告警;
		}
		
		
		//RTT("vTaskMappingUpload:%d\r\n",(int)uxTaskGetStackHighWaterMark(NULL));

		count++;
		vTaskMappingUpload_cnt++;	
		++cnt_task_1;		
        vTaskDelay(100);
    }

}


static void vTaskMapping(void *pvParameters)
{
	uint32_t count = 0 ;
	uint32_t vTaskMappingUpload_cnt_last = 0;
	
    while(1)
    {
		vTaskMappingUpload_cnt_last = vTaskMappingUpload_cnt;
		if(!GetCmdStartUpload())
		{
		if(isSearchCharge == false)
		{		
			bsp_GridMapUpdate(bsp_GetStrategyCurrentPosX()%4999,bsp_GetStrategyCurrentPosY()%4999,bsp_GetCurrentOrientation(),bsp_CollisionScan(),bsp_GetIRSensorData(),bsp_GetCliffSensorData());
		}
		}

		//RTT("vTaskMapping:%d\r\n",(int)uxTaskGetStackHighWaterMark(NULL));
		count++;
		vTaskMapping_cnt++;		
		++cnt_task_2;		
        vTaskDelay(100);
		
		if(count%20) 
		{
			if(vTaskMappingUpload_cnt_last == vTaskMappingUpload_cnt)
			{
				vTaskDelete(xHandleTaskMappingUpload);
				vTaskDelay(500);
				xTaskCreate( vTaskMappingUpload,/* 任务函数  */
                 "vTaskMappingUpload",   		/* 任务名    */
                 128,//512,            		    /* 任务栈大小，单位word，也就是4字节 */
                 NULL,           		        /* 任务参数  */
                 1,              		        /* 任务优先级*/
                 &xHandleTaskMappingUpload );   /* 任务句柄  */
				vTaskDelay(500);
			}
		}
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
	
	UNUSED(count);
	
    while(1)
    {

		bsp_GetVoltageFilterProc();
		
		//RTT("vTaskDecision:%d\r\n",(int)uxTaskGetStackHighWaterMark(NULL));
		
		++cnt_task_3;
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
		
		//RTT("vTaskControl:%d\r\n",(int)uxTaskGetStackHighWaterMark(NULL));
		
		count++;
		++cnt_task_4;
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
	
	/*开启地图更新*/
	bsp_StartUploadMap();
	
	

	
#if AT_POWER_ON_OPEN_ALL_MODULE_EN /*在开机的时候直接打开所有的电机轮子...，用于调试的时候使用*/
	bsp_MotorCleanSetPWM(MotorRollingBrush, CCW , CONSTANT_HIGH_PWM*0.9F);
	bsp_MotorCleanSetPWM(MotorSideBrush, CW , CONSTANT_HIGH_PWM*0.7F);
	bsp_SetMotorSpeed(MotorLeft,bsp_MotorSpeedMM2Pulse(250));
	bsp_SetMotorSpeed(MotorRight,bsp_MotorSpeedMM2Pulse(250));
#endif
	
    while(1)
    {
		bsp_ComAnalysis();
		
#if 1
		if(bsp_IsInitAW9523B_OK())
		{
			bsp_DetectAct();  /*红外对管轮询扫描*/
			bsp_DetectDeal(); /*红外对管扫描结果处理*/
		}
#endif
       /*测试床程序*/
		bsp_BedProc();
		
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
		if(!GetCmdStartUpload())
		{
			bsp_DustBoxProc();
		}
		
		
        /*寻找充电桩*/
		bsp_SearchChargePile();
		/*沿边行走*/
		bsp_EdgewiseRun();
        /*更新坐标*/
        bsp_PositionUpdate();
		
		
		if( ! bsp_IsBedProcRunning())
		{
			if(!GetCmdStartUpload()) /*上传命令的时候 ， 不执行正常的LED闪烁*/
			{
				bsp_LedAppProc();
			}
			else
			{
				if(count % 50 == 0)
				{
					bsp_OpenAllLed();
				}
			}
		}
		
		
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
		
		if(bsp_IsBedProcRunning() && count % 50 == 0)
		{
			bsp_LedBedTurns();
		}


		//RTT("vTaskPerception:%d\r\n",(int)uxTaskGetStackHighWaterMark(NULL));
		
		count++;
		++cnt_task_5;
        vTaskDelay(5);	
    }		
    
}



/*
*********************************************************************************************************
*	函 数 名: vTaskKey
*	功能说明: 按键处理
*	形    参: pvParameters 是在创建该任务时传递的形参
*	返 回 值: 无
*   优 先 级: 4  
*********************************************************************************************************
*/
static void vTaskKey(void *pvParameters)
{
	
	vTaskDelay(1000);
    while(1)
    {
		/* 处理按键事件 */
        bsp_KeyProc();
		
		//RTT("vTaskKey:%d\r\n",(int)uxTaskGetStackHighWaterMark(NULL));
		
		++cnt_task_6;
        vTaskDelay(20);	
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
	xTaskCreate( vTaskMappingUpload,     		/* 任务函数  */
                 "vTaskMappingUpload",   		/* 任务名    */
                 128,//512,            		    /* 任务栈大小，单位word，也就是4字节 */
                 NULL,           		        /* 任务参数  */
                 1,              		        /* 任务优先级*/
                 &xHandleTaskMappingUpload );         /* 任务句柄  */
	xTaskCreate( vTaskMapping,     		        /* 任务函数  */
                 "vTaskMapping",   		        /* 任务名    */
                 512,//512,            		    /* 任务栈大小，单位word，也就是4字节 */
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
                 512,//512,            		        /* 任务栈大小，单位word，也就是4字节 */
                 NULL,           		        /* 任务参数  */
                 3,              		        /* 任务优先级*/
                 &xHandleTaskControl );         /* 任务句柄  */	
    xTaskCreate( vTaskPerception,     		    /* 任务函数  */
                 "vTaskPerception",   		    /* 任务名    */
                 512,//512,            		        /* 任务栈大小，单位word，也就是4字节 */
                 NULL,           		        /* 任务参数  */
                 4,              		        /* 任务优先级*/
                 &xHandleTaskPerception );      /* 任务句柄  */	
	xTaskCreate( vTaskKey,     		            /* 任务函数  */
                 "vTaskKey",   		            /* 任务名    */
                 512,            		        /* 任务栈大小，单位word，也就是4字节 */
                 NULL,           		        /* 任务参数  */
                 4,              		        /* 任务优先级*/
                 &xHandleTaskKey );             /* 任务句柄  */	
	
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
//	mcu_dp_value_update(DPID_CLEAN_TIME, RealWorkTime/1000/60);
//	mcu_dp_value_update(DPID_CLEAN_AREA,(unsigned long)((bsp_Get_GridMapArea())*0.01)); 
//	mcu_dp_bool_update(DPID_SWITCH_GO,work_switch_go); //BOOL型数据上报;
//	mcu_dp_enum_update(DPID_MODE,work_mode); //枚举型数据上报;
//	//mcu_dp_enum_update(DPID_STATUS,当前设备状态); //枚举型数据上报;
	
}

static void bsp_StopAllMotor(void)
{
	bsp_SetMotorSpeed(MotorLeft, 0);
	bsp_SetMotorSpeed(MotorRight,0);
	bsp_MotorCleanSetPWM(MotorSideBrush, CW , 0);
	bsp_MotorCleanSetPWM(MotorRollingBrush, CW , 0);
	bsp_StopVacuum();
}



void bsp_CloseAllStateRun(void)
{
	bsp_StopSearchChargePile();
	bsp_StopCliffTest();
	bsp_StopUpdateCleanStrategyB();
	bsp_StopStrategyRandom();
	bsp_StopEdgewiseRun();
	bsp_BedStop();
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
	//清0方向角度
	bsp_CleanZeroYaw();
	//工作模式 待机
	//work_mode = standby;
	//清扫开关关闭
	work_switch_go = false;
	
	/*灯光亮3颗白色灯*/
	bsp_OpenThreeWhileLed();
	bsp_SetLedState(LED_DEFAULT_STATE);
	
	/*关闭所有电机*/
	bsp_StopAllMotor();
	
	/*关闭所有状态机*/
	bsp_CloseAllStateRun();

	/*上一次是清扫，本次就播放暂停清扫*/
	if(bsp_GetLastKeyState() == eKEY_CLEAN)
	{
		bsp_SperkerPlay(Song4);
	}
	
	/*上一次是清扫，本次就播放暂停清扫*/
	if(bsp_GetLastKeyState() == eKEY_SEARCH_CHARGE)
	{
		bsp_SperkerPlay(Song30);
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
				DEBUG("电源按键按下\r\n");
				bsp_KeySuspend();
				isNeedRun = true;
			}break;
				
			case KEY_DOWN_CHARGE:
			{
				DEBUG("充电按键按下\r\n");
				bsp_KeySuspend();
			}break;
				
			case KEY_DOWN_CLEAN:	
			{
				DEBUG("清扫按键按下\r\n");
				bsp_KeySuspend();
			}break;
			

			
			case KEY_9_DOWN: /*关机*/
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
				bsp_KeySuspend();

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
				bsp_SetLastKeyState(eKEY_SEARCH_CHARGE);
				/*设置LED状态*/
				bsp_SetLedState(AT_SEARCH_CHARGE);
				isSearchCharge = true;
				bsp_ClearKey();
				
			}break;
			
			case KEY_LONG_CLEAN: /*清扫*/
			{
				DEBUG("清扫按键长按\r\n");
				
				//清扫开关打开
				work_switch_go = true;
				bsp_SetUploadMapIdIndex();
				
				/*首先判断是否主机悬空*/
				if(!GetCmdStartUpload() && bsp_OffSiteGetState() == OffSiteBoth)   /*前提不处于上传状态*/
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
				
				bsp_SperkerPlay(Song3);
				bsp_IRD_StopWork();
				
				/*加入后退*/
			    if(bsp_IsTouchChargePile())
				{
					bsp_CleanZeroYaw();
					vTaskDelay(1000);
					bsp_SetMotorSpeed(MotorLeft, bsp_MotorSpeedMM2Pulse(-150));
					bsp_SetMotorSpeed(MotorRight,bsp_MotorSpeedMM2Pulse(-150));
					vTaskDelay(1000);
					bsp_SetMotorSpeed(MotorLeft, 0);
					bsp_SetMotorSpeed(MotorRight,0);
					vTaskDelay(500);
					if(isODDStart) 
					{	
						bsp_SetMotorSpeed(MotorLeft, bsp_MotorSpeedMM2Pulse(+200));
						bsp_SetMotorSpeed(MotorRight,bsp_MotorSpeedMM2Pulse(-200));
					}
					else
					{
						bsp_SetMotorSpeed(MotorLeft, bsp_MotorSpeedMM2Pulse(-200));
						bsp_SetMotorSpeed(MotorRight,bsp_MotorSpeedMM2Pulse(+200));
					}
					vTaskDelay(1900);
					bsp_SetMotorSpeed(MotorLeft, 0);
					bsp_SetMotorSpeed(MotorRight,0);
					vTaskDelay(500);
					if(isODDStart) 
					{	
						isODDStart = false;
					}
					else
					{
						isODDStart = true;
						bsp_CleanZeroYaw();
					}
						
					vTaskDelay(50);
				}
				
				if(!DEBUG_CLOSE_CLEAN_MOTOR){
				bsp_MotorCleanSetPWM(MotorSideBrush, CCW , CONSTANT_HIGH_PWM*0.7F);
				bsp_MotorCleanSetPWM(MotorRollingBrush, CW , CONSTANT_HIGH_PWM*0.7F);
				bsp_StartVacuum(bsp_GetVacuumPowerGrade());
				}
				vTaskDelay(2000);		
				
				bsp_StartUpdateCleanStrategyB();

				
				/*设置上一次按键值*/
				bsp_SetLastKeyState(eKEY_CLEAN);
				/*设置LED状态*/
				bsp_SetLedState(AT_CLEAN);
				isSearchCharge = false;
				bsp_ClearKey();
				
			}break;
			
			case KEY_LONG_POWER:
			{
				DEBUG("重新配网：同时按充电和清扫\r\n");
				bsp_SperkerPlay(Song29);
				bsp_StartChangeWifi2SmartConfigState();
				bsp_SetLedState(AT_LINK);
	
			}break;
			
			
			case KEY_10_LONG:
			{

				bsp_MotorCleanSetPWM(MotorSideBrush, CCW , CONSTANT_HIGH_PWM*0.7F);
				bsp_MotorCleanSetPWM(MotorRollingBrush, CW , CONSTANT_HIGH_PWM*0.7F);
				bsp_StartVacuum(bsp_GetVacuumPowerGrade());
				
				bsp_BedStart();
				
			}break;
			
			
			case KEY_WIFI_OPEN_CLEAN_CAR:
			{
				//bsp_SperkerPlay(Song1);
				//开启清扫
				bsp_PutKey(KEY_LONG_CLEAN); 
				
			}break;
			
			case KEY_WIFI_CLOSE_CLEAN_CAR:
			{
				//bsp_SperkerPlay(Song2);
				bsp_PutKey(KEY_DOWN_CLEAN);
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
				mcu_dp_enum_update(DPID_DIRECTION_CONTROL,forward); 
				bsp_SetMotorSpeed(MotorLeft, bsp_MotorSpeedMM2Pulse(250));
				bsp_SetMotorSpeed(MotorRight,bsp_MotorSpeedMM2Pulse(250));
				bsp_MotorCleanSetPWM(MotorSideBrush, CCW , CONSTANT_HIGH_PWM*0.7F);
				bsp_MotorCleanSetPWM(MotorRollingBrush, CW , CONSTANT_HIGH_PWM*0.7F);
				bsp_StartVacuum(bsp_GetVacuumPowerGrade());

			}break;
			
			case KEY_WIFI_DIR_BACK:
			{
				bsp_KeySuspend();
				mcu_dp_enum_update(DPID_DIRECTION_CONTROL,backward);
				bsp_SetMotorSpeed(MotorLeft, bsp_MotorSpeedMM2Pulse(-250));
				bsp_SetMotorSpeed(MotorRight,bsp_MotorSpeedMM2Pulse(-250));
				bsp_MotorCleanSetPWM(MotorSideBrush, CCW , CONSTANT_HIGH_PWM*0.7F);
				bsp_MotorCleanSetPWM(MotorRollingBrush, CW , CONSTANT_HIGH_PWM*0.7F);
				bsp_StartVacuum(bsp_GetVacuumPowerGrade());

			}break;
			
			case KEY_WIFI_DIR_LEFT:
			{
				bsp_KeySuspend();
				mcu_dp_enum_update(DPID_DIRECTION_CONTROL,turn_left);
				bsp_SetMotorSpeed(MotorLeft, bsp_MotorSpeedMM2Pulse(-100));
				bsp_SetMotorSpeed(MotorRight,bsp_MotorSpeedMM2Pulse(+100));
				bsp_MotorCleanSetPWM(MotorSideBrush, CCW , CONSTANT_HIGH_PWM*0.7F);
				bsp_MotorCleanSetPWM(MotorRollingBrush, CW , CONSTANT_HIGH_PWM*0.7F);
				bsp_StartVacuum(bsp_GetVacuumPowerGrade());

			}break;
			
			case KEY_WIFI_DIR_RIGHT:
			{
				bsp_KeySuspend();
				mcu_dp_enum_update(DPID_DIRECTION_CONTROL,turn_right);
				bsp_SetMotorSpeed(MotorLeft, bsp_MotorSpeedMM2Pulse(+100));
				bsp_SetMotorSpeed(MotorRight,bsp_MotorSpeedMM2Pulse(-100));
				bsp_MotorCleanSetPWM(MotorSideBrush, CCW , CONSTANT_HIGH_PWM*0.7F);
				bsp_MotorCleanSetPWM(MotorRollingBrush, CW , CONSTANT_HIGH_PWM*0.7F);
				bsp_StartVacuum(bsp_GetVacuumPowerGrade());

			}break;
			
			case KEY_WIFI_STOP:
			{
				bsp_KeySuspend();
				mcu_dp_enum_update(DPID_DIRECTION_CONTROL,stop);
			}break;
			
			case KEY_WIFI_EDGE:
			{

			}break;
		}   
	}
}


uint32_t bsp_GetTickCntTask_1(void)
{
	return cnt_task_1;
}

uint32_t bsp_GetTickCntTask_2(void)
{
	return cnt_task_2;
}

uint32_t bsp_GetTickCntTask_3(void)
{
	return cnt_task_3;
}

uint32_t bsp_GetTickCntTask_4(void)
{
	return cnt_task_4;
}

uint32_t bsp_GetTickCntTask_5(void)
{
	return cnt_task_5;
}

uint32_t bsp_GetTickCntTask_6(void)
{
	return cnt_task_6;
}


/***************************** 安富莱电子 www.armfly.com (END OF FILE) *********************************/
