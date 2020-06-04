#include "includes.h"

/*
**********************************************************************************************************
                                            宏定义
**********************************************************************************************************
*/
#define PAUSE_INTERVAL_RESPONSE_TIME         1
#define AT_POWER_ON_OPEN_ALL_MODULE_EN       0     /*在开机的时候直接打开所有的电机轮子...，用于调试的时候使用*/

#define DEBUG_CLOSE_CLEAN_MOTOR              0 //1 关闭清扫电机
#define main_debug(format, ...) //printf (format, ##__VA_ARGS__)
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
	uint32_t battery = 0 ;
    while(1)
    {
     		
#if 1 /*更新地图*/
		
		if(isSearchCharge){}
		else{		
			main_debug("bsp_GridMapUpdate() \n");
			bsp_GridMapUpdate(bsp_GetCurrentPosX(),bsp_GetCurrentPosY(),bsp_GetCurrentOrientation(),bsp_CollisionScan(),bsp_GetIRSensorData(),bsp_GetCliffSensorData());

		}
#endif
		
		//bsp_UploadMap();
		if(count % 20 == 0)
		{
			mcu_dp_value_update(DPID_RESIDUAL_ELECTRICITY,++battery % 100);
		}
        vTaskDelay(100);
		
		
    }

}


static void vTaskDecision(void *pvParameters)      //决策 整机软件控制流程
{
    
    uint32_t count = 0 ;

    while(1)
    {
        /* 处理按键事件 */
		main_debug("bsp_KeyProc() \n");
        bsp_KeyProc();
		
		
        if(count++ % 10 == 0)
        {
#if 0 
			bsp_PrintIR_Rev(); /*用于打印红外接收状态*/
#endif
			//main_debug("bsp_ChangeWifi2SmartConfigStateProc() \n");
			bsp_ChangeWifi2SmartConfigStateProc();
			
			/*下面是打印开关，酌情注释*/
			//main_debug("bsp_WifiStateProc() \n");
			bsp_WifiStateProc();
//			bsp_PrintCollision();
//			bsp_PrintIR_Rev();
//			bsp_PrintAllVoltage();
			/*更新跳崖传感器信息*/
			bsp_GetCliffStates();
//			bsp_PrintCliff();
			bsp_SendReportFrameWithCRC16();
			
			//DEBUG("bsp_AngleReadRaw:%d\r\n",bsp_AngleReadRaw());
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
			DEBUG("bsp_CleanZeroYaw()\r\n");
		}
		
		DEBUG("Orientation: %f ° \r\n", Rad2Deg(bsp_GetCurrentOrientation()));
#endif
		
		
		if(isSearchCharge)
		{
		
		}
		else
		{	
			main_debug("bsp_UpdateCleanStrategyB() \n");
			bsp_UpdateCleanStrategyB(bsp_GetCurrentPosX(),bsp_GetCurrentPosY(),bsp_GetCurrentOrientation(), bsp_CollisionScan(), \
			bsp_MotorGetPulseVector(MotorLeft), bsp_MotorGetPulseVector(MotorRight), bsp_GetIRSensorData(),bsp_GetCliffSensorData());
			
		}//DEBUG("%+4d,%+4d#%+3d \n",bsp_GetCurrentPosX()/10,bsp_GetCurrentPosY()/10,(int)Rad2Deg(bsp_GetCurrentOrientation()));
		
		
		
		if(GetReturnChargeStationStatus())
		{
			
			//main_debug("bsp_StopUpdateCleanStrategyB() \n");
			bsp_StopUpdateCleanStrategyB();
			
			
			//main_debug("ResetReturnChargeStationStatus() \n");
			ResetReturnChargeStationStatus();
			
			
			//main_debug("bsp_PutKey(KEY_LONG_CHARGE) \n");
			bsp_PutKey(KEY_LONG_CHARGE);
		}
		
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
float roll = 0.0F;
static void vTaskPerception(void *pvParameters)
{
	uint32_t count = 0 ;
	
    /*开启红外对管轮询扫描*/
    bsp_DetectStart(); 
	
	/*检测主机悬空*/
	bsp_StartOffSiteProc();
	
	bsp_StartDustBoxProc();
	
	/*开启寻找充电桩*/
	//bsp_StartSearchChargePile();
	
	/*开启沿边行走*/
	//bsp_StartEdgewiseRun();
	
	/*开启位置坐标更新*/
    bsp_StartUpdatePos();
	
    /*开启正面碰撞协助*/
	//bsp_StartAssistJudgeDirection();
	
	/*开启栅格地图跟新*/
	bsp_StartUpdateGridMap();

	/*开清扫策略*/
	//bsp_StartUpdateCleanStrategyB();
	
	//bsp_StartCliffTest();

	vTaskDelay(2000);		
	
	bsp_InitCliffSW();
	
	
	
#if AT_POWER_ON_OPEN_ALL_MODULE_EN /*在开机的时候直接打开所有的电机轮子...，用于调试的时候使用*/
	bsp_StartVacuum();
	bsp_MotorCleanSetPWM(MotorRollingBrush, CCW , CONSTANT_HIGH_PWM*0.9F);
	bsp_MotorCleanSetPWM(MotorSideBrush, CW , CONSTANT_HIGH_PWM*0.7F);
	bsp_SetMotorSpeed(MotorLeft,bsp_MotorSpeedMM2Pulse(250));
	bsp_SetMotorSpeed(MotorRight,bsp_MotorSpeedMM2Pulse(250));
#endif
	
    while(1)
    {
		bsp_ComAnalysis();
		//bsp_CliffTest();
		
		//roll = bsp_IMU_GetData(ROLL)*100*0.01F;
		//DEBUG("%.2F\r\n",roll);
//		if(roll <= 175 && roll >= 0)
//		{
//			bsp_SetMotorSpeed(MotorLeft,bsp_MotorSpeedMM2Pulse(500));
//			bsp_SetMotorSpeed(MotorRight,bsp_MotorSpeedMM2Pulse(500));
//		}
//		else
//		{
//			bsp_SetMotorSpeed(MotorLeft,bsp_MotorSpeedMM2Pulse(250));
//			bsp_SetMotorSpeed(MotorRight,bsp_MotorSpeedMM2Pulse(250));
//		}
		
		
//		if(isNeedRun)
//		{
//			if(bsp_CliffIsDangerous(CliffLeft) || bsp_CliffIsDangerous(CliffMiddle) || bsp_CliffIsDangerous(CliffRight))
//			{
//				bsp_SetMotorSpeed(MotorLeft,bsp_MotorSpeedMM2Pulse(0));
//				bsp_SetMotorSpeed(MotorRight,bsp_MotorSpeedMM2Pulse(0));
//				isNeedRun = false;
//			}
//			else
//			{
//				bsp_SetMotorSpeed(MotorLeft,bsp_MotorSpeedMM2Pulse(250));
//				bsp_SetMotorSpeed(MotorRight,bsp_MotorSpeedMM2Pulse(250));
//			}
//		}
		
#if 1
		if(bsp_IsInitAW9523B_OK())
		{
			//main_debug("bsp_DetectAct() \n");
			bsp_DetectAct();  /*红外对管轮询扫描*/
			//main_debug("bsp_DetectDeal() \n");
			bsp_DetectDeal(); /*红外对管扫描结果处理*/
		}
#endif
       
#if 0   /*测试红外测距的距离，测到后就停下来*/
		bsp_DetectMeasureTest();
#endif

#if 0  /*测试跳崖传感器 、红外、碰撞共同测试*/	 
		//main_debug("bsp_CliffTest() \n");
		bsp_CliffTest();
#endif
		
#if 0  /*测试IMU数据是否正常*/	 
		DEBUG("bsp_AngleReadRaw:%d\n",bsp_AngleReadRaw());
#endif
		
#if 0  /*测试电池电压数据是否正常*/	 
		
		DEBUG("BatteryVoltage:%f\n",((bsp_GetFeedbackVoltage(eBatteryVoltage) * 430 / 66.5) + bsp_GetFeedbackVoltage(eBatteryVoltage) + 0.2F));
#endif
		
		
		

		bsp_StrategyRandomProc();
		
		/*检测主机悬空*/
		//main_debug("bsp_OffSiteProc() \n");
		//bsp_OffSiteProc();
		//check dust box
		bsp_DustBoxProc();
        /*寻找充电桩*/
		//main_debug("bsp_SearchChargePile() \n");
		bsp_SearchChargePile();
		/*沿边行走*/
		//main_debug("bsp_EdgewiseRun() \n");
		bsp_EdgewiseRun();
        /*更新坐标*/
		//main_debug("bsp_PositionUpdate() \n");
        bsp_PositionUpdate();
		//main_debug("bsp_LedAppProc() \n");
		bsp_LedAppProc();
		
		//main_debug("wifi_uart_service() \n");
		wifi_uart_service();
		
		

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
	
	xTaskCreate( vTaskMapping,     		    /* 任务函数  */
                 "vTaskMapping",   		    /* 任务名    */
                 1024*1.5,            		    /* 任务栈大小，单位word，也就是4字节 */
                 NULL,           		        /* 任务参数  */
                 1,              		        /* 任务优先级*/
                 &xHandleTaskMapping );        /* 任务句柄  */
    xTaskCreate( vTaskDecision,     		    /* 任务函数  */
                 "vTaskDecision",   		    /* 任务名    */
                 512,            		    /* 任务栈大小，单位word，也就是4字节 */
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
    
    printf("%s", buf_str);
    
    xSemaphoreGive(xMutex);
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
//				if(bsp_OffSiteGetState() != OffSiteNone)
//				{
//					bsp_SperkerPlay(Song16);
//					return;
//				}
				
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
//				if(bsp_OffSiteGetState() == OffSiteBoth)
//				{
//					bsp_SperkerPlay(Song16);
//					return;
//				}
				
				/*首先判断尘盒*/
				if(bsp_DustBoxGetState() == DustBoxOutside)
				{
					bsp_SperkerPlay(Song9);
					return;
				}
				
				bsp_SperkerPlay(Song3);
				
				
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
					
				bsp_StartUpdateCleanStrategyB();
				//bsp_StartEdgewiseRun();
				
				if(!DEBUG_CLOSE_CLEAN_MOTOR){
				bsp_MotorCleanSetPWM(MotorSideBrush, CCW , CONSTANT_HIGH_PWM*0.7F);
				bsp_MotorCleanSetPWM(MotorRollingBrush, CCW , CONSTANT_HIGH_PWM*0.7F);
				bsp_StartVacuum();
				}
				
				
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
			
			
			case KEY_10_DOWN:
			{
				
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
