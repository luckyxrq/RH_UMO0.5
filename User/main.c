#include "includes.h"

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

/*
**********************************************************************************************************
                                            变量声明
**********************************************************************************************************
*/
static TaskHandle_t xHandleTaskDecision      = NULL;
static TaskHandle_t xHandleTaskControl       = NULL;
static TaskHandle_t xHandleTaskPerception    = NULL;

static SemaphoreHandle_t  xMutex = NULL;

static uint8_t home_key_down_flag = 0;
static uint8_t power_key_down_flag = 0;
static uint8_t charge_key_down_flag = 0;
static uint8_t clean_key_down_flag = 0;
static uint8_t last_robot_state = ROBOT_STATE_DEFAULT;
static uint8_t cur_robot_state  = ROBOT_STATE_DEFAULT;
static uint8_t robot_work_way 	= ROBOT_WORKWAY_DEFAULT;
static uint8_t robot_error_num  = ROBOT_ERROR_NUM_DEFAULT;




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
    /*解决变量定义未使用警告*/
    UNUSED(power_key_down_flag);
    
    
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
    uint8_t ucKeyCode;	
    uint32_t count = 0 ;
    
    
    
    
    bsp_AngleRst();
    
    
    
    
    
    
    while(1)
    {
        /* 处理按键事件 */
        ucKeyCode = bsp_GetKey();
        if (ucKeyCode > 0)
        {
            /* 有键按下 */
            switch (ucKeyCode)
            {
            case KEY_1_DOWN:/*按键1按下*/
            {
                
            }break;
                
            case KEY_2_DOWN:/*按键2按下*/
            {
                
            }break;
                
            case KEY_3_DOWN:/*按键3按下*/	
            {
                
            }break;
                
            case KEY_1_LONG:/*按键1长按*/	
            {
                
            }break;
                
            case KEY_2_LONG:/*按键2长按*/	
            {
                
            }break;
                
            case KEY_3_LONG:/*按键3长按*/	
            {
                
            }break;
            }
        }
        
        if(count++ % 20 == 0)
        {
            if(bsp_SpeakerIsBusy())
            {
                DEBUG("busy\r\n");
            }
            else
            {
                DEBUG("play\r\n");
                bsp_SperkerPlay(Song3);
            }
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
    while(1)
    {
#if 0
        bsp_SendReportFrame();
        bsp_PrintRemoteState();
#endif
        bsp_IWDG_Feed(); /* 喂狗 */
        
        bsp_PidSched(); /*10MS调用一次，这里面进行PWM计算，占空比设置，速度（脉冲为单位；MM为单位）计算*/
#if 0		
        DEBUG("L %d MM/S\r\n",bsp_MotorGetSpeed(MotorLeft));
        DEBUG("R %d MM/S\r\n",bsp_MotorGetSpeed(MotorRight));
#endif		
        bsp_ComAnalysis();
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
static void vTaskPerception(void *pvParameters)   //
{
    /*开启红外对管轮询扫描*/
    bsp_DetectStart(); 
    /*开启寻找充电桩*/
#if 0
    bsp_StartSearchChargingPile();
#endif
    bsp_StartUpdatePos();
    
    while(1)
    {
        bsp_DetectAct();  /*红外对管轮询扫描*/
        bsp_DetectDeal(); /*红外对管扫描结果处理*/
        
        /*四个红外接收管*/
#if 0 
        bsp_GetCapCnt(CapCH1);
        bsp_GetCapCnt(CapCH2);
        bsp_GetCapCnt(CapCH3);
        bsp_GetCapCnt(CapCH4);
#endif
        /*寻找充电桩*/
        bsp_SearchChargingPileAct();
        /*更新坐标*/
        bsp_PositionUpdate();
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



static void bsp_func(void)
{
    uint16_t clifadc_right[5] = {0};
    uint16_t clifadc_left[5] = {0};
    uint16_t clifadc_middle[5] = {0};
    
    uint32_t clifAdcRight = 0;
    uint32_t clifAdcLeft = 0;
    uint32_t clifAdcMiddle = 0;
    uint16_t clifAdcRef = 0;
    
    uint8_t clifRightCnt = 0;
    uint8_t clifLeftCnt = 0;
    uint8_t clifMiddleCnt = 0;
    uint8_t i;
    
    
    //*****************************************************************************************************//	
    // 主循环
    //a 获取按键信息标志位
    
    //b 更新机器状态并作出决策
    //0初始化
    //a 检查红外跳崖传感器数据并标定
    //b 检查红外碰撞传感器数据是否正常
    //1暂停
    //a 关闭所有电机电源
    //b 计算暂停时间，超时后进入待机状态
    //2工作
    //a 获取尘盒状态
    //b 获取离地开关状态
    //c 获取电机电流
    //d 获取跳崖传感器数据
    //e 获取红外对管碰撞沿边数据
    //f	获取电池电压数据
    //g 传感器数据正常根据策略清扫（乱撞 、惯导规划 、BOT3）
    //h 传感器数据异常，语音播报，返回暂停状态
    //3待机
    //a 低功耗模式
    //4回充
    //a 关闭清扫电机电流
    //b 获取离地开关状态
    //c 获取运动电机电流
    //d 获取跳崖传感器数据
    //e 获取红外对管碰撞沿边数据
    //f 获取两对红外编码数据
    //g 获取充电状态
    //h 传感器数据正常根据策略回充（自主、BOT3）
    //i 传感器数据异常，语音播报，返回暂停状态
    //*****************************************************************************************************//					
    last_robot_state = cur_robot_state;
    //*****************************************************************************************************//		
    if(home_key_down_flag == 1)
    {
        if(last_robot_state == ROBOT_STATE_DEFAULT)       cur_robot_state = ROBOT_STATE_INIT;
        else if(last_robot_state == ROBOT_STATE_WORKING)  cur_robot_state = ROBOT_STATE_SUSPEND;
        else if(last_robot_state == ROBOT_STATE_CHARGING) cur_robot_state = ROBOT_STATE_SUSPEND;
        else if(last_robot_state == ROBOT_STATE_SUSPEND)  cur_robot_state = ROBOT_STATE_WORKING;
        else if(last_robot_state == ROBOT_STATE_STANDBY)  cur_robot_state = ROBOT_STATE_WORKING;
        
        robot_work_way  = ROBOT_WORKWAY_HOME;
    }
    else if(charge_key_down_flag == 1)
    {
        if(last_robot_state == ROBOT_STATE_DEFAULT)       cur_robot_state = ROBOT_STATE_INIT;
        else if(last_robot_state == ROBOT_STATE_WORKING)  cur_robot_state = ROBOT_STATE_CHARGING;
        else if(last_robot_state == ROBOT_STATE_CHARGING) cur_robot_state = ROBOT_STATE_SUSPEND;
        else if(last_robot_state == ROBOT_STATE_SUSPEND)  cur_robot_state = ROBOT_STATE_CHARGING;
        else if(last_robot_state == ROBOT_STATE_STANDBY)  cur_robot_state = ROBOT_STATE_CHARGING;
        
        robot_work_way  = ROBOT_WORKWAY_CHARGE;
    }
    else if(clean_key_down_flag == 1)
    {
        if(last_robot_state == ROBOT_STATE_DEFAULT)       cur_robot_state = ROBOT_STATE_INIT;
        else if(last_robot_state == ROBOT_STATE_WORKING)  cur_robot_state = ROBOT_STATE_SUSPEND;
        else if(last_robot_state == ROBOT_STATE_CHARGING) cur_robot_state = ROBOT_STATE_SUSPEND;
        else if(last_robot_state == ROBOT_STATE_SUSPEND)  cur_robot_state = ROBOT_STATE_WORKING;
        else if(last_robot_state == ROBOT_STATE_STANDBY)  cur_robot_state = ROBOT_STATE_WORKING;
        
        robot_work_way  = ROBOT_WORKWAY_CLEAN;
    }
    //*****************************************************************************************************//
    
    //*****************************************************************************************************//		
    if(cur_robot_state == ROBOT_STATE_INIT)
    {
        for( i=0;i<5;i++)
        {
            clifadc_left[i]   =   bsp_GetCliffVoltage(CliffLeft);     //left
            clifadc_middle[i] =   bsp_GetCliffVoltage(CliffMiddle);   //middle
            clifadc_right[i]  =   bsp_GetCliffVoltage(CliffRight);    //right
            vTaskDelay(10);	
            clifAdcRight+=clifadc_right[i];
            clifAdcLeft +=clifadc_left[i];
            clifAdcMiddle+=clifadc_middle[i];
        }
        
        clifAdcRight/=5; 
        clifAdcLeft/=5; 
        clifAdcMiddle/=5; 
        clifAdcRef = (clifAdcRight+clifAdcLeft+clifAdcMiddle)/3;
        
        //	DEBUG("%d,%d,%d\n",clifAdcRight,clifAdcLeft,clifAdcMiddle);		
        //	DEBUG("clifAdcRef:%d, %d,%d,%d\n",clifAdcRef,clifAdcRight-clifAdcRef,clifAdcLeft-clifAdcRef,clifAdcMiddle-clifAdcRef);
        
        if(((clifAdcRef>clifAdcRight)?(clifAdcRef-clifAdcRight):(clifAdcRight-clifAdcRef)) > MAXCLIFFADCDT )
        {
            //	DEBUG("请擦拭悬崖传感器");
            cur_robot_state = ROBOT_STATE_DEFAULT;
            robot_error_num = ROBOT_ERROR_NUM_CLIF;
            while(bsp_SpeakerIsBusy()) vTaskDelay(100);	
            bsp_SperkerPlay(Song11);
            //continue;
        }
        if(((clifAdcRef>clifAdcLeft)?(clifAdcRef-clifAdcLeft):(clifAdcLeft-clifAdcRef)) > MAXCLIFFADCDT )
        {
            //	DEBUG("请擦拭悬崖传感器");
            cur_robot_state = ROBOT_STATE_DEFAULT;
            robot_error_num = ROBOT_ERROR_NUM_CLIF;
            while(bsp_SpeakerIsBusy()) vTaskDelay(100);	
            bsp_SperkerPlay(Song11);				
            //continue;
        }
        if(((clifAdcRef>clifAdcMiddle)?(clifAdcRef-clifAdcMiddle):(clifAdcMiddle-clifAdcRef)) > MAXCLIFFADCDT )
        {
            //	DEBUG("请擦拭悬崖传感器");
            cur_robot_state = ROBOT_STATE_DEFAULT;
            robot_error_num = ROBOT_ERROR_NUM_CLIF;
            while(bsp_SpeakerIsBusy()) vTaskDelay(100);	
            bsp_SperkerPlay(Song11);
            //continue;
        }
        
        cur_robot_state = (robot_work_way == ROBOT_WORKWAY_CHARGE)? ROBOT_STATE_CHARGING:ROBOT_STATE_WORKING;
        
        if(robot_work_way==ROBOT_WORKWAY_CLEAN || robot_work_way==ROBOT_WORKWAY_HOME)
        {
            bsp_MotorCleanSetPWM(MotorRollingBrush, CW , ROLLMOTORRPM);
            bsp_MotorCleanSetPWM(MotorSideBrush, CW , BRUSHMOTORRPM);
            bsp_StartVacuum();
            bsp_SetMotorSpeed(MotorLeft,0);
            bsp_SetMotorSpeed(MotorRight,0);
            
        }
        
    }
    //*****************************************************************************************************//
    else if(cur_robot_state == ROBOT_STATE_STANDBY)
    {
        //do nothing
    }
    //*****************************************************************************************************//
    else if(cur_robot_state == ROBOT_STATE_WORKING)
    {
        //a 获取尘盒状态
        //b 获取离地开关状态
        //c 获取电机电流
        //d 获取电池电压数据
        //e 获取红外对管碰撞沿边数据
        //f	获取跳崖传感器数据
        //g 传感器数据异常，语音播报，返回暂停状态
        //h 传感器数据正常根据策略清扫（乱撞 、惯导规划 、BOT3）
        
        if( DustBoxOutside == bsp_DustBoxGetState()) 
        {
            cur_robot_state = ROBOT_STATE_SUSPEND;
            robot_error_num = ROBOT_ERROR_NUM_DUST_HALL;
            //continue;
        }
        if( OffSiteRight == bsp_OffSiteGetState())
        {
            cur_robot_state = ROBOT_STATE_SUSPEND;
            robot_error_num = ROBOT_ERROR_NUM_OFFLANDR;
            //continue;
        }
        if( OffSiteLeft ==  bsp_OffSiteGetState())
        {
            cur_robot_state = ROBOT_STATE_SUSPEND;
            robot_error_num = ROBOT_ERROR_NUM_OFFLANDL;
            //continue;
        }
        //get roller motor adc value
        if(bsp_GetFeedbackVoltage(eRollingBrush) >ROLLER_MOTOR_MAX_ADC_VALUE)
        {
            cur_robot_state = ROBOT_STATE_SUSPEND;
            robot_error_num = ROBOT_ERROR_NUM_ROLLER_MOTOR;
            //continue;
        }
        
        //get brush motor adc vaule
        if(bsp_GetFeedbackVoltage(eSideBrush) >BRUSH_MOTOR_MAX_ADC_VALUE)
        {
            cur_robot_state = ROBOT_STATE_SUSPEND;
            robot_error_num = ROBOT_ERROR_NUM_BRUSH_MOTOR;
            //continue;
        }
        
        //get vacuum motor adc value
        if(bsp_GetFeedbackVoltage(eVacuum) >VACUUM_MOTOR_MAX_ADC_VALUE)
        {
            cur_robot_state = ROBOT_STATE_SUSPEND;
            robot_error_num = ROBOT_ERROR_NUM_VACUUM_MOTOR;
            //continue;
        }
        //get robot battery adc value
        if(bsp_GetFeedbackVoltage(eBatteryVoltage) <ROBOT_BATTERY_MIN_ADC_VALUE)
        {
            cur_robot_state = ROBOT_STATE_CHARGING;
            robot_error_num = ROBOT_ERROR_NUM_BATTERY;
            //continue;
        }
        //get left wheel adc value
        if(bsp_GetFeedbackVoltage(eMotorLeft) >WHEEL_MOTOR_MAX_ADC_VALUE)
        {
            cur_robot_state = ROBOT_STATE_SUSPEND;
            robot_error_num = ROBOT_ERROR_NUM_LEFT_WHEEL;
            //continue;
        }
        //get right wheel adc value
        if(bsp_GetFeedbackVoltage(eMotorRight) >WHEEL_MOTOR_MAX_ADC_VALUE)
        {
            cur_robot_state = ROBOT_STATE_SUSPEND;
            robot_error_num = ROBOT_ERROR_NUM_RIGHT_WHEEL;
            //continue;
        }
        
        bsp_GetCliffVoltage(CliffLeft);     //left
        bsp_GetCliffVoltage(CliffMiddle);   //middle
        bsp_GetCliffVoltage(CliffRight);
        //get cliff infrared data
        if((clifAdcLeft  -  bsp_GetCliffVoltage (CliffLeft   ))>1500) clifLeftCnt++;
        if((clifAdcMiddle - bsp_GetCliffVoltage (CliffMiddle ))>1500) clifMiddleCnt++;
        if((clifAdcRight -  bsp_GetCliffVoltage (CliffRight  ))>1500) clifRightCnt++;	
        //get  collision infrared data
        
        //get bumper data
        if(robot_work_way  == ROBOT_WORKWAY_HOME)
        {
            if( CollisionLeft == bsp_CollisionScan())
            {
                bsp_SetMotorSpeed(MotorLeft,-MOTORSPEED);
                bsp_SetMotorSpeed(MotorRight,-MOTORSPEED);
                vTaskDelay(500);
                bsp_SetMotorSpeed(MotorLeft,MOTORSPEED);
                bsp_SetMotorSpeed(MotorRight,-MOTORSPEED);
                vTaskDelay(800);
            }
            else if(CollisionRight == bsp_CollisionScan())
            {
                bsp_SetMotorSpeed(MotorLeft,-MOTORSPEED);
                bsp_SetMotorSpeed(MotorRight,-MOTORSPEED);
                vTaskDelay(500);
                bsp_SetMotorSpeed(MotorLeft,-MOTORSPEED);
                bsp_SetMotorSpeed(MotorRight,MOTORSPEED);
                vTaskDelay(800);
            }
            else if(CollisionAll == bsp_CollisionScan())
            {
                bsp_SetMotorSpeed(MotorLeft,-MOTORSPEED);
                bsp_SetMotorSpeed(MotorRight,-MOTORSPEED);
                vTaskDelay(500);
                bsp_SetMotorSpeed(MotorLeft,-MOTORSPEED);
                bsp_SetMotorSpeed(MotorRight,MOTORSPEED);
                vTaskDelay(1000);
            }
            else if(clifRightCnt>2 )
            {
                clifRightCnt = 0;
                bsp_SetMotorSpeed(MotorLeft,-MOTORSPEED);
                bsp_SetMotorSpeed(MotorRight,-MOTORSPEED);
                vTaskDelay(500);
                bsp_SetMotorSpeed(MotorLeft,-MOTORSPEED);
                bsp_SetMotorSpeed(MotorRight,MOTORSPEED);
                vTaskDelay(800);
                
            }
            else if(clifLeftCnt>2)
            {
                clifLeftCnt = 0;
                bsp_SetMotorSpeed(MotorLeft,-MOTORSPEED);
                bsp_SetMotorSpeed(MotorRight,-MOTORSPEED);
                vTaskDelay(500);
                bsp_SetMotorSpeed(MotorLeft,MOTORSPEED);
                bsp_SetMotorSpeed(MotorRight,-MOTORSPEED);
                vTaskDelay(800);
            }
            else if(clifMiddleCnt>2)
            {
                clifMiddleCnt = 0;
                bsp_SetMotorSpeed(MotorLeft,-MOTORSPEED);
                bsp_SetMotorSpeed(MotorRight,-MOTORSPEED);
                vTaskDelay(500);
                bsp_SetMotorSpeed(MotorLeft,MOTORSPEED);
                bsp_SetMotorSpeed(MotorRight,-MOTORSPEED);
                vTaskDelay(800);
            }
            else
            {
                bsp_SetMotorSpeed(MotorLeft,MOTORSPEED);
                bsp_SetMotorSpeed(MotorRight,MOTORSPEED);
            }
            
        }
        
        if(robot_work_way  == ROBOT_WORKWAY_CLEAN)
        {
            
        }
        
        
    }
    //*****************************************************************************************************//
    else if(cur_robot_state == ROBOT_STATE_SUSPEND)
    {		
        //WheelBrake();
        bsp_MotorCleanSetPWM(MotorRollingBrush, CW , 0);
        bsp_MotorCleanSetPWM(MotorSideBrush, CW , 0);
        bsp_StopVacuum();
        bsp_SetMotorSpeed(MotorLeft,0);
        bsp_SetMotorSpeed(MotorRight,0);
        
        switch(robot_error_num)
        {
        case ROBOT_ERROR_NUM_CLIF : 
            while(bsp_SpeakerIsBusy()) vTaskDelay(100);	
            bsp_SperkerPlay(Song11);
            break;
        case ROBOT_ERROR_NUM_LEFT_WHEEL :
            while(bsp_SpeakerIsBusy()) vTaskDelay(100);	
            bsp_SperkerPlay(Song11);					
            break;	
        case ROBOT_ERROR_NUM_RIGHT_WHEEL :
            while(bsp_SpeakerIsBusy()) vTaskDelay(100);	
            bsp_SperkerPlay(Song11);					
            break;	
        case ROBOT_ERROR_NUM_ROLLER_MOTOR :
            while(bsp_SpeakerIsBusy()) vTaskDelay(100);	
            bsp_SperkerPlay(Song11);					
            break;	
        case ROBOT_ERROR_NUM_BRUSH_MOTOR :
            while(bsp_SpeakerIsBusy()) vTaskDelay(100);	
            bsp_SperkerPlay(Song11);
            break;    
        case ROBOT_ERROR_NUM_VACUUM_MOTOR :
            while(bsp_SpeakerIsBusy()) vTaskDelay(100);	
            bsp_SperkerPlay(Song11);					
            break;	
        case ROBOT_ERROR_NUM_OFFLANDR :
            while(bsp_SpeakerIsBusy()) vTaskDelay(100);	
            bsp_SperkerPlay(Song11);					
            break;     
        case ROBOT_ERROR_NUM_OFFLANDL :
            while(bsp_SpeakerIsBusy()) vTaskDelay(100);	
            bsp_SperkerPlay(Song11);
            break;	
        case ROBOT_ERROR_NUM_BATTERY :
            while(bsp_SpeakerIsBusy()) vTaskDelay(100);	
            bsp_SperkerPlay(Song11);
            break;
        case ROBOT_ERROR_NUM_BUMPER :
            while(bsp_SpeakerIsBusy()) vTaskDelay(100);	
            bsp_SperkerPlay(Song11);
            break;	
        case ROBOT_ERROR_NUM_DUST_HALL :
            while(bsp_SpeakerIsBusy()) vTaskDelay(100);	
            bsp_SperkerPlay(Song11);
            break;	
        case ROBOT_ERROR_NUM_DEFAULT :
            while(bsp_SpeakerIsBusy()) vTaskDelay(100);	
            bsp_SperkerPlay(Song11);
            break;	
        default:
            break;
            
        }
        robot_error_num = ROBOT_ERROR_NUM_DEFAULT;
        cur_robot_state = ROBOT_STATE_STANDBY;
    }
    //*****************************************************************************************************//
    else if(cur_robot_state == ROBOT_STATE_CHARGING)
    {
        bsp_MotorCleanSetPWM(MotorRollingBrush, CW , 0);
        bsp_MotorCleanSetPWM(MotorSideBrush, CW , 0);
        bsp_StopVacuum();
        bsp_SetMotorSpeed(MotorLeft,0);
        bsp_SetMotorSpeed(MotorRight,0);
        //printf("AutogoChargeStation!!!! \n");	
    }
}

/***************************** 安富莱电子 www.armfly.com (END OF FILE) *********************************/
