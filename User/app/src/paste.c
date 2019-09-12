/*
	这个文件没有功能，用于文件对比粘贴方便
*/


IR_TX_LEFT   = 0x27 ,
IR_TX_CENTER = 0x39 ,
IR_TX_RIGHT  = 0x16

IR_TX_SITE_LEFT   = 0 ,
IR_TX_SITE_CENTER ,
IR_TX_SITE_RIGHT  


TotalWitdh
g_tIR.TotalWitdh[ch]


g_tIR.isRev[ch][IR_TX_SITE_LEFT] = false;
g_tIR.isRev[ch][IR_TX_SITE_CENTER] = false;
g_tIR.isRev[ch][IR_TX_SITE_RIGHT] = false;


static void bsp_SearchRunStraightFast(void);
static void bsp_SearchRunStraightSlow(void);
static void bsp_SearchTurnRightFast(void)  ;
static void bsp_SearchTurnRightSlow(void)  ;
static void bsp_SearchTurnLeftFast(void)   ;
static void bsp_SearchTurnLeftSlow(void)   ;


search.collision = eHasSignalCollision;
				}
				else
				{
					search.collision = eNoSignalCollision;
					
					
					
					
					
					
					
					
					
					
					
					
					
					
					
					
					
					
					
					
static uint8_t home_key_down_flag = 0;
static uint8_t power_key_down_flag = 0;
static uint8_t charge_key_down_flag = 0;
static uint8_t clean_key_down_flag = 0;
static uint8_t last_robot_state = ROBOT_STATE_DEFAULT;
static uint8_t cur_robot_state  = ROBOT_STATE_DEFAULT;
static uint8_t robot_work_way 	= ROBOT_WORKWAY_DEFAULT;
static uint8_t robot_error_num  = ROBOT_ERROR_NUM_DEFAULT;

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
    
	/*解决变量定义未使用警告*/
    UNUSED(power_key_down_flag);
    UNUSED(bsp_func);
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
