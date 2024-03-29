#include "bsp.h"
#include <math.h>

#define STRATEGY_DEBUG      0 //0 关闭串口调试
#define INT_COOR_X 250
#define INT_COOR_Y 250
#define ALL_CLEAN_COMPLETE 0
#define CLEAN_WORK_TIME 40*60*1000
#define EDGEWISE_CLEAN_WORK_TIME 5*60*1000
#define FORCE_RETURN_ORIGIN_WORK_TIME 2*60*1000

#define ZoomMultiple 4
#define compression_map_x 25
#define compression_map_y 25


unsigned char rightmapmin=0;
unsigned char rightmapmax=0;

short wheel_pulse_l;
short wheel_pulse_r;
short temporary_wheel_pulse_r;
short temporary_wheel_pulse_l;

short OVERALL_CLEANING_STRATEGY = 0;

short right_running_step_status = 0;
short collision_right_rightrun_step_status = 0;
short collision_left_rightrun_step_status = 0;
short collision_front_rightrun_step_status = 0;

short left_running_step_status = 0;
short collision_right_leftrun_step_status = 0;
short collision_left_leftrun_step_status = 0;
short collision_front_leftrun_step_status = 0;
short close_edge_map_run_step_status=0;

short stuck_right_run_step=0;

short return_origin_step_status = 0;
short a_star_motion_return_origin_status=0;
short a_star_not_motion_status=0;
short a_star_collision_status=0;
bool  over_clean_finish=false;

int8_t return_origin_positive_start= 1;
int8_t selectside=0;


//for  walk edge
short right_walk_edge_status = 0;
short right_reverse_walk_edge_status = 0;
short right_edge_dilemma_status = 0;
short right_forward_boundary_status = 0;
short right_ready_leaking_sweep_status = 0;

short left_walk_edge_status = 0;
short left_reverse_walk_edge_status = 0;
short left_edge_dilemma_status = 0;
short left_forward_boundary_status = 0;
short left_ready_leaking_sweep_status = 0;


int number = 0;
int leakingsweep = 0;
int leakingsweep_x = 0;
int leakingsweep_y = 0;
int leakingsweep_y_flag=0;
bool bool_leakingsweep_y=false;
short leakingsweep_X_interval = 200;
short leakingsweep_Y_interval = 100;

double linear_velocity = 0,angular_velocity = 0;

unsigned char distance_uptate  = 0;
unsigned char turn_start_update = 0;
int origin_last_position_x=0;
int origin_last_position_y=0;
int last_position_x = 0;
int last_position_xx = 0;
int last_position_y = 0;
int last_position_yy = 0;
bool b_last_position_yy = false;
int turn_start_x = 0;
int turn_start_y = 0;
int maintain_bow_distance = 0;
int cnt_update = 0;
bool bow_continue = false;
bool old_bow_continue = false;
int temporary_yaw = 0;
int temporary_close_edge = 0;
bool returnorigin = false;
int edge_length_start = 0;

unsigned char rightmapmin;
unsigned char rightmapmax;

unsigned char DelimmaNumber=0;

bool clill_start_update=false;
bool cliffruningStatus=false;


int FunctionStatus=0;
unsigned int RealWorkTime = 0;
unsigned int LastCleanTimeStamp = 0;
unsigned int CurrentCleanTimeStamp  = 0;
unsigned int EdgeWiseCleanTimeStamp = 0;
unsigned int ForceReturnOriginTimeStamp = 0;

//**************return origin value************************************
AStar_MapNode AStar_graph[AStar_Height][AStar_Width];
AStar_Close astar_close[AStar_Height][AStar_Width];
AStar_Close *AStar_start;
unsigned char AStar_srcX, AStar_srcY, AStar_dstX, AStar_dstY;
bool maze[compression_map_x][compression_map_y];
bool printclose;
unsigned char step;
char plansteps[50];
unsigned char motionSteps=0;


unsigned char startMotionStep=0;
bool b_startMotionStep=false;
unsigned char AStarMotionNumber=0;
bool astar_coll=false;
bool return_origin_collision;
bool close_edged_map=false;
unsigned char closeedgedmap=0;
bool detection_close_edge=false;
bool detection_close=false;
int8_t close_edge_min_x,close_edge_max_x,close_edge_min_y,close_edge_max_y,close_r_edge_min_x,close_r_edge_min_y,close_l_edge_max_x,close_l_edge_max_y;

unsigned char a_star_collision_total=0;
unsigned char Astarmarkingobstacle=0;


const AStarPoint astar_dir[8] ={
    {0, 1},
    {1, 1},
    {1, 0},
    {1, -1},
    {0, -1},
    {-1, -1},
    {-1, 0},
    {-1, 1}
};



//***********stuck status******************************
int judgment_Stuck_status = 0;
int judgment_Stuck_status_x = 0;
int judgment_Stuck_status_y = 0;
int judgment_Stuck_status_yaw = 0;
bool stuck_x = false;
bool stuck_y = false;
bool stuck=false;

int x_error = 0;
int y_error = 0;


//############MCU#############################
static CleanStrategyB cleanstrategy;
static POSE current_pose;
int32_t global_pose_x;
int32_t global_pose_y;
int32_t map_current_pose_x;
int32_t map_current_pose_y;

//static int Yaw;
static short speed_pid_cnt_goback  = 0;
static short speed_pid_cnt_default = 0;
static short speed_pid_cnt_realgo = 0;
static short speed_pid_cnt_ir = 0;
static short speed_pid_cnt_spin = 0;
static double Last_cmd_angular_velocity = 0;
static unsigned char* IRSensorData_StrategyB;


uint8_t check_sensor_cnt = 0;
const uint8_t time_out_flag = 1;
const uint8_t battery_out_flag = 2;
const uint8_t collision_error = 3;
const uint8_t cliff_error = 4;
const uint8_t infra_collision_error = 5;
const uint8_t imu_error = 5;
const uint8_t motorLeft_error = 6;
const uint8_t motorRight_error = 7;
const uint8_t motorVacuum_error = 8;
const uint8_t motorRolling_error = 9;
const uint8_t motorSide_error = 10;

uint8_t collision_error_cnt = 0;
uint8_t cliff_error_cnt = 0;
uint8_t imu_error_cnt = 0;
uint16_t infra_collision_error_cnt = 0;
static uint8_t time_battery_return_origin_statues = 1;


uint8_t return_charge_station_flag = 0;

bool boolleaksweep=false;
bool leftboolleaksweep=false;
bool more_map=false;
bool edge_dilemma=false;
bool x_more_map=false;
bool y_more_map=false;
bool y_less_map=false;
bool right_map_extreme=false;
signed char x_more_positive_start=0;
signed char y_more_positive_start=0;
bool astar_origin=false;
short thephi=0;
short origin_thephi=0;
signed char reverse_moremap=0;
bool b_reverse_moremap=false;

bool  infrared_collision=false;
unsigned char delimma_edge=0;

bool mapstopupdate=false;
bool closeedgesmap=false;


extern signed char Under_extreme_point_x_index;
extern signed char Under_extreme_point_y_index;
extern signed char On_extreme_point_x_index;
extern signed char On_extreme_point_y_index;
extern signed char Left_Under_extreme_point_x_index;
extern signed char Left_Under_extreme_point_y_index;
extern signed char Left_On_extreme_point_x_index;
extern signed char Left_On_extreme_point_y_index;

extern float adcRealTime[10];
extern char Under_extreme_point_x[10];
extern char Under_extreme_point_y[10];
extern char On_extreme_point_x[10];
extern char On_extreme_point_y[10];
extern char Left_Under_extreme_point_x[10];
extern char Left_On_extreme_point_x[10];
extern char Left_Under_extreme_point_y[10];
extern char Left_On_extreme_point_y[10];

extern GridMap gridmap;
extern CLIFFADCVALUE cliff_valueB;

extern uint8_t  work_mode;

static void right_edge_judgment_repeat(void)
{
	if(adcRealTime[9]>100&&adcRealTime[9]<1500){
		delimma_edge=0;
		linear_velocity = 200;
		angular_velocity = 0;
	}
	if(adcRealTime[9]>=1500){
		delimma_edge=0;
		linear_velocity = 200;
		angular_velocity = 10;
	}
	if(adcRealTime[9]<100){
		if(delimma_edge<10){
			delimma_edge++;
			linear_velocity = 200;
			angular_velocity = -10;
		}else{
			linear_velocity = 100;
			angular_velocity = -20;
		}
	}
}

static void left_edge_judgment_repeat(void)
{
	if(adcRealTime[8]>100&&adcRealTime[8]<1500){
		delimma_edge=0;
		linear_velocity = 200;
		angular_velocity = 0;
	}
	if(adcRealTime[8]>=1500){
		delimma_edge=0;
		linear_velocity = 200;
		angular_velocity = -10;
	}
	if(adcRealTime[8]<100){
		if(delimma_edge<10){
			delimma_edge++;
			linear_velocity = 200;
			angular_velocity = 10;
		}else{
			linear_velocity = 100;
			angular_velocity = 20;
		}
	}
}


double my_abs(double x){
    if(x<0){
        x=-x;
    }
    return x;
}


static void sendvelocity(double* linear_velocity,double* angular_velocity)
{
/*角速度范围：5~60 度/秒*/
/*线速度范围：20~250 毫米/秒*/
    short leftVelocity,rightVelocity;
    double linear_velocity_IR,cmd_linear_velocity,cmd_angular_velocity;
    cmd_linear_velocity = *linear_velocity;
    cmd_angular_velocity = *angular_velocity;
	
	if(Last_cmd_angular_velocity != cmd_angular_velocity)
	{
		bsp_PidClear(MotorLeft);
        bsp_PidClear(MotorRight);
//        speed_pid_cnt_default = 1;
//        speed_pid_cnt_ir = 1;
//        speed_pid_cnt_goback = 1;
		speed_pid_cnt_realgo =1;
		speed_pid_cnt_spin = 1;
	}
	
	if(cmd_angular_velocity > 10 || cmd_angular_velocity < -10)
	{
		if(cmd_angular_velocity < 0)
		{
                if(speed_pid_cnt_spin == 1){
                    bsp_PidClear(MotorLeft);
                    bsp_PidClear(MotorRight);
                }
                if(speed_pid_cnt_spin <=20) speed_pid_cnt_spin +=1;
                if(speed_pid_cnt_spin >20)  speed_pid_cnt_spin  =20; 
                cmd_angular_velocity = speed_pid_cnt_spin*0.05*(cmd_angular_velocity+10)-10;	
        }
        if(cmd_angular_velocity > 0)
		{
                if(speed_pid_cnt_spin == 1) {
                    bsp_PidClear(MotorLeft);
                    bsp_PidClear(MotorRight);
                }
                if(speed_pid_cnt_spin <=20) speed_pid_cnt_spin +=1;
                if(speed_pid_cnt_spin >20)  speed_pid_cnt_spin  =20; 
                cmd_angular_velocity = speed_pid_cnt_spin*0.05*(cmd_angular_velocity-10) + 10;	
		}
	}
	
    if(cmd_linear_velocity == 0 && cmd_angular_velocity == 0){
        leftVelocity = 0;
        rightVelocity = 0;
        bsp_PidClear(MotorLeft);
        bsp_PidClear(MotorRight);
        speed_pid_cnt_default = 1;
        speed_pid_cnt_ir = 1;
        speed_pid_cnt_goback = 1;
		speed_pid_cnt_realgo =1;
		speed_pid_cnt_spin = 1;
    }
    else{
        if(cmd_linear_velocity != 0 && (cmd_linear_velocity >100 || cmd_linear_velocity <-100)){
            if(IRSensorData_StrategyB[0]||IRSensorData_StrategyB[1]|| IRSensorData_StrategyB[2]||IRSensorData_StrategyB[3]||IRSensorData_StrategyB[4]||IRSensorData_StrategyB[5]||IRSensorData_StrategyB[6]||IRSensorData_StrategyB[7]){
                if(cmd_linear_velocity==200){
                    if(IRSensorData_StrategyB[2]||IRSensorData_StrategyB[3]||IRSensorData_StrategyB[4]){
                        cmd_linear_velocity = 0.5*cmd_linear_velocity;
                    }
                }
                else{
                    cmd_linear_velocity = 0.5*cmd_linear_velocity;
                }
                linear_velocity_IR = cmd_linear_velocity;            
            }
            if(cmd_linear_velocity <0){
                cmd_linear_velocity = -160;
                if(speed_pid_cnt_goback == 1){
                    bsp_PidClear(MotorLeft);
                    bsp_PidClear(MotorRight);
                }
                if(speed_pid_cnt_goback <=50) speed_pid_cnt_goback +=1;
                if(speed_pid_cnt_goback >50)  speed_pid_cnt_goback  =50; 
                cmd_linear_velocity = speed_pid_cnt_goback*0.02*(cmd_linear_velocity+40)-40;	
            }
            else{
                speed_pid_cnt_goback = 1;
            }
            if(cmd_linear_velocity == long_stra_vel || cmd_linear_velocity == 200 ){
                if(speed_pid_cnt_default == 1) {
                    bsp_PidClear(MotorLeft);
                    bsp_PidClear(MotorRight);
                }
                if(speed_pid_cnt_default <=20) speed_pid_cnt_default +=1;
                if(speed_pid_cnt_default >20)  speed_pid_cnt_default  =20; 
                cmd_linear_velocity = speed_pid_cnt_default*0.05*(cmd_linear_velocity-40) + 40;	
            }
            else{
                speed_pid_cnt_default = 1;
            }
			if(cmd_linear_velocity == real_gostaright_vel){
                if(speed_pid_cnt_realgo == 1) {
                    bsp_PidClear(MotorLeft);
                    bsp_PidClear(MotorRight);
                }
                if(speed_pid_cnt_realgo <=20) speed_pid_cnt_realgo +=1;
                if(speed_pid_cnt_realgo >20)  speed_pid_cnt_realgo  =20; 
                cmd_linear_velocity = speed_pid_cnt_realgo*0.05*(cmd_linear_velocity-40) + 40;	
            }
            else{
                speed_pid_cnt_realgo = 1;
            }
            if(cmd_linear_velocity ==  linear_velocity_IR){
                if(speed_pid_cnt_ir == 1){
                    bsp_PidClear(MotorLeft);
                    bsp_PidClear(MotorRight);
                }
                if(speed_pid_cnt_ir <=20) speed_pid_cnt_ir +=1;
                if(speed_pid_cnt_ir >20)  speed_pid_cnt_ir  =20; 
                if(cmd_linear_velocity >  40) cmd_linear_velocity = speed_pid_cnt_ir*0.05*(0.7*cmd_linear_velocity-40) + 40;	
                if(cmd_linear_velocity < -40) cmd_linear_velocity = speed_pid_cnt_ir*0.05*(0.7*cmd_linear_velocity+40) - 40;	
            }
            else{
                speed_pid_cnt_ir = 1;
            }            
        }
        else{
            speed_pid_cnt_default = 1;
            speed_pid_cnt_ir = 1;
            speed_pid_cnt_goback = 1;
			speed_pid_cnt_realgo =1;
        }
        leftVelocity = (short)((0.5*(2*cmd_linear_velocity*0.001 - Deg2Rad(cmd_angular_velocity)*WHEEL_LENGTH))* 1000);
        rightVelocity = (short)((0.5*(2*cmd_linear_velocity*0.001 + Deg2Rad(cmd_angular_velocity)*WHEEL_LENGTH))* 1000);
        
        if(leftVelocity>0){
            if(leftVelocity<50) leftVelocity = 50;
            if(leftVelocity>300) leftVelocity = 300;
        }
        if(rightVelocity>0){
            if(rightVelocity<50) rightVelocity = 50;
            if(rightVelocity>300) rightVelocity = 300;
        }
        if(leftVelocity<0){
            if(leftVelocity>-50) leftVelocity = -50;
            if(leftVelocity<-300) leftVelocity = -300;
        }
        if(rightVelocity<0){
            if(rightVelocity>-50) rightVelocity = -50;
            if(rightVelocity<-300) rightVelocity = -300;
        }
    }
    
    
	
	int roll = (int)bsp_IMU_GetData(ROLL);
    if(my_abs(roll)<172){
        if(leftVelocity>0&&rightVelocity>0){
            if(roll >= -165 && roll <= -100){
                leftVelocity=150;
                rightVelocity=150;
            }
            else if(roll > -170 && roll <= -100){
                leftVelocity=100;
                rightVelocity=100;
            }
            else if(roll > -172 && roll <= -100){
                leftVelocity=30;
                rightVelocity=30;
            }
            else if(roll >= 100 && roll <= 165){
                leftVelocity=300;
                rightVelocity=300;
            }
            else if(roll >= 100 && roll < 170){
                leftVelocity=280;
                rightVelocity=280;
            }
            else if(roll >= 100 && roll < 172){
                leftVelocity=250;
                rightVelocity=250;
            }
            else                                      
            {
            }
        }
    }
//	    if(my_abs(roll)<=175){
//        if(leftVelocity>0&&rightVelocity>0){
//            if(roll >= -165 && roll <= -100){
//                leftVelocity=200;
//                rightVelocity=200;
//            }
//            else if(roll >= -170 && roll <= -100){
//                leftVelocity=100;
//                rightVelocity=100;
//            }
//            else if(roll >= -175 && roll <= -100){
//                leftVelocity=30;
//                rightVelocity=30;
//            }
//            else if(roll >= 100 && roll <= 165){
//                leftVelocity=300;
//                rightVelocity=300;
//            }
//            else if(roll >= 100 && roll < 170){
//                leftVelocity=280;
//                rightVelocity=280;
//            }
//            else if(roll >= 100 && roll < 172){
//                leftVelocity=250;
//                rightVelocity=250;
//            }
//            else                                      
//            {
//            }
//        }
//   }
    bsp_SetMotorSpeed(MotorLeft,bsp_MotorSpeedMM2Pulse(leftVelocity));
    bsp_SetMotorSpeed(MotorRight,bsp_MotorSpeedMM2Pulse(rightVelocity));
	
	Last_cmd_angular_velocity = *angular_velocity;
}


bool isCleanRunning(void){
    return cleanstrategy.isRunning;
}

void bsp_StartUpdateCleanStrategyB(void){
    
    LastCleanTimeStamp = xTaskGetTickCount();
    bsp_ResetCleanStrategyBStatus();
    
//    cleanstrategy.work_step_status = RIGHTRUNNING_WORK_SETP;
//    cleanstrategy.right_running_complete  = 0;
//    cleanstrategy.right_return_origin_complete = 0;
//    cleanstrategy.left_running_complete = 0;
//    cleanstrategy.left_return_origin_complete = 0;
	
    cleanstrategy.action = 0 ;
    cleanstrategy.delay = 0 ;
    cleanstrategy.isRunning = true;
    linear_velocity = 0,angular_velocity = 0;
	RealWorkTime = 0;
	bsp_StartUpdateGridMap();
}


void bsp_ResetCleanStrategyBStatus(void){
    time_battery_return_origin_statues = 1;
    OVERALL_CLEANING_STRATEGY = 0;
    right_running_step_status = 0;
    collision_right_rightrun_step_status = 0;
    collision_left_rightrun_step_status = 0;
    collision_front_rightrun_step_status = 0;
    left_running_step_status = 0;
    collision_right_leftrun_step_status = 0;
    collision_left_leftrun_step_status = 0;
    collision_front_leftrun_step_status = 0;
    stuck_right_run_step=0;
    return_origin_step_status = 0;
    a_star_motion_return_origin_status=0;
    a_star_not_motion_status=0;
    a_star_collision_status=0;
    over_clean_finish=false;
    return_origin_positive_start= 1;
    selectside=0;
    //for  walk edge
    right_walk_edge_status = 0;
    right_reverse_walk_edge_status = 0;
    right_edge_dilemma_status = 0;
    right_forward_boundary_status = 0;
    right_ready_leaking_sweep_status = 0;
    left_walk_edge_status = 0;
    left_reverse_walk_edge_status = 0;
    left_edge_dilemma_status = 0;
    left_forward_boundary_status = 0;
    left_ready_leaking_sweep_status = 0;
    number = 0;
    leakingsweep = 0;
    leakingsweep_x = 0;
    leakingsweep_y = 0;
    leakingsweep_X_interval = 200;
    leakingsweep_Y_interval = 100;
    linear_velocity = 0,angular_velocity = 0;
    //bypass_velocity = 150;
    //for  collision step
    distance_uptate  = 0;
    turn_start_update = 0;
    last_position_x = 0;
    last_position_xx = 0;
    last_position_y = 0;
    last_position_yy = 0;
    b_last_position_yy = false;
    turn_start_x = 0;
    turn_start_y = 0;
    maintain_bow_distance = 0;
    cnt_update = 0;
    bow_continue = false;
    old_bow_continue = false;
    temporary_yaw = 0;
    temporary_close_edge = 0;
    returnorigin = false;
    edge_length_start = 0;
    DelimmaNumber=0;
    clill_start_update=false;
    cliffruningStatus=false;
    FunctionStatus=0;
    
    
    rightmapmin=0;
    rightmapmax=0;
    boolleaksweep=false;
    leftboolleaksweep=false;
    more_map=false;
    edge_dilemma=false;
    x_more_map=false;
    y_more_map=false;
    y_less_map=false;
    right_map_extreme=false;
    x_more_positive_start=0;
    y_more_positive_start=0;
    astar_origin=false;
    thephi=0;
    origin_thephi=0;
    x_error = 0;
    y_error = 0;
    reverse_moremap=0;
    b_reverse_moremap=false;
    mapstopupdate=false;
    
    //CurrentCleanTimeStamp  = 0;
    
    //位姿复位
    bsp_ResetPosArgument();
    //栅格图复位
    //bsp_StartUpdateGridMap();
    
}






void bsp_StopUpdateCleanStrategyB(void){
    cleanstrategy.action = 0 ;
    cleanstrategy.delay = 0 ;
    cleanstrategy.isRunning = false;
    
//    cleanstrategy.work_step_status = 0;
//    cleanstrategy.right_running_complete  = 0;
//    cleanstrategy.right_return_origin_complete = 0;
//    cleanstrategy.left_running_complete = 0;
//    cleanstrategy.left_return_origin_complete = 0;
    
    bsp_SetMotorSpeed(MotorLeft,bsp_MotorSpeedMM2Pulse(0));
    bsp_SetMotorSpeed(MotorRight,bsp_MotorSpeedMM2Pulse(0));
    
    bsp_StopEdgewiseRun();
    bsp_StopVacuum();
    bsp_MotorCleanSetPWM(MotorRollingBrush, CCW , 0);
    bsp_MotorCleanSetPWM(MotorSideBrush, CW , 0);
    
    bsp_ResetCleanStrategyBStatus();
}




void bsp_UpdateCleanStrategyB(int robotX,int robotY,double robotTheta, unsigned char obstacleSignal, int current_wheel_pulse_l, int current_wheel_pulse_r, unsigned char IRSensorData[],CLIFFADCVALUE * cliff_value){
    IRSensorData_StrategyB = IRSensorData;
    current_pose.x = INT_COOR_X + robotX;
    current_pose.y = INT_COOR_Y + robotY;
    current_pose.x =  robotX;
    current_pose.y =  robotY;
    global_pose_x =  robotX;
    global_pose_y =  robotY;
    current_pose.orientation = Rad2Deg(robotTheta)*100;
    
    wheel_pulse_r = current_wheel_pulse_r;
    wheel_pulse_l = current_wheel_pulse_l;
    
    if(cleanstrategy.isRunning)
    {		
        if(clean_strategyB(&current_pose,obstacleSignal) != ALL_CLEAN_COMPLETE)
        {
            //nothing...
        }
        else{
            bsp_SperkerPlay(Song5);
            bsp_StopUpdateCleanStrategyB();
            return_charge_station_flag = 1;
            
        }
    }
    
}	





uint8_t GetReturnChargeStationStatus(void){
    return return_charge_station_flag;
}

void ResetReturnChargeStationStatus(void){
    return_charge_station_flag  = 0;
}

static uint8_t check_sensor(unsigned char obstacleSignal){
    float batteryvoltage;
	//uint16_t motorLeftVoltage,motorRightVoltage,motorVacuumVoltage,motorRollingVoltage,motorSideVoltage,batteryCurrent;
    //	IRSensorData_StrategyB
    //	cliff_valueB
    
    /*如果 处在 上传数据的状态 ， 则屏蔽异异常检测*/
    if(GetCmdStartUpload())
    {
        return 0 ;
    }
    
    check_sensor_cnt++;
    if (check_sensor_cnt >201) check_sensor_cnt = 0;
    
    //工作时间检测
    if(check_sensor_cnt%100){
        
        CurrentCleanTimeStamp = xTaskGetTickCount();
        RealWorkTime = CurrentCleanTimeStamp - LastCleanTimeStamp;
        if(CurrentCleanTimeStamp - LastCleanTimeStamp >CLEAN_WORK_TIME) return time_out_flag;
    }
    
    //电池电量检测
    if(check_sensor_cnt%100){
        //	batteryCurrent = bsp_GetFeedbackVoltage(eBatteryCurrent)*100;
        batteryvoltage = bsp_GetFeedbackVoltage(eBatteryVoltage);
        batteryvoltage = (batteryvoltage * 430 / 66.5) + batteryvoltage + 0.2F; 
        if(batteryvoltage < 13)   //12v-16v
        {
            batteryvoltage = bsp_GetFeedbackVoltage(eBatteryVoltage);
            batteryvoltage = (batteryvoltage * 430 / 66.5) + batteryvoltage + 0.2F; 
            if(batteryvoltage < 13)
            {
                return  battery_out_flag;//battery_out_flag;
            }
        }
		
//		motorLeftVoltage = bsp_GetFeedbackVoltage(eMotorLeft)*1000;
//		if(motorLeftVoltage > 3000)   // 
//        {
//			return  motorLeft_error;// ;
//        }
//		
//		motorRightVoltage = bsp_GetFeedbackVoltage(eMotorRight)*100;
//		if(motorLeftVoltage > 3000)   // 
//        {
//			return  motorRight_error;// ;
//        }
//		motorVacuumVoltage = bsp_GetFeedbackVoltage(eVacuum)*100;
//		if(motorLeftVoltage > 3000)   // 
//        {
//			return  motorVacuum_error;// ;
//        }
//		motorRollingVoltage = bsp_GetFeedbackVoltage(eRollingBrush)*100;
//		if(motorLeftVoltage > 3000)   // 
//        {
//			return  motorRolling_error;// ;
//        }
//		motorSideVoltage = bsp_GetFeedbackVoltage(eSideBrush)*100;
//		if(motorLeftVoltage > 3000)   // 
//        {
//			return  motorSide_error;// ;
//        }
		   
		     
    }
    
    //碰撞异常检测
    if(check_sensor_cnt%20){
        if(obstacleSignal<3)   
        {
            collision_error_cnt++;
            if(collision_error_cnt > 200) //4.5s
            {
                collision_error_cnt = 0;
                return collision_error;
            }
        }else{
            collision_error_cnt = 0;
        }
    }
    //跳崖异常检测
    if(check_sensor_cnt%20){
        if(cliff_valueB.cliffValue0 == 1)   
        {
            cliff_error_cnt++;
            if(cliff_error_cnt >100) 
            {
                cliff_error_cnt = 0;
                return cliff_error;
            }
        }else{
            cliff_error_cnt = 0;
        }
    }
    //陀螺仪异常检测
    if(check_sensor_cnt%50){
        if(bsp_AngleReadRaw() == 0)   
        {
            imu_error_cnt++;
            if(imu_error_cnt >50)
            {
                imu_error_cnt = 0;
                return imu_error;
            }
        }else{
            imu_error_cnt = 0;
        }
    }
#if 0	
    //红外异常检测
    if(check_sensor_cnt%200){
        if(IRSensorData_StrategyB[1] == 1 || IRSensorData_StrategyB[3] == 1 || \
                IRSensorData_StrategyB[5] == 1 || IRSensorData_StrategyB[7] == 1) 
        {
            infra_collision_error_cnt++;
            if(infra_collision_error_cnt>500)
            {
                infra_collision_error_cnt = 0;
                return infra_collision_error;
            }
        }else{
            infra_collision_error_cnt = 0;
        }	
    }
#endif		
    return 0;
    
}

//#################################################################################

uint8_t clean_strategyB(POSE *current_pose,unsigned char obstacleSignal){
    int Yaw;
    Yaw = current_pose->orientation;
    Yaw= Yaw/100;
    uint8_t check_sensor_return_value = 0;

#if  1	
    
    check_sensor_return_value =  check_sensor(obstacleSignal);
    
    if( (check_sensor_return_value < 3 && check_sensor_return_value>0) && time_battery_return_origin_statues)
    {
        time_battery_return_origin_statues = 0;
        
        if(check_sensor_return_value  == time_out_flag )
        {
            bsp_SperkerPlay(Song5); /*返回充电*/
            //log_debug("时间到，返回充电！\n");
            bsp_StopVacuum();
            bsp_MotorCleanSetPWM(MotorRollingBrush, CCW , 0);
            bsp_MotorCleanSetPWM(MotorSideBrush, CCW , 0);
            
            while(bsp_SpeakerIsBusy()){}
        }
        if(check_sensor_return_value  == battery_out_flag)
        {
            bsp_SperkerPlay(Song6);/*电池电量低，请回充*/;
            //log_debug("电池电量低，返回充电！\n");
            bsp_StopVacuum();
            bsp_MotorCleanSetPWM(MotorRollingBrush, CCW , 0);
            bsp_MotorCleanSetPWM(MotorSideBrush, CCW , 0);
            
            while(bsp_SpeakerIsBusy()){}
        }
        linear_velocity = 0;
        angular_velocity = 0;
        //log_debug("开始返回原点，走A*策略！\n");
        over_clean_finish = true;
        bsp_StopEdgewiseRun();
        //selectside='L';
        OVERALL_CLEANING_STRATEGY = RETURN_ORIGIN_WORKING_OVERALL_CLEANING_STRATEGY;
        left_running_step_status = 0;
        FunctionStatus = 0;
    }
    
    if(check_sensor_return_value  == collision_error)
    {
        //log_debug("碰撞异常！\n");
        bsp_StopUpdateCleanStrategyB();
        bsp_SperkerPlay(Song18);/*碰撞开关异常*/
        return 1;
    }
    if(check_sensor_return_value  == cliff_error)
    {
        //log_debug("请擦拭跳崖传感器！\n");
        bsp_StopUpdateCleanStrategyB();
        bsp_SperkerPlay(Song11); /*请擦拭跳崖传感器*/
        return 1;
    }
    if(check_sensor_return_value  == imu_error)
    {
        //log_debug("陀螺仪数据异常！\n");
        bsp_InitAngle(); 
        bsp_StopUpdateCleanStrategyB();
        bsp_SperkerPlay(Song8); /*陀螺仪数据异常*/
        while(bsp_SpeakerIsBusy()){}
        return 1;
    }
    
#endif
	
	{	
		current_pose->x=current_pose->x-x_error;
		current_pose->y=current_pose->y-y_error;
		if(b_reverse_moremap==true){
			if(reverse_moremap==1){
				current_pose->x=current_pose->x+reverse_x_more_map;
			}
			else{
				current_pose->x=current_pose->x-reverse_x_more_map;
			}		
		}
		if(over_clean_finish==false){
			if(x_more_map==true||y_more_map==true){
				if(x_more_map==true){
					if(b_reverse_moremap==false){
						current_pose->x=current_pose->x-x_more_positive_start*half_map_wide;
					}
				}
				if(y_more_map==true){
					current_pose->y=current_pose->y-y_more_positive_start*half_map_wide;
				}
			}
		}
		if (detection_close_edge == true){
			DetectionCloseEdge();
		}
		else if(astar_origin==true){
			AStarReturnOrigin(current_pose, obstacleSignal);
		}
		else if(stuck==true){
			StuckRunStep(current_pose);
			stuck=false;
		}
		else if(boolleaksweep==true){
				leakingsweep =bsp_Right_ReturnExtreme_point(current_pose->x,current_pose->y,Yaw,obstacleSignal);
			boolleaksweep=false;
		}
		else if(leftboolleaksweep==true){
				leakingsweep =bsp_Left_ReturnExtreme_point(current_pose->x,current_pose->y,Yaw,obstacleSignal);
			leftboolleaksweep=false;
		}
		else if(more_map==true){
			MoreMap(current_pose);
			motionSteps=0;
			more_map=false;
		}
		else{
		}
		map_current_pose_x=current_pose->x;
		map_current_pose_y=current_pose->y;
		if(bool_leakingsweep_y==true){
			if(selectside =='R'){
				if(leakingsweep_y_flag+return_origin_distance>current_pose->y){
					bool_leakingsweep_y=false;
				}
			}
			else{
				if(leakingsweep_y_flag-return_origin_distance<current_pose->y){
					bool_leakingsweep_y=false;
				}
			}
		}
	}
	switch (OVERALL_CLEANING_STRATEGY)
    {
    case 0:
        linear_velocity=0;
        angular_velocity=0;
        temporary_wheel_pulse_r=wheel_pulse_r;	
        StartUpdateGridMap();
        ReturnExtreme_point_init();
        if(work_mode == 0 ) 
		{
			OVERALL_CLEANING_STRATEGY = START_OVERALL_CLEANING_STRATEGY;
		}
		if(work_mode == 1 ) 
		{
			bsp_SperkerPlay(Song34);
			bsp_StartEdgewiseRun();
			OVERALL_CLEANING_STRATEGY = EDGEWISERUN_CLEANING_STRATEGY;
		}
        break;
    case START_OVERALL_CLEANING_STRATEGY:
        OVERALL_CLEANING_STRATEGY = RIGHT_RUNNING_WORKING_OVERALL_CLEANING_STRATEGY;
        selectside = 'R';
        break;
    case RIGHT_RUNNING_WORKING_OVERALL_CLEANING_STRATEGY:
        FunctionStatus = RightRunningWorkStep(current_pose, obstacleSignal);
        if (1 == FunctionStatus)
        {
            if( my_abs(temporary_wheel_pulse_r-wheel_pulse_r)>10000){
                if(closeedgesmap==true){
                    OVERALL_CLEANING_STRATEGY = CLOSE_EDGED_MAP_OVERALL_CLEANING_STRATEGY;
                    right_running_step_status = 0;
                    FunctionStatus = 0;
                    break;
                }
                else{
                    selectside = 'L';
                    OVERALL_CLEANING_STRATEGY  = A_STAR_RETURN_ORIGIN_WORKING_OVERALL_CLEANING_STRATEGY;
                    right_running_step_status = 0;
                    FunctionStatus = 0;
                    if(y_more_map==true){
                        y_less_map=true;
                    }
                    ForceReturnOriginTimeStamp = xTaskGetTickCount();
                }
            }
            else{
                if(edge_dilemma==false){
                    edge_dilemma=true;
                    right_running_step_status =0;
                    OVERALL_CLEANING_STRATEGY =START_OVERALL_CLEANING_STRATEGY;
                    FunctionStatus=0;
                }
                else{
                    if(closeedgesmap==true){
                        OVERALL_CLEANING_STRATEGY = CLOSE_EDGED_MAP_OVERALL_CLEANING_STRATEGY;
                        right_running_step_status = 0;
                        FunctionStatus = 0;
                        break;
                    }
                    else{
                        selectside='L';
                        over_clean_finish=false;
                        OVERALL_CLEANING_STRATEGY =A_STAR_RETURN_ORIGIN_WORKING_OVERALL_CLEANING_STRATEGY;
                        right_running_step_status =0;
                        FunctionStatus=0;
                    }
                }
            }
            break;
        }
        break;
    case A_STAR_RETURN_ORIGIN_WORKING_OVERALL_CLEANING_STRATEGY:
        if(astar_origin==false){
            linear_velocity=0;
            angular_velocity=0;
            astar_origin=true;
        }
        else{
            OVERALL_CLEANING_STRATEGY =A_STAR_MOTION_RETURN_ORIGIN_WORKING_OVERALL_CLEANING_STRATEGY;
            astar_origin=false;
        }
        break;
    case A_STAR_MOTION_RETURN_ORIGIN_WORKING_OVERALL_CLEANING_STRATEGY:
        if(y_less_map==true){
            current_pose->y=global_pose_y;
            current_pose->y=current_pose->y-y_error;
        }
        if(over_clean_finish == true){
            current_pose->x=global_pose_x;
            current_pose->y=global_pose_y;
            current_pose->x=current_pose->x-x_error;
            current_pose->y=current_pose->y-y_error;
        }
        FunctionStatus =AStarMotionReturnOrigin(current_pose, obstacleSignal);
        if (1 == FunctionStatus)
        {
            a_star_motion_return_origin_status = 0;
            OVERALL_CLEANING_STRATEGY = RETURN_ORIGIN_WORKING_OVERALL_CLEANING_STRATEGY;
            FunctionStatus = 0;
            break;
        }
        if (2 == FunctionStatus)
        {
            a_star_motion_return_origin_status = 0;
            OVERALL_CLEANING_STRATEGY = RETURN_ORIGIN_WORKING_OVERALL_CLEANING_STRATEGY;
            FunctionStatus = 0;
            break;
        }
        if (3 == FunctionStatus)
        {
            a_star_motion_return_origin_status = 0;
            OVERALL_CLEANING_STRATEGY = RETURN_ORIGIN_WORKING_OVERALL_CLEANING_STRATEGY;
            FunctionStatus = 0;
            break;
        }
        if (4 == FunctionStatus)
        {
            OVERALL_CLEANING_STRATEGY = RETURN_ORIGIN_WORKING_OVERALL_CLEANING_STRATEGY;
            a_star_motion_return_origin_status = 0;
            FunctionStatus = 0;
            break;
        }
        break;
    case A_STAR_NOT_RETURN_ORIGIN_WORKING_OVERALL_CLEANING_STRATEGY:
        FunctionStatus = AStarNotMotionReturnOrigin(current_pose, obstacleSignal);
        if (1 == FunctionStatus)
        {
            OVERALL_CLEANING_STRATEGY = A_STAR_RETURN_ORIGIN_WORKING_OVERALL_CLEANING_STRATEGY;
            FunctionStatus = 0;
            break;
        }
        break;
    case A_STAR_COLLISION_RETURN_ORIGIN_WORKING_OVERALL_CLEANING_STRATEGY:
        FunctionStatus = AStarCollision(current_pose, obstacleSignal);
        if (1 == FunctionStatus)
        {
            OVERALL_CLEANING_STRATEGY = A_STAR_RETURN_ORIGIN_WORKING_OVERALL_CLEANING_STRATEGY;
            FunctionStatus = 0;
            break;
        }
        if (2 == FunctionStatus)
        {
            OVERALL_CLEANING_STRATEGY = A_STAR_MOTION_RETURN_ORIGIN_WORKING_OVERALL_CLEANING_STRATEGY;
            FunctionStatus = 0;
            break;
        }
        break;
    case RETURN_ORIGIN_WORKING_OVERALL_CLEANING_STRATEGY:
        FunctionStatus = ForceReturnOrigin(current_pose, obstacleSignal);
        if (1 == FunctionStatus)
        {
            if (over_clean_finish == true)
            {
                if(b_reverse_moremap==true){
                    linear_velocity = 0;
                    angular_velocity = 0;
                    OVERALL_CLEANING_STRATEGY = 0;
                    edge_dilemma=false;
                    return_origin_step_status = 0;
                    FunctionStatus = 0;
                    return 0;
                }
                else{
                    if(reverse_moremap!=0){
                        over_clean_finish=false;
                        edge_dilemma=false;
                        x_more_map=true;
                        y_more_map=false;
                        y_less_map=false; 						
                        right_running_step_status =0;
                        OVERALL_CLEANING_STRATEGY =0;
                        FunctionStatus=0;									
                        b_reverse_moremap=true;
                        break;
                    }
                    else{
                        linear_velocity = 0;
                        angular_velocity = 0;
                        OVERALL_CLEANING_STRATEGY = 0;
                        edge_dilemma=false;
                        return_origin_step_status = 0;
                        FunctionStatus = 0;
                        return 0;	
                    }
                }						               
            }
            else
            {						
                temporary_wheel_pulse_r=wheel_pulse_r;
                OVERALL_CLEANING_STRATEGY =  LEFT_RUNNING_WORKING_OVERALL_CLEANING_STRATEGY;
                linear_velocity = 0;
                angular_velocity = 0;
                y_more_map=false;
                edge_dilemma=false;
                return_origin_step_status = 0;
                FunctionStatus = 0;
                if(y_less_map==false){
                    RightMapExtreme();
                }
                if(y_less_map==true){
                    LessMap();
                    right_map_extreme=true;
                    y_less_map=false;
                    break;
                } 											
            }				
            break;
        }
        if(2==FunctionStatus){
            OVERALL_CLEANING_STRATEGY=0;
            return_origin_step_status = 0;
            return 2;
        }
        break;
    case LEFT_RUNNING_WORKING_OVERALL_CLEANING_STRATEGY:
        if(right_map_extreme==true){
            RightMapExtreme();
            right_map_extreme=false;
        }
        FunctionStatus = LeftRunningWorkStep(current_pose, obstacleSignal);
        if (1 == FunctionStatus)
        {
            if( my_abs(temporary_wheel_pulse_r-wheel_pulse_r)>10000){
                OVERALL_CLEANING_STRATEGY = CLOSE_EDGED_MAP_OVERALL_CLEANING_STRATEGY;
                left_running_step_status = 0;
                over_clean_finish = true;
                FunctionStatus = 0;
                break;
            }else{
                if(edge_dilemma==false){
                    edge_dilemma=true;
                    OVERALL_CLEANING_STRATEGY=LEFT_RUNNING_WORKING_OVERALL_CLEANING_STRATEGY;
                    left_running_step_status = 0;
                    FunctionStatus=0;
                }
                else{
                    OVERALL_CLEANING_STRATEGY = CLOSE_EDGED_MAP_OVERALL_CLEANING_STRATEGY;
                    left_running_step_status = 0;
                    FunctionStatus = 0;
                }
                break;
            }  
        }
        break;
    case CLOSE_EDGED_MAP_OVERALL_CLEANING_STRATEGY:
        FunctionStatus = CloseEdgedMap(current_pose, obstacleSignal);
        if(1 == FunctionStatus)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            over_clean_finish = true;
            close_edge_map_run_step_status=0;
            ForceReturnOriginTimeStamp = xTaskGetTickCount();
            OVERALL_CLEANING_STRATEGY =A_STAR_RETURN_ORIGIN_WORKING_OVERALL_CLEANING_STRATEGY;
            FunctionStatus = 0;
            break;
        }
        if(2 == FunctionStatus)
        {
            close_edge_map_run_step_status=0;
            OVERALL_CLEANING_STRATEGY = START_OVERALL_CLEANING_STRATEGY;
            FunctionStatus = 0;
            break;
        }
        if(3 == FunctionStatus)
        {
            close_edge_map_run_step_status=0;
            OVERALL_CLEANING_STRATEGY =  LEFT_RUNNING_WORKING_OVERALL_CLEANING_STRATEGY;
            FunctionStatus = 0;
            break;
        }
        break;
	case EDGEWISERUN_CLEANING_STRATEGY:
		{
			 return 1;//nothing...
		}
		//break;
    default:
        break;
    }
    //if(OVERALL_CLEANING_STRATEGY!=EDGEWISERUN_CLEANING_STRATEGY) 
	sendvelocity(&linear_velocity, &angular_velocity);
    return 1;
}

//####################################################           RIGHT        #####    
unsigned char  RightRunningWorkStep(POSE *current_pose, unsigned char obstacleSignal){
    int Yaw;
    unsigned char complete_flag = 0;
	unsigned char i=0,j=0;
    Yaw = current_pose->orientation;
    Yaw = Yaw /100;
    switch (right_running_step_status)
    {
    case 0:
		linear_velocity = 0;
        angular_velocity = 0;
        right_running_step_status = GOSTR_RIGHTRUN_STEP;
        break;
    case GOSTR_RIGHTRUN_STEP:
        if((&cliff_valueB)->cliffValue0 == 1){
            collision_front_rightrun_step_status = 0;
            right_running_step_status = COLLISION_FRONT_RIGHTRUN_STEP;
            break;
        }
        else if (front_obstacle == obstacleSignal)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            if(my_abs(leakingsweep_x-current_pose->x)>500&&my_abs(leakingsweep_y-current_pose->y)>100){
				if((my_abs(current_pose->x)<half_map_wide-GRIDHEIGHT)&&(my_abs(current_pose->y)<half_map_wide-GRIDHEIGHT)){
                leakingsweep_x=current_pose->x;
                leakingsweep_y=current_pose->y;
                boolleaksweep=true;
                break;
				}
            }
            if (0 != leakingsweep){
                right_running_step_status = LEAKING_SWEEP_RIGHTRUN_STEP;
                break;
            }
            else{
                collision_front_rightrun_step_status = 0;
                right_running_step_status = COLLISION_FRONT_RIGHTRUN_STEP;
                break;
            }
        }
        else if (right_obstacle == obstacleSignal)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            if(my_abs(leakingsweep_x-current_pose->x)>500&&my_abs(leakingsweep_y-current_pose->y)>100){
				if((my_abs(current_pose->x)<half_map_wide-GRIDHEIGHT)&&(my_abs(current_pose->y)<half_map_wide-GRIDHEIGHT)){
                leakingsweep_x=current_pose->x;
                leakingsweep_y=current_pose->y;
                boolleaksweep=true;
                break;
				}
            }
            if (0 != leakingsweep)
            {
                right_running_step_status = LEAKING_SWEEP_RIGHTRUN_STEP;
                break;
            }
            else
            {
                collision_right_rightrun_step_status = 0;
                right_running_step_status = COLLISION_RIGHT_RIGHTRUN_STEP;
                break;
            }
        }
        else if (left_obstacle == obstacleSignal)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            if(my_abs(leakingsweep_x-current_pose->x)>500&&my_abs(leakingsweep_y-current_pose->y)>100){
				if((my_abs(current_pose->x)<half_map_wide-GRIDHEIGHT)&&(my_abs(current_pose->y)<half_map_wide-GRIDHEIGHT)){
                leakingsweep_x=current_pose->x;
                leakingsweep_y=current_pose->y;
                boolleaksweep=true;
                break;
				}
            }
            if (0 != leakingsweep)
            {
                right_running_step_status = LEAKING_SWEEP_RIGHTRUN_STEP;
                break;
            }
            else
            {
                collision_left_rightrun_step_status = 0;
                right_running_step_status = COLLISION_LEFT_RIGHTRUN_STEP;
                break;
            }
        }
        else if (my_abs(current_pose->x) > half_map_wide - 2*GRIDHEIGHT)
        {
            right_running_step_status = FORWARD_BOUNDARY_RIGHTRUN_STEP;
            break;
        }
        else if (my_abs(current_pose->y) > half_map_wide - 2*GRIDHEIGHT)
        {
            right_running_step_status = FORWARD_BOUNDARY_RIGHTRUN_STEP;
            break;
        }
		else if(my_abs(Yaw) >= 90 && my_abs(Yaw) < 170){
            if (Yaw > 0){
                linear_velocity = 0;
                angular_velocity = turn_vel;
            }
            else{
                linear_velocity = 0;
                angular_velocity = -turn_vel;
            }
            break;
        }
        else if(my_abs(Yaw) >= 90 && my_abs(Yaw) < 171){
            if (Yaw > 0){
                linear_velocity = correction_big_straight_vel;
                angular_velocity = correction_big_turn_vel;
            }
            else{
                linear_velocity = correction_big_straight_vel;
                angular_velocity = -correction_big_turn_vel;
            }
            break;
        }
        else if (my_abs(Yaw) >= 90 && my_abs(Yaw) < 178){
            if (Yaw > 0){
                linear_velocity = correction_straight_vel;
                angular_velocity = correction_turn_vel;
            }
            else{
                linear_velocity = correction_straight_vel;
                angular_velocity = -correction_turn_vel;
            }
            break;
        }
        else if (my_abs(Yaw) < 90 && my_abs(Yaw) > 10){
            if (Yaw > 0){
                linear_velocity = 0;
                angular_velocity = -turn_vel;
            }
            else{
                linear_velocity = 0;
                angular_velocity = turn_vel;
            }
            break;
        }
        else if (my_abs(Yaw) < 90 && my_abs(Yaw) > 9)
        {
            if (Yaw > 0){
                linear_velocity = correction_big_straight_vel;
                angular_velocity = -correction_big_turn_vel;
            }
            else{
                linear_velocity = correction_big_straight_vel;
                angular_velocity = correction_big_turn_vel;
            }
            break;
        }
        else if (my_abs(Yaw) < 90 && my_abs(Yaw) > 2){
            if (Yaw > 0){
                linear_velocity = correction_straight_vel;
                angular_velocity = -correction_turn_vel;
            }
            else{
                linear_velocity = correction_straight_vel;
                angular_velocity = correction_turn_vel;
            }
            break;
        }
        else if((adcRealTime[2]>200||adcRealTime[3]>200||adcRealTime[4]>200)&&(adcRealTime[1]<100)&&(adcRealTime[5]<100)){
            linear_velocity = 0;
            angular_velocity = 0;
			i=(current_pose->x+half_map_wide)/GRIDWIDTH;
			j=(current_pose->y+half_map_wide)/GRIDWIDTH;
			if(my_abs(Yaw)>90){
				if(i>2&&i<GRIDWIDTH&&j>0&&j<GRIDWIDTH-1){
					gridmap.map[i-2][j-1]=0;
					gridmap.map[i-2][j]=0;
					gridmap.map[i-2][j+1]=0;
					gridmap.map[i-3][j-1]=0;
					gridmap.map[i-3][j]=0;
					gridmap.map[i-3][j+1]=0;
				}
			}
			else{
				if(i>2&&i<GRIDWIDTH&&j>0&&j<GRIDWIDTH-1){
					gridmap.map[i+2][j-1]=0;
					gridmap.map[i+2][j]=0;
					gridmap.map[i+2][j+1]=0;
					gridmap.map[i+3][j-1]=0;
					gridmap.map[i+3][j]=0;
					gridmap.map[i+3][j+1]=0;
				}
			}
            if(my_abs(leakingsweep_x-current_pose->x)>500&&my_abs(leakingsweep_y-current_pose->y)>100){
				if((my_abs(current_pose->x)<half_map_wide-GRIDHEIGHT)&&(my_abs(current_pose->y)<half_map_wide-GRIDHEIGHT)){
                leakingsweep_x=current_pose->x;
                leakingsweep_y=current_pose->y;
                boolleaksweep=true;
                break;
				}
            }
            if (0 != leakingsweep){
                right_running_step_status = LEAKING_SWEEP_RIGHTRUN_STEP;
                break;
            }
            else{
                collision_front_rightrun_step_status = 0;
                right_running_step_status = COLLISION_FRONT_RIGHTRUN_STEP;
                break;
            }
        }		
        else if((adcRealTime[0]>500||adcRealTime[1]>100||adcRealTime[2]>100)&&(adcRealTime[3]<25)&&(adcRealTime[4]<25)&&(adcRealTime[5]<25)&&my_abs(Yaw)<90){
            linear_velocity = 0;
            angular_velocity = 0;
		i=(current_pose->x+half_map_wide)/GRIDWIDTH;
		j=(current_pose->y+half_map_wide)/GRIDWIDTH;
		if(i>0&&i<97&&j>=1&&j<GRIDWIDTH-1){
			gridmap.map[i+2][j]=0;
			gridmap.map[i+3][j]=0;
            gridmap.map[i+2][j+1]=0;
            gridmap.map[i+3][j+1]=0;
		}
            if(my_abs(leakingsweep_x-current_pose->x)>500&&my_abs(leakingsweep_y-current_pose->y)>100){
				if((my_abs(current_pose->x)<half_map_wide-GRIDHEIGHT)&&(my_abs(current_pose->y)<half_map_wide-GRIDHEIGHT)){
                leakingsweep_x=current_pose->x;
                leakingsweep_y=current_pose->y;
                boolleaksweep=true;
                break;
				}
            }
            if (0 != leakingsweep)
            {
                right_running_step_status = LEAKING_SWEEP_RIGHTRUN_STEP;
                break;
            }
            else
            {
                collision_left_rightrun_step_status = 0;
                right_running_step_status = COLLISION_LEFT_RIGHTRUN_STEP;
                break;		
            }
        }
        else if((adcRealTime[0]>500||adcRealTime[1]>200||adcRealTime[2]>200)&&(adcRealTime[3]<100)&&(adcRealTime[4]<100)&&(adcRealTime[5]<100)&&my_abs(Yaw)>90){
            linear_velocity = 0;
            angular_velocity = 0;
		i=(current_pose->x+half_map_wide)/GRIDWIDTH;
		j=(current_pose->y+half_map_wide)/GRIDWIDTH;
		if(i>2&&i<GRIDWIDTH&&j>0&&j<GRIDWIDTH){
			gridmap.map[i-2][j]=0;
			gridmap.map[i-3][j]=0;
            gridmap.map[i-2][j-1]=0;
            gridmap.map[i-3][j-1]=0;
		}
            if(my_abs(leakingsweep_x-current_pose->x)>500&&my_abs(leakingsweep_y-current_pose->y)>100){
				if((my_abs(current_pose->x)<half_map_wide-GRIDHEIGHT)&&(my_abs(current_pose->y)<half_map_wide-GRIDHEIGHT)){
                leakingsweep_x=current_pose->x;
                leakingsweep_y=current_pose->y;
                boolleaksweep=true;
                break;
				}
            }
            if (0 != leakingsweep)
            {
                right_running_step_status = LEAKING_SWEEP_RIGHTRUN_STEP;
                break;
            }
            else
            {
                collision_left_rightrun_step_status = 0;
                right_running_step_status = COLLISION_LEFT_RIGHTRUN_STEP;
                break;		
            }
        }
        else if((adcRealTime[6]>500||adcRealTime[5]>200||adcRealTime[4]>200)&&(adcRealTime[3]<100)&&(adcRealTime[2]<100)&&(adcRealTime[1]<100)&&my_abs(Yaw)<90){
            linear_velocity = 0;
            angular_velocity = 0;
		i=(current_pose->x+half_map_wide)/GRIDWIDTH;
		j=(current_pose->y+half_map_wide)/GRIDWIDTH;
		if(i>0&&i<97&&j>0&&j<GRIDWIDTH){
			gridmap.map[i+2][j]=0;
			gridmap.map[i+3][j]=0;
            gridmap.map[i+2][j-1]=0;
            gridmap.map[i+3][j-1]=0;
		}
            if(my_abs(leakingsweep_x-current_pose->x)>500&&my_abs(leakingsweep_y-current_pose->y)>100){
				if((my_abs(current_pose->x)<half_map_wide-GRIDHEIGHT)&&(my_abs(current_pose->y)<half_map_wide-GRIDHEIGHT)){
                leakingsweep_x=current_pose->x;
                leakingsweep_y=current_pose->y;
                boolleaksweep=true;
                break;
				}
            }
            if (0 != leakingsweep)
            {
                right_running_step_status = LEAKING_SWEEP_RIGHTRUN_STEP;
                break;
            }
            else
            {
                collision_right_rightrun_step_status = 0;
                right_running_step_status = COLLISION_RIGHT_RIGHTRUN_STEP;
                break;
            }
        }
        else if((adcRealTime[6]>500||adcRealTime[5]>100||adcRealTime[4]>100)&&(adcRealTime[3]<25)&&(adcRealTime[2]<25)&&(adcRealTime[1]<25)&&my_abs(Yaw)>90){
            linear_velocity = 0;
            angular_velocity = 0;
		i=(current_pose->x+half_map_wide)/GRIDWIDTH;
		j=(current_pose->y+half_map_wide)/GRIDWIDTH;
		if(i>2&&i<GRIDWIDTH&&j>0&&j<GRIDWIDTH-1){
			gridmap.map[i-2][j]=0;
			gridmap.map[i-3][j]=0;
            gridmap.map[i-2][j+1]=0;
            gridmap.map[i-3][j+1]=0;
		}
            if(my_abs(leakingsweep_x-current_pose->x)>500&&my_abs(leakingsweep_y-current_pose->y)>100){
				if((my_abs(current_pose->x)<half_map_wide-GRIDHEIGHT)&&(my_abs(current_pose->y)<half_map_wide-GRIDHEIGHT)){
                leakingsweep_x=current_pose->x;
                leakingsweep_y=current_pose->y;
                boolleaksweep=true;
                break;
				}
            }
            if (0 != leakingsweep)
            {
                right_running_step_status = LEAKING_SWEEP_RIGHTRUN_STEP;
                break;
            }
            else
            {
                collision_right_rightrun_step_status = 0;
                right_running_step_status = COLLISION_RIGHT_RIGHTRUN_STEP;
                break;
            }
        }
        else{
            linear_velocity = real_gostaright_vel;
            angular_velocity = 0;
        }
        break;
    case FORWARD_BOUNDARY_RIGHTRUN_STEP:
        FunctionStatus = ForwardBoundaryRightRunStep(current_pose, obstacleSignal);
        if(FunctionStatus==1||FunctionStatus==3)
        {
            if((FunctionStatus==1&&x_more_map==false)||(FunctionStatus==3&&y_more_map==false)){
                if(FunctionStatus==1){
                    if(reverse_moremap==1){
                        if(current_pose->x>0){
                        }else{
                            ReturnExtreme_point_init();
                            motionSteps=1;
                            more_map=true;
                        }
                    }
                    else if(reverse_moremap==-1){
                        if(current_pose->x>0){
                            ReturnExtreme_point_init();
                            motionSteps=1;
                            more_map=true;	
                        }else{
                        }
                    }
                    else{
                        motionSteps=1;
                        more_map=true;
                    }
                }
                else{
                    ReturnExtreme_point_init();
                    motionSteps=3;
                    more_map=true;
                }						
            }
            else{
                if(FunctionStatus==3){
                    right_running_step_status = 0;
                    right_forward_boundary_status=0;
                    FunctionStatus=0;
                    complete_flag=1;
                    break;
                }
                if(FunctionStatus==1){
                    right_running_step_status = GOSTR_RIGHTRUN_STEP;
                    right_forward_boundary_status=0;
                    FunctionStatus=0;
                    break;
                }
            }
            right_running_step_status = 0;
            right_forward_boundary_status=0;
            FunctionStatus=0;
            break;
        }
        if (FunctionStatus == 2)
        {
            right_running_step_status = STUCK_FORWARD_BOUNDARY_RIGHT_RUNSTEP;
            right_forward_boundary_status = 0;
        }
        break;
    case STUCK_FORWARD_BOUNDARY_RIGHT_RUNSTEP:
        FunctionStatus = StuckRightRunStep(current_pose, obstacleSignal);
        if (FunctionStatus == 1)
        {
            right_running_step_status = GOSTR_RIGHTRUN_STEP;
            stuck_right_run_step = 0;
        }
        break;
    case COLLISION_RIGHT_RIGHTRUN_STEP:
        FunctionStatus = CollisionRightRightRunStep(current_pose, obstacleSignal);
        if (1 == FunctionStatus)
        {
            right_running_step_status = GOSTR_RIGHTRUN_STEP;
            collision_right_rightrun_step_status = 0;
            FunctionStatus = 0;
            break;
        }
        if (2 == FunctionStatus)
        {
			right_running_step_status = RIGHTWALKEDGE;	
            collision_right_rightrun_step_status = 0;
            FunctionStatus = 0;
            break;
        }
        break;
    case RIGHTWALKEDGE:
        FunctionStatus = RightWalkEdge(current_pose, obstacleSignal);
        if (1 == FunctionStatus)
        {
            right_running_step_status = GOSTR_RIGHTRUN_STEP;
            right_walk_edge_status = 0;
            FunctionStatus = 0;
            break;
        }
        if (2 == FunctionStatus)
        {
            right_running_step_status = RIGHTEDGEDILEMMA;
            right_walk_edge_status = 0;
            FunctionStatus = 0;
            break;
        }
        if (3 == FunctionStatus)
        {
            right_running_step_status = 0;
            right_walk_edge_status = 0;
            FunctionStatus = 0;
            complete_flag = 1;
            break;
        }
        break;
    case RIGHTEDGEDILEMMA:
        FunctionStatus=RightEdgeDilemma(current_pose,obstacleSignal);
        if(1==FunctionStatus){
            right_running_step_status = GOSTR_RIGHTRUN_STEP;
            right_edge_dilemma_status=0;
            FunctionStatus=0;
            break;
        }
        if(2==FunctionStatus){
            if(bool_leakingsweep_y==true){
                right_running_step_status = GOSTR_RIGHTRUN_STEP;
                right_edge_dilemma_status=0;
                FunctionStatus=0;
                break;
            }
            else{
                right_running_step_status =0;
                right_edge_dilemma_status=0;
                FunctionStatus=0;
                complete_flag=1;
                break;
            }
            
        }
        break;
    case COLLISION_LEFT_RIGHTRUN_STEP:
        FunctionStatus = CollisionLeftRightRunStep(current_pose, obstacleSignal);
        if (1 == FunctionStatus)
        {
            right_running_step_status = GOSTR_RIGHTRUN_STEP;
            collision_right_rightrun_step_status = 0;
            FunctionStatus = 0;
            break;
        }
        if (2 == FunctionStatus)
        {		
            right_running_step_status = RIGHTREVERSEWALKEDGE;
            collision_right_rightrun_step_status = 0;
            FunctionStatus = 0;
            break;
        }
        break;
    case RIGHTREVERSEWALKEDGE:
        FunctionStatus = RightReverseWalkEdge(current_pose, obstacleSignal);
        if (1 == FunctionStatus)
        {
            right_running_step_status = GOSTR_RIGHTRUN_STEP;
            right_reverse_walk_edge_status = 0;
            FunctionStatus = 0;
            break;
        }
        if (2 == FunctionStatus)
        {
            right_running_step_status = RIGHTEDGEDILEMMA;
            right_reverse_walk_edge_status = 0;
            FunctionStatus = 0;
            break;
        }
        if (3 == FunctionStatus)
        {
            right_running_step_status = 0;
            right_reverse_walk_edge_status = 0;
            FunctionStatus = 0;
            complete_flag = 1;
            break;
        }
        break;
    case COLLISION_FRONT_RIGHTRUN_STEP:
        FunctionStatus = CollisionFrontRightRunStep(current_pose, obstacleSignal);
        if (1 == FunctionStatus)
        {
            right_running_step_status = GOSTR_RIGHTRUN_STEP;
            collision_right_rightrun_step_status = 0;
            FunctionStatus = 0;
            break;
        }
        if (2 == FunctionStatus)
        {
            right_running_step_status = RIGHTWALKEDGE;
            collision_right_rightrun_step_status = 0;
            FunctionStatus = 0;
            break;
        }
        if (3 == FunctionStatus)
        {
            right_running_step_status = RIGHTREVERSEWALKEDGE;
            collision_right_rightrun_step_status = 0;
            FunctionStatus = 0;
            break;
        }
        break;
    case LEAKING_SWEEP_RIGHTRUN_STEP:
        if (RightReadyLeakingSweep(current_pose, obstacleSignal))
        {
            right_running_step_status = GOSTR_RIGHTRUN_STEP;
            right_ready_leaking_sweep_status = 0;
            leakingsweep = 0;
            break;
        }
        break;
    default:
        break;
    }
    return complete_flag;
}



unsigned char  CollisionRightRightRunStep(POSE *current_pose,unsigned char obstacleSignal){
    int Yaw;
    unsigned char complete_flag=0;
    Yaw = current_pose->orientation;
    Yaw= Yaw/100;
    switch(collision_right_rightrun_step_status)
    {
    case 0:
        if(turn_start_update == 0)
        {
            if(obstacleSignal==none_obstacle){
                obstacleSignal=right_obstacle;
                last_position_x = current_pose->x;
                last_position_y = current_pose->y;
                if(my_abs(Yaw) > 90){
                    collision_right_rightrun_step_status =GOSTR_BYPASS_LOOP_CR_DRYM;
                }
                else{
                    collision_right_rightrun_step_status = DIR_RIGHT_YAW_LESS_ABS90_CRRRS;
                }
                infrared_collision=true;
                break;
            } 
            turn_start_x = current_pose->x;
            turn_start_y = current_pose->y;
            turn_start_update = 1;
        }
        linear_velocity = -long_stra_vel;
        angular_velocity = 0;
        if (my_abs(turn_start_x - current_pose->x) > collision_backward_distance || my_abs(turn_start_y - current_pose->y) > collision_backward_distance)
        {
            collision_right_rightrun_step_status = GOBACK_DISTANCE_CRRRS;
            turn_start_update = 0;
            break;
        }
        break;
    case GOBACK_DISTANCE_CRRRS:
        if(turn_start_update == 0)
        {
            turn_start_x = current_pose->x;
            turn_start_y = current_pose->y;
            turn_start_update = 1;
        }
        linear_velocity = -long_stra_vel;
        angular_velocity = 0;
        
        if (my_abs(turn_start_x - current_pose->x) > side_backward_distance)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            if (my_abs(Yaw) < 90)
            {
                collision_right_rightrun_step_status = DIR_RIGHT_YAW_LESS_ABS90_CRRRS;
            }
            else
            {
                collision_right_rightrun_step_status = DIR_RIGHT_YAW_MORE_ABS90_CRRRS;
            }
            turn_start_update = 0;
            break;
        }
        break;
    case DIR_RIGHT_YAW_LESS_ABS90_CRRRS:
        linear_velocity = 0;
        angular_velocity = 0;
        last_position_y = current_pose->y;
        collision_right_rightrun_step_status  = TURN_CLOCK_TARGET_YAW_NEG57_CR_DRYL;
        break;
        
    case TURN_CLOCK_TARGET_YAW_NEG57_CR_DRYL:
        linear_velocity = 0;
        angular_velocity = -turn_vel;
        if (Yaw < -85)//-90
        {
            collision_right_rightrun_step_status  = GOSTR_YAW_EQUAL_NEG57_CR_DRYL;
            break;
        }
        if (obstacleSignal!=none_obstacle||(&cliff_valueB)->cliffValue0 == 1)
        {
            collision_right_rightrun_step_status  = TURN_CLOCK_TARGET_YAW_NEG57_COLLISION_CR_DRYL;
            break;
        }
        break;
    case TURN_CLOCK_TARGET_YAW_NEG57_COLLISION_CR_DRYL:
        if(turn_start_update == 0)
        {
            turn_start_x = current_pose->x;
            turn_start_y = current_pose->y;
            turn_start_update = 1;
        }
        linear_velocity = -long_stra_vel;
        angular_velocity = 0;
        if (my_abs(turn_start_x - current_pose->x) > second_turn_backward_distance || my_abs(turn_start_y - current_pose->y) > second_turn_backward_distance)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            turn_start_update = 0;
            collision_right_rightrun_step_status = TURN_CLOCK_TARGET_YAW_NEG57_CR_DRYL;
            break;
        }
        break;
    case GOSTR_YAW_EQUAL_NEG57_CR_DRYL:      
        if (obstacleSignal!=none_obstacle||(&cliff_valueB)->cliffValue0 == 1)
        {
			linear_velocity = 0;
            angular_velocity = 0;
            collision_right_rightrun_step_status = RIGHT_WALK_EDGE_CR_DRYL;
            break;		
        }
        if (my_abs(last_position_y - current_pose->y) > lateral_move_distance)
        {
            collision_right_rightrun_step_status = TURN_CLOCK_TARGET_YAW_NEG82_CR_DRYL;
            break;
        }
        linear_velocity = long_stra_vel;
        angular_velocity = 0;
        break;
    case TURN_CLOCK_TARGET_YAW_NEG82_CR_DRYL:
        if (Yaw < -90) //-95
        {
            collision_right_rightrun_step_status = GOSTR_YAW_EQUAL_NEG82_CR_DRYL;
            break;
        }
        if (obstacleSignal!=none_obstacle||(&cliff_valueB)->cliffValue0 == 1)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            collision_right_rightrun_step_status = TURN_CLOCK_TARGET_YAW_NEG82_COLLISION_CR_DRYL;
            break;
        }
        linear_velocity = 0;
        angular_velocity = -turn_vel;
        break;
    case TURN_CLOCK_TARGET_YAW_NEG82_COLLISION_CR_DRYL:
        if(turn_start_update == 0)
        {
            turn_start_x = current_pose->x;
            turn_start_y = current_pose->y;
            turn_start_update = 1;
        }
        linear_velocity = -long_stra_vel;
        angular_velocity = 0;
        if (my_abs(turn_start_x - current_pose->x) > turn_backward_distance || my_abs(turn_start_y - current_pose->y) > turn_backward_distance)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            turn_start_update = 0;
            collision_right_rightrun_step_status = TURN_CLOCK_TARGET_YAW_NEG82_CR_DRYL;
            break;
        }
        break;
    case GOSTR_YAW_EQUAL_NEG82_CR_DRYL:
        linear_velocity = long_stra_vel;
        angular_velocity = 0;
        if (obstacleSignal!=none_obstacle||(&cliff_valueB)->cliffValue0 == 1)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            collision_right_rightrun_step_status = RIGHT_WALK_EDGE_CR_DRYL;
            break;
        }
        if (my_abs(last_position_y - current_pose->y) > lateral_move_distance)
        {
            collision_right_rightrun_step_status = TURN_CLOCK_TARGET_YAW_ABS173_CR_DRYL;
            break;
        }
        break;
    case RIGHT_WALK_EDGE_CR_DRYL:
        infrared_collision=false;
        collision_right_rightrun_step_status = 0;
        complete_flag = 2;
        break;
    case TURN_CLOCK_TARGET_YAW_ABS173_CR_DRYL:
        linear_velocity = 0;
        angular_velocity = -turn_vel;
        if (my_abs(Yaw) > 176) //173
        {
            linear_velocity = 0;
            angular_velocity = 0;
            collision_right_rightrun_step_status = COMPLETE_CR_DRYL;
            break;
        }
        if (obstacleSignal!=none_obstacle||(&cliff_valueB)->cliffValue0 == 1)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            collision_right_rightrun_step_status = TURN_CLOCK_TARGET_YAW_ABS173_COLLISION_CR_DRYL;
            break;
        }
        break;
    case TURN_CLOCK_TARGET_YAW_ABS173_COLLISION_CR_DRYL:
        if(turn_start_update == 0)
        {
            turn_start_x = current_pose->x;
            turn_start_y = current_pose->y;
            turn_start_update = 1;
        }
        linear_velocity = -long_stra_vel;
        angular_velocity = 0;
        if (my_abs(turn_start_x - current_pose->x) > collision_backward_distance || my_abs(turn_start_y - current_pose->y) > collision_backward_distance)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            turn_start_update = 0;
            collision_right_rightrun_step_status = TURN_CLOCK_TARGET_YAW_ABS173_CR_DRYL;
            break;
        }
        break;
    case COMPLETE_CR_DRYL:
        infrared_collision=false;
        collision_right_rightrun_step_status = 0;
        complete_flag = 1;
        break;
    case DIR_RIGHT_YAW_MORE_ABS90_CRRRS:
        linear_velocity = 0;
        angular_velocity = 0;
        collision_right_rightrun_step_status = TURN_CCLCOK_TARGET_YAW_ABS153_CR_DRYM;
        break;
    case TURN_CCLCOK_TARGET_YAW_ABS153_CR_DRYM:
        linear_velocity = 0;
        angular_velocity = turn_vel;
        if (my_abs(Yaw) < 153)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            collision_right_rightrun_step_status = GOSTR_YAW_EQUAL_ABS153_CR_DRYM;
            break;
        }
        if (obstacleSignal!=none_obstacle||(&cliff_valueB)->cliffValue0 == 1)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            collision_right_rightrun_step_status = TURN_CCLCOK_TARGET_YAW_ABS153_COLLISION_CR_DRYM;
            break;
        }
        break;
    case TURN_CCLCOK_TARGET_YAW_ABS153_COLLISION_CR_DRYM:
        if(turn_start_update == 0)
        {
            turn_start_x = current_pose->x;
            turn_start_y = current_pose->y;
            turn_start_update = 1;
        }
        linear_velocity = -long_stra_vel;
        angular_velocity = 0;
        if (my_abs(turn_start_x - current_pose->x) > second_turn_backward_distance || my_abs(turn_start_y - current_pose->y) > second_turn_backward_distance)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            turn_start_update = 0;
            collision_right_rightrun_step_status = TURN_CCLCOK_TARGET_YAW_ABS153_CR_DRYM;
            break;
        }
        break;
    case GOSTR_YAW_EQUAL_ABS153_CR_DRYM:
        cnt_update = 0;
        last_position_y = current_pose->y;
        last_position_xx = current_pose->x;
        collision_right_rightrun_step_status = GOSTR_BYPASS_CR_DRYM;
        break;
    case GOSTR_BYPASS_CR_DRYM:
        last_position_x = current_pose->x;
        cnt_update +=1;
        linear_velocity = long_stra_vel;
        angular_velocity = 0;
        if(cnt_update > 1&&my_abs(last_position_xx - current_pose->x)<20&&my_abs(last_position_y - current_pose->y)<20)
        {
            cnt_update = 0;
            last_position_y=current_pose->y;
            collision_right_rightrun_step_status = TURN_CLOCK_TARGET_YAW_NEG90_CR_DRYL;
            break;
        }
        collision_right_rightrun_step_status = GOSTR_BYPASS_LOOP_CR_DRYM;
        break;
    case TURN_CLOCK_TARGET_YAW_NEG90_CR_DRYL:
        linear_velocity = 0;
        angular_velocity = turn_vel;
        if (my_abs(Yaw)<90)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            collision_right_rightrun_step_status = GOSTR_YAW_EQUAL_NEG90_CR_DRYL;
            break;
        }
        if (obstacleSignal!=none_obstacle||(&cliff_valueB)->cliffValue0 == 1)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            collision_right_rightrun_step_status = TURN_CLOCK_TARGET_YAW_NEG90_COLLISION_CR_DRYL;
            break;
        }
        break;
    case TURN_CLOCK_TARGET_YAW_NEG90_COLLISION_CR_DRYL:
        if(turn_start_update == 0)
        {
            turn_start_x = current_pose->x;
            turn_start_y = current_pose->y;
            turn_start_update = 1;
        }
        linear_velocity = -long_stra_vel;
        angular_velocity = 0;
        if (my_abs(turn_start_x - current_pose->x) > turn_backward_distance || my_abs(turn_start_y - current_pose->y) > turn_backward_distance)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            turn_start_update = 0;
            collision_right_rightrun_step_status = TURN_CLOCK_TARGET_YAW_NEG90_CR_DRYL;
            break;
        }
        break;
    case GOSTR_YAW_EQUAL_NEG90_CR_DRYL:
        linear_velocity = long_stra_vel;
        angular_velocity = 0;
        if (obstacleSignal!=none_obstacle||(&cliff_valueB)->cliffValue0 == 1)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            right_running_step_status = RIGHTREVERSEWALKEDGE;
            collision_right_rightrun_step_status=0;
            break;
        }
        if (my_abs(last_position_y - current_pose->y) > lateral_move_distance / 2)
        {
            collision_right_rightrun_step_status = MORE_TRY_BREAK_BYPASS_CR_DRYM;
            break;
        }
        break;
        
    case GOSTR_BYPASS_LOOP_CR_DRYM:
        if (obstacleSignal == right_obstacle)
        {
            collision_right_rightrun_step_status = RIGHT_COLLISION_BYPASS_CR_DRYM;
			infrared_collision=false;
            break;
        }
        
        if (my_abs(last_position_y - current_pose->y) > close_edge)
        {
            collision_right_rightrun_step_status = GOSTR_BYPASS_BOW_CONTINUE_CR_DRYM;
            break;
        }
        if(obstacleSignal == front_obstacle || obstacleSignal == left_obstacle||(&cliff_valueB)->cliffValue0 == 1){
            last_position_y=current_pose->y;
            collision_right_rightrun_step_status = TURN_CLOCK_TARGET_YAW_NEG90_CR_DRYL;
            break;
        }
        if (last_position_y < current_pose->y - 1)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            collision_right_rightrun_step_status = GOSTR_BYPASS_BOW_CONTINUE_EXIT_CR_DRYM;
            break;
        }
        if (my_abs(current_pose->x) > half_map_wide)
        {
            collision_right_rightrun_step_status = COMPLETE_CR_DRYM;
            break;
        }
        if (my_abs(Yaw) < 90)
        {
            collision_right_rightrun_step_status = COMPLETE_CR_DRYM;
            break;
        }
        if(infrared_collision==true&&my_abs(Yaw)>105){
            if(adcRealTime[6]>500|| adcRealTime[5]>50||adcRealTime[4]>50){
                linear_velocity = 100;
                angular_velocity = 57;
                if(adcRealTime[4]>50){
                    linear_velocity = 50;
                    angular_velocity = turn_vel;
                }
            }
            else{
				right_edge_judgment_repeat();
//                if(adcRealTime[9]>100&&adcRealTime[9]<1500){
//                    delimma_edge=0;
//                    linear_velocity = 200;
//                    angular_velocity = 0;
//                }
//                if(adcRealTime[9]>=1500){
//                    delimma_edge=0;
//                    linear_velocity = 200;
//                    angular_velocity = 10;
//                }
//                if(adcRealTime[9]<100){
//                    if(delimma_edge<10){
//                        delimma_edge++;
//                        linear_velocity = 200;
//                        angular_velocity = -10;
//                    }else{
//                        linear_velocity = 100;
//                        angular_velocity = -20;
//                    }
//                }
            }
        }
        else{
            if (my_abs(last_position_x - current_pose->x) > lateral_move_distance / 3){
                infrared_collision=true;
				last_position_x=current_pose->x;
                collision_right_rightrun_step_status = GOSTR_X_MORE_ONE_THIRD_LATERALDIS_BYPASS_CR_DRYM;
                break;
            }
            linear_velocity = long_stra_vel;
            angular_velocity = 0;
        }
        break;
        
    case GOSTR_BYPASS_BOW_CONTINUE_CR_DRYM:
        if (obstacleSignal!=none_obstacle||(&cliff_valueB)->cliffValue0 == 1)
        {
            collision_right_rightrun_step_status = GOSTR_BYPASS_BOW_CONTINUE_COLLISION_CR_DRYM;
            break;
        }
        if (my_abs(Yaw) < 8)
        {
            bow_continue = true;
            collision_right_rightrun_step_status = GOSTR_BYPASS_BOW_CONTINUE_EXIT_CR_DRYM;
            break;
        }
        linear_velocity = 0;
        angular_velocity = turn_vel;
        break;
        
    case GOSTR_BYPASS_BOW_CONTINUE_COLLISION_CR_DRYM:
        if(turn_start_update == 0)
        {
            turn_start_x = current_pose->x;
            turn_start_y = current_pose->y;
            turn_start_update = 1;
        }
        linear_velocity = -long_stra_vel;
        angular_velocity = 0;
        if (my_abs(turn_start_x - current_pose->x) > second_turn_backward_distance ||my_abs(turn_start_y - current_pose->y) > second_turn_backward_distance)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            bow_continue = true;
            collision_right_rightrun_step_status = GOSTR_BYPASS_BOW_CONTINUE_CR_DRYM;
            turn_start_update = 0;
            break;
        }
        break;
    case GOSTR_BYPASS_BOW_CONTINUE_EXIT_CR_DRYM:
        collision_right_rightrun_step_status = COMPLETE_CR_DRYM;
        break;
    case RIGHT_COLLISION_BYPASS_CR_DRYM:
        if(turn_start_update == 0)
        {
            turn_start_x = current_pose->x;
            turn_start_y = current_pose->y;
            turn_start_update = 1;
        }
        linear_velocity = -long_stra_vel;
        angular_velocity = 0;
        if(my_abs(turn_start_x - current_pose->x) > collision_backward_distance || my_abs(turn_start_y - current_pose->y) > collision_backward_distance){
            linear_velocity = 0;
            angular_velocity = 0;
            turn_start_update = 0;
            temporary_yaw = Yaw;
            if(my_abs(Yaw) > 165){
                collision_right_rightrun_step_status = TURN_CCLOCK_TARGET_YAW_MORE_ABS165_CR_DRYM;
            }
            else{
                collision_right_rightrun_step_status = TURN_CCLOCK_TARGET_YAW_LESS_ABS165_CR_DRYM;
            }
            break;
        }
        break;
    case TURN_CCLOCK_TARGET_YAW_MORE_ABS165_CR_DRYM:
        if(obstacleSignal!=none_obstacle||(&cliff_valueB)->cliffValue0 == 1){
            linear_velocity = 0;
            angular_velocity = 0;
            collision_right_rightrun_step_status = TURN_CCLOCK_TARGET_YAW_MORE_ABS165_COLLISION_CR_DRYM;
            break;
        }
        if (my_abs(Yaw) < 165)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            collision_right_rightrun_step_status = GOSTR_BYPASS_CR_DRYM;
            break;
        }
        if (my_abs(Yaw) < 90)
        {
            collision_right_rightrun_step_status = COMPLETE_CR_DRYM;
            break;
        }
        linear_velocity = 0;
        angular_velocity = turn_vel;
        break;
    case TURN_CCLOCK_TARGET_YAW_MORE_ABS165_COLLISION_CR_DRYM:
        if(turn_start_update == 0)
        {
            turn_start_x = current_pose->x;
            turn_start_y = current_pose->y;
            turn_start_update = 1;
        }
        linear_velocity = -long_stra_vel;
        angular_velocity = 0;
        if (my_abs(turn_start_x - current_pose->x) > collision_backward_distance || my_abs(turn_start_y - current_pose->y) > collision_backward_distance)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            collision_right_rightrun_step_status = TURN_CCLOCK_TARGET_YAW_MORE_ABS165_CR_DRYM;
            turn_start_update = 0;
            break;
        }
        break;
    case TURN_CCLOCK_TARGET_YAW_LESS_ABS165_CR_DRYM:
        if (obstacleSignal!=none_obstacle||(&cliff_valueB)->cliffValue0 == 1)
        {
            collision_right_rightrun_step_status = TURN_CCLOCK_TARGET_YAW_LESS_ABS165_COLLISION_CR_DRYM;
            break;
        }
        if (my_abs(Yaw - temporary_yaw) > 15)
        {
            collision_right_rightrun_step_status = GOSTR_BYPASS_CR_DRYM;
            break;
        }
        if (my_abs(Yaw) < 90)
        {
            collision_right_rightrun_step_status = COMPLETE_CR_DRYM;
            break;
        }
        linear_velocity = 0;
        angular_velocity = turn_vel;
        break;
    case TURN_CCLOCK_TARGET_YAW_LESS_ABS165_COLLISION_CR_DRYM:
        if(turn_start_update == 0)
        {
            turn_start_x = current_pose->x;
            turn_start_y = current_pose->y;
            turn_start_update = 1;
        }
        linear_velocity = -long_stra_vel;
        angular_velocity = 0;
        if (my_abs(turn_start_x - current_pose->x) > collision_backward_distance || my_abs(turn_start_y - current_pose->y) > collision_backward_distance)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            collision_right_rightrun_step_status = TURN_CCLOCK_TARGET_YAW_LESS_ABS165_CR_DRYM;
            turn_start_update = 0;
            break;
        }
        break;
    case GOSTR_X_MORE_ONE_THIRD_LATERALDIS_BYPASS_CR_DRYM:
        if (my_abs(Yaw) > 150)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            temporary_yaw = Yaw;
            collision_right_rightrun_step_status = TURN_CLOCK_TARGET_YAW_ABS150_CR_DRYM;
        }
        else if (my_abs(Yaw) < 150 && Yaw < 0)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            temporary_yaw = Yaw;
            collision_right_rightrun_step_status = TURN_CLOCK_YAW_ADD_ABS30_CR_DRYM;
        }
        else{
            linear_velocity = 0;
            angular_velocity = 0;
            collision_right_rightrun_step_status = GOSTR_BYPASS_CR_DRYM;
        }
        break;
    case TURN_CLOCK_TARGET_YAW_ABS150_CR_DRYM:
        if (my_abs(Yaw) < 150)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            collision_right_rightrun_step_status = GOSTR_BYPASS_CR_DRYM;
            break;
        }
        if (obstacleSignal!=none_obstacle||(&cliff_valueB)->cliffValue0 == 1)
        {
            collision_right_rightrun_step_status = TURN_CLOCK_TARGET_YAW_ABS150_COLLISION_CR_DRYM;
        }
        linear_velocity = 0;
        angular_velocity = -turn_vel;
        break;
    case TURN_CLOCK_TARGET_YAW_ABS150_COLLISION_CR_DRYM:
        if(turn_start_update == 0)
        {
            turn_start_x = current_pose->x;
            turn_start_y = current_pose->y;
            turn_start_update = 1;
        }
        linear_velocity = -long_stra_vel;
        angular_velocity = 0;
        if (my_abs(turn_start_x - current_pose->x) > collision_backward_distance || my_abs(turn_start_y - current_pose->y) > collision_backward_distance)
        {
            
            linear_velocity = 0;
            angular_velocity = 0;
            turn_start_update = 0;
            temporary_yaw = Yaw;
            if (my_abs(Yaw) > 165) collision_right_rightrun_step_status = TURN_CCLOCK_TARGET_YAW_MORE_ABS165_CR_DRYM;
            else collision_right_rightrun_step_status = TURN_CCLOCK_TARGET_YAW_LESS_ABS165_CR_DRYM;
            break;
        }
        break;
    case TURN_CLOCK_YAW_ADD_ABS30_CR_DRYM:
        if (my_abs(Yaw - temporary_yaw) > 30)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            collision_right_rightrun_step_status = GOSTR_BYPASS_CR_DRYM;
            break;
        }
        if (obstacleSignal!=none_obstacle||(&cliff_valueB)->cliffValue0 == 1)
        {
            collision_right_rightrun_step_status = TURN_CLOCK_YAW_ADD_ABS30_COLLISION_CR_DRYM;
        }
        linear_velocity = 0;
        angular_velocity = -turn_vel;
        break;
    case TURN_CLOCK_YAW_ADD_ABS30_COLLISION_CR_DRYM:
        if(turn_start_update == 0)
        {
            turn_start_x = current_pose->x;
            turn_start_y = current_pose->y;
            turn_start_update = 1;
        }
        linear_velocity = -long_stra_vel;
        angular_velocity = 0;
        if (my_abs(turn_start_x - current_pose->x) > collision_backward_distance || my_abs(turn_start_y - current_pose->y) > collision_backward_distance)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            turn_start_update = 0;
            temporary_yaw = Yaw;
            if (my_abs(Yaw) > 165) collision_right_rightrun_step_status = TURN_CCLOCK_TARGET_YAW_MORE_ABS165_CR_DRYM;
            else collision_right_rightrun_step_status = TURN_CCLOCK_TARGET_YAW_LESS_ABS165_CR_DRYM;
            break;
        }
        break;
    case MORE_TRY_BREAK_BYPASS_CR_DRYM:
        if (obstacleSignal!=none_obstacle||(&cliff_valueB)->cliffValue0 == 1)
        {
            collision_right_rightrun_step_status = GOSTR_BYPASS_BOW_CONTINUE_COLLISION_CR_DRYM;
            bow_continue = true;
            break;
        }
        if (my_abs(Yaw) < 5)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            bow_continue = true;
            collision_right_rightrun_step_status = GOSTR_BYPASS_BOW_CONTINUE_EXIT_CR_DRYM;
            break;
        }
        linear_velocity = 0;
        angular_velocity = turn_vel;
        break;
    case COMPLETE_CR_DRYM:
        infrared_collision=false;
        collision_right_rightrun_step_status = 0;
        complete_flag = 1;
        break;
    }
    return complete_flag;
}



unsigned char  CollisionLeftRightRunStep(POSE *current_pose,unsigned char obstacleSignal){
    int Yaw;
    unsigned char complete_flag=0;
    Yaw = current_pose->orientation;
    Yaw= Yaw/100;
    switch(collision_left_rightrun_step_status)
    {
    case 0:	
        if(turn_start_update == 0)
        {
            if(obstacleSignal==none_obstacle){
                obstacleSignal=left_obstacle;
                last_position_x = current_pose->x;
                last_position_y = current_pose->y;
                if(my_abs(Yaw) < 90){
                    collision_left_rightrun_step_status =GOSTR_BYPASS_LOOP_CL_DRYL;
                }
                else{
                    collision_left_rightrun_step_status =DIR_RIGHT_YAW_MORE_ABS90_CLRRS;
                }
                infrared_collision=true;
                break;
            } 
            turn_start_x = current_pose->x;
            turn_start_y = current_pose->y;
            turn_start_update = 1;
        }
        linear_velocity = -long_stra_vel;
        angular_velocity = 0;
        if (my_abs(turn_start_x - current_pose->x) > collision_backward_distance || my_abs(turn_start_y - current_pose->y) > collision_backward_distance)
        {
            collision_left_rightrun_step_status = GOBACK_DISTANCE_CLRRS;
            turn_start_update = 0;
        }
        break;
    case  GOBACK_DISTANCE_CLRRS:
        if(turn_start_update == 0)
        {
            turn_start_x = current_pose->x;
            turn_start_y = current_pose->y;
            turn_start_update = 1;
        }
        linear_velocity = -long_stra_vel;
        angular_velocity = 0;
        
        if (my_abs(turn_start_x - current_pose->x) > side_backward_distance)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            if (my_abs(Yaw) < 90)
            {
                collision_left_rightrun_step_status = DIR_RIGHT_YAW_LESS_ABS90_CLRRS;
            }
            else
            {
                collision_left_rightrun_step_status = DIR_RIGHT_YAW_MORE_ABS90_CLRRS;
            }
            turn_start_update = 0;
            break;
        }
        break;
    case  DIR_RIGHT_YAW_LESS_ABS90_CLRRS:
        linear_velocity = 0;
        angular_velocity = 0;
        collision_left_rightrun_step_status = TURN_CLOCK_TARGET_YAW_ABS30_CL_DRYL;
        break;
    case  TURN_CLOCK_TARGET_YAW_ABS30_CL_DRYL:
        linear_velocity = 0;
        angular_velocity = -turn_vel;
        if (my_abs(Yaw) >30)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            collision_left_rightrun_step_status = GOSTR_YAW_MORE_ABS30_CL_DRYL;
            break;
        }
        if (obstacleSignal!=none_obstacle||(&cliff_valueB)->cliffValue0 == 1)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            collision_left_rightrun_step_status = TURN_CLOCK_TARGET_YAW_ABS30_COLLISION_CL_DRYL;
            break;
        }
        break;
    case  TURN_CLOCK_TARGET_YAW_ABS30_COLLISION_CL_DRYL:
        if(turn_start_update == 0)
        {
            turn_start_x = current_pose->x;
            turn_start_y = current_pose->y;
            turn_start_update = 1;
        }
        linear_velocity = -long_stra_vel;
        angular_velocity = 0;
        if (my_abs(turn_start_x - current_pose->x) > second_turn_backward_distance || my_abs(turn_start_y - current_pose->y) > second_turn_backward_distance)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            turn_start_update = 0;
            collision_left_rightrun_step_status = TURN_CLOCK_TARGET_YAW_ABS30_CL_DRYL;
            break;
        }
        break;
    case  GOSTR_YAW_MORE_ABS30_CL_DRYL:
        cnt_update = 0;
        last_position_y = current_pose->y;
        last_position_xx = current_pose->x;
        collision_left_rightrun_step_status = GOSTR_BYPASS_CL_DRYL;
        break;
    case  GOSTR_BYPASS_CL_DRYL:
        last_position_x = current_pose->x;
        cnt_update +=1;
        linear_velocity = long_stra_vel;
        angular_velocity = 0;
        if(cnt_update > 1&&my_abs(last_position_xx - current_pose->x)<20&&my_abs(last_position_y - current_pose->y)<20)
        {
            cnt_update = 0;
            last_position_y = current_pose->y;
            collision_left_rightrun_step_status = TURN_CCLOCK_TARGET_YAW_ABS90_CL_DRYM;
            break;
        }
        collision_left_rightrun_step_status = GOSTR_BYPASS_LOOP_CL_DRYL;
        break;
        
    case  TURN_CCLOCK_TARGET_YAW_ABS90_CL_DRYM:
        linear_velocity = 0;
        angular_velocity =-turn_vel;
        if (my_abs(Yaw)>90)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            collision_left_rightrun_step_status = GOSTR_YAW_EQUAL_ABS90_CL_DRYM;
            break;
        }
        if (obstacleSignal !=none_obstacle||(&cliff_valueB)->cliffValue0 == 1)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            collision_left_rightrun_step_status = TURN_CCLOCK_TAEGET_YAW_ABS90_COLLISION_CL_DRYM;
            break;
        }
        break;
    case  TURN_CCLOCK_TAEGET_YAW_ABS90_COLLISION_CL_DRYM:
        if(turn_start_update == 0)
        {
            turn_start_x = current_pose->x;
            turn_start_y = current_pose->y;
            turn_start_update = 1;
        }
        linear_velocity = -long_stra_vel;
        angular_velocity = 0;
        if (my_abs(turn_start_x - current_pose->x) > turn_backward_distance || my_abs(turn_start_y - current_pose->y) > turn_backward_distance)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            turn_start_update = 0;
            collision_left_rightrun_step_status = TURN_CCLOCK_TARGET_YAW_ABS90_CL_DRYM;
            break;
        }
        break;
    case  GOSTR_YAW_EQUAL_ABS90_CL_DRYM:
        linear_velocity = long_stra_vel;
        angular_velocity = 0;
        if (obstacleSignal !=none_obstacle||(&cliff_valueB)->cliffValue0 == 1 )
        {
            linear_velocity = 0;
            angular_velocity = 0;
            right_running_step_status = RIGHTWALKEDGE;
            collision_left_rightrun_step_status=0;
            break;
        }
        if (my_abs(last_position_y - current_pose->y) > lateral_move_distance / 2)
        {
            collision_left_rightrun_step_status = MORE_TRY_BREAK_BYPASS_CL_DRYL;
            break;
        }
        break;
        
    case  GOSTR_BYPASS_LOOP_CL_DRYL:
        if (obstacleSignal == left_obstacle)
        {
            infrared_collision=false;
            collision_left_rightrun_step_status = LEFT_COLLISION_BYPASS_CL_DRYL;
            break;
        }
        if (my_abs(last_position_y - current_pose->y) > close_edge)
        {
            collision_left_rightrun_step_status = GOSTR_BYPASS_BOW_CONTINUE_CL_DRYL;
            break;
        }
        if(obstacleSignal == front_obstacle || obstacleSignal == right_obstacle||(&cliff_valueB)->cliffValue0 == 1){
            last_position_y = current_pose->y;
            collision_left_rightrun_step_status = TURN_CCLOCK_TARGET_YAW_ABS90_CL_DRYM;
            break;
        }
        if (last_position_y < current_pose->y - 1)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            collision_left_rightrun_step_status = GOSTR_BYPASS_BOW_CONTINUE_EXIT_CL_DRYL;
            break;
        }
        if (my_abs(current_pose->x) > half_map_wide-2*GRIDWIDTH)
        {
            collision_left_rightrun_step_status = COMPLETE_CL_DRYL;
            break;
        }
        if (my_abs(Yaw) > 90)
        {
            collision_left_rightrun_step_status = COMPLETE_CL_DRYL;
            break;
        }
        if(infrared_collision==true&&my_abs(Yaw)<75){
            if(adcRealTime[0]>500|| adcRealTime[1]>50||adcRealTime[2]>50){
                linear_velocity = 100;
                angular_velocity = -57;
                if(adcRealTime[2]>50){
                    linear_velocity = 50;
                    angular_velocity = -turn_vel;
                }
            }
            else{
				left_edge_judgment_repeat();
//                if(adcRealTime[8]>100&&adcRealTime[8]<1500){
//                    delimma_edge=0;
//                    linear_velocity = 200;
//                    angular_velocity = 0;
//                }
//                if(adcRealTime[8]>=1500){
//                    delimma_edge=0;
//                    linear_velocity = 200;
//                    angular_velocity = -10;
//                }
//                if(adcRealTime[8]<100){
//                    if(delimma_edge<10){
//                        delimma_edge++;
//                        linear_velocity = 200;
//                        angular_velocity = 10;
//                    }else{
//                        linear_velocity = 100;
//                        angular_velocity = 20;
//                    }
//                }
            }
        }
        else{
            if (my_abs(last_position_x - current_pose->x) > lateral_move_distance / 3)
            {
                infrared_collision=true;
				last_position_x=current_pose->x;
                collision_left_rightrun_step_status = GOSTR_X_MORE_ONE_THIRD_LATERALDIS_BYPASS_CL_DRYL;
                break;
            }
            
            linear_velocity = long_stra_vel;
            angular_velocity = 0;
        }
        break;
        
    case  GOSTR_BYPASS_BOW_CONTINUE_CL_DRYL:
        if (obstacleSignal!=none_obstacle||(&cliff_valueB)->cliffValue0 == 1)
        {
            collision_left_rightrun_step_status = GOSTR_BYPASS_BOW_CONTINUE_COLLISION_CL_DRYL;
            break;
        }
        if (my_abs(Yaw) > 175 )
        {
            linear_velocity = 0;
            angular_velocity = 0;
            bow_continue = true;
            collision_left_rightrun_step_status = GOSTR_BYPASS_BOW_CONTINUE_EXIT_CL_DRYL;
            break;
        }
        linear_velocity = 0;
        angular_velocity = -turn_vel;
        break;
    case  GOSTR_BYPASS_BOW_CONTINUE_COLLISION_CL_DRYL:
        if(turn_start_update == 0)
        {
            turn_start_x = current_pose->x;
            turn_start_y = current_pose->y;
            turn_start_update = 1;
        }
        linear_velocity = -long_stra_vel;
        angular_velocity = 0;
        if (my_abs(turn_start_x - current_pose->x) > second_turn_backward_distance ||my_abs(turn_start_y - current_pose->y) > second_turn_backward_distance)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            collision_left_rightrun_step_status = GOSTR_BYPASS_BOW_CONTINUE_CL_DRYL;
            turn_start_update = 0;
            break;
        }
        break;
    case  GOSTR_BYPASS_BOW_CONTINUE_EXIT_CL_DRYL:
        collision_left_rightrun_step_status = COMPLETE_CL_DRYL;
        break;
    case  LEFT_COLLISION_BYPASS_CL_DRYL:
        if(turn_start_update == 0)
        {
            turn_start_x = current_pose->x;
            turn_start_y = current_pose->y;
            turn_start_update = 1;
        }
        linear_velocity = -long_stra_vel;
        angular_velocity = 0;
        
        if (my_abs(turn_start_x - current_pose->x) > collision_backward_distance || my_abs(turn_start_y - current_pose->y) > collision_backward_distance)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            turn_start_update = 0;
            temporary_yaw = Yaw;
            if (my_abs(Yaw) <15){
                collision_left_rightrun_step_status = TURN_CLCOK_TARGET_YAW_LESS_ABS15_CL_DRYL;
            }
            else
            {
                collision_left_rightrun_step_status = TURN_CLCOK_TARGET_YAW_MORE_ABS15_CL_DRYL;
            }
            break;
        }
        break;
    case  TURN_CLCOK_TARGET_YAW_LESS_ABS15_CL_DRYL:
        if (obstacleSignal!=none_obstacle||(&cliff_valueB)->cliffValue0 == 1)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            collision_left_rightrun_step_status = TURN_CLCOK_TARGET_YAW_LESS_ABS15_COLLISION_CL_DRYL;
            break;
        }
        if (my_abs(Yaw) >15)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            collision_left_rightrun_step_status = GOSTR_BYPASS_CL_DRYL;
            break;
        }
		if (my_abs(Yaw) > 90)
        {
            collision_left_rightrun_step_status = COMPLETE_CL_DRYL;
            break;
        }
        linear_velocity = 0;
        angular_velocity = -turn_vel;
        break;
    case TURN_CLCOK_TARGET_YAW_LESS_ABS15_COLLISION_CL_DRYL:
        if(turn_start_update == 0)
        {
            turn_start_x = current_pose->x;
            turn_start_y = current_pose->y;
            turn_start_update = 1;
        }
        linear_velocity = -long_stra_vel;
        angular_velocity = 0;
        if (my_abs(turn_start_x - current_pose->x) > collision_backward_distance || my_abs(turn_start_y - current_pose->y) > collision_backward_distance)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            collision_left_rightrun_step_status = TURN_CLCOK_TARGET_YAW_LESS_ABS15_CL_DRYL;
            turn_start_update = 0;
            break;
        }
        break;
        
    case  TURN_CLCOK_TARGET_YAW_MORE_ABS15_CL_DRYL:
        if (obstacleSignal!=none_obstacle||(&cliff_valueB)->cliffValue0 == 1)
        {
            collision_left_rightrun_step_status = TURN_CLCOK_TARGET_YAW_MORE_ABS15_COLLISION_CL_DRYL;
            break;
        }
        if (my_abs(Yaw - temporary_yaw) > 15)
        {
            collision_left_rightrun_step_status = GOSTR_BYPASS_CL_DRYL;
            break;
        }
		if (my_abs(Yaw) > 90)
        {
            collision_left_rightrun_step_status = COMPLETE_CL_DRYL;
            break;
        }
        linear_velocity = 0;
        angular_velocity = -turn_vel;
        break;
    case TURN_CLCOK_TARGET_YAW_MORE_ABS15_COLLISION_CL_DRYL:
        if(turn_start_update == 0)
        {
            turn_start_x = current_pose->x;
            turn_start_y = current_pose->y;
            turn_start_update = 1;
        }
        linear_velocity = -long_stra_vel;
        angular_velocity = 0;
        if (my_abs(turn_start_x - current_pose->x) > collision_backward_distance || my_abs(turn_start_y - current_pose->y) > collision_backward_distance)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            collision_left_rightrun_step_status = TURN_CLCOK_TARGET_YAW_MORE_ABS15_CL_DRYL;
            turn_start_update = 0;
            break;
        }
        break;
    case  GOSTR_X_MORE_ONE_THIRD_LATERALDIS_BYPASS_CL_DRYL:
        if (my_abs(Yaw) < 30)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            temporary_yaw = Yaw;
            collision_left_rightrun_step_status = TURN_CCLOCK_TARGET_YAW_MORE_AB30_CL_DRYL;
        }
        else if (my_abs(Yaw) > 30 && (Yaw < 0))
        {
            linear_velocity = 0;
            angular_velocity = 0;
            temporary_yaw = Yaw;
            collision_left_rightrun_step_status = TURN_CCLOCK_YAW_ADD_ABS30_CL_DRYL;
        }
        else{
            linear_velocity = 0;
            angular_velocity = 0;
            collision_left_rightrun_step_status = GOSTR_BYPASS_CL_DRYL;
            break;
        }
        break;
    case  TURN_CCLOCK_TARGET_YAW_MORE_AB30_CL_DRYL:
        if (my_abs(Yaw) >30)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            collision_left_rightrun_step_status = GOSTR_BYPASS_CL_DRYL;
            break;
        }
        if (obstacleSignal!=none_obstacle||(&cliff_valueB)->cliffValue0 == 1)
        {
            collision_left_rightrun_step_status = TURN_CCLOCK_TARGET_YAW_MORE_AB30_COLLISION_CL_DRYL;
        }
        linear_velocity = 0;
        angular_velocity = turn_vel;
        break;
    case  TURN_CCLOCK_TARGET_YAW_MORE_AB30_COLLISION_CL_DRYL:
        if(turn_start_update == 0)
        {
            turn_start_x = current_pose->x;
            turn_start_y = current_pose->y;
            turn_start_update = 1;
        }
        linear_velocity = -long_stra_vel;
        angular_velocity = 0;
        if (my_abs(turn_start_x - current_pose->x) > collision_backward_distance || my_abs(turn_start_y - current_pose->y) > collision_backward_distance)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            turn_start_update = 0;
            if (my_abs(Yaw) <15){
                collision_left_rightrun_step_status = TURN_CLCOK_TARGET_YAW_LESS_ABS15_CL_DRYL;
            }
            else
            {
                temporary_yaw = Yaw;
                collision_left_rightrun_step_status = TURN_CLCOK_TARGET_YAW_MORE_ABS15_CL_DRYL;
            }
            break;
        }
        break;
    case  TURN_CCLOCK_YAW_ADD_ABS30_CL_DRYL:
        if (my_abs(Yaw - temporary_yaw) > 30)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            collision_left_rightrun_step_status = GOSTR_BYPASS_CL_DRYL;
            break;
        }
        
        if (obstacleSignal!=none_obstacle||(&cliff_valueB)->cliffValue0 == 1)
        {
            collision_left_rightrun_step_status = TURN_CCLOCK_YAW_ADD_ABS30_COLLISION_CL_DRYL;
        }
        linear_velocity = 0;
        angular_velocity = turn_vel;
        break;
    case  TURN_CCLOCK_YAW_ADD_ABS30_COLLISION_CL_DRYL:
        if(turn_start_update == 0)
        {
            turn_start_x = current_pose->x;
            turn_start_y = current_pose->y;
            turn_start_update = 1;
        }
        linear_velocity = -long_stra_vel;
        angular_velocity = 0;
        if (my_abs(turn_start_x - current_pose->x) > collision_backward_distance || my_abs(turn_start_y - current_pose->y) > collision_backward_distance)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            turn_start_update = 0;
            if (my_abs(Yaw) <15){
                collision_left_rightrun_step_status = TURN_CLCOK_TARGET_YAW_LESS_ABS15_CL_DRYL;
            }
            else
            {
                temporary_yaw = Yaw;
                collision_left_rightrun_step_status = TURN_CLCOK_TARGET_YAW_MORE_ABS15_CL_DRYL;
            }
            break;
        }
        break;
    case  MORE_TRY_BREAK_BYPASS_CL_DRYL:
        if (obstacleSignal!=none_obstacle||(&cliff_valueB)->cliffValue0 == 1)
        {
            collision_left_rightrun_step_status = GOSTR_BYPASS_BOW_CONTINUE_COLLISION_CL_DRYL;
            bow_continue = true;
            break;
        }
        if (my_abs(Yaw) > 175)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            bow_continue = true;
            collision_left_rightrun_step_status = GOSTR_BYPASS_BOW_CONTINUE_EXIT_CL_DRYL;
            break;
        }
        linear_velocity = 0;
        angular_velocity = -turn_vel;
        break;
    case  COMPLETE_CL_DRYL:
        infrared_collision=false;
        collision_left_rightrun_step_status = 0;
        complete_flag = 1;
        break;
    case  DIR_RIGHT_YAW_MORE_ABS90_CLRRS:
        linear_velocity = 0;
        angular_velocity = 0;
        last_position_y=current_pose->y;
        collision_left_rightrun_step_status  = TURN_CCLOCK_TARGET_YAW_NEG123_CL_DRYM;
        break;
        
    case  TURN_CCLOCK_TARGET_YAW_NEG123_CL_DRYM:
        linear_velocity = 0;
        angular_velocity = turn_vel;
        if (my_abs(Yaw)<90)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            collision_left_rightrun_step_status  = GOSTR_YAW_EQUAL_NEG123_CL_DRYM;
            break;
        }
        if (obstacleSignal !=none_obstacle||(&cliff_valueB)->cliffValue0 == 1 )
        {
            collision_left_rightrun_step_status  = TURN_CCLOCK_TARGET_YAW_NEG123_COLLISION_CL_DRYM;
            break;
        }
        break;
    case  TURN_CCLOCK_TARGET_YAW_NEG123_COLLISION_CL_DRYM:
        if(turn_start_update == 0)
        {
            turn_start_x = current_pose->x;
            turn_start_y = current_pose->y;
            turn_start_update = 1;
        }
        linear_velocity = -long_stra_vel;
        angular_velocity = 0;
        if (my_abs(turn_start_x - current_pose->x) > second_turn_backward_distance || my_abs(turn_start_y - current_pose->y) > second_turn_backward_distance)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            turn_start_update = 0;
            collision_left_rightrun_step_status = TURN_CCLOCK_TARGET_YAW_NEG123_CL_DRYM;
            break;
        }
        break;
        
    case  GOSTR_YAW_EQUAL_NEG123_CL_DRYM:
        if (obstacleSignal !=none_obstacle||(&cliff_valueB)->cliffValue0 == 1 )
        {
			linear_velocity = 0;
            angular_velocity = 0;
            collision_left_rightrun_step_status = RIGHT_REVERSE_WALK_EDGE_CL_DRYM;
            break;		
        }
        if (my_abs(last_position_y - current_pose->y) > lateral_move_distance)
        {
            collision_left_rightrun_step_status = TURN_CCLOCK_TARGET_YAW_NEG98_CL_DRYM;
            break;
        }
        linear_velocity = long_stra_vel;
        angular_velocity = 0;
        break;
        
    case  TURN_CCLOCK_TARGET_YAW_NEG98_CL_DRYM:
        if (Yaw > -85)
        {
            collision_left_rightrun_step_status = GOSTR_YAW_EQUAL_NEG98_CL_DRYM;
            break;
        }
        if (obstacleSignal !=none_obstacle||(&cliff_valueB)->cliffValue0 == 1)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            collision_left_rightrun_step_status = TURN_CCLOCK_TARGET_YAW_NEG98_COLLISION_CL_DRYM;
            break;
        }
        linear_velocity = 0;
        angular_velocity = turn_vel;
        break;
    case  TURN_CCLOCK_TARGET_YAW_NEG98_COLLISION_CL_DRYM:
        if(turn_start_update == 0)
        {
            turn_start_x = current_pose->x;
            turn_start_y = current_pose->y;
            turn_start_update = 1;
        }
        linear_velocity = -long_stra_vel;
        angular_velocity = 0;
        if (my_abs(turn_start_x - current_pose->x) > turn_backward_distance || my_abs(turn_start_y - current_pose->y) > turn_backward_distance)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            turn_start_update = 0;
            collision_left_rightrun_step_status = TURN_CCLOCK_TARGET_YAW_NEG98_CL_DRYM;
            break;
        }
        break;
    case  GOSTR_YAW_EQUAL_NEG98_CL_DRYM:
        linear_velocity = long_stra_vel;
        angular_velocity = 0;
        if (obstacleSignal != none_obstacle||(&cliff_valueB)->cliffValue0 == 1)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            collision_left_rightrun_step_status = RIGHT_REVERSE_WALK_EDGE_CL_DRYM;
            break;
        }
        if (my_abs(last_position_y - current_pose->y) > lateral_move_distance)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            collision_left_rightrun_step_status = TURN_CCLOCK_TARGET_YAW_ABS8_CL_DRYM;
            break;
        }
        break;
    case  RIGHT_REVERSE_WALK_EDGE_CL_DRYM:
        infrared_collision=false;
        collision_left_rightrun_step_status = 0;
        complete_flag = 2;
        break;
        
    case  TURN_CCLOCK_TARGET_YAW_ABS8_CL_DRYM:
        linear_velocity = 0;
        angular_velocity = turn_vel;
        if (my_abs(Yaw) < 5) //8
        {
            linear_velocity = 0;
            angular_velocity = 0;
            collision_left_rightrun_step_status = COMPLETE_CL_DRYM;
            break;
        }
        if (obstacleSignal !=none_obstacle||(&cliff_valueB)->cliffValue0 == 1 )
        {
            linear_velocity = 0;
            angular_velocity = 0;
            collision_left_rightrun_step_status = TURN_CCLOCK_TARGET_YAW_ABS8_COLLISION_CL_DRYM;
            break;
        }
        break;
    case  TURN_CCLOCK_TARGET_YAW_ABS8_COLLISION_CL_DRYM:
        if(turn_start_update == 0)
        {
            turn_start_x = current_pose->x;
            turn_start_y = current_pose->y;
            turn_start_update = 1;
        }
        linear_velocity = -long_stra_vel;
        angular_velocity = 0;
        if (my_abs(turn_start_x - current_pose->x) > collision_backward_distance || my_abs(turn_start_y - current_pose->y) > collision_backward_distance)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            turn_start_update = 0;
            collision_left_rightrun_step_status = TURN_CCLOCK_TARGET_YAW_ABS8_CL_DRYM;
            break;
        }
        break;
    case  COMPLETE_CL_DRYM:
        infrared_collision=false;
        collision_left_rightrun_step_status = 0;
        complete_flag = 1;
        break;
    }
    return complete_flag;
}




unsigned char  CollisionFrontRightRunStep(POSE *current_pose, unsigned char obstacleSignal){
    int Yaw;
    unsigned char complete_flag = 0;
    Yaw = current_pose->orientation;
    Yaw = Yaw /100;
    switch (collision_front_rightrun_step_status)
    {
    case 0:
        if (turn_start_update == 0)
        {
            if(obstacleSignal==none_obstacle){
				obstacleSignal=front_obstacle;
                if (my_abs(Yaw) < 90)
                {
                    collision_front_rightrun_step_status = DIR_RIGHT_YAW_LESS_ABS90_CFRRS;
                }
                else
                {
                    collision_front_rightrun_step_status = DIR_RIGHT_YAW_MORE_ABS90_CFRRS;
                }
                break;
            }
            turn_start_x = current_pose->x;
            turn_start_y = current_pose->y;
            turn_start_update = 1;
        }
        linear_velocity = -long_stra_vel;
        angular_velocity = 0;
        if (my_abs(turn_start_x - current_pose->x) > collision_backward_distance || my_abs(turn_start_y - current_pose->y) > collision_backward_distance)
        {
            collision_front_rightrun_step_status = GOBACK_DISTANCE_CFRRS;
            turn_start_update = 0;
            break;
        }
        break;
    case GOBACK_DISTANCE_CFRRS:
        if (turn_start_update == 0)
        {
            turn_start_x = current_pose->x;
            turn_start_y = current_pose->y;
            turn_start_update = 1;
        }
        linear_velocity = -long_stra_vel;
        angular_velocity = 0;
        if (my_abs(turn_start_x - current_pose->x) > front_backward_distance)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            if (my_abs(Yaw) < 90)
            {
                collision_front_rightrun_step_status = DIR_RIGHT_YAW_LESS_ABS90_CFRRS;
            }
            else
            {
                collision_front_rightrun_step_status = DIR_RIGHT_YAW_MORE_ABS90_CFRRS;
            }
            turn_start_update = 0;
            break;
        }
        break;
    case DIR_RIGHT_YAW_LESS_ABS90_CFRRS:
        linear_velocity = 0;
        angular_velocity = 0;
        last_position_y = current_pose->y;
        collision_front_rightrun_step_status = TURN_CLOCK_TARGET_YAW_NEG60_CF_DRYL;
        break;
    case TURN_CLOCK_TARGET_YAW_NEG60_CF_DRYL:
        linear_velocity = 0;
        angular_velocity = -turn_vel;
        if (Yaw< -80)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            collision_front_rightrun_step_status = GOSTR_YAW_EQUAL_NEG60_CF_CF_DRYL;
            break;
        }
        if (obstacleSignal != none_obstacle||(&cliff_valueB)->cliffValue0 == 1)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            collision_front_rightrun_step_status = TURN_CLOCK_TARGET_YAW_NEG60_COLLISION_CF_DRYL;
            break;
        }
        break;
    case TURN_CLOCK_TARGET_YAW_NEG60_COLLISION_CF_DRYL:
        if (turn_start_update == 0)
        {
            turn_start_x = current_pose->x;
            turn_start_y = current_pose->y;
            turn_start_update = 1;
        }
        linear_velocity = -long_stra_vel;
        angular_velocity = 0;
        if (my_abs(turn_start_x - current_pose->x) > collision_backward_distance || my_abs(turn_start_y - current_pose->y) > collision_backward_distance)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            turn_start_update = 0;
            collision_front_rightrun_step_status = TURN_CLOCK_TARGET_YAW_NEG60_CF_DRYL;
            break;
        }
        break;
    case GOSTR_YAW_EQUAL_NEG60_CF_CF_DRYL:
        linear_velocity = long_stra_vel;
        angular_velocity = 0;
        if (obstacleSignal != none_obstacle||(&cliff_valueB)->cliffValue0 == 1)
        {
			linear_velocity = 0;
            angular_velocity = 0;
            collision_front_rightrun_step_status = RIGHT_WALK_EDGE_CF_DRYL;
            break;		
        }
        if (my_abs(last_position_y - current_pose->y) > lateral_move_distance)
        {
            collision_front_rightrun_step_status = TURN_CLOCK_TARGET_YAW_NEG82_CF_DRYL;
            break;
        }
        break;
    case TURN_CLOCK_TARGET_YAW_NEG82_CF_DRYL:
        linear_velocity = 0;
        angular_velocity = -turn_vel;
        if (Yaw < -85)
        {
            collision_front_rightrun_step_status = GOSTR_YAW_EQUAL_NEG82_CF_DRYL;
            break;
        }
        if (obstacleSignal != none_obstacle||(&cliff_valueB)->cliffValue0 == 1)
        {
            collision_front_rightrun_step_status = TURN_CLOCK_TARGET_YAW_NEG82_COLLISION_CF_DRYL;
            break;
        }
        break;
    case TURN_CLOCK_TARGET_YAW_NEG82_COLLISION_CF_DRYL:
        if (turn_start_update == 0)
        {
            turn_start_x = current_pose->x;
            turn_start_y = current_pose->y;
            turn_start_update = 1;
        }
        linear_velocity = -long_stra_vel;
        angular_velocity = 0;
        if (my_abs(turn_start_x - current_pose->x) > collision_backward_distance || my_abs(turn_start_y - current_pose->y) > collision_backward_distance)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            turn_start_update = 0;
            collision_front_rightrun_step_status = TURN_CLOCK_TARGET_YAW_NEG82_CF_DRYL;
            break;
        }
        break;
    case GOSTR_YAW_EQUAL_NEG82_CF_DRYL:
        linear_velocity = long_stra_vel;
        angular_velocity = 0;
        if (obstacleSignal != none_obstacle||(&cliff_valueB)->cliffValue0 == 1)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            collision_front_rightrun_step_status = RIGHT_WALK_EDGE_CF_DRYL;
            break;
        }
        if (my_abs(last_position_y - current_pose->y) > (lateral_move_distance))
        {
            collision_front_rightrun_step_status = TURN_CLOCK_TARGET_YAW_ABS173_CF_DRYL;
            break;
        }
        break;
    case RIGHT_WALK_EDGE_CF_DRYL:
        collision_front_rightrun_step_status = 0;
        complete_flag = 2;
        break;
        
    case TURN_CLOCK_TARGET_YAW_ABS173_CF_DRYL:
        linear_velocity = 0;
        angular_velocity = -turn_vel;
        if (my_abs(Yaw) > 176) //173
        {
            linear_velocity = 0;
            angular_velocity = 0;
            collision_front_rightrun_step_status = COMPLETE_CF_DRYL;
            break;
        }
        if (obstacleSignal != none_obstacle||(&cliff_valueB)->cliffValue0 == 1)
        {
            collision_front_rightrun_step_status = TURN_CLOCK_TARGET_YAW_ABS173_COLLISION_CF_DRYL;
            break;
        }
        break;
    case TURN_CLOCK_TARGET_YAW_ABS173_COLLISION_CF_DRYL:
        if (turn_start_update == 0)
        {
            turn_start_x = current_pose->x;
            turn_start_y = current_pose->y;
            turn_start_update = 1;
        }
        linear_velocity = -long_stra_vel;
        angular_velocity = 0;
        if (my_abs(turn_start_x - current_pose->x) > collision_backward_distance || my_abs(turn_start_y - current_pose->y) > collision_backward_distance)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            turn_start_update = 0;
            collision_front_rightrun_step_status = TURN_CLOCK_TARGET_YAW_ABS173_CF_DRYL;
            break;
        }
        break;
    case COMPLETE_CF_DRYL:
        collision_front_rightrun_step_status = 0;
        complete_flag = 1;
        break;
    case DIR_RIGHT_YAW_MORE_ABS90_CFRRS:
        linear_velocity = 0;
        angular_velocity = 0;
        last_position_y = current_pose->y;
        collision_front_rightrun_step_status = TURN_CCLOCK_TARGET_YAW_ABS120_CF_DRYM;
        break;
    case TURN_CCLOCK_TARGET_YAW_ABS120_CF_DRYM:
        linear_velocity = 0;
        angular_velocity = turn_vel;
        if (my_abs(Yaw) < 95)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            collision_front_rightrun_step_status = GOSTR_YAW_ABS120_CF_DRYM;
            break;
        }
        if (obstacleSignal != none_obstacle||(&cliff_valueB)->cliffValue0 == 1)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            collision_front_rightrun_step_status = TURN_CCLOCK_TARGET_YAW_ABS120_COLLISION_CF_DRYM;
            break;
        }
        break;
    case TURN_CCLOCK_TARGET_YAW_ABS120_COLLISION_CF_DRYM:
        if (turn_start_update == 0)
        {
            turn_start_x = current_pose->x;
            turn_start_y = current_pose->y;
            turn_start_update = 1;
        }
        linear_velocity = -long_stra_vel;
        angular_velocity = 0;
        if (my_abs(turn_start_x - current_pose->x) > collision_backward_distance || my_abs(turn_start_y - current_pose->y) > collision_backward_distance)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            turn_start_update = 0;
            collision_front_rightrun_step_status = TURN_CCLOCK_TARGET_YAW_ABS120_CF_DRYM;
            break;
        }
        break;
    case GOSTR_YAW_ABS120_CF_DRYM:
        linear_velocity = long_stra_vel;
        angular_velocity = 0;
        if (obstacleSignal != none_obstacle||(&cliff_valueB)->cliffValue0 == 1)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            collision_front_rightrun_step_status = RIGHT_REVERSE_WALK_EDGE_CF_DRYM;
            break;						
//            collision_front_rightrun_step_status = TURN_CCLOCK_TARGET_YAW_ABS93_CF_DRYM;
//            break;
        }
        if (my_abs(last_position_y - current_pose->y) > lateral_move_distance)
        {
            collision_front_rightrun_step_status = TURN_CCLOCK_TARGET_YAW_ABS93_CF_DRYM;
            break;
        }
        break;
    case TURN_CCLOCK_TARGET_YAW_ABS93_CF_DRYM:
        linear_velocity = 0;
        angular_velocity = turn_vel;
        if (my_abs(Yaw) < 90)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            collision_front_rightrun_step_status = GOSTR_YAW_ABS93_CF_DRYM;
            break;
        }
        if (obstacleSignal != none_obstacle)
        {
            collision_front_rightrun_step_status = TURN_CCLOCK_TARGET_YAW_ABS93_COLLISION_CF_DRYM;
            break;
        }
        break;
    case TURN_CCLOCK_TARGET_YAW_ABS93_COLLISION_CF_DRYM:
        if (turn_start_update == 0)
        {
            turn_start_x = current_pose->x;
            turn_start_y = current_pose->y;
            turn_start_update = 1;
        }
        linear_velocity = -long_stra_vel;
        angular_velocity = 0;
        if (my_abs(turn_start_x - current_pose->x) > collision_backward_distance || my_abs(turn_start_y - current_pose->y) > collision_backward_distance)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            turn_start_update = 0;
            collision_front_rightrun_step_status = TURN_CCLOCK_TARGET_YAW_ABS93_CF_DRYM;
            break;
        }
        break;
    case GOSTR_YAW_ABS93_CF_DRYM:
        linear_velocity = long_stra_vel;
        angular_velocity = 0;
        if (obstacleSignal != none_obstacle||(&cliff_valueB)->cliffValue0 == 1)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            collision_front_rightrun_step_status = RIGHT_REVERSE_WALK_EDGE_CF_DRYM;
            break;
        }
        if (my_abs(last_position_y - current_pose->y) > (lateral_move_distance))
        {
            collision_front_rightrun_step_status = TURN_CCLOCK_TARGET_YAW_ABS10_CF_DRYM;
            break;
        }
        break;
    case RIGHT_REVERSE_WALK_EDGE_CF_DRYM:
        collision_front_rightrun_step_status = 0;
        complete_flag = 3;
        break;
    case TURN_CCLOCK_TARGET_YAW_ABS10_CF_DRYM:
        linear_velocity = 0;
        angular_velocity = turn_vel;
        if (my_abs(Yaw) < 5) //8
        {
            linear_velocity = 0;
            angular_velocity = 0;
            collision_front_rightrun_step_status = COMPLETE_CF_DRYL;
            break;
        }
        if (obstacleSignal != none_obstacle||(&cliff_valueB)->cliffValue0 == 1)
        {
            collision_front_rightrun_step_status = TURN_CCLOCK_TARGET_YAW_ABS10_COLLISION_CF_DRYM;
            break;
        }
        
        break;
    case TURN_CCLOCK_TARGET_YAW_ABS10_COLLISION_CF_DRYM:
        if (turn_start_update == 0)
        {
            turn_start_x = current_pose->x;
            turn_start_y = current_pose->y;
            turn_start_update = 1;
        }
        linear_velocity = -long_stra_vel;
        angular_velocity = 0;
        if (my_abs(turn_start_x - current_pose->x) > collision_backward_distance || my_abs(turn_start_y - current_pose->y) > collision_backward_distance)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            turn_start_update = 0;
            collision_front_rightrun_step_status = TURN_CCLOCK_TARGET_YAW_ABS10_CF_DRYM;
            break;
        }
        break;
    case COMPLETE_CF_DRYM:
        collision_front_rightrun_step_status = 0;
        complete_flag = 2;
        break;
    }
    return complete_flag;
}




unsigned char  RightEdgeDilemma(POSE *current_pose, unsigned char obstacleSignal){
    int Yaw;
    unsigned char complete_flag = 0;
    signed char i=0;
    signed char j=0;
    signed char k,ij;
    Yaw = current_pose->orientation;
    Yaw = Yaw /100;
    switch(right_edge_dilemma_status)
    {
    case 0:
        linear_velocity = 0;
        angular_velocity = 0;
        right_edge_dilemma_status =GOSTR_DILEMMA;
        break;
    case GOSTR_DILEMMA:
        if(my_abs(current_pose->x+half_map_wide)<=100*close_edge_max_x-500){
            if(my_abs(Yaw)<=90){
                right_edge_dilemma_status = COLLISION_YAW_LESS_ABS90_DILEMMA;
            }
            else{
                right_edge_dilemma_status = YAW_MORE_ABS90_LESS_ABS176_CCLOCK_LESS_ABS90_MORE_ABS4_CLOCK_DILEMMA;
            }
        }
        else{
            if(my_abs(Yaw)<=90){
                right_edge_dilemma_status =CCLOCK_TARGET_YAW_MORE_ABS178_DILEMMA;
            }
            else{
                right_edge_dilemma_status =COLLISION_YAW_MORE_ABS90_DILEMMA;
            }
			DelimmaNumber++;
        }
        DelimmaNumber++;
        step=1;
        break;
    case YAW_MORE_ABS90_LESS_ABS176_CCLOCK_LESS_ABS90_MORE_ABS4_CLOCK_DILEMMA:
        linear_velocity = 0;
        angular_velocity = turn_vel;
        if (my_abs(Yaw)<10)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            right_edge_dilemma_status =COLLISION_YAW_LESS_ABS90_DILEMMA;
            break;
        }
        if (obstacleSignal!=none_obstacle||(&cliff_valueB)->cliffValue0 == 1)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            right_edge_dilemma_status = CCLOCK_TARGET_YAW_MORE_ABS178_COLLISION_DILEMMA;
            break;
        }
        break;
    case CCLOCK_TARGET_YAW_MORE_ABS178_COLLISION_DILEMMA:
        if(turn_start_update == 0)
        {
            turn_start_x = current_pose->x;
            turn_start_y = current_pose->y;
            turn_start_update = 1;
        }
        linear_velocity = -long_stra_vel;
        angular_velocity = 0;
        if (my_abs(turn_start_x - current_pose->x) > collision_backward_distance || my_abs(turn_start_y - current_pose->y) > collision_backward_distance)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            turn_start_update = 0;
            right_edge_dilemma_status = YAW_MORE_ABS90_LESS_ABS176_CCLOCK_LESS_ABS90_MORE_ABS4_CLOCK_DILEMMA;
            break;
        }
        break;
    case CCLOCK_TARGET_YAW_MORE_ABS178_DILEMMA:
        linear_velocity = 0;
        angular_velocity =-turn_vel;
        if (my_abs(Yaw)>170)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            right_edge_dilemma_status =COLLISION_YAW_MORE_ABS90_DILEMMA;
            break;
        }
        if (obstacleSignal!=none_obstacle||(&cliff_valueB)->cliffValue0 == 1)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            right_edge_dilemma_status = GOSTR_CYL_DILEMMA;
            break;
        }
        break;
    case GOSTR_CYL_DILEMMA:
        if(turn_start_update == 0)
        {
            turn_start_x = current_pose->x;
            turn_start_y = current_pose->y;
            turn_start_update = 1;
        }
        linear_velocity = -long_stra_vel;
        angular_velocity = 0;
        if (my_abs(turn_start_x - current_pose->x) > collision_backward_distance || my_abs(turn_start_y - current_pose->y) > collision_backward_distance)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            turn_start_update = 0;
            right_edge_dilemma_status = CCLOCK_TARGET_YAW_MORE_ABS178_DILEMMA;
            break;
        }
        break;
    case COLLISION_YAW_LESS_ABS90_DILEMMA:
        linear_velocity = 0;
        angular_velocity = 0;
        last_position_x = current_pose->x;
        last_position_y = current_pose->y;
        right_edge_dilemma_status = LOOP_TEN_NUM_DILEMMA;
        temporary_close_edge=close_edge;
        break;
    case LOOP_TEN_NUM_DILEMMA:
		right_edge_judgment_repeat();
//        if(adcRealTime[9]>100&&adcRealTime[9]<1500){
//            delimma_edge=0;
//            linear_velocity = 200;
//            angular_velocity = 0;
//        }
//        if(adcRealTime[9]>=1500){
//            delimma_edge=0;
//            linear_velocity = 200;
//            angular_velocity = 10;
//        }
//        if(adcRealTime[9]<=100){
//            if(delimma_edge<10){
//                delimma_edge++;
//                linear_velocity = 200;
//                angular_velocity = -10;	
//            }
//            else{
//                linear_velocity = 100;
//                angular_velocity = -20;
//            }
//        }
        if(current_pose->y>-return_origin_distance){
            linear_velocity = 0;
            angular_velocity = 0;
            DelimmaNumber=0;
            right_edge_dilemma_status = 0;
            step=0;
            complete_flag = 2;
            break;
        }
        if(my_abs(current_pose->x+half_map_wide)>=100*close_edge_max_x-500){
            linear_velocity = 0;
            angular_velocity = 0;
            if(my_abs(leakingsweep_x-current_pose->x)>500&&my_abs(leakingsweep_y-current_pose->y)>100){
				if((my_abs(current_pose->x)<half_map_wide-GRIDHEIGHT)&&(my_abs(current_pose->y)<half_map_wide-GRIDHEIGHT)){
                leakingsweep_x=current_pose->x;
                leakingsweep_y=current_pose->y;
                boolleaksweep=true;
                break;
				}
            }
            if (0 != leakingsweep){
                right_edge_dilemma_status=0;
                DelimmaNumber=0;
                step=0;
                right_running_step_status = LEAKING_SWEEP_RIGHTRUN_STEP;
            }
            else{
                linear_velocity = 0;
                angular_velocity = 0;
                right_edge_dilemma_status =COMPLETE_EL_DRYM;
            }
            break;
        }
        if (obstacleSignal!=none_obstacle||(&cliff_valueB)->cliffValue0 == 1)
        {
            right_edge_dilemma_status = GOSTR_COLLISION_DILEMMA;
            break;
        }
        if(-105<Yaw&&Yaw<-60){
            i=(current_pose->x+half_map_long)/100;
            j=(current_pose->y+half_map_long-110)/100;
            linear_velocity = long_stra_vel;
            angular_velocity = 0;
            if(j<0||i<1||j>99||i>98){
            }
            else{
                j=((gridmap.map[i-1][j]==125)?1:0)+((gridmap.map[i][j]==125)?1:0)+((gridmap.map[i][j]==125)?1:0);
                if(j==3){
                    right_edge_dilemma_status=0;
                    DelimmaNumber=0;
                    step=0;
                    complete_flag = 1;
                }
            }
            break;
        }
        if (my_abs(last_position_x - current_pose->x) > temporary_close_edge||my_abs(last_position_y - current_pose->y) > temporary_close_edge)
        {
            if(my_abs(Yaw)<=45){
                k=(current_pose->x+half_map_wide)/GRIDWIDTH;
                ij=(current_pose->y+half_map_wide)/GRIDWIDTH-3;
                if(k>1&&k<99&&ij>1&&ij<GRIDWIDTH){
                    gridmap.map[k-1][ij+1]=0;
                    gridmap.map[k][ij+1]=0;
                    gridmap.map[k+1][ij+1]=0;
                    gridmap.map[k-1][ij]=0;
                    gridmap.map[k][ij]=0;
                    gridmap.map[k+1][ij]=0;
                    gridmap.map[k][ij-1]=0;
                    gridmap.map[k-1][ij-1]=0;
                }
            }
            else if(Yaw<135&&Yaw>45){
                k=(current_pose->x+half_map_wide)/GRIDWIDTH+3;
                ij=(current_pose->y+half_map_wide)/GRIDWIDTH;
                if(k>0&&k<GRIDWIDTH-2&&ij>=1&&ij<99){
                    gridmap.map[k-1][ij-1]=0;
                    gridmap.map[k-1][ij]=0;
                    gridmap.map[k-1][ij+1]=0;
                    gridmap.map[k][ij-1]=0;
                    gridmap.map[k][ij]=0;
                    gridmap.map[k][ij+1]=0;
                    gridmap.map[k+1][ij-1]=0;
                    gridmap.map[k+1][ij]=0;
                }
            }
            else if(my_abs(Yaw)>=135){
                k=(current_pose->x+half_map_wide)/GRIDWIDTH;
                ij=(current_pose->y+half_map_wide)/GRIDWIDTH+3;
                if(k>0&&k<GRIDWIDTH-1&&ij>0&&ij<GRIDWIDTH-1){
                    gridmap.map[k-1][ij-1]=0;
                    gridmap.map[k][ij-1]=0;
                    gridmap.map[k+1][ij-1]=0;
                    gridmap.map[k-1][ij]=0;
                    gridmap.map[k][ij]=0;
                    gridmap.map[k+1][ij]=0;
                    gridmap.map[k][ij+1]=0;
                    gridmap.map[k+1][ij+1]=0;
                }
            }
            else{
                k=(current_pose->x+half_map_wide)/GRIDWIDTH-3;
                ij=(current_pose->y+half_map_wide)/GRIDWIDTH;
                if(k>0&&k<99&&ij>0&&ij<98){
                    gridmap.map[k+1][ij-1]=0;
                    gridmap.map[k+1][ij]=0;
                    gridmap.map[k+1][ij+1]=0;
                    gridmap.map[k][ij-1]=0;
                    gridmap.map[k][ij]=0;
                    gridmap.map[k][ij+1]=0;
                    gridmap.map[k-1][ij]=0;
                    gridmap.map[k-1][ij+1]=0;
                }
            }
            last_position_x = current_pose->x;
            last_position_y = current_pose->y;
            step++;
            if(step>20){
                step=0;
                right_edge_dilemma_status = GOSTR_DELTA_Y_MORE_LATERAL_DIS_CYM_DILEMMA;
            }
            temporary_yaw = Yaw;
            break;
        }
        break;
    case COMPLETE_EL_DRYM:
        linear_velocity = 0;
        angular_velocity = 0;
        if(!detection_close_edge){
            detection_close_edge=true;
        }
        else{
            detection_close_edge=false;
            if(detection_close==true){
                detection_close=false;
                if(DelimmaNumber<2){
                    DelimmaNumber++;
                    right_edge_dilemma_status =CCLOCK_TARGET_YAW_MORE_ABS178_DILEMMA;
                }else{
                    right_edge_dilemma_status=0;
                    DelimmaNumber=0;
                    step=0;
                    complete_flag = 2;
                }
            }else{
                right_edge_dilemma_status=0;
                DelimmaNumber=0;
                step=0;
                complete_flag = 2;
            }
        }
        break;
    case GOSTR_COLLISION_DILEMMA:
        if(turn_start_update == 0)
        {
            turn_start_x = current_pose->x;
            turn_start_y = current_pose->y;
            turn_start_update = 1;
			if (obstacleSignal == left_obstacle){
                Astarmarkingobstacle = left_obstacle;
            }
            else if (obstacleSignal == right_obstacle){
                Astarmarkingobstacle = right_obstacle;
            }
            else{
                Astarmarkingobstacle = front_obstacle;
            }
        }
        linear_velocity = -long_stra_vel;
        angular_velocity = 0;
        if (my_abs(turn_start_x - current_pose->x) > collision_backward_distance || my_abs(turn_start_y - current_pose->y) > collision_backward_distance)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            turn_start_update = 0;
            temporary_yaw = Yaw;
			if(Astarmarkingobstacle == right_obstacle){
				right_edge_dilemma_status = CLOCK_TARGET_YAW_LESS_ABS90_DILEMMA;
            }
            else{
				right_edge_dilemma_status = FRONT_COLLISION_CLOCK_TARGET_YAW_LESS_ABS90_DILEMMA;
            }
            break;
        }
        break;
    case CLOCK_TARGET_YAW_LESS_ABS90_DILEMMA:
        if(my_abs(leakingsweep_x-current_pose->x)>500&&my_abs(leakingsweep_y-current_pose->y)>100){
			if((my_abs(current_pose->x)<half_map_wide-GRIDHEIGHT)&&(my_abs(current_pose->y)<half_map_wide-GRIDHEIGHT)){
            linear_velocity = 0;
            angular_velocity = 0;
            leakingsweep_x=current_pose->x;
            leakingsweep_y=current_pose->y;
            boolleaksweep=true;
            break;
			}
        }
        if (0 != leakingsweep)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            right_edge_dilemma_status=0;
            DelimmaNumber=0;
            step=0;
            right_running_step_status = LEAKING_SWEEP_RIGHTRUN_STEP;
            break;
        }
        else
        {
            linear_velocity = 0;
            angular_velocity = turn_vel;
            if (my_abs(Yaw - temporary_yaw) > 15)
            {
                if(my_abs(Yaw)>175){
                    right_edge_dilemma_status =CLOCK_TARGET_YAW_LESS_ABS90_DILEMMA;
                    temporary_yaw=Yaw;
                    break;
                }
                linear_velocity = 0;
                angular_velocity = 0;
                right_edge_dilemma_status =COLLISION_YAW_LESS_ABS90_DILEMMA;
                break;
            }
            if (obstacleSignal!=none_obstacle||(&cliff_valueB)->cliffValue0 == 1)
            {
                linear_velocity = 0;
                angular_velocity = 0;
                right_edge_dilemma_status = CLOCK_TARGET_YAW_LESS_ABS90_COLLISION_DILEMMA ;
                break;
            }
        }
        break;
    case CLOCK_TARGET_YAW_LESS_ABS90_COLLISION_DILEMMA:
        if(turn_start_update == 0)
        {
            turn_start_x = current_pose->x;
            turn_start_y = current_pose->y;
            turn_start_update = 1;
        }
        linear_velocity = -long_stra_vel;
        angular_velocity = 0;
        if (my_abs(turn_start_x - current_pose->x) > collision_backward_distance || my_abs(turn_start_y - current_pose->y) > collision_backward_distance)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            turn_start_update = 0;
            right_edge_dilemma_status = CLOCK_TARGET_YAW_LESS_ABS90_DILEMMA;
            break;
        }
        break;
		
	case FRONT_COLLISION_CLOCK_TARGET_YAW_LESS_ABS90_DILEMMA:
        if(my_abs(leakingsweep_x-current_pose->x)>500&&my_abs(leakingsweep_y-current_pose->y)>100){
			if((my_abs(current_pose->x)<half_map_wide-GRIDHEIGHT)&&(my_abs(current_pose->y)<half_map_wide-GRIDHEIGHT)){
            linear_velocity = 0;
            angular_velocity = 0;
            leakingsweep_x=current_pose->x;
            leakingsweep_y=current_pose->y;
            boolleaksweep=true;
            break;
			}
        }
        if (0 != leakingsweep)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            right_edge_dilemma_status=0;
            DelimmaNumber=0;
            step=0;
            right_running_step_status = LEAKING_SWEEP_RIGHTRUN_STEP;
            break;
        }
        else
        {
            linear_velocity = 0;
            angular_velocity = turn_vel;
            if (my_abs(Yaw - temporary_yaw) > 30)
            {
                if(my_abs(Yaw)>175){
                    right_edge_dilemma_status =FRONT_COLLISION_CLOCK_TARGET_YAW_LESS_ABS90_DILEMMA;
                    temporary_yaw=Yaw;
                    break;
                }
                linear_velocity = 0;
                angular_velocity = 0;
                right_edge_dilemma_status =COLLISION_YAW_LESS_ABS90_DILEMMA;
                break;
            }
            if (obstacleSignal!=none_obstacle||(&cliff_valueB)->cliffValue0 == 1)
            {
                linear_velocity = 0;
                angular_velocity = 0;
                right_edge_dilemma_status = FRONT_COLLISION_CLOCK_TARGET_YAW_LESS_ABS90_DILEMMA_COLLISION;
                break;
            }
        }
        break;
    case FRONT_COLLISION_CLOCK_TARGET_YAW_LESS_ABS90_DILEMMA_COLLISION:
        if(turn_start_update == 0)
        {
            turn_start_x = current_pose->x;
            turn_start_y = current_pose->y;
            turn_start_update = 1;
        }
        linear_velocity = -long_stra_vel;
        angular_velocity = 0;
        if (my_abs(turn_start_x - current_pose->x) > collision_backward_distance || my_abs(turn_start_y - current_pose->y) > collision_backward_distance)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            turn_start_update = 0;
            right_edge_dilemma_status = FRONT_COLLISION_CLOCK_TARGET_YAW_LESS_ABS90_DILEMMA;
            break;
        }
        break;
    case GOSTR_DELTA_Y_MORE_LATERAL_DIS_CYM_DILEMMA:
        if (-105<Yaw&&Yaw<-60)
        {
            linear_velocity = long_stra_vel;
            angular_velocity = 0;
            last_position_x = current_pose->x;
            last_position_y = current_pose->y;
            temporary_close_edge=dilemma_close_edge;
            right_edge_dilemma_status = LOOP_TEN_NUM_DILEMMA;
            break;
        }
        linear_velocity = 0;
        angular_velocity =-turn_vel;
        if (my_abs(temporary_yaw - Yaw) > 30||(-105<Yaw&&Yaw<-60))
        {
            linear_velocity = 0;
            angular_velocity = 0;
            last_position_x = current_pose->x;
            last_position_y = current_pose->y;
            temporary_close_edge=dilemma_close_edge;
            right_edge_dilemma_status = LOOP_TEN_NUM_DILEMMA;
            break;
        }
        if (obstacleSignal!=none_obstacle||(&cliff_valueB)->cliffValue0 == 1 )
        {
            linear_velocity = 0;
            angular_velocity = 0;
            right_edge_dilemma_status = TURN_CCLOCK_TARGET_YAW_LESS_135_DETAL_YAW_MORE_30_COLLISION_WE;
            break;
        }
        break;
        
    case TURN_CCLOCK_TARGET_YAW_LESS_135_DETAL_YAW_MORE_30_COLLISION_WE:
        if(turn_start_update == 0)
        {
            turn_start_x = current_pose->x;
            turn_start_y = current_pose->y;
            turn_start_update = 1;
        }
        linear_velocity = -long_stra_vel;
        angular_velocity = 0;
        if (my_abs(turn_start_x - current_pose->x) > collision_backward_distance || my_abs(turn_start_y - current_pose->y) > collision_backward_distance)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            turn_start_update = 0;
            temporary_yaw = Yaw;
            right_edge_dilemma_status = CLOCK_TARGET_YAW_LESS_ABS90_DILEMMA;
        }
        break;
    case COLLISION_YAW_MORE_ABS90_DILEMMA:
        linear_velocity = 0;
        angular_velocity = 0;
        last_position_x = current_pose->x;
        last_position_y = current_pose->y;
        temporary_close_edge=close_edge;
        right_edge_dilemma_status = GOSTR_CYM_DILEMMA;
        break;
    case GOSTR_CYM_DILEMMA:
		left_edge_judgment_repeat();
//        if(adcRealTime[8]>100&&adcRealTime[8]<1500){
//            delimma_edge=0;
//            linear_velocity = 200;
//            angular_velocity = 0;
//        }
//        if(adcRealTime[8]>=1500){
//            delimma_edge=0;
//            linear_velocity = 200;
//            angular_velocity = -10;
//        }
//        if(adcRealTime[8]<100){
//            if(delimma_edge<10){
//                delimma_edge++;
//                linear_velocity = 200;
//                angular_velocity = 10;
//            }
//            else{
//                linear_velocity = 100;
//                angular_velocity = 20;
//            }				
//        }
        if( current_pose->y>-return_origin_distance){
            linear_velocity = 0;
            angular_velocity = 0;
            DelimmaNumber=0;
            right_edge_dilemma_status = 0;
            step=0;
            complete_flag = 2;
            break;
        }
        if(my_abs(current_pose->x+half_map_wide)<100*close_edge_min_x+500){
            linear_velocity = 0;
            angular_velocity = 0;
            if(my_abs(leakingsweep_x-current_pose->x)>100&&my_abs(leakingsweep_y-current_pose->y)>100){
				if((my_abs(current_pose->x)<half_map_wide-GRIDHEIGHT)&&(my_abs(current_pose->y)<half_map_wide-GRIDHEIGHT)){
                leakingsweep_x=current_pose->x;
                leakingsweep_y=current_pose->y;
                boolleaksweep=true;
                break;
				}
            }
            if (0 != leakingsweep)
            {
                right_edge_dilemma_status=0;
                DelimmaNumber=0;
                step=0;
                right_running_step_status = LEAKING_SWEEP_RIGHTRUN_STEP;
                break;
            }
            else
            {
                linear_velocity = 0;
                angular_velocity = 0;
                right_edge_dilemma_status =DELTA_X_MORE_ONE_THIRD_CLEANED_MAP_WIDTH_DILEMMA;
                break;
            }
        }
        if (obstacleSignal!=none_obstacle||(&cliff_valueB)->cliffValue0 == 1)
        {
            right_edge_dilemma_status = GOSTR_COLLISION_CYM_DILEMMA;
            break;
        }
        if(Yaw<-75&&Yaw>-120){
            i=(current_pose->x+half_map_long)/100;
            j=(current_pose->y+half_map_long-110)/100;
            linear_velocity = long_stra_vel;
            angular_velocity = 0;
            if(j<0||i<1||j>MAPWIDECELLS-1||i>MAPLONGCELLS-2){
            }
            else{
                j=((gridmap.map[i-1][j]==125)?1:0)+((gridmap.map[i][j]==125)?1:0)+((gridmap.map[i][j]==125)?1:0);
                if(j==3){
                    right_edge_dilemma_status=0;
                    DelimmaNumber=0;
                    step=0;
					linear_velocity = 0;
					angular_velocity = 0;
                    complete_flag = 1;
                }
            }
            break;
        }		
        if (my_abs(last_position_x - current_pose->x) > dilemma_close_edge||my_abs(last_position_y - current_pose->y) > dilemma_close_edge)
        {
            if(my_abs(Yaw)>=135){
                k=(current_pose->x+half_map_wide)/GRIDWIDTH;
                ij=(current_pose->y+half_map_wide)/GRIDWIDTH-3;
                if(k>0&&k<98&&ij>0&&ij<GRIDWIDTH){
                    gridmap.map[k-1][ij+1]=0;
                    gridmap.map[k][ij+1]=0;
                    gridmap.map[k+1][ij+1]=0;
                    gridmap.map[k-1][ij]=0;
                    gridmap.map[k][ij]=0;
                    gridmap.map[k+1][ij]=0;
                    gridmap.map[k][ij-1]=0;
                    gridmap.map[k+1][ij-1]=0;
                }
            }
            else if(Yaw<-45&&Yaw>-135){
                k=(current_pose->x+half_map_wide)/GRIDWIDTH+3;
                ij=(current_pose->y+half_map_wide)/GRIDWIDTH;
                if(k>0&&k<GRIDWIDTH&&ij>0&&ij<98){
                    gridmap.map[k-1][ij-1]=0;
                    gridmap.map[k-1][ij]=0;
                    gridmap.map[k-1][ij+1]=0;
                    gridmap.map[k][ij-1]=0;
                    gridmap.map[k][ij]=0;
                    gridmap.map[k][ij+1]=0;
                    gridmap.map[k+1][ij]=0;
                    gridmap.map[k+1][ij+1]=0;
                }
            }
            else if(my_abs(Yaw)<=45){
                k=(current_pose->x+half_map_wide)/GRIDWIDTH;
                ij=(current_pose->y+half_map_wide)/GRIDWIDTH+3;
                if(k>1&&k<99&&ij>0&&ij<GRIDWIDTH){
                    gridmap.map[k-1][ij-1]=0;
                    gridmap.map[k][ij-1]=0;
                    gridmap.map[k+1][ij-1]=0;
                    gridmap.map[k-1][ij]=0;
                    gridmap.map[k][ij]=0;
                    gridmap.map[k+1][ij]=0;
                    gridmap.map[k][ij+1]=0;
                    gridmap.map[k-1][ij+1]=0;
                }
            }
            else{
                k=(current_pose->x+half_map_wide)/GRIDWIDTH-3;
                ij=(current_pose->y+half_map_wide)/GRIDWIDTH;
                if(k>=0&&k<99&&ij>1&&ij<99){
                    gridmap.map[k+1][ij-1]=0;
                    gridmap.map[k+1][ij]=0;
                    gridmap.map[k+1][ij+1]=0;
                    gridmap.map[k][ij-1]=0;
                    gridmap.map[k][ij]=0;
                    gridmap.map[k][ij+1]=0;
                    gridmap.map[k-1][ij]=0;
                    gridmap.map[k-1][ij-1]=0;
                }
            }
            last_position_x = current_pose->x;
            last_position_y = current_pose->y;
            step++;
            if(step>40){
                step=0;
                right_edge_dilemma_status = GOSTR_GOSTR_DELTA_Y_MORE_LATERAL_DIS_CYL_DILEMMA;
            }	
            temporary_yaw = Yaw;
            break;
        }
        break;
        
    case GOSTR_GOSTR_DELTA_Y_MORE_LATERAL_DIS_CYL_DILEMMA:
        if (Yaw<-75&&Yaw>-120)
        {
            last_position_x = current_pose->x;
            last_position_y = current_pose->y;
            linear_velocity = long_stra_vel;
            angular_velocity = 0;
            temporary_close_edge=dilemma_close_edge;
            right_edge_dilemma_status =GOSTR_CYM_DILEMMA;
            break;
        }
        linear_velocity = 0;
        angular_velocity =turn_vel;
        if (my_abs(temporary_yaw - Yaw) > 30||(Yaw<-75&&Yaw>-120))
        {
            linear_velocity = 0;
            angular_velocity = 0;
            last_position_x = current_pose->x;
            last_position_y = current_pose->y;
            temporary_close_edge=dilemma_close_edge;
            right_edge_dilemma_status =GOSTR_CYM_DILEMMA;
            break;
        }
        if (obstacleSignal!=none_obstacle||(&cliff_valueB)->cliffValue0 == 1)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            right_edge_dilemma_status = GOSTR_COLLISION_CYL_DILEMMA;
            break;
        }
        break;
        
    case GOSTR_COLLISION_CYL_DILEMMA:
        if(turn_start_update == 0)
        {
            turn_start_x = current_pose->x;
            turn_start_y = current_pose->y;
            turn_start_update = 1;
        }
        linear_velocity = -long_stra_vel;
        angular_velocity = 0;
        if (my_abs(turn_start_x - current_pose->x) > collision_backward_distance || my_abs(turn_start_y - current_pose->y) > collision_backward_distance)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            turn_start_update = 0;
            right_edge_dilemma_status = CLOCK_TARGET_YAW_LESS_ABS3_DILEMMA;
        }
        break;
        
    case GOSTR_COLLISION_CYM_DILEMMA:
        if(turn_start_update == 0)
        {
            turn_start_x = current_pose->x;
            turn_start_y = current_pose->y;
            turn_start_update = 1;
			if (obstacleSignal == left_obstacle){
                Astarmarkingobstacle = left_obstacle;
            }
            else if (obstacleSignal == right_obstacle){
                Astarmarkingobstacle = right_obstacle;
            }
            else{
                Astarmarkingobstacle = front_obstacle;
            }
        }
        linear_velocity = -long_stra_vel;
        angular_velocity = 0;
        if (my_abs(turn_start_x - current_pose->x) > collision_backward_distance || my_abs(turn_start_y - current_pose->y) > collision_backward_distance)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            turn_start_update = 0;
            temporary_yaw = Yaw;
			
			if(Astarmarkingobstacle == left_obstacle){
				right_edge_dilemma_status = CLOCK_TARGET_YAW_LESS_ABS3_DILEMMA;
            }
            else{
				right_edge_dilemma_status = FRONT_COLLISION_CLOCK_TARGET_YAW_LESS_ABS3_DILEMMA;
            }
            break;
        }
        break;
    case CLOCK_TARGET_YAW_LESS_ABS3_DILEMMA:
        if(my_abs(leakingsweep_x-current_pose->x)>500&&my_abs(leakingsweep_y-current_pose->y)>100){
			if((my_abs(current_pose->x)<half_map_wide-GRIDHEIGHT)&&(my_abs(current_pose->y)<half_map_wide-GRIDHEIGHT)){
            leakingsweep_x=current_pose->x;
            leakingsweep_y=current_pose->y;
            boolleaksweep=true;
            break;
			}
        }
        if (0 != leakingsweep)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            right_edge_dilemma_status=0;
            DelimmaNumber=0;
            step=0;
            right_running_step_status = LEAKING_SWEEP_RIGHTRUN_STEP;
            break;
        }
        else
        {
            
            linear_velocity = 0;
            angular_velocity =-turn_vel;
            if (my_abs(Yaw - temporary_yaw) > 15)
            {
                if(my_abs(Yaw)>175){
                    right_edge_dilemma_status =CLOCK_TARGET_YAW_LESS_ABS3_DILEMMA;
                    temporary_yaw=Yaw;
                    break;
                }
                linear_velocity = 0;
                angular_velocity = 0;
                right_edge_dilemma_status =COLLISION_YAW_MORE_ABS90_DILEMMA;
                break;
            }
            if (obstacleSignal!=none_obstacle||(&cliff_valueB)->cliffValue0 == 1)
            {
                linear_velocity = 0;
                angular_velocity = 0;
                right_edge_dilemma_status = CLOCK_TARGET_YAW_LESS_ABS3_COLLISION_DILEMMA;
                break;
            }
        }
        break;
    case CLOCK_TARGET_YAW_LESS_ABS3_COLLISION_DILEMMA:
        if(turn_start_update == 0)
        {
            turn_start_x = current_pose->x;
            turn_start_y = current_pose->y;
            turn_start_update = 1;
        }
        linear_velocity = -long_stra_vel;
        angular_velocity = 0;
        if (my_abs(turn_start_x - current_pose->x) > collision_backward_distance || my_abs(turn_start_y - current_pose->y) > collision_backward_distance)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            turn_start_update = 0;
            right_edge_dilemma_status = CLOCK_TARGET_YAW_LESS_ABS3_DILEMMA ;
            break;
        }
        break;
		
    case FRONT_COLLISION_CLOCK_TARGET_YAW_LESS_ABS3_DILEMMA:
        if(my_abs(leakingsweep_x-current_pose->x)>500&&my_abs(leakingsweep_y-current_pose->y)>100){
			if((my_abs(current_pose->x)<half_map_wide-GRIDHEIGHT)&&(my_abs(current_pose->y)<half_map_wide-GRIDHEIGHT)){
            leakingsweep_x=current_pose->x;
            leakingsweep_y=current_pose->y;
            boolleaksweep=true;
            break;
			}
        }
        if (0 != leakingsweep)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            right_edge_dilemma_status=0;
            DelimmaNumber=0;
            step=0;
            right_running_step_status = LEAKING_SWEEP_RIGHTRUN_STEP;
            break;
        }
        else
        {
            
            linear_velocity = 0;
            angular_velocity =-turn_vel;
            if (my_abs(Yaw - temporary_yaw) > 15)
            {
                if(my_abs(Yaw)>170){
                    right_edge_dilemma_status =FRONT_COLLISION_CLOCK_TARGET_YAW_LESS_ABS3_DILEMMA;
                    temporary_yaw=Yaw;
                    break;
                }
                linear_velocity = 0;
                angular_velocity = 0;
                right_edge_dilemma_status =COLLISION_YAW_MORE_ABS90_DILEMMA;
                break;
            }
            if (obstacleSignal!=none_obstacle||(&cliff_valueB)->cliffValue0 == 1)
            {
                linear_velocity = 0;
                angular_velocity = 0;
                right_edge_dilemma_status = FRONT_COLLISION_CLOCK_TARGET_YAW_LESS_ABS3_DILEMMA_COLLISION;
                break;
            }
        }
        break;
    case FRONT_COLLISION_CLOCK_TARGET_YAW_LESS_ABS3_DILEMMA_COLLISION:
        if(turn_start_update == 0)
        {
            turn_start_x = current_pose->x;
            turn_start_y = current_pose->y;
            turn_start_update = 1;
        }
        linear_velocity = -long_stra_vel;
        angular_velocity = 0;
        if (my_abs(turn_start_x - current_pose->x) > collision_backward_distance || my_abs(turn_start_y - current_pose->y) > collision_backward_distance)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            turn_start_update = 0;
            right_edge_dilemma_status = FRONT_COLLISION_CLOCK_TARGET_YAW_LESS_ABS3_DILEMMA;
            break;
        }
        break;
    case DELTA_X_MORE_ONE_THIRD_CLEANED_MAP_WIDTH_DILEMMA:
        linear_velocity = 0;
        angular_velocity = 0;
        if(!detection_close_edge){
            detection_close_edge=true;
        }
        else{
            detection_close_edge=false;
            if(detection_close==true){
                detection_close=false;
                if(DelimmaNumber<2){
                    DelimmaNumber++;
                    right_edge_dilemma_status = YAW_MORE_ABS90_LESS_ABS176_CCLOCK_LESS_ABS90_MORE_ABS4_CLOCK_DILEMMA;
                }else{
                    right_edge_dilemma_status=0;
                    DelimmaNumber=0;
                    step=0;
                    complete_flag = 2;
                }
            }else{
                right_edge_dilemma_status=0;
                DelimmaNumber=0;
                step=0;
                complete_flag = 2;
            }
        }
        break;
    }
    return complete_flag;
}



unsigned char  RightWalkEdge(POSE *current_pose,unsigned char obstacleSignal){
    int Yaw;
    unsigned char complete_flag=0;
    signed char i,j;
    Yaw = current_pose->orientation;
    Yaw = Yaw /100;
    if (my_abs(current_pose->x)>half_map_wide-2*GRIDWIDTH||my_abs(current_pose->y)>half_map_wide-2*GRIDWIDTH){
        if(my_abs(leakingsweep_x-current_pose->x)>500&&my_abs(leakingsweep_y-current_pose->y)>100){
			if((my_abs(current_pose->x)<half_map_wide-GRIDHEIGHT)&&(my_abs(current_pose->y)<half_map_wide-GRIDHEIGHT)){
            leakingsweep_x=current_pose->x;
            leakingsweep_y=current_pose->y;
            boolleaksweep=true;
            return complete_flag;
			}
        }
        if (0 != leakingsweep)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            returnorigin=false;
            right_walk_edge_status = 0;
            right_running_step_status = LEAKING_SWEEP_RIGHTRUN_STEP;
            return complete_flag;
        }
        else
        { 
            linear_velocity = 0;
            angular_velocity = 0;
            returnorigin=false;
            right_walk_edge_status = 0;
            complete_flag = 3;
            return complete_flag;
        }
    }
    switch(right_walk_edge_status)
    {
    case 0:
        right_walk_edge_status = GOBACK_WALK_EDGE;
        break;
    case GOBACK_WALK_EDGE:
        if(turn_start_update == 0)
        {
            turn_start_x = current_pose->x;
            turn_start_y = current_pose->y;
            turn_start_update = 1;
        }
        linear_velocity = -long_stra_vel;
        angular_velocity = 0;
        if (my_abs(turn_start_x - current_pose->x) > turn_backward_distance || my_abs(turn_start_y - current_pose->y) > turn_backward_distance)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            turn_start_update = 0;
            right_walk_edge_status = TURN_CLOCK_TARGET_YAW_MORE_ABS117_WE;
            break;
        }
        break;
    case TURN_CLOCK_TARGET_YAW_MORE_ABS117_WE:
        linear_velocity = 0;
        angular_velocity = -turn_vel;
        if (Yaw < -150)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            right_walk_edge_status = READY_GOSTR_BYPASS_WE;
            break;
        }
        if (obstacleSignal !=none_obstacle)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            right_walk_edge_status = TURN_CLOCK_TARGET_YAW_MORE_ABS117_COLLISION_WE;
            break;
        }
        break;
    case TURN_CLOCK_TARGET_YAW_MORE_ABS117_COLLISION_WE:
        if(turn_start_update == 0)
        {
            turn_start_x = current_pose->x;
            turn_start_y = current_pose->y;
            turn_start_update = 1;
        }
        linear_velocity = -long_stra_vel;
        angular_velocity = 0;
        if (my_abs(turn_start_x - current_pose->x) > collision_backward_distance || my_abs(turn_start_y - current_pose->y) > collision_backward_distance)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            turn_start_update = 0;
            right_walk_edge_status = TURN_CLOCK_TARGET_YAW_MORE_ABS117_WE;
            break;
        }
        break;
    case READY_GOSTR_BYPASS_WE:
        edge_length_start = current_pose->x + half_map_wide;
        returnorigin = false;
        b_last_position_yy = false;
        last_position_yy = 0;
        temporary_close_edge = close_edge;
        right_walk_edge_status = GOSTR_BYPASS_WE_X;
        break;
        
    case GOSTR_BYPASS_WE_X:
        linear_velocity = 0;
        angular_velocity = 0;
        last_position_x = current_pose->x;
        right_walk_edge_status = GOSTR_BYPASS_WE;
        break;
        
    case GOSTR_BYPASS_WE:
		if(Yaw>-75&&Yaw<0){
			  linear_velocity = 100;
              angular_velocity = -20;
		}
		else{
			left_edge_judgment_repeat();
//        if(adcRealTime[8]>100&&adcRealTime[8]<1500){
//            delimma_edge=0;
//            linear_velocity = 200;
//            angular_velocity = 0;
//        }
//        if(adcRealTime[8]>=1500){
//            delimma_edge=0;
//            linear_velocity = 200;
//            angular_velocity = -10;
//        }
//        if(adcRealTime[8]<100){
//            if(delimma_edge<10){
//                delimma_edge++;
//                linear_velocity = 200;
//                angular_velocity = 10;			
//            }else{
//                linear_velocity = 100;
//                angular_velocity = 20;
//                
//            }
//        }
		}
        if (obstacleSignal !=none_obstacle||(&cliff_valueB)->cliffValue0 == 1)
        {
            right_walk_edge_status = COLLISION_BYPASS_WE;
            break;
        }
        if (my_abs(last_position_x - current_pose->x) > lateral_move_distance)
        {
            if(Yaw>-135&&Yaw<0){
                right_walk_edge_status = GOSTR_X_MORE_LATERALDIS_BYPASS_WE;
            }
            else{
                i=(current_pose->x+half_map_wide)/GRIDWIDTH;
                j=(current_pose->y+half_map_wide)/GRIDWIDTH-3;
                if(i>0&&i<98&&j>=0&&j<GRIDWIDTH)
                gridmap.map[i-1][j]=0;
                gridmap.map[i][j]=0;
                gridmap.map[i+1][j]=0;
                gridmap.map[i-1][j+1]=0;
                gridmap.map[i][j+1]=0;
                gridmap.map[i+1][j+1]=0;
                gridmap.map[i][j-1]=0;
                gridmap.map[i+1][j-1]=0;
                last_position_x = current_pose->x;
            }
            break;			
        }
        if (last_position_y - current_pose->y > temporary_close_edge)
        {
            right_walk_edge_status = BOW_CONTINUE_WE;
            break;
        }
        if (b_last_position_yy == true && my_abs(last_position_yy - current_pose->y) > 3 * lateral_move_distance)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            right_walk_edge_status = TARGET_YAW_LESS_ABS105_MORE_0_BYPASS_WE;
            break;
        }
        if (my_abs(Yaw) < 120 && (Yaw > 0) && (b_last_position_yy == false))
        {
            last_position_yy = current_pose->y;
            b_last_position_yy = true;
            break;
        }
        break;
        
    case REBACK_GOSTR_BYPASS_CHECK_WE:
        if((Yaw < 135) && (Yaw > 0))
        {
            right_walk_edge_status = TARGET_YAW_LESS_ABS105_MORE_0_BYPASS_WE;
            break;
        }
        if(!returnorigin){
            if(my_abs(current_pose->x + half_map_wide - edge_length_start) > 500){
				if(my_abs(current_pose->x + half_map_wide- edge_length_start) > Edge_length()/2){
                right_walk_edge_status = DELTA_X_MORE_ONE_FOURTH_CLEANED_MAP_WIDTH_WE;
                break;
				}
            }
        }
        right_walk_edge_status = GOSTR_BYPASS_WE_X;
        break;
    case COLLISION_BYPASS_WE:
        if(turn_start_update == 0)
        {
            turn_start_x = current_pose->x;
            turn_start_y = current_pose->y;
            turn_start_update = 1;
			if (obstacleSignal == left_obstacle){
                Astarmarkingobstacle = left_obstacle;
            }
            else if (obstacleSignal == right_obstacle){
                Astarmarkingobstacle = right_obstacle;
            }
            else{
                Astarmarkingobstacle = front_obstacle;
            }
        }
        linear_velocity = -long_stra_vel;
        angular_velocity = 0;
        if (my_abs(turn_start_x - current_pose->x) > collision_backward_distance || my_abs(turn_start_y - current_pose->y) > collision_backward_distance)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            turn_start_update = 0;
            temporary_yaw = Yaw;
			if(Astarmarkingobstacle==left_obstacle){
				right_walk_edge_status = TURN_CLOCK_YAW_ADD_ABS15_WE;
            }
            else{
                right_walk_edge_status = FRONT_COLLISION_TURN_CLOCK_YAW_ADD_ABS15_WE;
            }
            break;
        }
        break;			
    case TURN_CLOCK_YAW_ADD_ABS15_WE:
        linear_velocity = 0;
        angular_velocity = -turn_vel;
        if (my_abs(Yaw - temporary_yaw) > 15)
        {
            if(my_abs(Yaw)>175){
                right_walk_edge_status =TURN_CLOCK_YAW_ADD_ABS15_WE;
                temporary_yaw=Yaw;
                break;
            }
            linear_velocity = 0;
            angular_velocity = 0;
            right_walk_edge_status = REBACK_GOSTR_BYPASS_CHECK_WE;
            break;
        }
        if (obstacleSignal !=none_obstacle||(&cliff_valueB)->cliffValue0 == 1)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            right_walk_edge_status = TURN_CLOCK_YAW_ADD_ABS15_COLLISION_WE;
            break;
        }
        break;
    case TURN_CLOCK_YAW_ADD_ABS15_COLLISION_WE:
        if(turn_start_update == 0)
        {
            turn_start_x = current_pose->x;
            turn_start_y = current_pose->y;
            turn_start_update = 1;
        }
        linear_velocity = -long_stra_vel;
        angular_velocity = 0;
        if (my_abs(turn_start_x - current_pose->x) > collision_backward_distance || my_abs(turn_start_y - current_pose->y) > collision_backward_distance)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            turn_start_update = 0;
            right_walk_edge_status = TURN_CLOCK_YAW_ADD_ABS15_WE;
            break;
        }
        break;
	
	case FRONT_COLLISION_TURN_CLOCK_YAW_ADD_ABS15_WE:
        linear_velocity = 0;
        angular_velocity = -turn_vel;
        if (my_abs(Yaw - temporary_yaw) > 30)
        {
            if(my_abs(Yaw)>175){
                right_walk_edge_status =FRONT_COLLISION_TURN_CLOCK_YAW_ADD_ABS15_WE;
                temporary_yaw=Yaw;
                break;
            }
            linear_velocity = 0;
            angular_velocity = 0;
            right_walk_edge_status = REBACK_GOSTR_BYPASS_CHECK_WE;
            break;
        }
        if (obstacleSignal !=none_obstacle||(&cliff_valueB)->cliffValue0 == 1)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            right_walk_edge_status = FRONT_COLLISION_TURN_CLOCK_YAW_ADD_ABS15_COLLISION_WE;
            break;
        }
        break;
    case FRONT_COLLISION_TURN_CLOCK_YAW_ADD_ABS15_COLLISION_WE:
        if(turn_start_update == 0)
        {
            turn_start_x = current_pose->x;
            turn_start_y = current_pose->y;
            turn_start_update = 1;
        }
        linear_velocity = -long_stra_vel;
        angular_velocity = 0;
        if (my_abs(turn_start_x - current_pose->x) > collision_backward_distance || my_abs(turn_start_y - current_pose->y) > collision_backward_distance)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            turn_start_update = 0;
            right_walk_edge_status = FRONT_COLLISION_TURN_CLOCK_YAW_ADD_ABS15_WE;
            break;
        }
        break;			
    case TARGET_YAW_LESS_ABS105_MORE_0_BYPASS_WE:
        linear_velocity = 0;
        angular_velocity = 0;
        if(returnorigin==true){
            right_walk_edge_status = RETURN_ORIGIN_WE;
            break;
        } 
        if(!detection_close_edge){
            detection_close_edge=true;
        }
        else{
            detection_close_edge=false;
            if(detection_close==true){
                detection_close=false;
                right_walk_edge_status = RIGHT_EDGE_DILEMMA_WE;
                break;
            }else{
                right_walk_edge_status = RETURN_ORIGIN_WE;
                break;
            }
        }
        break;
    case DELTA_X_MORE_ONE_FOURTH_CLEANED_MAP_WIDTH_WE:
        linear_velocity = 0;
        angular_velocity = 0;
        if(!detection_close_edge){
            detection_close_edge=true;
        }
        else{
            detection_close_edge=false;
            if(detection_close==true){
                detection_close=false;
                temporary_yaw = Yaw;
                right_walk_edge_status = CLOSE_EDGE_MAP_RIGHT_WALK_TURN_CLOCK_YAW_ADD_ABS15_WE;
            }else{
                returnorigin=true;
                right_walk_edge_status = GOSTR_BYPASS_WE_X;
            }
        }
        break;
        
    case  CLOSE_EDGE_MAP_RIGHT_WALK_TURN_CLOCK_YAW_ADD_ABS15_WE:
        linear_velocity = 0;
        angular_velocity = -turn_vel;
        if (my_abs(Yaw - temporary_yaw) > 15)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            last_position_x = current_pose->x;
            right_walk_edge_status = GOSTR_BYPASS_WE;
            break;
        }
        if (obstacleSignal !=none_obstacle||(&cliff_valueB)->cliffValue0 == 1)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            right_walk_edge_status = COLLISION_CLOSE_EDGE_MAP_RIGHT_WALK_TURN_CLOCK_YAW_ADD_ABS15_WE;
            break;
        }
        break;
    case COLLISION_CLOSE_EDGE_MAP_RIGHT_WALK_TURN_CLOCK_YAW_ADD_ABS15_WE:
        if(turn_start_update == 0)
        {
            turn_start_x = current_pose->x;
            turn_start_y = current_pose->y;
            turn_start_update = 1;
        }
        linear_velocity = -long_stra_vel;
        angular_velocity = 0;
        if (my_abs(turn_start_x - current_pose->x) > collision_backward_distance || my_abs(turn_start_y - current_pose->y) > collision_backward_distance)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            turn_start_update = 0;
            right_walk_edge_status = CLOSE_EDGE_MAP_RIGHT_WALK_TURN_CLOCK_YAW_ADD_ABS15_WE;
            break;
        }
        break;
        
    case TURN_CLOCK_TARGET_YAW_LESS_ABS3_WE:
        if (my_abs(Yaw) < 5)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            right_walk_edge_status = RIGHT_EDGE_DILEMMA_WE;
            break;
        }
        if (obstacleSignal !=none_obstacle||(&cliff_valueB)->cliffValue0 == 1)
        {
            right_walk_edge_status = TURN_CLOCK_TARGET_YAW_LESS_ABS3_COLLISION_WE;
            break;
        }
        linear_velocity = 0;
        angular_velocity = -turn_vel;
        break;
        
    case TURN_CLOCK_TARGET_YAW_LESS_ABS3_COLLISION_WE:
        if(turn_start_update == 0)
        {
            turn_start_x = current_pose->x;
            turn_start_y = current_pose->y;
            turn_start_update = 1;
        }
        linear_velocity = -long_stra_vel;
        angular_velocity = 0;
        if (my_abs(turn_start_x - current_pose->x) > collision_backward_distance || my_abs(turn_start_y - current_pose->y) > collision_backward_distance)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            turn_start_update = 0;
            right_walk_edge_status = TURN_CLOCK_TARGET_YAW_LESS_ABS3_WE;
            break;
        }
        break;
    case GOSTR_X_MORE_LATERALDIS_BYPASS_WE:
        temporary_yaw = Yaw;
        if (my_abs(Yaw) < 135 && (Yaw > 0))
        {
            right_walk_edge_status = TURN_CCLOCK_TARGET_YAW_LESS_0_WE;
        }
        else if (my_abs(Yaw) > 135)
        {
            right_walk_edge_status = TURN_CCLOCK_TARGET_YAW_LESS_135_DETAL_YAW_MORE_30_WE;
        }
        else{
            right_walk_edge_status = GOSTR_BYPASS_WE_X;
        }
        break;
    case TURN_CCLOCK_TARGET_YAW_LESS_0_WE:
        linear_velocity = 0;
        angular_velocity = turn_vel;
        if (Yaw  < 0 )
        {
            linear_velocity = 0;
            angular_velocity = 0;
            right_walk_edge_status = GOSTR_BYPASS_WE_X;
            break;
        }
        if (obstacleSignal !=none_obstacle||(&cliff_valueB)->cliffValue0 == 1)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            right_walk_edge_status = TURN_CCLOCK_TARGET_YAW_LESS_0_COLLISION_WE;
            break;
        }
        break;
    case TURN_CCLOCK_TARGET_YAW_LESS_0_COLLISION_WE:
        if(turn_start_update == 0)
        {
            turn_start_x = current_pose->x;
            turn_start_y = current_pose->y;
            turn_start_update = 1;
        }
        linear_velocity = -long_stra_vel;
        angular_velocity = 0;
        if (my_abs(turn_start_x - current_pose->x) > collision_backward_distance || my_abs(turn_start_y - current_pose->y) > collision_backward_distance)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            turn_start_update = 0;
            temporary_yaw = Yaw;
            right_walk_edge_status = TURN_CLOCK_YAW_ADD_ABS15_WE;
        }
        break;
    case TURN_CCLOCK_TARGET_YAW_LESS_135_DETAL_YAW_MORE_30_WE:
        linear_velocity = 0;
        angular_velocity = turn_vel;
        if (my_abs(temporary_yaw - Yaw) > 30 && my_abs(Yaw) < 135)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            right_walk_edge_status = GOSTR_BYPASS_WE_X;
            break;
        }
        if (obstacleSignal !=none_obstacle )
        {
            linear_velocity = 0;
            angular_velocity = 0;
            right_walk_edge_status = TURN_CCLOCK_TARGET_YAW_LESS_135_DETAL_YAW_MORE_30_COLLISION_WE;
            break;
        }
        break;
    case TURN_CCLOCK_TARGET_YAW_LESS_135_DETAL_YAW_MORE_30_COLLISION_WE:
        if(turn_start_update == 0)
        {
            turn_start_x = current_pose->x;
            turn_start_y = current_pose->y;
            turn_start_update = 1;
        }
        linear_velocity = -long_stra_vel;
        angular_velocity = 0;
        if (my_abs(turn_start_x - current_pose->x) > collision_backward_distance || my_abs(turn_start_y - current_pose->y) > collision_backward_distance)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            turn_start_update = 0;
            temporary_yaw = Yaw;
            right_walk_edge_status = TURN_CLOCK_YAW_ADD_ABS15_WE;
        }
        break;
    case BOW_CONTINUE_WE:
        right_walk_edge_status = 0;
        returnorigin=false;
        complete_flag = 1;
        break;
    case RIGHT_EDGE_DILEMMA_WE:
        linear_velocity = 0;
        angular_velocity = 0;
        if(my_abs(leakingsweep_x-current_pose->x)>500&&my_abs(leakingsweep_y-current_pose->y)>100){
			if((my_abs(current_pose->x)<half_map_wide-GRIDHEIGHT)&&(my_abs(current_pose->y)<half_map_wide-GRIDHEIGHT)){
            leakingsweep_x=current_pose->x;
            leakingsweep_y=current_pose->y;
            boolleaksweep=true;
            break;
			}
        }
        if (0 != leakingsweep)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            returnorigin=false;
            right_walk_edge_status = 0;
            right_running_step_status = LEAKING_SWEEP_RIGHTRUN_STEP;
            break;
        }
        else
        { 
            linear_velocity = 0;
            angular_velocity = 0;
            returnorigin=false;
            complete_flag = 2;
            returnorigin=false;
            right_walk_edge_status = 0;
        }
        break;
    case RETURN_ORIGIN_WE:
        linear_velocity = 0;
        angular_velocity = 0;
        if(my_abs(leakingsweep_x-current_pose->x)>500&&my_abs(leakingsweep_y-current_pose->y)>100){
			if((my_abs(current_pose->x)<half_map_wide-GRIDHEIGHT)&&(my_abs(current_pose->y)<half_map_wide-GRIDHEIGHT)){
            leakingsweep_x=current_pose->x;
            leakingsweep_y=current_pose->y;
            boolleaksweep=true;
            break;
			}
        }
        if (0 != leakingsweep)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            returnorigin=false;
            right_walk_edge_status = 0;
            right_running_step_status = LEAKING_SWEEP_RIGHTRUN_STEP;
            break;
        }
        else
        { 
            linear_velocity = 0;
            angular_velocity = 0;
            returnorigin=false;
            right_walk_edge_status = 0;
            complete_flag = 3;
        }
        break;
    }
    return complete_flag;
}





unsigned char  RightReverseWalkEdge(POSE *current_pose,unsigned char obstacleSignal){
    int Yaw;
    unsigned char complete_flag=0;
    signed char i,j;
    Yaw = current_pose->orientation;
    Yaw = Yaw /100;
    if (my_abs(current_pose->x)>half_map_wide-2*GRIDWIDTH||my_abs(current_pose->y)>half_map_wide-2*GRIDWIDTH){
        if(my_abs(leakingsweep_x-current_pose->x)>500&&my_abs(leakingsweep_y-current_pose->y)>100){
			if((my_abs(current_pose->x)<half_map_wide-GRIDHEIGHT)&&(my_abs(current_pose->y)<half_map_wide-GRIDHEIGHT)){
            linear_velocity = 0;
            angular_velocity = 0;
            leakingsweep_x=current_pose->x;
            leakingsweep_y=current_pose->y;
            boolleaksweep=true;
            return complete_flag;
			}
        }
        if (0 != leakingsweep)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            right_reverse_walk_edge_status =0;
            returnorigin=false;
            right_running_step_status = LEAKING_SWEEP_RIGHTRUN_STEP;
            return complete_flag;
        }
        else
        { 
            linear_velocity = 0;
            angular_velocity = 0;
            right_reverse_walk_edge_status = 0;
            returnorigin=false;
            complete_flag = 3;
            return complete_flag;
        }
    }
    
    switch(right_reverse_walk_edge_status)
    {
    case 0:
        right_reverse_walk_edge_status = GOBACK_REVERSE_WALK_EDGE;
        break;
    case GOBACK_REVERSE_WALK_EDGE:
        if(turn_start_update == 0)
        {
            turn_start_x = current_pose->x;
            turn_start_y = current_pose->y;
            turn_start_update = 1;
        }
        linear_velocity = -long_stra_vel;
        angular_velocity = 0;
        if (my_abs(turn_start_x - current_pose->x) >collision_backward_distance || my_abs(turn_start_y - current_pose->y) > collision_backward_distance)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            turn_start_update = 0;
            right_reverse_walk_edge_status = TURN_CCLOCK_TARGET_YAW_LESS_ABS63_RWE;
            break;
        }
        break;
    case TURN_CCLOCK_TARGET_YAW_LESS_ABS63_RWE:
        linear_velocity = 0;
        angular_velocity = turn_vel;
        if((Yaw>-30)&&(Yaw<0))
        {
            linear_velocity = 0;
            angular_velocity = 0;
            right_reverse_walk_edge_status = READY_GOSTR_BYPASS_RWE;
            break;
        }
        if (obstacleSignal !=none_obstacle||(&cliff_valueB)->cliffValue0 == 1)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            right_reverse_walk_edge_status = TURN_CCLOCK_TARGET_YAW_LESS_ABS63_COLLISION_RWE;
            break;
        }
        break;
    case TURN_CCLOCK_TARGET_YAW_LESS_ABS63_COLLISION_RWE:
        if(turn_start_update == 0)
        {
            turn_start_x = current_pose->x;
            turn_start_y = current_pose->y;
            turn_start_update = 1;
        }
        linear_velocity = -long_stra_vel;
        angular_velocity = 0;
        if (my_abs(turn_start_x - current_pose->x) > collision_backward_distance || my_abs(turn_start_y - current_pose->y) > collision_backward_distance)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            turn_start_update = 0;
            right_reverse_walk_edge_status = TURN_CCLOCK_TARGET_YAW_LESS_ABS63_RWE;
            break;
        }
        break;
    case READY_GOSTR_BYPASS_RWE:
        edge_length_start = current_pose->x + half_map_wide;
        returnorigin = false;
        b_last_position_yy = false;
        last_position_yy = 0;
        temporary_close_edge = close_edge;
        right_reverse_walk_edge_status = GOSTR_BYPASS_RWE_X;
        break;
    case GOSTR_BYPASS_RWE_X:
        linear_velocity = 0;
        angular_velocity = 0;
        last_position_x = current_pose->x;
        right_reverse_walk_edge_status = GOSTR_BYPASS_RWE;
        break;
        
    case GOSTR_BYPASS_RWE:
        if (obstacleSignal !=none_obstacle||(&cliff_valueB)->cliffValue0 == 1)
        {
            right_reverse_walk_edge_status = COLLISION_BYPASS_RWE;
            break;
        }
		right_edge_judgment_repeat();
//        if(adcRealTime[9]>100&&adcRealTime[9]<1500){
//            delimma_edge=0;
//            linear_velocity = 200;
//            angular_velocity = 0;
//        }
//        if(adcRealTime[9]>=1500){
//            delimma_edge=0;
//            linear_velocity = 200;
//            angular_velocity = 10;
//        }
//        if(adcRealTime[9]<100){
//            if(delimma_edge<10){
//                delimma_edge++;
//                linear_velocity = 200;
//                angular_velocity = -10;
//            }
//            else{
//                linear_velocity = 100;
//                angular_velocity = -20;
//            }
//        }
        if (my_abs(last_position_x - current_pose->x) > lateral_move_distance)
        {
            if(Yaw>20){				
                right_reverse_walk_edge_status = GOSTR_X_MORE_LATERALDIS_BYPASS_RWE;
            }
            else{
                i=(current_pose->x+half_map_wide)/GRIDWIDTH;
                j=(current_pose->y+half_map_wide)/GRIDWIDTH-3;
                if(i>1&&i<99&&j>=0&&j<GRIDWIDTH)
                gridmap.map[i-1][j]=0;
                gridmap.map[i][j]=0;
                gridmap.map[i+1][j]=0;
                gridmap.map[i-1][j+1]=0;
                gridmap.map[i][j+1]=0;
                gridmap.map[i+1][j+1]=0;
                gridmap.map[i][j-1]=0;
                gridmap.map[i-1][j-1]=0;	
                last_position_x = current_pose->x;
            }
            break;
        }
        if (last_position_y - current_pose->y > temporary_close_edge)
        {
            right_reverse_walk_edge_status = BOW_CONTINUE_RWE;
            break;
        }
        if (b_last_position_yy == true && my_abs(last_position_yy - current_pose->y) > 3 * lateral_move_distance)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            right_reverse_walk_edge_status = TARGET_YAW_MORE_ABS75_MORE_ABS0_RWE;
            break;
        }
        if (my_abs(Yaw) < 120 && (Yaw > 0) && (b_last_position_yy == false))
        {
            last_position_yy = current_pose->y;
            b_last_position_yy = true;
        }
        break;
    case REBACK_GOSTR_BYPASS_CHECK_RWE:
        if((Yaw>45)&&(Yaw<90)){
            right_reverse_walk_edge_status = TARGET_YAW_MORE_ABS75_MORE_ABS0_RWE;
            break;
        }
        if(!returnorigin){
            if ((my_abs(current_pose->x + half_map_wide - edge_length_start) > Edge_length()/2)&&(my_abs(current_pose->x + half_map_wide - edge_length_start) > 500)){
                right_reverse_walk_edge_status = DELTA_X_MORE_ONE_FOURTH_CLEANED_MAP_WIDTH_RWE;
                break;
            }
        }
        right_reverse_walk_edge_status = GOSTR_BYPASS_RWE_X;
        break;
    case COLLISION_BYPASS_RWE:
        if(turn_start_update == 0)
        {
            turn_start_x = current_pose->x;
            turn_start_y = current_pose->y;
            turn_start_update = 1;
			if (obstacleSignal == left_obstacle){
                Astarmarkingobstacle = left_obstacle;
            }
            else if (obstacleSignal == right_obstacle){
                Astarmarkingobstacle = right_obstacle;
            }
            else{
                Astarmarkingobstacle = front_obstacle;
            }
        }
        linear_velocity = -long_stra_vel;
        angular_velocity = 0;
        if (my_abs(turn_start_x - current_pose->x) > collision_backward_distance || my_abs(turn_start_y - current_pose->y) > collision_backward_distance)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            turn_start_update = 0;
            temporary_yaw = Yaw;			
			if(Astarmarkingobstacle==right_obstacle){
				right_reverse_walk_edge_status = TURN_CCLOCK_YAW_ADD_ABS15_RWE;
            }
            else{
                right_reverse_walk_edge_status = FRONT_COLLISION_TURN_CCLOCK_YAW_ADD_ABS15_RWE;
            }
            break;
        }
        break;
		
    case TURN_CCLOCK_YAW_ADD_ABS15_RWE:
        linear_velocity = 0;
        angular_velocity = turn_vel;
        if (my_abs(Yaw - temporary_yaw) > 15)
        {
            if(my_abs(Yaw)>175){
                right_reverse_walk_edge_status = TURN_CCLOCK_YAW_ADD_ABS15_RWE;
                temporary_yaw=Yaw;
                break;
            }
            linear_velocity = 0;
            angular_velocity = 0;
            right_reverse_walk_edge_status = REBACK_GOSTR_BYPASS_CHECK_RWE;
            break;
        }
        if (obstacleSignal !=none_obstacle||(&cliff_valueB)->cliffValue0 == 1)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            right_reverse_walk_edge_status = TURN_CCLOCK_YAW_ADD_ABS15_COLLISION_RWE;
            break;
        }
        break;
    case TURN_CCLOCK_YAW_ADD_ABS15_COLLISION_RWE:
        if(turn_start_update == 0)
        {
            turn_start_x = current_pose->x;
            turn_start_y = current_pose->y;
            turn_start_update = 1;
        }
        linear_velocity = -long_stra_vel;
        angular_velocity = 0;
        if (my_abs(turn_start_x - current_pose->x) > collision_backward_distance || my_abs(turn_start_y - current_pose->y) > collision_backward_distance)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            turn_start_update = 0;
            right_reverse_walk_edge_status = TURN_CCLOCK_YAW_ADD_ABS15_RWE;
            break;
        }
        break;
		
		
    case FRONT_COLLISION_TURN_CCLOCK_YAW_ADD_ABS15_RWE:
        linear_velocity = 0;
        angular_velocity = turn_vel;
        if (my_abs(Yaw - temporary_yaw) > 30)
        {
            if(my_abs(Yaw)>175){
                right_reverse_walk_edge_status = FRONT_COLLISION_TURN_CCLOCK_YAW_ADD_ABS15_RWE;
                temporary_yaw=Yaw;
                break;
            }
            linear_velocity = 0;
            angular_velocity = 0;
            right_reverse_walk_edge_status = REBACK_GOSTR_BYPASS_CHECK_RWE;
            break;
        }
        if (obstacleSignal !=none_obstacle||(&cliff_valueB)->cliffValue0 == 1)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            right_reverse_walk_edge_status = FRONT_COLLISION_TURN_CCLOCK_YAW_ADD_ABS15_COLLISION_RWE;
            break;
        }
        break;
    case FRONT_COLLISION_TURN_CCLOCK_YAW_ADD_ABS15_COLLISION_RWE:
        if(turn_start_update == 0)
        {
            turn_start_x = current_pose->x;
            turn_start_y = current_pose->y;
            turn_start_update = 1;
        }
        linear_velocity = -long_stra_vel;
        angular_velocity = 0;
        if (my_abs(turn_start_x - current_pose->x) > collision_backward_distance || my_abs(turn_start_y - current_pose->y) > collision_backward_distance)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            turn_start_update = 0;
            right_reverse_walk_edge_status = FRONT_COLLISION_TURN_CCLOCK_YAW_ADD_ABS15_RWE;
            break;
        }
        break;
		
		
    case TARGET_YAW_MORE_ABS75_MORE_ABS0_RWE:
        linear_velocity = 0;
        angular_velocity = 0;
        if(returnorigin==true){
            right_reverse_walk_edge_status =RETURN_ORIGIN_RWE;
            break;
        }
        if(!detection_close_edge){
            detection_close_edge=true;
            break;
        }else{
            detection_close_edge=false;
            if(detection_close==true){
                detection_close=false;
                right_reverse_walk_edge_status = RIGHT_EDGE_DILEMMA_RWE;
            }else{
                right_reverse_walk_edge_status =RETURN_ORIGIN_RWE;
            }
            break;
        }
        
    case DELTA_X_MORE_ONE_FOURTH_CLEANED_MAP_WIDTH_RWE:
        linear_velocity = 0;
        angular_velocity = 0;
        if(!detection_close_edge){
            detection_close_edge=true;
        }
        else{
            detection_close_edge=false;
            if(detection_close==true){
                detection_close=false;
                temporary_yaw = Yaw;
                right_reverse_walk_edge_status = CLOSE_EDGE_MAP_RIGHT_REVERSE_WALK_TURN_CCLOCK_YAW_ADD_ABS15_RWE;
            }else{
                returnorigin=true;
                right_reverse_walk_edge_status = GOSTR_BYPASS_RWE_X;
            }
        }
        break;
        
    case  CLOSE_EDGE_MAP_RIGHT_REVERSE_WALK_TURN_CCLOCK_YAW_ADD_ABS15_RWE:
        linear_velocity = 0;
        angular_velocity = turn_vel;
        if (my_abs(Yaw - temporary_yaw) > 15)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            last_position_x = current_pose->x;
            right_reverse_walk_edge_status = GOSTR_BYPASS_RWE;
            break;
        }
        if (obstacleSignal !=none_obstacle||(&cliff_valueB)->cliffValue0 == 1)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            right_reverse_walk_edge_status =COLLISION_CLOSE_EDGE_MAP_RIGHT_REVERSE_WALK_TURN_CCLOCK_YAW_ADD_ABS15_RWE;
            break;
        }
        break;
    case COLLISION_CLOSE_EDGE_MAP_RIGHT_REVERSE_WALK_TURN_CCLOCK_YAW_ADD_ABS15_RWE:
        if(turn_start_update == 0)
        {
            turn_start_x = current_pose->x;
            turn_start_y = current_pose->y;
            turn_start_update = 1;
        }
        linear_velocity = -long_stra_vel;
        angular_velocity = 0;
        if (my_abs(turn_start_x - current_pose->x) > collision_backward_distance || my_abs(turn_start_y - current_pose->y) > collision_backward_distance)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            turn_start_update = 0;
            right_reverse_walk_edge_status =CLOSE_EDGE_MAP_RIGHT_REVERSE_WALK_TURN_CCLOCK_YAW_ADD_ABS15_RWE ;
            break;
        }
        break;      
    case TURN_CCLOCK_YAW_MORE_178ABS_RWE:
        if (my_abs(Yaw) >175)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            right_reverse_walk_edge_status = RIGHT_EDGE_DILEMMA_RWE;
            break;
        }
        if (obstacleSignal !=none_obstacle||(&cliff_valueB)->cliffValue0 == 1)
        {
            right_reverse_walk_edge_status = TURN_CCLOCK_YAW_MORE_178ABS_COLLISION_RWE;
            break;
        }
        linear_velocity = 0;
        angular_velocity = turn_vel;
        break;
    case TURN_CCLOCK_YAW_MORE_178ABS_COLLISION_RWE:
        if(turn_start_update == 0)
        {
            turn_start_x = current_pose->x;
            turn_start_y = current_pose->y;
            turn_start_update = 1;
        }
        linear_velocity = -long_stra_vel;
        angular_velocity = 0;
        if (my_abs(turn_start_x - current_pose->x) > collision_backward_distance || my_abs(turn_start_y - current_pose->y) > collision_backward_distance)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            turn_start_update = 0;
            right_reverse_walk_edge_status = TURN_CCLOCK_YAW_MORE_178ABS_RWE;
            break;
        }
        break;
        
    case GOSTR_X_MORE_LATERALDIS_BYPASS_RWE:
        temporary_yaw = Yaw;
        if (my_abs(Yaw) > 45 && (Yaw > 0))
        {
            right_reverse_walk_edge_status = TURN_CLOCK_TARGET_YAW_LESS_0_RWE;
        }
        else if (my_abs(Yaw) <= 45 )
        {
            right_reverse_walk_edge_status = TURN_CLOCK_TARGET_YAW_LESS_ABS45_DETAL_YAW_MORE_30_RWE;
        }
        else{
            right_reverse_walk_edge_status = GOSTR_BYPASS_RWE_X;
        }
        break;
        
    case TURN_CLOCK_TARGET_YAW_LESS_0_RWE:
        linear_velocity = 0;
        angular_velocity = -turn_vel;
        if (Yaw < 0 )
        {
            linear_velocity = 0;
            angular_velocity = 0;
            right_reverse_walk_edge_status = GOSTR_BYPASS_RWE_X;
            break;
        }
        if (obstacleSignal !=none_obstacle||(&cliff_valueB)->cliffValue0 == 1)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            right_reverse_walk_edge_status = TURN_CLOCK_TARGET_YAW_LESS_0_COLLISION_RWE;
            break;
        }
        break;
    case TURN_CLOCK_TARGET_YAW_LESS_0_COLLISION_RWE:
        if(turn_start_update == 0)
        {
            turn_start_x = current_pose->x;
            turn_start_y = current_pose->y;
            turn_start_update = 1;
        }
        linear_velocity = -long_stra_vel;
        angular_velocity = 0;
        if (my_abs(turn_start_x - current_pose->x) > collision_backward_distance || my_abs(turn_start_y - current_pose->y) > collision_backward_distance)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            turn_start_update = 0;
            temporary_yaw = Yaw;
            right_reverse_walk_edge_status = TURN_CCLOCK_YAW_ADD_ABS15_RWE;
            break;
        }
        break;
    case TURN_CLOCK_TARGET_YAW_LESS_ABS45_DETAL_YAW_MORE_30_RWE:
        linear_velocity = 0;
        angular_velocity = -turn_vel;
        if (my_abs(temporary_yaw - Yaw) > 30 && my_abs(Yaw) > 45)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            right_reverse_walk_edge_status = GOSTR_BYPASS_RWE_X;
            break;
        }
        if (obstacleSignal !=none_obstacle||(&cliff_valueB)->cliffValue0 == 1)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            right_reverse_walk_edge_status = TURN_CLOCK_TARGET_YAW_LESS_ABS45_DETAL_YAW_MORE_30_COLLISION_RWE;
            break;
        }
        break;
    case TURN_CLOCK_TARGET_YAW_LESS_ABS45_DETAL_YAW_MORE_30_COLLISION_RWE:
        if(turn_start_update == 0)
        {
            turn_start_x = current_pose->x;
            turn_start_y = current_pose->y;
            turn_start_update = 1;
        }
        linear_velocity = -long_stra_vel;
        angular_velocity = 0;
        if (my_abs(turn_start_x - current_pose->x) > collision_backward_distance || my_abs(turn_start_y - current_pose->y) > collision_backward_distance)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            turn_start_update = 0;
            temporary_yaw = Yaw;
            right_reverse_walk_edge_status = TURN_CCLOCK_YAW_ADD_ABS15_RWE;
            break;
        }
        break;
    case BOW_CONTINUE_RWE:
        right_reverse_walk_edge_status = 0;
        returnorigin=false;
        complete_flag = 1;
        break;
    case RIGHT_EDGE_DILEMMA_RWE:
        linear_velocity = 0;
        angular_velocity = 0;
        if(my_abs(leakingsweep_x-current_pose->x)>500&&my_abs(leakingsweep_y-current_pose->y)>100){
			if((my_abs(current_pose->x)<half_map_wide-GRIDHEIGHT)&&(my_abs(current_pose->y)<half_map_wide-GRIDHEIGHT)){
            leakingsweep_x=current_pose->x;
            leakingsweep_y=current_pose->y;
            boolleaksweep=true;
            break;
			}
        }
        if (0 != leakingsweep)
        {
            right_reverse_walk_edge_status =0;
            returnorigin=false;
            right_running_step_status = LEAKING_SWEEP_RIGHTRUN_STEP;
            break;
        }
        else
        { 
            complete_flag = 2;
            right_reverse_walk_edge_status = 0;
            returnorigin=false;
        }
        break;
    case RETURN_ORIGIN_RWE:
        linear_velocity = 0;
        angular_velocity = 0;
        if(my_abs(leakingsweep_x-current_pose->x)>500&&my_abs(leakingsweep_y-current_pose->y)>100){
			if((my_abs(current_pose->x)<half_map_wide-GRIDHEIGHT)&&(my_abs(current_pose->y)<half_map_wide-GRIDHEIGHT)){
            leakingsweep_x=current_pose->x;
            leakingsweep_y=current_pose->y;
            boolleaksweep=true;
            break;
			}
        }
        if (0 != leakingsweep)
        {
            right_reverse_walk_edge_status =0;
            returnorigin=false;
            right_running_step_status = LEAKING_SWEEP_RIGHTRUN_STEP;
            break;
        }
        else
        { 
            right_reverse_walk_edge_status = 0;
            returnorigin=false;
            complete_flag = 3;
        }
        break;
    }
    return complete_flag;
}




unsigned char  ForwardBoundaryRightRunStep(POSE *current_pose, unsigned char obstacleSignal){
    int Yaw;
    unsigned char complete_flag = 0;
    Yaw = current_pose->orientation;
    Yaw = Yaw /100;
    if (judgment_Stuck_status == 0)
    {
        judgment_Stuck_status_x = current_pose->x;
        judgment_Stuck_status_y = current_pose->y;
        judgment_Stuck_status_yaw = Yaw;
        judgment_Stuck_status = 1;
    }
    if(!old_bow_continue){
        if(my_abs(current_pose->x-judgment_Stuck_status_x)>lateral_move_distance/2||my_abs(current_pose->y-judgment_Stuck_status_y)>lateral_move_distance/2){
            judgment_Stuck_status=0;
            judgment_Stuck_status_x = current_pose->x;
            judgment_Stuck_status_y = current_pose->y;
            if(my_abs(judgment_Stuck_status_yaw-Yaw)<5){
                if(my_abs(current_pose->x)>half_map_wide-2*GRIDWIDTH){
                    stuck_x=true;
                }
                else{
                    stuck_y=true;
                }
                old_bow_continue=false;
                complete_flag=2;
                return complete_flag;
            }
            else{
                if(my_abs(current_pose->y)>half_map_wide-2*GRIDWIDTH){
                    linear_velocity = 0;
                    angular_velocity = 0;
                    right_forward_boundary_status = 0;
                    judgment_Stuck_status=0;
                    complete_flag=3;
                    old_bow_continue=false;
                    return complete_flag;
                }
            }
        }
    }
    switch (right_forward_boundary_status)
    {
    case 0:
        if (my_abs(Yaw) < 90)
        {
            right_forward_boundary_status = FORWARDBOUNDARY_YAW_LESS_ABS10;
        }
        else
        {
            right_forward_boundary_status = FORWARDBOUNDARY_YAW_OTHER;
        }
        break;
    case FORWARDBOUNDARY_YAW_LESS_ABS10:
        right_forward_boundary_status = FORWARDBOUNDARY_CLOCK_TARGET_YAW_MORE_ABS178;
        break;
    case FORWARDBOUNDARY_CLOCK_TARGET_YAW_MORE_ABS178:
        linear_velocity = 100;
        angular_velocity = -57;
        if (my_abs(Yaw) > 175)
        {
            right_forward_boundary_status = FORWARDBOUNDARY_GOSTRAIGHT;
            old_bow_continue=true;
            break;
        }
        if (obstacleSignal != none_obstacle)
        {
            right_forward_boundary_status = FORWARDBOUNDARY_CLOCK_TARGET_YAW_MORE_ABS178_COLLISION;
            break;
        }
        break;
    case FORWARDBOUNDARY_CLOCK_TARGET_YAW_MORE_ABS178_COLLISION:
        if (turn_start_update == 0)
        {
            turn_start_x = current_pose->x;
            turn_start_y = current_pose->y;
            turn_start_update = 1;
        }
        linear_velocity = -long_stra_vel;
        angular_velocity = 0;
        if (my_abs(turn_start_x - current_pose->x) > collision_backward_distance || my_abs(turn_start_y - current_pose->y) > collision_backward_distance)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            turn_start_update = 0;
            right_forward_boundary_status = FORWARDBOUNDARY_CLOCK_TARGET_YAW_MORE_ABS178;
            break;
        }
        break;
    case FORWARDBOUNDARY_YAW_OTHER:
        right_forward_boundary_status = FORWARDBOUNDARY_CCLOCK_TARGET_YAW_LESS_ABS3;
        break;
    case FORWARDBOUNDARY_CCLOCK_TARGET_YAW_LESS_ABS3:
        linear_velocity = 100;
        angular_velocity = 57;
        if (my_abs(Yaw) < 5)
        {
            right_forward_boundary_status = FORWARDBOUNDARY_GOSTRAIGHT;
            old_bow_continue=true;
            break;
        }
        if (obstacleSignal != none_obstacle||(&cliff_valueB)->cliffValue0 == 1)
        {
            right_forward_boundary_status = FORWARDBOUNDARY_CCLOCK_TARGET_YAW_LESS_ABS3_COLLISION;
            break;
        }
        break;
    case FORWARDBOUNDARY_CCLOCK_TARGET_YAW_LESS_ABS3_COLLISION:
        if (turn_start_update == 0)
        {
            turn_start_x = current_pose->x;
            turn_start_y = current_pose->y;
            turn_start_update = 1;
        }
        linear_velocity = -long_stra_vel;
        angular_velocity = 0;
        if (my_abs(turn_start_x - current_pose->x) > collision_backward_distance || my_abs(turn_start_y - current_pose->y) > collision_backward_distance)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            turn_start_update = 0;
            right_forward_boundary_status = FORWARDBOUNDARY_CCLOCK_TARGET_YAW_LESS_ABS3;
            break;
        }
        break;
    case FORWARDBOUNDARY_GOSTRAIGHT:
        if (turn_start_update == 0)
        {
            turn_start_x = current_pose->x;
            turn_start_y = current_pose->y;
            turn_start_update = 1;
        }
        if ((my_abs(turn_start_x - current_pose->x) > 2*GRIDWIDTH || my_abs(turn_start_y - current_pose->y) > 2*GRIDWIDTH) && my_abs(Yaw) < 90 && current_pose->x < 0)
        {
            turn_start_update = 0;
            right_forward_boundary_status = FORWARDBOUNDARY_COMPLETE;
            break;
        }
        if ((my_abs(turn_start_x - current_pose->x) > 2*GRIDWIDTH || my_abs(turn_start_y - current_pose->y) > 2*GRIDWIDTH)&& my_abs(Yaw) > 90 && current_pose->x > 0)
        {
            turn_start_update = 0;
            right_forward_boundary_status = FORWARDBOUNDARY_COMPLETE;
            break;
        }
        linear_velocity = long_stra_vel;
        angular_velocity = 0;
        if (obstacleSignal != none_obstacle||(&cliff_valueB)->cliffValue0 == 1)
        {
            right_forward_boundary_status = FORWARDBOUNDARY_COMPLETE;
            break;
        }
        if (my_abs(current_pose->x) < half_map_wide-2*GRIDWIDTH)
        {
            right_forward_boundary_status = FORWARDBOUNDARY_COMPLETE;
            break;
        }
        break;
    case FORWARDBOUNDARY_COMPLETE:
        old_bow_continue=false;
        right_forward_boundary_status = 0;
        judgment_Stuck_status = 0;
        complete_flag = 1;
        break;
    }
    return complete_flag;
}



unsigned char  StuckRightRunStep(POSE *current_pose, unsigned char obstacleSignal){
    int Yaw;
    unsigned char complete_flag = 0;
    Yaw = current_pose->orientation;
    Yaw = Yaw /100;
    switch (stuck_right_run_step)
    {
    case 0:
        stuck_right_run_step = STUCK_FORWARD_BOUNDARY_STATUS;
        break;
    case STUCK_FORWARD_BOUNDARY_STATUS:
        if (turn_start_update == 0)
        {
            turn_start_x = current_pose->x;
            turn_start_y = current_pose->y;
            turn_start_update = 1;
        }
        linear_velocity = -long_stra_vel;
        angular_velocity = 0;
        if (my_abs(turn_start_x - current_pose->x) > lateral_move_distance || my_abs(turn_start_y - current_pose->y) > lateral_move_distance)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            turn_start_update = 0;
            temporary_yaw = Yaw;
            stuck_right_run_step = TURN_RIGHT_YAW_MORE_90_STUCK_FORWARD_BOUNDARY_STATUS;
            break;
        }
        break;
    case TURN_RIGHT_YAW_MORE_90_STUCK_FORWARD_BOUNDARY_STATUS:
        if (my_abs(Yaw - temporary_yaw) > 90)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            last_position_x = current_pose->x;
            last_position_y = current_pose->y;
            stuck_right_run_step = GO_STUCK_FORWARD_BOUNDARY_STATUS;
            break;
        }
        if (obstacleSignal != none_obstacle||(&cliff_valueB)->cliffValue0 == 1)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            stuck_right_run_step = COLLISION_TURN_RIGHT_YAW_MORE_90_STUCK_FORWARD_BOUNDARY_STATUS;
        }
        linear_velocity = 0;
        angular_velocity = -turn_vel;
        break;
    case COLLISION_TURN_RIGHT_YAW_MORE_90_STUCK_FORWARD_BOUNDARY_STATUS:
        if (turn_start_update == 0)
        {
            turn_start_x = current_pose->x;
            turn_start_y = current_pose->y;
            turn_start_update = 1;
        }
        linear_velocity = -long_stra_vel;
        angular_velocity = 0;
        if (my_abs(turn_start_x - current_pose->x) > collision_backward_distance || my_abs(turn_start_y - current_pose->y) > collision_backward_distance)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            turn_start_update = 0;
            stuck_right_run_step = TURN_RIGHT_YAW_MORE_90_STUCK_FORWARD_BOUNDARY_STATUS;
            break;
        }
        break;
    case GO_STUCK_FORWARD_BOUNDARY_STATUS:
        linear_velocity = long_stra_vel;
        angular_velocity = 0;
        if (obstacleSignal != none_obstacle||(&cliff_valueB)->cliffValue0 == 1)
        {
            stuck_right_run_step = COLLISION_GO_STUCK_FORWARD_BOUNDARY_STATUS;
            break;
        }
        if (my_abs(last_position_x - current_pose->x) > lateral_move_distance || my_abs(last_position_y - current_pose->y) > lateral_move_distance)
        {
            temporary_yaw = Yaw;
            linear_velocity = 0;
            angular_velocity = 0;
            stuck_right_run_step = GO_X_Y_MORE_STUCK_FORWARD_BOUNDARY_STATUS;
            break;
        }
        break;
    case COLLISION_GO_STUCK_FORWARD_BOUNDARY_STATUS:
        if (turn_start_update == 0)
        {
            turn_start_x = current_pose->x;
            turn_start_y = current_pose->y;
            turn_start_update = 1;
        }
        linear_velocity = -long_stra_vel;
        angular_velocity = 0;
        if (my_abs(turn_start_x - current_pose->x) > collision_backward_distance || my_abs(turn_start_y - current_pose->y) > collision_backward_distance)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            turn_start_update = 0;
            temporary_yaw = Yaw;
            stuck_right_run_step = TURN_RIGHT_YAW_MORE_10_STUCK_FORWARD_BOUNDARY_STATUS;
            break;
        }
        break;
    case TURN_RIGHT_YAW_MORE_10_STUCK_FORWARD_BOUNDARY_STATUS:
        if (my_abs(Yaw - temporary_yaw) > 10)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            stuck_right_run_step = GO_STUCK_FORWARD_BOUNDARY_STATUS;
            break;
        }
        if (obstacleSignal != none_obstacle||(&cliff_valueB)->cliffValue0 == 1)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            stuck_right_run_step = COLLISION_TURN_RIGHT_YAW_MORE_10_STUCK_FORWARD_BOUNDARY_STATUS;
        }
        linear_velocity = 0;
        angular_velocity = -turn_vel;
        break;
    case COLLISION_TURN_RIGHT_YAW_MORE_10_STUCK_FORWARD_BOUNDARY_STATUS:
        if (turn_start_update == 0)
        {
            turn_start_x = current_pose->x;
            turn_start_y = current_pose->y;
            turn_start_update = 1;
        }
        linear_velocity = -long_stra_vel;
        angular_velocity = 0;
        if (my_abs(turn_start_x - current_pose->x) > collision_backward_distance || my_abs(turn_start_y - current_pose->y) > collision_backward_distance)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            turn_start_update = 0;
            stuck_right_run_step = TURN_RIGHT_YAW_MORE_10_STUCK_FORWARD_BOUNDARY_STATUS;
            break;
        }
        break;
    case GO_X_Y_MORE_STUCK_FORWARD_BOUNDARY_STATUS:
        if (my_abs(Yaw - temporary_yaw) > 90)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            last_position_x = current_pose->x;
            last_position_y = current_pose->y;
            stuck_right_run_step = ESCAPE_FROM_STUCK_FORWARD_BOUNDARY_STATUS;
            break;
        }
        if (obstacleSignal != none_obstacle||(&cliff_valueB)->cliffValue0 == 1)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            temporary_yaw = Yaw;
            stuck_right_run_step = COLLISION_GO_X_Y_MORE_STUCK_FORWARD_BOUNDARY_STATUS;
        }
        linear_velocity = 0;
        angular_velocity = -turn_vel;
        break;
    case COLLISION_GO_X_Y_MORE_STUCK_FORWARD_BOUNDARY_STATUS:
        if (turn_start_update == 0)
        {
            turn_start_x = current_pose->x;
            turn_start_y = current_pose->y;
            turn_start_update = 1;
        }
        linear_velocity = -long_stra_vel;
        angular_velocity = 0;
        if (my_abs(turn_start_x - current_pose->x) > collision_backward_distance || my_abs(turn_start_y - current_pose->y) > collision_backward_distance)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            turn_start_update = 0;
            stuck_right_run_step = GO_X_Y_MORE_STUCK_FORWARD_BOUNDARY_STATUS;
            break;
        }
        break;
    case ESCAPE_FROM_STUCK_FORWARD_BOUNDARY_STATUS:
        linear_velocity = long_stra_vel;
        angular_velocity = 0;
        if (obstacleSignal != none_obstacle||(&cliff_valueB)->cliffValue0 == 1)
        {
            stuck_right_run_step = COLLISION_ESCAPE_FROM_STUCK_FORWARD_BOUNDARY_STATUS;
            break;
        }
        if (my_abs(last_position_x - current_pose->x) > lateral_move_distance || my_abs(last_position_y - current_pose->y) > lateral_move_distance)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            stuck_right_run_step = COMPETE_ESCAPE_FROM_STUCK_FORWARD_BOUNDARY_STATUS;
            break;
        }
        break;
    case COLLISION_ESCAPE_FROM_STUCK_FORWARD_BOUNDARY_STATUS:
        if (turn_start_update == 0)
        {
            turn_start_x = current_pose->x;
            turn_start_y = current_pose->y;
            turn_start_update = 1;
        }
        linear_velocity = -long_stra_vel;
        angular_velocity = 0;
        if (my_abs(turn_start_x - current_pose->x) > collision_backward_distance || my_abs(turn_start_y - current_pose->y) > collision_backward_distance)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            turn_start_update = 0;
            temporary_yaw = Yaw;
            stuck_right_run_step = TURN_COLLISION_ESCAPE_FROM_STUCK_FORWARD_BOUNDARY_STATUS;
            break;
        }
        break;
    case TURN_COLLISION_ESCAPE_FROM_STUCK_FORWARD_BOUNDARY_STATUS:
        if (my_abs(Yaw - temporary_yaw) > 10)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            stuck_right_run_step = ESCAPE_FROM_STUCK_FORWARD_BOUNDARY_STATUS;
            break;
        }
        if (obstacleSignal != none_obstacle||(&cliff_valueB)->cliffValue0 == 1)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            stuck_right_run_step = COLLISION_TURN_COLLISION_ESCAPE_FROM_STUCK_FORWARD_BOUNDARY_STATUS;
        }
        linear_velocity = 0;
        angular_velocity = turn_vel;
        break;
    case COLLISION_TURN_COLLISION_ESCAPE_FROM_STUCK_FORWARD_BOUNDARY_STATUS:
        if (turn_start_update == 0)
        {
            turn_start_x = current_pose->x;
            turn_start_y = current_pose->y;
            turn_start_update = 1;
        }
        linear_velocity = -long_stra_vel;
        angular_velocity = 0;
        if (my_abs(turn_start_x - current_pose->x) > collision_backward_distance || my_abs(turn_start_y - current_pose->y) > collision_backward_distance)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            turn_start_update = 0;
            stuck_right_run_step = TURN_COLLISION_ESCAPE_FROM_STUCK_FORWARD_BOUNDARY_STATUS;
            break;
        }
        break;
    case COMPETE_ESCAPE_FROM_STUCK_FORWARD_BOUNDARY_STATUS:
        linear_velocity=0;
        angular_velocity=0;
        if(stuck==false){
            stuck=true;
        }
        else{
            stuck_right_run_step = 0;
            complete_flag = 1;
        }
        break;
    }
    return complete_flag;
}



unsigned char  RightReadyLeakingSweep(POSE *current_pose, unsigned char obstacleSignal){
    int Yaw;
    unsigned char complete_flag = 0;
    unsigned char i=0;
    Yaw = current_pose->orientation;
    Yaw = Yaw /100;
    switch(right_ready_leaking_sweep_status)
    {
    case 0:
        right_ready_leaking_sweep_status = RIGHT_LEAKING_SWEEP_COLLISION;
        break;
    case RIGHT_LEAKING_SWEEP_COLLISION:
        if(turn_start_update == 0)
        {
            turn_start_x = current_pose->x;
            turn_start_y = current_pose->y;
            turn_start_update = 1;
        }
        linear_velocity = -long_stra_vel;
        angular_velocity = 0;
        if (my_abs(turn_start_x - current_pose->x) > collision_backward_distance || my_abs(turn_start_y - current_pose->y) > collision_backward_distance)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            turn_start_update = 0;
            
            if(my_abs(Yaw)>90)
            {
                right_ready_leaking_sweep_status = RIGHT_LEAKING_SWEEP_YAW_MORE_ABS90;
                break;
            }
            else
            {
                right_ready_leaking_sweep_status = RIGHT_LEAKING_SWEEP_YAW_OTHER;
                break;
            }
        }
        break;
    case RIGHT_LEAKING_SWEEP_YAW_MORE_ABS90:
        right_ready_leaking_sweep_status = RIGHT_LEAKING_SWEEP_CLOCK_TARGET_YAW_LESS_ABS90;
        break;
    case RIGHT_LEAKING_SWEEP_CLOCK_TARGET_YAW_LESS_ABS90:
        if(my_abs(Yaw)<95){
            linear_velocity=0;
            angular_velocity=0;
            last_position_y=current_pose->y;
            right_ready_leaking_sweep_status = RIGHT_LEAKING_SWEEP_GOSTRAIGHT_MORE;
            break;
        }
        if (obstacleSignal!=none_obstacle||(&cliff_valueB)->cliffValue0 == 1)
        {
            linear_velocity=0;
            angular_velocity=0;
            right_ready_leaking_sweep_status = RIGHT_LEAKING_SWEEP_CLOCK_TARGER_YAW_LESS_ABS90_COLLISION;
        }
        linear_velocity=0;
        angular_velocity=-turn_vel;
        break;
    case RIGHT_LEAKING_SWEEP_CLOCK_TARGER_YAW_LESS_ABS90_COLLISION  :
        if(turn_start_update == 0)
        {
            turn_start_x = current_pose->x;
            turn_start_y = current_pose->y;
            turn_start_update = 1;
        }
        linear_velocity = -long_stra_vel;
        angular_velocity = 0;
        if (my_abs(turn_start_x - current_pose->x) > collision_backward_distance || my_abs(turn_start_y - current_pose->y) > collision_backward_distance)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            turn_start_update = 0;
            right_ready_leaking_sweep_status = RIGHT_LEAKING_SWEEP_CLOCK_TARGET_YAW_LESS_ABS90;
            break;
        }
        break;
    case RIGHT_LEAKING_SWEEP_GOSTRAIGHT_MORE:
        linear_velocity=long_stra_vel;
        angular_velocity=0;
        if(obstacleSignal!=none_obstacle||(&cliff_valueB)->cliffValue0 == 1){
            last_position_xx=current_pose->x;
            right_ready_leaking_sweep_status = RIGHT_LEAKING_SWEEP_START_WALK_EDGE;
            break;
        }
        if(my_abs(last_position_y-current_pose->y)>leakingsweep){
            right_ready_leaking_sweep_status = RIGHT_LEAKING_SWEEP_CLOCK_TARGER_YAW_LESS_ABS3;
            break;
        }
        break;
        
    case RIGHT_LEAKING_SWEEP_START_WALK_EDGE:
        linear_velocity = 0;
        angular_velocity = 0;
        last_position_x = current_pose->x;
        last_position_yy = current_pose->y;
        right_ready_leaking_sweep_status = RIGHT_LEAKING_SWEEP_START_WALK_EDGE_LOOP;
        break;
        
    case RIGHT_LEAKING_SWEEP_START_WALK_EDGE_LOOP:
        if (obstacleSignal!=none_obstacle||(&cliff_valueB)->cliffValue0 == 1)
        {
            right_ready_leaking_sweep_status = RIGHT_LEAKING_SWEEP_START_WALK_EDGE_LOOP_COLLISION;
            break;
        }
        if(Yaw>30&&Yaw<150){
			left_edge_judgment_repeat();
//            if(adcRealTime[8]>100&&adcRealTime[8]<1500){
//                delimma_edge=0;
//                linear_velocity = 200;
//                angular_velocity = 0;
//            }
//            if(adcRealTime[8]>=1500){
//                delimma_edge=0;
//                linear_velocity = 200;
//                angular_velocity = -10;
//            }
//            if(adcRealTime[8]<100){
//                if(delimma_edge<10){
//                    delimma_edge++;
//                    linear_velocity = 200;
//                    angular_velocity = 10;
//                }
//                else{
//                    linear_velocity = 100;
//                    angular_velocity = 20;
//                }
//                
//            }
        }else{
            if (my_abs(last_position_x - current_pose->x) > lateral_move_distance||my_abs(last_position_yy - current_pose->y) > lateral_move_distance)
            {
                if(Yaw>=150){
                    last_position_x=current_pose->x;
                    last_position_yy=current_pose->y;
                    right_ready_leaking_sweep_status =RIGHT_LEAKING_SWEEP_START_WALK_EDGE_LOOP;
                }else{
                    temporary_yaw = Yaw;
                    right_ready_leaking_sweep_status =RIGHT_LEAKING_SWEEP_START_WALK_EDGE_LOOP_X_TURN;
                }
                break;
            }
            linear_velocity = long_stra_vel;
            angular_velocity = 0;
        }
        
        if(Yaw<-75||current_pose->y<last_position_y||my_abs(last_position_y-current_pose->y)>leakingsweep){
            right_ready_leaking_sweep_status = RIGHT_LEAKING_SWEEP_CLOCK_TARGER_YAW_LESS_ABS3;
            break;
        }
        if(my_abs(last_position_xx - current_pose->x)>1000){
            cnt_update=(current_pose->y%half_map_wide+half_map_wide)/100;
            for(i=MAPWIDECELLS-1;i>0;i--){
                if(gridmap.map[i][cnt_update]!=125){
                    break;
                }
            }
            if((current_pose->x+half_map_wide)/100+5>=i){
                right_ready_leaking_sweep_status = RIGHT_LEAKING_SWEEP_CLOCK_TARGER_YAW_LESS_ABS3;
            }
        }
        
        break;
        
    case RIGHT_LEAKING_SWEEP_START_WALK_EDGE_LOOP_COLLISION:
        if(turn_start_update == 0)
        {
            turn_start_x = current_pose->x;
            turn_start_y = current_pose->y;
            turn_start_update = 1;
        }
        linear_velocity = -long_stra_vel;
        angular_velocity = 0;
        if (my_abs(turn_start_x - current_pose->x) > collision_backward_distance || my_abs(turn_start_y - current_pose->y) > collision_backward_distance)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            turn_start_update = 0;
            temporary_yaw = Yaw;
            right_ready_leaking_sweep_status = RIGHT_LEAKING_SWEEP_START_WALK_EDGE_LOOP_COLLISION_TURN;
            break;
        }
        break;
    case RIGHT_LEAKING_SWEEP_START_WALK_EDGE_LOOP_COLLISION_TURN:
        linear_velocity = 0;
        angular_velocity = -turn_vel;
        if (my_abs(Yaw - temporary_yaw) > 15)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            last_position_x=current_pose->x;
            last_position_yy=current_pose->y;
            right_ready_leaking_sweep_status =RIGHT_LEAKING_SWEEP_START_WALK_EDGE_LOOP;
            break;
        }
        if (obstacleSignal!=none_obstacle||(&cliff_valueB)->cliffValue0 == 1)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            right_ready_leaking_sweep_status = RIGHT_LEAKING_SWEEP_START_WALK_EDGE_LOOP_COLLISION_TURN_COLLISION;
            break;
        }
        break;
    case RIGHT_LEAKING_SWEEP_START_WALK_EDGE_LOOP_COLLISION_TURN_COLLISION:
        if(turn_start_update == 0)
        {
            turn_start_x = current_pose->x;
            turn_start_y = current_pose->y;
            turn_start_update = 1;
        }
        linear_velocity = -long_stra_vel;
        angular_velocity = 0;
        if (my_abs(turn_start_x - current_pose->x) > collision_backward_distance || my_abs(turn_start_y - current_pose->y) > collision_backward_distance)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            turn_start_update = 0;
            right_ready_leaking_sweep_status = RIGHT_LEAKING_SWEEP_START_WALK_EDGE_LOOP_COLLISION_TURN;
            break;
        }
        break;
    case RIGHT_LEAKING_SWEEP_START_WALK_EDGE_LOOP_X_TURN:
        linear_velocity = 0;
        angular_velocity =turn_vel;
        if (my_abs(temporary_yaw - Yaw) > 30||(Yaw>30))
        {
            linear_velocity = 0;
            angular_velocity = 0;
            right_ready_leaking_sweep_status =RIGHT_LEAKING_SWEEP_START_WALK_EDGE_LOOP;
            break;
        }
        if (obstacleSignal!=none_obstacle||(&cliff_valueB)->cliffValue0 == 1)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            right_ready_leaking_sweep_status = RIGHT_LEAKING_SWEEP_START_WALK_EDGE_LOOP_COLLISION;
            break;
        }
        break;
    case RIGHT_LEAKING_SWEEP_CLOCK_TARGER_YAW_LESS_ABS3:
        if(my_abs(Yaw)>172){
            right_ready_leaking_sweep_status = RIGHT_LEAKING_SWEEP_COMPLETE;
            break;
        }
        if (obstacleSignal!=none_obstacle||(&cliff_valueB)->cliffValue0 == 1)
        {
            linear_velocity=0;
            angular_velocity=0;
            right_ready_leaking_sweep_status = RIGHT_LEAKING_SWEEP_CLOCK_TARGER_YAW_LESS_ABS3_COLLISION;
        }
        if(Yaw>0){
            linear_velocity=0;
            angular_velocity=turn_vel;
        }
        else{
            linear_velocity=0;
            angular_velocity=-turn_vel;
        }
        break;
    case RIGHT_LEAKING_SWEEP_CLOCK_TARGER_YAW_LESS_ABS3_COLLISION:
        if(turn_start_update == 0)
        {
            turn_start_x = current_pose->x;
            turn_start_y = current_pose->y;
            turn_start_update = 1;
        }
        linear_velocity = -long_stra_vel;
        angular_velocity = 0;
        if (my_abs(turn_start_x - current_pose->x) > collision_backward_distance || my_abs(turn_start_y - current_pose->y) > collision_backward_distance)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            turn_start_update = 0;
            right_ready_leaking_sweep_status = RIGHT_LEAKING_SWEEP_CLOCK_TARGER_YAW_LESS_ABS3;
            break;
        }
        break;
    case RIGHT_LEAKING_SWEEP_YAW_OTHER:
        right_ready_leaking_sweep_status = RIGHT_LEAKING_SWEEP_CCLOCK_TARGET_YAW_MORE_ABS90;
        break;
    case RIGHT_LEAKING_SWEEP_CCLOCK_TARGET_YAW_MORE_ABS90:
        if(my_abs(Yaw)>85){
            linear_velocity=0;
            angular_velocity=0;
            last_position_y=current_pose->y;
            right_ready_leaking_sweep_status = RIGHT_LEAKING_SWEEP_GOSTRAIGHT_OTHER;
            break;
        }
        if (obstacleSignal!=none_obstacle||(&cliff_valueB)->cliffValue0 == 1)
        {
            linear_velocity=0;
            angular_velocity=0;
            right_ready_leaking_sweep_status = RIGHT_LEAKING_SWEEP_CCLOCK_TARGET_YAW_MORE_ABS90_COLLISION;
        }
        linear_velocity=0;
        angular_velocity=turn_vel;
        break;
    case RIGHT_LEAKING_SWEEP_CCLOCK_TARGET_YAW_MORE_ABS90_COLLISION :
        if(turn_start_update == 0)
        {
            turn_start_x = current_pose->x;
            turn_start_y = current_pose->y;
            turn_start_update = 1;
        }
        linear_velocity = -long_stra_vel;
        angular_velocity = 0;
        if (my_abs(turn_start_x - current_pose->x) > collision_backward_distance || my_abs(turn_start_y - current_pose->y) > collision_backward_distance)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            turn_start_update = 0;
            right_ready_leaking_sweep_status = RIGHT_LEAKING_SWEEP_CCLOCK_TARGET_YAW_MORE_ABS90;
            break;
        }
        break;
    case RIGHT_LEAKING_SWEEP_GOSTRAIGHT_OTHER:
        linear_velocity=long_stra_vel;
        angular_velocity=0;
        if(obstacleSignal!=none_obstacle||(&cliff_valueB)->cliffValue0 == 1){
            last_position_xx = current_pose->x;
            right_ready_leaking_sweep_status = RIGHT_LEAKING_SWEEP_OTHER_START_WALK_EDGE;
            break;
        }
        if(my_abs(last_position_y-current_pose->y)>leakingsweep){
            right_ready_leaking_sweep_status = RIGHT_LEAKING_SWEEP_CCLOCK_TARGET_YAW_MORE_ABS178;
            break;
        }
        break;
        
    case RIGHT_LEAKING_SWEEP_OTHER_START_WALK_EDGE:
        linear_velocity = 0;
        angular_velocity = 0;
        last_position_x = current_pose->x;
        last_position_yy = current_pose->y;
        right_ready_leaking_sweep_status = RIGHT_LEAKING_SWEEP_OTHER_START_WALK_EDGE_LOOP;
        break;
        
    case RIGHT_LEAKING_SWEEP_OTHER_START_WALK_EDGE_LOOP:
        if (obstacleSignal!=none_obstacle||(&cliff_valueB)->cliffValue0 == 1)
        {
            right_ready_leaking_sweep_status = RIGHT_LEAKING_SWEEP_OTHER_START_WALK_EDGE_LOOP_COLLISION;
            break;
        }
        
        if(Yaw<150&&Yaw>30){
			right_edge_judgment_repeat();
//            if(adcRealTime[9]>100&&adcRealTime[9]<1500){
//                delimma_edge=0;
//                linear_velocity = 200;
//                angular_velocity = 0;
//            }
//            if(adcRealTime[9]>=1500){
//                delimma_edge=0;
//                linear_velocity = 200;
//                angular_velocity = 10;
//            }
//            if(adcRealTime[9]<100){
//                if(delimma_edge<10){
//                    delimma_edge++;
//                    linear_velocity = 200;
//                    angular_velocity = -10;
//                }
//                else{
//                    linear_velocity = 100;
//                    angular_velocity = -20;
//                }
//            }
        }else{
            if (my_abs(last_position_x - current_pose->x) > lateral_move_distance||my_abs(last_position_yy - current_pose->y) > lateral_move_distance)
            {
                if(Yaw>150){
                    last_position_x=current_pose->x;
                    last_position_yy=current_pose->y;
                    right_ready_leaking_sweep_status =RIGHT_LEAKING_SWEEP_OTHER_START_WALK_EDGE_LOOP;
                }else{
                    temporary_yaw = Yaw;
                    right_ready_leaking_sweep_status =RIGHT_LEAKING_SWEEP_OTHER_START_WALK_EDGE_LOOP_X_TURN;
                }
                break;
            }
            linear_velocity = long_stra_vel;
            angular_velocity = 0;
        }
        if((Yaw>-105&&Yaw<0)||current_pose->y<last_position_y||my_abs(last_position_y-current_pose->y)>leakingsweep){
            right_ready_leaking_sweep_status = RIGHT_LEAKING_SWEEP_CCLOCK_TARGET_YAW_MORE_ABS178;
            break;
        }
        if(my_abs(last_position_xx - current_pose->x)>1000){
            cnt_update=(current_pose->y%half_map_wide+half_map_wide)/100;
            for(i=0;i<MAPWIDECELLS;i++){
                if(gridmap.map[i][cnt_update]!=125){
                    break;
                }
            }
            if((current_pose->x+half_map_wide)/100-5<=i){
                right_ready_leaking_sweep_status = RIGHT_LEAKING_SWEEP_CCLOCK_TARGET_YAW_MORE_ABS178;
                break;
            }
        }
        break;
    case RIGHT_LEAKING_SWEEP_OTHER_START_WALK_EDGE_LOOP_COLLISION:
        if(turn_start_update == 0)
        {
            turn_start_x = current_pose->x;
            turn_start_y = current_pose->y;
            turn_start_update = 1;
        }
        linear_velocity = -long_stra_vel;
        angular_velocity = 0;
        if (my_abs(turn_start_x - current_pose->x) > collision_backward_distance || my_abs(turn_start_y - current_pose->y) > collision_backward_distance)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            turn_start_update = 0;
            temporary_yaw = Yaw;
            right_ready_leaking_sweep_status = RIGHT_LEAKING_SWEEP_OTHER_START_WALK_EDGE_LOOP_COLLISION_TURN;
            break;
        }
        break;
    case RIGHT_LEAKING_SWEEP_OTHER_START_WALK_EDGE_LOOP_COLLISION_TURN:
        linear_velocity = 0;
        angular_velocity = turn_vel;
        if (my_abs(Yaw - temporary_yaw) > 15)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            last_position_x=current_pose->x;
            last_position_yy=current_pose->y;
            right_ready_leaking_sweep_status =RIGHT_LEAKING_SWEEP_OTHER_START_WALK_EDGE_LOOP;
            break;
        }
        if (obstacleSignal!=none_obstacle||(&cliff_valueB)->cliffValue0 == 1)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            right_ready_leaking_sweep_status = RIGHT_LEAKING_SWEEP_OTHER_START_WALK_EDGE_LOOP_COLLISION_TURN_COLLISION;
            break;
        }
        break;
    case RIGHT_LEAKING_SWEEP_OTHER_START_WALK_EDGE_LOOP_COLLISION_TURN_COLLISION:
        if(turn_start_update == 0)
        {
            turn_start_x = current_pose->x;
            turn_start_y = current_pose->y;
            turn_start_update = 1;
        }
        linear_velocity = -long_stra_vel;
        angular_velocity = 0;
        if (my_abs(turn_start_x - current_pose->x) > collision_backward_distance || my_abs(turn_start_y - current_pose->y) > collision_backward_distance)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            turn_start_update = 0;
            right_ready_leaking_sweep_status = RIGHT_LEAKING_SWEEP_OTHER_START_WALK_EDGE_LOOP_COLLISION_TURN;
            break;
        }
        break;
    case RIGHT_LEAKING_SWEEP_OTHER_START_WALK_EDGE_LOOP_X_TURN:
        linear_velocity = 0;
        angular_velocity =-turn_vel;
        if (my_abs(temporary_yaw - Yaw) > 30||(Yaw>30))
        {
            linear_velocity = 0;
            angular_velocity = 0;
            last_position_x=current_pose->x;
            last_position_yy=current_pose->y;
            right_ready_leaking_sweep_status =RIGHT_LEAKING_SWEEP_OTHER_START_WALK_EDGE_LOOP;
            break;
        }
        if (obstacleSignal!=none_obstacle||(&cliff_valueB)->cliffValue0 == 1)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            right_ready_leaking_sweep_status = RIGHT_LEAKING_SWEEP_OTHER_START_WALK_EDGE_LOOP_COLLISION;
            break;
        }
        break;
        
    case RIGHT_LEAKING_SWEEP_CCLOCK_TARGET_YAW_MORE_ABS178:
        if(my_abs(Yaw)<8){
            right_ready_leaking_sweep_status = RIGHT_LEAKING_SWEEP_COMPLETE;
            break;
        }
        if (obstacleSignal!=none_obstacle||(&cliff_valueB)->cliffValue0 == 1)
        {
            linear_velocity=0;
            angular_velocity=0;
            right_ready_leaking_sweep_status = RIGHT_LEAKING_SWEEP_CCLOCK_TARGET_YAW_MOEE_ABS178_COLLISIION;
        }
        if(Yaw>0){
            linear_velocity=0;
            angular_velocity=-turn_vel;
        }
        else{
            linear_velocity=0;
            angular_velocity=turn_vel;
        }
        break;
    case RIGHT_LEAKING_SWEEP_CCLOCK_TARGET_YAW_MOEE_ABS178_COLLISIION:
        if(turn_start_update == 0)
        {
            turn_start_x = current_pose->x;
            turn_start_y = current_pose->y;
            turn_start_update = 1;
        }
        linear_velocity = -long_stra_vel;
        angular_velocity = 0;
        if (my_abs(turn_start_x - current_pose->x) > collision_backward_distance || my_abs(turn_start_y - current_pose->y) > collision_backward_distance)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            turn_start_update = 0;
            right_ready_leaking_sweep_status = RIGHT_LEAKING_SWEEP_CCLOCK_TARGET_YAW_MORE_ABS178;
            break;
        }
        break;
    case RIGHT_LEAKING_SWEEP_COMPLETE:
        leakingsweep=0;
        right_ready_leaking_sweep_status = 0;
        complete_flag = 1;
        break;
    }
    return complete_flag;
}




//##########           LEFT             ###########################################	
///////////////////////////////////////////////////////////////////////////////////
unsigned char  LeftRunningWorkStep(POSE *current_pose, unsigned char obstacleSignal){
    int Yaw;
    unsigned char complete_flag = 0;
	unsigned char i=0,j=0;
    Yaw = current_pose->orientation;
    Yaw = Yaw /100;
    complete_flag = 0;
    switch (left_running_step_status)
    {
    case 0:
        left_running_step_status = GOSTR_LEFTRUN_STEP;
        break;
    case GOSTR_LEFTRUN_STEP:
        if ((&cliff_valueB)->cliffValue0 == 1)
        {
            collision_front_rightrun_step_status = 0;
            left_running_step_status = COLLISION_FRONT_LEFTRUN_STEP;
            break;
        }
        else if (front_obstacle == obstacleSignal)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            if(my_abs(leakingsweep_x-current_pose->x)>500&&my_abs(leakingsweep_y-current_pose->y)>100){
				if((my_abs(current_pose->x)<half_map_wide-GRIDHEIGHT)&&(my_abs(current_pose->y)<half_map_wide-GRIDHEIGHT)){
                leakingsweep_x=current_pose->x;
                leakingsweep_y=current_pose->y;
                leftboolleaksweep=true;
                break;
				}
            }
            if (0 != leakingsweep){
                left_running_step_status = LEAKING_SWEEP_LEFTRUN_STEP;
                break;
            }
            else{
                collision_front_rightrun_step_status = 0;
                left_running_step_status = COLLISION_FRONT_LEFTRUN_STEP;
                break;
            }
        }
        else if (right_obstacle == obstacleSignal)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            if(my_abs(leakingsweep_x-current_pose->x)>500&&my_abs(leakingsweep_y-current_pose->y)>100){
				if((my_abs(current_pose->x)<half_map_wide-GRIDHEIGHT)&&(my_abs(current_pose->y)<half_map_wide-GRIDHEIGHT)){
                leakingsweep_x=current_pose->x;
                leakingsweep_y=current_pose->y;
                leftboolleaksweep=true;
                break;
				}
            }
            if (0 != leakingsweep){
                left_running_step_status = LEAKING_SWEEP_LEFTRUN_STEP;
                break;
            }
            else{
                collision_right_rightrun_step_status = 0;
                left_running_step_status = COLLISION_RIGHT_LEFTRUN_STEP;
                break;
            }
        }
        else if (left_obstacle == obstacleSignal)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            if(my_abs(leakingsweep_x-current_pose->x)>500&&my_abs(leakingsweep_y-current_pose->y)>100){
				if((my_abs(current_pose->x)<half_map_wide-GRIDHEIGHT)&&(my_abs(current_pose->y)<half_map_wide-GRIDHEIGHT)){
                leakingsweep_x=current_pose->x;
                leakingsweep_y=current_pose->y;
                leftboolleaksweep=true;
                break;
				}
            }
            if (0 != leakingsweep){
                left_running_step_status = LEAKING_SWEEP_LEFTRUN_STEP;
                break;
            }
            else{
                collision_left_rightrun_step_status = 0;
                left_running_step_status = COLLISION_LEFT_LEFTRUN_STEP;
                break;
            }
        }
        else if (my_abs(current_pose->x) > half_map_wide-2*GRIDWIDTH){
            left_running_step_status = FORWARD_BOUNDARY_LEFTRUN_STEP;
            break;
        }
        else if (my_abs(current_pose->y) > half_map_wide-2*GRIDWIDTH){
            left_running_step_status = FORWARD_BOUNDARY_LEFTRUN_STEP;
            break;
        }
		else if(my_abs(Yaw) >= 90 && my_abs(Yaw) < 170){
            if (Yaw > 0){
                linear_velocity = 0;
                angular_velocity = turn_vel;
                break;
            }
            else{
                linear_velocity = 0;
                angular_velocity = -turn_vel;
                break;
            }
        }
        else if(my_abs(Yaw) >= 90 && my_abs(Yaw) < 171){
            if (Yaw > 0){
                linear_velocity = correction_big_straight_vel;
                angular_velocity = correction_big_turn_vel;
                break;
            }
            else{
                linear_velocity = correction_big_straight_vel;
                angular_velocity = -correction_big_turn_vel;
                break;
            }
        }
        else if (my_abs(Yaw) >= 90 && my_abs(Yaw) < 178)
        {
            if (Yaw > 0){
                linear_velocity = correction_straight_vel;
                angular_velocity = correction_turn_vel;
            }
            else{
                linear_velocity = correction_straight_vel;
                angular_velocity = -correction_turn_vel;
            }
            break;
        }
        else if (my_abs(Yaw) < 90 && my_abs(Yaw) > 10)
        {
            if (Yaw > 0)
            {
                linear_velocity = 0;
                angular_velocity = -turn_vel;
                break;
            }
            else
            {
                linear_velocity = 0;
                angular_velocity = turn_vel;
                break;
            }
        }
        else if (my_abs(Yaw) < 90 && my_abs(Yaw) > 9)
        {
            if (Yaw > 0){
                linear_velocity = correction_big_straight_vel;
                angular_velocity = -correction_big_turn_vel;
                break;
            }
            else{
                linear_velocity = correction_big_straight_vel;
                angular_velocity = correction_big_turn_vel;
                break;
            }
        }
        else if (my_abs(Yaw) < 90 && my_abs(Yaw) > 2)
        {
            if (Yaw > 0){
                linear_velocity = correction_straight_vel;
                angular_velocity = -correction_turn_vel;
                break;
            }
            else{
                linear_velocity = correction_straight_vel;
                angular_velocity = correction_turn_vel;
                break;
            }
        }
		else if((adcRealTime[2]>200||adcRealTime[3]>200||adcRealTime[4]>200)&&(adcRealTime[1]<100)&&(adcRealTime[5]<100)){
            linear_velocity = 0;
            angular_velocity = 0;
			i=(current_pose->x+half_map_wide)/GRIDWIDTH;
			j=(current_pose->y+half_map_wide)/GRIDWIDTH;
			if(my_abs(Yaw)>90){
				if(i>2&&i<GRIDWIDTH&&j>0&&j<GRIDWIDTH-1){
					gridmap.map[i-2][j-1]=0;
					gridmap.map[i-2][j]=0;
					gridmap.map[i-2][j+1]=0;
					gridmap.map[i-3][j-1]=0;
					gridmap.map[i-3][j]=0;
					gridmap.map[i-3][j+1]=0;
				}
			}
			else{
				if(i>2&&i<GRIDWIDTH&&j>0&&j<GRIDWIDTH-1){
					gridmap.map[i+2][j-1]=0;
					gridmap.map[i+2][j]=0;
					gridmap.map[i+2][j+1]=0;
					gridmap.map[i+3][j-1]=0;
					gridmap.map[i+3][j]=0;
					gridmap.map[i+3][j+1]=0;
				}
			}
            if(my_abs(leakingsweep_x-current_pose->x)>500&&my_abs(leakingsweep_y-current_pose->y)>100){
				if((my_abs(current_pose->x)<half_map_wide-GRIDHEIGHT)&&(my_abs(current_pose->y)<half_map_wide-GRIDHEIGHT)){
                leakingsweep_x=current_pose->x;
                leakingsweep_y=current_pose->y;
                leftboolleaksweep=true;
                break;
				}
            }
            if (0 != leakingsweep)
            {
                left_running_step_status = LEAKING_SWEEP_LEFTRUN_STEP;
                break;
            }
            else
            {
                collision_front_rightrun_step_status = 0;
                left_running_step_status = COLLISION_FRONT_LEFTRUN_STEP;
                break;
            }
        }
        else if((adcRealTime[0]>500||adcRealTime[1]>100||adcRealTime[2]>100)&&(adcRealTime[3]<25)&&(adcRealTime[4]<25)&&(adcRealTime[5]<25)&&(my_abs(Yaw)>90)){
            linear_velocity = 0;
            angular_velocity = 0;
		i=(current_pose->x+half_map_wide)/GRIDWIDTH;
		j=(current_pose->y+half_map_wide)/GRIDWIDTH;
		if(i>2&&i<GRIDWIDTH&&j>0&&j<GRIDWIDTH){
			gridmap.map[i-2][j]=0;
			gridmap.map[i-3][j]=0;
            gridmap.map[i-2][j-1]=0;
            gridmap.map[i-3][j-1]=0;
		}
            if(my_abs(leakingsweep_x-current_pose->x)>500&&my_abs(leakingsweep_y-current_pose->y)>100){
				if((my_abs(current_pose->x)<half_map_wide-GRIDHEIGHT)&&(my_abs(current_pose->y)<half_map_wide-GRIDHEIGHT)){
                leakingsweep_x=current_pose->x;
                leakingsweep_y=current_pose->y;
                leftboolleaksweep=true;
                break;
				}
            }
            if (0 != leakingsweep){
                left_running_step_status = LEAKING_SWEEP_LEFTRUN_STEP;
                break;
            }
            else{
                collision_left_rightrun_step_status = 0;
                left_running_step_status = COLLISION_LEFT_LEFTRUN_STEP;
                break;
            }
        }
        else if((adcRealTime[0]>500||adcRealTime[1]>200||adcRealTime[2]>200)&&(adcRealTime[3]<100)&&(adcRealTime[4]<100)&&(adcRealTime[5]<100)&&(my_abs(Yaw)<90)){
            linear_velocity = 0;
            angular_velocity = 0;
		i=(current_pose->x+half_map_wide)/GRIDWIDTH;
		j=(current_pose->y+half_map_wide)/GRIDWIDTH;
		if(i>0&&i<97&&j>0&&j<GRIDWIDTH-1){
			gridmap.map[i+2][j]=0;
			gridmap.map[i+3][j]=0;
            gridmap.map[i+2][j+1]=0;
            gridmap.map[i+3][j+1]=0;
		}
            if(my_abs(leakingsweep_x-current_pose->x)>500&&my_abs(leakingsweep_y-current_pose->y)>100){
				if((my_abs(current_pose->x)<half_map_wide-GRIDHEIGHT)&&(my_abs(current_pose->y)<half_map_wide-GRIDHEIGHT)){
                leakingsweep_x=current_pose->x;
                leakingsweep_y=current_pose->y;
                leftboolleaksweep=true;
                break;
				}
            }
            if (0 != leakingsweep)
            {
                left_running_step_status = LEAKING_SWEEP_LEFTRUN_STEP;
                break;
            }
            else
            {
                collision_left_rightrun_step_status = 0;
                left_running_step_status = COLLISION_LEFT_LEFTRUN_STEP;
                break;
            }
        }
        
        else if((adcRealTime[6]>500||adcRealTime[5]>100||adcRealTime[4]>100)&&(adcRealTime[3]<25)&&(adcRealTime[2]<25)&&(adcRealTime[1]<25)&&(my_abs(Yaw)<90)){
            linear_velocity = 0;
            angular_velocity = 0;
		i=(current_pose->x+half_map_wide)/GRIDWIDTH;
		j=(current_pose->y+half_map_wide)/GRIDWIDTH;
		if(i>0&&i<97&&j>0&&j<GRIDWIDTH){
			gridmap.map[i+2][j]=0;
			gridmap.map[i+3][j]=0;
            gridmap.map[i+2][j-1]=0;
            gridmap.map[i+3][j-1]=0;
		}
            if(my_abs(leakingsweep_x-current_pose->x)>500&&my_abs(leakingsweep_y-current_pose->y)>100){
				if((my_abs(current_pose->x)<half_map_wide-GRIDHEIGHT)&&(my_abs(current_pose->y)<half_map_wide-GRIDHEIGHT)){
                leakingsweep_x=current_pose->x;
                leakingsweep_y=current_pose->y;
                leftboolleaksweep=true;
                break;
				}
            }
            if (0 != leakingsweep){
                left_running_step_status = LEAKING_SWEEP_LEFTRUN_STEP;
                break;
            }
            else{
                collision_right_rightrun_step_status = 0;
                left_running_step_status = COLLISION_RIGHT_LEFTRUN_STEP;
                break;
            }
        }
        else if((adcRealTime[6]>500||adcRealTime[5]>200||adcRealTime[4]>200)&&(adcRealTime[3]<100)&&(adcRealTime[2]<100)&&(adcRealTime[1]<100)&&(my_abs(Yaw)>90)){
            linear_velocity = 0;
            angular_velocity = 0;
		i=(current_pose->x+half_map_wide)/GRIDWIDTH;
		j=(current_pose->y+half_map_wide)/GRIDWIDTH;
		if(i>2&&i<GRIDWIDTH&&j>0&&j<GRIDWIDTH-1){
			gridmap.map[i-2][j]=0;
			gridmap.map[i-3][j]=0;
            gridmap.map[i-2][j+1]=0;
            gridmap.map[i-3][j+1]=0;
		}
            if(my_abs(leakingsweep_x-current_pose->x)>500&&my_abs(leakingsweep_y-current_pose->y)>100){
				if((my_abs(current_pose->x)<half_map_wide-GRIDHEIGHT)&&(my_abs(current_pose->y)<half_map_wide-GRIDHEIGHT)){
                leakingsweep_x=current_pose->x;
                leakingsweep_y=current_pose->y;
                leftboolleaksweep=true;
                break;
				}
            }
            if (0 != leakingsweep){
                left_running_step_status = LEAKING_SWEEP_LEFTRUN_STEP;
                break;
            }
            else{
                collision_right_rightrun_step_status = 0;
                left_running_step_status = COLLISION_RIGHT_LEFTRUN_STEP;
                break;
            }
        }
        else
        {
            linear_velocity = real_gostaright_vel;
            angular_velocity = 0;
        }
        break;
    case FORWARD_BOUNDARY_LEFTRUN_STEP:
        FunctionStatus=ForwardBoundaryLeftRunStep(current_pose,obstacleSignal);
        if(FunctionStatus==1||FunctionStatus==3)
        {
            if((FunctionStatus==1&&x_more_map==false)||(FunctionStatus==3&&y_more_map==false)){
                if(FunctionStatus==1){
                    if(reverse_moremap==1){
                        if(current_pose->x>0){
                        }else{
                            motionSteps=1;
                            more_map=true;
                        }
                    }
                    else if(reverse_moremap==-1){
                        if(current_pose->x>0){
                            motionSteps=1;
                            more_map=true;	
                        }else{
                        }
                    }
                    else{
                        motionSteps=1;
                        more_map=true;
                    }
                }
                else{
                    motionSteps=3;
                    more_map=true;
                }
            }						
            else{
                if(FunctionStatus==3){
                    left_running_step_status = 0;
                    right_forward_boundary_status=0;
                    FunctionStatus=0;
                    complete_flag=1;
                    break;
                }
                if(FunctionStatus==1){
                    left_running_step_status = GOSTR_LEFTRUN_STEP;
                    right_forward_boundary_status=0;
                    FunctionStatus=0;
                    break;
                }
            }
            left_running_step_status = 0;
            right_forward_boundary_status=0;
            FunctionStatus=0;
            break;
        }
        if(FunctionStatus==2){
            FunctionStatus=0;
            left_running_step_status = LEFT_STUCK_FORWARD_BOUNDARY_LEFT_RUNSTEP;
            right_forward_boundary_status=0;
        }
        break;
    case LEFT_STUCK_FORWARD_BOUNDARY_LEFT_RUNSTEP:
        FunctionStatus=StuckLeftRunStep(current_pose,obstacleSignal);
        if(FunctionStatus==1){
            left_running_step_status = GOSTR_LEFTRUN_STEP;
            stuck_right_run_step=0;
            break;
        }
        break;
    case COLLISION_RIGHT_LEFTRUN_STEP:
        FunctionStatus = CollisionRightLeftRunStep(current_pose, obstacleSignal);
        if (1 == FunctionStatus)
        {
            left_running_step_status = GOSTR_LEFTRUN_STEP;
            collision_right_rightrun_step_status = 0;
            FunctionStatus = 0;
            break;
        }
        if (2 == FunctionStatus)
        {
            left_running_step_status = LEFTREVERSEWALKEDGE;
            collision_right_rightrun_step_status = 0;
            FunctionStatus = 0;
            break;
        }
        break;
    case LEFTREVERSEWALKEDGE:
        FunctionStatus = LeftReverseWalkEdge(current_pose, obstacleSignal);
        if (1 == FunctionStatus)
        {
            left_running_step_status = GOSTR_LEFTRUN_STEP;
            right_reverse_walk_edge_status = 0;
            FunctionStatus = 0;
            break;
        }
        if (2 == FunctionStatus)
        {
            left_running_step_status = LEFTEDGEDILEMMA;
            right_reverse_walk_edge_status = 0;
            FunctionStatus = 0;
            break;
        }
        if (3 == FunctionStatus)
        {
            left_running_step_status = GOSTR_LEFTRUN_STEP;
            right_reverse_walk_edge_status = 0;
            complete_flag = 1;
            FunctionStatus = 0;
            break;
        }
        break;
    case LEFTEDGEDILEMMA:
        FunctionStatus=LeftEdgeDilemma(current_pose,obstacleSignal);
        if(1==FunctionStatus){
            left_running_step_status = GOSTR_LEFTRUN_STEP;
            right_edge_dilemma_status=0;
            FunctionStatus=0;
            break;
        }
        if(2==FunctionStatus){
            if(bool_leakingsweep_y==true){
                left_running_step_status = GOSTR_LEFTRUN_STEP;
                right_edge_dilemma_status=0;
                FunctionStatus=0;
                break;
            }
            else{
                left_running_step_status =0;
                right_edge_dilemma_status=0;
                complete_flag=1;
                FunctionStatus=0;
                break;
            }
        }
        break;
    case COLLISION_LEFT_LEFTRUN_STEP:
        FunctionStatus = CollisionLeftLeftRunStep(current_pose, obstacleSignal);
        if (1 == FunctionStatus)
        {
            left_running_step_status = GOSTR_LEFTRUN_STEP;
            collision_left_rightrun_step_status = 0;
            FunctionStatus = 0;
            break;
        }
        if (2 == FunctionStatus)
        {
            left_running_step_status = LEFTWALKEDGE;
            collision_left_rightrun_step_status = 0;
            FunctionStatus = 0;
            break;
        }
        break;
    case LEFTWALKEDGE:
        FunctionStatus = LeftWalkEdge(current_pose, obstacleSignal);
        if (1 == FunctionStatus)
        {
            left_running_step_status = GOSTR_LEFTRUN_STEP;
            right_walk_edge_status = 0;
            FunctionStatus = 0;
            break;
        }
        if (2 == FunctionStatus)
        {
            left_running_step_status = LEFTEDGEDILEMMA;
            right_walk_edge_status = 0;
            FunctionStatus = 0;
            break;
        }
        if (3 == FunctionStatus)
        {
            left_running_step_status = GOSTR_LEFTRUN_STEP;
            right_walk_edge_status = 0;
            complete_flag = 1;
            FunctionStatus = 0;
            break;
        }
        break;
    case COLLISION_FRONT_LEFTRUN_STEP:
        FunctionStatus = CollisionFrontLeftRunStep(current_pose, obstacleSignal);
        if (1 == FunctionStatus)
        {
            left_running_step_status = GOSTR_LEFTRUN_STEP;
            collision_front_rightrun_step_status = 0;
            FunctionStatus = 0;
            break;
        }
        if (2 == FunctionStatus)
        {
            left_running_step_status = LEFTWALKEDGE;
            collision_front_rightrun_step_status = 0;
            FunctionStatus = 0;
            break;
        }
        if (3 == FunctionStatus)
        {
            left_running_step_status = LEFTREVERSEWALKEDGE;
            collision_front_rightrun_step_status = 0;
            FunctionStatus = 0;
            break;
        }
        break;
    case LEAKING_SWEEP_LEFTRUN_STEP:
        if (LeftReadyLeakingSweep(current_pose, obstacleSignal))
        {
            left_running_step_status = GOSTR_LEFTRUN_STEP;
            leakingsweep = 0;
        }
        break;
    default:
        break;
    }
    return complete_flag;
}



unsigned char  ForwardBoundaryLeftRunStep(POSE *current_pose, unsigned char obstacleSignal){
    int Yaw;
    unsigned char complete_flag = 0;
    Yaw = current_pose->orientation;
    Yaw = Yaw /100;
    if(judgment_Stuck_status==0){
        judgment_Stuck_status_x = current_pose->x;
        judgment_Stuck_status_y = current_pose->y;
        judgment_Stuck_status_yaw = Yaw;
        judgment_Stuck_status = 1;
    }
    if(!old_bow_continue){
        if(my_abs(current_pose->x-judgment_Stuck_status_x)>lateral_move_distance/2||my_abs(current_pose->y-judgment_Stuck_status_y)>lateral_move_distance/2 ){
            judgment_Stuck_status=0;
            judgment_Stuck_status_x = current_pose->x;
            judgment_Stuck_status_y = current_pose->y;
            if(my_abs(judgment_Stuck_status_yaw-Yaw)<5){
                if(my_abs(current_pose->x)>half_map_wide-2*GRIDWIDTH){
                    stuck_x=true;
                }
                else{
                    stuck_y=true;
                }
                old_bow_continue=false;
                complete_flag=2;
                return complete_flag;
            }
            else{
                if(my_abs(current_pose->y)>half_map_wide-2*GRIDWIDTH){
                    linear_velocity = 0;
                    angular_velocity = 0;
                    right_forward_boundary_status = 0;
                    judgment_Stuck_status=0;
                    old_bow_continue=false;
                    complete_flag=3;
                    return complete_flag;
                }
            }
        }
    }
    switch (right_forward_boundary_status)
    {
    case 0:
        if (my_abs(Yaw) < 90){
            right_forward_boundary_status = LEFT_FORWARDBOUNDARY_YAW_LESS_ABS10;
        }
        else
        {
            right_forward_boundary_status = LEFT_FORWARDBOUNDARY_YAW_OTHER;
        }
        break;
    case LEFT_FORWARDBOUNDARY_YAW_LESS_ABS10:
        right_forward_boundary_status = LEFT_FORWARDBOUNDARY_CCLOCK_TARGET_YAW_MORE_ABS178;
        break;
    case LEFT_FORWARDBOUNDARY_CCLOCK_TARGET_YAW_MORE_ABS178:
        linear_velocity = 100;
        angular_velocity = 57;
        if(my_abs(Yaw) > 175){
            right_forward_boundary_status = LEFT_FORWARDBOUNDARY_GOSTRAIGHT;
            old_bow_continue=true;
            break;
        }
        if (obstacleSignal != none_obstacle||(&cliff_valueB)->cliffValue0 == 1){
            right_forward_boundary_status = LEFT_FORWARDBOUNDARY_CCLOCK_TARGET_YAW_MORE_ABS178_COLLISION;
            break;
        }
        break;
    case LEFT_FORWARDBOUNDARY_CCLOCK_TARGET_YAW_MORE_ABS178_COLLISION:
        if (turn_start_update == 0)
        {
            turn_start_x = current_pose->x;
            turn_start_y = current_pose->y;
            turn_start_update = 1;
        }
        linear_velocity = -long_stra_vel;
        angular_velocity = 0;
        if (my_abs(turn_start_x - current_pose->x) > turn_backward_distance || my_abs(turn_start_y - current_pose->y) > turn_backward_distance)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            turn_start_update = 0;
            right_forward_boundary_status = LEFT_FORWARDBOUNDARY_CCLOCK_TARGET_YAW_MORE_ABS178;
            break;
        }
        break;
    case LEFT_FORWARDBOUNDARY_YAW_OTHER:
        right_forward_boundary_status = LEFT_FORWARDBOUNDARY_CLOCK_TARGET_YAW_LESS_ABS3;
        break;
    case LEFT_FORWARDBOUNDARY_CLOCK_TARGET_YAW_LESS_ABS3:
        linear_velocity = 100;
        angular_velocity = -57;
        if (my_abs(Yaw) < 5)
        {
            right_forward_boundary_status = LEFT_FORWARDBOUNDARY_GOSTRAIGHT;
            old_bow_continue=true;
            break;
        }
        if (obstacleSignal != none_obstacle)
        {
            right_forward_boundary_status = LEFT_FORWARDBOUNDARY_CLOCK_TARGET_YAW_LESS_ABS3_COLLISION;
            break;
        }
        break;
    case LEFT_FORWARDBOUNDARY_CLOCK_TARGET_YAW_LESS_ABS3_COLLISION:
        if (turn_start_update == 0)
        {
            turn_start_x = current_pose->x;
            turn_start_y = current_pose->y;
            turn_start_update = 1;
        }
        linear_velocity = -long_stra_vel;
        angular_velocity = 0;
        if (my_abs(turn_start_x - current_pose->x) > collision_backward_distance || my_abs(turn_start_y - current_pose->y) > collision_backward_distance)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            turn_start_update = 0;
            right_forward_boundary_status = LEFT_FORWARDBOUNDARY_CLOCK_TARGET_YAW_LESS_ABS3;
            break;
        }
        break;
    case LEFT_FORWARDBOUNDARY_GOSTRAIGHT:
        if (turn_start_update == 0)
        {
            turn_start_x = current_pose->x;
            turn_start_y = current_pose->y;
            turn_start_update = 1;
        }
        linear_velocity = long_stra_vel;
        angular_velocity = 0;
        if ((my_abs(turn_start_x - current_pose->x) > 2*GRIDWIDTH || my_abs(turn_start_y - current_pose->y) > 2*GRIDWIDTH) && my_abs(Yaw) < 90 && current_pose->x > 0)
        {
            turn_start_update = 0;
            right_forward_boundary_status = LEFT_FORWARDBOUNDARY_COMPLETE;
            break;
        }
        
        if ((my_abs(turn_start_x - current_pose->x) > 2*GRIDWIDTH || my_abs(turn_start_y - current_pose->y) > 2*GRIDWIDTH) && my_abs(Yaw) > 90 && current_pose->x < 0)
        {
            turn_start_update = 0;
            right_forward_boundary_status = LEFT_FORWARDBOUNDARY_COMPLETE;
            break;
        }
        linear_velocity = long_stra_vel;
        angular_velocity = 0;
        if (obstacleSignal != none_obstacle||(&cliff_valueB)->cliffValue0 == 1)
        {
            right_forward_boundary_status = LEFT_FORWARDBOUNDARY_COMPLETE;
            break;
        }
        if (my_abs(current_pose->x) < half_map_wide-2*GRIDWIDTH)
        {
            right_forward_boundary_status = LEFT_FORWARDBOUNDARY_COMPLETE;
            break;
        }
        break;
    case LEFT_FORWARDBOUNDARY_COMPLETE:
        right_forward_boundary_status = 0;
        judgment_Stuck_status=0;
        old_bow_continue=false;
        complete_flag=1;
        break;
    }
    return complete_flag;
}



unsigned char  CollisionRightLeftRunStep(POSE *current_pose,unsigned char obstacleSignal){
    int Yaw;
    unsigned char complete_flag=0;
    Yaw = current_pose->orientation;
    Yaw = Yaw /100;
    switch(collision_right_rightrun_step_status)
    {
    case 0:
        if(turn_start_update == 0)
        {
            if(obstacleSignal==none_obstacle){
                obstacleSignal=right_obstacle;
                last_position_x = current_pose->x;
                last_position_y = current_pose->y;
                if(my_abs(Yaw) < 90){
                    collision_right_rightrun_step_status = GOSTR_BYPASS_LOOP_LRUN_CR_DLYL;
                }
                else{
                    collision_right_rightrun_step_status = DIR_LEFT_YAW_MORE_ABS90_CRLRS;
                }
                infrared_collision=true;
                break;
            } 
            turn_start_x = current_pose->x;
            turn_start_y = current_pose->y;
            turn_start_update = 1;
        }
        linear_velocity = -long_stra_vel;
        angular_velocity = 0;
        if (my_abs(turn_start_x - current_pose->x) > collision_backward_distance || my_abs(turn_start_y - current_pose->y) > collision_backward_distance)
        {
            collision_right_rightrun_step_status = GOBACK_DISTANCE_CRLRS;
            turn_start_update = 0;
            break;
        }
        break;
    case GOBACK_DISTANCE_CRLRS:
        if(turn_start_update == 0)
        {
            turn_start_x = current_pose->x;
            turn_start_y = current_pose->y;
            turn_start_update = 1;
        }
        linear_velocity = -long_stra_vel;
        angular_velocity = 0;
        
        if (my_abs(turn_start_x - current_pose->x) > side_backward_distance||my_abs(turn_start_y - current_pose->y) > side_backward_distance)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            if (my_abs(Yaw) > 90)
            {
                collision_right_rightrun_step_status = DIR_LEFT_YAW_MORE_ABS90_CRLRS;
            }
            else
            {
                collision_right_rightrun_step_status = DIR_LEFT_YAW_LESS_ABS90_CRLRS;
            }
            turn_start_update = 0;
            break;
        }
        break;
    case DIR_LEFT_YAW_MORE_ABS90_CRLRS:
        linear_velocity = 0;
        angular_velocity = 0;
        last_position_y = current_pose->y;
        collision_right_rightrun_step_status  = TURN_CLOCK_TARGET_YAW_POS123_LRUN_CR_DLYM;
        break;
    case TURN_CLOCK_TARGET_YAW_POS123_LRUN_CR_DLYM:
        linear_velocity = 0;
        angular_velocity = -turn_vel;
        if (my_abs(Yaw) < 90)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            collision_right_rightrun_step_status  = GOSTR_YAW_EQUAL_POS123_LRUN_CR_DLYM;
            break;
        }
        if (obstacleSignal !=none_obstacle||(&cliff_valueB)->cliffValue0 == 1)
        {
            collision_right_rightrun_step_status  = TURN_CLOCK_TARGET_YAW_POS123_COLLISION_LRUN_CR_DLYM;
            break;
        }
        break;
    case TURN_CLOCK_TARGET_YAW_POS123_COLLISION_LRUN_CR_DLYM:
        if(turn_start_update == 0)
        {
            turn_start_x = current_pose->x;
            turn_start_y = current_pose->y;
            turn_start_update = 1;
        }
        linear_velocity = -long_stra_vel;
        angular_velocity = 0;
        if (my_abs(turn_start_x - current_pose->x) > second_turn_backward_distance || my_abs(turn_start_y - current_pose->y) > second_turn_backward_distance)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            turn_start_update = 0;
            collision_right_rightrun_step_status = TURN_CLOCK_TARGET_YAW_POS123_LRUN_CR_DLYM;
            break;
        }
        break;
    case GOSTR_YAW_EQUAL_POS123_LRUN_CR_DLYM:
        if (obstacleSignal !=none_obstacle||(&cliff_valueB)->cliffValue0 == 1)
        {
			linear_velocity = 0;
            angular_velocity = 0;
            collision_right_rightrun_step_status = LEFT_REVERSE_WALK_EDGE_LRUN_CR_DLYM;
            break;			
        }
        if (my_abs(last_position_y - current_pose->y) > lateral_move_distance)
        {
            collision_right_rightrun_step_status = TURN_CLOCK_TARGET_YAW_POS98_LRUN_CR_DLYM;
            break;
        }
        linear_velocity = long_stra_vel;
        angular_velocity = 0;
        break;
    case TURN_CLOCK_TARGET_YAW_POS98_LRUN_CR_DLYM:
        linear_velocity = 0;
        angular_velocity = -turn_vel;
        if (Yaw < 85)
        {
            collision_right_rightrun_step_status = GOSTR_YAW_EQUAL_POS98_LRUN_CR_DLYM;
            break;
        }
        if (obstacleSignal !=none_obstacle||(&cliff_valueB)->cliffValue0 == 1)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            collision_right_rightrun_step_status = TURN_CLOCK_TARGET_YAW_POS98_COLLISION_LRUN_CR_DLYM;
            break;
        }
        break;
    case TURN_CLOCK_TARGET_YAW_POS98_COLLISION_LRUN_CR_DLYM:
        if(turn_start_update == 0)
        {
            turn_start_x = current_pose->x;
            turn_start_y = current_pose->y;
            turn_start_update = 1;
        }
        linear_velocity = -long_stra_vel;
        angular_velocity = 0;
        if (my_abs(turn_start_x - current_pose->x) > turn_backward_distance || my_abs(turn_start_y - current_pose->y) > turn_backward_distance)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            turn_start_update = 0;
            collision_right_rightrun_step_status = TURN_CLOCK_TARGET_YAW_POS98_LRUN_CR_DLYM;
            break;
        }
        break;
    case GOSTR_YAW_EQUAL_POS98_LRUN_CR_DLYM:
        linear_velocity = long_stra_vel;
        angular_velocity = 0;
        if (obstacleSignal !=none_obstacle||(&cliff_valueB)->cliffValue0 == 1)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            collision_right_rightrun_step_status = LEFT_REVERSE_WALK_EDGE_LRUN_CR_DLYM;
            break;
        }
        if (my_abs(last_position_y - current_pose->y) > lateral_move_distance)
        {
            collision_right_rightrun_step_status = TURN_CLOCK_TARGET_YAW_ABS3_LRUN_CR_DLYM;
            break;
        }
        break;
    case LEFT_REVERSE_WALK_EDGE_LRUN_CR_DLYM:
        collision_right_rightrun_step_status = 0;
        complete_flag = 2;
        break;
    case TURN_CLOCK_TARGET_YAW_ABS3_LRUN_CR_DLYM:
        linear_velocity = 0;
        angular_velocity = -turn_vel;
        if (my_abs(Yaw) < 5) //8
        {
            collision_right_rightrun_step_status = COMPLETE_LRUN_CR_DLYM;
            break;
        }
        if (obstacleSignal !=none_obstacle||(&cliff_valueB)->cliffValue0 == 1)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            collision_right_rightrun_step_status = TURN_CLOCK_TARGET_YAW_ABS3_COLLISION_LRUN_CR_DLYM;
            break;
        }
        break;
    case TURN_CLOCK_TARGET_YAW_ABS3_COLLISION_LRUN_CR_DLYM:
        if(turn_start_update == 0)
        {
            turn_start_x = current_pose->x;
            turn_start_y = current_pose->y;
            turn_start_update = 1;
        }
        linear_velocity = -long_stra_vel;
        angular_velocity = 0;
        if (my_abs(turn_start_x - current_pose->x) > collision_backward_distance || my_abs(turn_start_y - current_pose->y) > collision_backward_distance)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            turn_start_update = 0;
            collision_right_rightrun_step_status = TURN_CLOCK_TARGET_YAW_ABS3_LRUN_CR_DLYM;
            break;
        }
        break;
    case COMPLETE_LRUN_CR_DLYM:
        collision_right_rightrun_step_status = 0;
        complete_flag = 1;
        break;
    case DIR_LEFT_YAW_LESS_ABS90_CRLRS:
        linear_velocity = 0;
        angular_velocity = 0;
        collision_right_rightrun_step_status = TURN_CCLCOK_TARGET_YAW_ABS27_LRUN_CR_DLYL;
        break;
    case TURN_CCLCOK_TARGET_YAW_ABS27_LRUN_CR_DLYL:
        linear_velocity = 0;
        angular_velocity = turn_vel;
        if (my_abs(Yaw) > 27)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            collision_right_rightrun_step_status = GOSTR_YAW_EQUAL_ABS27_LRUN_CR_DLYL;
            break;
        }
        if (obstacleSignal !=none_obstacle||(&cliff_valueB)->cliffValue0 == 1)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            collision_right_rightrun_step_status = TURN_CCLCOK_TARGET_YAW_ABS27_COLLISION_LRUN_CR_DLYL;
            break;
        }
        break;
    case TURN_CCLCOK_TARGET_YAW_ABS27_COLLISION_LRUN_CR_DLYL:
        if(turn_start_update == 0)
        {
            turn_start_x = current_pose->x;
            turn_start_y = current_pose->y;
            turn_start_update = 1;
        }
        linear_velocity = -long_stra_vel;
        angular_velocity = 0;
        if (my_abs(turn_start_x - current_pose->x) > second_turn_backward_distance || my_abs(turn_start_y - current_pose->y) > second_turn_backward_distance)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            turn_start_update = 0;
            collision_right_rightrun_step_status = TURN_CCLCOK_TARGET_YAW_ABS27_LRUN_CR_DLYL;
            break;
        }
        break;
    case GOSTR_YAW_EQUAL_ABS27_LRUN_CR_DLYL:
        cnt_update = 0;
        last_position_y = current_pose->y;
        last_position_xx = current_pose->x;
        collision_right_rightrun_step_status = GOSTR_BYPASS_LRUN_CR_DLYL;
        break;
    case GOSTR_BYPASS_LRUN_CR_DLYL:
        last_position_x = current_pose->x;
        cnt_update +=1;
        linear_velocity = long_stra_vel;
        angular_velocity = 0;
        if(cnt_update > 1&&my_abs(last_position_xx - current_pose->x)<20&&my_abs(last_position_y - current_pose->y)<20)
        {
            cnt_update = 0;
            last_position_y=current_pose->y;
            collision_right_rightrun_step_status = TURN_CLOCK_TARGET_YAW_ABS90_LRUN_CR_DLYM;
            break;
        }
        collision_right_rightrun_step_status = GOSTR_BYPASS_LOOP_LRUN_CR_DLYL;
        break;
        
    case TURN_CLOCK_TARGET_YAW_ABS90_LRUN_CR_DLYM:
        linear_velocity = 0;
        angular_velocity = turn_vel;
        if (my_abs(Yaw)>90)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            collision_right_rightrun_step_status = GOSTR_YAW_EQUAL_ABS90_LRUN_CR_DLYM;
            break;
        }
        if (obstacleSignal !=none_obstacle||(&cliff_valueB)->cliffValue0 == 1)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            collision_right_rightrun_step_status = TURN_CLOCK_TARGET_YAW_ABS90_COLLISION_LRUN_CR_DLYM;
            break;
        }
        break;
    case TURN_CLOCK_TARGET_YAW_ABS90_COLLISION_LRUN_CR_DLYM:
        if(turn_start_update == 0)
        {
            turn_start_x = current_pose->x;
            turn_start_y = current_pose->y;
            turn_start_update = 1;
        }
        linear_velocity = -long_stra_vel;
        angular_velocity = 0;
        if (my_abs(turn_start_x - current_pose->x) > turn_backward_distance || my_abs(turn_start_y - current_pose->y) > turn_backward_distance)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            turn_start_update = 0;
            collision_right_rightrun_step_status = TURN_CLOCK_TARGET_YAW_ABS90_LRUN_CR_DLYM;
            break;
        }
        break;
    case GOSTR_YAW_EQUAL_ABS90_LRUN_CR_DLYM:
        linear_velocity = long_stra_vel;
        angular_velocity = 0;
        if (obstacleSignal !=none_obstacle||(&cliff_valueB)->cliffValue0 == 1)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            left_running_step_status = LEFTWALKEDGE;
            collision_right_rightrun_step_status = 0;
            break;
        }
        if (my_abs(last_position_y - current_pose->y) > lateral_move_distance / 2)
        {
            collision_right_rightrun_step_status = MORE_TRY_BREAK_BYPASS_LRUN_CR_DLYL;
            break;
        }
        break;
        
    case MORE_TRY_BREAK_BYPASS_LRUN_CR_DLYL:
        if (obstacleSignal !=none_obstacle)
        {
            collision_right_rightrun_step_status = MORE_TRY_BREAK_BYPASS_COLLISION_LRUN_CR_DLYL;
            bow_continue = true;
            break;
        }
        if (my_abs(Yaw) > 172)
        {
            bow_continue = true;
            collision_right_rightrun_step_status = GOSTR_BYPASS_BOW_CONTINUE_EXIT_LRUN_CR_DLYL;
            break;
        }
        linear_velocity = 0;
        angular_velocity = turn_vel;
        break;
    case MORE_TRY_BREAK_BYPASS_COLLISION_LRUN_CR_DLYL:
        if(turn_start_update == 0)
        {
            turn_start_x = current_pose->x;
            turn_start_y = current_pose->y;
            turn_start_update = 1;
        }
        linear_velocity = -long_stra_vel;
        angular_velocity = 0;
        if (my_abs(turn_start_x - current_pose->x) > collision_backward_distance || my_abs(turn_start_y - current_pose->y) > collision_backward_distance)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            collision_right_rightrun_step_status = MORE_TRY_BREAK_BYPASS_LRUN_CR_DLYL;
            turn_start_update = 0;
            break;
        }
        break;
    case GOSTR_BYPASS_LOOP_LRUN_CR_DLYL:
        if (obstacleSignal == right_obstacle)
        {
            collision_right_rightrun_step_status = RIGHT_COLLISION_BYPASS_LRUN_CR_DLYL;
			last_position_x=current_pose->x;
			temporary_yaw = Yaw;
			infrared_collision=false;
            break;
        }
        if ((current_pose->y - last_position_y) > close_edge)
        {
            collision_right_rightrun_step_status = GOSTR_BYPASS_BOW_CONTINUE_LRUN_CR_DLYL;
            break;
        }
        if(obstacleSignal == front_obstacle || obstacleSignal == left_obstacle||(&cliff_valueB)->cliffValue0 == 1){
            last_position_y=current_pose->y;
            collision_right_rightrun_step_status = TURN_CLOCK_TARGET_YAW_ABS90_LRUN_CR_DLYM;
            break;
        }
        if (last_position_y > current_pose->y + 1)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            collision_right_rightrun_step_status = GOSTR_BYPASS_BOW_CONTINUE_TARGET_YAW_ABS3_LRUN_CR_DLYL;
            break;
        }
        if (my_abs(current_pose->x) > half_map_wide-2*GRIDWIDTH)
        {
            collision_right_rightrun_step_status = COMPLETE_LRUN_CR_DLYL;
            break;
        }
        if (my_abs(Yaw) >90)
        {
            collision_right_rightrun_step_status = TURN_CCLOCK_TARGET_YAW_MORE_AB173_LRUN_CR_DLYL;
            break;
        }
        if(infrared_collision==true&&my_abs(Yaw)<75){
            if(adcRealTime[6]>500|| adcRealTime[5]>50||adcRealTime[4]>50){
                linear_velocity = 100;
                angular_velocity = 57;
                if(adcRealTime[4]>50){
                    linear_velocity = 50;
                    angular_velocity = turn_vel;
                }
            }
            else{
				right_edge_judgment_repeat();
//                if(adcRealTime[9]>100&&adcRealTime[9]<1500){
//                    delimma_edge=0;
//                    linear_velocity = 200;
//                    angular_velocity = 0;
//                }
//                if(adcRealTime[9]>=1500){
//                    delimma_edge=0;
//                    linear_velocity = 200;
//                    angular_velocity = 10;
//                }
//                if(adcRealTime[9]<100){
//                    if(delimma_edge<10){
//                        delimma_edge++;
//                        linear_velocity = 200;
//                        angular_velocity = -10;
//                    }else{
//                        linear_velocity = 100;
//                        angular_velocity = -20;
//                    }
//                }
            }
        }
        else{
            if(my_abs(last_position_x - current_pose->x) > lateral_move_distance/3){
                infrared_collision=true;
				last_position_x=current_pose->x;
                collision_right_rightrun_step_status = GOSTR_X_MORE_ONE_THIRD_LATERALDIS_BYPASS_LRUN_CR_DLYL;
                break;
            }
            linear_velocity = long_stra_vel;
            angular_velocity = 0;
        }
        break;
    case GOSTR_BYPASS_BOW_CONTINUE_TARGET_YAW_ABS3_LRUN_CR_DLYL:
        linear_velocity = 0;
        angular_velocity =turn_vel;
        if (obstacleSignal !=none_obstacle||(&cliff_valueB)->cliffValue0 == 1)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            collision_right_rightrun_step_status =  GOSTR_BYPASS_BOW_CONTINUE_TARGET_YAW_ABS3_COLLISION_LRUN_CR_DLYL;
            break;
        }
        if (my_abs(Yaw) < 8)
        {
            collision_right_rightrun_step_status =  GOSTR_BYPASS_BOW_CONTINUE_EXIT_LRUN_CR_DLYL;
            break;
        }
        break;
    case GOSTR_BYPASS_BOW_CONTINUE_TARGET_YAW_ABS3_COLLISION_LRUN_CR_DLYL:
        if(turn_start_update == 0)
        {
            turn_start_x = current_pose->x;
            turn_start_y = current_pose->y;
            turn_start_update = 1;
        }
        linear_velocity = -long_stra_vel;
        angular_velocity = 0;
        if (my_abs(turn_start_x - current_pose->x) > collision_backward_distance || my_abs(turn_start_y - current_pose->y) > collision_backward_distance)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            turn_start_update = 0;
            collision_right_rightrun_step_status = GOSTR_BYPASS_BOW_CONTINUE_TARGET_YAW_ABS3_LRUN_CR_DLYL;
            break;
        }
        break;
        
    case GOSTR_BYPASS_BOW_CONTINUE_LRUN_CR_DLYL:
        if (obstacleSignal !=none_obstacle||(&cliff_valueB)->cliffValue0 == 1)
        {
            collision_right_rightrun_step_status = GOSTR_BYPASS_BOW_CONTINUE_COLLISION_LRUN_CR_DLYL;
            break;
        }
        if (my_abs(Yaw) > 172)
        {
            bow_continue = true;
            collision_right_rightrun_step_status = GOSTR_BYPASS_BOW_CONTINUE_EXIT_LRUN_CR_DLYL;
            break;
        }
        linear_velocity = 0;
        angular_velocity = turn_vel;
        break;
        
    case GOSTR_BYPASS_BOW_CONTINUE_COLLISION_LRUN_CR_DLYL:
        if(turn_start_update == 0)
        {
            turn_start_x = current_pose->x;
            turn_start_y = current_pose->y;
            turn_start_update = 1;
        }
        linear_velocity = -long_stra_vel;
        angular_velocity = 0;
        if (my_abs(turn_start_x - current_pose->x) > collision_backward_distance ||my_abs(turn_start_y - current_pose->y) > collision_backward_distance)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            bow_continue = true;
            collision_right_rightrun_step_status = GOSTR_BYPASS_BOW_CONTINUE_LRUN_CR_DLYL;
            turn_start_update = 0;
            break;
        }
        break;
        
    case RIGHT_COLLISION_BYPASS_LRUN_CR_DLYL:
        if(turn_start_update == 0)
        {
            turn_start_x = current_pose->x;
            turn_start_y = current_pose->y;
            turn_start_update = 1;
        }
        linear_velocity = -long_stra_vel;
        angular_velocity = 0;
        
        if (my_abs(turn_start_x - current_pose->x) > collision_backward_distance || my_abs(turn_start_y - current_pose->y) > collision_backward_distance)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            turn_start_update = 0;
            temporary_yaw = Yaw;
            if (my_abs(Yaw) < 15){
				collision_right_rightrun_step_status = TURN_CCLOCK_TARGET_YAW_LESS_ABS15_LRUN_CR_DLYL;
			}
            else{
				collision_right_rightrun_step_status = TURN_CCLOCK_TARGET_YAW_MORE_ABS15_LRUN_CR_DLYL;
			}
            break;
        }
        break;
    case TURN_CCLOCK_TARGET_YAW_LESS_ABS15_LRUN_CR_DLYL:
        if (obstacleSignal !=none_obstacle||(&cliff_valueB)->cliffValue0 == 1)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            collision_right_rightrun_step_status = TURN_CCLOCK_TARGET_YAW_LESS_ABS15_COLLISION_LRUN_CR_DLYL;
            break;
        }
        if (my_abs(Yaw) > 15)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            collision_right_rightrun_step_status = GOSTR_BYPASS_LRUN_CR_DLYL;
            break;
        }
		if (my_abs(Yaw) >90)
        {
            collision_right_rightrun_step_status = TURN_CCLOCK_TARGET_YAW_MORE_AB173_LRUN_CR_DLYL;
            break;
        }
        linear_velocity = 0;
        angular_velocity = turn_vel;
        break;
    case TURN_CCLOCK_TARGET_YAW_LESS_ABS15_COLLISION_LRUN_CR_DLYL:
        if(turn_start_update == 0)
        {
            turn_start_x = current_pose->x;
            turn_start_y = current_pose->y;
            turn_start_update = 1;
        }
        linear_velocity = -long_stra_vel;
        angular_velocity = 0;
        if (my_abs(turn_start_x - current_pose->x) > collision_backward_distance || my_abs(turn_start_y - current_pose->y) > collision_backward_distance)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            turn_start_update = 0;
            collision_right_rightrun_step_status = TURN_CCLOCK_TARGET_YAW_LESS_ABS15_LRUN_CR_DLYL;
            break;
        }
        break;
        
    case TURN_CCLOCK_TARGET_YAW_MORE_ABS15_LRUN_CR_DLYL:
        if (obstacleSignal !=none_obstacle||(&cliff_valueB)->cliffValue0 == 1)
        {
            collision_right_rightrun_step_status = TURN_CCLOCK_TARGET_YAW_MORE_ABS15_COLLISION_LRUN_CR_DLYL;
            break;
        }
        if (my_abs(Yaw - temporary_yaw) > 15)
        {
            collision_right_rightrun_step_status = GOSTR_BYPASS_LRUN_CR_DLYL;
            break;
        }
		if (my_abs(Yaw) >90)
        {
            collision_right_rightrun_step_status = TURN_CCLOCK_TARGET_YAW_MORE_AB173_LRUN_CR_DLYL;
            break;
        }
        linear_velocity = 0;
        angular_velocity = turn_vel;
        break;
    case TURN_CCLOCK_TARGET_YAW_MORE_ABS15_COLLISION_LRUN_CR_DLYL:
        if(turn_start_update == 0)
        {
            turn_start_x = current_pose->x;
            turn_start_y = current_pose->y;
            turn_start_update = 1;
        }
        linear_velocity = -long_stra_vel;
        angular_velocity = 0;
        if (my_abs(turn_start_x - current_pose->x) > collision_backward_distance || my_abs(turn_start_y - current_pose->y) > collision_backward_distance)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            turn_start_update = 0;
            collision_right_rightrun_step_status = TURN_CCLOCK_TARGET_YAW_MORE_ABS15_LRUN_CR_DLYL;
            break;
        }
        break;
        
    case GOSTR_X_MORE_ONE_THIRD_LATERALDIS_BYPASS_LRUN_CR_DLYL:
        if (my_abs(Yaw) <=30)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            temporary_yaw = Yaw;
            collision_right_rightrun_step_status = TURN_CLOCK_TARGET_YAW_ABS30_LRUN_CR_DLYL;
            break;
        }
        else if (my_abs(Yaw) > 30 && Yaw > 0)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            temporary_yaw = Yaw;
            collision_right_rightrun_step_status = TURN_CLOCK_YAW_ADD_ABS30_LRUN_CR_DLYL;
            break;
        }
        else{
            collision_right_rightrun_step_status = GOSTR_BYPASS_LRUN_CR_DLYL;
            break;
        }
    case TURN_CLOCK_TARGET_YAW_ABS30_LRUN_CR_DLYL:
        if (my_abs(Yaw) > 30)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            collision_right_rightrun_step_status = GOSTR_BYPASS_LRUN_CR_DLYL;
            break;
        }
        if (obstacleSignal !=none_obstacle||(&cliff_valueB)->cliffValue0 == 1)
        {
            collision_right_rightrun_step_status = TURN_CLOCK_TARGET_YAW_ABS30_COLLISION_LRUN_CR_DLYL;
        }
        linear_velocity = 0;
        angular_velocity = -turn_vel;
        break;
    case TURN_CLOCK_TARGET_YAW_ABS30_COLLISION_LRUN_CR_DLYL:
        if(turn_start_update == 0)
        {
            turn_start_x = current_pose->x;
            turn_start_y = current_pose->y;
            turn_start_update = 1;
        }
        linear_velocity = -long_stra_vel;
        angular_velocity = 0;
        if (my_abs(turn_start_x - current_pose->x) > collision_backward_distance || my_abs(turn_start_y - current_pose->y) > collision_backward_distance)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            turn_start_update = 0;
            temporary_yaw = Yaw;
            if (my_abs(Yaw) < 15) collision_right_rightrun_step_status = TURN_CCLOCK_TARGET_YAW_LESS_ABS15_LRUN_CR_DLYL;
            else collision_right_rightrun_step_status = TURN_CCLOCK_TARGET_YAW_MORE_ABS15_LRUN_CR_DLYL;
            break;
        }
        break;
    case TURN_CLOCK_YAW_ADD_ABS30_LRUN_CR_DLYL:
        if (my_abs(Yaw - temporary_yaw) > 30)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            collision_right_rightrun_step_status = GOSTR_BYPASS_LRUN_CR_DLYL;
            break;
        }
        if (obstacleSignal !=none_obstacle||(&cliff_valueB)->cliffValue0 == 1)
        {
            collision_right_rightrun_step_status = TURN_CLOCK_YAW_ADD_ABS30_COLLISION_LRUN_CR_DLYL;
            break;
        }
        linear_velocity = 0;
        angular_velocity = -turn_vel;
        break;
    case TURN_CLOCK_YAW_ADD_ABS30_COLLISION_LRUN_CR_DLYL:
        if(turn_start_update == 0)
        {
            turn_start_x = current_pose->x;
            turn_start_y = current_pose->y;
            turn_start_update = 1;
        }
        linear_velocity = -long_stra_vel;
        angular_velocity = 0;
        if (my_abs(turn_start_x - current_pose->x) > collision_backward_distance || my_abs(turn_start_y - current_pose->y) > collision_backward_distance)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            turn_start_update = 0;
            temporary_yaw = Yaw;
            if (my_abs(Yaw) < 15) collision_right_rightrun_step_status = TURN_CCLOCK_TARGET_YAW_LESS_ABS15_LRUN_CR_DLYL;
            else collision_right_rightrun_step_status = TURN_CCLOCK_TARGET_YAW_MORE_ABS15_LRUN_CR_DLYL;
            break;
        }
        break;
    case TURN_CCLOCK_TARGET_YAW_MORE_AB173_LRUN_CR_DLYL:
        if (obstacleSignal !=none_obstacle||(&cliff_valueB)->cliffValue0 == 1)
        {
            collision_right_rightrun_step_status = TURN_CCLOCK_TARGET_YAW_MORE_AB173_COLLISION_LRUN_CR_DLYL;
            break;
        }
        if (my_abs(Yaw) > 173) 
        {
            collision_right_rightrun_step_status = COMPLETE_LRUN_CR_DLYL;
            break;
        }
        linear_velocity = 0;
        angular_velocity = turn_vel;
        break;
    case TURN_CCLOCK_TARGET_YAW_MORE_AB173_COLLISION_LRUN_CR_DLYL:
        if(turn_start_update == 0)
        {
            turn_start_x = current_pose->x;
            turn_start_y = current_pose->y;
            turn_start_update = 1;
        }
        linear_velocity = -long_stra_vel;
        angular_velocity = 0;
        if (my_abs(turn_start_x - current_pose->x) > collision_backward_distance || my_abs(turn_start_y - current_pose->y) > collision_backward_distance)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            collision_right_rightrun_step_status = COMPLETE_LRUN_CR_DLYL;
            turn_start_update = 0;
            break;
        }
        break;
    case GOSTR_BYPASS_BOW_CONTINUE_EXIT_LRUN_CR_DLYL:
        collision_right_rightrun_step_status = COMPLETE_LRUN_CR_DLYL;
        break;
    case COMPLETE_LRUN_CR_DLYL:
        collision_right_rightrun_step_status = 0;
        complete_flag = 1;
        break;
    }
    return complete_flag;
}


unsigned char  CollisionLeftLeftRunStep(POSE *current_pose,unsigned char obstacleSignal){
    int Yaw;
    unsigned char complete_flag=0;
    Yaw = current_pose->orientation;
    Yaw = Yaw /100;
    switch(collision_left_rightrun_step_status)
    {
    case 0:
        if(turn_start_update == 0)
        {
            if(obstacleSignal==none_obstacle){
                obstacleSignal=left_obstacle;
                last_position_x = current_pose->x;
                last_position_y = current_pose->y;
                if(my_abs(Yaw) > 90){
                    collision_left_rightrun_step_status = GOSTR_BYPASS_LOOP_LRUN_CL_DLYM;
                }
                else{
                    collision_left_rightrun_step_status = DIR_LEFT_YAW_LESS_ABS90_CLLRS;
                }
                infrared_collision=true;
                break;
            } 			
            turn_start_x = current_pose->x;
            turn_start_y = current_pose->y;
            turn_start_update = 1;
        }
        linear_velocity = -long_stra_vel;
        angular_velocity = 0;
        if (my_abs(turn_start_x - current_pose->x) > collision_backward_distance || my_abs(turn_start_y - current_pose->y) > collision_backward_distance)
        {
            collision_left_rightrun_step_status = GOBACK_DISTANCE_CLLRS;
            turn_start_update = 0;
            break;
        }
        break;
    case  GOBACK_DISTANCE_CLLRS:
        if(turn_start_update == 0)
        {
            turn_start_x = current_pose->x;
            turn_start_y = current_pose->y;
            turn_start_update = 1;
        }
        linear_velocity = -long_stra_vel;
        angular_velocity = 0;
        
        if (my_abs(turn_start_x - current_pose->x) > side_backward_distance||my_abs(turn_start_y - current_pose->y) > side_backward_distance)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            if (my_abs(Yaw) > 90)
            {
                collision_left_rightrun_step_status = DIR_LEFT_YAW_MORE_ABS90_CLLRS;
            }
            else
            {
                collision_left_rightrun_step_status = DIR_LEFT_YAW_LESS_ABS90_CLLRS;
            }
            turn_start_update = 0;
            break;
        }
        break;
    case  DIR_LEFT_YAW_MORE_ABS90_CLLRS:
        linear_velocity = 0;
        angular_velocity = 0;
        collision_left_rightrun_step_status = TURN_CLOCK_TARGET_YAW_ABS150_LRUN_CL_DLYM;
        break;
    case  TURN_CLOCK_TARGET_YAW_ABS150_LRUN_CL_DLYM:
        linear_velocity = 0;
        angular_velocity = -turn_vel;
        if (my_abs(Yaw) < 150)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            collision_left_rightrun_step_status = GOSTR_YAW_EQUAL_ABS150_LRUN_CL_DLYM;
            break;
        }
        if (obstacleSignal !=none_obstacle||(&cliff_valueB)->cliffValue0 == 1)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            collision_left_rightrun_step_status = TURN_CLOCK_TARGET_YAW_ABS150_COLLISION_LRUN_CL_DLYM;
            break;
        }
        break;
    case  TURN_CLOCK_TARGET_YAW_ABS150_COLLISION_LRUN_CL_DLYM:
        if(turn_start_update == 0)
        {
            turn_start_x = current_pose->x;
            turn_start_y = current_pose->y;
            turn_start_update = 1;
        }
        linear_velocity = -long_stra_vel;
        angular_velocity = 0;
        if (my_abs(turn_start_x - current_pose->x) > collision_backward_distance || my_abs(turn_start_y - current_pose->y) > collision_backward_distance)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            turn_start_update = 0;
            collision_left_rightrun_step_status = TURN_CLOCK_TARGET_YAW_ABS150_LRUN_CL_DLYM;
            break;
        }
        break;
    case  GOSTR_YAW_EQUAL_ABS150_LRUN_CL_DLYM:
        cnt_update = 0;
        last_position_y = current_pose->y;
        last_position_xx = current_pose->x;
        collision_left_rightrun_step_status = GOSTR_BYPASS_LRUN_CL_DLYM;
        break;
    case  GOSTR_BYPASS_LRUN_CL_DLYM:
        last_position_x = current_pose->x;
        cnt_update +=1;
        linear_velocity = long_stra_vel;
        angular_velocity = 0;
        if(cnt_update > 1&&my_abs(last_position_xx - current_pose->x)<20&&my_abs(last_position_y - current_pose->y)<20)
        {
            cnt_update = 0;
            last_position_y = current_pose->y;
            collision_left_rightrun_step_status = TURN_CCLOCK_TARGET_YAW_ABS90_LRUN_CL_DLYL;
            break;
        }
        collision_left_rightrun_step_status = GOSTR_BYPASS_LOOP_LRUN_CL_DLYM;
        break;
        
    case  TURN_CCLOCK_TARGET_YAW_ABS90_LRUN_CL_DLYL:
        linear_velocity = 0;
        angular_velocity = -turn_vel;
        if (my_abs(Yaw)<90)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            collision_left_rightrun_step_status = GOSTR_YAW_EQUAL_ABS90_LRUN_CL_DLYL;
            break;
        }
        if (obstacleSignal !=none_obstacle||(&cliff_valueB)->cliffValue0 == 1)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            collision_left_rightrun_step_status = TURN_CCLOCK_TAEGET_YAW_ABS90_COLLISION_LRUN_CL_DLYL;
            break;
        }
        break;
    case  TURN_CCLOCK_TAEGET_YAW_ABS90_COLLISION_LRUN_CL_DLYL:
        if(turn_start_update == 0)
        {
            turn_start_x = current_pose->x;
            turn_start_y = current_pose->y;
            turn_start_update = 1;
        }
        linear_velocity = -long_stra_vel;
        angular_velocity = 0;
        if (my_abs(turn_start_x - current_pose->x) > turn_backward_distance || my_abs(turn_start_y - current_pose->y) > turn_backward_distance)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            turn_start_update = 0;
            collision_left_rightrun_step_status = TURN_CCLOCK_TARGET_YAW_ABS90_LRUN_CL_DLYL;
            break;
        }
        break;
    case  GOSTR_YAW_EQUAL_ABS90_LRUN_CL_DLYL:
        linear_velocity = long_stra_vel;
        angular_velocity = 0;
        if (obstacleSignal !=none_obstacle||(&cliff_valueB)->cliffValue0 == 1)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            left_running_step_status = LEFTREVERSEWALKEDGE;
            collision_left_rightrun_step_status =0;
            break;
        }
        if (my_abs(last_position_y - current_pose->y) > lateral_move_distance / 2)
        {
            collision_left_rightrun_step_status = MORE_TRY_BREAK_BYPASS_LRUN_CL_DLYM;
            break;
        }
        break;
        
    case  GOSTR_BYPASS_LOOP_LRUN_CL_DLYM:
        if (obstacleSignal == left_obstacle)
        {
			infrared_collision=false;
			last_position_x = current_pose->x;
            collision_left_rightrun_step_status = LEFT_COLLISION_BYPASS_LRUN_CL_DLYM;
            break;
        }
        if (my_abs(last_position_y - current_pose->y) > close_edge)
        {
            collision_left_rightrun_step_status = GOSTR_BYPASS_BOW_CONTINUE_LRUN_CL_DLYM;
            break;
        }
        if(obstacleSignal == front_obstacle || obstacleSignal == right_obstacle||(&cliff_valueB)->cliffValue0 == 1){
            last_position_y = current_pose->y;
            collision_left_rightrun_step_status = TURN_CCLOCK_TARGET_YAW_ABS90_LRUN_CL_DLYL;
            break;
        }
        if (last_position_y > current_pose->y+1)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            collision_left_rightrun_step_status = GOSTR_BYPASS_OLD_BOW_CONTINUE_LRUN_CL_DLYM;
            break;
        }
        if (my_abs(current_pose->x) > half_map_wide-2*GRIDWIDTH)
        {
            collision_left_rightrun_step_status = COMPLETE_LRUN_CL_DLYM;
            break;
        }
        if (my_abs(Yaw) < 90)
        {
            collision_left_rightrun_step_status = TURN_CCLOCK_TARGET_YAW_LESS_ABS3_LRUN_CL_DLYM;
            break;
        }
        if(infrared_collision==true&&my_abs(Yaw)>105){
            if(adcRealTime[0]>500|| adcRealTime[1]>50||adcRealTime[2]>50){
                linear_velocity = 50;
                angular_velocity = -57;
                if(adcRealTime[2]>50){
                    linear_velocity = 50;
                    angular_velocity = -turn_vel;
                }
            }
            else{
				left_edge_judgment_repeat();
//                if(adcRealTime[8]>100&&adcRealTime[8]<1500){
//                    delimma_edge=0;
//                    linear_velocity = 200;
//                    angular_velocity = 0;
//                }
//                if(adcRealTime[8]>=1500){
//                    delimma_edge=0;
//                    linear_velocity = 200;
//                    angular_velocity = -10;
//                }
//                if(adcRealTime[8]<100){
//                    if(delimma_edge<10){
//                        delimma_edge++;
//                        linear_velocity = 200;
//                        angular_velocity = 10;
//                    }else{
//                        linear_velocity = 100;
//                        angular_velocity = 20;
//                    }
//                }
            }
        }
        else{
            if (my_abs(last_position_x - current_pose->x) > lateral_move_distance / 3)
            {
                infrared_collision=true;
				last_position_x=current_pose->x;
                collision_left_rightrun_step_status = GOSTR_X_MORE_ONE_THIRD_LATERALDIS_BYPASS_LRUN_CL_DLYM;
                break;
            }    
            linear_velocity = long_stra_vel;
            angular_velocity = 0;
        }
        break;
        
    case  GOSTR_BYPASS_OLD_BOW_CONTINUE_LRUN_CL_DLYM:
        if (obstacleSignal !=none_obstacle||(&cliff_valueB)->cliffValue0 == 1)
        {
            collision_left_rightrun_step_status = GOSTR_BYPASS_OLD_BOW_CONTINUE_COLLISION_LRUN_CL_DLYM;
            break;
        }
        if (my_abs(Yaw) > 172 )
        {
            bow_continue = true;
            collision_left_rightrun_step_status = COMPLETE_LRUN_CL_DLYM;
            break;
        }
        linear_velocity = 0;
        angular_velocity = -turn_vel;
        break;
    case  GOSTR_BYPASS_OLD_BOW_CONTINUE_COLLISION_LRUN_CL_DLYM:
        if(turn_start_update == 0)
        {
            turn_start_x = current_pose->x;
            turn_start_y = current_pose->y;
            turn_start_update = 1;
        }
        linear_velocity = -long_stra_vel;
        angular_velocity = 0;
        if (my_abs(turn_start_x - current_pose->x) > collision_backward_distance ||my_abs(turn_start_y - current_pose->y) > collision_backward_distance)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            bow_continue = true;
            collision_left_rightrun_step_status = GOSTR_BYPASS_OLD_BOW_CONTINUE_LRUN_CL_DLYM;
            turn_start_update = 0;
            break;
        }
        break;
    case  TURN_CCLOCK_TARGET_YAW_LESS_ABS3_LRUN_CL_DLYM:
        if (obstacleSignal !=none_obstacle||(&cliff_valueB)->cliffValue0 == 1)
        {
            collision_left_rightrun_step_status = TURN_CCLOCK_TARGET_YAW_LESS_ABS3_COLLISION_LRUN_CL_DLYM;
            break;
        }
        if (my_abs(Yaw) < 8 )
        {
            bow_continue = true;
            collision_left_rightrun_step_status = COMPLETE_LRUN_CL_DLYM;
            break;
        }
        linear_velocity = 0;
        angular_velocity = -turn_vel;
        break;
    case  TURN_CCLOCK_TARGET_YAW_LESS_ABS3_COLLISION_LRUN_CL_DLYM:
        if(turn_start_update == 0)
        {
            turn_start_x = current_pose->x;
            turn_start_y = current_pose->y;
            turn_start_update = 1;
        }
        linear_velocity = -long_stra_vel;
        angular_velocity = 0;
        if (my_abs(turn_start_x - current_pose->x) > collision_backward_distance ||my_abs(turn_start_y - current_pose->y) > collision_backward_distance)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            bow_continue = true;
            collision_left_rightrun_step_status = TURN_CCLOCK_TARGET_YAW_LESS_ABS3_LRUN_CL_DLYM;
            turn_start_update = 0;
            break;
        }
        break;
    case  GOSTR_BYPASS_BOW_CONTINUE_LRUN_CL_DLYM:
        if (obstacleSignal !=none_obstacle||(&cliff_valueB)->cliffValue0 == 1)
        {
            collision_left_rightrun_step_status = GOSTR_BYPASS_BOW_CONTINUE_COLLISION_LRUN_CL_DLYM;
            break;
        }
        if (my_abs(Yaw) < 8 )
        {
            bow_continue = true;
            collision_left_rightrun_step_status = GOSTR_BYPASS_BOW_CONTINUE_EXIT_LRUN_CL_DLYM;
            break;
        }
        linear_velocity = 0;
        angular_velocity = -turn_vel;
        break;
    case  GOSTR_BYPASS_BOW_CONTINUE_COLLISION_LRUN_CL_DLYM:
        if(turn_start_update == 0)
        {
            turn_start_x = current_pose->x;
            turn_start_y = current_pose->y;
            turn_start_update = 1;
        }
        linear_velocity = -long_stra_vel;
        angular_velocity = 0;
        if (my_abs(turn_start_x - current_pose->x) > collision_backward_distance ||my_abs(turn_start_y - current_pose->y) > collision_backward_distance)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            bow_continue = true;
            collision_left_rightrun_step_status = GOSTR_BYPASS_BOW_CONTINUE_LRUN_CL_DLYM;
            turn_start_update = 0;
            break;
        }
        break;
    case  GOSTR_BYPASS_BOW_CONTINUE_EXIT_LRUN_CL_DLYM:
        collision_left_rightrun_step_status = COMPLETE_LRUN_CL_DLYM;
        break;
    case  LEFT_COLLISION_BYPASS_LRUN_CL_DLYM:
        if(turn_start_update == 0)
        {
            turn_start_x = current_pose->x;
            turn_start_y = current_pose->y;
            turn_start_update = 1;
        }
        linear_velocity = -long_stra_vel;
        angular_velocity = 0;
        
        if (my_abs(turn_start_x - current_pose->x) > collision_backward_distance || my_abs(turn_start_y - current_pose->y) > collision_backward_distance)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            turn_start_update = 0;
            temporary_yaw = Yaw;
            if (my_abs(Yaw) > 150) collision_left_rightrun_step_status = TURN_CLCOK_TARGET_YAW_MORE_ABS150_LRUN_CL_DLYM;
            else collision_left_rightrun_step_status = TURN_CLCOK_TARGET_YAW_LESS_ABS150_LRUN_CL_DLYM;
            break;
        }
        break;
    case  TURN_CLCOK_TARGET_YAW_MORE_ABS150_LRUN_CL_DLYM:
        if (obstacleSignal !=none_obstacle||(&cliff_valueB)->cliffValue0 == 1)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            collision_left_rightrun_step_status = TURN_CLCOK_TARGET_YAW_MORE_ABS150_COLLISION_LRUN_CL_DLYM;
            break;
        }
        if (my_abs(Yaw) < 150)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            collision_left_rightrun_step_status = GOSTR_BYPASS_LRUN_CL_DLYM;
            break;
        }
		if (my_abs(Yaw) < 90)
        {
            collision_left_rightrun_step_status = TURN_CCLOCK_TARGET_YAW_LESS_ABS3_LRUN_CL_DLYM;
            break;
        }
        linear_velocity = 0;
        angular_velocity = -turn_vel;
        break;
    case TURN_CLCOK_TARGET_YAW_MORE_ABS150_COLLISION_LRUN_CL_DLYM:
        if(turn_start_update == 0)
        {
            turn_start_x = current_pose->x;
            turn_start_y = current_pose->y;
            turn_start_update = 1;
        }
        linear_velocity = -long_stra_vel;
        angular_velocity = 0;
        
        if (my_abs(turn_start_x - current_pose->x) > collision_backward_distance || my_abs(turn_start_y - current_pose->y) > collision_backward_distance)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            turn_start_update = 0;
            collision_left_rightrun_step_status = TURN_CLCOK_TARGET_YAW_MORE_ABS150_LRUN_CL_DLYM;
            break;
        }
        break;
        
    case  TURN_CLCOK_TARGET_YAW_LESS_ABS150_LRUN_CL_DLYM:
        if (obstacleSignal !=none_obstacle||(&cliff_valueB)->cliffValue0 == 1)
        {
            collision_left_rightrun_step_status = TURN_CLCOK_TARGET_YAW_LESS_ABS150_COLLISION_LRUN_CL_DLYM;
            break;
        }
        if (my_abs(Yaw - temporary_yaw) > 15)
        {
            collision_left_rightrun_step_status = GOSTR_BYPASS_LRUN_CL_DLYM;
            break;
        }
		if (my_abs(Yaw) < 90)
        {
            collision_left_rightrun_step_status = TURN_CCLOCK_TARGET_YAW_LESS_ABS3_LRUN_CL_DLYM;
            break;
        }
        linear_velocity = 0;
        angular_velocity = -turn_vel;
        break;
        
    case TURN_CLCOK_TARGET_YAW_LESS_ABS150_COLLISION_LRUN_CL_DLYM:
        if(turn_start_update == 0)
        {
            turn_start_x = current_pose->x;
            turn_start_y = current_pose->y;
            turn_start_update = 1;
        }
        linear_velocity = -long_stra_vel;
        angular_velocity = 0;
        
        if (my_abs(turn_start_x - current_pose->x) > collision_backward_distance || my_abs(turn_start_y - current_pose->y) > collision_backward_distance)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            turn_start_update = 0;
            collision_left_rightrun_step_status = TURN_CLCOK_TARGET_YAW_LESS_ABS150_LRUN_CL_DLYM;
            break;
        }
        break;
    case  GOSTR_X_MORE_ONE_THIRD_LATERALDIS_BYPASS_LRUN_CL_DLYM:
        if (my_abs(Yaw) > 150 || Yaw > 0)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            if(Yaw>0&&my_abs(Yaw) > 150){
                turn_start_update=1;
            }
            temporary_yaw = Yaw;
            collision_left_rightrun_step_status = TURN_CCLOCK_YAW_ADD_ABS30_LRUN_CL_DLYM;
            break;
        }
        collision_left_rightrun_step_status = GOSTR_BYPASS_LRUN_CL_DLYM;
        break;
    case  TURN_CCLOCK_YAW_ADD_ABS30_LRUN_CL_DLYM:
        if(turn_start_update==1){
            if(my_abs(Yaw)<150){
                turn_start_update=0;
                linear_velocity = 0;
                angular_velocity = 0;
                collision_left_rightrun_step_status = GOSTR_BYPASS_LRUN_CL_DLYM;
                break;
            }
        }
        else{
            if (my_abs(Yaw - temporary_yaw) > 30)
            {
                linear_velocity = 0;
                angular_velocity = 0;
                collision_left_rightrun_step_status = GOSTR_BYPASS_LRUN_CL_DLYM;
                break;
            }
        }
        if (obstacleSignal !=none_obstacle||(&cliff_valueB)->cliffValue0 == 1)
        {
            collision_left_rightrun_step_status = TURN_CCLOCK_YAW_ADD_ABS30_COLLISION_LRUN_CL_DLYM;
        }
        linear_velocity = 0;
        angular_velocity = turn_vel;
        break;
    case  TURN_CCLOCK_YAW_ADD_ABS30_COLLISION_LRUN_CL_DLYM:
        if(turn_start_update == 0)
        {
            turn_start_x = current_pose->x;
            turn_start_y = current_pose->y;
            turn_start_update = 1;
        }
        linear_velocity = -long_stra_vel;
        angular_velocity = 0;
        if (my_abs(turn_start_x - current_pose->x) > collision_backward_distance || my_abs(turn_start_y - current_pose->y) > collision_backward_distance)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            turn_start_update = 0;
            temporary_yaw = Yaw;
            if (my_abs(Yaw) > 150) collision_left_rightrun_step_status = TURN_CLCOK_TARGET_YAW_MORE_ABS150_LRUN_CL_DLYM;
            else collision_left_rightrun_step_status = TURN_CLCOK_TARGET_YAW_LESS_ABS150_LRUN_CL_DLYM;
            break;
        }
        break;
    case  MORE_TRY_BREAK_BYPASS_LRUN_CL_DLYM:
        if (obstacleSignal !=none_obstacle||(&cliff_valueB)->cliffValue0 == 1)
        {
            collision_left_rightrun_step_status = MORE_TRY_BREAK_BYPASS_COLLISION_LRUN_CL_DLYM;
            bow_continue = true;
            break;
        }
        if (my_abs(Yaw) < 8)
        {
            bow_continue = true;
            collision_left_rightrun_step_status = GOSTR_BYPASS_BOW_CONTINUE_EXIT_LRUN_CL_DLYM;
            break;
        }
        linear_velocity = 0;
        angular_velocity = -turn_vel;
        break;
    case MORE_TRY_BREAK_BYPASS_COLLISION_LRUN_CL_DLYM:
        if(turn_start_update == 0)
        {
            turn_start_x = current_pose->x;
            turn_start_y = current_pose->y;
            turn_start_update = 1;
        }
        linear_velocity = -long_stra_vel;
        angular_velocity = 0;
        if (my_abs(turn_start_x - current_pose->x) > collision_backward_distance || my_abs(turn_start_y - current_pose->y) > collision_backward_distance)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            collision_left_rightrun_step_status = MORE_TRY_BREAK_BYPASS_LRUN_CL_DLYM;
            turn_start_update = 0;
            break;
        }
        break;
        
    case  COMPLETE_LRUN_CL_DLYM:
        collision_left_rightrun_step_status = 0;
        complete_flag = 1;
        break;
    case  DIR_LEFT_YAW_LESS_ABS90_CLLRS:
        linear_velocity = 0;
        angular_velocity = 0;
        last_position_y = current_pose->y;
        collision_left_rightrun_step_status  = TURN_CCLOCK_TARGET_YAW_ABS57_LRUN_CL_DLYL;
        break;
        
    case  TURN_CCLOCK_TARGET_YAW_ABS57_LRUN_CL_DLYL:
        linear_velocity = 0;
        angular_velocity = turn_vel;
        if (Yaw > 85)//90
        {
            linear_velocity = 0;
            angular_velocity = 0;
            collision_left_rightrun_step_status  = GOSTR_YAW_EQUAL_ABS57_LRUN_CL_DLYL;
            break;
        }
        if (obstacleSignal !=none_obstacle||(&cliff_valueB)->cliffValue0 == 1)
        {
            collision_left_rightrun_step_status  = TURN_CCLOCK_TARGET_YAW_ABS57_COLLISION_LRUN_CL_DLYL;
            break;
        }
        break;
    case  TURN_CCLOCK_TARGET_YAW_ABS57_COLLISION_LRUN_CL_DLYL:
        if(turn_start_update == 0)
        {
            turn_start_x = current_pose->x;
            turn_start_y = current_pose->y;
            turn_start_update = 1;
        }
        linear_velocity = -long_stra_vel;
        angular_velocity = 0;
        if (my_abs(turn_start_x - current_pose->x) > second_turn_backward_distance || my_abs(turn_start_y - current_pose->y) > second_turn_backward_distance)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            turn_start_update = 0;
            collision_left_rightrun_step_status = TURN_CCLOCK_TARGET_YAW_ABS57_LRUN_CL_DLYL;
            break;
        }
        break;
    case  GOSTR_YAW_EQUAL_ABS57_LRUN_CL_DLYL:
        if (obstacleSignal !=none_obstacle)
        {
			linear_velocity = 0;
            angular_velocity = 0;
            collision_left_rightrun_step_status = LEFT_WALK_EDGE_LRUN_CL_DLYL;
            break;	
        }
        if (my_abs(last_position_y - current_pose->y) > lateral_move_distance)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            collision_left_rightrun_step_status = TURN_CCLOCK_TARGET_YAW_ABS87_LRUN_CL_DLYL;
            break;
        }
        linear_velocity = long_stra_vel;
        angular_velocity = 0;
        break;
    case  TURN_CCLOCK_TARGET_YAW_ABS87_LRUN_CL_DLYL:
        linear_velocity = 0;
        angular_velocity = turn_vel;
        if (Yaw > 90)//95
        {
            linear_velocity = 0;
            angular_velocity = 0;
            collision_left_rightrun_step_status = GOSTR_YAW_EQUAL_ABS87_LRUN_CL_DLYL;
            break;
        }
        if (obstacleSignal !=none_obstacle||(&cliff_valueB)->cliffValue0 == 1)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            collision_left_rightrun_step_status = TURN_CCLOCK_TARGET_YAW_ABS87_COLLISION_LRUN_CL_DLYL;
            break;
        }
        break;
    case  TURN_CCLOCK_TARGET_YAW_ABS87_COLLISION_LRUN_CL_DLYL:
        if(turn_start_update == 0)
        {
            turn_start_x = current_pose->x;
            turn_start_y = current_pose->y;
            turn_start_update = 1;
        }
        linear_velocity = -long_stra_vel;
        angular_velocity = 0;
        if (my_abs(turn_start_x - current_pose->x) > turn_backward_distance || my_abs(turn_start_y - current_pose->y) > turn_backward_distance)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            turn_start_update = 0;
            collision_left_rightrun_step_status = TURN_CCLOCK_TARGET_YAW_ABS87_LRUN_CL_DLYL;
            break;
        }
        break;
    case  GOSTR_YAW_EQUAL_ABS87_LRUN_CL_DLYL:
        linear_velocity = long_stra_vel;
        angular_velocity = 0;
        if (obstacleSignal !=none_obstacle||(&cliff_valueB)->cliffValue0 == 1)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            collision_left_rightrun_step_status = LEFT_WALK_EDGE_LRUN_CL_DLYL;
            break;
        }
        if (my_abs(last_position_y - current_pose->y) > lateral_move_distance)
        {
            collision_left_rightrun_step_status = TURN_CCLOCK_TARGET_YAW_ABS173_LRUN_CL_DLYL;
            break;
        }
        break;
    case  LEFT_WALK_EDGE_LRUN_CL_DLYL:
        collision_left_rightrun_step_status = 0;
        complete_flag = 2;
        break;
    case  TURN_CCLOCK_TARGET_YAW_ABS173_LRUN_CL_DLYL:
        linear_velocity = 0;
        angular_velocity = turn_vel;
        if (my_abs(Yaw)>176) //173
        {
            collision_left_rightrun_step_status = COMPLETE_LRUN_CL_DLYL;
            break;
        }
        if (obstacleSignal!=none_obstacle)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            collision_left_rightrun_step_status = TURN_CCLOCK_TARGET_YAW_ABS173_COLLISION_LRUN_CL_DLYL;
            break;
        }
        break;
    case  TURN_CCLOCK_TARGET_YAW_ABS173_COLLISION_LRUN_CL_DLYL:
        if(turn_start_update == 0)
        {
            turn_start_x = current_pose->x;
            turn_start_y = current_pose->y;
            turn_start_update = 1;
        }
        linear_velocity = -long_stra_vel;
        angular_velocity = 0;
        if (my_abs(turn_start_x - current_pose->x) > collision_backward_distance || my_abs(turn_start_y - current_pose->y) > collision_backward_distance)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            turn_start_update = 0;
            collision_left_rightrun_step_status = TURN_CCLOCK_TARGET_YAW_ABS173_LRUN_CL_DLYL;
            break;
        }
        break;
    case  COMPLETE_LRUN_CL_DLYL:
        collision_left_rightrun_step_status = 0;
        complete_flag = 1;
        break;
    }
    return complete_flag;
}



unsigned char  CollisionFrontLeftRunStep(POSE *current_pose, unsigned char obstacleSignal){
    int Yaw;
    unsigned char complete_flag = 0;
    Yaw = current_pose->orientation;
    Yaw = Yaw /100;
    switch (collision_front_rightrun_step_status)
    {
    case 0:
        if (turn_start_update == 0)
        {
            if(obstacleSignal==none_obstacle){
				obstacleSignal=front_obstacle;
                if (my_abs(Yaw) < 90)
                {
                    collision_front_rightrun_step_status = DIR_LEFT_YAW_LESS_ABS90_CFRLS;
                }
                else
                {
                    collision_front_rightrun_step_status = DIR_LEFT_YAW_MORE_ABS90_CFRLS;
                }
                break;
            }
            turn_start_x = current_pose->x;
            turn_start_y = current_pose->y;
            turn_start_update = 1;
        }
        linear_velocity = -long_stra_vel;
        angular_velocity = 0;
        if (my_abs(turn_start_x - current_pose->x) > collision_backward_distance || my_abs(turn_start_y - current_pose->y) > collision_backward_distance)
        {
            collision_front_rightrun_step_status = GOBACK_DISTANCE_CFLRS;
            turn_start_update = 0;
            break;
        }
        break;
    case GOBACK_DISTANCE_CFLRS:
        if (turn_start_update == 0)
        {
            turn_start_x = current_pose->x;
            turn_start_y = current_pose->y;
            turn_start_update = 1;
        }
        linear_velocity = -long_stra_vel;
        angular_velocity = 0;
        
        if (my_abs(turn_start_x - current_pose->x) > front_backward_distance)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            if (my_abs(Yaw) < 90)
            {
                collision_front_rightrun_step_status = DIR_LEFT_YAW_LESS_ABS90_CFRLS;
            }
            else
            {
                collision_front_rightrun_step_status = DIR_LEFT_YAW_MORE_ABS90_CFRLS;
            }
            turn_start_update = 0;
            break;
        }
        break;
    case DIR_LEFT_YAW_LESS_ABS90_CFRLS:
        linear_velocity = 0;
        angular_velocity = 0;
        last_position_y = current_pose->y;
        collision_front_rightrun_step_status = TURN_CCLOCK_TARGET_YAW_ABS60_LRUN_CF_DLYL;
        break;
    case TURN_CCLOCK_TARGET_YAW_ABS60_LRUN_CF_DLYL:
        linear_velocity = 0;
        angular_velocity = turn_vel;
        if (Yaw>85)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            collision_front_rightrun_step_status = GOSTR_YAW_ABS60_LRUN_CF_DLYL;
            break;
        }
        if (obstacleSignal != none_obstacle||(&cliff_valueB)->cliffValue0 == 1)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            collision_front_rightrun_step_status = TURN_CCLOCK_TARGET_YAW_ABS60_COLLISION_LRUN_CF_DLYL;
            break;
        }
        break;
    case TURN_CCLOCK_TARGET_YAW_ABS60_COLLISION_LRUN_CF_DLYL:
        if (turn_start_update == 0)
        {
            turn_start_x = current_pose->x;
            turn_start_y = current_pose->y;
            turn_start_update = 1;
        }
        linear_velocity = -long_stra_vel;
        angular_velocity = 0;
        if (my_abs(turn_start_x - current_pose->x) > collision_backward_distance || my_abs(turn_start_y - current_pose->y) > collision_backward_distance)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            turn_start_update = 0;
            collision_front_rightrun_step_status = TURN_CCLOCK_TARGET_YAW_ABS60_LRUN_CF_DLYL;
            break;
        }
        break;
    case GOSTR_YAW_ABS60_LRUN_CF_DLYL:
        linear_velocity = long_stra_vel;
        angular_velocity = 0;
        if (obstacleSignal != none_obstacle||(&cliff_valueB)->cliffValue0 == 1)
        {
			linear_velocity = 0;
            angular_velocity = 0;
            collision_front_rightrun_step_status = LEFT_WALK_EDGE_LRUN_CF_DLYL;
            break;			
        }
        if (my_abs(last_position_y - current_pose->y) > lateral_move_distance)
        {
            collision_front_rightrun_step_status = TURN_CCLOCK_TARGET_YAW_ABS82_LRUN_CF_DLYL;
            break;
        }
        break;
    case TURN_CCLOCK_TARGET_YAW_ABS82_LRUN_CF_DLYL:
        linear_velocity = 0;
        angular_velocity = turn_vel;
        if (Yaw > 90)
        {
            collision_front_rightrun_step_status = GOSTR_YAW_ABS82_LRUN_CF_DLYL;
            break;
        }
        if (obstacleSignal != none_obstacle||(&cliff_valueB)->cliffValue0 == 1)
        {
            collision_front_rightrun_step_status = TURN_CCLOCK_TARGET_YAW_ABS82_COLLISION_LRUN_CF_DLYL;
            break;
        }
        break;
    case TURN_CCLOCK_TARGET_YAW_ABS82_COLLISION_LRUN_CF_DLYL:
        if (turn_start_update == 0)
        {
            turn_start_x = current_pose->x;
            turn_start_y = current_pose->y;
            turn_start_update = 1;
        }
        linear_velocity = -long_stra_vel;
        angular_velocity = 0;
        if (my_abs(turn_start_x - current_pose->x) > collision_backward_distance || my_abs(turn_start_y - current_pose->y) > collision_backward_distance)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            turn_start_update = 0;
            collision_front_rightrun_step_status = TURN_CCLOCK_TARGET_YAW_ABS82_LRUN_CF_DLYL;
            break;
        }
        break;
    case GOSTR_YAW_ABS82_LRUN_CF_DLYL:
        linear_velocity = long_stra_vel;
        angular_velocity = 0;
        if (obstacleSignal != none_obstacle||(&cliff_valueB)->cliffValue0 == 1)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            collision_front_rightrun_step_status = LEFT_WALK_EDGE_LRUN_CF_DLYL;
            break;
        }
        if (my_abs(last_position_y - current_pose->y) > (lateral_move_distance))
        {
            collision_front_rightrun_step_status = TURN_CCLOCK_TARGET_YAW_ABS173_LRUN_CF_DLYL;
            break;
        }
        break;
    case LEFT_WALK_EDGE_LRUN_CF_DLYL:
        collision_front_rightrun_step_status = 0;
        complete_flag = 2;
        break;
    case TURN_CCLOCK_TARGET_YAW_ABS173_LRUN_CF_DLYL:
        linear_velocity = 0;
        angular_velocity = turn_vel;
        if (my_abs(Yaw) > 176) //173
        {
            collision_front_rightrun_step_status = COMPLETE_LRUN_CF_DLYL;
            break;
        }
        if (obstacleSignal != none_obstacle||(&cliff_valueB)->cliffValue0 == 1)
        {
            collision_front_rightrun_step_status = TURN_CCLOCK_TARGET_YAW_ABS173_COLLISION_LRUN_CF_DLYL;
            break;
        }
        break;
    case TURN_CCLOCK_TARGET_YAW_ABS173_COLLISION_LRUN_CF_DLYL:
        if (turn_start_update == 0)
        {
            turn_start_x = current_pose->x;
            turn_start_y = current_pose->y;
            turn_start_update = 1;
        }
        linear_velocity = -long_stra_vel;
        angular_velocity = 0;
        if (my_abs(turn_start_x - current_pose->x) > collision_backward_distance || my_abs(turn_start_y - current_pose->y) > collision_backward_distance)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            turn_start_update = 0;
            collision_front_rightrun_step_status = TURN_CCLOCK_TARGET_YAW_ABS173_LRUN_CF_DLYL;
            break;
        }
        break;
    case COMPLETE_LRUN_CF_DLYL:
        collision_front_rightrun_step_status = 0;
        complete_flag = 1;
        break;
    case DIR_LEFT_YAW_MORE_ABS90_CFRLS:
        linear_velocity = 0;
        angular_velocity = 0;
        last_position_y = current_pose->y;
        collision_front_rightrun_step_status = TURN_CLOCK_TARGET_YAW_ABS120_LRUN_CF_DLYM;
        break;
    case TURN_CLOCK_TARGET_YAW_ABS120_LRUN_CF_DLYM:
        linear_velocity = 0;
        angular_velocity = -turn_vel;
        if (my_abs(Yaw) < 90)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            collision_front_rightrun_step_status = GOSTR_YAW_EQUAL_ABS120_LRUN_CF_CF_DLYM;
            break;
        }
        if (obstacleSignal != none_obstacle||(&cliff_valueB)->cliffValue0 == 1)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            collision_front_rightrun_step_status = TURN_CLOCK_TARGET_YAW_ABS120_COLLISION_LRUN_CF_DLYM;
            break;
        }
        break;
    case TURN_CLOCK_TARGET_YAW_ABS120_COLLISION_LRUN_CF_DLYM:
        if (turn_start_update == 0)
        {
            turn_start_x = current_pose->x;
            turn_start_y = current_pose->y;
            turn_start_update = 1;
        }
        linear_velocity = -long_stra_vel;
        angular_velocity = 0;
        if (my_abs(turn_start_x - current_pose->x) > collision_backward_distance || my_abs(turn_start_y - current_pose->y) > collision_backward_distance)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            turn_start_update = 0;
            collision_front_rightrun_step_status = TURN_CLOCK_TARGET_YAW_ABS120_LRUN_CF_DLYM;
            break;
        }
        break;
    case GOSTR_YAW_EQUAL_ABS120_LRUN_CF_CF_DLYM:
        linear_velocity = long_stra_vel;
        angular_velocity = 0;
        if (obstacleSignal != none_obstacle||(&cliff_valueB)->cliffValue0 == 1)
        {
			linear_velocity = 0;
            angular_velocity = 0;
            collision_front_rightrun_step_status = LEFT_REVERSE_WALK_EDGE_LRUN_CF_DLYM;
            break;		
        }
        if (my_abs(last_position_y - current_pose->y) > lateral_move_distance)
        {
            collision_front_rightrun_step_status = TURN_CLOCK_TARGET_YAW_ABS98_LRUN_CF_DLYM;
            break;
        }
        break;
    case TURN_CLOCK_TARGET_YAW_ABS98_LRUN_CF_DLYM:
        linear_velocity = 0;
        angular_velocity = -turn_vel;
        if (my_abs(Yaw) < 85)
        {
            collision_front_rightrun_step_status = GOSTR_YAW_EQUAL_ABS98_LRUN_CF_DLYM;
            break;
        }
        if (obstacleSignal != none_obstacle||(&cliff_valueB)->cliffValue0 == 1)
        {
            collision_front_rightrun_step_status = TURN_CLOCK_TARGET_YAW_ABS98_COLLISION_LRUN_CF_DLYM;
            break;
        }
        break;
    case TURN_CLOCK_TARGET_YAW_ABS98_COLLISION_LRUN_CF_DLYM:
        if (turn_start_update == 0)
        {
            turn_start_x = current_pose->x;
            turn_start_y = current_pose->y;
            turn_start_update = 1;
        }
        linear_velocity = -long_stra_vel;
        angular_velocity = 0;
        if (my_abs(turn_start_x - current_pose->x) > collision_backward_distance || my_abs(turn_start_y - current_pose->y) > collision_backward_distance)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            turn_start_update = 0;
            collision_front_rightrun_step_status = TURN_CLOCK_TARGET_YAW_ABS98_LRUN_CF_DLYM;
            break;
        }
        break;
    case GOSTR_YAW_EQUAL_ABS98_LRUN_CF_DLYM:
        linear_velocity = long_stra_vel;
        angular_velocity = 0;
        if (obstacleSignal != none_obstacle||(&cliff_valueB)->cliffValue0 == 1)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            collision_front_rightrun_step_status = LEFT_REVERSE_WALK_EDGE_LRUN_CF_DLYM;
            break;
        }
        if (my_abs(last_position_y - current_pose->y) > (lateral_move_distance))
        {
            collision_front_rightrun_step_status = TURN_CLOCK_TARGET_YAW_ABS8_LRUN_CF_DLYM;
            break;
        }
        break;
    case LEFT_REVERSE_WALK_EDGE_LRUN_CF_DLYM:
        collision_front_rightrun_step_status = 0;
        complete_flag = 3;
        break;
    case TURN_CLOCK_TARGET_YAW_ABS8_LRUN_CF_DLYM:
        linear_velocity = 0;
        angular_velocity = -turn_vel;
        if (my_abs(Yaw) < 5) //8
        {
            collision_front_rightrun_step_status = COMPLETE_LRUN_CF_DLYM;
            break;
        }
        if (obstacleSignal != none_obstacle||(&cliff_valueB)->cliffValue0 == 1)
        {
            collision_front_rightrun_step_status = TURN_CLOCK_TARGET_YAW_ABS8_COLLISION_LRUN_CF_DLYM;
            break;
        }        
        break;
    case TURN_CLOCK_TARGET_YAW_ABS8_COLLISION_LRUN_CF_DLYM:
        if (turn_start_update == 0)
        {
            turn_start_x = current_pose->x;
            turn_start_y = current_pose->y;
            turn_start_update = 1;
        }
        linear_velocity = -long_stra_vel;
        angular_velocity = 0;
        if (my_abs(turn_start_x - current_pose->x) > collision_backward_distance || my_abs(turn_start_y - current_pose->y) > collision_backward_distance)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            turn_start_update = 0;
            collision_front_rightrun_step_status = TURN_CLOCK_TARGET_YAW_ABS8_LRUN_CF_DLYM;
            break;
        }
        break;
    case COMPLETE_LRUN_CF_DLYM:
        collision_front_rightrun_step_status = 0;
        complete_flag = 1;
        break;
    }
    return complete_flag;
}



unsigned char  LeftWalkEdge(POSE *current_pose,unsigned char obstacleSignal){
    int Yaw;
    unsigned char complete_flag=0;
    signed char i,j;
    Yaw = current_pose->orientation;
    Yaw = Yaw /100;
    if (my_abs(current_pose->x)>half_map_wide-2*GRIDWIDTH||my_abs(current_pose->y)>half_map_wide-2*GRIDWIDTH){
        linear_velocity = 0;
        angular_velocity = 0;
        if(my_abs(leakingsweep_x-current_pose->x)>500&&my_abs(leakingsweep_y-current_pose->y)>100){
			if((my_abs(current_pose->x)<half_map_wide-GRIDHEIGHT)&&(my_abs(current_pose->y)<half_map_wide-GRIDHEIGHT)){
            leakingsweep_x=current_pose->x;
            leakingsweep_y=current_pose->y;
            leftboolleaksweep=true;
            return complete_flag;
			}
        }
        if (0 != leakingsweep)
        {
            right_walk_edge_status = 0;
            returnorigin=false;
            left_running_step_status = LEAKING_SWEEP_LEFTRUN_STEP;
            return complete_flag;
        }
        else
        {
            right_walk_edge_status = 0;
            returnorigin=false;
            complete_flag = 3;
            return complete_flag;
        }
    }
    switch(right_walk_edge_status)
    {
    case 0:
        right_walk_edge_status = LEFT_GOBACK_WALK_EDGE;
        break;
    case LEFT_GOBACK_WALK_EDGE:
        if(turn_start_update == 0)
        {
            turn_start_x = current_pose->x;
            turn_start_y = current_pose->y;
            turn_start_update = 1;
        }
        linear_velocity = -long_stra_vel;
        angular_velocity = 0;
        if (my_abs(turn_start_x - current_pose->x) > turn_backward_distance || my_abs(turn_start_y - current_pose->y) > turn_backward_distance)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            turn_start_update = 0;
            right_walk_edge_status = LEFT_EDGE_TURN_CCLOCK_TARGET_YAW_MORE_ABS117_WE;
            break;
        }
        break;
    case LEFT_EDGE_TURN_CCLOCK_TARGET_YAW_MORE_ABS117_WE:
        linear_velocity = 0;
        angular_velocity = turn_vel;
        if (Yaw > 150)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            right_walk_edge_status = LEFT_EDGE_READY_GOSTR_BYPASS_WE;
            break;
        }
        if (obstacleSignal!=none_obstacle||(&cliff_valueB)->cliffValue0 == 1)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            right_walk_edge_status = LEFT_EDGE_TURN_CCLOCK_TARGET_YAW_MORE_ABS117_COLLISION_WE;
            break;
        }
        break;
    case LEFT_EDGE_TURN_CCLOCK_TARGET_YAW_MORE_ABS117_COLLISION_WE:
        if(turn_start_update == 0)
        {
            turn_start_x = current_pose->x;
            turn_start_y = current_pose->y;
            turn_start_update = 1;
        }
        linear_velocity = -long_stra_vel;
        angular_velocity = 0;
        if (my_abs(turn_start_x - current_pose->x) > collision_backward_distance || my_abs(turn_start_y - current_pose->y) > collision_backward_distance)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            turn_start_update = 0;
            right_walk_edge_status = LEFT_EDGE_TURN_CCLOCK_TARGET_YAW_MORE_ABS117_WE;
            break;
        }
        break;
    case LEFT_EDGE_READY_GOSTR_BYPASS_WE:
        edge_length_start = current_pose->x + half_map_wide;
        returnorigin = false;
        b_last_position_yy = false;
        last_position_yy = 0;
        temporary_close_edge = close_edge;
        right_walk_edge_status = LEFT_EDGE_GOSTR_BYPASS_WE_X;
        break;
        
    case LEFT_EDGE_GOSTR_BYPASS_WE_X:
        linear_velocity = 0;
        angular_velocity = 0;
        last_position_x = current_pose->x;
        right_walk_edge_status = LEFT_EDGE_GOSTR_BYPASS_WE;
        break;
    case CLOSE_EDGE_MAP_LEFT_WALK:
        linear_velocity = 0;
        angular_velocity = turn_vel;
        if (my_abs(Yaw)>175)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            right_walk_edge_status =STRAIGHT_CLOSE_EDGE_MAP_LEFT_WALK;
            break;
        }
        if (obstacleSignal!=none_obstacle||(&cliff_valueB)->cliffValue0 == 1)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            right_walk_edge_status =COLLISION_TURN_CLOSE_EDGE_MAP_LEFT_WALK;
            break;
        }
        break;
        
    case  COLLISION_TURN_CLOSE_EDGE_MAP_LEFT_WALK:
        if(turn_start_update == 0)
        {
            turn_start_x = current_pose->x;
            turn_start_y = current_pose->y;
            turn_start_update = 1;
        }
        linear_velocity = -long_stra_vel;
        angular_velocity = 0;
        if (my_abs(turn_start_x - current_pose->x) > collision_backward_distance || my_abs(turn_start_y - current_pose->y) > collision_backward_distance)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            turn_start_update = 0;
            right_walk_edge_status = CLOSE_EDGE_MAP_LEFT_WALK;
            break;
        }
        break;
    case STRAIGHT_CLOSE_EDGE_MAP_LEFT_WALK:
        linear_velocity = long_stra_vel;
        angular_velocity =0;
        if (obstacleSignal!=none_obstacle||(&cliff_valueB)->cliffValue0 == 1)
        {
            last_position_x = current_pose->x;
            right_walk_edge_status = LEFT_EDGE_GOSTR_BYPASS_WE;
            break;
        }
        if (my_abs(last_position_x - current_pose->x) > close_map_move_distance*lateral_move_distance)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            right_walk_edge_status =LEFT_EDGE_GOSTR_BYPASS_WE_X;
            break;
        }
        break;
        
    case LEFT_EDGE_GOSTR_BYPASS_WE:
        if (obstacleSignal !=none_obstacle||(&cliff_valueB)->cliffValue0 == 1)
        {
            right_walk_edge_status = LEFT_EDGE_COLLISION_BYPASS_WE;
            break;
        }
		right_edge_judgment_repeat();
//        if(adcRealTime[9]>100&&adcRealTime[9]<1500){
//            delimma_edge=0;
//            linear_velocity = 200;
//            angular_velocity = 0;
//        }
//        if(adcRealTime[9]>=1500){
//            delimma_edge=0;
//            linear_velocity = 200;
//            angular_velocity = 10;
//        }
//        if(adcRealTime[9]<100){
//            if(delimma_edge<10){
//                delimma_edge++;
//                linear_velocity = 200;
//                angular_velocity = -10;
//                
//            }
//            else{
//                linear_velocity = 100;
//                angular_velocity = -20;
//                
//            }
//        }
        if (my_abs(last_position_x - current_pose->x) > lateral_move_distance)
        {
            if(Yaw<135&&Yaw>0){
                right_walk_edge_status = LEFT_EDGE_GOSTR_X_MORE_LATERALDIS_BYPASS_WE;
            }
            else{
                i=(current_pose->x+half_map_wide)/GRIDWIDTH;
                j=(current_pose->y+half_map_wide)/GRIDWIDTH+3;
                if(i>0&&i<99&&j>=0&&j<GRIDWIDTH)
                gridmap.map[i-1][j]=0;
                gridmap.map[i][j]=0;
                gridmap.map[i+1][j]=0;
                gridmap.map[i-1][j-1]=0;
                gridmap.map[i][j-1]=0;
                gridmap.map[i+1][j-1]=0;
				gridmap.map[i][j+1]=0;
				gridmap.map[i+1][j+1]=0;
                last_position_x = current_pose->x;
            }
            break;			
        }
        if ((current_pose->y - last_position_y)  > temporary_close_edge)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            right_walk_edge_status = LEFT_EDGE_BOW_CONTINUE_WE;
            break;
        }
        if (b_last_position_yy == true && my_abs(last_position_yy - current_pose->y) > 3 * lateral_move_distance)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            right_walk_edge_status = LEFT_EDGE_TARGET_YAW_LESS_ABS105_LESS_0_BYPASS_WE;
            break;
        }
        if (my_abs(Yaw) < 120 && (Yaw < 0) && (b_last_position_yy == false))
        {
            last_position_yy = current_pose->y;
            b_last_position_yy = true;
        }
        break;
        
    case LEFT_EDGE_REBACK_GOSTR_BYPASS_CHECK_WE:
        if((Yaw > -135)&&(Yaw < 0))
        {
            right_walk_edge_status = LEFT_EDGE_TARGET_YAW_LESS_ABS105_LESS_0_BYPASS_WE;
            break;
        }
        if(!returnorigin){
            if ((my_abs(current_pose->x + half_map_wide - edge_length_start) > Edge_length()/2)&&(my_abs(current_pose->x + half_map_wide - edge_length_start) >500))
            {
                right_walk_edge_status = LEFT_EDGE_DELTA_X_MORE_ONE_FOURTH_CLEANED_MAP_WIDTH_WE;
                break;
            }
        }
        right_walk_edge_status = LEFT_EDGE_GOSTR_BYPASS_WE_X;
        break;
    case LEFT_EDGE_COLLISION_BYPASS_WE:
        if(turn_start_update == 0)
        {
            turn_start_x = current_pose->x;
            turn_start_y = current_pose->y;
            turn_start_update = 1;
			if (obstacleSignal == left_obstacle){
                Astarmarkingobstacle = left_obstacle;
            }
            else if (obstacleSignal == right_obstacle){
                Astarmarkingobstacle = right_obstacle;
            }
            else{
                Astarmarkingobstacle = front_obstacle;
            }
        }
        linear_velocity = -long_stra_vel;
        angular_velocity = 0;
        if (my_abs(turn_start_x - current_pose->x) > collision_backward_distance || my_abs(turn_start_y - current_pose->y) > collision_backward_distance)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            turn_start_update = 0;
            temporary_yaw = Yaw;
			if(Astarmarkingobstacle==right_obstacle){
				right_walk_edge_status = LEFT_EDGE_TURN_CCLOCK_YAW_ADD_ABS15_WE;
            }
            else{
                right_walk_edge_status = LEFT_EDGE_TURN_CCLOCK_YAW_ADD_ABS15_WE_FRONT_COLLISION;
            }
            break;
        }
        break;
    case LEFT_EDGE_TURN_CCLOCK_YAW_ADD_ABS15_WE:
        linear_velocity = 0;
        angular_velocity = turn_vel;
        if (my_abs(Yaw - temporary_yaw) > 15)
        {
            if(my_abs(Yaw)>175){
                temporary_yaw=Yaw;
                right_walk_edge_status =LEFT_EDGE_TURN_CCLOCK_YAW_ADD_ABS15_WE;
                break;
            }
            linear_velocity = 0;
            angular_velocity = 0;
            right_walk_edge_status = LEFT_EDGE_REBACK_GOSTR_BYPASS_CHECK_WE;
            break;
        }
        if (obstacleSignal!=none_obstacle||(&cliff_valueB)->cliffValue0 == 1)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            right_walk_edge_status = LEFT_EDGE_TURN_CCLOCK_YAW_ADD_ABS15_COLLISION_WE;
            break;
        }
        break;
    case LEFT_EDGE_TURN_CCLOCK_YAW_ADD_ABS15_COLLISION_WE:
        if(turn_start_update == 0)
        {
            turn_start_x = current_pose->x;
            turn_start_y = current_pose->y;
            turn_start_update = 1;
        }
        linear_velocity = -long_stra_vel;
        angular_velocity = 0;
        if (my_abs(turn_start_x - current_pose->x) > collision_backward_distance || my_abs(turn_start_y - current_pose->y) > collision_backward_distance)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            turn_start_update = 0;
            right_walk_edge_status = LEFT_EDGE_TURN_CCLOCK_YAW_ADD_ABS15_WE;
            break;
        }
        break;
	case LEFT_EDGE_TURN_CCLOCK_YAW_ADD_ABS15_WE_FRONT_COLLISION:
        linear_velocity = 0;
        angular_velocity = turn_vel;
        if (my_abs(Yaw - temporary_yaw) > 30)
        {
            if(my_abs(Yaw)>175){
                temporary_yaw=Yaw;
                right_walk_edge_status =LEFT_EDGE_TURN_CCLOCK_YAW_ADD_ABS15_WE_FRONT_COLLISION;
                break;
            }
            linear_velocity = 0;
            angular_velocity = 0;
            right_walk_edge_status = LEFT_EDGE_REBACK_GOSTR_BYPASS_CHECK_WE;
            break;
        }
        if (obstacleSignal!=none_obstacle||(&cliff_valueB)->cliffValue0 == 1)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            right_walk_edge_status = LEFT_EDGE_TURN_CCLOCK_YAW_ADD_ABS15_WE_FRONT_COLLISION_COLLISION;
            break;
        }
        break;
    case LEFT_EDGE_TURN_CCLOCK_YAW_ADD_ABS15_WE_FRONT_COLLISION_COLLISION:
        if(turn_start_update == 0)
        {
            turn_start_x = current_pose->x;
            turn_start_y = current_pose->y;
            turn_start_update = 1;
        }
        linear_velocity = -long_stra_vel;
        angular_velocity = 0;
        if (my_abs(turn_start_x - current_pose->x) > collision_backward_distance || my_abs(turn_start_y - current_pose->y) > collision_backward_distance)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            turn_start_update = 0;
            right_walk_edge_status = LEFT_EDGE_TURN_CCLOCK_YAW_ADD_ABS15_WE_FRONT_COLLISION;
            break;
        }
        break;
		
    case LEFT_EDGE_TARGET_YAW_LESS_ABS105_LESS_0_BYPASS_WE:
        linear_velocity = 0;
        angular_velocity = 0;
        if(returnorigin==true){
            right_walk_edge_status = LEFT_EDGE_RETURN_ORIGIN_WE;
            break;
        }
        if(!detection_close_edge){
            detection_close_edge=true;
        }
        else{
            detection_close_edge=false;
            if(detection_close==true){
                detection_close=false;
                right_walk_edge_status = LEFT_EDGE_LEFT_EDGE_DILEMMA_WE;
            }else{
                right_walk_edge_status = LEFT_EDGE_RETURN_ORIGIN_WE;
            }
        }
        break;
    case LEFT_EDGE_DELTA_X_MORE_ONE_FOURTH_CLEANED_MAP_WIDTH_WE:
        linear_velocity = 0;
        angular_velocity = 0;
        if(!detection_close_edge){
            detection_close_edge=true;
        }
        else{
            detection_close_edge=false;
            if(detection_close==true){
                detection_close=false;
                temporary_yaw = Yaw;
                right_walk_edge_status = CLOSE_EDGE_MAP_LEFT_WALK_TURN_CCLOCK_YAW_ADD_ABS15_WE;
            }else{
                returnorigin=true;
                right_walk_edge_status = LEFT_EDGE_GOSTR_BYPASS_WE_X;
            }
        }
        break;
        
    case CLOSE_EDGE_MAP_LEFT_WALK_TURN_CCLOCK_YAW_ADD_ABS15_WE:
        linear_velocity = 0;
        angular_velocity = turn_vel;
        if (my_abs(Yaw - temporary_yaw) > 15)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            last_position_x = current_pose->x;
            right_walk_edge_status = LEFT_EDGE_GOSTR_BYPASS_WE;
            break;
        }
        if (obstacleSignal!=none_obstacle||(&cliff_valueB)->cliffValue0 == 1)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            right_walk_edge_status = COLLISION_CLOSE_EDGE_MAP_LEFT_WALK_TURN_CCLOCK_YAW_ADD_ABS15_WE;
            break;
        }
        break;
    case COLLISION_CLOSE_EDGE_MAP_LEFT_WALK_TURN_CCLOCK_YAW_ADD_ABS15_WE:
        if(turn_start_update == 0)
        {
            turn_start_x = current_pose->x;
            turn_start_y = current_pose->y;
            turn_start_update = 1;
        }
        linear_velocity = -long_stra_vel;
        angular_velocity = 0;
        if (my_abs(turn_start_x - current_pose->x) > collision_backward_distance || my_abs(turn_start_y - current_pose->y) > collision_backward_distance)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            turn_start_update = 0;
            right_walk_edge_status = CLOSE_EDGE_MAP_LEFT_WALK_TURN_CCLOCK_YAW_ADD_ABS15_WE;
            break;
        }
        break;
        
    case LEFT_EDGE_TURN_CCLOCK_TARGET_YAW_LESS_ABS3_WE:
        if (my_abs(Yaw) < 8)
        {
            right_walk_edge_status = LEFT_EDGE_LEFT_EDGE_DILEMMA_WE;
            break;
        }
        
        if (obstacleSignal !=none_obstacle||(&cliff_valueB)->cliffValue0 == 1)
        {
            right_walk_edge_status = LEFT_EDGE_TURN_CCLOCK_TARGET_YAW_LESS_ABS3_COLLISION_WE;
            break;
        }
        linear_velocity = 0;
        angular_velocity = turn_vel;
        break;
        
    case LEFT_EDGE_TURN_CCLOCK_TARGET_YAW_LESS_ABS3_COLLISION_WE:
        if(turn_start_update == 0)
        {
            turn_start_x = current_pose->x;
            turn_start_y = current_pose->y;
            turn_start_update = 1;
        }
        linear_velocity = -long_stra_vel;
        angular_velocity = 0;
        if (my_abs(turn_start_x - current_pose->x) > collision_backward_distance || my_abs(turn_start_y - current_pose->y) > collision_backward_distance)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            turn_start_update = 0;
            right_walk_edge_status = LEFT_EDGE_TURN_CCLOCK_TARGET_YAW_LESS_ABS3_WE;
        }
        break;
        
    case LEFT_EDGE_GOSTR_X_MORE_LATERALDIS_BYPASS_WE:
        temporary_yaw = Yaw;
        if (my_abs(Yaw) < 150 && (Yaw < 0))
        {
            right_walk_edge_status = LEFT_EDGE_TURN_CLOCK_TARGET_YAW_MORE_0_WE;
            break;
        }
        else if (my_abs(Yaw) >= 150)
        {
            right_walk_edge_status = LEFT_EDGE_TURN_CLOCK_TARGET_YAW_LESS_135_DETAL_YAW_MORE_30_WE;
            break;
        }
        else{
            right_walk_edge_status = LEFT_EDGE_GOSTR_BYPASS_WE_X;
            break;
        }
    case LEFT_EDGE_TURN_CLOCK_TARGET_YAW_MORE_0_WE:
        linear_velocity = 0;
        angular_velocity = -turn_vel;
        if (Yaw > 0 )
        {
            linear_velocity = 0;
            angular_velocity = 0;
            right_walk_edge_status = LEFT_EDGE_GOSTR_BYPASS_WE_X;
            break;
        }
        if (obstacleSignal !=none_obstacle||(&cliff_valueB)->cliffValue0 == 1)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            right_walk_edge_status = LEFT_EDGE_TURN_CLOCK_TARGET_YAW_MORE_0_COLLISION_WE;
            break;
        }
        break;
    case LEFT_EDGE_TURN_CLOCK_TARGET_YAW_MORE_0_COLLISION_WE:
        if(turn_start_update == 0)
        {
            turn_start_x = current_pose->x;
            turn_start_y = current_pose->y;
            turn_start_update = 1;
        }
        linear_velocity = -long_stra_vel;
        angular_velocity = 0;
        if (my_abs(turn_start_x - current_pose->x) > collision_backward_distance || my_abs(turn_start_y - current_pose->y) > collision_backward_distance)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            turn_start_update = 0;
            temporary_yaw = Yaw;
            right_walk_edge_status = LEFT_EDGE_TURN_CCLOCK_YAW_ADD_ABS15_WE;
            break;
        }
        break;
    case LEFT_EDGE_TURN_CLOCK_TARGET_YAW_LESS_135_DETAL_YAW_MORE_30_WE:
        linear_velocity = 0;
        angular_velocity = -turn_vel;
        if (my_abs(temporary_yaw - Yaw) > 30 && my_abs(Yaw) < 150)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            right_walk_edge_status = LEFT_EDGE_GOSTR_BYPASS_WE_X;
            break;
        }
        if (obstacleSignal !=none_obstacle||(&cliff_valueB)->cliffValue0 == 1)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            right_walk_edge_status = LEFT_EDGE_TURN_CLOCK_TARGET_YAW_LESS_135_DETAL_YAW_MORE_30_COLLISION_WE;
            break;
        }
        break;
    case LEFT_EDGE_TURN_CLOCK_TARGET_YAW_LESS_135_DETAL_YAW_MORE_30_COLLISION_WE:
        if(turn_start_update == 0)
        {
            turn_start_x = current_pose->x;
            turn_start_y = current_pose->y;
            turn_start_update = 1;
        }
        linear_velocity = -long_stra_vel;
        angular_velocity = 0;
        if (my_abs(turn_start_x - current_pose->x) > collision_backward_distance || my_abs(turn_start_y - current_pose->y) > collision_backward_distance)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            turn_start_update = 0;
            temporary_yaw = Yaw;
            right_walk_edge_status = LEFT_EDGE_TURN_CCLOCK_YAW_ADD_ABS15_WE;
            break;
        }
        break;
    case LEFT_EDGE_BOW_CONTINUE_WE:
        right_walk_edge_status = 0;
        returnorigin=false;
        complete_flag = 1;
        break;
    case LEFT_EDGE_LEFT_EDGE_DILEMMA_WE:
        linear_velocity = 0;
        angular_velocity = 0;
        if(my_abs(leakingsweep_x-current_pose->x)>500&&my_abs(leakingsweep_y-current_pose->y)>100){
			if((my_abs(current_pose->x)<half_map_wide-GRIDHEIGHT)&&(my_abs(current_pose->y)<half_map_wide-GRIDHEIGHT)){
            leakingsweep_x=current_pose->x;
            leakingsweep_y=current_pose->y;
            leftboolleaksweep=true;
            break;
			}
        }
        if (0 != leakingsweep)
        {
            right_walk_edge_status = 0;
            returnorigin=false;
            left_running_step_status = LEAKING_SWEEP_LEFTRUN_STEP;
            break;
        }
        else
        {
            right_walk_edge_status = 0;
            returnorigin=false;
            complete_flag = 2;
        }
        break;
    case LEFT_EDGE_RETURN_ORIGIN_WE:
        linear_velocity = 0;
        angular_velocity = 0;
        if(my_abs(leakingsweep_x-current_pose->x)>500&&my_abs(leakingsweep_y-current_pose->y)>100){
			if((my_abs(current_pose->x)<half_map_wide-GRIDHEIGHT)&&(my_abs(current_pose->y)<half_map_wide-GRIDHEIGHT)){
            leakingsweep_x=current_pose->x;
            leakingsweep_y=current_pose->y;
            leftboolleaksweep=true;
            break;
			}
        }
        if (0 != leakingsweep)
        {
            right_walk_edge_status = 0;
            returnorigin=false;
            left_running_step_status = LEAKING_SWEEP_LEFTRUN_STEP;
            break;
        }
        else
        {
            complete_flag = 3;
            returnorigin=false;
            right_walk_edge_status = 0;
        }
        
        break;
    }
    return complete_flag;
}




unsigned char  LeftReverseWalkEdge(POSE *current_pose,unsigned char obstacleSignal){
    int Yaw;
    unsigned char complete_flag=0;
    signed char i,j;
    Yaw = current_pose->orientation;
    Yaw = Yaw /100;
    if (my_abs(current_pose->x)>half_map_wide-2*GRIDWIDTH||my_abs(current_pose->y)>half_map_wide-2*GRIDWIDTH){
        linear_velocity = 0;
        angular_velocity = 0;
        if(my_abs(leakingsweep_x-current_pose->x)>500&&my_abs(leakingsweep_y-current_pose->y)>100){
			if((my_abs(current_pose->x)<half_map_wide-GRIDHEIGHT)&&(my_abs(current_pose->y)<half_map_wide-GRIDHEIGHT)){
            leakingsweep_x=current_pose->x;
            leakingsweep_y=current_pose->y;
            leftboolleaksweep=true;
            return complete_flag;
			}
        }
        if (0 != leakingsweep)
        {
            right_reverse_walk_edge_status =  0;
            returnorigin=false;
            left_running_step_status = LEAKING_SWEEP_LEFTRUN_STEP;
            return complete_flag;
        }
        else
        {
            right_reverse_walk_edge_status =  0;
            returnorigin=false;
            complete_flag = 3;
            return complete_flag;
        }
    }
    switch(right_reverse_walk_edge_status)
    {
    case 0:
        right_reverse_walk_edge_status = LEFT_GOBACK_REVERSE_WALK_EDGE;
        break;
    case LEFT_GOBACK_REVERSE_WALK_EDGE:
        if(turn_start_update == 0)
        {
            turn_start_x = current_pose->x;
            turn_start_y = current_pose->y;
            turn_start_update = 1;
        }
        linear_velocity = -long_stra_vel;
        angular_velocity = 0;
        if (my_abs(turn_start_x - current_pose->x) > turn_backward_distance || my_abs(turn_start_y - current_pose->y) > turn_backward_distance)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            turn_start_update = 0;
            right_reverse_walk_edge_status = LEFT_REVERSE_EDGE_TURN_CLOCK_TARGET_YAW_LESS_ABS63_RWE;
            break;
        }
        break;
    case LEFT_REVERSE_EDGE_TURN_CLOCK_TARGET_YAW_LESS_ABS63_RWE:
        linear_velocity = 0;
        angular_velocity = -turn_vel;
        if((Yaw < 30)&&(Yaw > 0))
        {
            linear_velocity = 0;
            angular_velocity = 0;
            right_reverse_walk_edge_status = LEFT_REVERSE_EDGE_READY_GOSTR_BYPASS_RWE;
            break;
        }
        if (obstacleSignal !=none_obstacle||(&cliff_valueB)->cliffValue0 == 1)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            right_reverse_walk_edge_status = LEFT_REVERSE_EDGE_TURN_CLOCK_TARGET_YAW_LESS_ABS63_COLLISION_RWE;
            break;
        }
        break;
    case LEFT_REVERSE_EDGE_TURN_CLOCK_TARGET_YAW_LESS_ABS63_COLLISION_RWE:
        if(turn_start_update == 0)
        {
            turn_start_x = current_pose->x;
            turn_start_y = current_pose->y;
            turn_start_update = 1;
        }
        linear_velocity = -long_stra_vel;
        angular_velocity = 0;
        if (my_abs(turn_start_x - current_pose->x) > collision_backward_distance || my_abs(turn_start_y - current_pose->y) > collision_backward_distance)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            turn_start_update = 0;
            right_reverse_walk_edge_status = LEFT_REVERSE_EDGE_TURN_CLOCK_TARGET_YAW_LESS_ABS63_RWE;
            break;
        }
        break;
    case LEFT_REVERSE_EDGE_READY_GOSTR_BYPASS_RWE:
        edge_length_start = current_pose->x + half_map_wide;
        returnorigin = false;
        b_last_position_yy = false;
        last_position_yy = 0;
        temporary_close_edge = close_edge;
        right_reverse_walk_edge_status = LEFT_REVERSE_EDGE_GOSTR_BYPASS_RWE_X;
        break;
    case LEFT_REVERSE_EDGE_GOSTR_BYPASS_RWE_X:
        linear_velocity = 0;
        angular_velocity = 0;
        last_position_x = current_pose->x;
        right_reverse_walk_edge_status = LEFT_REVERSE_EDGE_GOSTR_BYPASS_RWE;
        break;        
    case LEFT_REVERSE_EDGE_GOSTR_BYPASS_RWE:
        if (obstacleSignal!=none_obstacle||(&cliff_valueB)->cliffValue0 == 1)
        {
            right_reverse_walk_edge_status = LEFT_REVERSE_EDGE_COLLISION_BYPASS_RWE;
            break;
        }
		left_edge_judgment_repeat();
//        if(adcRealTime[8]>100&&adcRealTime[8]<1500){
//            delimma_edge=0;
//            linear_velocity = 200;
//            angular_velocity = 0;
//        }
//        if(adcRealTime[8]>=1500){
//            delimma_edge=0;
//            linear_velocity = 200;
//            angular_velocity = -10;
//        }
//        if(adcRealTime[8]<100){
//            if(delimma_edge<10){
//                linear_velocity = 200;
//                angular_velocity = 10;
//                delimma_edge++;
//            }
//            else{
//                linear_velocity = 100;
//                angular_velocity = 20;
//            } 
//        }
        if (my_abs(last_position_x - current_pose->x) > lateral_move_distance)
        {
            if(Yaw>45&&Yaw<90){
                right_reverse_walk_edge_status = LEFT_REVERSE_EDGE_GOSTR_X_MORE_LATERALDIS_BYPASS_RWE;
            }
            else{
                i=(current_pose->x+half_map_wide)/GRIDWIDTH;
                j=(current_pose->y+half_map_wide)/GRIDWIDTH+3;
                if(i>0&&i<99&&j>=0&&j<GRIDWIDTH)
                gridmap.map[i-1][j]=0;
                gridmap.map[i][j]=0;
                gridmap.map[i+1][j]=0;
                gridmap.map[i-1][j-1]=0;
                gridmap.map[i][j-1]=0;
                gridmap.map[i+1][j-1]=0;
				gridmap.map[i][j+1]=0;
                gridmap.map[i-1][j+1]=0;
                last_position_x = current_pose->x;
            }
            break;			
        }
        if ((current_pose->y - last_position_y)  > temporary_close_edge)
        {
            right_reverse_walk_edge_status = LEFT_REVERSE_EDGE_BOW_CONTINUE_RWE;
            break;
        }
        if (b_last_position_yy == true && my_abs(last_position_yy - current_pose->y) > 3 * lateral_move_distance)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            right_reverse_walk_edge_status = LEFT_REVERSE_EDGE_TARGET_YAW_MORE_ABS75_LESS_ABS0_RWE;
            break;
        }
        if (my_abs(Yaw) > 60 && (Yaw < 0) && (b_last_position_yy == false))
        {
            last_position_yy = current_pose->y;
            b_last_position_yy = true;
        }
        break;
    case LEFT_REVERSE_EDGE_REBACK_GOSTR_BYPASS_CHECK_RWE:
        if((Yaw < -45)&& (Yaw > -120))
        {
            right_reverse_walk_edge_status = LEFT_REVERSE_EDGE_TARGET_YAW_MORE_ABS75_LESS_ABS0_RWE;
            break;
        }
        if(!returnorigin){		
            if ((my_abs(current_pose->x + half_map_wide - edge_length_start) > Edge_length()/2)&&(my_abs(current_pose->x + half_map_wide - edge_length_start) > 500))
            {
                right_reverse_walk_edge_status = LEFT_REVERSE_EDGE_DELTA_X_MORE_ONE_FOURTH_CLEANED_MAP_WIDTH_RWE;
                break;
            }
        }
        right_reverse_walk_edge_status = LEFT_REVERSE_EDGE_GOSTR_BYPASS_RWE_X;
        break;
    case LEFT_REVERSE_EDGE_COLLISION_BYPASS_RWE:
        if(turn_start_update == 0)
        {
            turn_start_x = current_pose->x;
            turn_start_y = current_pose->y;
            turn_start_update = 1;
			if (obstacleSignal == left_obstacle){
                Astarmarkingobstacle = left_obstacle;
            }
            else if (obstacleSignal == right_obstacle){
                Astarmarkingobstacle = right_obstacle;
            }
            else{
                Astarmarkingobstacle = front_obstacle;
            }
        }
        linear_velocity = -long_stra_vel;
        angular_velocity = 0;
        if (my_abs(turn_start_x - current_pose->x) > collision_backward_distance || my_abs(turn_start_y - current_pose->y) > collision_backward_distance)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            turn_start_update = 0;
            temporary_yaw = Yaw;			
			if(Astarmarkingobstacle==left_obstacle){
				right_reverse_walk_edge_status = LEFT_REVERSE_EDGE_TURN_CLOCK_YAW_ADD_ABS15_RWE;
            }
            else{
                right_reverse_walk_edge_status = LEFT_REVERSE_EDGE_TURN_CLOCK_YAW_ADD_ABS15_RWE_FRONT_COLLISION;
            }
            break;
        }
        break;
    case LEFT_REVERSE_EDGE_TURN_CLOCK_YAW_ADD_ABS15_RWE:
        linear_velocity = 0;
        angular_velocity = -turn_vel;
        if (my_abs(Yaw - temporary_yaw) > 15)
        {
            if(my_abs(Yaw)>175){
                temporary_yaw=Yaw;
                right_reverse_walk_edge_status = LEFT_REVERSE_EDGE_TURN_CLOCK_YAW_ADD_ABS15_RWE;
                break;
            }
            linear_velocity = 0;
            angular_velocity = 0;
            right_reverse_walk_edge_status = LEFT_REVERSE_EDGE_REBACK_GOSTR_BYPASS_CHECK_RWE;
            break;
        }
        if (obstacleSignal !=none_obstacle||(&cliff_valueB)->cliffValue0 == 1)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            right_reverse_walk_edge_status = LEFT_REVERSE_EDGE_TURN_CLOCK_YAW_ADD_ABS15_COLLISION_RWE;
            break;
        }
        break;
    case LEFT_REVERSE_EDGE_TURN_CLOCK_YAW_ADD_ABS15_COLLISION_RWE:
        if(turn_start_update == 0)
        {
            turn_start_x = current_pose->x;
            turn_start_y = current_pose->y;
            turn_start_update = 1;
        }
        linear_velocity = -long_stra_vel;
        angular_velocity = 0;
        if (my_abs(turn_start_x - current_pose->x) > collision_backward_distance || my_abs(turn_start_y - current_pose->y) > collision_backward_distance)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            turn_start_update = 0;
            right_reverse_walk_edge_status = LEFT_REVERSE_EDGE_TURN_CLOCK_YAW_ADD_ABS15_RWE;
            break;
        }
        break;
    case LEFT_REVERSE_EDGE_TURN_CLOCK_YAW_ADD_ABS15_RWE_FRONT_COLLISION:
        linear_velocity = 0;
        angular_velocity = -turn_vel;
        if (my_abs(Yaw - temporary_yaw) > 30)
        {
            if(my_abs(Yaw)>175){
                temporary_yaw=Yaw;
                right_reverse_walk_edge_status = LEFT_REVERSE_EDGE_TURN_CLOCK_YAW_ADD_ABS15_RWE_FRONT_COLLISION;
                break;
            }
            linear_velocity = 0;
            angular_velocity = 0;
            right_reverse_walk_edge_status = LEFT_REVERSE_EDGE_REBACK_GOSTR_BYPASS_CHECK_RWE;
            break;
        }
        if (obstacleSignal !=none_obstacle||(&cliff_valueB)->cliffValue0 == 1)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            right_reverse_walk_edge_status = LEFT_REVERSE_EDGE_TURN_CLOCK_YAW_ADD_ABS15_RWE_FRONT_COLLISION_COLLISION;
            break;
        }
        break;
    case LEFT_REVERSE_EDGE_TURN_CLOCK_YAW_ADD_ABS15_RWE_FRONT_COLLISION_COLLISION:
        if(turn_start_update == 0)
        {
            turn_start_x = current_pose->x;
            turn_start_y = current_pose->y;
            turn_start_update = 1;
        }
        linear_velocity = -long_stra_vel;
        angular_velocity = 0;
        if (my_abs(turn_start_x - current_pose->x) > collision_backward_distance || my_abs(turn_start_y - current_pose->y) > collision_backward_distance)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            turn_start_update = 0;
            right_reverse_walk_edge_status = LEFT_REVERSE_EDGE_TURN_CLOCK_YAW_ADD_ABS15_RWE_FRONT_COLLISION;
            break;
        }
        break;
		
    case LEFT_REVERSE_EDGE_TARGET_YAW_MORE_ABS75_LESS_ABS0_RWE:
        linear_velocity = 0;
        angular_velocity = 0;
        if(returnorigin==true){
            right_reverse_walk_edge_status =LEFT_REVERSE_EDGE_RETURN_ORIGIN_RWE;
            break;
        }
        if(!detection_close_edge){
            detection_close_edge=true;
            break;
        }else{
            detection_close_edge=false;
            if(detection_close==true){
                detection_close=false;
                right_reverse_walk_edge_status = LEFT_REVERSE_EDGE_LEFT_EDGE_DILEMMA_RWE;
            }else{
                right_reverse_walk_edge_status =LEFT_REVERSE_EDGE_RETURN_ORIGIN_RWE;
            }
            break;
        }
    case LEFT_REVERSE_EDGE_DELTA_X_MORE_ONE_FOURTH_CLEANED_MAP_WIDTH_RWE:              
        linear_velocity = 0;
        angular_velocity = 0;
        if(!detection_close_edge){
            detection_close_edge=true;
        }
        else{
            detection_close_edge=false;
            if(detection_close==true){
                detection_close=false;
                temporary_yaw = Yaw;
                right_reverse_walk_edge_status = CLOSE_EDGE_MAP_LEFT_REVERSE_WALK_TURN_CLOCK_YAW_ADD_ABS15_RWE;
            }else{
                returnorigin=true;
                right_reverse_walk_edge_status = LEFT_REVERSE_EDGE_GOSTR_BYPASS_RWE_X;
            }
        }
        break;
        
    case CLOSE_EDGE_MAP_LEFT_REVERSE_WALK_TURN_CLOCK_YAW_ADD_ABS15_RWE:
        linear_velocity = 0;
        angular_velocity =-turn_vel;
        if (my_abs(Yaw - temporary_yaw) > 15)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            last_position_x = current_pose->x;
            right_reverse_walk_edge_status = LEFT_REVERSE_EDGE_GOSTR_BYPASS_RWE;
            break;
        }
        if (obstacleSignal!=none_obstacle||(&cliff_valueB)->cliffValue0 == 1)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            right_reverse_walk_edge_status = COLLISION_CLOSE_EDGE_MAP_LEFT_REVERSE_WALK_TURN_CLOCK_YAW_ADD_ABS15_RWE;
            break;
        }
        break;
    case COLLISION_CLOSE_EDGE_MAP_LEFT_REVERSE_WALK_TURN_CLOCK_YAW_ADD_ABS15_RWE:
        if(turn_start_update == 0)
        {
            turn_start_x = current_pose->x;
            turn_start_y = current_pose->y;
            turn_start_update = 1;
        }
        linear_velocity = -long_stra_vel;
        angular_velocity = 0;
        if (my_abs(turn_start_x - current_pose->x) > collision_backward_distance || my_abs(turn_start_y - current_pose->y) > collision_backward_distance)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            turn_start_update = 0;
            right_reverse_walk_edge_status = CLOSE_EDGE_MAP_LEFT_REVERSE_WALK_TURN_CLOCK_YAW_ADD_ABS15_RWE;
            break;
        }
        break;
    case LEFT_REVERSE_EDGE_TURN_CCLOCK_YAW_LESS_ABS8_RWE:
        if (my_abs(Yaw) > 172)
        {
            right_reverse_walk_edge_status = LEFT_REVERSE_EDGE_LEFT_EDGE_DILEMMA_RWE;
            break;
        }
        if (obstacleSignal !=none_obstacle||(&cliff_valueB)->cliffValue0 == 1)
        {
            right_reverse_walk_edge_status = LEFT_REVERSE_EDGE_TURN_CCLOCK_YAW_LESS_ABS8_COLLISION_RWE;
            break;
        }
        linear_velocity = 0;
        angular_velocity = -turn_vel;
        break;
    case LEFT_REVERSE_EDGE_TURN_CCLOCK_YAW_LESS_ABS8_COLLISION_RWE:
        if(turn_start_update == 0)
        {
            turn_start_x = current_pose->x;
            turn_start_y = current_pose->y;
            turn_start_update = 1;
        }
        linear_velocity = -long_stra_vel;
        angular_velocity = 0;
        if (my_abs(turn_start_x - current_pose->x) > collision_backward_distance || my_abs(turn_start_y - current_pose->y) > collision_backward_distance)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            turn_start_update = 0;
            right_reverse_walk_edge_status = LEFT_REVERSE_EDGE_TURN_CCLOCK_YAW_LESS_ABS8_RWE;
            break;
        }
        break;
        
    case LEFT_REVERSE_EDGE_GOSTR_X_MORE_LATERALDIS_BYPASS_RWE:
        temporary_yaw = Yaw;
        if (my_abs(Yaw) > 30 && (Yaw < 0))
        {
            right_reverse_walk_edge_status = LEFT_REVERSE_EDGE_TURN_CCLOCK_TARGET_YAW_MORE_0_RWE;
            
            break;
        }
        else if (my_abs(Yaw) <= 30 )
        {
            right_reverse_walk_edge_status = LEFT_REVERSE_EDGE_TURN_CCLOCK_TARGET_YAW_MORE_ABS45_DETAL_YAW_MORE_30_RWE;
            break;
        }
        else{
            right_reverse_walk_edge_status = LEFT_REVERSE_EDGE_GOSTR_BYPASS_RWE_X;
            break;
        }
    case LEFT_REVERSE_EDGE_TURN_CCLOCK_TARGET_YAW_MORE_0_RWE:
        linear_velocity = 0;
        angular_velocity = turn_vel;
        if (Yaw > 0 )
        {
            linear_velocity = 0;
            angular_velocity = 0;
            right_reverse_walk_edge_status = LEFT_REVERSE_EDGE_GOSTR_BYPASS_RWE_X;
            break;
        }
        if (obstacleSignal !=none_obstacle||(&cliff_valueB)->cliffValue0 == 1)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            right_reverse_walk_edge_status = LEFT_REVERSE_EDGE_TURN_CCLOCK_TARGET_YAW_MORE_0_COLLISION_RWE;
            break;
        }
        break;
    case LEFT_REVERSE_EDGE_TURN_CCLOCK_TARGET_YAW_MORE_0_COLLISION_RWE:
        if(turn_start_update == 0)
        {
            turn_start_x = current_pose->x;
            turn_start_y = current_pose->y;
            turn_start_update = 1;
        }
        linear_velocity = -long_stra_vel;
        angular_velocity = 0;
        if (my_abs(turn_start_x - current_pose->x) > collision_backward_distance || my_abs(turn_start_y - current_pose->y) > collision_backward_distance)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            turn_start_update = 0;
            temporary_yaw = Yaw;
            right_reverse_walk_edge_status = LEFT_REVERSE_EDGE_TURN_CLOCK_YAW_ADD_ABS15_RWE;
            break;
        }
        break;
    case LEFT_REVERSE_EDGE_TURN_CCLOCK_TARGET_YAW_MORE_ABS45_DETAL_YAW_MORE_30_RWE:
        linear_velocity = 0;
        angular_velocity = turn_vel;
        if (my_abs(temporary_yaw - Yaw) > 30 && my_abs(Yaw) > 30)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            right_reverse_walk_edge_status = LEFT_REVERSE_EDGE_GOSTR_BYPASS_RWE_X;
            break;
        }
        if (obstacleSignal !=none_obstacle||(&cliff_valueB)->cliffValue0 == 1)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            right_reverse_walk_edge_status = LEFT_REVERSE_EDGE_TURN_CCLOCK_TARGET_YAW_MORE_ABS45_DETAL_YAW_MORE_30_COLLISION_RWE;
            break;
        }
        break;
    case LEFT_REVERSE_EDGE_TURN_CCLOCK_TARGET_YAW_MORE_ABS45_DETAL_YAW_MORE_30_COLLISION_RWE:
        if(turn_start_update == 0)
        {
            turn_start_x = current_pose->x;
            turn_start_y = current_pose->y;
            turn_start_update = 1;
        }
        linear_velocity = -long_stra_vel;
        angular_velocity = 0;
        if (my_abs(turn_start_x - current_pose->x) > collision_backward_distance || my_abs(turn_start_y - current_pose->y) > collision_backward_distance)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            turn_start_update = 0;
            temporary_yaw = Yaw;
            right_reverse_walk_edge_status = LEFT_REVERSE_EDGE_TURN_CLOCK_YAW_ADD_ABS15_RWE;
            break;
        }
        break;
    case LEFT_REVERSE_EDGE_BOW_CONTINUE_RWE:
        right_reverse_walk_edge_status = 0;
        returnorigin=false;
        complete_flag = 1;
        break;
    case LEFT_REVERSE_EDGE_LEFT_EDGE_DILEMMA_RWE:
        linear_velocity = 0;
        angular_velocity = 0;
        if(my_abs(leakingsweep_x-current_pose->x)>500&&my_abs(leakingsweep_y-current_pose->y)>100){
			if((my_abs(current_pose->x)<half_map_wide-GRIDHEIGHT)&&(my_abs(current_pose->y)<half_map_wide-GRIDHEIGHT)){
            leakingsweep_x=current_pose->x;
            leakingsweep_y=current_pose->y;
            leftboolleaksweep=true;
            break;
			}
        }
        if (0 != leakingsweep)
        {
            right_reverse_walk_edge_status =  0;
            returnorigin=false;
            left_running_step_status = LEAKING_SWEEP_LEFTRUN_STEP;
            break;
        }
        else
        {
            returnorigin=false;
            complete_flag = 2;
            right_reverse_walk_edge_status = 0;
            returnorigin=false;
        }
        break;
        
    case LEFT_REVERSE_EDGE_RETURN_ORIGIN_RWE:
        linear_velocity = 0;
        angular_velocity = 0;
        if(my_abs(leakingsweep_x-current_pose->x)>500&&my_abs(leakingsweep_y-current_pose->y)>100){
			if((my_abs(current_pose->x)<half_map_wide-GRIDHEIGHT)&&(my_abs(current_pose->y)<half_map_wide-GRIDHEIGHT)){
            leakingsweep_x=current_pose->x;
            leakingsweep_y=current_pose->y;
            leftboolleaksweep=true;
            break;
			}
        }
        if (0 != leakingsweep)
        {
            right_reverse_walk_edge_status =  0;
            returnorigin=false;
            left_running_step_status = LEAKING_SWEEP_LEFTRUN_STEP;
            break;
        }
        else
        {
            complete_flag = 3;
            returnorigin=false;
            right_reverse_walk_edge_status = 0;
        }
        break;
    }
    return complete_flag;
}



unsigned char  LeftEdgeDilemma(POSE *current_pose, unsigned char obstacleSignal){
    int Yaw;
    unsigned char complete_flag = 0;
    signed char i=0;
    signed char j=0;
    signed char k,ij;
    Yaw = current_pose->orientation;
    Yaw = Yaw /100;
    switch(right_edge_dilemma_status)
    {
    case 0:
        linear_velocity = 0;
        angular_velocity = 0;
        right_edge_dilemma_status =LEFT_DILEMMA_GOSTR_DILEMMA;
        break;
    case LEFT_DILEMMA_GOSTR_DILEMMA:
        if(my_abs(current_pose->x+half_map_wide)<=100*close_edge_max_x-500){
            if(my_abs(Yaw)<=90){
                right_edge_dilemma_status = LEFT_DILEMMA_COLLISION_YAW_LESS_ABS90_DILEMMA;
            }
            else{
                right_edge_dilemma_status = LEFT_DILEMMA_YAW_MORE_ABS90_LESS_ABS176_CCLOCK_LESS_ABS90_MORE_ABS4_CLOCK_DILEMMA;
            }
        }
        else{
            if(my_abs(Yaw)<=90){
                right_edge_dilemma_status =LEFT_DILEMMA_CCLOCK_TARGET_YAW_LESS_ABS3_DILEMMA;
            }
            else{
                right_edge_dilemma_status =LEFT_DILEMMA_COLLISION_YAW_MORE_ABS90_DILEMMA;
            }
			DelimmaNumber++;
        }
        step=1;
        DelimmaNumber++;
        break;
    case LEFT_DILEMMA_YAW_MORE_ABS90_LESS_ABS176_CCLOCK_LESS_ABS90_MORE_ABS4_CLOCK_DILEMMA:
        linear_velocity = 0;
        angular_velocity =-turn_vel;
        if (my_abs(Yaw)<10)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            right_edge_dilemma_status =LEFT_DILEMMA_COLLISION_YAW_LESS_ABS90_DILEMMA;
            break;
        }
        if (obstacleSignal!=none_obstacle||(&cliff_valueB)->cliffValue0 == 1)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            right_edge_dilemma_status = LEFT_DILEMMA_CCLOCK_TARGET_YAW_LESS_ABS3_COLLISION_DILEMMA;
            break;
        }
        break;
    case LEFT_DILEMMA_CCLOCK_TARGET_YAW_LESS_ABS3_COLLISION_DILEMMA:
        if(turn_start_update == 0)
        {
            turn_start_x = current_pose->x;
            turn_start_y = current_pose->y;
            turn_start_update = 1;
        }
        linear_velocity = -long_stra_vel;
        angular_velocity = 0;
        if (my_abs(turn_start_x - current_pose->x) > collision_backward_distance || my_abs(turn_start_y - current_pose->y) > collision_backward_distance)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            turn_start_update = 0;
            right_edge_dilemma_status = LEFT_DILEMMA_YAW_MORE_ABS90_LESS_ABS176_CCLOCK_LESS_ABS90_MORE_ABS4_CLOCK_DILEMMA;
            break;
        }
        break;
    case LEFT_DILEMMA_CCLOCK_TARGET_YAW_LESS_ABS3_DILEMMA:
        linear_velocity = 0;
        angular_velocity =turn_vel;
        if (my_abs(Yaw)>170)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            right_edge_dilemma_status =LEFT_DILEMMA_COLLISION_YAW_MORE_ABS90_DILEMMA;
            break;
        }
        if (obstacleSignal!=none_obstacle||(&cliff_valueB)->cliffValue0 == 1)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            right_edge_dilemma_status = LEFT_DILEMMA_GOSTR_CYL_DILEMMA;
            break;
        }
        break;
    case LEFT_DILEMMA_GOSTR_CYL_DILEMMA:
        if(turn_start_update == 0)
        {
            turn_start_x = current_pose->x;
            turn_start_y = current_pose->y;
            turn_start_update = 1;
        }
        linear_velocity = -long_stra_vel;
        angular_velocity = 0;
        if (my_abs(turn_start_x - current_pose->x) > collision_backward_distance || my_abs(turn_start_y - current_pose->y) > collision_backward_distance)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            turn_start_update = 0;
            right_edge_dilemma_status = LEFT_DILEMMA_CCLOCK_TARGET_YAW_LESS_ABS3_DILEMMA;
            break;
        }
        break;
    case LEFT_DILEMMA_COLLISION_YAW_LESS_ABS90_DILEMMA:
        linear_velocity = 0;
        angular_velocity = 0;
        last_position_x = current_pose->x;
        last_position_y = current_pose->y;
        temporary_close_edge=close_edge;
        right_edge_dilemma_status = LEFT_DILEMMA_LOOP_TEN_NUM_DILEMMA;
        break;
    case LEFT_DILEMMA_LOOP_TEN_NUM_DILEMMA:
		left_edge_judgment_repeat();
//        if(adcRealTime[8]>100&&adcRealTime[8]<1500){
//            delimma_edge=0;
//            linear_velocity = 200;
//            angular_velocity = 0;
//        }
//        if(adcRealTime[8]>=1500){
//            delimma_edge=0;
//            linear_velocity = 200;
//            angular_velocity = -10;
//        }
//        if(adcRealTime[8]<100){
//            if(delimma_edge<10){
//                linear_velocity = 200;
//                angular_velocity = 10;
//                delimma_edge++;
//            }
//            else{
//                linear_velocity = 100;
//                angular_velocity = 20;
//            }
//        }		
        if(my_abs(current_pose->y)<return_origin_distance){
            if(close_edge_max_y<GRIDHEIGHT/2+5){
            }
            else{
                linear_velocity = 0;
                angular_velocity = 0;
                right_edge_dilemma_status=0;
                DelimmaNumber=0;
                step=0;
                temporary_yaw=0;
                complete_flag = 2;
                break;
            }
        }
        if(my_abs(current_pose->x+half_map_wide)>=100*close_edge_max_x-500){
            linear_velocity = 0;
            angular_velocity = 0;
            if(my_abs(leakingsweep_x-current_pose->x)>500&&my_abs(leakingsweep_y-current_pose->y)>100){
				if((my_abs(current_pose->x)<half_map_wide-GRIDHEIGHT)&&(my_abs(current_pose->y)<half_map_wide-GRIDHEIGHT)){
                leakingsweep_x=current_pose->x;
                leakingsweep_y=current_pose->y;
                leftboolleaksweep=true;
                break;
				}
            }
            if (0 != leakingsweep)
            {
                right_edge_dilemma_status=0;
                DelimmaNumber=0;
                step=0;
                left_running_step_status = LEAKING_SWEEP_LEFTRUN_STEP;
                break;
            }
            else
            {
                right_edge_dilemma_status=COMPLETE_LEFT_DILEMMA;
                break;
            }
        }
        if (obstacleSignal!=none_obstacle||(&cliff_valueB)->cliffValue0 == 1)
        {
            right_edge_dilemma_status = LEFT_DILEMMA_GOSTR_COLLISION_DILEMMA;
            break;
        }
        if(60<Yaw&&Yaw<105){
            linear_velocity = long_stra_vel;
            angular_velocity = 0;
            last_position_x = current_pose->x;
            last_position_y = current_pose->y;
            i=(current_pose->x+half_map_long)/100;
            j=(current_pose->y+half_map_long+110)/100;			
            if(j<0||i<1||j>MAPWIDECELLS-1||i>MAPLONGCELLS-2){
            }
            else{
                j=((gridmap.map[i-1][j]==125)?1:0)+((gridmap.map[i][j]==125)?1:0)+((gridmap.map[i][j]==125)?1:0);
                if(j==3){
					linear_velocity = 0;
                    angular_velocity = 0;
                    right_edge_dilemma_status=0;
                    DelimmaNumber=0;
                    step=0;
                    complete_flag = 1;
                }
            }           
            break;
        }
        if (my_abs(last_position_x - current_pose->x) > dilemma_close_edge||my_abs(last_position_y - current_pose->y) > dilemma_close_edge)
        {
            if(my_abs(Yaw)>=135){
                k=(current_pose->x+half_map_wide)/GRIDWIDTH;
                ij=(current_pose->y+half_map_wide)/GRIDWIDTH-3;
                if(k>0&&k<99&&ij>0&&ij<GRIDWIDTH){
                    gridmap.map[k-1][ij+1]=0;
                    gridmap.map[k][ij+1]=0;
                    gridmap.map[k+1][ij+1]=0;
                    gridmap.map[k-1][ij]=0;
                    gridmap.map[k][ij]=0;
                    gridmap.map[k+1][ij]=0;
					gridmap.map[k][ij-1]=0;
					gridmap.map[k+1][ij-1]=0;
                }
            }
            else if(Yaw<-45&&Yaw>-135){
                k=(current_pose->x+half_map_wide)/GRIDWIDTH+3;
                ij=(current_pose->y+half_map_wide)/GRIDWIDTH;
                if(k>0&&k<GRIDWIDTH&&ij>0&&ij<99){
                    gridmap.map[k-1][ij-1]=0;
                    gridmap.map[k-1][ij]=0;
                    gridmap.map[k-1][ij+1]=0;
                    gridmap.map[k][ij-1]=0;
                    gridmap.map[k][ij]=0;
                    gridmap.map[k][ij+1]=0;
				    gridmap.map[k+1][ij]=0;
				    gridmap.map[k+1][ij+1]=0;
                }
            }
            else if(my_abs(Yaw)<=45){
                k=(current_pose->x+half_map_wide)/GRIDWIDTH;
                ij=(current_pose->y+half_map_wide)/GRIDWIDTH+3;
                if(k>0&&k<99&&ij>0&&ij<GRIDWIDTH){
                    gridmap.map[k-1][ij-1]=0;
                    gridmap.map[k][ij-1]=0;
                    gridmap.map[k+1][ij-1]=0;
                    gridmap.map[k-1][ij]=0;
                    gridmap.map[k][ij]=0;
                    gridmap.map[k+1][ij]=0;
                    gridmap.map[k][ij+1]=0;
					gridmap.map[k-1][ij+1]=0;
                }
            }
            else{
                k=(current_pose->x+half_map_wide)/GRIDWIDTH-3;
                ij=(current_pose->y+half_map_wide)/GRIDWIDTH;
                if(k>=0&&k<99&&ij>0&&ij<99){
                    gridmap.map[k+1][ij-1]=0;
                    gridmap.map[k+1][ij]=0;
                    gridmap.map[k+1][ij+1]=0;
                    gridmap.map[k][ij-1]=0;
                    gridmap.map[k][ij]=0;
                    gridmap.map[k][ij+1]=0;
					gridmap.map[k-1][ij]=0;
					gridmap.map[k-1][ij-1]=0;
                }
            }
            last_position_x = current_pose->x;
            last_position_y = current_pose->y;
            step++;
            if(step>20){
                step=0;
                right_edge_dilemma_status = LEFT_DILEMMA_GOSTR_DELTA_Y_MORE_LATERAL_DIS_CYM_DILEMMA;
            }
            temporary_yaw = Yaw;
            break;
        }
        break;
    case COMPLETE_LEFT_DILEMMA:
        linear_velocity = 0;
        angular_velocity = 0;
        if(!detection_close_edge){
            detection_close_edge=true;
        }
        else{
            detection_close_edge=false;
            if(detection_close==true){
                detection_close=false;
                if(DelimmaNumber<2){
                    DelimmaNumber++;
                    right_edge_dilemma_status =LEFT_DILEMMA_CCLOCK_TARGET_YAW_LESS_ABS3_DILEMMA;
                }else{
                    right_edge_dilemma_status=0;
                    DelimmaNumber=0;
                    step=0;
                    temporary_yaw=0;
                    complete_flag = 2;
                }
            }else{
                right_edge_dilemma_status=0;
                DelimmaNumber=0;
                step=0;
                temporary_yaw=0;
                complete_flag = 2;
            }
        }
        break;
    case LEFT_DILEMMA_GOSTR_COLLISION_DILEMMA:
        if(turn_start_update == 0)
        {
            turn_start_x = current_pose->x;
            turn_start_y = current_pose->y;
            turn_start_update = 1;
			if (obstacleSignal == left_obstacle){
                Astarmarkingobstacle = left_obstacle;
            }
            else if (obstacleSignal == right_obstacle){
                Astarmarkingobstacle = right_obstacle;
            }
            else{
                Astarmarkingobstacle = front_obstacle;
            }
        }
        linear_velocity = -long_stra_vel;
        angular_velocity = 0;
        if (my_abs(turn_start_x - current_pose->x) > collision_backward_distance || my_abs(turn_start_y - current_pose->y) > collision_backward_distance)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            turn_start_update = 0;
            temporary_yaw = Yaw;
			if(Astarmarkingobstacle==left_obstacle){
				right_edge_dilemma_status = LEFT_DILEMMA_CCLOCK_TARGET_YAW_LESS_ABS90_DILEMMA;
            }
            else{
                right_edge_dilemma_status = LEFT_DILEMMA_CCLOCK_TARGET_YAW_LESS_ABS90_DILEMMA_FRONT_COLLISION;
            }
            break;
        }
        break;
		
    case LEFT_DILEMMA_CCLOCK_TARGET_YAW_LESS_ABS90_DILEMMA:
        if(my_abs(leakingsweep_x-current_pose->x)>500&&my_abs(leakingsweep_y-current_pose->y)>100){
			if((my_abs(current_pose->x)<half_map_wide-GRIDHEIGHT)&&(my_abs(current_pose->y)<half_map_wide-GRIDHEIGHT)){
            leakingsweep_x=current_pose->x;
            leakingsweep_y=current_pose->y;
            leftboolleaksweep=true;
            linear_velocity = 0;
            angular_velocity = 0;
            break;
			}
        }
        if (0 != leakingsweep)
        {
            right_edge_dilemma_status=0;
            DelimmaNumber=0;
            step=0;
            linear_velocity = 0;
            angular_velocity = 0;
            left_running_step_status = LEAKING_SWEEP_LEFTRUN_STEP;
            break;
        }
        else
        {
            linear_velocity = 0;
            angular_velocity = -turn_vel;
            if (my_abs(Yaw - temporary_yaw) > 15)
            {
                if(my_abs(Yaw)>175){
                    temporary_yaw=Yaw;
                    right_edge_dilemma_status =LEFT_DILEMMA_CCLOCK_TARGET_YAW_LESS_ABS90_DILEMMA;
                    break;
                }
                linear_velocity = 0;
                angular_velocity = 0;
                right_edge_dilemma_status =LEFT_DILEMMA_COLLISION_YAW_LESS_ABS90_DILEMMA;
                break;
            }
            if (obstacleSignal!=none_obstacle||(&cliff_valueB)->cliffValue0 == 1)
            {
                linear_velocity = 0;
                angular_velocity = 0;
                right_edge_dilemma_status = LEFT_DILEMMA_CCLOCK_TARGET_YAW_LESS_ABS90_COLLISION_DILEMMA ;
                break;
            }
        }
        break;
    case LEFT_DILEMMA_CCLOCK_TARGET_YAW_LESS_ABS90_COLLISION_DILEMMA:
        if(turn_start_update == 0)
        {
            turn_start_x = current_pose->x;
            turn_start_y = current_pose->y;
            turn_start_update = 1;
        }
        linear_velocity = -long_stra_vel;
        angular_velocity = 0;
        if (my_abs(turn_start_x - current_pose->x) > collision_backward_distance || my_abs(turn_start_y - current_pose->y) > collision_backward_distance)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            turn_start_update = 0;
            right_edge_dilemma_status = LEFT_DILEMMA_CCLOCK_TARGET_YAW_LESS_ABS90_DILEMMA;
            break;
        }
        break;
		
    case LEFT_DILEMMA_CCLOCK_TARGET_YAW_LESS_ABS90_DILEMMA_FRONT_COLLISION:
        if(my_abs(leakingsweep_x-current_pose->x)>500&&my_abs(leakingsweep_y-current_pose->y)>100){
			if((my_abs(current_pose->x)<half_map_wide-GRIDHEIGHT)&&(my_abs(current_pose->y)<half_map_wide-GRIDHEIGHT)){
            leakingsweep_x=current_pose->x;
            leakingsweep_y=current_pose->y;
            leftboolleaksweep=true;
            linear_velocity = 0;
            angular_velocity = 0;
            break;
			}
        }
        if (0 != leakingsweep)
        {
            right_edge_dilemma_status=0;
            DelimmaNumber=0;
            step=0;
            linear_velocity = 0;
            angular_velocity = 0;
            left_running_step_status = LEAKING_SWEEP_LEFTRUN_STEP;
            break;
        }
        else
        {
            linear_velocity = 0;
            angular_velocity = -turn_vel;
            if (my_abs(Yaw - temporary_yaw) > 30)
            {
                if(my_abs(Yaw)>175){
                    temporary_yaw=Yaw;
                    right_edge_dilemma_status =LEFT_DILEMMA_CCLOCK_TARGET_YAW_LESS_ABS90_DILEMMA_FRONT_COLLISION;
                    break;
                }
                linear_velocity = 0;
                angular_velocity = 0;
                right_edge_dilemma_status =LEFT_DILEMMA_COLLISION_YAW_LESS_ABS90_DILEMMA;
                break;
            }
            if (obstacleSignal!=none_obstacle||(&cliff_valueB)->cliffValue0 == 1)
            {
                linear_velocity = 0;
                angular_velocity = 0;
                right_edge_dilemma_status = LEFT_DILEMMA_CCLOCK_TARGET_YAW_LESS_ABS90_DILEMMA_FRONT_COLLISION_COLLISION;
                break;
            }
        }
        break;
    case LEFT_DILEMMA_CCLOCK_TARGET_YAW_LESS_ABS90_DILEMMA_FRONT_COLLISION_COLLISION:
        if(turn_start_update == 0)
        {
            turn_start_x = current_pose->x;
            turn_start_y = current_pose->y;
            turn_start_update = 1;
        }
        linear_velocity = -long_stra_vel;
        angular_velocity = 0;
        if (my_abs(turn_start_x - current_pose->x) > collision_backward_distance || my_abs(turn_start_y - current_pose->y) > collision_backward_distance)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            turn_start_update = 0;
            right_edge_dilemma_status = LEFT_DILEMMA_CCLOCK_TARGET_YAW_LESS_ABS90_DILEMMA_FRONT_COLLISION;
            break;
        }
        break;
	
    case LEFT_DILEMMA_GOSTR_DELTA_Y_MORE_LATERAL_DIS_CYM_DILEMMA:
        if( 60<Yaw && Yaw<105)
        { 
            linear_velocity = long_stra_vel;
            angular_velocity = 0;
            last_position_x = current_pose->x;
            last_position_y = current_pose->y;
            right_edge_dilemma_status = LEFT_DILEMMA_LOOP_TEN_NUM_DILEMMA;
            temporary_close_edge=dilemma_close_edge;
            break;
        }
        linear_velocity = 0;
        angular_velocity =turn_vel;
        if (my_abs(temporary_yaw - Yaw) > 30||( 60<Yaw && Yaw<105))
        { 
            linear_velocity = 0;
            angular_velocity = 0;
            last_position_x = current_pose->x;
            last_position_y = current_pose->y;
            right_edge_dilemma_status = LEFT_DILEMMA_LOOP_TEN_NUM_DILEMMA;
            temporary_close_edge=dilemma_close_edge;
            break;
        }
        if (obstacleSignal!=none_obstacle||(&cliff_valueB)->cliffValue0 == 1 )
        {
            linear_velocity = 0;
            angular_velocity = 0;
            right_edge_dilemma_status = LEFT_DILEMMA_CLOCK_TARGET_YAW_MORE_ABS87_COLLISION_DILEMMA;
            break;
        }
        break;
        
    case LEFT_DILEMMA_CLOCK_TARGET_YAW_MORE_ABS87_COLLISION_DILEMMA:
        if(turn_start_update == 0)
        {
            turn_start_x = current_pose->x;
            turn_start_y = current_pose->y;
            turn_start_update = 1;
        }
        linear_velocity = -long_stra_vel;
        angular_velocity = 0;
        if (my_abs(turn_start_x - current_pose->x) > collision_backward_distance || my_abs(turn_start_y - current_pose->y) > collision_backward_distance)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            turn_start_update = 0;
            temporary_yaw = Yaw;
            right_edge_dilemma_status = LEFT_DILEMMA_CCLOCK_TARGET_YAW_LESS_ABS90_DILEMMA;
        }
        break;
    case LEFT_DILEMMA_COLLISION_YAW_MORE_ABS90_DILEMMA:
        linear_velocity = 0;
        angular_velocity = 0;
        last_position_x = current_pose->x;
        last_position_y = current_pose->y;
        temporary_close_edge=close_edge;
        right_edge_dilemma_status = LEFT_DILEMMA_GOSTR_CYM_DILEMMA;
        break;
    case LEFT_DILEMMA_GOSTR_CYM_DILEMMA:
		right_edge_judgment_repeat();
//        if(adcRealTime[9]>100&&adcRealTime[9]<1500){
//            delimma_edge=0;
//            linear_velocity = 200;
//            angular_velocity = 0;
//        }
//        if(adcRealTime[9]>=1500){
//            delimma_edge=0;
//            linear_velocity = 200;
//            angular_velocity = 10;
//        }
//        if(adcRealTime[9]<100){
//            if(delimma_edge<10){
//                delimma_edge++;
//                linear_velocity = 200;
//                angular_velocity = -10;
//            }
//            else{
//                linear_velocity = 100;
//                angular_velocity = -20;
//            }
//        }
        if(my_abs(current_pose->y)<return_origin_distance){
            if(close_edge_max_y<GRIDHEIGHT/2+5){
            }
            else{
                linear_velocity = 0;
                angular_velocity = 0;
                right_edge_dilemma_status=0;
                DelimmaNumber=0;
                step=0;
                temporary_yaw=180;
                complete_flag = 2;
                break;
            }
        }
        if(my_abs(current_pose->x+half_map_wide)<100*close_edge_min_x+500){
            linear_velocity = 0;
            angular_velocity = 0;
            if(my_abs(leakingsweep_x-current_pose->x)>100&&my_abs(leakingsweep_y-current_pose->y)>100){
				if((my_abs(current_pose->x)<half_map_wide-GRIDHEIGHT)&&(my_abs(current_pose->y)<half_map_wide-GRIDHEIGHT)){
                leakingsweep_x=current_pose->x;
                leakingsweep_y=current_pose->y;
                leftboolleaksweep=true;
                break;
				}
            }
            if (0 != leakingsweep)
            {
                right_edge_dilemma_status=0;
                DelimmaNumber=0;
                step=0;
                left_running_step_status = LEAKING_SWEEP_LEFTRUN_STEP;
                break;
            }
            else
            {
                right_edge_dilemma_status=LEFT_DILEMMA_DELTA_X_MORE_ONE_THIRD_CLEANED_MAP_WIDTH_DILEMMA;
                break;
            }
        }
        
        if (obstacleSignal!=none_obstacle||(&cliff_valueB)->cliffValue0 == 1)
        {
            right_edge_dilemma_status = LEFT_DILEMMA_GOSTR_COLLISION_CYM_DILEMMA;
            break;
        }
        if(Yaw<120&&Yaw>75){
            linear_velocity = long_stra_vel;
            angular_velocity = 0;
            i=(current_pose->x+half_map_long)/100;
            j=(current_pose->y+half_map_long+110)/100;
            last_position_x = current_pose->x;
            last_position_y = current_pose->y;
            if(j<0||i<1||j>MAPWIDECELLS-1||i>MAPLONGCELLS-2){
            }
            else{
                j=((gridmap.map[i-1][j]==125)?1:0)+((gridmap.map[i][j]==125)?1:0)+((gridmap.map[i][j]==125)?1:0);
                if(j==2){
                    right_edge_dilemma_status=0;
                    DelimmaNumber=0;
					            linear_velocity = 0;
            angular_velocity = 0;
                    step=0;
                    complete_flag = 1;
                }
            }
            break;
        }
        if (my_abs(last_position_x - current_pose->x) > dilemma_close_edge||my_abs(last_position_y - current_pose->y) > dilemma_close_edge)
        {
            if(my_abs(Yaw)<=45){
                k=(current_pose->x+half_map_wide)/GRIDWIDTH;
                ij=(current_pose->y+half_map_wide)/GRIDWIDTH-3;
                if(k>0&&k<99&&ij>0&&ij<GRIDWIDTH){
                    gridmap.map[k-1][ij+1]=0;
                    gridmap.map[k][ij+1]=0;
                    gridmap.map[k+1][ij+1]=0;
                    gridmap.map[k-1][ij]=0;
                    gridmap.map[k][ij]=0;
                    gridmap.map[k+1][ij]=0;
                    gridmap.map[k][ij-1]=0;
                    gridmap.map[k-1][ij-1]=0;
                }
            }
            else if(Yaw<135&&Yaw>45){
                k=(current_pose->x+half_map_wide)/GRIDWIDTH+3;
                ij=(current_pose->y+half_map_wide)/GRIDWIDTH;
                if(k>0&&k<GRIDWIDTH&&ij>0&&ij<99){
                    gridmap.map[k-1][ij-1]=0;
                    gridmap.map[k-1][ij]=0;
                    gridmap.map[k-1][ij+1]=0;
                    gridmap.map[k][ij-1]=0;
                    gridmap.map[k][ij]=0;
                    gridmap.map[k][ij+1]=0;
                    gridmap.map[k+1][ij]=0;
                    gridmap.map[k+1][ij-1]=0;
                }
            }
            else if(my_abs(Yaw)>=135){
                k=(current_pose->x+half_map_wide)/GRIDWIDTH;
                ij=(current_pose->y+half_map_wide)/GRIDWIDTH+3;
                if(k>0&&k<99&&ij>0&&ij<GRIDWIDTH){
                    gridmap.map[k-1][ij-1]=0;
                    gridmap.map[k][ij-1]=0;
                    gridmap.map[k+1][ij-1]=0;
                    gridmap.map[k-1][ij]=0;
                    gridmap.map[k][ij]=0;
                    gridmap.map[k+1][ij]=0;
                    gridmap.map[k][ij+1]=0;
                    gridmap.map[k+1][ij+1]=0;
                }
            }
            else{
                k=(current_pose->x+half_map_wide)/GRIDWIDTH-3;
                ij=(current_pose->y+half_map_wide)/GRIDWIDTH;
                if(k>=0&&k<99&&ij>0&&ij<99){
                    gridmap.map[k+1][ij-1]=0;
                    gridmap.map[k+1][ij]=0;
                    gridmap.map[k+1][ij+1]=0;
                    gridmap.map[k][ij-1]=0;
                    gridmap.map[k][ij]=0;
                    gridmap.map[k][ij+1]=0;
                    gridmap.map[k-1][ij]=0;
                    gridmap.map[k-1][ij+1]=0;
                }
            }
            last_position_x = current_pose->x;
            last_position_y = current_pose->y;
            step++;
            if(step>40){
                step=0;
                right_edge_dilemma_status = LEFT_DILEMMA_GOSTR_GOSTR_DELTA_Y_MORE_LATERAL_DIS_CYL_DILEMMA;
            }
            temporary_yaw = Yaw;
            break;
        }
        break;
    case LEFT_DILEMMA_GOSTR_GOSTR_DELTA_Y_MORE_LATERAL_DIS_CYL_DILEMMA:
        if (Yaw>75&&Yaw<120)
        {
            linear_velocity = long_stra_vel;
            angular_velocity = 0;
            last_position_x = current_pose->x;
            last_position_y = current_pose->y;
            temporary_close_edge=dilemma_close_edge;
            right_edge_dilemma_status = LEFT_DILEMMA_GOSTR_CYM_DILEMMA;
            break;
        }
        linear_velocity = 0;
        angular_velocity =-turn_vel;
        if (my_abs(temporary_yaw - Yaw) > 30||(Yaw>75&&Yaw<120))
        {
            linear_velocity = 0;
            angular_velocity = 0;
            last_position_x = current_pose->x;
            last_position_y = current_pose->y;
            temporary_close_edge=dilemma_close_edge;
            right_edge_dilemma_status = LEFT_DILEMMA_GOSTR_CYM_DILEMMA;
            break;
        }
        if (obstacleSignal!=none_obstacle||(&cliff_valueB)->cliffValue0 == 1 )
        {
            linear_velocity = 0;
            angular_velocity = 0;
            step=1;
            right_edge_dilemma_status = LEFT_DILEMMA_GOSTR_COLLISION_CYL_DILEMMA;
            break;
        }
        break;
    case LEFT_DILEMMA_GOSTR_COLLISION_CYL_DILEMMA:
        if(turn_start_update == 0)
        {
            turn_start_x = current_pose->x;
            turn_start_y = current_pose->y;
            turn_start_update = 1;
        }
        linear_velocity = -long_stra_vel;
        angular_velocity = 0;
        if (my_abs(turn_start_x - current_pose->x) > collision_backward_distance || my_abs(turn_start_y - current_pose->y) > collision_backward_distance)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            turn_start_update = 0;
            right_edge_dilemma_status = LEFT_DILEMMA_CLOCK_TARGET_YAW_MORE_ABS173_DILEMMA;
        }
        break;
        
    case LEFT_DILEMMA_GOSTR_COLLISION_CYM_DILEMMA:
        if(turn_start_update == 0)
        {
            turn_start_x = current_pose->x;
            turn_start_y = current_pose->y;
            turn_start_update = 1;
			if (obstacleSignal == left_obstacle){
                Astarmarkingobstacle = left_obstacle;
            }
            else if (obstacleSignal == right_obstacle){
                Astarmarkingobstacle = right_obstacle;
            }
            else{
                Astarmarkingobstacle = front_obstacle;
            }
        }
        linear_velocity = -long_stra_vel;
        angular_velocity = 0;
        if (my_abs(turn_start_x - current_pose->x) > collision_backward_distance || my_abs(turn_start_y - current_pose->y) > collision_backward_distance)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            turn_start_update = 0;
            temporary_yaw = Yaw;
			if(Astarmarkingobstacle==right_obstacle){
				right_edge_dilemma_status = LEFT_DILEMMA_CLOCK_TARGET_YAW_MORE_ABS173_DILEMMA;
            }
            else{
                right_edge_dilemma_status = LEFT_DILEMMA_CLOCK_TARGET_YAW_MORE_ABS173_DILEMMA_FRONT_COLLISION;
            }
            break;
        }
        break;
    case LEFT_DILEMMA_CLOCK_TARGET_YAW_MORE_ABS173_DILEMMA:
        if(my_abs(leakingsweep_x-current_pose->x)>500&&my_abs(leakingsweep_y-current_pose->y)>100){
			if((my_abs(current_pose->x)<half_map_wide-GRIDHEIGHT)&&(my_abs(current_pose->y)<half_map_wide-GRIDHEIGHT)){
            linear_velocity = 0;
            angular_velocity = 0;
            leakingsweep_x=current_pose->x;
            leakingsweep_y=current_pose->y;
            leftboolleaksweep=true;
            break;
			}
        }
        if (0 != leakingsweep)
        {
            right_edge_dilemma_status=0;
            DelimmaNumber=0;
            step=0;
            left_running_step_status = LEAKING_SWEEP_LEFTRUN_STEP;
            break;
        }
        else
        {
            
            linear_velocity = 0;
            angular_velocity =turn_vel;
            if (my_abs(Yaw - temporary_yaw) > 15)
            {
                if(my_abs(Yaw)>175){
                    temporary_yaw=Yaw;
                    right_edge_dilemma_status =LEFT_DILEMMA_CLOCK_TARGET_YAW_MORE_ABS173_DILEMMA;
                    break;
                }
                linear_velocity = 0;
                angular_velocity = 0;
                right_edge_dilemma_status =LEFT_DILEMMA_COLLISION_YAW_MORE_ABS90_DILEMMA;
                break;
            }
            if (obstacleSignal!=none_obstacle||(&cliff_valueB)->cliffValue0 == 1)
            {
                linear_velocity = 0;
                angular_velocity = 0;
                right_edge_dilemma_status = LEFT_DILEMMA_CLOCK_TARGET_YAW_MORE_ABS87_DILEMMA;
                break;
            }
        }
        break;
    case LEFT_DILEMMA_CLOCK_TARGET_YAW_MORE_ABS87_DILEMMA:
        if(turn_start_update == 0)
        {
            turn_start_x = current_pose->x;
            turn_start_y = current_pose->y;
            turn_start_update = 1;
        }
        linear_velocity = -long_stra_vel;
        angular_velocity = 0;
        if (my_abs(turn_start_x - current_pose->x) > collision_backward_distance || my_abs(turn_start_y - current_pose->y) > collision_backward_distance)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            turn_start_update = 0;
            right_edge_dilemma_status = LEFT_DILEMMA_CLOCK_TARGET_YAW_MORE_ABS173_DILEMMA ;
            break;
        }
        break;
		
	case LEFT_DILEMMA_CLOCK_TARGET_YAW_MORE_ABS173_DILEMMA_FRONT_COLLISION:
        if(my_abs(leakingsweep_x-current_pose->x)>500&&my_abs(leakingsweep_y-current_pose->y)>100){
			if((my_abs(current_pose->x)<half_map_wide-GRIDHEIGHT)&&(my_abs(current_pose->y)<half_map_wide-GRIDHEIGHT)){
            linear_velocity = 0;
            angular_velocity = 0;
            leakingsweep_x=current_pose->x;
            leakingsweep_y=current_pose->y;
            leftboolleaksweep=true;
            break;
			}
        }
        if (0 != leakingsweep)
        {
            right_edge_dilemma_status=0;
            DelimmaNumber=0;
            step=0;
            left_running_step_status = LEAKING_SWEEP_LEFTRUN_STEP;
            break;
        }
        else
        {
            
            linear_velocity = 0;
            angular_velocity =turn_vel;
            if (my_abs(Yaw - temporary_yaw) > 30)
            {
                if(my_abs(Yaw)>175){
                    temporary_yaw=Yaw;
                    right_edge_dilemma_status =LEFT_DILEMMA_CLOCK_TARGET_YAW_MORE_ABS173_DILEMMA_FRONT_COLLISION;
                    break;
                }
                linear_velocity = 0;
                angular_velocity = 0;
                right_edge_dilemma_status =LEFT_DILEMMA_COLLISION_YAW_MORE_ABS90_DILEMMA;
                break;
            }
            if (obstacleSignal!=none_obstacle||(&cliff_valueB)->cliffValue0 == 1)
            {
                linear_velocity = 0;
                angular_velocity = 0;
                right_edge_dilemma_status = LEFT_DILEMMA_CLOCK_TARGET_YAW_MORE_ABS173_DILEMMA_FRONT_COLLISION_COLLISION;
                break;
            }
        }
        break;
    case LEFT_DILEMMA_CLOCK_TARGET_YAW_MORE_ABS173_DILEMMA_FRONT_COLLISION_COLLISION:
        if(turn_start_update == 0)
        {
            turn_start_x = current_pose->x;
            turn_start_y = current_pose->y;
            turn_start_update = 1;
        }
        linear_velocity = -long_stra_vel;
        angular_velocity = 0;
        if (my_abs(turn_start_x - current_pose->x) > collision_backward_distance || my_abs(turn_start_y - current_pose->y) > collision_backward_distance)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            turn_start_update = 0;
            right_edge_dilemma_status = LEFT_DILEMMA_CLOCK_TARGET_YAW_MORE_ABS173_DILEMMA_FRONT_COLLISION;
            break;
        }
        break;
		
        
    case LEFT_DILEMMA_DELTA_X_MORE_ONE_THIRD_CLEANED_MAP_WIDTH_DILEMMA:
        linear_velocity = 0;
        angular_velocity = 0;
        if(!detection_close_edge){
            detection_close_edge=true;
        }
        else{
            detection_close_edge=false;
            if(detection_close==true){
                detection_close=false;
                if(DelimmaNumber<2){
                    DelimmaNumber++;
                    right_edge_dilemma_status = LEFT_DILEMMA_YAW_MORE_ABS90_LESS_ABS176_CCLOCK_LESS_ABS90_MORE_ABS4_CLOCK_DILEMMA;
                }else{
                    right_edge_dilemma_status=0;
                    DelimmaNumber=0;
                    temporary_yaw=180;
                    step=0;
                    complete_flag = 2;
                }
            }else{
                right_edge_dilemma_status=0;
                DelimmaNumber=0;
                temporary_yaw=180;
                step=0;
                complete_flag = 2;
            }
        }
        break;
    }
    return complete_flag;
}



unsigned char LeftReadyLeakingSweep(POSE *current_pose,unsigned char obstacleSignal){
    int Yaw;
    unsigned char complete_flag = 0;
    unsigned char i=0;
    Yaw = current_pose->orientation;
    Yaw = Yaw /100;
    switch(right_ready_leaking_sweep_status)
    {
    case 0:
        right_ready_leaking_sweep_status = LEFT_LEAKING_SWEEP_COLLISION;
        break;
    case LEFT_LEAKING_SWEEP_COLLISION:
        if(turn_start_update == 0)
        {
            turn_start_x = current_pose->x;
            turn_start_y = current_pose->y;
            turn_start_update = 1;
        }
        linear_velocity = -long_stra_vel;
        angular_velocity = 0;
        if (my_abs(turn_start_x - current_pose->x) > collision_backward_distance || my_abs(turn_start_y - current_pose->y) > collision_backward_distance)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            turn_start_update = 0;
            if(my_abs(Yaw)>90)
            {
                right_ready_leaking_sweep_status = LEFT_LEAKING_SWEEP_YAW_MORE_ABS90;
                break;
            }
            else
            {
                right_ready_leaking_sweep_status = LEFT_LEAKING_SWEEP_YAW_OTHER;
                break;
            }
        }
        break;
    case LEFT_LEAKING_SWEEP_YAW_MORE_ABS90:
        right_ready_leaking_sweep_status = LEFT_LEAKING_SWEEP_CCLOCK_TARGET_YAW_LESS_ABS90;
        break;
    case LEFT_LEAKING_SWEEP_CCLOCK_TARGET_YAW_LESS_ABS90:
        if(my_abs(Yaw)<95){
            linear_velocity=0;
            angular_velocity=0;
            last_position_y=current_pose->y;
            right_ready_leaking_sweep_status = LEFT_LEAKING_SWEEP_GOSTRAIGHT_MORE;
            break;
        }
        if (obstacleSignal!=none_obstacle||(&cliff_valueB)->cliffValue0 == 1)
        {
            linear_velocity=0;
            angular_velocity=0;
            right_ready_leaking_sweep_status = LEFT_LEAKING_SWEEP_CCLOCK_TARGER_YAW_LESS_ABS90_COLLISION;
        }
        linear_velocity=0;
        angular_velocity=turn_vel;
        break;
    case LEFT_LEAKING_SWEEP_CCLOCK_TARGER_YAW_LESS_ABS90_COLLISION:
        if(turn_start_update == 0)
        {
            turn_start_x = current_pose->x;
            turn_start_y = current_pose->y;
            turn_start_update = 1;
        }
        linear_velocity = -long_stra_vel;
        angular_velocity = 0;
        if(my_abs(turn_start_x - current_pose->x) > collision_backward_distance || my_abs(turn_start_y - current_pose->y) > collision_backward_distance)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            turn_start_update = 0;
            right_ready_leaking_sweep_status = LEFT_LEAKING_SWEEP_CCLOCK_TARGET_YAW_LESS_ABS90;
            break;
        }
        break;
    case LEFT_LEAKING_SWEEP_GOSTRAIGHT_MORE:
        linear_velocity=long_stra_vel;
        angular_velocity=0;
        if(obstacleSignal!=none_obstacle||(&cliff_valueB)->cliffValue0 == 1){
            last_position_xx=current_pose->x;
            right_ready_leaking_sweep_status = LEFT_LEAKING_SWEEP_START_WALK_EDGE;
            break;
        }
        if(my_abs(last_position_y-current_pose->y)>leakingsweep){
            right_ready_leaking_sweep_status = LEFT_LEAKING_SWEEP_CCLOCK_TARGER_YAW_LESS_ABS8;
            break;
        }
        break;
        
    case LEFT_LEAKING_SWEEP_START_WALK_EDGE:
        linear_velocity = 0;
        angular_velocity = 0;
        last_position_x = current_pose->x;
        last_position_yy = current_pose->y;
        right_ready_leaking_sweep_status = LEFT_LEAKING_SWEEP_START_WALK_EDGE_LOOP;
        break;
    case LEFT_LEAKING_SWEEP_START_WALK_EDGE_LOOP:
        if (obstacleSignal!=none_obstacle||(&cliff_valueB)->cliffValue0 == 1)
        {
            right_ready_leaking_sweep_status = LEFT_LEAKING_SWEEP_START_WALK_EDGE_LOOP_COLLISION;
            break;
        }
        if(Yaw>-150&&Yaw<-30){
			right_edge_judgment_repeat();
//            if(adcRealTime[9]>100&&adcRealTime[9]<1500){
//                delimma_edge=0;
//                linear_velocity = 200;
//                angular_velocity = 0;
//            }
//            if(adcRealTime[9]>=1500){
//                delimma_edge=0;
//                linear_velocity = 200;
//                angular_velocity = 10;
//            }
//            if(adcRealTime[9]<100){
//                if(delimma_edge<10){
//                    delimma_edge++;
//                    linear_velocity = 200;
//                    angular_velocity = -10;
//                }
//                else{
//                    linear_velocity = 100;
//                    angular_velocity = -20;
//                }
//            }
        }else{
            if (my_abs(last_position_x - current_pose->x) > lateral_move_distance||my_abs(last_position_yy - current_pose->y) > lateral_move_distance)
            {
                if(Yaw<-150){
                    last_position_x=current_pose->x;
                    last_position_yy=current_pose->y;
                    right_ready_leaking_sweep_status =LEFT_LEAKING_SWEEP_START_WALK_EDGE_LOOP;
                }else{
                    temporary_yaw = Yaw;
                    right_ready_leaking_sweep_status =LEFT_LEAKING_SWEEP_START_WALK_EDGE_LOOP_X_TURN;
                }
                break;
            }
            linear_velocity = long_stra_vel;
            angular_velocity = 0;
            
        }
        
        if(Yaw>75||current_pose->y>last_position_y||my_abs(last_position_y-current_pose->y)>leakingsweep){
            right_ready_leaking_sweep_status = LEFT_LEAKING_SWEEP_CCLOCK_TARGER_YAW_LESS_ABS8;
            break;
        }
        if(my_abs(last_position_xx - current_pose->x)>1000){
            cnt_update=(current_pose->y%half_map_wide+half_map_wide)/100;
            for(i=MAPWIDECELLS-1;i>0;i--){
                if(gridmap.map[i][cnt_update]!=125){
                    break;
                }
            }
            if((current_pose->x+half_map_wide)/100+5>=i){
                right_ready_leaking_sweep_status = LEFT_LEAKING_SWEEP_CCLOCK_TARGER_YAW_LESS_ABS8;
                break;
            }
        }
        break;
    case LEFT_LEAKING_SWEEP_START_WALK_EDGE_LOOP_COLLISION:
        if(turn_start_update == 0)
        {
            turn_start_x = current_pose->x;
            turn_start_y = current_pose->y;
            turn_start_update = 1;
        }
        linear_velocity = -long_stra_vel;
        angular_velocity = 0;
        if (my_abs(turn_start_x - current_pose->x) > collision_backward_distance || my_abs(turn_start_y - current_pose->y) > collision_backward_distance)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            turn_start_update = 0;
            temporary_yaw = Yaw;
            right_ready_leaking_sweep_status = LEFT_LEAKING_SWEEP_START_WALK_EDGE_LOOP_COLLISION_TURN;
            break;
        }
        break;
    case LEFT_LEAKING_SWEEP_START_WALK_EDGE_LOOP_COLLISION_TURN:
        linear_velocity = 0;
        angular_velocity = turn_vel;
        if (my_abs(Yaw - temporary_yaw) > 15)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            last_position_x=current_pose->x;
            last_position_yy=current_pose->y;
            right_ready_leaking_sweep_status =LEFT_LEAKING_SWEEP_START_WALK_EDGE_LOOP;
            break;
        }
        if (obstacleSignal!=none_obstacle||(&cliff_valueB)->cliffValue0 == 1)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            right_ready_leaking_sweep_status = LEFT_LEAKING_SWEEP_START_WALK_EDGE_LOOP_COLLISION_TURN_COLLISION;
            break;
        }
        break;
    case LEFT_LEAKING_SWEEP_START_WALK_EDGE_LOOP_COLLISION_TURN_COLLISION:
        if(turn_start_update == 0)
        {
            turn_start_x = current_pose->x;
            turn_start_y = current_pose->y;
            turn_start_update = 1;
        }
        linear_velocity = -long_stra_vel;
        angular_velocity = 0;
        if (my_abs(turn_start_x - current_pose->x) > collision_backward_distance || my_abs(turn_start_y - current_pose->y) > collision_backward_distance)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            turn_start_update = 0;
            right_ready_leaking_sweep_status = LEFT_LEAKING_SWEEP_START_WALK_EDGE_LOOP_COLLISION_TURN;
            break;
        }
        break;
    case LEFT_LEAKING_SWEEP_START_WALK_EDGE_LOOP_X_TURN:
        linear_velocity = 0;
        angular_velocity =-turn_vel;
        if (my_abs(temporary_yaw - Yaw) > 30||(Yaw<-30))
        {
            linear_velocity = 0;
            angular_velocity = 0;
            last_position_x=current_pose->x;
            last_position_yy=current_pose->y;
            right_ready_leaking_sweep_status =LEFT_LEAKING_SWEEP_START_WALK_EDGE_LOOP;
            break;
        }
        if (obstacleSignal!=none_obstacle||(&cliff_valueB)->cliffValue0 == 1)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            right_ready_leaking_sweep_status = LEFT_LEAKING_SWEEP_START_WALK_EDGE_LOOP_COLLISION;
            break;
        }
        break;
        
    case LEFT_LEAKING_SWEEP_CCLOCK_TARGER_YAW_LESS_ABS8:
        if(my_abs(Yaw)>172){
            right_ready_leaking_sweep_status = LEFT_LEAKING_SWEEP_COMPLETE;
            break;
        }
        if (obstacleSignal!=none_obstacle||(&cliff_valueB)->cliffValue0 == 1)
        {
            linear_velocity=0;
            angular_velocity=0;
            right_ready_leaking_sweep_status = LEFT_LEAKING_SWEEP_CCLOCK_TARGER_YAW_LESS_ABS8_COLLISION;
        }
        if(Yaw>0){
            linear_velocity=0;
            angular_velocity=turn_vel;
        }else{
            linear_velocity=0;
            angular_velocity=-turn_vel;
        }
        break;
    case LEFT_LEAKING_SWEEP_CCLOCK_TARGER_YAW_LESS_ABS8_COLLISION:
        if(turn_start_update == 0)
        {
            turn_start_x = current_pose->x;
            turn_start_y = current_pose->y;
            turn_start_update = 1;
        }
        linear_velocity = -long_stra_vel;
        angular_velocity = 0;
        if (my_abs(turn_start_x - current_pose->x) > collision_backward_distance || my_abs(turn_start_y - current_pose->y) > collision_backward_distance)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            turn_start_update = 0;
            right_ready_leaking_sweep_status = LEFT_LEAKING_SWEEP_CCLOCK_TARGER_YAW_LESS_ABS8;
            break;
        }
        break;
    case LEFT_LEAKING_SWEEP_YAW_OTHER:
        right_ready_leaking_sweep_status = LEFT_LEAKING_SWEEP_CLOCK_TARGET_YAW_MORE_ABS90;
        break;
    case LEFT_LEAKING_SWEEP_CLOCK_TARGET_YAW_MORE_ABS90:
        if(my_abs(Yaw)>85){
            linear_velocity=0;
            angular_velocity=0;
            last_position_y=current_pose->y;
            right_ready_leaking_sweep_status = LEFT_LEAKING_SWEEP_GOSTRAIGHT_OTHER;
            break;
        }
        if (obstacleSignal!=none_obstacle||(&cliff_valueB)->cliffValue0 == 1)
        {
            linear_velocity=0;
            angular_velocity=0;
            right_ready_leaking_sweep_status = LEFT_LEAKING_SWEEP_CLOCK_TARGET_YAW_MORE_ABS90_COLLISION;
        }
        linear_velocity=0;
        angular_velocity=-turn_vel;
        break;
    case LEFT_LEAKING_SWEEP_CLOCK_TARGET_YAW_MORE_ABS90_COLLISION:
        if(turn_start_update == 0)
        {
            turn_start_x = current_pose->x;
            turn_start_y = current_pose->y;
            turn_start_update = 1;
        }
        linear_velocity = -long_stra_vel;
        angular_velocity = 0;
        if (my_abs(turn_start_x - current_pose->x) > collision_backward_distance || my_abs(turn_start_y - current_pose->y) > collision_backward_distance)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            turn_start_update = 0;
            right_ready_leaking_sweep_status = LEFT_LEAKING_SWEEP_CLOCK_TARGET_YAW_MORE_ABS90;
            break;
        }
        break;
    case LEFT_LEAKING_SWEEP_GOSTRAIGHT_OTHER:
        linear_velocity=long_stra_vel;
        angular_velocity=0;
        if(obstacleSignal!=none_obstacle||(&cliff_valueB)->cliffValue0 == 1){
            last_position_xx=current_pose->x;
            right_ready_leaking_sweep_status = LEFT_LEAKING_SWEEP_OTHER_START_WALK_EDGE;
            break;
        }
        if(my_abs(last_position_y-current_pose->y)>leakingsweep){
            right_ready_leaking_sweep_status = LEFT_LEAKING_SWEEP_CLOCK_TARGET_YAW_MORE_ABS173;
            break;
        }
        break;
    case LEFT_LEAKING_SWEEP_OTHER_START_WALK_EDGE:
        linear_velocity = 0;
        angular_velocity = 0;
        last_position_x = current_pose->x;
        last_position_yy = current_pose->y;
        right_ready_leaking_sweep_status = LEFT_LEAKING_SWEEP_OTHER_START_WALK_EDGE_LOOP;
        break;
        
    case LEFT_LEAKING_SWEEP_OTHER_START_WALK_EDGE_LOOP:
        if(Yaw<-30&&Yaw>-150){
			left_edge_judgment_repeat();
//            if(adcRealTime[8]>100&&adcRealTime[8]<1500){
//                delimma_edge=0;
//                linear_velocity = 200;
//                angular_velocity = 0;
//            }
//            if(adcRealTime[8]>=1500){
//                delimma_edge=0;
//                linear_velocity = 200;
//                angular_velocity = -10;
//            }
//            if(adcRealTime[8]<100){
//                if(delimma_edge<10){
//                    delimma_edge++;
//                    linear_velocity = 200;
//                    angular_velocity = 10;
//                }
//                else{
//                    linear_velocity = 100;
//                    angular_velocity = 20;
//                }
//                
//            }
        }
        else{
            if (my_abs(last_position_x - current_pose->x) > lateral_move_distance||my_abs(last_position_yy - current_pose->y) > lateral_move_distance)
            {
                if(Yaw>-30){
                    last_position_x=current_pose->x;
                    last_position_yy=current_pose->y;
                    right_ready_leaking_sweep_status =LEFT_LEAKING_SWEEP_OTHER_START_WALK_EDGE_LOOP;
                }else{
                    temporary_yaw = Yaw;
                    right_ready_leaking_sweep_status =LEFT_LEAKING_SWEEP_OTHER_START_WALK_EDGE_LOOP_X_TURN;
                }
                break;
            }
            linear_velocity = long_stra_vel;
            angular_velocity = 0;
            
        }
        if (obstacleSignal!=none_obstacle||(&cliff_valueB)->cliffValue0 == 1)
        {
            right_ready_leaking_sweep_status = LEFT_LEAKING_SWEEP_OTHER_START_WALK_EDGE_LOOP_COLLISION;
            break;
        }
        
        if((Yaw<105&&Yaw>0)||current_pose->y>last_position_y||my_abs(last_position_y-current_pose->y)>leakingsweep){
            right_ready_leaking_sweep_status = LEFT_LEAKING_SWEEP_CLOCK_TARGET_YAW_MORE_ABS173;
            break;
        }
        if(my_abs(last_position_xx - current_pose->x)>1000){
            cnt_update=(current_pose->y%half_map_wide+half_map_wide)/100;
            for(i=0;i<MAPWIDECELLS;i++){
                if(gridmap.map[i][cnt_update]!=125){
                    break;
                }
            }
            if((current_pose->x+half_map_wide)/100-5<=i){
                right_ready_leaking_sweep_status = LEFT_LEAKING_SWEEP_CLOCK_TARGET_YAW_MORE_ABS173;
                break;
            }
        }
        break;
        
    case LEFT_LEAKING_SWEEP_OTHER_START_WALK_EDGE_LOOP_COLLISION:
        if(turn_start_update == 0)
        {
            turn_start_x = current_pose->x;
            turn_start_y = current_pose->y;
            turn_start_update = 1;
        }
        linear_velocity = -long_stra_vel;
        angular_velocity = 0;
        if (my_abs(turn_start_x - current_pose->x) > collision_backward_distance || my_abs(turn_start_y - current_pose->y) > collision_backward_distance)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            turn_start_update = 0;
            temporary_yaw = Yaw;
            right_ready_leaking_sweep_status = LEFT_LEAKING_SWEEP_OTHER_START_WALK_EDGE_LOOP_COLLISION_TURN;
            break;
        }
        break;
    case LEFT_LEAKING_SWEEP_OTHER_START_WALK_EDGE_LOOP_COLLISION_TURN:
        linear_velocity = 0;
        angular_velocity = -turn_vel;
        if (my_abs(Yaw - temporary_yaw) > 15)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            last_position_x=current_pose->x;
            last_position_yy=current_pose->y;
            right_ready_leaking_sweep_status =LEFT_LEAKING_SWEEP_OTHER_START_WALK_EDGE_LOOP;
            break;
        }
        if (obstacleSignal!=none_obstacle||(&cliff_valueB)->cliffValue0 == 1)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            right_ready_leaking_sweep_status = LEFT_LEAKING_SWEEP_OTHER_START_WALK_EDGE_LOOP_COLLISION_TURN_COLLISION;
            break;
        }
        break;
    case LEFT_LEAKING_SWEEP_OTHER_START_WALK_EDGE_LOOP_COLLISION_TURN_COLLISION:
        if(turn_start_update == 0)
        {
            turn_start_x = current_pose->x;
            turn_start_y = current_pose->y;
            turn_start_update = 1;
        }
        linear_velocity = -long_stra_vel;
        angular_velocity = 0;
        if (my_abs(turn_start_x - current_pose->x) > collision_backward_distance || my_abs(turn_start_y - current_pose->y) > collision_backward_distance)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            turn_start_update = 0;
            right_ready_leaking_sweep_status = LEFT_LEAKING_SWEEP_OTHER_START_WALK_EDGE_LOOP_COLLISION_TURN;
            break;
        }
        break;
    case LEFT_LEAKING_SWEEP_OTHER_START_WALK_EDGE_LOOP_X_TURN:
        linear_velocity = 0;
        angular_velocity =turn_vel;
        if (my_abs(temporary_yaw - Yaw) > 30||(Yaw>-150)){
            linear_velocity = 0;
            angular_velocity = 0;
            last_position_x=current_pose->x;
            last_position_yy=current_pose->y;
            right_ready_leaking_sweep_status =LEFT_LEAKING_SWEEP_OTHER_START_WALK_EDGE_LOOP;
            break;
        }
        if (obstacleSignal!=none_obstacle||(&cliff_valueB)->cliffValue0 == 1)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            right_ready_leaking_sweep_status =LEFT_LEAKING_SWEEP_OTHER_START_WALK_EDGE_LOOP_COLLISION;
            break;
        }
        break;
        
    case LEFT_LEAKING_SWEEP_CLOCK_TARGET_YAW_MORE_ABS173:
        if(my_abs(Yaw)<8){
            right_ready_leaking_sweep_status = LEFT_LEAKING_SWEEP_COMPLETE;
            break;
        }
        if (obstacleSignal!=none_obstacle||(&cliff_valueB)->cliffValue0 == 1)
        {
            linear_velocity=0;
            angular_velocity=0;
            right_ready_leaking_sweep_status = LEFT_LEAKING_SWEEP_CLOCK_TARGET_YAW_MOEE_ABS173_COLLISIION;
        }
        if(Yaw>0){
            linear_velocity=0;
            angular_velocity=-turn_vel;
        }else{
            linear_velocity=0;
            angular_velocity=turn_vel;
        }
        break;
    case LEFT_LEAKING_SWEEP_CLOCK_TARGET_YAW_MOEE_ABS173_COLLISIION:
        if(turn_start_update == 0)
        {
            turn_start_x = current_pose->x;
            turn_start_y = current_pose->y;
            turn_start_update = 1;
        }
        linear_velocity = -long_stra_vel;
        angular_velocity = 0;
        if (my_abs(turn_start_x - current_pose->x) > collision_backward_distance || my_abs(turn_start_y - current_pose->y) > collision_backward_distance)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            turn_start_update = 0;
            right_ready_leaking_sweep_status = LEFT_LEAKING_SWEEP_CLOCK_TARGET_YAW_MORE_ABS173;
            break;
        }
        break;
    case LEFT_LEAKING_SWEEP_COMPLETE:
        right_ready_leaking_sweep_status = 0;
        complete_flag = 1;
        break;
    }
    return complete_flag;
}



unsigned char StuckLeftRunStep(POSE *current_pose,unsigned char obstacleSignal){
    int Yaw;
    unsigned char complete_flag=0;
    Yaw = current_pose->orientation;
    Yaw = Yaw/100;
    switch(stuck_right_run_step){
    case 0:
        stuck_right_run_step = LEFT_STUCK_FORWARD_BOUNDARY_STATUS;
        break;
    case LEFT_STUCK_FORWARD_BOUNDARY_STATUS:
        if(turn_start_update == 0)
        {
            turn_start_x = current_pose->x;
            turn_start_y = current_pose->y;
            turn_start_update = 1;
        }
        linear_velocity = -long_stra_vel;
        angular_velocity = 0;
        if (my_abs(turn_start_x - current_pose->x) > lateral_move_distance || my_abs(turn_start_y - current_pose->y) > lateral_move_distance)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            turn_start_update = 0;
            temporary_yaw=Yaw;
            stuck_right_run_step = LEFT_TURN_RIGHT_YAW_MORE_90_STUCK_FORWARD_BOUNDARY_STATUS;
            break;
        }
        break;
    case LEFT_TURN_RIGHT_YAW_MORE_90_STUCK_FORWARD_BOUNDARY_STATUS:
        if(my_abs(Yaw- temporary_yaw)>90){
            linear_velocity=0;
            angular_velocity=0;
            last_position_x=current_pose->x;
            last_position_y=current_pose->y;
            stuck_right_run_step = LEFT_GO_STUCK_FORWARD_BOUNDARY_STATUS;
            break;
        }
        if (obstacleSignal!=none_obstacle||(&cliff_valueB)->cliffValue0 == 1)
        {
            linear_velocity=0;
            angular_velocity=0;
            stuck_right_run_step = LEFT_COLLISION_TURN_RIGHT_YAW_MORE_90_STUCK_FORWARD_BOUNDARY_STATUS;
        }
        linear_velocity=0;
        angular_velocity=-turn_vel;
        break;
    case LEFT_COLLISION_TURN_RIGHT_YAW_MORE_90_STUCK_FORWARD_BOUNDARY_STATUS:
        if(turn_start_update == 0)
        {
            turn_start_x = current_pose->x;
            turn_start_y = current_pose->y;
            turn_start_update = 1;
        }
        linear_velocity = -long_stra_vel;
        angular_velocity = 0;
        if (my_abs(turn_start_x - current_pose->x) > collision_backward_distance|| my_abs(turn_start_y - current_pose->y) > collision_backward_distance)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            turn_start_update = 0;
            stuck_right_run_step = LEFT_TURN_RIGHT_YAW_MORE_90_STUCK_FORWARD_BOUNDARY_STATUS;
            break;
        }
        break;
    case LEFT_GO_STUCK_FORWARD_BOUNDARY_STATUS:
        linear_velocity = long_stra_vel;
        angular_velocity =0;
        if (obstacleSignal!=none_obstacle||(&cliff_valueB)->cliffValue0 == 1)
        {
            stuck_right_run_step = LEFT_COLLISION_GO_STUCK_FORWARD_BOUNDARY_STATUS;
            break;
        }
        if (my_abs(last_position_x - current_pose->x) > lateral_move_distance||my_abs(last_position_y - current_pose->y) > lateral_move_distance)
        {
            temporary_yaw=Yaw;
            linear_velocity=0;
            angular_velocity=0;
            stuck_right_run_step = LEFT_GO_X_Y_MORE_STUCK_FORWARD_BOUNDARY_STATUS;
            break;
        }
        break;
    case LEFT_COLLISION_GO_STUCK_FORWARD_BOUNDARY_STATUS:
        if(turn_start_update == 0)
        {
            turn_start_x = current_pose->x;
            turn_start_y = current_pose->y;
            turn_start_update = 1;
        }
        linear_velocity = -long_stra_vel;
        angular_velocity = 0;
        if (my_abs(turn_start_x - current_pose->x) > collision_backward_distance|| my_abs(turn_start_y - current_pose->y) > collision_backward_distance)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            turn_start_update = 0;
            temporary_yaw=Yaw;
            stuck_right_run_step = LEFT_TURN_RIGHT_YAW_MORE_10_STUCK_FORWARD_BOUNDARY_STATUS;
            break;
        }
        break;
    case LEFT_TURN_RIGHT_YAW_MORE_10_STUCK_FORWARD_BOUNDARY_STATUS:
        if(my_abs(Yaw- temporary_yaw)>10){
            linear_velocity=0;
            angular_velocity=0;
            stuck_right_run_step = LEFT_GO_STUCK_FORWARD_BOUNDARY_STATUS;
            break;
        }
        if (obstacleSignal!=none_obstacle||(&cliff_valueB)->cliffValue0 == 1)
        {
            linear_velocity=0;
            angular_velocity=0;
            stuck_right_run_step = LEFT_COLLISION_TURN_RIGHT_YAW_MORE_10_STUCK_FORWARD_BOUNDARY_STATUS;
        }
        linear_velocity=0;
        angular_velocity=-turn_vel;
        break;
    case LEFT_COLLISION_TURN_RIGHT_YAW_MORE_10_STUCK_FORWARD_BOUNDARY_STATUS:
        if(turn_start_update == 0)
        {
            turn_start_x = current_pose->x;
            turn_start_y = current_pose->y;
            turn_start_update = 1;
        }
        linear_velocity = -long_stra_vel;
        angular_velocity = 0;
        if (my_abs(turn_start_x - current_pose->x) > collision_backward_distance|| my_abs(turn_start_y - current_pose->y) > collision_backward_distance)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            turn_start_update = 0;
            stuck_right_run_step = LEFT_TURN_RIGHT_YAW_MORE_10_STUCK_FORWARD_BOUNDARY_STATUS;
            break;
        }
        break;
    case LEFT_GO_X_Y_MORE_STUCK_FORWARD_BOUNDARY_STATUS:
        if(my_abs(Yaw- temporary_yaw)>90){
            linear_velocity=0;
            angular_velocity=0;
            last_position_x=current_pose->x;
            last_position_y=current_pose->y;
            stuck_right_run_step = LEFT_ESCAPE_FROM_STUCK_FORWARD_BOUNDARY_STATUS;
            break;
        }
        if (obstacleSignal!=none_obstacle||(&cliff_valueB)->cliffValue0 == 1 )
        {
            linear_velocity=0;
            angular_velocity=0;
            temporary_yaw=Yaw;
            stuck_right_run_step = LEFT_COLLISION_GO_X_Y_MORE_STUCK_FORWARD_BOUNDARY_STATUS;
        }
        linear_velocity=0;
        angular_velocity=-turn_vel;
        break;
    case LEFT_COLLISION_GO_X_Y_MORE_STUCK_FORWARD_BOUNDARY_STATUS:
        if(turn_start_update == 0)
        {
            turn_start_x = current_pose->x;
            turn_start_y = current_pose->y;
            turn_start_update = 1;
        }
        linear_velocity = -long_stra_vel;
        angular_velocity = 0;
        if (my_abs(turn_start_x - current_pose->x) > collision_backward_distance|| my_abs(turn_start_y - current_pose->y) > collision_backward_distance)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            turn_start_update = 0;
            stuck_right_run_step = LEFT_GO_X_Y_MORE_STUCK_FORWARD_BOUNDARY_STATUS;
            break;
        }
        break;
    case LEFT_ESCAPE_FROM_STUCK_FORWARD_BOUNDARY_STATUS:
        linear_velocity = long_stra_vel;
        angular_velocity =0;
        if (obstacleSignal!=none_obstacle||(&cliff_valueB)->cliffValue0 == 1)
        {
            stuck_right_run_step = LEFT_COLLISION_ESCAPE_FROM_STUCK_FORWARD_BOUNDARY_STATUS;
            break;
        }
        if (my_abs(last_position_x - current_pose->x) > lateral_move_distance||my_abs(last_position_y - current_pose->y) > lateral_move_distance)
        {
            linear_velocity=0;
            angular_velocity=0;
            stuck_right_run_step = LEFT_COMPETE_ESCAPE_FROM_STUCK_FORWARD_BOUNDARY_STATUS;
            break;
        }
        break;
    case LEFT_COLLISION_ESCAPE_FROM_STUCK_FORWARD_BOUNDARY_STATUS:
        if(turn_start_update == 0)
        {
            turn_start_x = current_pose->x;
            turn_start_y = current_pose->y;
            turn_start_update = 1;
        }
        linear_velocity = -long_stra_vel;
        angular_velocity = 0;
        if (my_abs(turn_start_x - current_pose->x) > collision_backward_distance|| my_abs(turn_start_y - current_pose->y) > collision_backward_distance)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            turn_start_update = 0;
            temporary_yaw=Yaw;
            stuck_right_run_step = LEFT_TURN_COLLISION_ESCAPE_FROM_STUCK_FORWARD_BOUNDARY_STATUS;
            break;
        }
        break;
    case LEFT_TURN_COLLISION_ESCAPE_FROM_STUCK_FORWARD_BOUNDARY_STATUS:
        if(my_abs(Yaw- temporary_yaw)>10){
            linear_velocity=0;
            angular_velocity=0;
            stuck_right_run_step = LEFT_ESCAPE_FROM_STUCK_FORWARD_BOUNDARY_STATUS;
            break;
        }
        if (obstacleSignal!=none_obstacle||(&cliff_valueB)->cliffValue0 == 1)
        {
            linear_velocity=0;
            angular_velocity=0;
            stuck_right_run_step = LEFT_COLLISION_TURN_COLLISION_ESCAPE_FROM_STUCK_FORWARD_BOUNDARY_STATUS;
        }
        linear_velocity=0;
        angular_velocity=turn_vel;
        break;
    case LEFT_COLLISION_TURN_COLLISION_ESCAPE_FROM_STUCK_FORWARD_BOUNDARY_STATUS:
        if(turn_start_update == 0)
        {
            turn_start_x = current_pose->x;
            turn_start_y = current_pose->y;
            turn_start_update = 1;
        }
        linear_velocity = -long_stra_vel;
        angular_velocity = 0;
        if (my_abs(turn_start_x - current_pose->x) > collision_backward_distance|| my_abs(turn_start_y - current_pose->y) > collision_backward_distance)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            turn_start_update = 0;
            stuck_right_run_step = LEFT_TURN_COLLISION_ESCAPE_FROM_STUCK_FORWARD_BOUNDARY_STATUS;
            break;
        }
        break;
    case LEFT_COMPETE_ESCAPE_FROM_STUCK_FORWARD_BOUNDARY_STATUS:
        linear_velocity=0;
        angular_velocity=0;
        if(stuck==false){
            stuck=true;
        }
        else{
            stuck_right_run_step = 0;
            complete_flag = 1;
        }
        break;
    }
    return complete_flag;
}





void StuckRunStep(POSE *current_pose){
    signed char i,j,k;
    if(stuck_x==true){
        if(current_pose->x>0){
            for(i=0;i<MAPLONGCELLS;i++){
                k=0;
                for(j=0;j<MAPWIDECELLS;j++){
                    if(gridmap.map[i][j]!=125){
                        k++;
                    }
                }
                if(k>=10){
                    x_error=100*i-half_map_wide;
                    x_error=current_pose->x-x_error;
                    break;
                }
            }
        }
        else{
            for(i=MAPLONGCELLS-1;i>=0;i--){
                k=0;
                for(j=MAPWIDECELLS-1;j>=0;j--){
                    if(gridmap.map[i][j]!=125){
                        k++;
                    }
                }
                if(k>=10){
                    x_error=100*i-half_map_wide;
                    x_error=current_pose->x-x_error;
                    break;
                }
            }
        }
        for(i=0;i<MAPLONGCELLS;i++){
            k=0;
            for(j=0;j<MAPWIDECELLS;j++){
                if(gridmap.map[j][i]!=125){
                    k++;
                }
            }
            if(k>=10){
                for(j=0;j<MAPWIDECELLS;j++){
                    gridmap.map[j][i]=250;
                }
            }
        }
    }
    if(stuck_y==true){
        if(current_pose->y<0){
            for(i=0;i<MAPLONGCELLS;i++){
                k=0;
                for(j=0;j<MAPWIDECELLS;j++){
                    if(gridmap.map[j][i]!=125){
                        k++;
                    }
                }
                if(k>=10){
                    y_error=100*i-half_map_wide;
                    y_error=current_pose->y-y_error;
                    break;
                }
            }
        }
        else{
            for(i=MAPLONGCELLS-1;i>=0;i--){
                k=0;
                for(j=MAPWIDECELLS-1;j>=0;j--){
                    if(gridmap.map[j][i]!=125){
                        k++;
                    }
                }
                if(k>=10){
                    y_error=100*i-half_map_wide;
                    y_error=current_pose->y-y_error;
                    break;
                }
            }
        }
        for(i=0;i<MAPLONGCELLS;i++){
            k=0;
            for(j=0;j<MAPWIDECELLS;j++){
                if(gridmap.map[i][j]!=125){
                    k++;
                }
            }
            if(k>=10){
                for(j=0;j<MAPWIDECELLS;j++){
                    gridmap.map[j][i]=0;
                }
            }
        }
    }
    stuck_x=false;
    stuck_y=false;
}



//##############A* return origin function define###################################
//#################################################################################
unsigned char ForceReturnOrigin(POSE *current_pose,unsigned char obstacleSignal){
    int Yaw;
    unsigned char complete_flag=0;
    Yaw = current_pose->orientation;
    Yaw = Yaw/100;
    unsigned char i;
    if((xTaskGetTickCount() - ForceReturnOriginTimeStamp)> EDGEWISE_CLEAN_WORK_TIME){
        return 1;
    }
    switch(return_origin_step_status)
    {
    case 0:
        origin_last_position_x=current_pose->x;
        origin_last_position_y=current_pose->y;
        return_origin_step_status = GOSTR_RETURN_ORIGIN_STEP;
        break;
    case GOSTR_RETURN_ORIGIN_STEP:
        if(over_clean_finish == true){
            current_pose->x=global_pose_x;
            current_pose->y=global_pose_y;
            current_pose->x=current_pose->x-x_error;
            current_pose->y=current_pose->y-y_error;
        }
        if(y_less_map==true){
            current_pose->y=global_pose_y;
            current_pose->y=current_pose->y-y_error;
        }
        if(over_clean_finish==true){
            if(my_abs(current_pose->x)>2*return_origin_distance||my_abs(current_pose->y)>2*return_origin_distance){
                linear_velocity=0;
                angular_velocity=0;
                if((current_pose->x>=0&&current_pose->y>=0)||((current_pose->x<=0&&current_pose->y<=0))){
                    if(current_pose->x>=0){
                        DelimmaNumber=1;
                    }else{
                        DelimmaNumber=2;
                    }
                    return_origin_step_status=DIR_X_MORE_POSITIVE_200;
                }else{
                    if(current_pose->x>=0){
                        DelimmaNumber=3;
                    }else{
                        DelimmaNumber=4;
                    }
                    return_origin_step_status=DIR_X_LESS_NEGATIVE_200;
                }
                thephi=57*atan2(current_pose->y,current_pose->x);
                if(thephi<=0){
                    thephi=thephi+180;
                }else{
                    thephi=thephi-180;
                }
                origin_thephi=thephi;
                leakingsweep=1;
                edge_dilemma=false;
            }
            else{
                if(my_abs(Yaw)>10){
                    if(Yaw>0){
                        linear_velocity=0;
                        angular_velocity=-turn_vel;
                    }
                    else{
                        linear_velocity=0;
                        angular_velocity=turn_vel;
                    }
                }
                else{
                    linear_velocity=0;
                    angular_velocity=0;
                    return_origin_step_status=0;
                    DelimmaNumber=0;
                    step=0;
                    leakingsweep=0;
                    cnt_update=0;
                    bow_continue=false;
                    edge_dilemma=false;
                    complete_flag = 1;
                }
            }
        }
        else{
            if(my_abs(current_pose->x)>return_origin_distance||my_abs(current_pose->y)>return_origin_distance){
                if((current_pose->x>=0&&current_pose->y>=0)||((current_pose->x<=0&&current_pose->y<=0))){
                    if(current_pose->x>=0){
                        DelimmaNumber=1;
                    }else{
                        DelimmaNumber=2;
                    }
                    return_origin_step_status=DIR_X_MORE_POSITIVE_200;
                }else{
                    if(current_pose->x>=0){
                        DelimmaNumber=3;
                    }else{
                        DelimmaNumber=4;
                    }
                    return_origin_step_status=DIR_X_LESS_NEGATIVE_200;
                }
                thephi=57*atan2(current_pose->y,current_pose->x);
                if(thephi<=0){
                    thephi=thephi+180;
                }else{
                    thephi=thephi-180;
                }
                origin_thephi=thephi;
                leakingsweep=1;
                edge_dilemma=false;
            }
            else{
                step=0;
                leakingsweep=0;
                DelimmaNumber=0;
                cnt_update=0;
                return_origin_step_status=0;
                bow_continue=false;
                edge_dilemma=false;
                complete_flag = 1;
            }
			linear_velocity=0;
			angular_velocity=0;
        }
        break;
    case DIR_X_MORE_POSITIVE_200:
        if(over_clean_finish == true){
            current_pose->x=global_pose_x;
            current_pose->y=global_pose_y;
            current_pose->x=current_pose->x-x_error;
            current_pose->y=current_pose->y-y_error;
        }
        if(y_less_map==true){
            current_pose->y=global_pose_y;
            current_pose->y=current_pose->y-y_error;
        }
        thephi=180*atan2(current_pose->y,current_pose->x)/3.1415926;
        if(thephi<=0){
            thephi=thephi+180;
        }else{
            thephi=thephi-180;
        }
        thephi=thephi-Yaw;
        if(my_abs(thephi)>180){
            if(thephi>180){
                thephi=thephi-360;
            }else{
                thephi=thephi+360;
            }
        }
        if(thephi>0){
            return_origin_positive_start=1;
        }else{
            return_origin_positive_start=-1;
        }
        return_origin_step_status=DIR_Y_MORE_POSITIVE_200;
        break;
        
    case DIR_Y_MORE_POSITIVE_200:
        if(obstacleSignal!=none_obstacle||(&cliff_valueB)->cliffValue0 == 1){
            linear_velocity=0;
            angular_velocity=0;
            return_origin_step_status = TURN_CLOCK_DIR_Y_MORE_POSITIVE_200;
            break;
        }
        if(my_abs(origin_thephi)>177){
            if(my_abs(Yaw)>177){
                linear_velocity=0;
                angular_velocity=0;
                last_position_x=current_pose->x;
                last_position_y=current_pose->y;
                return_origin_step_status=TURN_CLOCK_TARGET_YAW_POSITIVE_90_RETURN_ORIGIN;
                break;
            }
        }
        else{
            if(origin_thephi-3<=Yaw&&Yaw<=origin_thephi+3){
                linear_velocity=0;
                angular_velocity=0;
                last_position_x=current_pose->x;
                last_position_y=current_pose->y;
                return_origin_step_status=TURN_CLOCK_TARGET_YAW_POSITIVE_90_RETURN_ORIGIN;
                break;
            }
        }
        linear_velocity=0;
        angular_velocity=return_origin_positive_start*turn_vel;
        break;
    case TURN_CLOCK_DIR_Y_MORE_POSITIVE_200:
        if(turn_start_update == 0)
        {
            turn_start_x = current_pose->x;
            turn_start_y = current_pose->y;
            turn_start_update = 1;
        }
        linear_velocity = -long_stra_vel;
        angular_velocity = 0;
        if (my_abs(turn_start_x - current_pose->x) > collision_backward_distance || my_abs(turn_start_y - current_pose->y) > collision_backward_distance)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            turn_start_update = 0;
            return_origin_step_status =DIR_Y_MORE_POSITIVE_200;
        }
        break;
    case TURN_CLOCK_TARGET_YAW_POSITIVE_90_RETURN_ORIGIN:
        if (obstacleSignal!=none_obstacle||(&cliff_valueB)->cliffValue0 == 1)
        {
            step=0;
            bow_continue=false;
            return_origin_step_status = COLLISION_Y_POSITIVE_90_RETURN_ORIGIN;
            break;
        }
        if(sqrt(pow(last_position_x, 2) + pow(last_position_y, 2))<sqrt(pow(current_pose->x, 2) + pow(current_pose->y, 2))-20){
            linear_velocity = 0;
            angular_velocity = 0;
            step=0;
            return_origin_step_status = GOSTR_RETURN_ORIGIN_STEP;
        }
        step++;
        if(step>50){
            step=0;
            last_position_x=current_pose->x;
            last_position_y=current_pose->y;
        }
        if(bow_continue==false){
            if(my_abs(current_pose->x)<500&&my_abs(current_pose->y)<500){
                bow_continue=true;
                linear_velocity = 0;
                angular_velocity = 0;
                step=0;
                return_origin_step_status = DIR_X_MORE_POSITIVE_200;
            }
        }
        if(over_clean_finish == true){
            current_pose->x=global_pose_x;
            current_pose->y=global_pose_y;
            current_pose->x=current_pose->x-x_error;
            current_pose->y=current_pose->y-y_error;
        }
        if(y_less_map==true){
            current_pose->y=global_pose_y;
            current_pose->y=current_pose->y-y_error;
        }
        if (my_abs(current_pose->x)<return_origin_distance&&my_abs(current_pose->y)<return_origin_distance)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            step=0;
            return_origin_step_status = GOSTR_RETURN_ORIGIN_STEP;
            break;
        }
        
        linear_velocity = long_stra_vel;
        angular_velocity = 0;
        break;
    case COLLISION_Y_POSITIVE_90_RETURN_ORIGIN:
        if(turn_start_update == 0)
        {
            turn_start_x = current_pose->x;
            turn_start_y = current_pose->y;
            turn_start_update = 1;
        }
        linear_velocity = -long_stra_vel;
        angular_velocity = 0;
        if (my_abs(turn_start_x - current_pose->x) > side_backward_distance || my_abs(turn_start_y - current_pose->y) > side_backward_distance)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            turn_start_update = 0;
            temporary_yaw = Yaw;
            return_origin_step_status =TURN_CLOCK_Y_POSITIVE_90_RETURN_ORIGIN;
        }
        break;
        
    case TURN_CLOCK_Y_POSITIVE_90_RETURN_ORIGIN:
        if (obstacleSignal!=none_obstacle||(&cliff_valueB)->cliffValue0 == 1)
        {
            return_origin_step_status = COLLISION_Y_POSITIVE_90_RETURN_ORIGIN;
            break;
        }
        if (my_abs(Yaw - temporary_yaw) > 15)
        {
            thephi=origin_thephi-Yaw;
            if(my_abs(thephi)>180){
                if(thephi>180){
                    thephi=thephi-360;
                }else{
                    thephi=thephi+360;
                }
            }
            if(my_abs(thephi)>90){
                cnt_update=(current_pose->y%half_map_wide+half_map_wide)/100;
                if((DelimmaNumber==1&&leakingsweep==1)||(DelimmaNumber==2&&leakingsweep==-1)){
                    for(i=0;i<MAPWIDECELLS;i++){
                        if(gridmap.map[i][cnt_update]!=125){
                            break;
                        }
                    }
                    if((current_pose->x+half_map_wide)/100-i<=5){
                        if(over_clean_finish==false&&leakingsweep==-1){
                            return_origin_step_status =TURN_CLOCK_COLLISION_Y_POSITIVE_90_RETURN_ORIGIN;
                            break;
                        }
                        if(over_clean_finish==true&&leakingsweep==-1){
                            if(edge_dilemma==true){
                                edge_dilemma=false;
                                complete_flag=2;
                            }else{
                                return_origin_step_status = BYPASS_OBSTACLES_Y_POSITIVE_90_RETURN_ORIGIN;
                                leakingsweep=1;
                                edge_dilemma=true;
                            }
                            break;
                        }
                        leakingsweep=-1;
                    }
                }
                if((DelimmaNumber==2&&leakingsweep==1)||(DelimmaNumber==1&&leakingsweep==-1)){
                    for(i=MAPWIDECELLS-1;i>0;i--){
                        if(gridmap.map[i][cnt_update]!=125){
                            break;
                        }
                    }
                    if((current_pose->x+half_map_wide)/100+5>=i){
                        if(over_clean_finish==false&&leakingsweep==-1){
                            return_origin_step_status =TURN_CLOCK_COLLISION_Y_POSITIVE_90_RETURN_ORIGIN;
                            break;
                        }
                        if(over_clean_finish==true&&leakingsweep==-1){
                            if(edge_dilemma==true){
                                complete_flag=2;
                                edge_dilemma=false;
                            }else{
                                return_origin_step_status = BYPASS_OBSTACLES_Y_POSITIVE_90_RETURN_ORIGIN;
                                leakingsweep=1;
                                edge_dilemma=true;
                            }
                            break;
                        }
                        leakingsweep=-1;
                    }
                }
            }
            return_origin_step_status = BYPASS_OBSTACLES_Y_POSITIVE_90_RETURN_ORIGIN;
            break;
        }
        linear_velocity = 0;
        angular_velocity =-leakingsweep*turn_vel;
        break;
        
    case TURN_CLOCK_COLLISION_Y_POSITIVE_90_RETURN_ORIGIN:
        linear_velocity=0;
        angular_velocity=0;
        leakingsweep=0;
        DelimmaNumber=0;
        cnt_update=0;
        return_origin_step_status=0;
        step=0;
        bow_continue=false;
        edge_dilemma=false;
        complete_flag = 1;
        break;
    case BYPASS_OBSTACLES_Y_POSITIVE_90_RETURN_ORIGIN:
        last_position_yy = current_pose->y;
        last_position_xx= current_pose->x;
        return_origin_step_status =LOOP_BYPASS_OBSTACLES_Y_POSITIVE_90_RETURN_ORIGIN;
        break;
    case LOOP_BYPASS_OBSTACLES_Y_POSITIVE_90_RETURN_ORIGIN:
        if (obstacleSignal!=none_obstacle||(&cliff_valueB)->cliffValue0 == 1)
        {
            temporary_yaw=Yaw;
            return_origin_step_status = COLLISION_Y_POSITIVE_90_RETURN_ORIGIN;
            break;
        }
        if (my_abs(last_position_yy - current_pose->y) > lateral_move_distance/2||my_abs(last_position_xx - current_pose->x) > lateral_move_distance/2)
        {
            temporary_yaw=Yaw;
            return_origin_step_status = COLLISION_LOOP_BYPASS_OBSTACLES_Y_POSITIVE_90_RETURN_ORIGIN;
            break;
        }
        if(over_clean_finish == true){
            current_pose->x=global_pose_x;
            current_pose->y=global_pose_y;
            current_pose->x=current_pose->x-x_error;
            current_pose->y=current_pose->y-y_error;
        }
        if(y_less_map==true){
            current_pose->y=global_pose_y;
            current_pose->y=current_pose->y-y_error;
        }
        if (my_abs(current_pose->x)<return_origin_distance&&my_abs(current_pose->y)<return_origin_distance)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            return_origin_step_status = GOSTR_RETURN_ORIGIN_STEP;
            break;
        }
        linear_velocity = long_stra_vel;
        angular_velocity = 0;
        break;
        
    case COLLISION_LOOP_BYPASS_OBSTACLES_Y_POSITIVE_90_RETURN_ORIGIN:
        if (obstacleSignal!=none_obstacle||(&cliff_valueB)->cliffValue0 == 1)
        {
            return_origin_step_status = COLLISION_Y_POSITIVE_90_RETURN_ORIGIN;
            break;
        }
        if (my_abs(Yaw - temporary_yaw) >60)
        {
            return_origin_step_status = Y_MORE_LOOP_BYPASS_OBSTACLES_Y_POSITIVE_90_RETURN_ORIGIN;
            break;
        }
        if(origin_thephi-5<=Yaw&&Yaw<=origin_thephi+5){
            return_origin_step_status = DIR_X_MORE_POSITIVE_200;
            break;
        }
        linear_velocity = 0;
        angular_velocity = leakingsweep*turn_vel;
        break;
    case Y_MORE_LOOP_BYPASS_OBSTACLES_Y_POSITIVE_90_RETURN_ORIGIN:
        if (obstacleSignal!=none_obstacle||(&cliff_valueB)->cliffValue0 == 1)
        {
            return_origin_step_status = COLLISION_Y_NEGATIVE_90_RETURN_ORIGIN;
            break;
        }
        if (my_abs(last_position_yy - current_pose->y) > lateral_move_distance||my_abs(last_position_xx - current_pose->x) > lateral_move_distance)
        {
            thephi=origin_thephi-Yaw;
            if(my_abs(thephi)>180){
                if(thephi>180){
                    thephi=thephi-360;
                }else{
                    thephi=thephi+360;
                }
            }
            if(my_abs(thephi)<30){
                return_origin_step_status = DIR_X_MORE_POSITIVE_200;
                break;
            }else{
                return_origin_step_status = BYPASS_OBSTACLES_Y_POSITIVE_90_RETURN_ORIGIN;
                break;
            }
        }
        linear_velocity = long_stra_vel;
        angular_velocity = 0;
        break;
    case DIR_X_LESS_NEGATIVE_200:
        if(over_clean_finish == true){
            current_pose->x=global_pose_x;
            current_pose->y=global_pose_y;
            current_pose->x=current_pose->x-x_error;
            current_pose->y=current_pose->y-y_error;
        }
        if(y_less_map==true){
            current_pose->y=global_pose_y;
            current_pose->y=current_pose->y-y_error;
        }
        thephi=180*atan2(current_pose->y,current_pose->x)/3.1415926;
        if(thephi<=0){
            thephi=thephi+180;
        }else{
            thephi=thephi-180;
        }
        thephi=thephi-Yaw;
        if(my_abs(thephi)>180){
            if(thephi>180){
                thephi=thephi-360;
            }else{
                thephi=thephi+360;
            }
        }
        if(thephi>0){
            return_origin_positive_start=1;
        }else{
            return_origin_positive_start=-1;
        }
        return_origin_step_status=DIR_Y_LESS_NEGATIVE_200;
        break;
    case DIR_Y_LESS_NEGATIVE_200:
        if(obstacleSignal!=none_obstacle||(&cliff_valueB)->cliffValue0 == 1){
            linear_velocity=0;
            angular_velocity=0;
            return_origin_step_status = TURN_CLOCK_DIR_Y_LESS_NEGATIVE_200;;
            break;
        }
        if(my_abs(origin_thephi)>177){
            if(my_abs(Yaw)>177){
                linear_velocity=0;
                angular_velocity=0;
                last_position_x=current_pose->x;
                last_position_y=current_pose->y;
                return_origin_step_status=TURN_CLOCK_TARGET_YAW_NEGATIVE_90_RETURN_ORIGIN;
                break;
            }
        }
        else{
            if(origin_thephi-3<=Yaw&&Yaw<=origin_thephi+3){
                linear_velocity=0;
                angular_velocity=0;
                last_position_x=current_pose->x;
                last_position_y=current_pose->y;
                return_origin_step_status=TURN_CLOCK_TARGET_YAW_NEGATIVE_90_RETURN_ORIGIN;
                break;
            }
        }
        linear_velocity=0;
        angular_velocity=return_origin_positive_start*turn_vel;
        break;
    case TURN_CLOCK_DIR_Y_LESS_NEGATIVE_200:
        if(turn_start_update == 0)
        {
            turn_start_x = current_pose->x;
            turn_start_y = current_pose->y;
            turn_start_update = 1;
        }
        linear_velocity = -long_stra_vel;
        angular_velocity = 0;
        if (my_abs(turn_start_x - current_pose->x) > collision_backward_distance || my_abs(turn_start_y - current_pose->y) > collision_backward_distance)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            turn_start_update = 0;
            return_origin_step_status =DIR_Y_LESS_NEGATIVE_200;
        }
        break;
        
    case TURN_CLOCK_TARGET_YAW_NEGATIVE_90_RETURN_ORIGIN:
        if (obstacleSignal!=none_obstacle||(&cliff_valueB)->cliffValue0 == 1)
        {
            step=0;
            bow_continue=false;
            return_origin_step_status = COLLISION_Y_NEGATIVE_90_RETURN_ORIGIN;
            break;
        }
        if(sqrt(pow(last_position_x, 2) + pow(last_position_y, 2))<sqrt(pow(current_pose->x, 2) + pow(current_pose->y, 2))-20){
            linear_velocity = 0;
            angular_velocity = 0;
            step=0;
            return_origin_step_status = GOSTR_RETURN_ORIGIN_STEP;
        }
        step++;
        if(step>50){
            step=0;
            last_position_x=current_pose->x;
            last_position_y=current_pose->y;
        }
        if(bow_continue==false){
            if(my_abs(current_pose->x)<500&&my_abs(current_pose->y)<500){
                bow_continue=true;
                linear_velocity = 0;
                angular_velocity = 0;
                step=0;
                return_origin_step_status = DIR_X_LESS_NEGATIVE_200;
            }
        }
        if(over_clean_finish == true){
            current_pose->x=global_pose_x;
            current_pose->y=global_pose_y;
            current_pose->x=current_pose->x-x_error;
            current_pose->y=current_pose->y-y_error;
        }
        if(y_less_map==true){
            current_pose->y=global_pose_y;
            current_pose->y=current_pose->y-y_error;
        }
        if (my_abs(current_pose->x)<return_origin_distance&&my_abs(current_pose->y)<return_origin_distance)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            step=0;
            return_origin_step_status = GOSTR_RETURN_ORIGIN_STEP;
            break;
        }
        linear_velocity = long_stra_vel;
        angular_velocity = 0;
        break;
        
    case COLLISION_Y_NEGATIVE_90_RETURN_ORIGIN:
        if(turn_start_update == 0)
        {
            turn_start_x = current_pose->x;
            turn_start_y = current_pose->y;
            turn_start_update = 1;
        }
        linear_velocity = -long_stra_vel;
        angular_velocity = 0;
        if (my_abs(turn_start_x - current_pose->x) > collision_backward_distance || my_abs(turn_start_y - current_pose->y) > collision_backward_distance)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            turn_start_update = 0;
            temporary_yaw=Yaw;
            return_origin_step_status =TURN_CLOCK_Y_NEGATIVE_90_RETURN_ORIGIN;
        }
        break;
    case TURN_CLOCK_Y_NEGATIVE_90_RETURN_ORIGIN:
        if (obstacleSignal!=none_obstacle||(&cliff_valueB)->cliffValue0 == 1)
        {
            return_origin_step_status = COLLISION_Y_NEGATIVE_90_RETURN_ORIGIN;
            break;
        }
        if (my_abs(Yaw - temporary_yaw) > 15)
        {
            thephi=origin_thephi-Yaw;
            if(my_abs(thephi)>180){
                if(thephi>180){
                    thephi=thephi-360;
                }else{
                    thephi=thephi+360;
                }
            }
            if(my_abs(thephi)>90){
                cnt_update=(current_pose->y%half_map_wide+half_map_wide)/100;
                if((DelimmaNumber==3&&leakingsweep==1)||(DelimmaNumber==4&&leakingsweep==-1)){
                    for(i=0;i<MAPWIDECELLS;i++){
                        if(gridmap.map[i][cnt_update]!=125){
                            break;
                        }
                    }
                    if((current_pose->x+half_map_wide)/100-i<=5){
                        if(over_clean_finish==false&&leakingsweep==-1){
                            return_origin_step_status = TURN_CLOCK_COLLISION_Y_POSITIVE_90_RETURN_ORIGIN;
                            break;
                        }
                        if(over_clean_finish==true&&leakingsweep==-1){
                            if(edge_dilemma==true){
                                complete_flag=2;
                                edge_dilemma=false;
                            }else{
                                return_origin_step_status = BYPASS_OBSTACLES_Y_NEGATIVE_90_RETURN_ORIGIN;
                                leakingsweep=1;
                                edge_dilemma=true;
                            }
                            break;
                        }
                        leakingsweep=-1;
                    }
                }
                if((DelimmaNumber==4&&leakingsweep==1)||(DelimmaNumber==3&&leakingsweep==-1)){
                    for(i=MAPWIDECELLS-1;i>0;i--){
                        if(gridmap.map[i][cnt_update]!=125){
                            break;
                        }
                    }
                    if((current_pose->x+half_map_wide)/100+5
                            >=i){
                        if(over_clean_finish==false&&leakingsweep==-1){
                            return_origin_step_status = TURN_CLOCK_COLLISION_Y_POSITIVE_90_RETURN_ORIGIN;
                            break;
                        }
                        if(over_clean_finish==true&&leakingsweep==-1){
                            if(edge_dilemma==true){
                                complete_flag=2;
                                edge_dilemma=false;
                            }else{
                                return_origin_step_status = BYPASS_OBSTACLES_Y_NEGATIVE_90_RETURN_ORIGIN;
                                leakingsweep=1;
                                edge_dilemma=true;
                            }
                            break;
                        }
                        leakingsweep=-1;
                    }
                }
            }
            return_origin_step_status = BYPASS_OBSTACLES_Y_NEGATIVE_90_RETURN_ORIGIN;
            break;
        }
        linear_velocity = 0;
        angular_velocity =leakingsweep*turn_vel;
        break;
        //    case TURN_CLOCK_COLLISION_Y_NEGATIVE_90_RETURN_ORIGIN:
        //        linear_velocity=0;
        //        angular_velocity=0;
        //        leakingsweep=0;
        //        DelimmaNumber=0;
        //        cnt_update=0;
        //        return_origin_step_status=0;
        //        step=0;
        //        bow_continue=false;
        //        edge_dilemma=false;
        //        complete_flag = 1;
        //        break;
    case BYPASS_OBSTACLES_Y_NEGATIVE_90_RETURN_ORIGIN:
        last_position_yy = current_pose->y;
        last_position_xx= current_pose->x;
        return_origin_step_status =LOOP_BYPASS_OBSTACLES_Y_NEGATIVE_90_RETURN_ORIGIN;
        break;
    case LOOP_BYPASS_OBSTACLES_Y_NEGATIVE_90_RETURN_ORIGIN:
        if (obstacleSignal!=none_obstacle||(&cliff_valueB)->cliffValue0 == 1)
        {
            temporary_yaw=Yaw;
            return_origin_step_status = COLLISION_Y_NEGATIVE_90_RETURN_ORIGIN;
            break;
        }
        if (my_abs(last_position_yy - current_pose->y) > lateral_move_distance/2||my_abs(last_position_xx - current_pose->x) > lateral_move_distance/2)
        {
            temporary_yaw=Yaw;
            return_origin_step_status = COLLISION_LOOP_BYPASS_OBSTACLES_Y_NEGATIVE_90_RETURN_ORIGIN;
            break;
        }
        if(over_clean_finish == true){
            current_pose->x=global_pose_x;
            current_pose->y=global_pose_y;
            current_pose->x=current_pose->x-x_error;
            current_pose->y=current_pose->y-y_error;
        }
        if(y_less_map==true){
            current_pose->y=global_pose_y;
            current_pose->y=current_pose->y-y_error;
        }
        if (my_abs(current_pose->x)<return_origin_distance&&my_abs(current_pose->y)<return_origin_distance)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            return_origin_step_status = GOSTR_RETURN_ORIGIN_STEP;
            break;
        }
        linear_velocity = long_stra_vel;
        angular_velocity = 0;
        break;
        
    case COLLISION_LOOP_BYPASS_OBSTACLES_Y_NEGATIVE_90_RETURN_ORIGIN:
        if (obstacleSignal!=none_obstacle||(&cliff_valueB)->cliffValue0 == 1)
        {
            return_origin_step_status = COLLISION_Y_NEGATIVE_90_RETURN_ORIGIN;
            break;
        }
        if (my_abs(Yaw - temporary_yaw) >30)
        {
            return_origin_step_status = Y_MORE_LOOP_BYPASS_OBSTACLES_Y_NEGATIVE_90_RETURN_ORIGIN;
            break;
        }
        if(origin_thephi-5<=Yaw&&Yaw<=origin_thephi+5){
            return_origin_step_status = DIR_X_LESS_NEGATIVE_200;
            break;
        }
        linear_velocity = 0;
        angular_velocity = -leakingsweep*turn_vel;
        break;
    case Y_MORE_LOOP_BYPASS_OBSTACLES_Y_NEGATIVE_90_RETURN_ORIGIN:
        if (obstacleSignal!=none_obstacle||(&cliff_valueB)->cliffValue0 == 1)
        {
            return_origin_step_status = COLLISION_Y_NEGATIVE_90_RETURN_ORIGIN;
            break;
        }
        if (my_abs(last_position_yy - current_pose->y) > lateral_move_distance||my_abs(last_position_xx - current_pose->x) > lateral_move_distance)
        {
            thephi=origin_thephi-Yaw;
            if(my_abs(thephi)>180){
                if(thephi>180){
                    thephi=thephi-360;
                }else{
                    thephi=thephi+360;
                }
            }
            if(my_abs(thephi)<20){
                return_origin_step_status = DIR_X_LESS_NEGATIVE_200;
            }else{
                return_origin_step_status =BYPASS_OBSTACLES_Y_NEGATIVE_90_RETURN_ORIGIN;
            }
        }
        linear_velocity = long_stra_vel;
        angular_velocity = 0;
        break;
    }
    if(my_abs(origin_last_position_x-current_pose->x)>1000||my_abs(origin_last_position_y-current_pose->y)>1000){
        current_pose->x=current_pose->x-x_error;
        current_pose->y=current_pose->y-y_error;
        if(b_reverse_moremap==true){
            if(reverse_moremap==1){
                current_pose->x=current_pose->x+reverse_x_more_map;
            }
            else{
                current_pose->x=current_pose->x-reverse_x_more_map;
            }
        }
        if(over_clean_finish==false){
            if(x_more_map==true||y_more_map==true){
                if(x_more_map==true){
                    if(b_reverse_moremap==false){
                        current_pose->x=current_pose->x-x_more_positive_start*half_map_wide;
                    }
                }
                if(y_more_map==true){
                    current_pose->y=current_pose->y-y_more_positive_start*half_map_wide;
                }
            }
        }
        linear_velocity=0;
        angular_velocity=0;
        leakingsweep=0;
        DelimmaNumber=0;
        cnt_update=0;
        return_origin_step_status=0;
        step=0;
        bow_continue=false;
        edge_dilemma=false;
        OVERALL_CLEANING_STRATEGY =A_STAR_RETURN_ORIGIN_WORKING_OVERALL_CLEANING_STRATEGY;
        complete_flag=0;
        return complete_flag;
    }
    return complete_flag;
}


void RightMapExtreme(void){
    bool end_x=false;
    unsigned char i,j;
    for (i=0;i<GRIDWIDTH;i++) {
        for(j=GRIDHEIGHT/2-5;j>0;j--){
            if(gridmap.map[i][j]!=125){
                rightmapmin=j;
                end_x=true;
                break;
            }
        }
        if(end_x==true){
            end_x=false;
            break;
        }
    }
    for (i=GRIDWIDTH-1;i>0;i--){
        for(j=GRIDHEIGHT/2-3;j>0;j--){
            if(gridmap.map[i][j]!=125){
                rightmapmax=j;
                end_x=true;
                break;
            }
        }
        if(end_x==true){
            end_x=false;
            break;
        }
    }
}








void  initOpen(AStar_Open *q){
    q->length = 0;
}





void  push(AStar_Open *q, AStar_Close cls[AStar_Height][AStar_Width], unsigned char x, unsigned char y, short g){
    //log_debug("A*  push(),\n");
    
    AStar_Close *t;
    short i, mintag;
    cls[x][y].AStar_G = g;
    cls[x][y].AStar_F = cls[x][y].AStar_G + cls[x][y].AStar_H;
    q->Array[q->length++] = &(cls[x][y]);
    mintag = q->length - 1;
    for (i = 0; i < q->length - 1; i++)
    {
        if (q->Array[i]->AStar_F < q->Array[mintag]->AStar_F)
        {
            mintag = i;
        }
    }
    t = q->Array[q->length - 1];
    q->Array[q->length - 1] = q->Array[mintag];
    q->Array[mintag] = t;
}





AStar_Close * shift(AStar_Open *q){
    return q->Array[--q->length];
}







void  initClose(AStar_Close cls[AStar_Height][AStar_Width], unsigned char sx, unsigned char sy, unsigned char dx, unsigned char dy){
    unsigned char i, j;
    //log_debug("A*  initClose(),\n");
    for (i = 0; i < AStar_Height; i++)
    {
        for (j = 0; j < AStar_Width; j++)
        {
            cls[i][j].cur = &AStar_graph[i][j];
            cls[i][j].vis = !AStar_graph[i][j].reachable;
            cls[i][j].from = NULL;
            cls[i][j].AStar_G = cls[i][j].AStar_F = 0;
            cls[i][j].AStar_H = abs(dx - i) + abs(dy - j);
        }
    }
    cls[sx][sy].AStar_F = cls[sx][sy].AStar_H;
    cls[dx][dy].AStar_G = AStar_Infinity;
}





void  initGraph(bool maze[AStar_Height][AStar_Width], unsigned char sx, unsigned char sy, unsigned char dx, unsigned char dy){
    unsigned char i, j;
    AStar_srcX = sx;
    AStar_srcY = sy;
    AStar_dstX = dx;
    AStar_dstY = dy;
    //log_debug("A*  initGraph(),\n");
    for (i = 0; i < AStar_Height; i++)
    {
        for (j = 0; j < AStar_Width; j++)
        {
            AStar_graph[i][j].x = i;
            AStar_graph[i][j].y = j;
            AStar_graph[i][j].reachable = (maze[i][j] == AStar_Reachable);
            AStar_graph[i][j].sur = 0;
            if (!AStar_graph[i][j].reachable)
            {
                continue;
            }
            if (j > 0)
            {
                if (AStar_graph[i][j - 1].reachable)
                {
                    AStar_graph[i][j].sur |= AStar_West;
                    AStar_graph[i][j - 1].sur |= AStar_East;
                }
                if (i > 0)
                {
                    if (AStar_graph[i - 1][j - 1].reachable && AStar_graph[i - 1][j].reachable && AStar_graph[i][j - 1].reachable)
                    {
                        AStar_graph[i][j].sur |= AStar_North_West;
                        AStar_graph[i - 1][j - 1].sur |= AStar_South_East;
                    }
                }
            }
            if (i > 0)
            {
                if (AStar_graph[i - 1][j].reachable)
                {
                    AStar_graph[i][j].sur |= AStar_North;
                    AStar_graph[i - 1][j].sur |= AStar_South;
                }
                if (j < AStar_Width - 1)
                {
                    if (AStar_graph[i - 1][j + 1].reachable && AStar_graph[i - 1][j].reachable && maze[i][j + 1] == AStar_Reachable)
                    {
                        AStar_graph[i][j].sur |= AStar_North_East;
                        AStar_graph[i - 1][j + 1].sur |= AStar_South_West;
                    }
                }
            }
        }
    }
}





bool  astar(){
    unsigned char i, curX, curY, surX, surY;
    float surG;
    AStar_Open q;
    AStar_Close *p;
    initOpen(&q);
    initClose(astar_close, AStar_srcX, AStar_srcY, AStar_dstX, AStar_dstY);
    astar_close[AStar_srcX][AStar_srcY].vis = 1;
    push(&q, astar_close, AStar_srcX, AStar_srcY, 0);
    
    //log_debug("A*  astar(),\n");
    while (q.length)
    {
        p = shift(&q);
        curX = p->cur->x;
        curY = p->cur->y;
        if (!p->AStar_H)
        {
            return AStar_Sequential;
        }
        for (i = 0; i < 8; i++)
        {
            if (!(p->cur->sur & (1 << i)))
            {
                continue;
            }
            surX = curX + astar_dir[i].x;
            surY = curY + astar_dir[i].y;
            if (!astar_close[surX][surY].vis)
            {
                astar_close[surX][surY].vis = 1;
                astar_close[surX][surY].from = p;
                surG = p->AStar_G + sqrt((curX - surX) * (curX - surX) + (curY - surY) * (curY - surY));
                push(&q, astar_close, surX, surY, surG);
            }
        }
    }
    return AStar_NoSolution;
}



AStar_Close * getShortest(){
    bool result = astar();
    AStar_Close *p, *t, *q = NULL;
    switch(result)
    {
    case AStar_Sequential:
        p = &(astar_close[AStar_dstX][AStar_dstY]);
        while (p)
        {
            t = p->from;
            p->from = q;
            q = p;
            p = t;
        }
        astar_close[AStar_srcX][AStar_srcY].from = q->from;
        return &(astar_close[AStar_srcX][AStar_srcY]);
    case AStar_NoSolution:
        return NULL;
    }
    return NULL;
}



unsigned char  printShortest(){
    AStar_Close *p;
    step = 0;
    p = getShortest();
    AStar_start = p;
    
    
    //log_debug("A*  printShortest(),\n");
    if (!p)
    {
        return 0;
    }
    else
    {
        short previous_x = -3, previous_y = -3;
        printclose = false;
        while (p->from || printclose == false)
        {
            if (!p->from)
            {
                printclose = true;
            }
            if (p->cur->x - previous_x == 1 && p->cur->y - previous_y == 1)
            {
                plansteps[motionSteps] = 'g';
                motionSteps++;
                plansteps[motionSteps] = 'g';
                motionSteps++;
            }
            else if (p->cur->x - previous_x == 0 && p->cur->y - previous_y == 1)
            {
                plansteps[motionSteps] = 'h';
                motionSteps++;
                plansteps[motionSteps] = 'h';
                motionSteps++;
            }
            else if (p->cur->x - previous_x == -1 && p->cur->y - previous_y == 1)
            {
                plansteps[motionSteps] = 'a';
                motionSteps++;
                plansteps[motionSteps] = 'a';
                motionSteps++;
            }
            else if (p->cur->x - previous_x == -1 && p->cur->y - previous_y == 0)
            {
                plansteps[motionSteps] = 'b';
                motionSteps++;
                plansteps[motionSteps] = 'b';
                motionSteps++;
            }
            else if (p->cur->x - previous_x == -1 && p->cur->y - previous_y == -1)
            {
                plansteps[motionSteps] = 'c';
                motionSteps++;
                plansteps[motionSteps] = 'c';
                motionSteps++;
            }
            else if (p->cur->x - previous_x == 0 && p->cur->y - previous_y == -1)
            {
                plansteps[motionSteps] = 'd';
                motionSteps++;
            }
            else if (p->cur->x - previous_x == 1 && p->cur->y - previous_y == -1)
            {
                plansteps[motionSteps] = 'e';
                motionSteps++;
                plansteps[motionSteps] = 'e';
                motionSteps++;
            }
            else if (p->cur->x - previous_x == 1 && p->cur->y - previous_y == 0)
            {
                plansteps[motionSteps] = 'f';
                motionSteps++;
                plansteps[motionSteps] = 'f';
                motionSteps++;
            }
            else
            {
            }
            if (motionSteps > 48){
                return step;
            }
            maze[p->cur->x][p->cur->y] = 1;
            previous_x = p->cur->x;
            previous_y = p->cur->y;
            if (printclose == false)
            {
                p = p->from;
                step++;
            }
        }
    }
    return step;
}


unsigned char  AStarReturnOrigin(POSE *current_pose, unsigned char obstacleSignal){
    signed char complete_flag = 0;
    bool canmove = true;
    signed char goal_robot_x = 0;
    signed char goal_robot_y = 0;
    signed char firsttrap = 0;
    signed char secondtrap = 0;
    signed char thirdtrap = 0;
    signed char forthtrap = 0;
    signed char astar_underboundary;
    signed char astar_onboundary;
    signed char leftboundary = 0;
    signed char rightboundary = 0;
    bool end_x = false;
    signed char a_map_x = 0;
    signed char a_map_y = 0;
    signed char goal_map_x = (goal_robot_x + MAPHEIGHT / 2) / (ZoomMultiple * GRIDWIDTH);
    signed char goal_map_y = (goal_robot_y + MAPHEIGHT / 2) / (ZoomMultiple * GRIDWIDTH);
    signed char i, j, k, ij, jk, ik;
    if((current_pose->x + MAPHEIGHT / 2) / (ZoomMultiple * GRIDWIDTH)<0||(current_pose->y + MAPHEIGHT / 2) / (ZoomMultiple * GRIDWIDTH)<0){
        return 0;
    }
    a_map_x = (current_pose->x + MAPHEIGHT / 2) / (ZoomMultiple * GRIDWIDTH);
    a_map_y = (current_pose->y + MAPHEIGHT / 2) / (ZoomMultiple * GRIDWIDTH);
    complete_flag = 0;
    if(y_less_map==true){
        goal_map_x = 12;
        goal_map_y = 24;
    }
    if(x_more_map==true){
        if(over_clean_finish==true){
            if(x_more_positive_start==1){
                goal_map_x = 0;
            }
            else{
                goal_map_x = 24;
            }
        }
    }
    if(y_more_map==true){
        if(over_clean_finish==true){
            goal_map_y = 0;
        }
    }
    for (i = 0; i < 4; i++)
    {
        jk = i > 0 ? 1 : 0;
        for (j = 0; j < GRIDWIDTH; j++)
        {
            for (k = 0; k < GRIDWIDTH; k++)
            {
                if (gridmap.map[j][k] == 250 || gridmap.map[j][k] == 0)
                {
                    firsttrap = j;
                    end_x = true;
                    break;
                }
            }
            if (end_x == true)
            {
                end_x = false;
                break;
            }
        }
        astar_underboundary = (firsttrap - i) / 4 > 0 ? (firsttrap - i) / 4 - 1 : 0;
        for (j = GRIDWIDTH - 1; j >= 0; j--)
        {
            for (k = 0; k < GRIDWIDTH; k++)
            {
                if (gridmap.map[j][k] == 250 || gridmap.map[j][k] == 0)
                {
                    secondtrap = j;
                    end_x = true;
                    break;
                }
            }
            if (end_x == true)
            {
                end_x = false;
                break;
            }
        }
        astar_onboundary = (secondtrap - i) / 4 < MAPHEIGHT / (ZoomMultiple * GRIDHEIGHT) - 1 ? (secondtrap - i) / 4 + 1 : MAPHEIGHT / (ZoomMultiple * GRIDHEIGHT) - 1;
        for (j = 0; j < GRIDWIDTH; j++)
        {
            for (k = 0; k < GRIDWIDTH; k++)
            {
                if (gridmap.map[k][j] == 250 || gridmap.map[k][j] == 0)
                {
                    thirdtrap = j;
                    end_x = true;
                    break;
                }
            }
            if (end_x == true)
            {
                end_x = false;
                break;
            }
        }
        leftboundary = thirdtrap / 4 > 0 ? (firsttrap - i) / 4 - 1 : 0;
        
        for (j = GRIDWIDTH - 1; j >= 0; j--)
        {
            for (k = 0; k < GRIDWIDTH; k++)
            {
                if (gridmap.map[k][j] == 250 || gridmap.map[k][j] == 0)
                {
                    forthtrap = j;
                    end_x = true;
                    break;
                }
            }
            if (end_x == true)
            {
                end_x = false;
                break;
            }
        }
        rightboundary = forthtrap / 4 > MAPHEIGHT / (ZoomMultiple * GRIDHEIGHT) - 2 ? MAPHEIGHT / (ZoomMultiple * GRIDHEIGHT) - 1 : forthtrap / 4 + 1;
        for (j = 0; j < MAPHEIGHT / (ZoomMultiple * GRIDHEIGHT) - jk; j++)
        {
            if (j == astar_onboundary || j == astar_underboundary)
            {
                for (k = 0; k < MAPWIDTH / (ZoomMultiple * GRIDWIDTH); k++)
                {
                    if ((j == a_map_x && k == a_map_y) || (j == goal_map_x && k == goal_map_y))
                    {
                        maze[j][k] = 0;
                    }
                    else
                    {
                        maze[j][k] = 1;
                    }
                }
            }
            else
            {
                for (k = 0; k < MAPWIDTH / (ZoomMultiple * GRIDWIDTH); k++)
                {
                    if (k == leftboundary || k == rightboundary)
                    {
                        maze[j][k] = 1;
                    }
                    else
                    {
                        for (ij = 0; ij < ZoomMultiple; ij++)
                        {
                            for (ik = 0; ik < ZoomMultiple; ik++)
                            {
                                if (gridmap.map[(ZoomMultiple * ij + ik + i)%100][(ZoomMultiple * k + ik)%100] == 0)
                                {
                                    canmove = false;
                                    break;
                                }
                                else
                                {
                                }
                            }
                            if (canmove == false)
                            {
                                break;
                            }
                        }
                        if (canmove == true)
                        {
                            maze[j][k] = 0;
                        }
                        else
                        {
                            if ((i == a_map_x && k == a_map_y) || (i == MAPHEIGHT / (ZoomMultiple * GRIDHEIGHT) && k == MAPHEIGHT / (ZoomMultiple * GRIDHEIGHT)))
                            {
                                maze[j][k] = 0;
                            }
                            else
                            {
                                maze[j][k] = 1;
                            }
                            canmove = true;
                        }
                    }
                }
            }
        }
    }
    initGraph(maze, a_map_x, a_map_y, goal_map_x, goal_map_y);
    printShortest();
    return complete_flag;
}



unsigned char  AStarNotMotionReturnOrigin(POSE *current_pose, unsigned char obstacleSignal){
    int Yaw;
    unsigned char complete_flag = 0;
    Yaw = current_pose->orientation;
    Yaw = Yaw/100;
    //log_debug("a_star_not_motion_status =======>>>,%x,\n",a_star_not_motion_status);
    switch (a_star_not_motion_status)
    {
    case 0:
        temporary_wheel_pulse_r = wheel_pulse_r;
        a_star_not_motion_status = START_PLAN_ASTAR_MOTION_GOSTR_RETURN;
        break;
    case START_PLAN_ASTAR_MOTION_GOSTR_RETURN:
        if (obstacleSignal != none_obstacle||(&cliff_valueB)->cliffValue0 == 1)
        {
            a_star_not_motion_status = COLLISION_START_PLAN_ASTAR_MOTION_GOSTR_RETURN;
            break;
        }
        if (my_abs(wheel_pulse_r - temporary_wheel_pulse_r) > 958)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            a_star_not_motion_status = 0;
            complete_flag = 1;
            break;
        }
        linear_velocity = long_stra_vel;
        angular_velocity = 0;
        break;
    case COLLISION_START_PLAN_ASTAR_MOTION_GOSTR_RETURN:
        if (turn_start_update == 0)
        {
            turn_start_x = current_pose->x;
            turn_start_y = current_pose->y;
            turn_start_update = 1;
        }
        linear_velocity = -long_stra_vel;
        angular_velocity = 0;
        if (my_abs(turn_start_x - current_pose->x) > side_backward_distance || my_abs(turn_start_y - current_pose->y) > side_backward_distance)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            turn_start_update = 0;
            temporary_yaw = Yaw;
            a_star_not_motion_status = TURN_CLOCK_START_PLAN_ASTAR_MOTION_GOSTR_RETURN;
        }
        break;
    case TURN_CLOCK_START_PLAN_ASTAR_MOTION_GOSTR_RETURN:
        if (my_abs(temporary_yaw - Yaw) > 30)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            temporary_wheel_pulse_r = wheel_pulse_r;
            a_star_not_motion_status = START_PLAN_ASTAR_MOTION_GOSTR_RETURN;
            break;
        }
        if (obstacleSignal != none_obstacle)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            a_star_not_motion_status = COLLISION_TURN_CLOCK_START_PLAN_ASTAR_MOTION_GOSTR_RETURN;
            break;
        }
        linear_velocity = 0;
        angular_velocity = turn_vel;
        break;
    case COLLISION_TURN_CLOCK_START_PLAN_ASTAR_MOTION_GOSTR_RETURN:
        if (turn_start_update == 0)
        {
            turn_start_x = current_pose->x;
            turn_start_y = current_pose->y;
            turn_start_update = 1;
        }
        linear_velocity = -long_stra_vel;
        angular_velocity = 0;
        if (my_abs(turn_start_x - current_pose->x) > side_backward_distance || my_abs(turn_start_y - current_pose->y) > side_backward_distance)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            turn_start_update = 0;
            a_star_not_motion_status = TURN_CLOCK_START_PLAN_ASTAR_MOTION_GOSTR_RETURN;
        }
        break;
    }
    return complete_flag;
}



unsigned char  AStarMotionReturnOrigin(POSE *current_pose, unsigned char obstacleSignal){
    int Yaw;
    unsigned char complete_flag = 0;
    Yaw = current_pose->orientation;
    Yaw = Yaw /100;
    //log_debug("a_star_motion_return_origin_status =======>>>,%x,\n",a_star_motion_return_origin_status);
    switch (a_star_motion_return_origin_status)
    {
    case 0:
        a_star_motion_return_origin_status = ASTAR_MOTION_GOSTR_RETURN;
        break;
    case ASTAR_MOTION_GOSTR_RETURN:
        if(y_less_map==true){
            current_pose->y=global_pose_y;
            current_pose->y=current_pose->y-y_error;
        }
        if(over_clean_finish == true){
            current_pose->x=global_pose_x;
            current_pose->y=global_pose_y;
            current_pose->x=current_pose->x-x_error;
            current_pose->y=current_pose->y-y_error;
        }
        
        if (my_abs(current_pose->x) > 2*return_origin_distance || my_abs(current_pose->y) > 2*return_origin_distance)
        {
            a_star_motion_return_origin_status = PLAN_ASTAR_MOTION_GOSTR_RETURN;
            linear_velocity = long_stra_vel;
            angular_velocity = 0;
        }
        else
        {
            a_star_motion_return_origin_status = A_STAR_COMPLETED;
        }
        
        break;
        
    case PLAN_ASTAR_MOTION_GOSTR_RETURN:
        if (motionSteps == 0)
        {
            AStarMotionNumber++;
            if (AStarMotionNumber > 4)
            {
                complete_flag = 4;
            }
            else
            {
                complete_flag = 3;
            }
            a_star_motion_return_origin_status = 0;
            break;
        }
        if (motionSteps > 0)
        {
            linear_velocity = long_stra_vel;
            angular_velocity = 0;
            AStarMotionNumber = 0;
            a_star_motion_return_origin_status = DIRECT_START_PLAN_ASTAR_MOTION_GOSTR_RETURN;
            break;
        }
        break;
    case DIRECT_START_PLAN_ASTAR_MOTION_GOSTR_RETURN:
        if(y_less_map==true){
            current_pose->y=global_pose_y;
            current_pose->y=current_pose->y-y_error;
        }
        if(over_clean_finish == true){
            current_pose->x=global_pose_x;
            current_pose->y=global_pose_y;
            current_pose->x=current_pose->x-x_error;
            current_pose->y=current_pose->y-y_error;
        }
        if ((my_abs(current_pose->x) < 3*return_origin_distance && my_abs(current_pose->y) < 3*return_origin_distance) || startMotionStep >= motionSteps)
        {
            a_star_motion_return_origin_status = A_STAR_COMPLETED;
            break;
        }
        if (motionSteps > 45)
        {
            a_star_motion_return_origin_status = 0;
            complete_flag = 2;
            startMotionStep = 0;
            motionSteps = 0;
            AStarMotionNumber = 0;
            break;
        }
        if (plansteps[startMotionStep] == 'a')
        {
            a_star_motion_return_origin_status = A_DIRECT_START_PLAN_ASTAR_MOTION_GOSTR_RETURN;
        }
        else if (plansteps[startMotionStep] == 'b')
        {
            a_star_motion_return_origin_status = B_DIRECT_START_PLAN_ASTAR_MOTION_GOSTR_RETURN;
        }
        else if (plansteps[startMotionStep] == 'c')
        {
            a_star_motion_return_origin_status = C_DIRECT_START_PLAN_ASTAR_MOTION_GOSTR_RETURN;
        }
        else if (plansteps[startMotionStep] == 'd')
        {
            a_star_motion_return_origin_status = D_DIRECT_START_PLAN_ASTAR_MOTION_GOSTR_RETURN;
        }
        else if (plansteps[startMotionStep] == 'e')
        {
            a_star_motion_return_origin_status = E_DIRECT_START_PLAN_ASTAR_MOTION_GOSTR_RETURN;
        }
        else if (plansteps[startMotionStep] == 'f')
        {
            a_star_motion_return_origin_status = F_DIRECT_START_PLAN_ASTAR_MOTION_GOSTR_RETURN;
        }
        else if (plansteps[startMotionStep] == 'g')
        {
            a_star_motion_return_origin_status = G_DIRECT_START_PLAN_ASTAR_MOTION_GOSTR_RETURN;
        }
        else if (plansteps[startMotionStep] == 'h')
        {
            a_star_motion_return_origin_status = H_DIRECT_START_PLAN_ASTAR_MOTION_GOSTR_RETURN;
        }
        else
        {
            a_star_motion_return_origin_status = A_STAR_COMPLETED;
        }
        linear_velocity = long_stra_vel;
        angular_velocity = 0;
        startMotionStep++;
        break;
    case A_DIRECT_START_PLAN_ASTAR_MOTION_GOSTR_RETURN:
        if (my_abs(Yaw) <= 45 || (Yaw > 45 && Yaw < 130))
        {
            a_star_motion_return_origin_status = LESS_45_A_DIRECT_START_PLAN_ASTAR_MOTION_GOSTR_RETURN;
        }
        else if (my_abs(Yaw) > 140 || (Yaw >= -140 && Yaw < -45))
        {
            a_star_motion_return_origin_status = MORE_45_A_DIRECT_START_PLAN_ASTAR_MOTION_GOSTR_RETURN;
        }
        else
        {
            temporary_wheel_pulse_l = wheel_pulse_l;
            a_star_motion_return_origin_status = GO_A_DIRECT_START_PLAN_ASTAR_MOTION_GOSTR_RETURN;
        }
        break;
    case LESS_45_A_DIRECT_START_PLAN_ASTAR_MOTION_GOSTR_RETURN:
        if (Yaw < 138 && Yaw > 132)
        {
            //原先是0，现在是long_stra_vel
            linear_velocity = long_stra_vel;
            angular_velocity = 0;
            temporary_wheel_pulse_l = wheel_pulse_l;
            a_star_motion_return_origin_status = GO_A_DIRECT_START_PLAN_ASTAR_MOTION_GOSTR_RETURN;
            break;
        }
        if (obstacleSignal != none_obstacle||(&cliff_valueB)->cliffValue0 == 1)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            a_star_motion_return_origin_status = COLLISION_LESS_45_A_DIRECT_START_PLAN_ASTAR_MOTION_GOSTR_RETURN;
            break;
        }
        linear_velocity = 0;
        angular_velocity = turn_vel;
        break;
    case COLLISION_LESS_45_A_DIRECT_START_PLAN_ASTAR_MOTION_GOSTR_RETURN:
        if (turn_start_update == 0)
        {
            turn_start_x = current_pose->x;
            turn_start_y = current_pose->y;
            turn_start_update = 1;
        }
        linear_velocity = -long_stra_vel;
        angular_velocity = 0;
        if (my_abs(turn_start_x - current_pose->x) > collision_backward_distance || my_abs(turn_start_y - current_pose->y) > collision_backward_distance)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            turn_start_update = 0;
            a_star_motion_return_origin_status = LESS_45_A_DIRECT_START_PLAN_ASTAR_MOTION_GOSTR_RETURN;
        }
        break;
    case MORE_45_A_DIRECT_START_PLAN_ASTAR_MOTION_GOSTR_RETURN:
        if (Yaw < 138 && Yaw > 132)
        {
            linear_velocity = long_stra_vel;
            angular_velocity = 0;
            temporary_wheel_pulse_l = wheel_pulse_l;
            a_star_motion_return_origin_status = GO_A_DIRECT_START_PLAN_ASTAR_MOTION_GOSTR_RETURN;
            break;
        }
        if (obstacleSignal != none_obstacle||(&cliff_valueB)->cliffValue0 == 1)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            a_star_motion_return_origin_status = COLLISION_MORE_45_A_DIRECT_START_PLAN_ASTAR_MOTION_GOSTR_RETURN;
            break;
        }
        linear_velocity = 0;
        angular_velocity = -turn_vel;
        break;
    case COLLISION_MORE_45_A_DIRECT_START_PLAN_ASTAR_MOTION_GOSTR_RETURN:
        if (turn_start_update == 0)
        {
            turn_start_x = current_pose->x;
            turn_start_y = current_pose->y;
            turn_start_update = 1;
        }
        linear_velocity = -long_stra_vel;
        angular_velocity = 0;
        if (my_abs(turn_start_x - current_pose->x) > collision_backward_distance || my_abs(turn_start_y - current_pose->y) > collision_backward_distance)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            turn_start_update = 0;
            a_star_motion_return_origin_status = MORE_45_A_DIRECT_START_PLAN_ASTAR_MOTION_GOSTR_RETURN;
        }
        break;
    case GO_A_DIRECT_START_PLAN_ASTAR_MOTION_GOSTR_RETURN:
        if (obstacleSignal != none_obstacle||(&cliff_valueB)->cliffValue0 == 1)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            a_star_motion_return_origin_status = 0;
            complete_flag = 2;
            startMotionStep = 0;
            motionSteps = 0;
            AStarMotionNumber = 0;
            break;
        }
        if (wheel_pulse_l - temporary_wheel_pulse_l > 1055)
        {
            a_star_motion_return_origin_status = DIRECT_START_PLAN_ASTAR_MOTION_GOSTR_RETURN;
            break;
        }
        linear_velocity = long_stra_vel;
        angular_velocity = 0;
        break;
        
    case B_DIRECT_START_PLAN_ASTAR_MOTION_GOSTR_RETURN:
        if (my_abs(Yaw) < 175 && Yaw >= 0)
        {
            a_star_motion_return_origin_status = LESS_45_B_DIRECT_START_PLAN_ASTAR_MOTION_GOSTR_RETURN;
        }
        else if (my_abs(Yaw) < 175 && Yaw < 0)
        {
            a_star_motion_return_origin_status = MORE_45_B_DIRECT_START_PLAN_ASTAR_MOTION_GOSTR_RETURN;
        }
        else
        {
            temporary_wheel_pulse_l = wheel_pulse_l;
            a_star_motion_return_origin_status = GO_B_DIRECT_START_PLAN_ASTAR_MOTION_GOSTR_RETURN;
        }
        break;
    case LESS_45_B_DIRECT_START_PLAN_ASTAR_MOTION_GOSTR_RETURN:
        if (my_abs(Yaw) > 177)
        {
            linear_velocity = long_stra_vel;
            angular_velocity = 0;
            temporary_wheel_pulse_l = wheel_pulse_l;
            a_star_motion_return_origin_status = GO_B_DIRECT_START_PLAN_ASTAR_MOTION_GOSTR_RETURN;
            break;
        }
        if (obstacleSignal != none_obstacle||(&cliff_valueB)->cliffValue0 == 1)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            a_star_motion_return_origin_status = COLLISION_LESS_45_B_DIRECT_START_PLAN_ASTAR_MOTION_GOSTR_RETURN;
            break;
        }
        linear_velocity = 0;
        angular_velocity = turn_vel;
        break;
    case COLLISION_LESS_45_B_DIRECT_START_PLAN_ASTAR_MOTION_GOSTR_RETURN:
        if (turn_start_update == 0)
        {
            turn_start_x = current_pose->x;
            turn_start_y = current_pose->y;
            turn_start_update = 1;
        }
        linear_velocity = -long_stra_vel;
        angular_velocity = 0;
        if (my_abs(turn_start_x - current_pose->x) > collision_backward_distance || my_abs(turn_start_y - current_pose->y) > collision_backward_distance)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            turn_start_update = 0;
            a_star_motion_return_origin_status = LESS_45_B_DIRECT_START_PLAN_ASTAR_MOTION_GOSTR_RETURN;
        }
        break;
    case MORE_45_B_DIRECT_START_PLAN_ASTAR_MOTION_GOSTR_RETURN:
        if (my_abs(Yaw) > 177)
        {
            linear_velocity = long_stra_vel;
            angular_velocity = 0;
            temporary_wheel_pulse_l = wheel_pulse_l;
            a_star_motion_return_origin_status = GO_B_DIRECT_START_PLAN_ASTAR_MOTION_GOSTR_RETURN;
            break;
        }
        if (obstacleSignal != none_obstacle)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            a_star_motion_return_origin_status = COLLISION_MORE_45_B_DIRECT_START_PLAN_ASTAR_MOTION_GOSTR_RETURN;
            break;
        }
        linear_velocity = 0;
        angular_velocity = -turn_vel;
        break;
    case COLLISION_MORE_45_B_DIRECT_START_PLAN_ASTAR_MOTION_GOSTR_RETURN:
        if (turn_start_update == 0)
        {
            turn_start_x = current_pose->x;
            turn_start_y = current_pose->y;
            turn_start_update = 1;
        }
        linear_velocity = -long_stra_vel;
        angular_velocity = 0;
        if (my_abs(turn_start_x - current_pose->x) > collision_backward_distance || my_abs(turn_start_y - current_pose->y) > collision_backward_distance)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            turn_start_update = 0;
            a_star_motion_return_origin_status = MORE_45_B_DIRECT_START_PLAN_ASTAR_MOTION_GOSTR_RETURN;
        }
        break;
    case GO_B_DIRECT_START_PLAN_ASTAR_MOTION_GOSTR_RETURN:
        if (obstacleSignal != none_obstacle||(&cliff_valueB)->cliffValue0 == 1)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            a_star_motion_return_origin_status = 0;
            complete_flag = 2;
            startMotionStep = 0;
            motionSteps = 0;
            AStarMotionNumber = 0;
            break;
        }
        if (wheel_pulse_l - temporary_wheel_pulse_l > 758)
        {
            a_star_motion_return_origin_status = DIRECT_START_PLAN_ASTAR_MOTION_GOSTR_RETURN;
            break;
        }
        linear_velocity = long_stra_vel;
        angular_velocity = 0;
        break;
    case C_DIRECT_START_PLAN_ASTAR_MOTION_GOSTR_RETURN:
        if (my_abs(Yaw) > 140 || (Yaw > 45 && Yaw <= 140))
        {
            a_star_motion_return_origin_status = LESS_45_C_DIRECT_START_PLAN_ASTAR_MOTION_GOSTR_RETURN;
        }
        else if (my_abs(Yaw) <= 45 || (Yaw > -130 && Yaw < -45))
        {
            a_star_motion_return_origin_status = MORE_45_C_DIRECT_START_PLAN_ASTAR_MOTION_GOSTR_RETURN;
        }
        else
        {
            temporary_wheel_pulse_l = wheel_pulse_l;
            a_star_motion_return_origin_status = GO_A_DIRECT_START_PLAN_ASTAR_MOTION_GOSTR_RETURN;
        }
        break;
    case LESS_45_C_DIRECT_START_PLAN_ASTAR_MOTION_GOSTR_RETURN:
        if (Yaw > -138 && Yaw < -132)
        {
            linear_velocity = long_stra_vel;
            angular_velocity = 0;
            temporary_wheel_pulse_l = wheel_pulse_l;
            a_star_motion_return_origin_status = GO_A_DIRECT_START_PLAN_ASTAR_MOTION_GOSTR_RETURN;
            break;
        }
        if (obstacleSignal != none_obstacle||(&cliff_valueB)->cliffValue0 == 1)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            a_star_motion_return_origin_status = COLLISION_LESS_45_C_DIRECT_START_PLAN_ASTAR_MOTION_GOSTR_RETURN;
            break;
        }
        linear_velocity = 0;
        angular_velocity = turn_vel;
        break;
    case COLLISION_LESS_45_C_DIRECT_START_PLAN_ASTAR_MOTION_GOSTR_RETURN:
        if (turn_start_update == 0)
        {
            turn_start_x = current_pose->x;
            turn_start_y = current_pose->y;
            turn_start_update = 1;
        }
        linear_velocity = -long_stra_vel;
        angular_velocity = 0;
        if (my_abs(turn_start_x - current_pose->x) > collision_backward_distance || my_abs(turn_start_y - current_pose->y) > collision_backward_distance)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            turn_start_update = 0;
            a_star_motion_return_origin_status = LESS_45_C_DIRECT_START_PLAN_ASTAR_MOTION_GOSTR_RETURN;
        }
        break;
    case MORE_45_C_DIRECT_START_PLAN_ASTAR_MOTION_GOSTR_RETURN:
        if (Yaw > -138 && Yaw < -132)
        {
            linear_velocity = long_stra_vel;
            angular_velocity = 0;
            temporary_wheel_pulse_l = wheel_pulse_l;
            a_star_motion_return_origin_status = GO_A_DIRECT_START_PLAN_ASTAR_MOTION_GOSTR_RETURN;
            break;
        }
        if (obstacleSignal != none_obstacle||(&cliff_valueB)->cliffValue0 == 1)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            a_star_motion_return_origin_status = COLLISION_MORE_45_C_DIRECT_START_PLAN_ASTAR_MOTION_GOSTR_RETURN;
            break;
        }
        linear_velocity = 0;
        angular_velocity = -turn_vel;
        break;
    case COLLISION_MORE_45_C_DIRECT_START_PLAN_ASTAR_MOTION_GOSTR_RETURN:
        if (turn_start_update == 0)
        {
            turn_start_x = current_pose->x;
            turn_start_y = current_pose->y;
            turn_start_update = 1;
        }
        linear_velocity = -long_stra_vel;
        angular_velocity = 0;
        if (my_abs(turn_start_x - current_pose->x) > collision_backward_distance || my_abs(turn_start_y - current_pose->y) > collision_backward_distance)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            turn_start_update = 0;
            a_star_motion_return_origin_status = MORE_45_C_DIRECT_START_PLAN_ASTAR_MOTION_GOSTR_RETURN;
        }
        break;
        
    case D_DIRECT_START_PLAN_ASTAR_MOTION_GOSTR_RETURN:
        if (my_abs(Yaw) > 95)
        {
            a_star_motion_return_origin_status = LESS_45_D_DIRECT_START_PLAN_ASTAR_MOTION_GOSTR_RETURN;
        }
        else if (my_abs(Yaw) < 85 || (Yaw >= 85 && Yaw <= 95))
        {
            a_star_motion_return_origin_status = MORE_45_D_DIRECT_START_PLAN_ASTAR_MOTION_GOSTR_RETURN;
        }
        else
        {
            temporary_wheel_pulse_l = wheel_pulse_l;
            a_star_motion_return_origin_status = GO_B_DIRECT_START_PLAN_ASTAR_MOTION_GOSTR_RETURN;
        }
        break;
    case LESS_45_D_DIRECT_START_PLAN_ASTAR_MOTION_GOSTR_RETURN:
        if (Yaw > -138 && Yaw < -132)
        {
            linear_velocity = long_stra_vel;
            angular_velocity = 0;
            temporary_wheel_pulse_l = wheel_pulse_l;
            a_star_motion_return_origin_status = GO_B_DIRECT_START_PLAN_ASTAR_MOTION_GOSTR_RETURN;
            break;
        }
        if (obstacleSignal != none_obstacle||(&cliff_valueB)->cliffValue0 == 1)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            a_star_motion_return_origin_status = COLLISION_LESS_45_D_DIRECT_START_PLAN_ASTAR_MOTION_GOSTR_RETURN;
            break;
        }
        linear_velocity = 0;
        angular_velocity = turn_vel;
        break;
    case COLLISION_LESS_45_D_DIRECT_START_PLAN_ASTAR_MOTION_GOSTR_RETURN:
        if (turn_start_update == 0)
        {
            turn_start_x = current_pose->x;
            turn_start_y = current_pose->y;
            turn_start_update = 1;
        }
        linear_velocity = -long_stra_vel;
        angular_velocity = 0;
        if (my_abs(turn_start_x - current_pose->x) > collision_backward_distance || my_abs(turn_start_y - current_pose->y) > collision_backward_distance)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            turn_start_update = 0;
            a_star_motion_return_origin_status = LESS_45_D_DIRECT_START_PLAN_ASTAR_MOTION_GOSTR_RETURN;
        }
        break;
    case MORE_45_D_DIRECT_START_PLAN_ASTAR_MOTION_GOSTR_RETURN:
        if (Yaw > -138 && Yaw < -132)
        {
            linear_velocity = long_stra_vel;
            angular_velocity = 0;
            temporary_wheel_pulse_l = wheel_pulse_l;
            a_star_motion_return_origin_status = GO_B_DIRECT_START_PLAN_ASTAR_MOTION_GOSTR_RETURN;
            break;
        }
        if (obstacleSignal != none_obstacle||(&cliff_valueB)->cliffValue0 == 1)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            a_star_motion_return_origin_status = COLLISION_MORE_45_D_DIRECT_START_PLAN_ASTAR_MOTION_GOSTR_RETURN;
            break;
        }
        linear_velocity = 0;
        angular_velocity = -turn_vel;
        break;
    case COLLISION_MORE_45_D_DIRECT_START_PLAN_ASTAR_MOTION_GOSTR_RETURN:
        if (turn_start_update == 0)
        {
            turn_start_x = current_pose->x;
            turn_start_y = current_pose->y;
            turn_start_update = 1;
        }
        linear_velocity = -long_stra_vel;
        angular_velocity = 0;
        if (my_abs(turn_start_x - current_pose->x) > collision_backward_distance || my_abs(turn_start_y - current_pose->y) > collision_backward_distance)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            turn_start_update = 0;
            a_star_motion_return_origin_status = MORE_45_D_DIRECT_START_PLAN_ASTAR_MOTION_GOSTR_RETURN;
        }
        break;
        
    case E_DIRECT_START_PLAN_ASTAR_MOTION_GOSTR_RETURN:
        if (my_abs(Yaw) >= 135 || (Yaw < -135 && Yaw > -50))
        {
            a_star_motion_return_origin_status = LESS_45_E_DIRECT_START_PLAN_ASTAR_MOTION_GOSTR_RETURN;
        }
        else if (my_abs(Yaw) < 40 || (Yaw >= 40 && Yaw < 135))
        {
            a_star_motion_return_origin_status = MORE_45_E_DIRECT_START_PLAN_ASTAR_MOTION_GOSTR_RETURN;
        }
        else
        {
            temporary_wheel_pulse_l = wheel_pulse_l;
            a_star_motion_return_origin_status = GO_A_DIRECT_START_PLAN_ASTAR_MOTION_GOSTR_RETURN;
        }
        break;
    case LESS_45_E_DIRECT_START_PLAN_ASTAR_MOTION_GOSTR_RETURN:
        if (Yaw < -42 && Yaw > -48)
        {
            linear_velocity =long_stra_vel;
            angular_velocity = 0;
            temporary_wheel_pulse_l = wheel_pulse_l;
            a_star_motion_return_origin_status = GO_A_DIRECT_START_PLAN_ASTAR_MOTION_GOSTR_RETURN;
            break;
        }
        if (obstacleSignal != none_obstacle||(&cliff_valueB)->cliffValue0 == 1)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            a_star_motion_return_origin_status = COLLISION_LESS_45_E_DIRECT_START_PLAN_ASTAR_MOTION_GOSTR_RETURN;
            break;
        }
        linear_velocity = 0;
        angular_velocity = turn_vel;
        break;
    case COLLISION_LESS_45_E_DIRECT_START_PLAN_ASTAR_MOTION_GOSTR_RETURN:
        if (turn_start_update == 0)
        {
            turn_start_x = current_pose->x;
            turn_start_y = current_pose->y;
            turn_start_update = 1;
        }
        linear_velocity = -long_stra_vel;
        angular_velocity = 0;
        if (my_abs(turn_start_x - current_pose->x) > collision_backward_distance || my_abs(turn_start_y - current_pose->y) > collision_backward_distance)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            turn_start_update = 0;
            a_star_motion_return_origin_status = LESS_45_E_DIRECT_START_PLAN_ASTAR_MOTION_GOSTR_RETURN;
        }
        break;
    case MORE_45_E_DIRECT_START_PLAN_ASTAR_MOTION_GOSTR_RETURN:
        if (Yaw < -42 && Yaw > -48)
        {
            linear_velocity = long_stra_vel;
            angular_velocity = 0;
            temporary_wheel_pulse_l = wheel_pulse_l;
            a_star_motion_return_origin_status = GO_A_DIRECT_START_PLAN_ASTAR_MOTION_GOSTR_RETURN;
            break;
        }
        if (obstacleSignal != none_obstacle)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            a_star_motion_return_origin_status = COLLISION_MORE_45_E_DIRECT_START_PLAN_ASTAR_MOTION_GOSTR_RETURN;
            break;
        }
        linear_velocity = 0;
        angular_velocity = -turn_vel;
        break;
    case COLLISION_MORE_45_E_DIRECT_START_PLAN_ASTAR_MOTION_GOSTR_RETURN:
        if (turn_start_update == 0)
        {
            turn_start_x = current_pose->x;
            turn_start_y = current_pose->y;
            turn_start_update = 1;
        }
        linear_velocity = -long_stra_vel;
        angular_velocity = 0;
        if (my_abs(turn_start_x - current_pose->x) > collision_backward_distance || my_abs(turn_start_y - current_pose->y) > collision_backward_distance)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            turn_start_update = 0;
            a_star_motion_return_origin_status = MORE_45_E_DIRECT_START_PLAN_ASTAR_MOTION_GOSTR_RETURN;
        }
        break;
        
    case F_DIRECT_START_PLAN_ASTAR_MOTION_GOSTR_RETURN:
        if (my_abs(Yaw) > 5 && Yaw < 0)
        {
            a_star_motion_return_origin_status = LESS_45_F_DIRECT_START_PLAN_ASTAR_MOTION_GOSTR_RETURN;
        }
        else if (my_abs(Yaw) > 5 && Yaw > 0)
        {
            a_star_motion_return_origin_status = MORE_45_F_DIRECT_START_PLAN_ASTAR_MOTION_GOSTR_RETURN;
        }
        else
        {
            temporary_wheel_pulse_l = wheel_pulse_l;
            a_star_motion_return_origin_status = GO_B_DIRECT_START_PLAN_ASTAR_MOTION_GOSTR_RETURN;
        }
        break;
    case LESS_45_F_DIRECT_START_PLAN_ASTAR_MOTION_GOSTR_RETURN:
        if (my_abs(Yaw) < 3)
        {
            linear_velocity = long_stra_vel;
            angular_velocity = 0;
            temporary_wheel_pulse_l = wheel_pulse_l;
            a_star_motion_return_origin_status = GO_B_DIRECT_START_PLAN_ASTAR_MOTION_GOSTR_RETURN;
            break;
        }
        if (obstacleSignal != none_obstacle||(&cliff_valueB)->cliffValue0 == 1)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            a_star_motion_return_origin_status = COLLISION_LESS_45_F_DIRECT_START_PLAN_ASTAR_MOTION_GOSTR_RETURN;
            break;
        }
        linear_velocity = 0;
        angular_velocity = turn_vel;
        break;
    case COLLISION_LESS_45_F_DIRECT_START_PLAN_ASTAR_MOTION_GOSTR_RETURN:
        if (turn_start_update == 0)
        {
            turn_start_x = current_pose->x;
            turn_start_y = current_pose->y;
            turn_start_update = 1;
        }
        linear_velocity = -long_stra_vel;
        angular_velocity = 0;
        if (my_abs(turn_start_x - current_pose->x) > collision_backward_distance || my_abs(turn_start_y - current_pose->y) > collision_backward_distance)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            turn_start_update = 0;
            a_star_motion_return_origin_status = LESS_45_F_DIRECT_START_PLAN_ASTAR_MOTION_GOSTR_RETURN;
        }
        break;
    case MORE_45_F_DIRECT_START_PLAN_ASTAR_MOTION_GOSTR_RETURN:
        if (my_abs(Yaw) < 3)
        {
            linear_velocity = long_stra_vel;
            angular_velocity = 0;
            temporary_wheel_pulse_l = wheel_pulse_l;
            a_star_motion_return_origin_status = GO_B_DIRECT_START_PLAN_ASTAR_MOTION_GOSTR_RETURN;
            break;
        }
        if (obstacleSignal != none_obstacle||(&cliff_valueB)->cliffValue0 == 1)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            a_star_motion_return_origin_status = COLLISION_MORE_45_F_DIRECT_START_PLAN_ASTAR_MOTION_GOSTR_RETURN;
            break;
        }
        linear_velocity = 0;
        angular_velocity = -turn_vel;
        break;
    case COLLISION_MORE_45_F_DIRECT_START_PLAN_ASTAR_MOTION_GOSTR_RETURN:
        if (turn_start_update == 0)
        {
            turn_start_x = current_pose->x;
            turn_start_y = current_pose->y;
            turn_start_update = 1;
        }
        linear_velocity = -long_stra_vel;
        angular_velocity = 0;
        if (my_abs(turn_start_x - current_pose->x) > collision_backward_distance || my_abs(turn_start_y - current_pose->y) > collision_backward_distance)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            turn_start_update = 0;
            a_star_motion_return_origin_status = MORE_45_F_DIRECT_START_PLAN_ASTAR_MOTION_GOSTR_RETURN;
        }
        break;
        
    case G_DIRECT_START_PLAN_ASTAR_MOTION_GOSTR_RETURN:
        if (my_abs(Yaw) <= 40 || (Yaw < -40 && Yaw > -135))
        {
            a_star_motion_return_origin_status = LESS_45_G_DIRECT_START_PLAN_ASTAR_MOTION_GOSTR_RETURN;
        }
        else if (my_abs(Yaw) >= 135 || (Yaw > 50 && Yaw < 135))
        {
            a_star_motion_return_origin_status = MORE_45_G_DIRECT_START_PLAN_ASTAR_MOTION_GOSTR_RETURN;
        }
        else
        {
            temporary_wheel_pulse_l = wheel_pulse_l;
            a_star_motion_return_origin_status = GO_A_DIRECT_START_PLAN_ASTAR_MOTION_GOSTR_RETURN;
        }
        break;
    case LESS_45_G_DIRECT_START_PLAN_ASTAR_MOTION_GOSTR_RETURN:
        if (Yaw < 48 && Yaw > 42)
        {
            linear_velocity = long_stra_vel;
            angular_velocity = 0;
            temporary_wheel_pulse_l = wheel_pulse_l;
            a_star_motion_return_origin_status = GO_A_DIRECT_START_PLAN_ASTAR_MOTION_GOSTR_RETURN;
            break;
        }
        if (obstacleSignal != none_obstacle||(&cliff_valueB)->cliffValue0 == 1)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            a_star_motion_return_origin_status = COLLISION_LESS_45_G_DIRECT_START_PLAN_ASTAR_MOTION_GOSTR_RETURN;
            break;
        }
        linear_velocity = 0;
        angular_velocity = turn_vel;
        break;
    case COLLISION_LESS_45_G_DIRECT_START_PLAN_ASTAR_MOTION_GOSTR_RETURN:
        if (turn_start_update == 0)
        {
            turn_start_x = current_pose->x;
            turn_start_y = current_pose->y;
            turn_start_update = 1;
        }
        linear_velocity = -long_stra_vel;
        angular_velocity = 0;
        if (my_abs(turn_start_x - current_pose->x) > collision_backward_distance || my_abs(turn_start_y - current_pose->y) > collision_backward_distance)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            turn_start_update = 0;
            a_star_motion_return_origin_status = LESS_45_G_DIRECT_START_PLAN_ASTAR_MOTION_GOSTR_RETURN;
        }
        break;
    case MORE_45_G_DIRECT_START_PLAN_ASTAR_MOTION_GOSTR_RETURN:
        if (Yaw < 48 && Yaw > 42)
        {
            linear_velocity = long_stra_vel;
            angular_velocity = 0;
            temporary_wheel_pulse_l = wheel_pulse_l;
            a_star_motion_return_origin_status = GO_A_DIRECT_START_PLAN_ASTAR_MOTION_GOSTR_RETURN;
            break;
        }
        if (obstacleSignal != none_obstacle||(&cliff_valueB)->cliffValue0 == 1)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            a_star_motion_return_origin_status = COLLISION_MORE_45_G_DIRECT_START_PLAN_ASTAR_MOTION_GOSTR_RETURN;
            break;
        }
        linear_velocity = 0;
        angular_velocity = -turn_vel;
        break;
    case COLLISION_MORE_45_G_DIRECT_START_PLAN_ASTAR_MOTION_GOSTR_RETURN:
        if (turn_start_update == 0)
        {
            turn_start_x = current_pose->x;
            turn_start_y = current_pose->y;
            turn_start_update = 1;
        }
        linear_velocity = -long_stra_vel;
        angular_velocity = 0;
        if (my_abs(turn_start_x - current_pose->x) > collision_backward_distance || my_abs(turn_start_y - current_pose->y) > collision_backward_distance)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            turn_start_update = 0;
            a_star_motion_return_origin_status = MORE_45_G_DIRECT_START_PLAN_ASTAR_MOTION_GOSTR_RETURN;
        }
        break;
        
    case H_DIRECT_START_PLAN_ASTAR_MOTION_GOSTR_RETURN:
        if (my_abs(Yaw) < 85)
        {
            a_star_motion_return_origin_status = LESS_45_H_DIRECT_START_PLAN_ASTAR_MOTION_GOSTR_RETURN;
        }
        else if (my_abs(Yaw) > 95 || (Yaw >= -95 && Yaw <= -85))
        {
            a_star_motion_return_origin_status = MORE_45_H_DIRECT_START_PLAN_ASTAR_MOTION_GOSTR_RETURN;
        }
        else
        {
            temporary_wheel_pulse_l = wheel_pulse_l;
            a_star_motion_return_origin_status = GO_B_DIRECT_START_PLAN_ASTAR_MOTION_GOSTR_RETURN;
        }
        break;
    case LESS_45_H_DIRECT_START_PLAN_ASTAR_MOTION_GOSTR_RETURN:
        if (Yaw > 87 && Yaw < 93)
        {
            linear_velocity = long_stra_vel;
            angular_velocity = 0;
            temporary_wheel_pulse_l = wheel_pulse_l;
            a_star_motion_return_origin_status = GO_B_DIRECT_START_PLAN_ASTAR_MOTION_GOSTR_RETURN;
            break;
        }
        if (obstacleSignal != none_obstacle||(&cliff_valueB)->cliffValue0 == 1)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            a_star_motion_return_origin_status = COLLISION_LESS_45_H_DIRECT_START_PLAN_ASTAR_MOTION_GOSTR_RETURN;
            break;
        }
        linear_velocity = 0;
        angular_velocity = turn_vel;
        break;
    case COLLISION_LESS_45_H_DIRECT_START_PLAN_ASTAR_MOTION_GOSTR_RETURN:
        if (turn_start_update == 0)
        {
            turn_start_x = current_pose->x;
            turn_start_y = current_pose->y;
            turn_start_update = 1;
        }
        linear_velocity = -long_stra_vel;
        angular_velocity = 0;
        if (my_abs(turn_start_x - current_pose->x) > collision_backward_distance || my_abs(turn_start_y - current_pose->y) > collision_backward_distance)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            turn_start_update = 0;
            a_star_motion_return_origin_status = LESS_45_H_DIRECT_START_PLAN_ASTAR_MOTION_GOSTR_RETURN;
        }
        break;
    case MORE_45_H_DIRECT_START_PLAN_ASTAR_MOTION_GOSTR_RETURN:
        if (Yaw > 87 && Yaw < 93)
        {
            linear_velocity = long_stra_vel;
            angular_velocity = 0;
            temporary_wheel_pulse_l = wheel_pulse_l;
            a_star_motion_return_origin_status = GO_B_DIRECT_START_PLAN_ASTAR_MOTION_GOSTR_RETURN;
            break;
        }
        if (obstacleSignal != none_obstacle||(&cliff_valueB)->cliffValue0 == 1)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            a_star_motion_return_origin_status = COLLISION_MORE_45_H_DIRECT_START_PLAN_ASTAR_MOTION_GOSTR_RETURN;
            break;
        }
        linear_velocity = 0;
        angular_velocity = -turn_vel;
        break;
    case COLLISION_MORE_45_H_DIRECT_START_PLAN_ASTAR_MOTION_GOSTR_RETURN:
        if (turn_start_update == 0)
        {
            turn_start_x = current_pose->x;
            turn_start_y = current_pose->y;
            turn_start_update = 1;
        }
        linear_velocity = -long_stra_vel;
        angular_velocity = 0;
        if (my_abs(turn_start_x - current_pose->x) > collision_backward_distance || my_abs(turn_start_y - current_pose->y) > collision_backward_distance)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            turn_start_update = 0;
            a_star_motion_return_origin_status = MORE_45_H_DIRECT_START_PLAN_ASTAR_MOTION_GOSTR_RETURN;
        }
        break;
        
    case A_STAR_COMPLETED:
        linear_velocity = 0;
        angular_velocity = 0;
        complete_flag = 1;
        startMotionStep = 0;
        motionSteps = 0;
        AStarMotionNumber = 0;
        a_star_motion_return_origin_status = 0;
        break;
    }
    return complete_flag;
}


unsigned char  AStarCollision(POSE *current_pose, unsigned char obstacleSignal){
    int Yaw;
    unsigned char complete_flag = 0;
    Yaw = current_pose->orientation;
    Yaw = Yaw /100;
    //log_debug("a_star_collision_status =======>>>,%x,\n",a_star_collision_status);
    switch (a_star_collision_status)
    {
    case 0:
        a_star_collision_status = A_STAR_COLLISION;
        break;
    case A_STAR_COLLISION:
        //log_debug("A_STAR_COLLISION_GOSTR");
        if(y_less_map==true){
            current_pose->y=global_pose_y;
            current_pose->y=current_pose->y-y_error;
        }
        if(over_clean_finish == true){
            current_pose->x=global_pose_x;
            current_pose->y=global_pose_y;
            current_pose->x=current_pose->x-x_error;
            current_pose->y=current_pose->y-y_error;
        }	
        if (my_abs(current_pose->x) > 2*return_origin_distance || my_abs(current_pose->y) > 2*return_origin_distance)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            a_star_collision_status = START_A_STAR_COLLISION;
        }
        else
        {
            linear_velocity = 0;
            angular_velocity = 0;
            complete_flag = 2;
            a_star_collision_status = A_STAR_COLLISION_COMPLETED;
        }
        break;
    case START_A_STAR_COLLISION:
        if (obstacleSignal == left_obstacle)
        {
            Astarmarkingobstacle = left_obstacle;
        }
        else if (obstacleSignal == right_obstacle)
        {
            Astarmarkingobstacle = right_obstacle;
        }
        else
        {
            Astarmarkingobstacle = front_obstacle;
        }
        a_star_collision_status = BACK_START_A_STAR_COLLISION;
        break;
    case BACK_START_A_STAR_COLLISION:
        if (turn_start_update == 0)
        {
            turn_start_x = current_pose->x;
            turn_start_y = current_pose->y;
            turn_start_update = 1;
        }
        linear_velocity = -long_stra_vel;
        angular_velocity = 0;
        if (my_abs(turn_start_x - current_pose->x) > side_backward_distance || my_abs(turn_start_y - current_pose->y) > side_backward_distance)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            turn_start_update = 0;
            temporary_yaw = Yaw;
            if (Astarmarkingobstacle == left_obstacle)
            {
                a_star_collision_status = LEFT_OBSTACLE_START_A_STAR_COLLISION;
            }
            else if (Astarmarkingobstacle ==  right_obstacle)
            {
                a_star_collision_status = RIGHT_OBSTACLE_START_A_STAR_COLLISION;
            }
            else
            {
                a_star_collision_status = FRONT_OBSTACLE_START_A_STAR_COLLISION;
            }
        }
        break;
    case LEFT_OBSTACLE_START_A_STAR_COLLISION:
        if (my_abs(temporary_yaw - Yaw) > 45)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            a_star_collision_status = GO_LEFT_OBSTACLE_START_A_STAR_COLLISION;
            break;
        }
        if (obstacleSignal != none_obstacle||(&cliff_valueB)->cliffValue0 == 1)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            a_star_collision_status = COLLISION_LEFT_OBSTACLE_START_A_STAR_COLLISION;
            break;
        }
        linear_velocity = 0;
        angular_velocity = -turn_vel;
        break;
    case COLLISION_LEFT_OBSTACLE_START_A_STAR_COLLISION:
        if (turn_start_update == 0)
        {
            turn_start_x = current_pose->x;
            turn_start_y = current_pose->y;
            turn_start_update = 1;
        }
        linear_velocity = -long_stra_vel;
        angular_velocity = 0;
        if (my_abs(turn_start_x - current_pose->x) > collision_backward_distance || my_abs(turn_start_y - current_pose->y) > collision_backward_distance)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            turn_start_update = 0;
            a_star_collision_status = LEFT_OBSTACLE_START_A_STAR_COLLISION;
        }
        break;
    case GO_LEFT_OBSTACLE_START_A_STAR_COLLISION:
        if (turn_start_update == 0)
        {
            turn_start_x = current_pose->x;
            turn_start_y = current_pose->y;
            turn_start_update = 1;
        }
        if (obstacleSignal != none_obstacle||(&cliff_valueB)->cliffValue0 == 1)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            turn_start_update = 0;
            a_star_collision_total++;
            a_star_collision_status = BACK_START_A_STAR_COLLISION;
            break;
        }
        if (a_star_collision_total > 4)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            turn_start_update = 0;
            a_star_collision_status = RECALCULATE_A_STAR_COLLISION_COMPLETED;
            break;
        }
        if (my_abs(turn_start_x - current_pose->x) > star_collision_go || my_abs(turn_start_y - current_pose->y) > star_collision_go)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            turn_start_update = 0;
            a_star_collision_status = A_STAR_COLLISION_COMPLETED;
        }
        linear_velocity = long_stra_vel;
        angular_velocity = 0;
        break;
    case RIGHT_OBSTACLE_START_A_STAR_COLLISION:
        if (my_abs(temporary_yaw - Yaw) > 45)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            a_star_collision_status = GO_RIGHT_OBSTACLE_START_A_STAR_COLLISION;
            break;
        }
        if (obstacleSignal != none_obstacle||(&cliff_valueB)->cliffValue0 == 1)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            a_star_collision_status = COLLISION_RIGHT_OBSTACLE_START_A_STAR_COLLISION;
            break;
        }
        linear_velocity = 0;
        angular_velocity = turn_vel;
        break;
    case COLLISION_RIGHT_OBSTACLE_START_A_STAR_COLLISION:
        if (turn_start_update == 0)
        {
            turn_start_x = current_pose->x;
            turn_start_y = current_pose->y;
            turn_start_update = 1;
        }
        linear_velocity = -long_stra_vel;
        angular_velocity = 0;
        if (my_abs(turn_start_x - current_pose->x) > collision_backward_distance || my_abs(turn_start_y - current_pose->y) > collision_backward_distance)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            turn_start_update = 0;
            a_star_collision_status = RIGHT_OBSTACLE_START_A_STAR_COLLISION;
        }
        break;
    case GO_RIGHT_OBSTACLE_START_A_STAR_COLLISION:
        if (turn_start_update == 0)
        {
            turn_start_x = current_pose->x;
            turn_start_y = current_pose->y;
            turn_start_update = 1;
        }
        linear_velocity = long_stra_vel;
        angular_velocity = 0;
        if (obstacleSignal != none_obstacle||(&cliff_valueB)->cliffValue0 == 1)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            turn_start_update = 0;
            a_star_collision_total++;
            a_star_collision_status = BACK_START_A_STAR_COLLISION;
            break;
        }
        if (a_star_collision_total > 4)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            turn_start_update = 0;
            a_star_collision_status = RECALCULATE_A_STAR_COLLISION_COMPLETED;
            break;
        }
        if (my_abs(turn_start_x - current_pose->x) > star_collision_go || my_abs(turn_start_y - current_pose->y) > star_collision_go)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            turn_start_update = 0;
            a_star_collision_status = A_STAR_COLLISION_COMPLETED;
        }
        break;
    case FRONT_OBSTACLE_START_A_STAR_COLLISION:
        if (my_abs(temporary_yaw - Yaw) > 60)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            a_star_collision_status = GO_FRONT_OBSTACLE_START_A_STAR_COLLISION;
            break;
        }
        if (obstacleSignal != none_obstacle||(&cliff_valueB)->cliffValue0 == 1)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            a_star_collision_status = COLLISION_FRONT_OBSTACLE_START_A_STAR_COLLISION;
            break;
        }
        linear_velocity = 0;
        angular_velocity = turn_vel;
        break;
    case COLLISION_FRONT_OBSTACLE_START_A_STAR_COLLISION:
        if (turn_start_update == 0)
        {
            turn_start_x = current_pose->x;
            turn_start_y = current_pose->y;
            turn_start_update = 1;
        }
        linear_velocity = -long_stra_vel;
        angular_velocity = 0;
        if (my_abs(turn_start_x - current_pose->x) > collision_backward_distance || my_abs(turn_start_y - current_pose->y) > collision_backward_distance)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            turn_start_update = 0;
            a_star_collision_status = FRONT_OBSTACLE_START_A_STAR_COLLISION;
        }
        break;
    case GO_FRONT_OBSTACLE_START_A_STAR_COLLISION:
        if (turn_start_update == 0)
        {
            turn_start_x = current_pose->x;
            turn_start_y = current_pose->y;
            turn_start_update = 1;
        }
        linear_velocity = long_stra_vel;
        angular_velocity = 0;
        if (obstacleSignal != none_obstacle||(&cliff_valueB)->cliffValue0 == 1)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            turn_start_update = 0;
            a_star_collision_total++;
            a_star_collision_status = BACK_START_A_STAR_COLLISION;
            break;
        }
        if (a_star_collision_total > 3)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            turn_start_update = 0;
            a_star_collision_status = RECALCULATE_A_STAR_COLLISION_COMPLETED;
            break;
        }
        if (my_abs(turn_start_x - current_pose->x) > star_collision_go || my_abs(turn_start_y - current_pose->y) > star_collision_go)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            turn_start_update = 0;
            a_star_collision_status = A_STAR_COLLISION_COMPLETED;
        }
        break;
    case RECALCULATE_A_STAR_COLLISION_COMPLETED:
        linear_velocity = 0;
        angular_velocity = 0;
        a_star_collision_status = 0;
        a_star_collision_total = 0;
        complete_flag = 1;
        break;
    case A_STAR_COLLISION_COMPLETED:
        linear_velocity = 0;
        angular_velocity = 0;
        a_star_collision_status = 0;
        a_star_collision_total = 0;
        complete_flag = 1;
        break;
    }
    return complete_flag;
}
unsigned char  CloseEdgedMap(POSE *current_pose, unsigned char obstacleSignal){
    int Yaw;
    unsigned char i;
    unsigned char j;
    unsigned char k;
    unsigned char ij;
    unsigned char complete_flag = 0;
    bool end_x=false;
    Yaw = current_pose->orientation;
    Yaw = Yaw/100;
    switch (close_edge_map_run_step_status){
    case 0:
        closeedgesmap=false;
        step=0;
        motionSteps=0;
        mapstopupdate=true;
        detection_close=false;
        linear_velocity = 0;
        angular_velocity = 0;
        close_edge_map_run_step_status=START_TURN_CLOCK_TARGET_CLOSE_EDGE_MAP;
        break;
    case START_TURN_CLOCK_TARGET_CLOSE_EDGE_MAP:
//	DelimmaNumber==2;	
		for(i=0;i<GRIDWIDTH;i++){
            for(j=GRIDHEIGHT-1;j>0;j--){
                if(gridmap.map[i][j]!=125){
                    k=i;
				 close_edge_min_x=i;
				 close_edge_min_y=j;					
                    end_x=true;
                    break;
                }
            }
            if(end_x==true){
                end_x=false;
                break;
            }
			i++;
        }
		for(j=GRIDHEIGHT-1;j>0;j--){
			if(gridmap.map[close_edge_min_x+5][j]!=125){
				close_edge_min_x=close_edge_min_x+5;
				close_edge_min_y=j;
				break;				
			}	
		}
//	DelimmaNumber==4;	
        for (i=GRIDWIDTH-1;i>0;i--){
            for(j=0;j<GRIDHEIGHT;j++){
                if(gridmap.map[i][j]!=125){
                 close_edge_max_x=i;
                 close_edge_max_y=j;					
                    edge_length_start=i-k;
                    edge_length_start=100*edge_length_start;
                    end_x=true;
                    break;
                }
            }
            if(end_x==true){
                end_x=false;
                break;
            }
			i--;
        }
		for(j=0;j<GRIDWIDTH;j++){
			if(gridmap.map[close_edge_max_x-5][j]!=125){
				 close_edge_max_x=close_edge_max_x-5;
                 close_edge_max_y=j;					
				break;				
			}	
		} 
		close_edge_map_run_step_status=CALCULATION_DELIMMANUMBER_CLOSE_EDGE_MAP;
		break;
	case CALCULATION_DELIMMANUMBER_CLOSE_EDGE_MAP:
//	DelimmaNumber==3;	
		 for(j=0;j<GRIDWIDTH;j++){
            for(i=0;i<GRIDHEIGHT;i++){
                if(gridmap.map[i][j]!=125){
                    k=j;
                 close_l_edge_max_x=i;
                 close_l_edge_max_y=j;
                    end_x=true;
                    break;
                }
            }
            if(end_x==true){
                end_x=false;
                break;
            }
			j++;
        }
		 for(i=0;i<GRIDHEIGHT;i++){
			  if(gridmap.map[i][close_l_edge_max_y+5]!=125){
				 close_l_edge_max_x=i;
                 close_l_edge_max_y=close_l_edge_max_y+5;
				  break;
			  }
		 }
//	DelimmaNumber==1;	
        for(j=GRIDWIDTH-1;j>0;j--){
            for(i=GRIDHEIGHT-1;i>0;i--){
                if(gridmap.map[i][j]!=125){
				 close_r_edge_min_x=i;
                 close_r_edge_min_y=j;					
                    if(100*(j-k)>edge_length_start){
                        edge_length_start=100*(j-k)/2-500;
                    }
                    else{
                        edge_length_start=edge_length_start/2-500;
                    }
                    end_x=true;
                    break;
                }
            }
            if(end_x==true){
                end_x=false;
                break;
            }
			j--;
        }
		for(i=GRIDHEIGHT-1;i>0;i--){
			if(gridmap.map[i][close_r_edge_min_y-5]!=125){
				close_r_edge_min_x=i;
                close_r_edge_min_y=close_r_edge_min_y-5;
				break;				
			}
		}
		
       if( my_abs(temporary_yaw)>90){
            close_edge_map_run_step_status=START_LOOP_CLOSE_EDGE_MAP;
        }
        else{
            close_edge_map_run_step_status=LESS_ABS176__CLOSE_EDGE_MAP;
        }
        break;				
    case LESS_ABS176__CLOSE_EDGE_MAP:
        linear_velocity = 0;
        angular_velocity =turn_vel;
        if (my_abs(Yaw)>170)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            close_edge_map_run_step_status=START_LOOP_CLOSE_EDGE_MAP;
            break;
        }
        if (obstacleSignal!=none_obstacle||(&cliff_valueB)->cliffValue0 == 1)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            close_edge_map_run_step_status=COLLISION_LESS_ABS176_CLOSE_EDGE_MAP;
            break;
        }
        break;
    case COLLISION_LESS_ABS176_CLOSE_EDGE_MAP:
        if(turn_start_update == 0)
        {
            turn_start_x = current_pose->x;
            turn_start_y = current_pose->y;
            turn_start_update = 1;
        }
        linear_velocity = -long_stra_vel;
        angular_velocity = 0;
        if (my_abs(turn_start_x - current_pose->x) > collision_backward_distance || my_abs(turn_start_y - current_pose->y) > collision_backward_distance)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            turn_start_update = 0;
            close_edge_map_run_step_status= LESS_ABS176__CLOSE_EDGE_MAP;
            break;
        }
        break;
    case START_LOOP_CLOSE_EDGE_MAP:
        linear_velocity = 0;
        angular_velocity = 0;
        last_position_x = current_pose->x;
        last_position_y = current_pose->y;
        last_position_xx = current_pose->x;
        last_position_yy = current_pose->y;
        DelimmaNumber=1;
        close_edge_map_run_step_status=LOOP_CLOSE_EDGE_MAP;
        break;
    case LOOP_CLOSE_EDGE_MAP:	
		right_edge_judgment_repeat();
//        if(adcRealTime[9]>100&&adcRealTime[9]<1500){
//            delimma_edge=0;
//            linear_velocity = 200;
//            angular_velocity = 0;
//        }
//        if(adcRealTime[9]>=1500){
//            delimma_edge=0;
//            linear_velocity = 200;
//            angular_velocity = 10;
//        }
//        if(adcRealTime[9]<100){
//            if(delimma_edge<10){
//                delimma_edge++;
//                linear_velocity = 200;
//                angular_velocity = -10;
//            }
//            else{
//                linear_velocity = 100;
//                angular_velocity = -20;
//            }
//        }
        if (obstacleSignal!=none_obstacle||(&cliff_valueB)->cliffValue0 == 1){
            linear_velocity = 0;
            angular_velocity = 0;
            close_edge_map_run_step_status= COLLISION_LOOP_CLOSE_EDGE_MAP;
            break;
        }
        if(detection_close==false){
            if(my_abs(last_position_xx - current_pose->x)>edge_length_start||my_abs(last_position_yy - current_pose->y)>edge_length_start){
                detection_close=true;
            }
        }
        if(my_abs(current_pose->x)>half_map_wide-2*GRIDWIDTH||my_abs(current_pose->y)>half_map_wide-2*GRIDWIDTH){
            close_edge_map_run_step_status=MORE_LOOP_CLOSE_EDGE_MAP;
            break;
        }
        i=(current_pose->x+half_map_wide)/GRIDWIDTH;
        j=(current_pose->y+half_map_wide)/GRIDWIDTH;
        if(my_abs(last_position_x - current_pose->x)>lateral_move_distance||my_abs(last_position_y - current_pose->y)>lateral_move_distance){
            end_x=false;
            if(i>3&&i<96&&j>3&&j<96){
                for(k=i-4;k<=i+4;k++){
                    for(ij=j-4;ij<=j+4;ij++){
                        if(gridmap.map[k][ij]!=125){
                            end_x=true;
                            break;
                        }
                    }
                    if(end_x==true){
                        break;
                    }
					k++;
                }
                if(end_x==true){
                    last_position_x = current_pose->x;
                    last_position_y = current_pose->y;
                    close_edge_map_run_step_status=LOOP_CLOSE_EDGE_MAP;
                    break;
                }
                else{
                    close_edge_map_run_step_status= COMPLETE_LOOP_CLOSE_EDGE_MAP;
                    break;
                }
            }
            else{
                last_position_x = current_pose->x;
                last_position_y = current_pose->y;
                close_edge_map_run_step_status=LOOP_CLOSE_EDGE_MAP;
            }
            break;
        }
        if(DelimmaNumber==1){
            if(my_abs(Yaw)<10){
                if(Yaw<0){
                    linear_velocity = 100;
                    angular_velocity = -20;
                }
                else{
					temporary_yaw=Yaw;
					close_edge_map_run_step_status=LEFT_COLLISION_LOOP_CLOSE_EDGE_MAP;
					break;
                }
            }
            if(my_abs(i-close_edge_min_x)<10&&my_abs(j-close_edge_min_x)<10){
                motionSteps++;
                DelimmaNumber=2;
                step=0;
            }
            if(my_abs(i-close_l_edge_max_x)<10&&my_abs(j-close_l_edge_max_y)<10){
                motionSteps++;
                DelimmaNumber=3;
                step=0;
            }
        }
        else if(DelimmaNumber==2){
            if(Yaw<100&&Yaw>80){
                if(Yaw<90){
                    linear_velocity = 100;
                    angular_velocity = -20;
                }
                else{
					temporary_yaw=Yaw;
					close_edge_map_run_step_status=LEFT_COLLISION_LOOP_CLOSE_EDGE_MAP;
					break;
                }
            }
            if(my_abs(i-close_l_edge_max_x)<10&&my_abs(j-close_l_edge_max_y)<10){
                motionSteps++;
                DelimmaNumber=3;
                step=0;
            }
            if(my_abs(i-close_edge_max_x)<10&&my_abs(j-close_edge_max_y)<10){
                motionSteps++;
                DelimmaNumber=4;
                step=0;
            }
        }
        else if(DelimmaNumber==3){
            if(my_abs(Yaw)>170){
                if(Yaw>0){
                    linear_velocity = 100;
                    angular_velocity = -20;
                }
                else{
					temporary_yaw=Yaw;
					close_edge_map_run_step_status=LEFT_COLLISION_LOOP_CLOSE_EDGE_MAP;
					break;
                }
            }
            if(my_abs(i-close_edge_max_x)<10&&my_abs(j-close_edge_max_y)<10){
                motionSteps++;
                DelimmaNumber=4;
                step=0;
            }
            if(detection_close==true){
                if(my_abs(i-close_r_edge_min_x)<10&&my_abs(j-close_r_edge_min_y)<10){
                    motionSteps++;
                    DelimmaNumber=1;
                    step=0;
                    close_edge_map_run_step_status=COMPLETE_CLOSE_EDGE_MAP;
                    break;
                }
            }
        }
        else if(DelimmaNumber==4){
            if(Yaw>-100&&Yaw<-80){
                if(Yaw<-90){
                    linear_velocity = 100;
                    angular_velocity = -20;
                }
                else{
					temporary_yaw=Yaw;
 					close_edge_map_run_step_status=LEFT_COLLISION_LOOP_CLOSE_EDGE_MAP;
					break;
                }
            }
            if(detection_close==true){
                if(my_abs(i-close_r_edge_min_x)<10&&my_abs(j-close_r_edge_min_y)<10){
                    motionSteps++;
                    DelimmaNumber=1;
                }
                if(my_abs(i-close_edge_min_x)<10&&my_abs(j-close_edge_min_x)<10){
                    motionSteps++;
                    DelimmaNumber=2;
                }
                if(DelimmaNumber!=4){
                    step=0;
                    motionSteps++;
                    close_edge_map_run_step_status= COMPLETE_CLOSE_EDGE_MAP;
                    break;
                }
            }
        }
        else{
            linear_velocity = long_stra_vel;
            angular_velocity = 0;
        }
        if(motionSteps>4){
            step=0;
            close_edge_map_run_step_status= COMPLETE_CLOSE_EDGE_MAP;
        }
        break;
    case COLLISION_LOOP_CLOSE_EDGE_MAP:
        if(turn_start_update == 0)
        {
            turn_start_x = current_pose->x;
            turn_start_y = current_pose->y;
            turn_start_update = 1;
            if (obstacleSignal == left_obstacle){
                Astarmarkingobstacle = left_obstacle;
            }
            else if (obstacleSignal == right_obstacle){
                Astarmarkingobstacle = right_obstacle;
            }
            else{
                Astarmarkingobstacle = front_obstacle;
            }
        }
        linear_velocity = -long_stra_vel;
        angular_velocity = 0;
        if (my_abs(turn_start_x - current_pose->x) > collision_backward_distance || my_abs(turn_start_y - current_pose->y) > collision_backward_distance)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            turn_start_update = 0;
			temporary_yaw=Yaw;
            if(Astarmarkingobstacle == left_obstacle){
                close_edge_map_run_step_status=LEFT_COLLISION_LOOP_CLOSE_EDGE_MAP;
            }
            else if(Astarmarkingobstacle == right_obstacle){
                close_edge_map_run_step_status=RIGHT_COLLISION_LOOP_CLOSE_EDGE_MAP;
            }
            else{
                close_edge_map_run_step_status=FRONT_COLLISION_LOOP_CLOSE_EDGE_MAP;
            }
            break;
        }
        break;
    case LEFT_COLLISION_LOOP_CLOSE_EDGE_MAP:
        if (my_abs(temporary_yaw - Yaw) > 30)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            last_position_x = current_pose->x;
            last_position_y = current_pose->y;
            close_edge_map_run_step_status = LOOP_CLOSE_EDGE_MAP;
            break;
        }
        if (obstacleSignal != none_obstacle||(&cliff_valueB)->cliffValue0 == 1)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            close_edge_map_run_step_status = COLLISION_LEFT_COLLISION_LOOP_CLOSE_EDGE_MAP;
            break;
        }
        linear_velocity = 0;
        angular_velocity = turn_vel;
        break;
    case RIGHT_COLLISION_LOOP_CLOSE_EDGE_MAP:
        if (my_abs(temporary_yaw - Yaw) > 15)
        {
            if(my_abs(Yaw) > 170){
                temporary_yaw = Yaw;
                close_edge_map_run_step_status=RIGHT_COLLISION_LOOP_CLOSE_EDGE_MAP;
                break;
            }
            linear_velocity = 0;
            angular_velocity = 0;
            last_position_x = current_pose->x;
            last_position_y = current_pose->y;
            close_edge_map_run_step_status = LOOP_CLOSE_EDGE_MAP;
            break;
        }
        if (obstacleSignal != none_obstacle||(&cliff_valueB)->cliffValue0 == 1)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            close_edge_map_run_step_status = COLLISION_LEFT_COLLISION_LOOP_CLOSE_EDGE_MAP;
            break;
        }
        linear_velocity = 0;
        angular_velocity = turn_vel;
        break;
    case FRONT_COLLISION_LOOP_CLOSE_EDGE_MAP:
        if (my_abs(temporary_yaw - Yaw) > 30)
        {
            if(my_abs(Yaw) > 175){
                temporary_yaw = Yaw;
                close_edge_map_run_step_status=FRONT_COLLISION_LOOP_CLOSE_EDGE_MAP;
                break;
            }
            linear_velocity = 0;
            angular_velocity = 0;
            last_position_x = current_pose->x;
            last_position_y = current_pose->y;
            close_edge_map_run_step_status = LOOP_CLOSE_EDGE_MAP;
            break;
        }
        if (obstacleSignal != none_obstacle||(&cliff_valueB)->cliffValue0 == 1)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            close_edge_map_run_step_status = COLLISION_LEFT_COLLISION_LOOP_CLOSE_EDGE_MAP;
            break;
        }
        linear_velocity = 0;
        angular_velocity = turn_vel;
        break;
    case COLLISION_LEFT_COLLISION_LOOP_CLOSE_EDGE_MAP:
        if (turn_start_update == 0)
        {
            turn_start_x = current_pose->x;
            turn_start_y = current_pose->y;
            turn_start_update = 1;
        }
        linear_velocity = -long_stra_vel;
        angular_velocity = 0;
        if (my_abs(turn_start_x - current_pose->x) > collision_backward_distance || my_abs(turn_start_y - current_pose->y) > collision_backward_distance)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            turn_start_update = 0;
            if(Astarmarkingobstacle == left_obstacle){
                close_edge_map_run_step_status=LEFT_COLLISION_LOOP_CLOSE_EDGE_MAP;
            }
            else if(Astarmarkingobstacle == right_obstacle){
                close_edge_map_run_step_status=RIGHT_COLLISION_LOOP_CLOSE_EDGE_MAP;
            }
            else{
                close_edge_map_run_step_status=FRONT_COLLISION_LOOP_CLOSE_EDGE_MAP;
            }
            break;
        }
        break;
    case MORE_LOOP_CLOSE_EDGE_MAP:
		left_edge_judgment_repeat();
//        if(adcRealTime[8]>100&&adcRealTime[8]<1500){
//            delimma_edge=0;
//            linear_velocity = 200;
//            angular_velocity = 0;
//        }
//        if(adcRealTime[8]>=1500){
//            delimma_edge=0;
//            linear_velocity = 200;
//            angular_velocity = -10;
//        }
//        if(adcRealTime[8]<100){
//            if(delimma_edge<10){
//                delimma_edge++;
//                linear_velocity = 200;
//                angular_velocity = 10;
//            }
//            else{
//                linear_velocity = 100;
//                angular_velocity = 20;
//            }
//        }
        if (obstacleSignal!=none_obstacle||(&cliff_valueB)->cliffValue0 == 1){
            linear_velocity = 0;
            angular_velocity = 0;
            close_edge_map_run_step_status= COLLISION_LOOP_CLOSE_EDGE_MAP;
            break;
        }
        if(current_pose->x<half_map_wide-1000){
            last_position_x = current_pose->x;
            last_position_y = current_pose->y;
            close_edge_map_run_step_status = LOOP_CLOSE_EDGE_MAP;
            break;
        }
        if(my_abs(current_pose->x)>half_map_wide||my_abs(current_pose->y)>half_map_wide){
            if(my_abs(current_pose->x)>half_map_wide){
                if(my_abs(current_pose->y)>half_map_wide){
                    close_edge_map_run_step_status=COMPLETE_CLOSE_EDGE_MAP;
                    break;
                }
                else{
                    if(current_pose->x>half_map_wide){
                        close_edge_map_run_step_status=X_MORE_LOOP_CLOSE_EDGE_MAP;
                    }
                    else{
                        close_edge_map_run_step_status=X_LESS_LOOP_CLOSE_EDGE_MAP;
                    }
                }
            }
            else{
                if(current_pose->y>half_map_wide){
                    close_edge_map_run_step_status=Y_MORE_LOOP_CLOSE_EDGE_MAP;
                }
                else{
                    close_edge_map_run_step_status=Y_LESS_LOOP_CLOSE_EDGE_MAP;
                }
            }
        }
        i=(current_pose->x+half_map_wide)/GRIDWIDTH;
        j=(current_pose->y+half_map_wide)/GRIDWIDTH;
        if (my_abs(last_position_x - current_pose->x) > lateral_move_distance||my_abs(last_position_y - current_pose->y) > lateral_move_distance){
            end_x=false;
            if(i>3&&i<96&&j>3&&j<96){
                for(k=i-4;k<=i+4;k++){
                    for(ij=j-4;ij<=j+4;ij++){
                        if(gridmap.map[k][ij]!=125){
                            end_x=true;
                            break;
                        }
                    }
                    if(end_x==true){
                        break;
                    }
					k++;
                }			
                if(end_x==true){
                    last_position_x = current_pose->x;
                    last_position_y = current_pose->y;
                    close_edge_map_run_step_status=MORE_LOOP_CLOSE_EDGE_MAP;
                }
                else{
                    close_edge_map_run_step_status= COMPLETE_LOOP_CLOSE_EDGE_MAP;
                }
                break;
            }
            else{
                last_position_x = current_pose->x;
                last_position_y = current_pose->y;
                close_edge_map_run_step_status=MORE_LOOP_CLOSE_EDGE_MAP;
            }
            break;
        }
        if(DelimmaNumber==1){
            if(Yaw<0){
            }
            else{
                linear_velocity = 0;
                angular_velocity = turn_vel;
            }
            if(my_abs(i-close_edge_min_x)<5&&my_abs(j-close_edge_min_x)<5){
                motionSteps++;
                DelimmaNumber=2;
                step=0;
            }
            if(my_abs(i-close_l_edge_max_x)<5&&my_abs(j-close_l_edge_max_y)<5){
                motionSteps++;
                DelimmaNumber=3;
                step=0;
            }
        }
        else if(DelimmaNumber==2){
            if(my_abs(Yaw)<90){
            }
            else{
                linear_velocity = 0;
                angular_velocity = turn_vel;
            }
            if(my_abs(i-close_l_edge_max_x)<5&&my_abs(j-close_l_edge_max_y)<5){
                motionSteps++;
                DelimmaNumber=3;
                step=0;
            }
            if(my_abs(i-close_edge_max_x)<5&&my_abs(j-close_edge_max_y)<5){
                motionSteps++;
                DelimmaNumber=4;
                step=0;
            }
        }
        else if(DelimmaNumber==3){
            if(Yaw>0){
            }
            else{
                linear_velocity = 0;
                angular_velocity = turn_vel;
            }
            if(my_abs(i-close_edge_max_x)<5&&my_abs(j-close_edge_max_y)<5){
                motionSteps++;
                DelimmaNumber=4;
                step=0;
            }
            if(detection_close==true){
                if(my_abs(i-close_r_edge_min_x)<10&&my_abs(j-close_r_edge_min_y)<10){
                    motionSteps++;
                    DelimmaNumber=1;
                    step=0;
                    close_edge_map_run_step_status= COMPLETE_CLOSE_EDGE_MAP;
                    break;
                }
            }
        }
        else if(DelimmaNumber==4){
            if(my_abs(Yaw)>90){
            }
            else{
                linear_velocity = 0;
                angular_velocity = turn_vel;
            }
            if(detection_close==true){
                if(my_abs(i-close_r_edge_min_x)<5&&my_abs(j-close_r_edge_min_y)<5){
                    motionSteps++;
                    DelimmaNumber=1;
                }
                if(my_abs(i-close_edge_min_x)<5&&my_abs(j-close_edge_min_x)<5){
                    motionSteps++;
                    DelimmaNumber=2;
                }
                if(DelimmaNumber!=4){
                    step=0;
                    motionSteps++;
                    close_edge_map_run_step_status= COMPLETE_CLOSE_EDGE_MAP;
                    break;
                }
            }
        }
        else{
            linear_velocity = long_stra_vel;
            angular_velocity = 0;
        }
        if(motionSteps>4){
            motionSteps++;
            close_edge_map_run_step_status= COMPLETE_CLOSE_EDGE_MAP;
        }
        break;
    case X_MORE_LOOP_CLOSE_EDGE_MAP:
        if (my_abs(Yaw) > 175)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            close_edge_map_run_step_status =GO_X_MORE_LOOP_CLOSE_EDGE_MAP;
            break;
        }
        if (obstacleSignal != none_obstacle||(&cliff_valueB)->cliffValue0 == 1)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            close_edge_map_run_step_status = COLLISION_X_MORE_LOOP_CLOSE_EDGE_MAP;
            break;
        }
        linear_velocity = 0;
        angular_velocity = turn_vel;
        break;
    case COLLISION_X_MORE_LOOP_CLOSE_EDGE_MAP:
        if (turn_start_update == 0)
        {
            turn_start_x = current_pose->x;
            turn_start_y = current_pose->y;
            turn_start_update = 1;
        }
        linear_velocity = -long_stra_vel;
        angular_velocity = 0;
        if (my_abs(turn_start_x - current_pose->x) > collision_backward_distance || my_abs(turn_start_y - current_pose->y) > collision_backward_distance)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            turn_start_update = 0;
            close_edge_map_run_step_status=X_MORE_LOOP_CLOSE_EDGE_MAP;
        }
        break;
    case GO_X_MORE_LOOP_CLOSE_EDGE_MAP:
        linear_velocity = long_stra_vel;
        angular_velocity = 0;
        if(current_pose->x<half_map_wide-500){
            last_position_x = current_pose->x;
            last_position_y = current_pose->y;
            close_edge_map_run_step_status = LOOP_CLOSE_EDGE_MAP;
            break;
        }
        if(obstacleSignal != none_obstacle||(&cliff_valueB)->cliffValue0 == 1){
            linear_velocity = 0;
            angular_velocity = 0;
            close_edge_map_run_step_status=COLLISION_GO_X_MORE_LOOP_CLOSE_EDGE_MAP;
            break;
        }
        break;
    case COLLISION_GO_X_MORE_LOOP_CLOSE_EDGE_MAP:
        if (turn_start_update == 0)
        {
            turn_start_x = current_pose->x;
            turn_start_y = current_pose->y;
            turn_start_update = 1;
        }
        linear_velocity = -long_stra_vel;
        angular_velocity = 0;
        if (my_abs(turn_start_x - current_pose->x) > collision_backward_distance || my_abs(turn_start_y - current_pose->y) > collision_backward_distance)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            turn_start_update = 0;
            temporary_yaw=Yaw;
            close_edge_map_run_step_status=TURN_COLLISION_GO_X_MORE_LOOP_CLOSE_EDGE_MAP;
            break;
        }
    case TURN_COLLISION_GO_X_MORE_LOOP_CLOSE_EDGE_MAP:
        if (Yaw<90&&Yaw>0){
            step=6;
        }
        if(step<3){
            linear_velocity = 0;
            angular_velocity = -turn_vel;
        }
        else{
            if(Yaw>-90&&Yaw<0){
                close_edge_map_run_step_status= COMPLETE_CLOSE_EDGE_MAP;
                break;
            }
            linear_velocity = 0;
            angular_velocity = turn_vel;
        }
        if (my_abs(temporary_yaw - Yaw) > 30){
            linear_velocity = 0;
            angular_velocity = 0;
            close_edge_map_run_step_status=GO_X_MORE_LOOP_CLOSE_EDGE_MAP;
        }
        if(obstacleSignal != none_obstacle||(&cliff_valueB)->cliffValue0 == 1){
            linear_velocity = 0;
            angular_velocity = 0;
            close_edge_map_run_step_status=COLLISION_TURN_COLLISION_GO_X_MORE_LOOP_CLOSE_EDGE_MAP;
            break;
        }
    case COLLISION_TURN_COLLISION_GO_X_MORE_LOOP_CLOSE_EDGE_MAP:
        if (turn_start_update == 0)
        {
            turn_start_x = current_pose->x;
            turn_start_y = current_pose->y;
            turn_start_update = 1;
        }
        linear_velocity = -long_stra_vel;
        angular_velocity = 0;
        if (my_abs(turn_start_x - current_pose->x) > collision_backward_distance || my_abs(turn_start_y - current_pose->y) > collision_backward_distance)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            turn_start_update = 0;
            close_edge_map_run_step_status=TURN_COLLISION_GO_X_MORE_LOOP_CLOSE_EDGE_MAP;
            break;
        }
        
    case X_LESS_LOOP_CLOSE_EDGE_MAP:
        if (my_abs(Yaw) < 5){
            linear_velocity = 0;
            angular_velocity = 0;
            close_edge_map_run_step_status =GO_X_LESS_LOOP_CLOSE_EDGE_MAP;
            break;
        }
        if (obstacleSignal != none_obstacle||(&cliff_valueB)->cliffValue0 == 1)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            close_edge_map_run_step_status = COLLISION_X_LESS_LOOP_CLOSE_EDGE_MAP;
            break;
        }
        linear_velocity = 0;
        angular_velocity = turn_vel;
        break;
    case COLLISION_X_LESS_LOOP_CLOSE_EDGE_MAP:
        if (turn_start_update == 0){
            turn_start_x = current_pose->x;
            turn_start_y = current_pose->y;
            turn_start_update = 1;
        }
        linear_velocity = -long_stra_vel;
        angular_velocity = 0;
        if (my_abs(turn_start_x - current_pose->x) > collision_backward_distance || my_abs(turn_start_y - current_pose->y) > collision_backward_distance)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            turn_start_update = 0;
            close_edge_map_run_step_status=X_LESS_LOOP_CLOSE_EDGE_MAP;
        }
        break;
        
    case GO_X_LESS_LOOP_CLOSE_EDGE_MAP:
        linear_velocity = long_stra_vel;
        angular_velocity = 0;
        if(current_pose->x>-half_map_wide+500){
            last_position_x = current_pose->x;
            last_position_y = current_pose->y;
            close_edge_map_run_step_status = LOOP_CLOSE_EDGE_MAP;
            break;
        }
        if(obstacleSignal != none_obstacle||(&cliff_valueB)->cliffValue0 == 1){
            linear_velocity = 0;
            angular_velocity = 0;
            close_edge_map_run_step_status=COLLISION_GO_X_LESS_LOOP_CLOSE_EDGE_MAP;
            break;
        }
        break;
    case COLLISION_GO_X_LESS_LOOP_CLOSE_EDGE_MAP:
        if (turn_start_update == 0)
        {
            turn_start_x = current_pose->x;
            turn_start_y = current_pose->y;
            turn_start_update = 1;
        }
        linear_velocity = -long_stra_vel;
        angular_velocity = 0;
        if (my_abs(turn_start_x - current_pose->x) > collision_backward_distance || my_abs(turn_start_y - current_pose->y) > collision_backward_distance)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            turn_start_update = 0;
            temporary_yaw=Yaw;
            close_edge_map_run_step_status=TURN_COLLISION_GO_X_LESS_LOOP_CLOSE_EDGE_MAP;
            break;
        }
    case TURN_COLLISION_GO_X_LESS_LOOP_CLOSE_EDGE_MAP:
        if (Yaw<-90){
            step=6;
        }
        if(step<3){
            linear_velocity = 0;
            angular_velocity = -turn_vel;
        }
        else{
            if(Yaw>90){
                close_edge_map_run_step_status= COMPLETE_CLOSE_EDGE_MAP;
                break;
            }
            linear_velocity = 0;
            angular_velocity = turn_vel;
        }
        if (my_abs(temporary_yaw - Yaw) > 30){
            linear_velocity = 0;
            angular_velocity = 0;
            close_edge_map_run_step_status=GO_X_LESS_LOOP_CLOSE_EDGE_MAP;
        }
        if(obstacleSignal != none_obstacle||(&cliff_valueB)->cliffValue0 == 1){
            linear_velocity = 0;
            angular_velocity = 0;
            close_edge_map_run_step_status=COLLISION_TURN_COLLISION_GO_X_LESS_LOOP_CLOSE_EDGE_MAP;
            break;
        }
    case COLLISION_TURN_COLLISION_GO_X_LESS_LOOP_CLOSE_EDGE_MAP:
        if (turn_start_update == 0)
        {
            turn_start_x = current_pose->x;
            turn_start_y = current_pose->y;
            turn_start_update = 1;
        }
        linear_velocity = -long_stra_vel;
        angular_velocity = 0;
        if (my_abs(turn_start_x - current_pose->x) > collision_backward_distance || my_abs(turn_start_y - current_pose->y) > collision_backward_distance)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            turn_start_update = 0;
            close_edge_map_run_step_status=TURN_COLLISION_GO_X_LESS_LOOP_CLOSE_EDGE_MAP;
            break;
        }
        break;
    case Y_MORE_LOOP_CLOSE_EDGE_MAP:
        if (Yaw>-95&&Yaw<-85)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            close_edge_map_run_step_status =GO_Y_MORE_LOOP_CLOSE_EDGE_MAP;
            break;
        }
        if (obstacleSignal != none_obstacle||(&cliff_valueB)->cliffValue0 == 1)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            close_edge_map_run_step_status = COLLISION_Y_MORE_LOOP_CLOSE_EDGE_MAP;
            break;
        }
        linear_velocity = 0;
        angular_velocity = turn_vel;
        break;
    case COLLISION_Y_MORE_LOOP_CLOSE_EDGE_MAP:
        if (turn_start_update == 0)
        {
            turn_start_x = current_pose->x;
            turn_start_y = current_pose->y;
            turn_start_update = 1;
        }
        linear_velocity = -long_stra_vel;
        angular_velocity = 0;
        if (my_abs(turn_start_x - current_pose->x) > collision_backward_distance || my_abs(turn_start_y - current_pose->y) > collision_backward_distance)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            turn_start_update = 0;
            close_edge_map_run_step_status=Y_MORE_LOOP_CLOSE_EDGE_MAP;
        }
        break;
        
    case GO_Y_MORE_LOOP_CLOSE_EDGE_MAP:
        linear_velocity = long_stra_vel;
        angular_velocity = 0;
        if(current_pose->y<half_map_wide-500){
            last_position_x = current_pose->x;
            last_position_y = current_pose->y;
            close_edge_map_run_step_status = LOOP_CLOSE_EDGE_MAP;
            break;
        }
        if(obstacleSignal != none_obstacle||(&cliff_valueB)->cliffValue0 == 1){
            linear_velocity = 0;
            angular_velocity = 0;
            close_edge_map_run_step_status=COLLISION_GO_Y_MORE_LOOP_CLOSE_EDGE_MAP;
            break;
        }
        break;
    case COLLISION_GO_Y_MORE_LOOP_CLOSE_EDGE_MAP:
        if (turn_start_update == 0)
        {
            turn_start_x = current_pose->x;
            turn_start_y = current_pose->y;
            turn_start_update = 1;
        }
        linear_velocity = -long_stra_vel;
        angular_velocity = 0;
        if (my_abs(turn_start_x - current_pose->x) > collision_backward_distance || my_abs(turn_start_y - current_pose->y) > collision_backward_distance)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            turn_start_update = 0;
            temporary_yaw=Yaw;
            close_edge_map_run_step_status=TURN_COLLISION_GO_Y_MORE_LOOP_CLOSE_EDGE_MAP;
            break;
        }
    case TURN_COLLISION_GO_Y_MORE_LOOP_CLOSE_EDGE_MAP:
        if (Yaw>90){
            step=6;
        }
        if(step<3){
            linear_velocity = 0;
            angular_velocity = -turn_vel;
        }
        else{
            if(Yaw<90&&Yaw>0){
                close_edge_map_run_step_status= COMPLETE_CLOSE_EDGE_MAP;
                break;
            }
            linear_velocity = 0;
            angular_velocity = turn_vel;
        }
        if (my_abs(temporary_yaw - Yaw) > 30){
            linear_velocity = 0;
            angular_velocity = 0;
            close_edge_map_run_step_status=GO_Y_MORE_LOOP_CLOSE_EDGE_MAP;
        }
        if(obstacleSignal != none_obstacle||(&cliff_valueB)->cliffValue0 == 1){
            linear_velocity = 0;
            angular_velocity = 0;
            close_edge_map_run_step_status=COLLISION_TURN_COLLISION_GO_Y_MORE_LOOP_CLOSE_EDGE_MAP;
            break;
        }
    case COLLISION_TURN_COLLISION_GO_Y_MORE_LOOP_CLOSE_EDGE_MAP:
        if (turn_start_update == 0)
        {
            turn_start_x = current_pose->x;
            turn_start_y = current_pose->y;
            turn_start_update = 1;
        }
        linear_velocity = -long_stra_vel;
        angular_velocity = 0;
        if (my_abs(turn_start_x - current_pose->x) > collision_backward_distance || my_abs(turn_start_y - current_pose->y) > collision_backward_distance)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            turn_start_update = 0;
            close_edge_map_run_step_status=TURN_COLLISION_GO_Y_MORE_LOOP_CLOSE_EDGE_MAP;
            break;
        }
        
    case Y_LESS_LOOP_CLOSE_EDGE_MAP:
        if (Yaw<95&&Yaw>85)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            close_edge_map_run_step_status =GO_Y_LESS_LOOP_CLOSE_EDGE_MAP;
            break;
        }
        if (obstacleSignal != none_obstacle||(&cliff_valueB)->cliffValue0 == 1)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            close_edge_map_run_step_status = COLLISION_Y_LESS_LOOP_CLOSE_EDGE_MAP;
            break;
        }
        linear_velocity = 0;
        angular_velocity = turn_vel;
        break;
    case COLLISION_Y_LESS_LOOP_CLOSE_EDGE_MAP:
        if (turn_start_update == 0)
        {
            turn_start_x = current_pose->x;
            turn_start_y = current_pose->y;
            turn_start_update = 1;
        }
        linear_velocity = -long_stra_vel;
        angular_velocity = 0;
        if (my_abs(turn_start_x - current_pose->x) > collision_backward_distance || my_abs(turn_start_y - current_pose->y) > collision_backward_distance)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            turn_start_update = 0;
            close_edge_map_run_step_status=Y_LESS_LOOP_CLOSE_EDGE_MAP;
        }
        break;
    case GO_Y_LESS_LOOP_CLOSE_EDGE_MAP:
        linear_velocity = long_stra_vel;
        angular_velocity = 0;
        if(current_pose->y>-half_map_wide+500){
            last_position_x = current_pose->x;
            last_position_y = current_pose->y;
            close_edge_map_run_step_status = LOOP_CLOSE_EDGE_MAP;
            break;
        }
        if(obstacleSignal != none_obstacle||(&cliff_valueB)->cliffValue0 == 1){
            linear_velocity = 0;
            angular_velocity = 0;
            close_edge_map_run_step_status=COLLISION_GO_Y_LESS_LOOP_CLOSE_EDGE_MAP;
            break;
        }
        break;
    case COLLISION_GO_Y_LESS_LOOP_CLOSE_EDGE_MAP:
        if (turn_start_update == 0)
        {
            turn_start_x = current_pose->x;
            turn_start_y = current_pose->y;
            turn_start_update = 1;
        }
        linear_velocity = -long_stra_vel;
        angular_velocity = 0;
        if (my_abs(turn_start_x - current_pose->x) > collision_backward_distance || my_abs(turn_start_y - current_pose->y) > collision_backward_distance)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            turn_start_update = 0;
            temporary_yaw=Yaw;
            close_edge_map_run_step_status=TURN_COLLISION_GO_Y_LESS_LOOP_CLOSE_EDGE_MAP;
            break;
        }
    case TURN_COLLISION_GO_Y_LESS_LOOP_CLOSE_EDGE_MAP:
        if (Yaw<0){
            step=6;
        }
        if(step<3){
            linear_velocity = 0;
            angular_velocity = -turn_vel;
        }
        else{
            if(Yaw<-90){
                close_edge_map_run_step_status= COMPLETE_CLOSE_EDGE_MAP;
                break;
            }
            linear_velocity = 0;
            angular_velocity = turn_vel;
        }
        if (my_abs(temporary_yaw - Yaw) > 30){
            linear_velocity = 0;
            angular_velocity = 0;
            close_edge_map_run_step_status=GO_Y_LESS_LOOP_CLOSE_EDGE_MAP;
        }
        if(obstacleSignal != none_obstacle||(&cliff_valueB)->cliffValue0 == 1){
            linear_velocity = 0;
            angular_velocity = 0;
            close_edge_map_run_step_status=COLLISION_TURN_COLLISION_GO_Y_LESS_LOOP_CLOSE_EDGE_MAP;
            break;
        }
    case COLLISION_TURN_COLLISION_GO_Y_LESS_LOOP_CLOSE_EDGE_MAP:
        if (turn_start_update == 0)
        {
            turn_start_x = current_pose->x;
            turn_start_y = current_pose->y;
            turn_start_update = 1;
        }
        linear_velocity = -long_stra_vel;
        angular_velocity = 0;
        if (my_abs(turn_start_x - current_pose->x) > collision_backward_distance || my_abs(turn_start_y - current_pose->y) > collision_backward_distance)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            turn_start_update = 0;
            close_edge_map_run_step_status=TURN_COLLISION_GO_Y_LESS_LOOP_CLOSE_EDGE_MAP;
            break;
        }
    case COMPLETE_LOOP_CLOSE_EDGE_MAP:
        if(current_pose->y<0){
            complete_flag=2;
        }
        else{
            complete_flag=3;
        }
        linear_velocity = 0;
        angular_velocity = 0;
        step=0;
        motionSteps=0;
        mapstopupdate=false;
        DelimmaNumber=0;
        close_edge_map_run_step_status=0;
        closeedgesmap=true;
        break;
    case COMPLETE_CLOSE_EDGE_MAP:
        complete_flag=1;
        linear_velocity = 0;
        angular_velocity = 0;
        step=0;
        motionSteps=0;
        mapstopupdate=false;
        DelimmaNumber=0;
        close_edge_map_run_step_status=0;
        detection_close=false;
        break;
    }
    return complete_flag;
}
unsigned char  DetectionCloseEdge(){
    int8_t i,j,k;
    bool end_x = false;
    if (selectside == 'R')
    {
        for (i = 0; i < GRIDHEIGHT-2; i++)
        {
            for (j = 0; j < GRIDHEIGHT/2; j++)
            {
                if (gridmap.map[i][j] == 0)
                {
                    close_edge_min_x = i;
                    close_edge_min_y = j;
                    end_x = true;
                    break;
                }
            }
            if (end_x == true)
            {
                end_x = false;
                break;
            }
			i++;
        }
        for (i = GRIDHEIGHT - 1; i > 1; i--)
        {
            for (j = 0; j < GRIDHEIGHT / 2; j++)
            {
                if (gridmap.map[i][j] == 0)
                {
                    close_edge_max_x = i;
                    close_edge_max_y = j;
                    end_x = true;
                    break;
                }
            }
            if (end_x == true)
            {
                end_x = false;
                break;
            }
			i--;
        }
        close_l_edge_max_x=GRIDHEIGHT;
        while (close_edge_max_x - close_edge_min_x > 3)
        {
            for(k=close_edge_max_y-3;k<=close_edge_max_y;k++){
                for (i = close_edge_max_x - 3; i <close_edge_max_x; i++)
                {               
                    for (j = 0; j < GRIDHEIGHT/2+10; j++)
                    {              
                        if (gridmap.map[i][j] == 0)
                        {
                            if (my_abs(close_edge_max_y - j) <=3)
                            {
                                close_edge_max_x = i;
                                close_edge_max_y = j;
                                end_x = true;
                                break;
                            }
                        }                                     
                    }
                    if (end_x == true)
                    {
                        break;
                    }
                }
            }
            if(end_x == false){
                if(close_edge_max_x-close_l_edge_max_x<0){
                    i=close_edge_max_x;
                    for (j = 0; j < close_edge_max_y;j++)
                    {				
                        if (gridmap.map[i][j] == 0)
                        {
                            if (my_abs(close_edge_max_y - j) <=3)
                            {
                                for(k=j;k>0;k-- ){
									k--;
                                    if(gridmap.map[i][k] != 0){
                                        close_edge_max_x = i;
                                        close_l_edge_max_x = i;
                                        close_edge_max_y = k+1;
                                        end_x = true;
                                        break;                                        
                                    }
                                }                                
                            }
                        }
                        if (end_x == true){
                            break;
                        }
                       j++;                        
                    }
                    if(end_x == false){
                        for (j =close_edge_max_y+1; j <GRIDHEIGHT/2+10 ; j++)
                        {
                            if (gridmap.map[i][j] == 0)
                            {
                                if (my_abs(close_edge_max_y - j) <=3)
                                {
                                    for(k=j;k<GRIDHEIGHT/2+10;k++){
										k++;
                                        if(gridmap.map[i][k] != 0){                                           
                                            close_edge_max_x = i;
                                            close_l_edge_max_x = i;
                                            close_edge_max_y = k-1;
                                            end_x = true;
                                            break;
                                        }
                                    }
                                }
                            }                            
                            if (end_x == true){
                                break;
                            }
							j++;							
                        }                    
                    }
                }                
            }
            if (end_x == true)
            {
                end_x = false;
            }
            else
            {
                break;
            }
        }
    }
    else
    {
        for (i = 0; i < GRIDHEIGHT; i++)
        {
            for (j = GRIDHEIGHT - 1; j > 1; j--)
            {
                if (gridmap.map[i][j] == 0)
                {
                    close_edge_min_x = i;
                    close_edge_min_y = j;
                    end_x = true;
                    break;
                }
            }
            if (end_x == true)
            {
                end_x = false;
                break;
            }
			i++;
        }
        for (i = GRIDHEIGHT - 1; i > 0; i--)
        {
            for (j = GRIDHEIGHT - 1; j > 1; j--)
            {
                if (gridmap.map[i][j] == 0)
                {
                    close_edge_max_x = i;
                    close_edge_max_y = j;
                    end_x = true;
                    break;
                }
            }
            if (end_x == true)
            {
                end_x = false;
                break;
            }
			i--;
        }
        close_l_edge_max_x=GRIDHEIGHT;
        while (close_edge_max_x - close_edge_min_x > 3)
        {
            for(k=close_edge_max_y;k<=close_edge_max_y+3;k++){
                for (i = close_edge_max_x - 3; i < close_edge_max_x; i++)
                {
                    for (j = GRIDHEIGHT - 1; j >GRIDHEIGHT/2-40; j--)
                    {           
                        if (gridmap.map[i][j] == 0)
                        {
                            if (my_abs(close_edge_max_y - j) <=3)
                            {							
                                close_edge_max_x = i;
                                close_edge_max_y = j;
                                end_x = true;
                                break;
                            }
                        } 
                    }
                    if (end_x == true)
                    {
                        break;
                    }
                }
            }
            if(end_x == false){
                if(close_edge_max_x-close_l_edge_max_x<0){
                    i=close_edge_max_x;
                    for (j = GRIDHEIGHT - 1; j > close_edge_max_y; j--)
                    {
						j--;						
                        if (gridmap.map[i][j] == 0)
                        {
                            if (my_abs(close_edge_max_y - j) <=3)
                            {
                                for(k=j;k<GRIDHEIGHT;k++){
									k++;
                                    if(gridmap.map[i][k] != 0){                                           
                                        close_edge_max_x = i;
                                        close_l_edge_max_x = i;
                                        close_edge_max_y = k-1;
                                        end_x = true;
                                        break;
                                    }
                                }
                            }
                        }
                        if (end_x == true)
                        {
                            break;
                        }
                    }
                    if(end_x == false){
                        for (j =close_edge_max_y-1; j >GRIDHEIGHT/2-40 ; j--)
                        {							
                            if (gridmap.map[i][j] == 0)
                            {
                                if (my_abs(close_edge_max_y - j) <=3)
                                {                                    
                                    for(k=j;k>GRIDHEIGHT/2-20;k--){
										k--;
                                        if(gridmap.map[i][k] != 0){                                           
                                            close_edge_max_x = i;
                                            close_l_edge_max_x = i;
                                            close_edge_max_y = k+1;
                                            end_x = true;
                                            break;
                                        }
                                    }
                                }
                            }
                            if (end_x == true)
                            {
                                break;
                            }
							j--;                            
                        }                    
                    }
                }                
            }           
            if (end_x == true)
            {
                end_x = false;
            }
            else
            {
                break;
            }
        }
    }
    if (close_edge_max_x - close_edge_min_x > 3)
    {
        detection_close = true;
    }
    else
    {
        detection_close = false;
    }
    return 1;
}



void MoreMap(POSE *current_pose){
    unsigned char i,j;
    if(motionSteps==1){
        x_more_map=true;
        if(current_pose->x>0){
            if(b_reverse_moremap==false){
                reverse_moremap=1;
            }
            for(i=0;i<MAPWIDECELLS/2;i++){
                for(j=0;j<MAPWIDECELLS;j++){
                    gridmap.map[i][j]=gridmap.map[i+MAPWIDECELLS/2][j];
                    gridmap.map[i+MAPWIDECELLS/2][j]=125;
                }
            }
            x_more_positive_start=1;
        }else{
            if(b_reverse_moremap==false){
                reverse_moremap=-1;
            }
            for(i=MAPWIDECELLS/2;i<MAPWIDECELLS;i++){
                for(j=0;j<MAPWIDECELLS;j++){
                    gridmap.map[i][j]=gridmap.map[i-MAPWIDECELLS/2][j];
                    gridmap.map[i-MAPWIDECELLS/2][j]=125;
                }
            }
            x_more_positive_start=-1;
        }
    }
    if(motionSteps==3){
        y_more_map=true;
        if(current_pose->y>0){
            for(i=0;i<MAPWIDECELLS;i++){
                for(j=0;j<MAPWIDECELLS/2;j++){
                    gridmap.map[i][j]=gridmap.map[i][j+MAPWIDECELLS/2];
                    gridmap.map[i][j+MAPWIDECELLS/2]=125;
                }
            }
            y_more_positive_start=1;
        }else{
            for(i=0;i<MAPWIDECELLS;i++){
                for(j=MAPWIDECELLS/2;j<MAPWIDECELLS;j++){
                    gridmap.map[i][j]=gridmap.map[i][j-MAPWIDECELLS/2];
                    gridmap.map[i][j-MAPWIDECELLS/2]=125;
                }
            }
            y_more_positive_start=-1;
        }
    }
}







void LessMap(void){
    unsigned char i,j;
    for(i=0;i<MAPWIDECELLS;i++){
        for(j=MAPWIDECELLS/2;j<MAPWIDECELLS;j++){
            gridmap.map[i][j-MAPWIDECELLS/2]=gridmap.map[i][j];
            gridmap.map[i][j]=125;
        }
    }
}



void StartUpdateGridMap(void){
    unsigned char i,j;
    for(i=0;i<MAPWIDECELLS;i++){
        for(j=0;j<MAPWIDECELLS;j++){
            gridmap.map[i][j]=125;
        }
    }
}




int32_t bsp_GetStrategyCurrentPosX(void){
    return 	map_current_pose_x;
}



int32_t bsp_GetStrategyCurrentPosY(void){
    return 	map_current_pose_y;
}





void ReturnExtreme_point_init(void){
    signed char  i;
    Under_extreme_point_x_index = 0;
    Under_extreme_point_y_index=0;
    On_extreme_point_x_index = 0;
    On_extreme_point_y_index = 0;
    Left_Under_extreme_point_x_index = 0;
    Left_Under_extreme_point_y_index=0;
    Left_On_extreme_point_x_index = 0;
    Left_On_extreme_point_y_index = 0;
    for(i=0;i<10;i++){
        Under_extreme_point_x[i] = 0;
        Under_extreme_point_y[i] = 0;
        On_extreme_point_x[i]=GRIDWIDTH;
        On_extreme_point_y[i]=GRIDWIDTH;
        Left_Under_extreme_point_x[i] = 0;
        Left_Under_extreme_point_y[i] = 0;
        Left_On_extreme_point_x[i]=GRIDWIDTH;
        Left_On_extreme_point_y[i]=GRIDWIDTH;
    }
}















