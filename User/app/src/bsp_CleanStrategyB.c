#include "bsp.h"
#include <math.h>


#define INT_COOR_X 250
#define INT_COOR_Y 250
#define ALL_CLEAN_COMPLETE 0
#define LEFT_CLEAN_WORK_TIME 10*60*1000
#define RIGHT_CLEAN_WORK_TIME 10*60*1000

#define ZoomMultiple 4
#define compression_map_x 25
#define compression_map_y 25

int wheel_pulse_l;
int wheel_pulse_r;
int temporary_wheel_pulse_r;
int temporary_wheel_pulse_l;

int OVERALL_CLEANING_STRATEGY = 0;

int right_running_step_status = 0;
int collision_right_rightrun_step_status = 0;
int collision_left_rightrun_step_status = 0;
int collision_front_rightrun_step_status = 0;

int left_running_step_status = 0;
int collision_right_leftrun_step_status = 0;
int collision_left_leftrun_step_status = 0;
int collision_front_leftrun_step_status = 0;

short stuck_right_run_step=0;

int return_origin_step_status = 0;
short a_star_motion_return_origin_status=0;
short a_star_not_motion_status=0;
short a_star_collision_status=0;
short cliff_running_step_status=0;
bool  over_clean_finish=false;

int8_t return_origin_positive_start= 1;
int8_t selectside=0;


//for  walk edge
int right_walk_edge_status = 0;
int right_reverse_walk_edge_status = 0;
int right_edge_dilemma_status = 0;
int right_forward_boundary_status = 0;
int right_ready_leaking_sweep_status = 0;

int left_walk_edge_status = 0;
int left_reverse_walk_edge_status = 0;
int left_edge_dilemma_status = 0;
int left_forward_boundary_status = 0;
int left_ready_leaking_sweep_status = 0;


int number = 0;
int leakingsweep = 0;
int leakingsweep_x = 0;
int leakingsweep_y = 0;
short leakingsweep_X_interval = 200;
short leakingsweep_Y_interval = 100;



double linear_velocity = 0,angular_velocity = 0;
int bypass_velocity = 150;
	
//for  collision step
unsigned char distance_uptate  = 0;
unsigned char turn_start_update = 0;
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

unsigned char DelimmaNumber=0;
unsigned char CliffDelimmaNumber = 0;
unsigned char CliffFunctionStatus=0;
unsigned char CliffNumber=0;
unsigned char totalCDN=0;

bool clill_start_update=false;
bool cliffruningStatus=false;


int FunctionStatus=0;
unsigned int LastCleanTimeStamp = 0;
unsigned int CurrentCleanTimeStamp  = 0;

//**************return origin value************************************
AStar_MapNode AStar_graph[AStar_Height][AStar_Width];
AStar_Close astar_close[AStar_Height][AStar_Width];
AStar_Close *AStar_start;
unsigned char AStar_srcX, AStar_srcY, AStar_dstX, AStar_dstY;
bool maze[compression_map_x][compression_map_y];
bool printclose;
unsigned char step;
char plansteps[100];
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



const AStarPoint astar_dir[8] =
{
	{0, 1},
	{1, 1},
	{1, 0},
	{1, -1},
	{0, -1},
	{-1, -1},
	{-1, 0},
	{-1, 1}
};

//**************************************************


//***********stuck status******************************
int judgment_Stuck_status = 0;
int judgment_Stuck_status_x = 0;
int judgment_Stuck_status_y = 0;
int judgment_Stuck_status_yaw = 0;
bool stuck_x = false;
bool stuck_y = false;
int x_error = 0;
int y_error = 0;
//


static CleanStrategyB cleanstrategy;
static POSE current_pose;
//static int Yaw;
static short speed_pid_cnt = 0;
static unsigned char* IRSensorData_StrategyB;




static double my_abs(double x){
    if (x<0){
        x= -x;
    }
    return x;}
	
static void log_debug(char* str)
{
    //########################
    //DEBUG("%s\n",str);
    //########################
}
static void sendvelocity(double* linear_velocity,double* angular_velocity)
{
    short leftVelocity,rightVelocity;
	
	if(IRSensorData_StrategyB[1] == 1 || IRSensorData_StrategyB[3] == 1 || IRSensorData_StrategyB[5] == 1 || IRSensorData_StrategyB[7] == 1)
	{
		if(*linear_velocity == long_stra_vel)
		{
			*linear_velocity = 0.7**linear_velocity;	
		}
	}

	if(*linear_velocity == -long_stra_vel)
	{
		if(*angular_velocity == 0)
		{
			*linear_velocity = -150;
		}
	}

	if(*linear_velocity == long_stra_vel)
	{
		if(speed_pid_cnt <=100) speed_pid_cnt +=1;
	
		*linear_velocity = speed_pid_cnt*0.01**linear_velocity;	
	}
	else
	{
		speed_pid_cnt = 30;
	}
	
	
    leftVelocity = (short)((0.5*(2**linear_velocity*0.001 - Deg2Rad(*angular_velocity)*WHEEL_LENGTH))* 1000);
    rightVelocity = (short)((0.5*(2**linear_velocity*0.001 + Deg2Rad(*angular_velocity)*WHEEL_LENGTH))* 1000);
	
    bsp_SetMotorSpeed(MotorLeft,bsp_MotorSpeedMM2Pulse(leftVelocity));
    bsp_SetMotorSpeed(MotorRight,bsp_MotorSpeedMM2Pulse(rightVelocity));

}


void bsp_StartUpdateCleanStrategyB(void)
{
	cleanstrategy.work_step_status = RIGHTRUNNING_WORK_SETP;
	cleanstrategy.right_running_complete  = 0;
	cleanstrategy.right_return_origin_complete = 0;
	cleanstrategy.left_running_complete = 0;
	cleanstrategy.left_return_origin_complete = 0;
	
	cleanstrategy.action = 0 ;
	cleanstrategy.delay = 0 ;
	cleanstrategy.isRunning = true;
	linear_velocity = 0,angular_velocity = 0;
	
	bsp_ResetCleanStrategyBStatus();

}


void bsp_ResetCleanStrategyBStatus(void)
{
	LastCleanTimeStamp = xTaskGetTickCount();
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
	cliff_running_step_status=0;
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
	bypass_velocity = 150;
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
	CliffDelimmaNumber = 0;
	CliffFunctionStatus=0;
	CliffNumber=0;
	totalCDN=0;
	clill_start_update=false;
	cliffruningStatus=false;
	FunctionStatus=0;
	LastCleanTimeStamp = 0;
	CurrentCleanTimeStamp  = 0;
	
	bsp_ResetPosArgument();
	bsp_StartUpdateGridMap();
	
}

void bsp_StopUpdateCleanStrategyB(void)
{
	cleanstrategy.action = 0 ;
	cleanstrategy.delay = 0 ;
	cleanstrategy.isRunning = false;
	
	cleanstrategy.work_step_status = 0;
	cleanstrategy.right_running_complete  = 0;
	cleanstrategy.right_return_origin_complete = 0;
	cleanstrategy.left_running_complete = 0;
	cleanstrategy.left_return_origin_complete = 0;

	bsp_SetMotorSpeed(MotorLeft,bsp_MotorSpeedMM2Pulse(0));
	bsp_SetMotorSpeed(MotorRight,bsp_MotorSpeedMM2Pulse(0));
	
	bsp_ResetCleanStrategyBStatus();
	
}



void bsp_CleanStrategyUpdateB(int robotX,int robotY,double robotTheta, unsigned char obstacleSignal, int current_wheel_pulse_l, int current_wheel_pulse_r, unsigned char IRSensorData[],CLIFFADCVALUE * cliff_value)
{
	IRSensorData_StrategyB = IRSensorData;
	current_pose.x = INT_COOR_X + robotX;
	current_pose.y = INT_COOR_Y + robotY;
	current_pose.x =  robotX;
	current_pose.y =  robotY;
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
			bsp_SperkerPlay(Song24);
			bsp_StopUpdateCleanStrategyB();
		}
	}
	
}	
//#################################################################################
uint8_t clean_strategy(POSE *current_pose,unsigned char obstacleSignal)
{
	CurrentCleanTimeStamp = xTaskGetTickCount();
	
    switch (cleanstrategy.work_step_status)
    {
        case RIGHTRUNNING_WORK_SETP:
			if(RightRunningWorkStep(current_pose,obstacleSignal) || ((CurrentCleanTimeStamp - LastCleanTimeStamp) > RIGHT_CLEAN_WORK_TIME))
			{
				cleanstrategy.work_step_status  = RIGHTRETURN_ORIGIN_WORK_SETP;
				bsp_ResetCleanStrategyBStatus();
			}
			else
			{
			  break;
			}
            break;	 
        case RIGHTRETURN_ORIGIN_WORK_SETP:
			if(ForceReturnOrigin(current_pose,obstacleSignal) || ((CurrentCleanTimeStamp - LastCleanTimeStamp) > RIGHT_CLEAN_WORK_TIME))
			{
				cleanstrategy.work_step_status  = LEFTRUNNING_WORK_SETP;
				bsp_ResetCleanStrategyBStatus();
			}
			else
			{
				break;
			}
            break;
        case LEFTRUNNING_WORK_SETP:
			if(LeftRunningWorkStep(current_pose,obstacleSignal) || ((CurrentCleanTimeStamp - LastCleanTimeStamp) > LEFT_CLEAN_WORK_TIME))
			{
				cleanstrategy.work_step_status  = LEFTRETURN_ORIGIN_WORK_SETP;
				bsp_ResetCleanStrategyBStatus();
			}
			else
			{
			  break;
			}
            break;
        case LEFTRETURN_ORIGIN_WORK_SETP:
			if(ForceReturnOrigin(current_pose,obstacleSignal) || ((CurrentCleanTimeStamp - LastCleanTimeStamp) > LEFT_CLEAN_WORK_TIME))
			{
				cleanstrategy.work_step_status  = ALL_FINSHED_WORK_SETP;
				bsp_ResetCleanStrategyBStatus();
			}
			else
			{
				break;
			}
            break;
        case ALL_FINSHED_WORK_SETP:
			bsp_SperkerPlay(Song24);
			cleanstrategy.work_step_status  = 0;
            return ALL_CLEAN_COMPLETE;//" clean complete"
        default:
            break;
    }
	return 0;
}



//#################################################################################
uint8_t clean_strategyB(POSE *current_pose,unsigned char obstacleSignal)
{
	
	if (detection_close_edge == true)
	{
		DetectionCloseEdge();
	}
	if ((&cliff_valueB)->cliffValue0 == 1)
	{
		CliffNumber++;
		if (CliffNumber > 5)
		{
			cliffruningStatus = true;
		}
	}else
	{
		CliffNumber = 0;
	}
	if (cliffruningStatus == true)
	{
		CliffFunctionStatus = CliffRuningWorkStep(current_pose,(&cliff_valueB), obstacleSignal);
		if (CliffFunctionStatus == 2)
		{
			DetectionCloseEdge();
			if (detection_close == false)
			{
				OVERALL_CLEANING_STRATEGY = A_STAR_RETURN_ORIGIN_WORKING_OVERALL_CLEANING_STRATEGY;
				cliff_running_step_status = 0;
			}
			else
			{
				detection_close = false;
				cliffruningStatus = false;
				cliff_running_step_status = 0;
			}
		}
		if (CliffFunctionStatus == 1)
		{
			cliffruningStatus = false;
			cliff_running_step_status = 0;
		}
	}
	if (totalCDN > 4)
	{
		totalCDN = CliffCloseEdge(current_pose);
		if (totalCDN == 1)
		{
			return 0;
		}
		if (totalCDN == 2)
		{
			cliffruningStatus = false;
			OVERALL_CLEANING_STRATEGY = A_STAR_RETURN_ORIGIN_WORKING_OVERALL_CLEANING_STRATEGY;
		}
		totalCDN = 0;
	}
	if (!cliffruningStatus)
	{
		switch (OVERALL_CLEANING_STRATEGY)
		{
		case 0:
			OVERALL_CLEANING_STRATEGY = START_OVERALL_CLEANING_STRATEGY;
			break;
		case START_OVERALL_CLEANING_STRATEGY:
			OVERALL_CLEANING_STRATEGY = RIGHT_RUNNING_WORKING_OVERALL_CLEANING_STRATEGY;
			break;
		case RIGHT_RUNNING_WORKING_OVERALL_CLEANING_STRATEGY:
			FunctionStatus = RightRunningWorkStep(current_pose, obstacleSignal);
			if (1 == FunctionStatus)
			{
				selectside = 'L';
				over_clean_finish = false;
				OVERALL_CLEANING_STRATEGY = A_STAR_RETURN_ORIGIN_WORKING_OVERALL_CLEANING_STRATEGY;
				right_running_step_status = 0;
				FunctionStatus = 0;
				break;
			}
			break;
		case A_STAR_RETURN_ORIGIN_WORKING_OVERALL_CLEANING_STRATEGY:
			AStarReturnOrigin(current_pose, obstacleSignal);
			OVERALL_CLEANING_STRATEGY = A_STAR_MOTION_RETURN_ORIGIN_WORKING_OVERALL_CLEANING_STRATEGY;
			break;
		case A_STAR_MOTION_RETURN_ORIGIN_WORKING_OVERALL_CLEANING_STRATEGY:
			FunctionStatus = AStarMotionReturnOrigin(current_pose, obstacleSignal);
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
				OVERALL_CLEANING_STRATEGY = A_STAR_COLLISION_RETURN_ORIGIN_WORKING_OVERALL_CLEANING_STRATEGY;
				FunctionStatus = 0;
				break;
			}
			if (3 == FunctionStatus)
			{
				a_star_motion_return_origin_status = 0;
				OVERALL_CLEANING_STRATEGY = A_STAR_NOT_RETURN_ORIGIN_WORKING_OVERALL_CLEANING_STRATEGY;
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
			log_debug("A_STAR_NOT_RETURN_ORIGIN_WORKING_OVERALL_CLEANING_STRATEGY");
			FunctionStatus = AStarNotMotionReturnOrigin(current_pose, obstacleSignal);
			if (1 == FunctionStatus)
			{
				OVERALL_CLEANING_STRATEGY = A_STAR_RETURN_ORIGIN_WORKING_OVERALL_CLEANING_STRATEGY;
				FunctionStatus = 0;
				break;
			}
			break;
		case A_STAR_COLLISION_RETURN_ORIGIN_WORKING_OVERALL_CLEANING_STRATEGY:
			log_debug("A_STAR_COLLISION_RETURN_ORIGIN_WORKING_OVERALL_CLEANING_STRATEGY");
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
			log_debug("RETURN_ORIGIN_WORKING_OVERALL_CLEANING_STRATEGY");
			FunctionStatus = ForceReturnOrigin(current_pose, obstacleSignal);
			if (1 == FunctionStatus)
			{
				if (over_clean_finish == true)
				{
					OVERALL_CLEANING_STRATEGY = 0;
					return_origin_step_status = 0;
					FunctionStatus = 0;
					return 0;
				}
				else
				{
					OVERALL_CLEANING_STRATEGY = LEFT_RUNNING_WORKING_OVERALL_CLEANING_STRATEGY;
					selectside = 'L';
					return_origin_step_status = 0;
					FunctionStatus = 0;
				}
				break;
			}
			break;
		case LEFT_RUNNING_WORKING_OVERALL_CLEANING_STRATEGY:
			log_debug("LEFT_RUNNING_WORKING_OVERALL_CLEANING_STRATEGY");
			FunctionStatus = LeftRunningWorkStep(current_pose, obstacleSignal);
			if (1 == FunctionStatus)
			{
				over_clean_finish = true;
				OVERALL_CLEANING_STRATEGY = A_STAR_RETURN_ORIGIN_WORKING_OVERALL_CLEANING_STRATEGY;
				left_running_step_status = 0;
				FunctionStatus = 0;
				break;
			}
			break;
		}
	}
	
	
	sendvelocity(&linear_velocity, &angular_velocity);
	
	return 1;
	

}





//#################################################################################
//####################################################           RIGHT        #####    
//#################################################################################
////////////////////////////////////////////////////////////////////////////////////////////////////////

unsigned char  RightRunningWorkStep(POSE *current_pose, unsigned char obstacleSignal)
{
    int Yaw;
    unsigned char complete_flag = 0;
    Yaw = current_pose->orientation;
    switch (right_running_step_status)
    {
    case 0:
        right_running_step_status = GOSTR_RIGHTRUN_STEP;
        break;
    case GOSTR_RIGHTRUN_STEP:
        log_debug("gostraight right run step!");
        if (my_abs(Yaw / 100) >= 90 && my_abs(Yaw / 100) < 175)
        {
            log_debug("backaward Corrected heading angle !");
            if (Yaw > 0)
            {
                linear_velocity = 0;
                angular_velocity = 5 * turn_vel / 6;
                break;
            }
            else
            {
                linear_velocity = 0;
                angular_velocity = -5 * turn_vel / 6;
                break;
            }
        }
        else if (my_abs(Yaw / 100) < 90 && my_abs(Yaw / 100) > 5)
        {
            log_debug("gostraight Corrected heading angle !");
            if (Yaw > 0)
            {
                linear_velocity = 0;
                angular_velocity = -5 * turn_vel / 6;
                break;
            }
            else
            {
                linear_velocity = 0;
                angular_velocity = 5 * turn_vel / 6;
                break;
            }
        }
        else if (my_abs(current_pose->x) > half_map_wide)
        {
            log_debug("current pose x arrived width max!");
            right_running_step_status = FORWARD_BOUNDARY_RIGHTRUN_STEP;
            break;
        }
        else if (my_abs(current_pose->y) > half_map_wide)
        {
            log_debug("current pose y arrived width max!");
            right_running_step_status = FORWARD_BOUNDARY_RIGHTRUN_STEP;
            break;
        }
        else if (right_obstacle == obstacleSignal)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            if (my_abs(leakingsweep_x - current_pose->x) > 500 && my_abs(leakingsweep_y - current_pose->y) > 100)
            {
                leakingsweep_x = current_pose->x;
                leakingsweep_y = current_pose->y;
                leakingsweep = bsp_Right_ReturnExtreme_point(current_pose->x, current_pose->y, current_pose->orientation / 100, obstacleSignal);
            }
            if (0 != leakingsweep)
            {
                log_debug("right obstacle,ready goto LEAKING_SWEEP_RIGHTRUN_STEP");
                right_running_step_status = LEAKING_SWEEP_RIGHTRUN_STEP;
                break;
            }
            else
            {
                log_debug("right obstacle,ready goto COLLISION_RIGHT_RIGHTRUN_STEP");
                collision_right_rightrun_step_status = 0;
                right_running_step_status = COLLISION_RIGHT_RIGHTRUN_STEP;
                break;
            }
        }
        else if (front_obstacle == obstacleSignal)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            if (my_abs(leakingsweep_x - current_pose->x) > 500 && my_abs(leakingsweep_y - current_pose->y) > 100)
            {
                leakingsweep_x = current_pose->x;
                leakingsweep_y = current_pose->y;
                leakingsweep = bsp_Right_ReturnExtreme_point(current_pose->x, current_pose->y, current_pose->orientation / 100, obstacleSignal);
            }
            if (0 != leakingsweep)
            {
                log_debug("front obstacle,ready goto LEAKING_SWEEP_RIGHTRUN_STEP");
                right_running_step_status = LEAKING_SWEEP_RIGHTRUN_STEP;
                break;
            }
            else
            {
                log_debug("front obstacle,ready goto COLLISION_FRONT_RIGHTRUN_STEP");
                collision_front_rightrun_step_status = 0;
                right_running_step_status = COLLISION_FRONT_RIGHTRUN_STEP;
                break;
            }
        }
        else if (left_obstacle == obstacleSignal)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            if (my_abs(leakingsweep_x - current_pose->x) > 500 && my_abs(leakingsweep_y - current_pose->y) > 100)
            {
                leakingsweep_x = current_pose->x;
                leakingsweep_y = current_pose->y;
                leakingsweep = bsp_Right_ReturnExtreme_point(current_pose->x, current_pose->y, current_pose->orientation / 100, obstacleSignal);
            }
            if (0 != leakingsweep)
            {
                log_debug("left obstacle,ready goto LEAKING_SWEEP_RIGHTRUN_STEP");
                right_running_step_status = LEAKING_SWEEP_RIGHTRUN_STEP;
                break;
            }
            else
            {
                log_debug("left obstacle,ready goto COLLISION_LEFT_RIGHTRUN_STEP");
                collision_left_rightrun_step_status = 0;
                right_running_step_status = COLLISION_LEFT_RIGHTRUN_STEP;
                break;
            }
        }
        else
        {
            log_debug("go straight...");
            linear_velocity = long_stra_vel;
            angular_velocity = 0;
        }
        break;
    case FORWARD_BOUNDARY_RIGHTRUN_STEP:
        log_debug("forward boundary rightrun step!");
        FunctionStatus = ForwardBoundaryRightRunStep(current_pose, obstacleSignal);
        if (FunctionStatus == 1)
        {
            right_running_step_status = GOSTR_RIGHTRUN_STEP;
            right_forward_boundary_status = 0;
        }
        if (FunctionStatus == 2)
        {
            right_running_step_status = STUCK_FORWARD_BOUNDARY_RIGHT_RUNSTEP;
            right_forward_boundary_status = 0;
        }
        break;
    case STUCK_FORWARD_BOUNDARY_RIGHT_RUNSTEP:
        log_debug("STUCK_FORWARD_BOUNDARY_RIGHT_RUNSTEP!");
        FunctionStatus = StuckRightRunStep(current_pose, obstacleSignal);
        if (FunctionStatus == 1)
        {
            right_running_step_status = GOSTR_RIGHTRUN_STEP;
            stuck_right_run_step = 0;
        }
        break;
    case COLLISION_RIGHT_RIGHTRUN_STEP:
        log_debug("COLLISION right right run step!");
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
        log_debug("RIGHTWALKEDGE");
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
            right_running_step_status = GOSTR_RIGHTRUN_STEP;
            right_walk_edge_status = 0;
            FunctionStatus = 0;
            complete_flag = 1;
            break;
        }
        break;
    case RIGHTEDGEDILEMMA:
        log_debug("RIGHTEDGEDILEMMA!");
        if (RightEdgeDilemma(current_pose, obstacleSignal))
        {
            right_running_step_status = GOSTR_RIGHTRUN_STEP;
            right_edge_dilemma_status = 0;
        }
        break;
    case COLLISION_LEFT_RIGHTRUN_STEP:
        log_debug("COLLISION left right run step!");
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
        log_debug("RIGHTREVERSEWALKEDGE");
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
            right_running_step_status = GOSTR_RIGHTRUN_STEP;
            right_reverse_walk_edge_status = 0;
            FunctionStatus = 0;
            complete_flag = 1;
            break;
        }
        break;
    case COLLISION_FRONT_RIGHTRUN_STEP:
        log_debug("COLLISION front right run step!");
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
        log_debug("leaking sweep right run step!");
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

unsigned char  CollisionRightRightRunStep(POSE *current_pose, unsigned char obstacleSignal)
{
    ////cout << "Collision_Right_temporary_current_pose.x................============>>>>>>>>>>>" << endl;
    int Yaw;
    unsigned char complete_flag = 0;
    Yaw = current_pose->orientation;
    switch (collision_right_rightrun_step_status)
    {
    case 0:
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
            collision_right_rightrun_step_status = GOBACK_DISTANCE_CRRRS;
            turn_start_update = 0;
            break;
        }
        break;
    case GOBACK_DISTANCE_CRRRS:
        if (turn_start_update == 0)
        {
            turn_start_x = current_pose->x;
            turn_start_y = current_pose->y;
            turn_start_update = 1;
        }
        linear_velocity = -long_stra_vel;
        angular_velocity = 0;

        if (obstacleSignal == front_obstacle || obstacleSignal == left_obstacle || obstacleSignal == right_obstacle)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            if (my_abs(Yaw / 100) < 90)
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
        if (my_abs(turn_start_x - current_pose->x) > side_backward_distance)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            if (my_abs(Yaw / 100) < 90)
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
        collision_right_rightrun_step_status = TURN_CLOCK_TARGET_YAW_NEG27_CR_DRYL;
        break;
    case TURN_CLOCK_TARGET_YAW_NEG27_CR_DRYL:
        linear_velocity = 0;
        angular_velocity = -turn_vel;
        if ((Yaw / 100) < -27)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            collision_right_rightrun_step_status = GOSTR_YAW_EQUAL_NEG27_CR_DRYL;
            break;
        }
        if (obstacleSignal == front_obstacle || obstacleSignal == left_obstacle || obstacleSignal == right_obstacle)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            collision_right_rightrun_step_status = TURN_CLOCK_TARGET_YAW_NEG27_COLLISION_CR_DRYL;
            break;
        }
        break;
    case TURN_CLOCK_TARGET_YAW_NEG27_COLLISION_CR_DRYL:
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
            collision_right_rightrun_step_status = TURN_CLOCK_TARGET_YAW_NEG27_CR_DRYL;
            break;
        }
        break;
    case GOSTR_YAW_EQUAL_NEG27_CR_DRYL:
        linear_velocity = long_stra_vel;
        angular_velocity = 0;
        if (obstacleSignal == front_obstacle || obstacleSignal == left_obstacle || obstacleSignal == right_obstacle)
        {
            collision_right_rightrun_step_status = TURN_CLOCK_TARGET_YAW_NEG57_CR_DRYL;
            break;
        }
        if (my_abs(last_position_y - current_pose->y) > lateral_move_distance / 3)
        {
            collision_right_rightrun_step_status = TURN_CLOCK_TARGET_YAW_NEG57_CR_DRYL;
            break;
        }
        break;
    case TURN_CLOCK_TARGET_YAW_NEG57_CR_DRYL:
        linear_velocity = 0;
        angular_velocity = -turn_vel;
        if (Yaw / 100 < -57)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            collision_right_rightrun_step_status = GOSTR_YAW_EQUAL_NEG57_CR_DRYL;
            break;
        }
        if (obstacleSignal == front_obstacle || obstacleSignal == left_obstacle || obstacleSignal == right_obstacle)
        {
            collision_right_rightrun_step_status = TURN_CLOCK_TARGET_YAW_NEG57_COLLISION_CR_DRYL;
            break;
        }
        break;
    case TURN_CLOCK_TARGET_YAW_NEG57_COLLISION_CR_DRYL:
        if (turn_start_update == 0)
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
        linear_velocity = long_stra_vel;
        angular_velocity = 0;
        if (obstacleSignal == front_obstacle || obstacleSignal == left_obstacle || obstacleSignal == right_obstacle)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            collision_right_rightrun_step_status = TURN_CLOCK_TARGET_YAW_NEG82_CR_DRYL;
            break;
        }
        if (my_abs(last_position_y - current_pose->y) > (2 * lateral_move_distance) / 3)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            collision_right_rightrun_step_status = TURN_CLOCK_TARGET_YAW_NEG82_CR_DRYL;
            break;
        }
        break;
    case TURN_CLOCK_TARGET_YAW_NEG82_CR_DRYL:
        linear_velocity = 0;
        angular_velocity = -turn_vel;
        if (Yaw / 100 < -82)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            collision_right_rightrun_step_status = GOSTR_YAW_EQUAL_NEG82_CR_DRYL;
            break;
        }
        if (obstacleSignal == front_obstacle || obstacleSignal == left_obstacle || obstacleSignal == right_obstacle)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            collision_right_rightrun_step_status = TURN_CLOCK_TARGET_YAW_NEG82_COLLISION_CR_DRYL;
            break;
        }
        break;
    case TURN_CLOCK_TARGET_YAW_NEG82_COLLISION_CR_DRYL:
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
            collision_right_rightrun_step_status = TURN_CLOCK_TARGET_YAW_NEG82_CR_DRYL;
            break;
        }
        break;
    case GOSTR_YAW_EQUAL_NEG82_CR_DRYL:
        linear_velocity = long_stra_vel;
        angular_velocity = 0;
        if (obstacleSignal == front_obstacle || obstacleSignal == left_obstacle || obstacleSignal == right_obstacle)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            collision_right_rightrun_step_status = RIGHT_WALK_EDGE_CR_DRYL;
            break;
        }
        if (my_abs(last_position_y - current_pose->y) > lateral_move_distance)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            collision_right_rightrun_step_status = TURN_CLOCK_TARGET_YAW_ABS173_CR_DRYL;
            break;
        }
        break;
    case RIGHT_WALK_EDGE_CR_DRYL:
        collision_right_rightrun_step_status = 0;
        complete_flag = 2;
        break;
    case TURN_CLOCK_TARGET_YAW_ABS173_CR_DRYL:
        linear_velocity = 0;
        angular_velocity = -turn_vel;
        if (my_abs(Yaw / 100) > 173)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            collision_right_rightrun_step_status = COMPLETE_CR_DRYL;
            break;
        }
        if (obstacleSignal == front_obstacle || obstacleSignal == left_obstacle || obstacleSignal == right_obstacle)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            collision_right_rightrun_step_status = TURN_CLOCK_TARGET_YAW_ABS173_COLLISION_CR_DRYL;
            break;
        }
        break;
    case TURN_CLOCK_TARGET_YAW_ABS173_COLLISION_CR_DRYL:
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
            collision_right_rightrun_step_status = TURN_CLOCK_TARGET_YAW_ABS173_CR_DRYL;
            break;
        }
        break;
    case COMPLETE_CR_DRYL:
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
        if (my_abs(Yaw / 100) < 153)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            collision_right_rightrun_step_status = GOSTR_YAW_EQUAL_ABS153_CR_DRYM;
            break;
        }
        if (obstacleSignal == front_obstacle || obstacleSignal == left_obstacle || obstacleSignal == right_obstacle)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            collision_right_rightrun_step_status = TURN_CCLCOK_TARGET_YAW_ABS153_COLLISION_CR_DRYM;
            break;
        }
        break;
    case TURN_CCLCOK_TARGET_YAW_ABS153_COLLISION_CR_DRYM:
        if (turn_start_update == 0)
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
        cnt_update += 1;
        linear_velocity = long_stra_vel;
        angular_velocity = 0;
        if (cnt_update > 4 && my_abs(last_position_xx - current_pose->x) < 20 && my_abs(last_position_y - current_pose->y) < 20)
        {
            cnt_update = 0;
            collision_right_rightrun_step_status = MORE_TRY_BREAK_BYPASS_CR_DRYM;
            break;
        }
        collision_right_rightrun_step_status = GOSTR_BYPASS_LOOP_CR_DRYM;
        break;
    case GOSTR_BYPASS_LOOP_CR_DRYM:
        if (obstacleSignal == right_obstacle)
        {
            collision_right_rightrun_step_status = RIGHT_COLLISION_BYPASS_CR_DRYM;
            break;
        }
        if (my_abs(last_position_x - current_pose->x) > lateral_move_distance / 3)
        {
            collision_right_rightrun_step_status = GOSTR_X_MORE_ONE_THIRD_LATERALDIS_BYPASS_CR_DRYM;
            break;
        }
        if (my_abs(last_position_y - current_pose->y) > close_edge || obstacleSignal == front_obstacle || obstacleSignal == left_obstacle)
        {
            collision_right_rightrun_step_status = GOSTR_BYPASS_BOW_CONTINUE_CR_DRYM;
            break;
        }
        if (last_position_y < current_pose->y - 1)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            old_bow_continue = true;
            collision_right_rightrun_step_status = GOSTR_BYPASS_BOW_CONTINUE_EXIT_CR_DRYM;
            break;
        }
        if (my_abs(current_pose->x) > half_map_wide)
        {
            collision_right_rightrun_step_status = COMPLETE_CR_DRYM;
            break;
        }
        if (my_abs(Yaw / 100) < 90)
        {
            collision_right_rightrun_step_status = COMPLETE_CR_DRYM;
            break;
        }
        break;

    case GOSTR_BYPASS_BOW_CONTINUE_CR_DRYM:
        if (obstacleSignal == front_obstacle || obstacleSignal == right_obstacle || obstacleSignal == left_obstacle)
        {
            collision_right_rightrun_step_status = GOSTR_BYPASS_BOW_CONTINUE_COLLISION_CR_DRYM;
            break;
        }
        if (my_abs(Yaw / 100) < 3)
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

    case GOSTR_BYPASS_BOW_CONTINUE_COLLISION_CR_DRYM:
        if (turn_start_update == 0)
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
            temporary_yaw = Yaw / 100;
            if (my_abs(Yaw / 100) > 165)
                collision_right_rightrun_step_status = TURN_CCLOCK_TARGET_YAW_MORE_ABS165_CR_DRYM;
            else
                collision_right_rightrun_step_status = TURN_CCLOCK_TARGET_YAW_LESS_ABS165_CR_DRYM;
            break;
        }
        break;
    case TURN_CCLOCK_TARGET_YAW_MORE_ABS165_CR_DRYM:
        if (obstacleSignal == front_obstacle || obstacleSignal == right_obstacle || obstacleSignal == left_obstacle)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            collision_right_rightrun_step_status = TURN_CCLOCK_TARGET_YAW_MORE_ABS165_COLLISION_CR_DRYM;
            break;
        }
        if (my_abs(Yaw / 100) < 165)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            collision_right_rightrun_step_status = GOSTR_BYPASS_CR_DRYM;
            break;
        }
        linear_velocity = 0;
        angular_velocity = turn_vel;
        break;
    case TURN_CCLOCK_TARGET_YAW_MORE_ABS165_COLLISION_CR_DRYM:
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
            collision_right_rightrun_step_status = TURN_CCLOCK_TARGET_YAW_MORE_ABS165_CR_DRYM;
            turn_start_update = 0;
            break;
        }
        break;
    case TURN_CCLOCK_TARGET_YAW_LESS_ABS165_CR_DRYM:
        if (obstacleSignal == front_obstacle || obstacleSignal == right_obstacle || obstacleSignal == left_obstacle)
        {
            collision_right_rightrun_step_status = TURN_CCLOCK_TARGET_YAW_LESS_ABS165_COLLISION_CR_DRYM;
            break;
        }
        if (my_abs(Yaw / 100 - temporary_yaw) > 15)
        {
            collision_right_rightrun_step_status = GOSTR_BYPASS_CR_DRYM;
            break;
        }
        linear_velocity = 0;
        angular_velocity = turn_vel;
        break;
    case TURN_CCLOCK_TARGET_YAW_LESS_ABS165_COLLISION_CR_DRYM:
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
            collision_right_rightrun_step_status = TURN_CCLOCK_TARGET_YAW_LESS_ABS165_CR_DRYM;
            turn_start_update = 0;
            break;
        }
        break;
    case GOSTR_X_MORE_ONE_THIRD_LATERALDIS_BYPASS_CR_DRYM:
        if (my_abs(Yaw / 100) > 150)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            temporary_yaw = Yaw / 100;
            collision_right_rightrun_step_status = TURN_CLOCK_TARGET_YAW_ABS150_CR_DRYM;
        }
        else if (my_abs(Yaw / 100) < 150 && Yaw / 100 < 0)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            temporary_yaw = Yaw / 100;
            collision_right_rightrun_step_status = TURN_CLOCK_YAW_ADD_ABS30_CR_DRYM;
        }
        else
        {
            linear_velocity = 0;
            angular_velocity = 0;
            collision_right_rightrun_step_status = GOSTR_BYPASS_CR_DRYM;
        }
        break;
    case TURN_CLOCK_TARGET_YAW_ABS150_CR_DRYM:
        if (my_abs(Yaw / 100) < 150)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            collision_right_rightrun_step_status = GOSTR_BYPASS_CR_DRYM;
            break;
        }
        if (obstacleSignal == front_obstacle || obstacleSignal == left_obstacle || obstacleSignal == right_obstacle)
        {
            collision_right_rightrun_step_status = TURN_CLOCK_TARGET_YAW_ABS150_COLLISION_CR_DRYM;
        }
        linear_velocity = 0;
        angular_velocity = -turn_vel;
        break;
    case TURN_CLOCK_TARGET_YAW_ABS150_COLLISION_CR_DRYM:
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
            collision_right_rightrun_step_status = TURN_CLOCK_TARGET_YAW_ABS150_CR_DRYM;
            turn_start_update = 0;
            break;
        }
        break;
    case TURN_CLOCK_YAW_ADD_ABS30_CR_DRYM:
        if (my_abs(Yaw / 100 - temporary_yaw) > 30)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            collision_right_rightrun_step_status = GOSTR_BYPASS_CR_DRYM;
            break;
        }
        if (obstacleSignal == front_obstacle || obstacleSignal == left_obstacle || obstacleSignal == right_obstacle)
        {
            collision_right_rightrun_step_status = TURN_CLOCK_YAW_ADD_ABS30_COLLISION_CR_DRYM;
        }
        linear_velocity = 0;
        angular_velocity = -turn_vel;
        break;
    case TURN_CLOCK_YAW_ADD_ABS30_COLLISION_CR_DRYM:
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
            collision_right_rightrun_step_status = TURN_CLOCK_YAW_ADD_ABS30_CR_DRYM;
            turn_start_update = 0;
            break;
        }
        break;
    case MORE_TRY_BREAK_BYPASS_CR_DRYM:
        if (obstacleSignal == front_obstacle || obstacleSignal == right_obstacle || obstacleSignal == left_obstacle)
        {
            collision_right_rightrun_step_status = GOSTR_BYPASS_BOW_CONTINUE_COLLISION_CR_DRYM;
            bow_continue = true;
            break;
        }
        if (my_abs(Yaw / 100) < 3)
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
        collision_right_rightrun_step_status = 0;
        complete_flag = 1;
        break;
    }
    return complete_flag;
}

unsigned char  CollisionLeftRightRunStep(POSE *current_pose, unsigned char obstacleSignal)
{
    int Yaw;
    unsigned char complete_flag = 0;
    Yaw = current_pose->orientation;
    ////cout << "collision_left_rightrun_step_status................============>>>>>>>>>>>" << collision_left_rightrun_step_status << endl;
    switch (collision_left_rightrun_step_status)
    {
    case 0:
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
            collision_left_rightrun_step_status = GOBACK_DISTANCE_CLRRS;
            turn_start_update = 0;
        }
        break;
    case GOBACK_DISTANCE_CLRRS:
        if (turn_start_update == 0)
        {
            turn_start_x = current_pose->x;
            turn_start_y = current_pose->y;
            turn_start_update = 1;
        }
        linear_velocity = -long_stra_vel;
        angular_velocity = 0;

        if (obstacleSignal == front_obstacle || obstacleSignal == left_obstacle || obstacleSignal == right_obstacle)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            if (my_abs(Yaw / 100) < 90)
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
        if (my_abs(turn_start_x - current_pose->x) > side_backward_distance)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            if (my_abs(Yaw / 100) < 90)
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
    case DIR_RIGHT_YAW_LESS_ABS90_CLRRS:
        linear_velocity = 0;
        angular_velocity = 0;
        collision_left_rightrun_step_status = TURN_CLOCK_TARGET_YAW_ABS30_CL_DRYL;
        break;
    case TURN_CLOCK_TARGET_YAW_ABS30_CL_DRYL:
        linear_velocity = 0;
        angular_velocity = -turn_vel;
        if (my_abs(Yaw / 100) > 30)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            collision_left_rightrun_step_status = GOSTR_YAW_MORE_ABS30_CL_DRYL;
            break;
        }
        if (obstacleSignal == front_obstacle || obstacleSignal == left_obstacle || obstacleSignal == right_obstacle)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            collision_left_rightrun_step_status = TURN_CLOCK_TARGET_YAW_ABS30_COLLISION_CL_DRYL;
            break;
        }
        break;
    case TURN_CLOCK_TARGET_YAW_ABS30_COLLISION_CL_DRYL:
        if (turn_start_update == 0)
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
    case GOSTR_YAW_MORE_ABS30_CL_DRYL:
        cnt_update = 0;
        last_position_y = current_pose->y;
        last_position_xx = current_pose->x;
        collision_left_rightrun_step_status = GOSTR_BYPASS_CL_DRYL;
        break;
    case GOSTR_BYPASS_CL_DRYL:
        last_position_x = current_pose->x;
        cnt_update += 1;
        linear_velocity = long_stra_vel;
        angular_velocity = 0;
        if (cnt_update > 4 && my_abs(last_position_xx - current_pose->x) < 20 && my_abs(last_position_y - current_pose->y) < 20)
        {
            cnt_update = 0;
            collision_left_rightrun_step_status = MORE_TRY_BREAK_BYPASS_CL_DRYL;
            break;
        }
        collision_left_rightrun_step_status = GOSTR_BYPASS_LOOP_CL_DRYL;
        break;
    case GOSTR_BYPASS_LOOP_CL_DRYL:
        if (obstacleSignal == left_obstacle)
        {
            collision_left_rightrun_step_status = LEFT_COLLISION_BYPASS_CL_DRYL;
            break;
        }
        if (my_abs(last_position_x - current_pose->x) > lateral_move_distance / 3)
        {
            collision_left_rightrun_step_status = GOSTR_X_MORE_ONE_THIRD_LATERALDIS_BYPASS_CL_DRYL;
            break;
        }
        if (my_abs(last_position_y - current_pose->y) > close_edge || obstacleSignal == front_obstacle || obstacleSignal == right_obstacle)
        {
            collision_left_rightrun_step_status = GOSTR_BYPASS_BOW_CONTINUE_CL_DRYL;
            break;
        }
        if (last_position_y < current_pose->y - 1)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            old_bow_continue = true;
            collision_left_rightrun_step_status = GOSTR_BYPASS_BOW_CONTINUE_EXIT_CL_DRYL;
            break;
        }
        if (my_abs(current_pose->x) > half_map_wide)
        {
            collision_left_rightrun_step_status = COMPLETE_CL_DRYL;
            break;
        }
        if (my_abs(Yaw / 100) > 90)
        {
            collision_left_rightrun_step_status = COMPLETE_CL_DRYL;
            break;
        }
        linear_velocity = long_stra_vel;
        angular_velocity = 0;
        break;

    case GOSTR_BYPASS_BOW_CONTINUE_CL_DRYL:
        if (obstacleSignal == front_obstacle || obstacleSignal == right_obstacle || obstacleSignal == left_obstacle)
        {
            collision_left_rightrun_step_status = GOSTR_BYPASS_BOW_CONTINUE_COLLISION_CL_DRYL;
            break;
        }
        if (my_abs(Yaw / 100) > 175)
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
    case GOSTR_BYPASS_BOW_CONTINUE_COLLISION_CL_DRYL:
        if (turn_start_update == 0)
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
            collision_left_rightrun_step_status = GOSTR_BYPASS_BOW_CONTINUE_CL_DRYL;
            turn_start_update = 0;
            break;
        }
        break;
    case GOSTR_BYPASS_BOW_CONTINUE_EXIT_CL_DRYL:
        collision_left_rightrun_step_status = COMPLETE_CL_DRYL;
        break;
    case LEFT_COLLISION_BYPASS_CL_DRYL:
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
            if (my_abs(Yaw / 100) < 15)
            {
                collision_left_rightrun_step_status = TURN_CLCOK_TARGET_YAW_LESS_ABS15_CL_DRYL;
            }
            else
            {
                temporary_yaw = Yaw / 100;
                collision_left_rightrun_step_status = TURN_CLCOK_TARGET_YAW_MORE_ABS15_CL_DRYL;
            }
            break;
        }
        break;
    case TURN_CLCOK_TARGET_YAW_LESS_ABS15_CL_DRYL:
        if (obstacleSignal == front_obstacle || obstacleSignal == right_obstacle || obstacleSignal == left_obstacle)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            collision_left_rightrun_step_status = TURN_CLCOK_TARGET_YAW_LESS_ABS15_COLLISION_CL_DRYL;
            break;
        }
        if (my_abs(Yaw / 100) > 15)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            collision_left_rightrun_step_status = GOSTR_BYPASS_CL_DRYL;
            break;
        }
        linear_velocity = 0;
        angular_velocity = -turn_vel;
        break;
    case TURN_CLCOK_TARGET_YAW_LESS_ABS15_COLLISION_CL_DRYL:
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
            collision_left_rightrun_step_status = TURN_CLCOK_TARGET_YAW_LESS_ABS15_CL_DRYL;
            turn_start_update = 0;
            break;
        }
        break;

    case TURN_CLCOK_TARGET_YAW_MORE_ABS15_CL_DRYL:
        if (obstacleSignal == front_obstacle || obstacleSignal == right_obstacle || obstacleSignal == left_obstacle)
        {
            collision_left_rightrun_step_status = TURN_CLCOK_TARGET_YAW_MORE_ABS15_COLLISION_CL_DRYL;
            break;
        }
        if (my_abs(Yaw / 100 - temporary_yaw) > 15)
        {
            collision_left_rightrun_step_status = GOSTR_BYPASS_CL_DRYL;
            break;
        }
        linear_velocity = 0;
        angular_velocity = -turn_vel;
        break;
    case TURN_CLCOK_TARGET_YAW_MORE_ABS15_COLLISION_CL_DRYL:
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
            collision_left_rightrun_step_status = TURN_CLCOK_TARGET_YAW_MORE_ABS15_CL_DRYL;
            turn_start_update = 0;
            break;
        }
        break;
    case GOSTR_X_MORE_ONE_THIRD_LATERALDIS_BYPASS_CL_DRYL:
        if (my_abs(Yaw / 100) < 30)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            temporary_yaw = Yaw / 100;
            collision_left_rightrun_step_status = TURN_CCLOCK_TARGET_YAW_MORE_AB30_CL_DRYL;
        }
        else if (my_abs(Yaw / 100) > 30 && (Yaw / 100 < 0))
        {
            linear_velocity = 0;
            angular_velocity = 0;
            temporary_yaw = Yaw / 100;
            collision_left_rightrun_step_status = TURN_CCLOCK_YAW_ADD_ABS30_CL_DRYL;
        }
        else
        {
            linear_velocity = 0;
            angular_velocity = 0;
            collision_left_rightrun_step_status = GOSTR_BYPASS_CL_DRYL;
            break;
        }
        break;
    case TURN_CCLOCK_TARGET_YAW_MORE_AB30_CL_DRYL:
        if (my_abs(Yaw / 100) > 30)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            collision_left_rightrun_step_status = GOSTR_BYPASS_CL_DRYL;
            break;
        }
        if (obstacleSignal == front_obstacle || obstacleSignal == left_obstacle || obstacleSignal == right_obstacle)
        {
            collision_left_rightrun_step_status = TURN_CCLOCK_TARGET_YAW_MORE_AB30_COLLISION_CL_DRYL;
        }
        linear_velocity = 0;
        angular_velocity = turn_vel;
        break;
    case TURN_CCLOCK_TARGET_YAW_MORE_AB30_COLLISION_CL_DRYL:
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
            collision_left_rightrun_step_status = TURN_CCLOCK_TARGET_YAW_MORE_AB30_CL_DRYL;
            turn_start_update = 0;
            break;
        }
        break;
    case TURN_CCLOCK_YAW_ADD_ABS30_CL_DRYL:
        if (my_abs(Yaw / 100 - temporary_yaw) > 30)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            collision_left_rightrun_step_status = GOSTR_BYPASS_CL_DRYL;
            break;
        }

        if (obstacleSignal == front_obstacle || obstacleSignal == left_obstacle || obstacleSignal == right_obstacle)
        {
            collision_left_rightrun_step_status = TURN_CCLOCK_YAW_ADD_ABS30_COLLISION_CL_DRYL;
        }
        linear_velocity = 0;
        angular_velocity = turn_vel;
        break;
    case TURN_CCLOCK_YAW_ADD_ABS30_COLLISION_CL_DRYL:
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
            collision_left_rightrun_step_status = TURN_CCLOCK_YAW_ADD_ABS30_CL_DRYL;
            turn_start_update = 0;
            break;
        }
        break;
    case MORE_TRY_BREAK_BYPASS_CL_DRYL:
        if (obstacleSignal != none_obstacle)
        {
            collision_left_rightrun_step_status = GOSTR_BYPASS_BOW_CONTINUE_COLLISION_CL_DRYL;
            bow_continue = true;
            break;
        }
        if (my_abs(Yaw / 100) > 175)
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
    case COMPLETE_CL_DRYL:
        collision_left_rightrun_step_status = 0;
        complete_flag = 1;
        break;
    case DIR_RIGHT_YAW_MORE_ABS90_CLRRS:
        linear_velocity = 0;
        angular_velocity = 0;
        collision_left_rightrun_step_status = TURN_CCLOCK_TARGET_YAW_ABS153_CL_DRYM;
        break;
    case TURN_CCLOCK_TARGET_YAW_ABS153_CL_DRYM:
        linear_velocity = 0;
        angular_velocity = turn_vel;
        if (my_abs(Yaw / 100) < 153)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            last_position_y = current_pose->y;
            collision_left_rightrun_step_status = GOSTR_YAW_EQUAL_ABS153_CL_DRYM;
            break;
        }
        if (obstacleSignal != none_obstacle)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            collision_left_rightrun_step_status = TURN_CCLOCK_TAEGET_YAW_ABS153_COLLISION_CL_DRYM;
            break;
        }
        break;
    case TURN_CCLOCK_TAEGET_YAW_ABS153_COLLISION_CL_DRYM:
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
            collision_left_rightrun_step_status = TURN_CCLOCK_TARGET_YAW_ABS153_CL_DRYM;
            break;
        }
        break;
    case GOSTR_YAW_EQUAL_ABS153_CL_DRYM:
        linear_velocity = long_stra_vel;
        angular_velocity = 0;
        if (obstacleSignal != none_obstacle)
        {
            collision_left_rightrun_step_status = TURN_CCLOCK_TARGET_YAW_NEG123_CL_DRYM;
            break;
        }
        if (my_abs(last_position_y - current_pose->y) > lateral_move_distance / 3)
        {
            collision_left_rightrun_step_status = TURN_CCLOCK_TARGET_YAW_NEG123_CL_DRYM;
            break;
        }
        break;
    case TURN_CCLOCK_TARGET_YAW_NEG123_CL_DRYM:
        linear_velocity = 0;
        angular_velocity = turn_vel;
        if (Yaw / 100 > -123)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            collision_left_rightrun_step_status = GOSTR_YAW_EQUAL_NEG123_CL_DRYM;
            break;
        }
        if (obstacleSignal != none_obstacle)
        {
            collision_left_rightrun_step_status = TURN_CCLOCK_TARGET_YAW_NEG123_COLLISION_CL_DRYM;
            break;
        }
        break;
    case TURN_CCLOCK_TARGET_YAW_NEG123_COLLISION_CL_DRYM:
        if (turn_start_update == 0)
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
    case GOSTR_YAW_EQUAL_NEG123_CL_DRYM:
        linear_velocity = long_stra_vel;
        angular_velocity = 0;
        if (obstacleSignal != none_obstacle)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            collision_left_rightrun_step_status = TURN_CCLOCK_TARGET_YAW_NEG98_CL_DRYM;
            break;
        }
        if (my_abs(last_position_y - current_pose->y) > (2 * lateral_move_distance) / 3)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            collision_left_rightrun_step_status = TURN_CCLOCK_TARGET_YAW_NEG98_CL_DRYM;
            break;
        }
        break;
    case TURN_CCLOCK_TARGET_YAW_NEG98_CL_DRYM:
        linear_velocity = 0;
        angular_velocity = turn_vel;
        if (Yaw / 100 > -98)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            collision_left_rightrun_step_status = GOSTR_YAW_EQUAL_NEG98_CL_DRYM;
            break;
        }
        if (obstacleSignal != none_obstacle)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            collision_left_rightrun_step_status = TURN_CCLOCK_TARGET_YAW_NEG98_COLLISION_CL_DRYM;
            break;
        }
        break;
    case TURN_CCLOCK_TARGET_YAW_NEG98_COLLISION_CL_DRYM:
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
            collision_left_rightrun_step_status = TURN_CCLOCK_TARGET_YAW_NEG98_CL_DRYM;
            break;
        }
        break;
    case GOSTR_YAW_EQUAL_NEG98_CL_DRYM:
        linear_velocity = long_stra_vel;
        angular_velocity = 0;
        if (obstacleSignal != none_obstacle)
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
    case RIGHT_REVERSE_WALK_EDGE_CL_DRYM:
        collision_left_rightrun_step_status = 0;
        complete_flag = 2;
        break;

    case TURN_CCLOCK_TARGET_YAW_ABS8_CL_DRYM:
        linear_velocity = 0;
        angular_velocity = turn_vel;
        if (my_abs(Yaw / 100) < 8)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            collision_left_rightrun_step_status = COMPLETE_CL_DRYM;
            break;
        }
        if (obstacleSignal != none_obstacle)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            collision_left_rightrun_step_status = TURN_CCLOCK_TARGET_YAW_ABS8_COLLISION_CL_DRYM;
            break;
        }
        break;
    case TURN_CCLOCK_TARGET_YAW_ABS8_COLLISION_CL_DRYM:
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
            collision_left_rightrun_step_status = TURN_CCLOCK_TARGET_YAW_ABS8_CL_DRYM;
            break;
        }
        break;
    case COMPLETE_CL_DRYM:
        collision_left_rightrun_step_status = 0;
        complete_flag = 1;
        break;
    }
    return complete_flag;
}

unsigned char  CollisionFrontRightRunStep(POSE *current_pose, unsigned char obstacleSignal)
{
    int Yaw;
    unsigned char complete_flag = 0;
    Yaw = current_pose->orientation;
    switch (collision_front_rightrun_step_status)
    {
    case 0:
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
        if (obstacleSignal != none_obstacle)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            if (my_abs(Yaw / 100) < 90)
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
        if (my_abs(turn_start_x - current_pose->x) > front_backward_distance)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            if (my_abs(Yaw / 100) < 90)
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
        if ((Yaw / 100) < -60)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            collision_front_rightrun_step_status = GOSTR_YAW_EQUAL_NEG60_CF_CF_DRYL;
            break;
        }
        if (obstacleSignal != none_obstacle)
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
        if (obstacleSignal != none_obstacle)
        {
            collision_front_rightrun_step_status = TURN_CLOCK_TARGET_YAW_NEG82_CF_DRYL;
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
        if (Yaw / 100 < -82)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            collision_front_rightrun_step_status = GOSTR_YAW_EQUAL_NEG82_CF_DRYL;
            break;
        }
        if (obstacleSignal != none_obstacle)
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
        if (obstacleSignal != none_obstacle)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            collision_front_rightrun_step_status = RIGHT_WALK_EDGE_CF_DRYL;
            break;
        }
        if (my_abs(last_position_y - current_pose->y) > (lateral_move_distance))
        {
            linear_velocity = 0;
            angular_velocity = 0;
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
        if (my_abs(Yaw / 100) > 173)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            collision_front_rightrun_step_status = COMPLETE_CF_DRYL;
            break;
        }
        if (obstacleSignal != none_obstacle)
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
        if (my_abs(Yaw / 100) < 120)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            collision_front_rightrun_step_status = GOSTR_YAW_ABS120_CF_DRYM;
            break;
        }
        if (obstacleSignal != none_obstacle)
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
        if (obstacleSignal != none_obstacle)
        {
            collision_front_rightrun_step_status = TURN_CCLOCK_TARGET_YAW_ABS93_CF_DRYM;
            break;
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
        if (my_abs(Yaw / 100) < 93)
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
        if (obstacleSignal != none_obstacle)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            collision_front_rightrun_step_status = RIGHT_REVERSE_WALK_EDGE_CF_DRYM;
            break;
        }
        if (my_abs(last_position_y - current_pose->y) > (lateral_move_distance))
        {
            linear_velocity = 0;
            angular_velocity = 0;
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
        if (my_abs(Yaw / 100) < 10)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            collision_front_rightrun_step_status = COMPLETE_CF_DRYL;
            break;
        }
        if (obstacleSignal != none_obstacle)
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

unsigned char  RightEdgeDilemma(POSE *current_pose, unsigned char obstacleSignal)
{
    ////cout << " RightEdgeDilemma..............==============RightEdgeDilemma" << endl;
    short Yaw;
    unsigned char complete_flag = 0;
    Yaw = current_pose->orientation;
    switch (right_edge_dilemma_status)
    {
    case 0:
        last_position_x = current_pose->x + half_map_wide;
        linear_velocity = 0;
        angular_velocity = 0;
        right_edge_dilemma_status = DELTA_X_MORE_ONE_THIRD_CLEANED_MAP_WIDTH_DILEMMA;
        break;
    case DELTA_X_MORE_ONE_THIRD_CLEANED_MAP_WIDTH_DILEMMA:
        if (my_abs(last_position_x - (current_pose->x + half_map_wide)) > Edge_length() / 3)
        {
            complete_flag = 1;
            right_edge_dilemma_status = COMPLETE_EL_DRYM;
            DelimmaNumber = 0;
            break;
        }
        right_edge_dilemma_status = LOOP_TEN_NUM_DILEMMA;
        break;
    case LOOP_TEN_NUM_DILEMMA:
        if (DelimmaNumber > 4)
        {
            complete_flag = 1;
            right_edge_dilemma_status = COMPLETE_EL_DRYM;
            DelimmaNumber = 0;
            break;
        }
        right_edge_dilemma_status = YAW_MORE_ABS90_LESS_ABS176_CCLOCK_LESS_ABS90_MORE_ABS4_CLOCK_DILEMMA;
        break;

    case COMPLETE_EL_DRYM:
        complete_flag = 1;
        right_edge_dilemma_status = 0;
        DelimmaNumber = 0;
        break;
    case YAW_MORE_ABS90_LESS_ABS176_CCLOCK_LESS_ABS90_MORE_ABS4_CLOCK_DILEMMA:
        if (my_abs(Yaw / 100) > 90 && my_abs(Yaw / 100) < 175)
        {
            if (Yaw > 0)
            {
                linear_velocity = 0;
                angular_velocity = turn_vel;
            }
            else
            {
                linear_velocity = 0;
                angular_velocity = -turn_vel;
            }
        }
        else if (my_abs(Yaw / 100) < 90 && my_abs(Yaw / 100) > 5)
        {
            if (Yaw > 0)
            {
                linear_velocity = 0;
                angular_velocity = -turn_vel;
            }
            else
            {
                linear_velocity = 0;
                angular_velocity = turn_vel;
            }
        }
        else
        {
            linear_velocity = long_stra_vel;
            angular_velocity = 0;
        }
        right_edge_dilemma_status = GOSTR_DILEMMA;
        break;
    case GOSTR_DILEMMA:
        if (obstacleSignal != none_obstacle)
        {
            right_edge_dilemma_status = GOSTR_COLLISION_DILEMMA;
            break;
        }
        right_edge_dilemma_status = DELTA_X_MORE_ONE_THIRD_CLEANED_MAP_WIDTH_DILEMMA;
        break;
    case GOSTR_COLLISION_DILEMMA:
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
            if (my_abs(Yaw / 100) >= 90)
            {
                right_edge_dilemma_status = COLLISION_YAW_MORE_ABS90_DILEMMA;
                break;
            }
            else
            {
                right_edge_dilemma_status = COLLISION_YAW_LESS_ABS90_DILEMMA;
                break;
            }
        }
        break;
    case COLLISION_YAW_MORE_ABS90_DILEMMA:
        right_edge_dilemma_status = CLOCK_TARGET_YAW_LESS_ABS90_DILEMMA;
        break;
    case CLOCK_TARGET_YAW_LESS_ABS90_DILEMMA:
        if (my_abs(Yaw / 100) < 90)
        {
            last_position_y = current_pose->y;
            last_position_x = current_pose->x + half_map_wide;
            right_edge_dilemma_status = GOSTR_CYM_DILEMMA;
            break;
        }
        if (obstacleSignal != none_obstacle)
        {
            right_edge_dilemma_status = CLOCK_TARGET_YAW_LESS_ABS90_COLLISION_DILEMMA;
        }
        linear_velocity = 0;
        angular_velocity = -turn_vel;
        break;
    case CLOCK_TARGET_YAW_LESS_ABS90_COLLISION_DILEMMA:
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
            right_edge_dilemma_status = CLOCK_TARGET_YAW_LESS_ABS90_DILEMMA;
        }
        break;
    case GOSTR_CYM_DILEMMA:
        if (obstacleSignal != none_obstacle)
        {
            DelimmaNumber++;
            right_edge_dilemma_status = GOSTR_COLLISION_CYM_DILEMMA;
            break;
        }
        if (my_abs(last_position_y - current_pose->y) > lateral_move_distance / 3)
        {
            right_edge_dilemma_status = GOSTR_DELTA_Y_MORE_LATERAL_DIS_CYM_DILEMMA;
            break;
        }
        linear_velocity = long_stra_vel;
        angular_velocity = 0;
        break;
    case GOSTR_COLLISION_CYM_DILEMMA:
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
            right_edge_dilemma_status = CLOCK_TARGET_YAW_LESS_ABS3_DILEMMA;
            break;
        }
        break;
    case GOSTR_DELTA_Y_MORE_LATERAL_DIS_CYM_DILEMMA:
        right_edge_dilemma_status = CLOCK_TARGET_YAW_LESS_ABS3_DILEMMA;
        break;
    case CLOCK_TARGET_YAW_LESS_ABS3_DILEMMA:
        if (my_abs(Yaw / 100) < 5)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            right_edge_dilemma_status = DELTA_X_MORE_ONE_THIRD_CLEANED_MAP_WIDTH_DILEMMA;
            break;
        }
        if (obstacleSignal != none_obstacle)
        {
            right_edge_dilemma_status = CLOCK_TARGET_YAW_LESS_ABS3_COLLISION_DILEMMA;
            break;
        }
        linear_velocity = 0;
        angular_velocity = -turn_vel;
        break;
    case CLOCK_TARGET_YAW_LESS_ABS3_COLLISION_DILEMMA:
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
            right_edge_dilemma_status = CLOCK_TARGET_YAW_LESS_ABS3_DILEMMA;
        }
        break;
    case COLLISION_YAW_LESS_ABS90_DILEMMA:
        right_edge_dilemma_status = CCLOCK_TARGET_YAW_MORE_ABS90_DILEMMA;
        break;
    case CCLOCK_TARGET_YAW_MORE_ABS90_DILEMMA:
        if (my_abs(Yaw / 100) > 90)
        {
            last_position_y = current_pose->y;
            last_position_x = current_pose->x + half_map_wide;
            right_edge_dilemma_status = GOSTR_CYL_DILEMMA;
            break;
        }
        if (obstacleSignal != none_obstacle)
        {
            right_edge_dilemma_status = CCLOCK_TARGET_YAW_MORE_ABS90_COLLISION_DILEMMA;
            break;
        }

        linear_velocity = 0;
        angular_velocity = turn_vel;
        break;
    case CCLOCK_TARGET_YAW_MORE_ABS90_COLLISION_DILEMMA:
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
            right_edge_dilemma_status = CCLOCK_TARGET_YAW_MORE_ABS90_DILEMMA;
        }
        break;
    case GOSTR_CYL_DILEMMA:
        if (obstacleSignal != none_obstacle)
        {
            DelimmaNumber++;
            right_edge_dilemma_status = GOSTR_COLLISION_CYL_DILEMMA;
            break;
        }
        if (my_abs(last_position_y - current_pose->y) > lateral_move_distance / 3)
        {
            right_edge_dilemma_status = GOSTR_GOSTR_DELTA_Y_MORE_LATERAL_DIS_CYL_DILEMMA;
            break;
        }
        linear_velocity = long_stra_vel;
        angular_velocity = 0;
        break;
    case GOSTR_COLLISION_CYL_DILEMMA:
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
            right_edge_dilemma_status = CCLOCK_TARGET_YAW_MORE_ABS178_DILEMMA;
            break;
        }
        break;
    case GOSTR_GOSTR_DELTA_Y_MORE_LATERAL_DIS_CYL_DILEMMA:
        right_edge_dilemma_status = CCLOCK_TARGET_YAW_MORE_ABS178_DILEMMA;
        break;
    case CCLOCK_TARGET_YAW_MORE_ABS178_DILEMMA:
        if (my_abs(Yaw / 100) > 175)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            right_edge_dilemma_status = DELTA_X_MORE_ONE_THIRD_CLEANED_MAP_WIDTH_DILEMMA;
            break;
        }
        if (obstacleSignal != none_obstacle)
        {
            right_edge_dilemma_status = CCLOCK_TARGET_YAW_MORE_ABS178_COLLISION_DILEMMA;
            break;
        }
        linear_velocity = 0;
        angular_velocity = turn_vel;
        break;
    case CCLOCK_TARGET_YAW_MORE_ABS178_COLLISION_DILEMMA:
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
            right_edge_dilemma_status = CCLOCK_TARGET_YAW_MORE_ABS178_DILEMMA;
        }
        break;
    }
    return complete_flag;
}

unsigned char  RightWalkEdge(POSE *current_pose, unsigned char obstacleSignal)
{
    int Yaw;
    unsigned char complete_flag = 0;
    Yaw = current_pose->orientation;
    switch (right_walk_edge_status)
    {
    case 0:
        right_walk_edge_status = GOBACK_WALK_EDGE;
        break;
    case GOBACK_WALK_EDGE:
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
            right_walk_edge_status = TURN_CLOCK_TARGET_YAW_MORE_ABS117_WE;
            break;
        }
        break;
    case TURN_CLOCK_TARGET_YAW_MORE_ABS117_WE:
        linear_velocity = 0;
        angular_velocity = -turn_vel;
        if (my_abs(Yaw / 100) > 147)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            right_walk_edge_status = READY_GOSTR_BYPASS_WE;
            break;
        }
        if (obstacleSignal != none_obstacle)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            right_walk_edge_status = TURN_CLOCK_TARGET_YAW_MORE_ABS117_COLLISION_WE;
            break;
        }
        break;
    case TURN_CLOCK_TARGET_YAW_MORE_ABS117_COLLISION_WE:
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
            right_walk_edge_status = TURN_CLOCK_TARGET_YAW_MORE_ABS117_WE;
            break;
        }
        break;
    case READY_GOSTR_BYPASS_WE:
        edge_length_start = current_pose->x + half_map_wide;
        returnorigin = false;
        b_last_position_yy = false;
        last_position_y = current_pose->y;
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
        linear_velocity = long_stra_vel;
        angular_velocity = 0;
        if (obstacleSignal != none_obstacle)
        {
            right_walk_edge_status = COLLISION_BYPASS_WE;
            break;
        }

        if (my_abs(last_position_x - current_pose->x) > lateral_move_distance)
        {
            right_walk_edge_status = GOSTR_X_MORE_LATERALDIS_BYPASS_WE;
            break;
        }
        if (last_position_y - current_pose->y > temporary_close_edge)
        {
            right_walk_edge_status = BOW_CONTINUE_WE;
            break;
        }
        if (b_last_position_yy == true && my_abs(last_position_yy - current_pose->y) > 3 * lateral_move_distance)
        {
            right_walk_edge_status = BOW_CONTINUE_WE;
            break;
        }
        if (my_abs(Yaw / 100) < 120 && (Yaw / 100 > 0) && (b_last_position_yy == false))
        {
            last_position_yy = current_pose->y;
            b_last_position_yy = true;
        }
        break;

    case REBACK_GOSTR_BYPASS_CHECK_WE:
        if (my_abs(Yaw / 100) < 135 && (Yaw / 100 > 0))
        {
            right_walk_edge_status = TARGET_YAW_LESS_ABS105_MORE_0_BYPASS_WE;
            break;
        }

        if ((my_abs(current_pose->x + half_map_wide - edge_length_start) > Edge_length() / 2) && (my_abs(current_pose->x + half_map_wide - edge_length_start) > 500))
        {
            right_walk_edge_status = DELTA_X_MORE_ONE_FOURTH_CLEANED_MAP_WIDTH_WE;
            break;
        }
        right_walk_edge_status = GOSTR_BYPASS_WE_X;
        break;
    case COLLISION_BYPASS_WE:
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
            temporary_yaw = Yaw / 100;
            right_walk_edge_status = TURN_CLOCK_YAW_ADD_ABS15_WE;
            break;
        }
        break;
    case TURN_CLOCK_YAW_ADD_ABS15_WE:
        linear_velocity = 0;
        angular_velocity = -turn_vel;
        if (my_abs(Yaw / 100 - temporary_yaw) > 15)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            right_walk_edge_status = REBACK_GOSTR_BYPASS_CHECK_WE;
            break;
        }
        if (obstacleSignal != none_obstacle)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            right_walk_edge_status = TURN_CLOCK_YAW_ADD_ABS15_COLLISION_WE;
            break;
        }
        break;
    case TURN_CLOCK_YAW_ADD_ABS15_COLLISION_WE:
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
            right_walk_edge_status = TURN_CLOCK_YAW_ADD_ABS15_WE;
            break;
        }
        break;
    case TARGET_YAW_LESS_ABS105_MORE_0_BYPASS_WE:
        linear_velocity = 0;
        angular_velocity = 0;
        if (!detection_close_edge)
        {
            detection_close_edge = true;
        }
        else
        {
            detection_close_edge = false;
            if (detection_close == true)
            {
                right_walk_edge_status = TURN_CLOCK_TARGET_YAW_LESS_ABS3_WE;
                break;
            }
            else
            {
                right_walk_edge_status = RETURN_ORIGIN_WE;
                break;
            }
        }
        break;
    case DELTA_X_MORE_ONE_FOURTH_CLEANED_MAP_WIDTH_WE:
        linear_velocity = 0;
        angular_velocity = 0;
        if (!detection_close_edge)
        {
            detection_close_edge = true;
        }
        else
        {
            detection_close_edge = false;
            if (detection_close == true)
            {
                temporary_yaw = Yaw / 100;
                right_walk_edge_status = CLOSE_EDGE_MAP_RIGHT_WALK_TURN_CLOCK_YAW_ADD_ABS15_WE;
            }
            else
            {
                right_walk_edge_status = RETURN_ORIGIN_WE;
            }
        }
        break;

    case CLOSE_EDGE_MAP_RIGHT_WALK_TURN_CLOCK_YAW_ADD_ABS15_WE:
        linear_velocity = 0;
        angular_velocity = -turn_vel;
        if (my_abs(Yaw / 100 - temporary_yaw) > 15)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            last_position_x = current_pose->x;
            right_walk_edge_status = GOSTR_BYPASS_WE;
            break;
        }
        if (obstacleSignal != none_obstacle)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            right_walk_edge_status = COLLISION_CLOSE_EDGE_MAP_RIGHT_WALK_TURN_CLOCK_YAW_ADD_ABS15_WE;
            break;
        }
        break;
    case COLLISION_CLOSE_EDGE_MAP_RIGHT_WALK_TURN_CLOCK_YAW_ADD_ABS15_WE:
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
            right_walk_edge_status = CLOSE_EDGE_MAP_RIGHT_WALK_TURN_CLOCK_YAW_ADD_ABS15_WE;
            break;
        }
        break;

    case TURN_CLOCK_TARGET_YAW_LESS_ABS3_WE:
        if (my_abs(Yaw / 100) < 5)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            right_walk_edge_status = RIGHT_EDGE_DILEMMA_WE;
            break;
        }
        if (obstacleSignal != none_obstacle)
        {
            right_walk_edge_status = TURN_CLOCK_TARGET_YAW_LESS_ABS3_COLLISION_WE;
            break;
        }
        linear_velocity = 0;
        angular_velocity = -turn_vel;
        break;

    case TURN_CLOCK_TARGET_YAW_LESS_ABS3_COLLISION_WE:
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
            right_walk_edge_status = TURN_CLOCK_TARGET_YAW_LESS_ABS3_WE;
            break;
        }
        break;
    case GOSTR_X_MORE_LATERALDIS_BYPASS_WE:
        temporary_yaw = Yaw / 100;
        if (my_abs(Yaw / 100) < 135 && (Yaw / 100 > 0))
        {
            right_walk_edge_status = TURN_CCLOCK_TARGET_YAW_LESS_0_WE;
        }
        else if (my_abs(Yaw / 100) > 135)
        {
            right_walk_edge_status = TURN_CCLOCK_TARGET_YAW_LESS_135_DETAL_YAW_MORE_30_WE;
        }
        else
        {
            right_walk_edge_status = GOSTR_BYPASS_WE_X;
        }
        break;
    case TURN_CCLOCK_TARGET_YAW_LESS_0_WE:
        linear_velocity = 0;
        angular_velocity = turn_vel;
        if (Yaw / 100 < 0)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            right_walk_edge_status = GOSTR_BYPASS_WE_X;
            break;
        }
        if (obstacleSignal != none_obstacle)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            right_walk_edge_status = TURN_CCLOCK_TARGET_YAW_LESS_0_COLLISION_WE;
            break;
        }
        break;
    case TURN_CCLOCK_TARGET_YAW_LESS_0_COLLISION_WE:
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
            right_walk_edge_status = TURN_CCLOCK_TARGET_YAW_LESS_0_WE;
            break;
        }
        break;
    case TURN_CCLOCK_TARGET_YAW_LESS_135_DETAL_YAW_MORE_30_WE:
        linear_velocity = 0;
        angular_velocity = turn_vel;
        if (my_abs(temporary_yaw - Yaw / 100) > 30 && my_abs(Yaw / 100) < 135)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            right_walk_edge_status = GOSTR_BYPASS_WE_X;
            break;
        }
        if (obstacleSignal != none_obstacle)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            right_walk_edge_status = TURN_CCLOCK_TARGET_YAW_LESS_135_DETAL_YAW_MORE_30_COLLISION_WE;
            break;
        }
        break;
    case TURN_CCLOCK_TARGET_YAW_LESS_135_DETAL_YAW_MORE_30_COLLISION_WE:
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
            right_walk_edge_status = TURN_CCLOCK_TARGET_YAW_LESS_135_DETAL_YAW_MORE_30_WE;
            break;
        }
        break;
    case BOW_CONTINUE_WE:
        linear_velocity = 0;
        angular_velocity = 0;
        if (!detection_close_edge)
        {
            detection_close_edge = true;
        }
        else
        {
            detection_close_edge = false;
            if (detection_close == true)
            {
                right_walk_edge_status = 0;
                complete_flag = 1;
            }
            else
            {
				if(my_abs(current_pose->x + half_map_wide - edge_length_start) > Edge_length() / 2){
                right_walk_edge_status = RETURN_ORIGIN_WE;
				}else{
				right_walk_edge_status = 0;
                complete_flag = 1;
				}
            }
        }
        break;

    case RIGHT_EDGE_DILEMMA_WE:
        linear_velocity = long_stra_vel;
        angular_velocity = 0;
        complete_flag = 2;
        right_walk_edge_status = 0;
        break;
    case RETURN_ORIGIN_WE:
        linear_velocity = 0;
        angular_velocity = 0;
        complete_flag = 3;
        right_walk_edge_status = 0;
        break;
    }
    return complete_flag;
}

unsigned char  RightReverseWalkEdge(POSE *current_pose, unsigned char obstacleSignal)
{
    int Yaw;
    unsigned char complete_flag = 0;
    Yaw = current_pose->orientation;
    switch (right_reverse_walk_edge_status)
    {
    case 0:
        right_reverse_walk_edge_status = GOBACK_REVERSE_WALK_EDGE;
        break;
    case GOBACK_REVERSE_WALK_EDGE:
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
            right_reverse_walk_edge_status = TURN_CCLOCK_TARGET_YAW_LESS_ABS63_RWE;
            break;
        }
        break;
    case TURN_CCLOCK_TARGET_YAW_LESS_ABS63_RWE:
        linear_velocity = 0;
        angular_velocity = turn_vel;
        if (my_abs(Yaw / 100) < 45)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            right_reverse_walk_edge_status = READY_GOSTR_BYPASS_RWE;
            break;
        }
        if (obstacleSignal != none_obstacle)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            right_reverse_walk_edge_status = TURN_CCLOCK_TARGET_YAW_LESS_ABS63_COLLISION_RWE;
            break;
        }
        break;
    case TURN_CCLOCK_TARGET_YAW_LESS_ABS63_COLLISION_RWE:
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
            right_reverse_walk_edge_status = TURN_CCLOCK_TARGET_YAW_LESS_ABS63_RWE;
            break;
        }
        break;
    case READY_GOSTR_BYPASS_RWE:
        edge_length_start = current_pose->x + half_map_wide;
        returnorigin = false;
        b_last_position_yy = false;
        last_position_y = current_pose->y;
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
        linear_velocity = long_stra_vel;
        angular_velocity = 0;
        if (obstacleSignal != none_obstacle)
        {
            right_reverse_walk_edge_status = COLLISION_BYPASS_RWE;
            break;
        }
        if (my_abs(last_position_x - current_pose->x) > lateral_move_distance)
        {
            right_reverse_walk_edge_status = GOSTR_X_MORE_LATERALDIS_BYPASS_RWE;
            break;
        }
        if (last_position_y - current_pose->y > temporary_close_edge)
        {
            right_reverse_walk_edge_status = BOW_CONTINUE_RWE;
            break;
        }
        if (b_last_position_yy == true && my_abs(last_position_yy - current_pose->y) > 3 * lateral_move_distance)
        {
            right_reverse_walk_edge_status = BOW_CONTINUE_RWE;
            break;
        }
        if (my_abs(Yaw / 100) < 120 && (Yaw / 100 > 0) && (b_last_position_yy == false))
        {
            last_position_yy = current_pose->y;
            b_last_position_yy = true;
        }
        break;
    case REBACK_GOSTR_BYPASS_CHECK_RWE:
        if (Yaw / 100 > 45)
        {
            right_reverse_walk_edge_status = TARGET_YAW_MORE_ABS75_MORE_ABS0_RWE;
            break;
        }

        if ((my_abs(current_pose->x + half_map_wide - edge_length_start) > Edge_length() / 2) && (my_abs(current_pose->x + half_map_wide - edge_length_start) > 500))
        {
            right_reverse_walk_edge_status = DELTA_X_MORE_ONE_FOURTH_CLEANED_MAP_WIDTH_RWE;
            break;
        }
        right_reverse_walk_edge_status = GOSTR_BYPASS_RWE_X;
        break;
    case COLLISION_BYPASS_RWE:
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
            temporary_yaw = Yaw / 100;
            right_reverse_walk_edge_status = TURN_CCLOCK_YAW_ADD_ABS15_RWE;
            break;
        }
        break;
    case TURN_CCLOCK_YAW_ADD_ABS15_RWE:
        linear_velocity = 0;
        angular_velocity = turn_vel;
        if (my_abs(Yaw / 100 - temporary_yaw) > 15)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            right_reverse_walk_edge_status = REBACK_GOSTR_BYPASS_CHECK_RWE;
            break;
        }
        if (obstacleSignal != none_obstacle)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            right_reverse_walk_edge_status = TURN_CCLOCK_YAW_ADD_ABS15_COLLISION_RWE;
            break;
        }
        break;
    case TURN_CCLOCK_YAW_ADD_ABS15_COLLISION_RWE:
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
            right_reverse_walk_edge_status = TURN_CCLOCK_YAW_ADD_ABS15_RWE;
            break;
        }
        break;
    case TARGET_YAW_MORE_ABS75_MORE_ABS0_RWE:
        linear_velocity = 0;
        angular_velocity = 0;
        if (!detection_close_edge)
        {
            detection_close_edge = true;
            break;
        }
        else
        {
            detection_close_edge = false;
            if (detection_close == true)
            {
                right_reverse_walk_edge_status = TURN_CCLOCK_YAW_MORE_178ABS_RWE;
            }
            else
            {
                right_reverse_walk_edge_status = RETURN_ORIGIN_RWE;
            }
            break;
        }

    case DELTA_X_MORE_ONE_FOURTH_CLEANED_MAP_WIDTH_RWE:
        linear_velocity = 0;
        angular_velocity = 0;
        if (!detection_close_edge)
        {
            detection_close_edge = true;
        }
        else
        {
            detection_close_edge = false;
            if (detection_close == true)
            {
                temporary_yaw = Yaw / 100;
                right_reverse_walk_edge_status = CLOSE_EDGE_MAP_RIGHT_REVERSE_WALK_TURN_CCLOCK_YAW_ADD_ABS15_RWE;
            }
            else
            {
                right_reverse_walk_edge_status = RETURN_ORIGIN_RWE;
            }
        }
        break;

    case CLOSE_EDGE_MAP_RIGHT_REVERSE_WALK_TURN_CCLOCK_YAW_ADD_ABS15_RWE:
        linear_velocity = 0;
        angular_velocity = turn_vel;
        if (my_abs(Yaw / 100 - temporary_yaw) > 15)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            last_position_x = current_pose->x;
            right_reverse_walk_edge_status = GOSTR_BYPASS_RWE;
            break;
        }
        if (obstacleSignal != none_obstacle)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            right_reverse_walk_edge_status = COLLISION_CLOSE_EDGE_MAP_RIGHT_REVERSE_WALK_TURN_CCLOCK_YAW_ADD_ABS15_RWE;
            break;
        }
        break;
    case COLLISION_CLOSE_EDGE_MAP_RIGHT_REVERSE_WALK_TURN_CCLOCK_YAW_ADD_ABS15_RWE:
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
            right_reverse_walk_edge_status = CLOSE_EDGE_MAP_RIGHT_REVERSE_WALK_TURN_CCLOCK_YAW_ADD_ABS15_RWE;
            break;
        }
        break;

    case TURN_CCLOCK_YAW_MORE_178ABS_RWE:
        if (my_abs(Yaw / 100) > 175)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            right_reverse_walk_edge_status = RIGHT_EDGE_DILEMMA_RWE;
            break;
        }
        if (obstacleSignal != none_obstacle)
        {
            right_reverse_walk_edge_status = TURN_CCLOCK_YAW_MORE_178ABS_COLLISION_RWE;
            break;
        }
        linear_velocity = 0;
        angular_velocity = turn_vel;
        break;
    case TURN_CCLOCK_YAW_MORE_178ABS_COLLISION_RWE:
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
            right_reverse_walk_edge_status = TURN_CCLOCK_YAW_MORE_178ABS_RWE;
            break;
        }
        break;

    case GOSTR_X_MORE_LATERALDIS_BYPASS_RWE:
        temporary_yaw = Yaw / 100;
        if (my_abs(Yaw / 100) > 45 && (Yaw / 100 > 0))
        {
            right_reverse_walk_edge_status = TURN_CLOCK_TARGET_YAW_LESS_0_RWE;
        }
        else if (my_abs(Yaw / 100) <= 45)
        {
            right_reverse_walk_edge_status = TURN_CLOCK_TARGET_YAW_LESS_ABS45_DETAL_YAW_MORE_30_RWE;
        }
        else
        {
            right_reverse_walk_edge_status = GOSTR_BYPASS_RWE_X;
        }
        break;

    case TURN_CLOCK_TARGET_YAW_LESS_0_RWE:
        linear_velocity = 0;
        angular_velocity = -turn_vel;
        if (Yaw / 100 < 0)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            right_reverse_walk_edge_status = GOSTR_BYPASS_RWE_X;
            break;
        }
        if (obstacleSignal != none_obstacle)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            right_reverse_walk_edge_status = TURN_CLOCK_TARGET_YAW_LESS_0_COLLISION_RWE;
            break;
        }
        break;
    case TURN_CLOCK_TARGET_YAW_LESS_0_COLLISION_RWE:
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
            right_reverse_walk_edge_status = TURN_CLOCK_TARGET_YAW_LESS_0_RWE;
            break;
        }
        break;
    case TURN_CLOCK_TARGET_YAW_LESS_ABS45_DETAL_YAW_MORE_30_RWE:
        linear_velocity = 0;
        angular_velocity = -turn_vel;
        if (my_abs(temporary_yaw - Yaw / 100) > 30 && my_abs(Yaw / 100) > 45)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            right_reverse_walk_edge_status = GOSTR_BYPASS_RWE_X;
            break;
        }
        if (obstacleSignal != none_obstacle)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            right_reverse_walk_edge_status = TURN_CLOCK_TARGET_YAW_LESS_ABS45_DETAL_YAW_MORE_30_COLLISION_RWE;
            break;
        }
        break;
    case TURN_CLOCK_TARGET_YAW_LESS_ABS45_DETAL_YAW_MORE_30_COLLISION_RWE:
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
            right_reverse_walk_edge_status = TURN_CLOCK_TARGET_YAW_LESS_ABS45_DETAL_YAW_MORE_30_RWE;
            break;
        }
        break;
    case BOW_CONTINUE_RWE:
        linear_velocity = 0;
        angular_velocity = 0;
        if (!detection_close_edge)
        {
            detection_close_edge = true;
        }
        else
        {
            detection_close_edge = false;
            if (detection_close == true)
            {
                right_reverse_walk_edge_status = 0;
                complete_flag = 1;
            }
            else
            {
				if(my_abs(current_pose->x + half_map_wide - edge_length_start) > Edge_length() / 2){
					right_reverse_walk_edge_status = RETURN_ORIGIN_RWE;
				}
                else{
				right_reverse_walk_edge_status = 0;
                complete_flag = 1;
				}
            }
        }
        break;
    case RIGHT_EDGE_DILEMMA_RWE:
        linear_velocity = long_stra_vel;
        angular_velocity = 0;
        complete_flag = 2;
        right_reverse_walk_edge_status = 0;
        break;
    case RETURN_ORIGIN_RWE:
        linear_velocity = 0;
        angular_velocity = 0;
        complete_flag = 3;
        right_reverse_walk_edge_status = 0;
        break;
    }
    return complete_flag;
}

unsigned char  ForwardBoundaryRightRunStep(POSE *current_pose, unsigned char obstacleSignal)
{
    int Yaw;
    unsigned char complete_flag = 0;
    Yaw = current_pose->orientation;
    if (judgment_Stuck_status == 0)
    {
        judgment_Stuck_status_x = current_pose->x;
        judgment_Stuck_status_y = current_pose->y;
        judgment_Stuck_status_yaw = Yaw / 100;
        judgment_Stuck_status = 1;
    }
    if (my_abs(current_pose->x - judgment_Stuck_status_x) > lateral_move_distance / 2 || my_abs(current_pose->y - judgment_Stuck_status_y) > lateral_move_distance / 2)
    {
        judgment_Stuck_status = 0;
        if (my_abs(judgment_Stuck_status_yaw - Yaw / 100) < 10)
        {
            if (my_abs(current_pose->x) > half_map_wide)
            {
                stuck_x = true;
            }
            else
            {
                stuck_y = true;
            }
            return complete_flag = 2;
        }
    }
    switch (right_forward_boundary_status)
    {
    case 0:
        if (my_abs(Yaw / 100) < 90)
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
        if (my_abs(Yaw / 100) > 175)
        {
            right_forward_boundary_status = FORWARDBOUNDARY_GOSTRAIGHT;
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
        if (my_abs(Yaw / 100) < 5)
        {
            right_forward_boundary_status = FORWARDBOUNDARY_GOSTRAIGHT;
            break;
        }
        if (obstacleSignal != none_obstacle)
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
        if ((my_abs(turn_start_x - current_pose->x) > 50 || my_abs(turn_start_y - current_pose->y) > 50) && my_abs(Yaw / 100) < 90 && current_pose->x > 0)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            turn_start_update = 0;
            right_forward_boundary_status = FORWARDBOUNDARY_COMPLETE;
            break;
        }
        if ((my_abs(turn_start_x - current_pose->x) > 50 || my_abs(turn_start_y - current_pose->y) > 50) && my_abs(Yaw / 100) > 90 && current_pose->x < 0)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            turn_start_update = 0;
            right_forward_boundary_status = FORWARDBOUNDARY_COMPLETE;
            break;
        }

        linear_velocity = long_stra_vel;
        angular_velocity = 0;
        if (obstacleSignal != none_obstacle)
        {
            right_forward_boundary_status = FORWARDBOUNDARY_COMPLETE;
            break;
        }
        if (my_abs(current_pose->x) < half_map_wide)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            right_forward_boundary_status = FORWARDBOUNDARY_COMPLETE;
            break;
        }
        break;
    case FORWARDBOUNDARY_COMPLETE:
        right_forward_boundary_status = 0;
        judgment_Stuck_status = 0;
        complete_flag = 1;
        break;
    }
    return complete_flag;
}

unsigned char  StuckRightRunStep(POSE *current_pose, unsigned char obstacleSignal)
{
    int Yaw;
	int8_t i = 0,k=0, j = 0;
    unsigned char complete_flag = 0;
    Yaw = current_pose->orientation;
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
            temporary_yaw = Yaw / 100;
            stuck_right_run_step = TURN_RIGHT_YAW_MORE_90_STUCK_FORWARD_BOUNDARY_STATUS;
            break;
        }
        break;
    case TURN_RIGHT_YAW_MORE_90_STUCK_FORWARD_BOUNDARY_STATUS:
        if (my_abs(Yaw / 100 - temporary_yaw) > 90)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            last_position_x = current_pose->x;
            last_position_y = current_pose->y;
            stuck_right_run_step = GO_STUCK_FORWARD_BOUNDARY_STATUS;
            break;
        }
        if (obstacleSignal != none_obstacle)
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
        if (obstacleSignal != none_obstacle)
        {
            stuck_right_run_step = COLLISION_GO_STUCK_FORWARD_BOUNDARY_STATUS;
            break;
        }
        if (my_abs(last_position_x - current_pose->x) > lateral_move_distance || my_abs(last_position_y - current_pose->y) > lateral_move_distance)
        {
            temporary_yaw = Yaw / 100;
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
            temporary_yaw = Yaw / 100;
            stuck_right_run_step = TURN_RIGHT_YAW_MORE_10_STUCK_FORWARD_BOUNDARY_STATUS;
            break;
        }
        break;
    case TURN_RIGHT_YAW_MORE_10_STUCK_FORWARD_BOUNDARY_STATUS:
        if (my_abs(Yaw / 100 - temporary_yaw) > 10)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            stuck_right_run_step = GO_STUCK_FORWARD_BOUNDARY_STATUS;
            break;
        }
        if (obstacleSignal != none_obstacle)
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
        if (my_abs(Yaw / 100 - temporary_yaw) > 90)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            last_position_x = current_pose->x;
            last_position_y = current_pose->y;
            stuck_right_run_step = ESCAPE_FROM_STUCK_FORWARD_BOUNDARY_STATUS;
            break;
        }
        if (obstacleSignal != none_obstacle)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            temporary_yaw = Yaw / 100;
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
        if (obstacleSignal != none_obstacle)
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
            temporary_yaw = Yaw / 100;
            stuck_right_run_step = TURN_COLLISION_ESCAPE_FROM_STUCK_FORWARD_BOUNDARY_STATUS;
            break;
        }
        break;
    case TURN_COLLISION_ESCAPE_FROM_STUCK_FORWARD_BOUNDARY_STATUS:
        if (my_abs(Yaw / 100 - temporary_yaw) > 10)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            stuck_right_run_step = ESCAPE_FROM_STUCK_FORWARD_BOUNDARY_STATUS;
            break;
        }
        if (obstacleSignal != none_obstacle)
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
        linear_velocity = 0;
        angular_velocity = 0;
        if (stuck_x == true)
        {
            if (current_pose->x > 0)
            {
                for (i = 0; i < MAPHEIGHT / GRIDHEIGHT; i++)
                {
                    k = 0;
                    ////cout << endl;
                    for (j = 0; j < MAPWIDTH / GRIDWIDTH; j++)
                    {
                        if (gridmap.map[i][j] != 125)
                        {
                            k++;
                        }
                    }
                    if (k >= 10)
                    {
                        x_error = 100 * i - half_map_wide;
                        x_error = current_pose->x - x_error;
                        break;
                    }
                }
            }
            else
            {
                for (i = MAPHEIGHT / GRIDHEIGHT - 1; i >= 0; i--)
                {
                    k = 0;
                    ////cout << endl;
                    for (  j = MAPWIDTH / GRIDWIDTH - 1; j >= 0; j--)
                    {
                        if (gridmap.map[i][j] != 125)
                        {
                            k++;
                        }
                    }
                    if (k >= 10)
                    {
                        x_error = 100 * i - half_map_wide;
                        x_error = current_pose->x - x_error;
                        break;
                    }
                }
            }
            for (  i = 0; i < MAPHEIGHT / GRIDHEIGHT; i++)
            {
                  k = 0;
                ////cout << endl;
                for (  j = 0; j < MAPWIDTH / GRIDWIDTH; j++)
                {
                    if (gridmap.map[j][i] != 125)
                    {
                        k++;
                    }
                }
                if (k >= 10)
                {
                    for ( j = 0; j < MAPWIDTH / GRIDWIDTH; j++)
                    {
                        gridmap.map[j][i] = 250;
                    }
                }
            }
        }
        if (stuck_y == true)
        {
            if (current_pose->y < 0)
            {
                for (  i = 0; i < MAPHEIGHT / GRIDHEIGHT; i++)
                {
                      k = 0;
                    ////cout << endl;
                    for (  j = 0; j < MAPWIDTH / GRIDWIDTH; j++)
                    {
                        if (gridmap.map[j][i] != 125)
                        {
                            k++;
                        }
                    }
                    if (k >= 10)
                    {
                        y_error = 100 * i - half_map_wide;
                        y_error = current_pose->y - y_error;
                        break;
                    }
                }
            }
            else
            {
                for (  i = MAPHEIGHT / GRIDHEIGHT - 1; i >= 0; i--)
                {
                      k = 0;
                    ////cout << endl;
                    for (  j = MAPWIDTH / GRIDWIDTH - 1; j >= 0; j--)
                    {
                        if (gridmap.map[j][i] != 125)
                        {
                            k++;
                        }
                    }
                    if (k >= 10)
                    {
                        y_error = 100 * i - half_map_wide;
                        y_error = current_pose->x - y_error;
                        break;
                    }
                }
            }
            for (  i = 0; i < MAPHEIGHT / GRIDHEIGHT; i++)
            {
                  k = 0;
                ////cout << endl;
                for (  j = 0; j < MAPWIDTH / GRIDWIDTH; j++)
                {
                    if (gridmap.map[i][j] != 125)
                    {
                        k++;
                    }
                }
                if (k >= 10)
                {
                    for (  j = 0; j < MAPWIDTH / GRIDWIDTH; j++)
                    {
                        gridmap.map[j][i] = 250;
                    }
                }
            }
        }
        stuck_x = false;
        stuck_y = false;
        stuck_right_run_step = 0;
        complete_flag = 1;
        break;
    }
    return complete_flag;
}

unsigned char  RightReadyLeakingSweep(POSE *current_pose, unsigned char obstacleSignal)
{
    int Yaw;
    unsigned char complete_flag = 0;
    Yaw = current_pose->orientation;
    switch (right_ready_leaking_sweep_status)
    {

    case 0:
        right_ready_leaking_sweep_status = RIGHT_LEAKING_SWEEP_COLLISION;
        break;
    case RIGHT_LEAKING_SWEEP_COLLISION:
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

            if (my_abs(Yaw / 100) > 90)
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
        if (my_abs(Yaw / 100) < 90)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            last_position_y = current_pose->y;
            right_ready_leaking_sweep_status = RIGHT_LEAKING_SWEEP_GOSTRAIGHT_MORE;
            break;
        }
        if (obstacleSignal != none_obstacle)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            right_ready_leaking_sweep_status = RIGHT_LEAKING_SWEEP_CLOCK_TARGER_YAW_LESS_ABS90_COLLISION;
        }
        linear_velocity = 0;
        angular_velocity = -turn_vel;
        break;
    case RIGHT_LEAKING_SWEEP_CLOCK_TARGER_YAW_LESS_ABS90_COLLISION:
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
            right_ready_leaking_sweep_status = RIGHT_LEAKING_SWEEP_CLOCK_TARGET_YAW_LESS_ABS90;
            break;
        }
        break;
    case RIGHT_LEAKING_SWEEP_GOSTRAIGHT_MORE:
        linear_velocity = long_stra_vel;
        angular_velocity = 0;
        if (obstacleSignal != none_obstacle)
        {
            right_ready_leaking_sweep_status = RIGHT_LEAKING_SWEEP_CLOCK_TARGER_YAW_LESS_ABS3;
            break;
        }
        if (my_abs(last_position_y - current_pose->y) > leakingsweep)
        {
            right_ready_leaking_sweep_status = RIGHT_LEAKING_SWEEP_CLOCK_TARGER_YAW_LESS_ABS3;
            break;
        }
        break;
    case RIGHT_LEAKING_SWEEP_CLOCK_TARGER_YAW_LESS_ABS3:
        if (my_abs(Yaw / 100) < 5)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            right_ready_leaking_sweep_status = RIGHT_LEAKING_SWEEP_COMPLETE;
            break;
        }
        if (obstacleSignal != none_obstacle)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            right_ready_leaking_sweep_status = RIGHT_LEAKING_SWEEP_CLOCK_TARGER_YAW_LESS_ABS3_COLLISION;
        }
        linear_velocity = 0;
        angular_velocity = -turn_vel;
        break;
    case RIGHT_LEAKING_SWEEP_CLOCK_TARGER_YAW_LESS_ABS3_COLLISION:
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
            right_ready_leaking_sweep_status = RIGHT_LEAKING_SWEEP_CLOCK_TARGER_YAW_LESS_ABS3;
            break;
        }
        break;
    case RIGHT_LEAKING_SWEEP_YAW_OTHER:
        right_ready_leaking_sweep_status = RIGHT_LEAKING_SWEEP_CCLOCK_TARGET_YAW_MORE_ABS90;
        break;
    case RIGHT_LEAKING_SWEEP_CCLOCK_TARGET_YAW_MORE_ABS90:
        if (my_abs(Yaw / 100) > 90)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            last_position_y = current_pose->y;
            right_ready_leaking_sweep_status = RIGHT_LEAKING_SWEEP_GOSTRAIGHT_OTHER;
            break;
        }
        if (obstacleSignal != none_obstacle)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            right_ready_leaking_sweep_status = RIGHT_LEAKING_SWEEP_CCLOCK_TARGET_YAW_MORE_ABS90_COLLISION;
        }
        linear_velocity = 0;
        angular_velocity = turn_vel;
        break;
    case RIGHT_LEAKING_SWEEP_CCLOCK_TARGET_YAW_MORE_ABS90_COLLISION:
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
            right_ready_leaking_sweep_status = RIGHT_LEAKING_SWEEP_CCLOCK_TARGET_YAW_MORE_ABS90;
            break;
        }
        break;
    case RIGHT_LEAKING_SWEEP_GOSTRAIGHT_OTHER:
        linear_velocity = long_stra_vel;
        angular_velocity = 0;
        if (obstacleSignal != none_obstacle)
        {
            right_ready_leaking_sweep_status = RIGHT_LEAKING_SWEEP_CCLOCK_TARGET_YAW_MORE_ABS178;
            break;
        }
        if (my_abs(last_position_y - current_pose->y) > leakingsweep)
        {
            right_ready_leaking_sweep_status = RIGHT_LEAKING_SWEEP_CCLOCK_TARGET_YAW_MORE_ABS178;
            break;
        }
        break;
    case RIGHT_LEAKING_SWEEP_CCLOCK_TARGET_YAW_MORE_ABS178:
        if (my_abs(Yaw / 100) > 175)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            right_ready_leaking_sweep_status = RIGHT_LEAKING_SWEEP_COMPLETE;
            break;
        }
        if (obstacleSignal != none_obstacle)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            right_ready_leaking_sweep_status = RIGHT_LEAKING_SWEEP_CCLOCK_TARGET_YAW_MOEE_ABS178_COLLISIION;
        }
        linear_velocity = 0;
        angular_velocity = turn_vel;
        break;
    case RIGHT_LEAKING_SWEEP_CCLOCK_TARGET_YAW_MOEE_ABS178_COLLISIION:
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
            right_ready_leaking_sweep_status = RIGHT_LEAKING_SWEEP_CCLOCK_TARGET_YAW_MORE_ABS178;
            break;
        }
        break;
    case RIGHT_LEAKING_SWEEP_COMPLETE:
        right_ready_leaking_sweep_status = 0;
        complete_flag = 1;
        break;
    }
    return complete_flag;
}



////////////////////////////////////////////////////////////////////////////////////////////////////////
//#################################################################################
//##########           LEFT             ###########################################	
//#################################################################################	
////////////////////////////////////////////////////////////////////////////////////////////////////////	

unsigned char  LeftRunningWorkStep(POSE *current_pose, unsigned char obstacleSignal)
{
    int Yaw;
    unsigned char complete_flag = 0;
    Yaw = current_pose->orientation;
    complete_flag = 0;
    switch (left_running_step_status)
    {
    case 0:
        left_running_step_status = GOSTR_LEFTRUN_STEP;
        break;
    case GOSTR_LEFTRUN_STEP:
        log_debug("gostraight left run step!");
        if (my_abs(Yaw / 100) > 90 && my_abs(Yaw / 100) < 175)
        {
            log_debug("backaward Corrected heading angle !");
            if (Yaw > 0)
            {
                linear_velocity = 0;
                angular_velocity = 5 * turn_vel / 6;
                break;
            }
            else
            {
                linear_velocity = 0;
                angular_velocity = -5 * turn_vel / 6;
                break;
            }
        }
        else if (my_abs(Yaw / 100) < 90 && my_abs(Yaw / 100) > 5)
        {
            log_debug("gostraight Corrected heading angle !");
            if (Yaw > 0)
            {
                linear_velocity = 0;
                angular_velocity = -5 * turn_vel / 6;
                break;
            }
            else
            {
                linear_velocity = 0;
                angular_velocity = 5 * turn_vel / 6;
                break;
            }
        }
        else if (my_abs(current_pose->x) > half_map_wide)
        {
            log_debug("current pose x arrived width max!");
            left_running_step_status = FORWARD_BOUNDARY_LEFTRUN_STEP;
            break;
        }
        else if (my_abs(current_pose->y) > half_map_wide)
        {
            log_debug("current pose y arrived width max!");
            complete_flag = 1;
            break;
        }
        else if (right_obstacle == obstacleSignal)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            if (my_abs(leakingsweep_x - current_pose->x) > 500 && my_abs(leakingsweep_y - current_pose->y) > 100)
            {
                leakingsweep_x = current_pose->x;
                leakingsweep_y = current_pose->y;
                leakingsweep = bsp_Left_ReturnExtreme_point(current_pose->x, current_pose->y, current_pose->orientation / 100, obstacleSignal);
            }
            if (0 != leakingsweep)
            {
                log_debug("right obstacle,ready goto LEAKING_SWEEP_LEFTRUN_STEP");
                left_running_step_status = LEAKING_SWEEP_LEFTRUN_STEP;
                break;
            }
            else
            {
                log_debug("right obstacle,ready goto COLLISION_RIGHT_LEFTRUN_STEP");
                collision_right_rightrun_step_status = 0;
                left_running_step_status = COLLISION_RIGHT_LEFTRUN_STEP;
                break;
            }
        }
        else if (front_obstacle == obstacleSignal)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            if (my_abs(leakingsweep_x - current_pose->x) > 500 && my_abs(leakingsweep_y - current_pose->y) > 100)
            {
                leakingsweep_x = current_pose->x;
                leakingsweep_y = current_pose->y;
                leakingsweep = bsp_Left_ReturnExtreme_point(current_pose->x, current_pose->y, current_pose->orientation / 100, obstacleSignal);
            }
            if (0 != leakingsweep)
            {
                log_debug("front obstacle,ready goto LEAKING_SWEEP_LEFTRUN_STEP");
                left_running_step_status = LEAKING_SWEEP_LEFTRUN_STEP;
                break;
            }
            else
            {
                log_debug("front obstacle,ready goto COLLISION_FRONT_LEFTRUN_STEP");
                collision_front_rightrun_step_status = 0;
                left_running_step_status = COLLISION_FRONT_LEFTRUN_STEP;
                break;
            }
        }
        else if (left_obstacle == obstacleSignal)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            if (my_abs(leakingsweep_x - current_pose->x) > 500 && my_abs(leakingsweep_y - current_pose->y) > 100)
            {
                leakingsweep_x = current_pose->x;
                leakingsweep_y = current_pose->y;
                leakingsweep = bsp_Left_ReturnExtreme_point(current_pose->x, current_pose->y, current_pose->orientation / 100, obstacleSignal);
            }
            if (0 != leakingsweep)
            {
                log_debug("left obstacle,ready goto LEAKING_SWEEP_LEFTRUN_STEP");
                left_running_step_status = LEAKING_SWEEP_LEFTRUN_STEP;
                break;
            }
            else
            {
                log_debug("left obstacle,ready goto COLLISION_LEFT_LEFTRUN_STEP");
                collision_left_rightrun_step_status = 0;
                left_running_step_status = COLLISION_LEFT_LEFTRUN_STEP;
                break;
            }
        }
        else
        {
            log_debug("go straight...");
            linear_velocity = long_stra_vel;
            angular_velocity = 0;
        }
        break;
    case FORWARD_BOUNDARY_LEFTRUN_STEP:
        log_debug("forward boundary leftrun step!");
        if (ForwardBoundaryLeftRunStep(current_pose, obstacleSignal))
        {
            left_running_step_status = GOSTR_LEFTRUN_STEP;
            right_forward_boundary_status = 0;
        }
        break;

    case COLLISION_RIGHT_LEFTRUN_STEP:
        log_debug("collsion right right run step!");
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
        log_debug("LEFTREVERSEWALKEDGE");
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
        log_debug("LEFTEDGEDILEMMA");
        if (LeftEdgeDilemma(current_pose, obstacleSignal))
        {
            left_running_step_status = GOSTR_LEFTRUN_STEP;
            right_edge_dilemma_status = 0;
        }
        break;
    case COLLISION_LEFT_LEFTRUN_STEP:
        log_debug("collsion left left run step!");
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
        log_debug("LEFTWALKEDGE");
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
        log_debug("collsion front left run step!");
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
        log_debug("leaking sweep left run step!");
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

unsigned char  ForwardBoundaryLeftRunStep(POSE *current_pose, unsigned char obstacleSignal)
{
    int Yaw;
    unsigned char complete_flag = 0;
    Yaw = current_pose->orientation;
    switch (right_forward_boundary_status)
    {
    case 0:
        if (my_abs(Yaw / 100) < 90)
        {
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
        if (my_abs(Yaw / 100) > 175)
        {
            right_forward_boundary_status = LEFT_FORWARDBOUNDARY_GOSTRAIGHT;
            break;
        }
        if (obstacleSignal != none_obstacle)
        {
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
        if (my_abs(Yaw / 100) < 5)
        {
            right_forward_boundary_status = LEFT_FORWARDBOUNDARY_GOSTRAIGHT;
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
        if ((my_abs(turn_start_x - current_pose->x) > 50 || my_abs(turn_start_y - current_pose->y) > 50) && my_abs(Yaw / 100) < 90 && current_pose->x > 0)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            turn_start_update = 0;
            right_forward_boundary_status = LEFT_FORWARDBOUNDARY_COMPLETE;
            break;
        }

        if ((my_abs(turn_start_x - current_pose->x) > 50 || my_abs(turn_start_y - current_pose->y) > 50) && my_abs(Yaw / 100) > 90 && current_pose->x < 0)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            turn_start_update = 0;
            right_forward_boundary_status = LEFT_FORWARDBOUNDARY_COMPLETE;
            break;
        }
        linear_velocity = long_stra_vel;
        angular_velocity = 0;
        if (obstacleSignal != none_obstacle)
        {
            right_forward_boundary_status = LEFT_FORWARDBOUNDARY_COMPLETE;
            break;
        }
        if (my_abs(current_pose->x) < half_map_wide)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            right_forward_boundary_status = FORWARDBOUNDARY_COMPLETE;
            break;
        }
        break;
    case LEFT_FORWARDBOUNDARY_COMPLETE:
        right_forward_boundary_status = 0;
        complete_flag = 1;
        break;
    }
    return complete_flag;
}

unsigned char  CollisionRightLeftRunStep(POSE *current_pose, unsigned char obstacleSignal)
{
    //cout << "Collision_Right_Left_RunStep" << endl;
    int Yaw;
    unsigned char complete_flag = 0;
    Yaw = current_pose->orientation;
    switch (collision_right_rightrun_step_status)
    {
    case 0:
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
            collision_right_rightrun_step_status = GOBACK_DISTANCE_CRLRS;
            turn_start_update = 0;
            break;
        }
        break;
    case GOBACK_DISTANCE_CRLRS:
        if (turn_start_update == 0)
        {
            turn_start_x = current_pose->x;
            turn_start_y = current_pose->y;
            turn_start_update = 1;
        }
        linear_velocity = -long_stra_vel;
        angular_velocity = 0;

        if (obstacleSignal != none_obstacle)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            if (my_abs(Yaw / 100) > 90)
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
        if (my_abs(turn_start_x - current_pose->x) > side_backward_distance || my_abs(turn_start_y - current_pose->y) > side_backward_distance)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            if (my_abs(Yaw / 100) > 90)
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
        collision_right_rightrun_step_status = TURN_CLOCK_TARGET_YAW_ABS153_LRUN_CR_DLYM;
        break;
    case TURN_CLOCK_TARGET_YAW_ABS153_LRUN_CR_DLYM:
        linear_velocity = 0;
        angular_velocity = -turn_vel;
        if (my_abs(Yaw / 100) < 153)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            collision_right_rightrun_step_status = GOSTR_YAW_EQUAL_ABS153_LRUN_CR_DLYM;
            break;
        }
        if (obstacleSignal != none_obstacle)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            collision_right_rightrun_step_status = TURN_CLOCK_TARGET_YAW_ABS153_COLLISION_LRUN_CR_DLYM;
            break;
        }
        break;
    case TURN_CLOCK_TARGET_YAW_ABS153_COLLISION_LRUN_CR_DLYM:
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
            collision_right_rightrun_step_status = TURN_CLOCK_TARGET_YAW_ABS153_LRUN_CR_DLYM;
            break;
        }
        break;
    case GOSTR_YAW_EQUAL_ABS153_LRUN_CR_DLYM:
        linear_velocity = long_stra_vel;
        angular_velocity = 0;
        if (obstacleSignal != none_obstacle)
        {
            collision_right_rightrun_step_status = TURN_CLOCK_TARGET_YAW_POS123_LRUN_CR_DLYM;
            break;
        }
        if (my_abs(last_position_y - current_pose->y) > lateral_move_distance / 3)
        {
            collision_right_rightrun_step_status = TURN_CLOCK_TARGET_YAW_POS123_LRUN_CR_DLYM;
            break;
        }
        break;
    case TURN_CLOCK_TARGET_YAW_POS123_LRUN_CR_DLYM:
        linear_velocity = 0;
        angular_velocity = -turn_vel;
        if (Yaw / 100 < 123)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            collision_right_rightrun_step_status = GOSTR_YAW_EQUAL_POS123_LRUN_CR_DLYM;
            break;
        }
        if (obstacleSignal != none_obstacle)
        {
            collision_right_rightrun_step_status = TURN_CLOCK_TARGET_YAW_POS123_COLLISION_LRUN_CR_DLYM;
            break;
        }
        break;
    case TURN_CLOCK_TARGET_YAW_POS123_COLLISION_LRUN_CR_DLYM:
        if (turn_start_update == 0)
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
        linear_velocity = long_stra_vel;
        angular_velocity = 0;
        if (obstacleSignal != none_obstacle)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            collision_right_rightrun_step_status = TURN_CLOCK_TARGET_YAW_POS98_LRUN_CR_DLYM;
            break;
        }
        if (my_abs(last_position_y - current_pose->y) > (2 * lateral_move_distance) / 3)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            collision_right_rightrun_step_status = TURN_CLOCK_TARGET_YAW_POS98_LRUN_CR_DLYM;
            break;
        }
        break;
    case TURN_CLOCK_TARGET_YAW_POS98_LRUN_CR_DLYM:
        linear_velocity = 0;
        angular_velocity = -turn_vel;
        if (Yaw / 100 < 98)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            collision_right_rightrun_step_status = GOSTR_YAW_EQUAL_POS98_LRUN_CR_DLYM;
            break;
        }
        if (obstacleSignal != none_obstacle)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            collision_right_rightrun_step_status = TURN_CLOCK_TARGET_YAW_POS98_COLLISION_LRUN_CR_DLYM;
            break;
        }
        break;
    case TURN_CLOCK_TARGET_YAW_POS98_COLLISION_LRUN_CR_DLYM:
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
            collision_right_rightrun_step_status = TURN_CLOCK_TARGET_YAW_POS98_LRUN_CR_DLYM;
            break;
        }
        break;
    case GOSTR_YAW_EQUAL_POS98_LRUN_CR_DLYM:
        linear_velocity = long_stra_vel;
        angular_velocity = 0;
        if (obstacleSignal != none_obstacle)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            collision_right_rightrun_step_status = LEFT_REVERSE_WALK_EDGE_LRUN_CR_DLYM;
            break;
        }
        if (my_abs(last_position_y - current_pose->y) > lateral_move_distance)
        {
            linear_velocity = 0;
            angular_velocity = 0;
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
        if (my_abs(Yaw / 100) < 3)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            collision_right_rightrun_step_status = COMPLETE_LRUN_CR_DLYM;
            break;
        }
        if (obstacleSignal != none_obstacle)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            collision_right_rightrun_step_status = TURN_CLOCK_TARGET_YAW_ABS3_COLLISION_LRUN_CR_DLYM;
            break;
        }
        break;
    case TURN_CLOCK_TARGET_YAW_ABS3_COLLISION_LRUN_CR_DLYM:
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
        if (my_abs(Yaw / 100) > 27)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            collision_right_rightrun_step_status = GOSTR_YAW_EQUAL_ABS27_LRUN_CR_DLYL;
            break;
        }
        if (obstacleSignal != none_obstacle)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            collision_right_rightrun_step_status = TURN_CCLCOK_TARGET_YAW_ABS27_COLLISION_LRUN_CR_DLYL;
            break;
        }
        break;
    case TURN_CCLCOK_TARGET_YAW_ABS27_COLLISION_LRUN_CR_DLYL:
        if (turn_start_update == 0)
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
        cnt_update += 1;
        linear_velocity = long_stra_vel;
        angular_velocity = 0;
        if (cnt_update > 4 && my_abs(last_position_xx - current_pose->x) < 20 && my_abs(last_position_y - current_pose->y) < 20)
        {
            cnt_update = 0;
            collision_right_rightrun_step_status = MORE_TRY_BREAK_BYPASS_LRUN_CR_DLYL;
            break;
        }
        collision_right_rightrun_step_status = GOSTR_BYPASS_LOOP_LRUN_CR_DLYL;
        break;
    case MORE_TRY_BREAK_BYPASS_LRUN_CR_DLYL:
        if (obstacleSignal != none_obstacle)
        {
            collision_right_rightrun_step_status = MORE_TRY_BREAK_BYPASS_COLLISION_LRUN_CR_DLYL;
            bow_continue = true;
            break;
        }
        if (my_abs(Yaw / 100) > 173)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            bow_continue = true;
            collision_right_rightrun_step_status = GOSTR_BYPASS_BOW_CONTINUE_EXIT_LRUN_CR_DLYL;
            break;
        }
        linear_velocity = 0;
        angular_velocity = turn_vel;
        break;
    case MORE_TRY_BREAK_BYPASS_COLLISION_LRUN_CR_DLYL:
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
            collision_right_rightrun_step_status = MORE_TRY_BREAK_BYPASS_LRUN_CR_DLYL;
            turn_start_update = 0;
            break;
        }
        break;
    case GOSTR_BYPASS_LOOP_LRUN_CR_DLYL:
        if (obstacleSignal == right_obstacle)
        {
            collision_right_rightrun_step_status = RIGHT_COLLISION_BYPASS_LRUN_CR_DLYL;
            break;
        }
        if (my_abs(last_position_x - current_pose->x) > lateral_move_distance / 3)
        {
            collision_right_rightrun_step_status = GOSTR_X_MORE_ONE_THIRD_LATERALDIS_BYPASS_LRUN_CR_DLYL;
            break;
        }
        if ((current_pose->y - last_position_y) > close_edge || obstacleSignal == front_obstacle || obstacleSignal == left_obstacle)
        {
            collision_right_rightrun_step_status = GOSTR_BYPASS_BOW_CONTINUE_LRUN_CR_DLYL;
            break;
        }
        if (last_position_y > current_pose->y + 1)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            old_bow_continue = true;
            collision_right_rightrun_step_status = GOSTR_BYPASS_BOW_CONTINUE_TARGET_YAW_ABS3_LRUN_CR_DLYL;
            break;
        }
        if (my_abs(current_pose->x) > half_map_wide)
        {
            collision_right_rightrun_step_status = COMPLETE_LRUN_CR_DLYL;
            break;
        }
        if (my_abs(Yaw / 100) > 90)
        {
            collision_right_rightrun_step_status = TURN_CCLOCK_TARGET_YAW_MORE_AB173_LRUN_CR_DLYL;
            break;
        }
        break;
    case GOSTR_BYPASS_BOW_CONTINUE_TARGET_YAW_ABS3_LRUN_CR_DLYL:
        linear_velocity = 0;
        angular_velocity = turn_vel;
        if (obstacleSignal != none_obstacle)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            collision_right_rightrun_step_status = GOSTR_BYPASS_BOW_CONTINUE_TARGET_YAW_ABS3_COLLISION_LRUN_CR_DLYL;
            break;
        }
        if (my_abs(Yaw / 100) < 5)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            collision_right_rightrun_step_status = GOSTR_BYPASS_BOW_CONTINUE_EXIT_LRUN_CR_DLYL;
            break;
        }
        break;
    case GOSTR_BYPASS_BOW_CONTINUE_TARGET_YAW_ABS3_COLLISION_LRUN_CR_DLYL:
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
            collision_right_rightrun_step_status = GOSTR_BYPASS_BOW_CONTINUE_TARGET_YAW_ABS3_LRUN_CR_DLYL;
            break;
        }
        break;

    case GOSTR_BYPASS_BOW_CONTINUE_LRUN_CR_DLYL:
        if (obstacleSignal != none_obstacle)
        {
            collision_right_rightrun_step_status = GOSTR_BYPASS_BOW_CONTINUE_COLLISION_LRUN_CR_DLYL;
            break;
        }
        if (my_abs(Yaw / 100) > 175)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            bow_continue = true;
            collision_right_rightrun_step_status = GOSTR_BYPASS_BOW_CONTINUE_EXIT_LRUN_CR_DLYL;
            break;
        }
        linear_velocity = 0;
        angular_velocity = turn_vel;
        break;

    case GOSTR_BYPASS_BOW_CONTINUE_COLLISION_LRUN_CR_DLYL:
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
            bow_continue = true;
            collision_right_rightrun_step_status = GOSTR_BYPASS_BOW_CONTINUE_LRUN_CR_DLYL;
            turn_start_update = 0;
            break;
        }
        break;

    case RIGHT_COLLISION_BYPASS_LRUN_CR_DLYL:
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
            temporary_yaw = Yaw / 100;
            if (my_abs(Yaw / 100) < 15)
                collision_right_rightrun_step_status = TURN_CCLOCK_TARGET_YAW_LESS_ABS15_LRUN_CR_DLYL;
            else
                collision_right_rightrun_step_status = TURN_CCLOCK_TARGET_YAW_MORE_ABS15_LRUN_CR_DLYL;
            break;
        }
        break;
    case TURN_CCLOCK_TARGET_YAW_LESS_ABS15_LRUN_CR_DLYL:
        if (obstacleSignal != none_obstacle)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            collision_right_rightrun_step_status = TURN_CCLOCK_TARGET_YAW_LESS_ABS15_COLLISION_LRUN_CR_DLYL;
            break;
        }
        if (my_abs(Yaw / 100) > 15)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            collision_right_rightrun_step_status = GOSTR_BYPASS_LRUN_CR_DLYL;
            break;
        }
        linear_velocity = 0;
        angular_velocity = turn_vel;
        break;
    case TURN_CCLOCK_TARGET_YAW_LESS_ABS15_COLLISION_LRUN_CR_DLYL:
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
            collision_right_rightrun_step_status = TURN_CCLOCK_TARGET_YAW_LESS_ABS15_LRUN_CR_DLYL;
            break;
        }
        break;

    case TURN_CCLOCK_TARGET_YAW_MORE_ABS15_LRUN_CR_DLYL:
        if (obstacleSignal != none_obstacle)
        {
            collision_right_rightrun_step_status = TURN_CCLOCK_TARGET_YAW_MORE_ABS15_COLLISION_LRUN_CR_DLYL;
            break;
        }
        if (my_abs(Yaw / 100 - temporary_yaw) > 15)
        {
            collision_right_rightrun_step_status = GOSTR_BYPASS_LRUN_CR_DLYL;
            break;
        }
        linear_velocity = 0;
        angular_velocity = turn_vel;
        break;
    case TURN_CCLOCK_TARGET_YAW_MORE_ABS15_COLLISION_LRUN_CR_DLYL:
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
            collision_right_rightrun_step_status = TURN_CCLOCK_TARGET_YAW_MORE_ABS15_LRUN_CR_DLYL;
            break;
        }
        break;

    case GOSTR_X_MORE_ONE_THIRD_LATERALDIS_BYPASS_LRUN_CR_DLYL:
        if (my_abs(Yaw / 100) <= 30)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            temporary_yaw = Yaw / 100;
            collision_right_rightrun_step_status = TURN_CLOCK_TARGET_YAW_ABS30_LRUN_CR_DLYL;
            break;
        }
        else if (my_abs(Yaw / 100) > 30 && Yaw / 100 > 0)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            temporary_yaw = Yaw / 100;
            collision_right_rightrun_step_status = TURN_CLOCK_YAW_ADD_ABS30_LRUN_CR_DLYL;
            break;
        }
        else
        {
            collision_right_rightrun_step_status = GOSTR_BYPASS_LRUN_CR_DLYL;
            break;
        }
    case TURN_CLOCK_TARGET_YAW_ABS30_LRUN_CR_DLYL:
        if (my_abs(Yaw / 100) > 30)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            collision_right_rightrun_step_status = GOSTR_BYPASS_LRUN_CR_DLYL;
            break;
        }
        if (obstacleSignal != none_obstacle)
        {
            collision_right_rightrun_step_status = TURN_CLOCK_TARGET_YAW_ABS30_COLLISION_LRUN_CR_DLYL;
        }
        linear_velocity = 0;
        angular_velocity = -turn_vel;
        break;
    case TURN_CLOCK_TARGET_YAW_ABS30_COLLISION_LRUN_CR_DLYL:
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
            collision_right_rightrun_step_status = TURN_CLOCK_TARGET_YAW_ABS30_LRUN_CR_DLYL;
            turn_start_update = 0;
            break;
        }
        break;
    case TURN_CLOCK_YAW_ADD_ABS30_LRUN_CR_DLYL:
        if (my_abs(Yaw / 100 - temporary_yaw) > 30)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            collision_right_rightrun_step_status = GOSTR_BYPASS_LRUN_CR_DLYL;
            break;
        }
        if (obstacleSignal != none_obstacle)
        {
            collision_right_rightrun_step_status = TURN_CLOCK_YAW_ADD_ABS30_COLLISION_LRUN_CR_DLYL;
            break;
        }
        linear_velocity = 0;
        angular_velocity = -turn_vel;
        break;
    case TURN_CLOCK_YAW_ADD_ABS30_COLLISION_LRUN_CR_DLYL:
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
            collision_right_rightrun_step_status = TURN_CLOCK_YAW_ADD_ABS30_LRUN_CR_DLYL;
            turn_start_update = 0;
            break;
        }
        break;
    case TURN_CCLOCK_TARGET_YAW_MORE_AB173_LRUN_CR_DLYL:
        if (obstacleSignal != none_obstacle)
        {
            collision_right_rightrun_step_status = TURN_CCLOCK_TARGET_YAW_MORE_AB173_COLLISION_LRUN_CR_DLYL;
            break;
        }
        if (my_abs(Yaw / 100) > 173)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            collision_right_rightrun_step_status = COMPLETE_LRUN_CR_DLYL;
            break;
        }
        linear_velocity = 0;
        angular_velocity = turn_vel;

        break;
    case TURN_CCLOCK_TARGET_YAW_MORE_AB173_COLLISION_LRUN_CR_DLYL:
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

unsigned char  CollisionLeftLeftRunStep(POSE *current_pose, unsigned char obstacleSignal)
{
    int Yaw;
    unsigned char complete_flag = 0;
    Yaw = current_pose->orientation;
    switch (collision_left_rightrun_step_status)
    {
    case 0:
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
            collision_left_rightrun_step_status = GOBACK_DISTANCE_CLLRS;
            turn_start_update = 0;
            break;
        }
        break;
    case GOBACK_DISTANCE_CLLRS:
        if (turn_start_update == 0)
        {
            turn_start_x = current_pose->x;
            turn_start_y = current_pose->y;
            turn_start_update = 1;
        }
        linear_velocity = -long_stra_vel;
        angular_velocity = 0;

        if (obstacleSignal != none_obstacle)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            if (my_abs(Yaw / 100) > 90)
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
        if (my_abs(turn_start_x - current_pose->x) > side_backward_distance || my_abs(turn_start_y - current_pose->y) > side_backward_distance)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            if (my_abs(Yaw / 100) > 90)
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
    case DIR_LEFT_YAW_MORE_ABS90_CLLRS:
        linear_velocity = 0;
        angular_velocity = 0;
        collision_left_rightrun_step_status = TURN_CLOCK_TARGET_YAW_ABS150_LRUN_CL_DLYM;
        break;
    case TURN_CLOCK_TARGET_YAW_ABS150_LRUN_CL_DLYM:
        linear_velocity = 0;
        angular_velocity = -turn_vel;
        if (my_abs(Yaw / 100) < 150)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            collision_left_rightrun_step_status = GOSTR_YAW_EQUAL_ABS150_LRUN_CL_DLYM;
            break;
        }
        if (obstacleSignal != none_obstacle)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            collision_left_rightrun_step_status = TURN_CLOCK_TARGET_YAW_ABS150_COLLISION_LRUN_CL_DLYM;
            break;
        }
        break;
    case TURN_CLOCK_TARGET_YAW_ABS150_COLLISION_LRUN_CL_DLYM:
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
            collision_left_rightrun_step_status = TURN_CLOCK_TARGET_YAW_ABS150_LRUN_CL_DLYM;
            break;
        }
        break;
    case GOSTR_YAW_EQUAL_ABS150_LRUN_CL_DLYM:
        cnt_update = 0;
        last_position_y = current_pose->y;
        last_position_xx = current_pose->x;
        collision_left_rightrun_step_status = GOSTR_BYPASS_LRUN_CL_DLYM;
        break;
    case GOSTR_BYPASS_LRUN_CL_DLYM:
        last_position_x = current_pose->x;
        cnt_update += 1;
        linear_velocity = long_stra_vel;
        angular_velocity = 0;
        if (cnt_update > 4 && my_abs(last_position_xx - current_pose->x) < 20 && my_abs(last_position_y - current_pose->y) < 20)
        {
            cnt_update = 0;
            collision_left_rightrun_step_status = MORE_TRY_BREAK_BYPASS_LRUN_CL_DLYM;
            break;
        }
        collision_left_rightrun_step_status = GOSTR_BYPASS_LOOP_LRUN_CL_DLYM;
        break;
    case GOSTR_BYPASS_LOOP_LRUN_CL_DLYM:
        if (obstacleSignal == left_obstacle)
        {
            collision_left_rightrun_step_status = LEFT_COLLISION_BYPASS_LRUN_CL_DLYM;
            break;
        }
        if (my_abs(last_position_x - current_pose->x) > lateral_move_distance / 3)
        {
            collision_left_rightrun_step_status = GOSTR_X_MORE_ONE_THIRD_LATERALDIS_BYPASS_LRUN_CL_DLYM;
            break;
        }
        if (my_abs(last_position_y - current_pose->y) > close_edge || obstacleSignal == front_obstacle || obstacleSignal == right_obstacle)
        {
            collision_left_rightrun_step_status = GOSTR_BYPASS_BOW_CONTINUE_LRUN_CL_DLYM;
            break;
        }
        if (last_position_y > current_pose->y + 1)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            collision_left_rightrun_step_status = GOSTR_BYPASS_OLD_BOW_CONTINUE_LRUN_CL_DLYM;
            break;
        }
        if (my_abs(current_pose->x) > half_map_wide)
        {
            collision_left_rightrun_step_status = COMPLETE_LRUN_CL_DLYM;
            break;
        }
        if (my_abs(Yaw / 100) < 90)
        {
            collision_left_rightrun_step_status = TURN_CCLOCK_TARGET_YAW_LESS_ABS3_LRUN_CL_DLYM;
            break;
        }
        break;

    case GOSTR_BYPASS_OLD_BOW_CONTINUE_LRUN_CL_DLYM:
        if (obstacleSignal != none_obstacle)
        {
            collision_left_rightrun_step_status = GOSTR_BYPASS_OLD_BOW_CONTINUE_COLLISION_LRUN_CL_DLYM;
            break;
        }
        if (my_abs(Yaw / 100) > 175)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            bow_continue = true;
            collision_left_rightrun_step_status = COMPLETE_LRUN_CL_DLYM;
            break;
        }
        linear_velocity = 0;
        angular_velocity = -turn_vel;
        break;
    case GOSTR_BYPASS_OLD_BOW_CONTINUE_COLLISION_LRUN_CL_DLYM:
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
            bow_continue = true;
            collision_left_rightrun_step_status = GOSTR_BYPASS_OLD_BOW_CONTINUE_LRUN_CL_DLYM;
            turn_start_update = 0;
            break;
        }
        break;
    case TURN_CCLOCK_TARGET_YAW_LESS_ABS3_LRUN_CL_DLYM:
        if (obstacleSignal != none_obstacle)
        {
            collision_left_rightrun_step_status = TURN_CCLOCK_TARGET_YAW_LESS_ABS3_COLLISION_LRUN_CL_DLYM;
            break;
        }
        if (my_abs(Yaw / 100) < 5)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            bow_continue = true;
            collision_left_rightrun_step_status = COMPLETE_LRUN_CL_DLYM;
            break;
        }
        linear_velocity = 0;
        angular_velocity = -turn_vel;
        break;
    case TURN_CCLOCK_TARGET_YAW_LESS_ABS3_COLLISION_LRUN_CL_DLYM:
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
            bow_continue = true;
            collision_left_rightrun_step_status = TURN_CCLOCK_TARGET_YAW_LESS_ABS3_LRUN_CL_DLYM;
            turn_start_update = 0;
            break;
        }
        break;
    case GOSTR_BYPASS_BOW_CONTINUE_LRUN_CL_DLYM:
        if (obstacleSignal != none_obstacle)
        {
            collision_left_rightrun_step_status = GOSTR_BYPASS_BOW_CONTINUE_COLLISION_LRUN_CL_DLYM;
            break;
        }
        if (my_abs(Yaw / 100) < 5)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            bow_continue = true;
            collision_left_rightrun_step_status = GOSTR_BYPASS_BOW_CONTINUE_EXIT_LRUN_CL_DLYM;
            break;
        }
        linear_velocity = 0;
        angular_velocity = -turn_vel;
        break;
    case GOSTR_BYPASS_BOW_CONTINUE_COLLISION_LRUN_CL_DLYM:
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
            bow_continue = true;
            collision_left_rightrun_step_status = GOSTR_BYPASS_BOW_CONTINUE_LRUN_CL_DLYM;
            turn_start_update = 0;
            break;
        }
        break;
    case GOSTR_BYPASS_BOW_CONTINUE_EXIT_LRUN_CL_DLYM:
        collision_left_rightrun_step_status = COMPLETE_LRUN_CL_DLYM;
        break;
    case LEFT_COLLISION_BYPASS_LRUN_CL_DLYM:
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
            temporary_yaw = Yaw / 100;
            if (my_abs(Yaw / 100) > 150)
                collision_left_rightrun_step_status = TURN_CLCOK_TARGET_YAW_MORE_ABS150_LRUN_CL_DLYM;
            else
                collision_left_rightrun_step_status = TURN_CLCOK_TARGET_YAW_LESS_ABS150_LRUN_CL_DLYM;
            break;
        }
        break;
    case TURN_CLCOK_TARGET_YAW_MORE_ABS150_LRUN_CL_DLYM:
        if (obstacleSignal != none_obstacle)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            collision_left_rightrun_step_status = TURN_CLCOK_TARGET_YAW_MORE_ABS150_COLLISION_LRUN_CL_DLYM;
            break;
        }
        if (my_abs(Yaw / 100) < 150)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            collision_left_rightrun_step_status = GOSTR_BYPASS_LRUN_CL_DLYM;
            break;
        }
        linear_velocity = 0;
        angular_velocity = -turn_vel;
        break;
    case TURN_CLCOK_TARGET_YAW_MORE_ABS150_COLLISION_LRUN_CL_DLYM:
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
            collision_left_rightrun_step_status = TURN_CLCOK_TARGET_YAW_MORE_ABS150_LRUN_CL_DLYM;
            break;
        }
        break;

    case TURN_CLCOK_TARGET_YAW_LESS_ABS150_LRUN_CL_DLYM:
        if (obstacleSignal != none_obstacle)
        {
            collision_left_rightrun_step_status = TURN_CLCOK_TARGET_YAW_LESS_ABS150_COLLISION_LRUN_CL_DLYM;
            break;
        }
        if (my_abs(Yaw / 100 - temporary_yaw) > 15)
        {
            collision_left_rightrun_step_status = GOSTR_BYPASS_LRUN_CL_DLYM;
            break;
        }
        linear_velocity = 0;
        angular_velocity = -turn_vel;
        break;

    case TURN_CLCOK_TARGET_YAW_LESS_ABS150_COLLISION_LRUN_CL_DLYM:
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
            collision_left_rightrun_step_status = TURN_CLCOK_TARGET_YAW_LESS_ABS150_LRUN_CL_DLYM;
            break;
        }
        break;
    case GOSTR_X_MORE_ONE_THIRD_LATERALDIS_BYPASS_LRUN_CL_DLYM:
        if (my_abs(Yaw / 100) > 150 || Yaw / 100 > 0)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            if (Yaw / 100 > 0 && my_abs(Yaw / 100) > 150)
            {
                turn_start_update = 1;
            }
            temporary_yaw = Yaw / 100;
            collision_left_rightrun_step_status = TURN_CCLOCK_YAW_ADD_ABS30_LRUN_CL_DLYM;
            break;
        }
        collision_left_rightrun_step_status = GOSTR_BYPASS_LRUN_CL_DLYM;
        break;
    case TURN_CCLOCK_YAW_ADD_ABS30_LRUN_CL_DLYM:
        if (turn_start_update == 1)
        {
            if (my_abs(Yaw / 100) < 150)
            {
                turn_start_update = 0;
                linear_velocity = 0;
                angular_velocity = 0;
                collision_left_rightrun_step_status = GOSTR_BYPASS_LRUN_CL_DLYM;
                break;
            }
        }
        else
        {
            if (my_abs(Yaw / 100 - temporary_yaw) > 30)
            {
                linear_velocity = 0;
                angular_velocity = 0;
                collision_left_rightrun_step_status = GOSTR_BYPASS_LRUN_CL_DLYM;
                break;
            }
        }
        if (obstacleSignal != none_obstacle)
        {
            collision_left_rightrun_step_status = TURN_CCLOCK_YAW_ADD_ABS30_COLLISION_LRUN_CL_DLYM;
        }
        linear_velocity = 0;
        angular_velocity = turn_vel;
        break;
    case TURN_CCLOCK_YAW_ADD_ABS30_COLLISION_LRUN_CL_DLYM:
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
            collision_left_rightrun_step_status = TURN_CCLOCK_YAW_ADD_ABS30_LRUN_CL_DLYM;
            turn_start_update = 0;
            break;
        }
        break;
    case MORE_TRY_BREAK_BYPASS_LRUN_CL_DLYM:
        if (obstacleSignal != none_obstacle)
        {
            collision_left_rightrun_step_status = MORE_TRY_BREAK_BYPASS_COLLISION_LRUN_CL_DLYM;
            bow_continue = true;
            break;
        }
        if (my_abs(Yaw / 100) < 5)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            bow_continue = true;
            collision_left_rightrun_step_status = GOSTR_BYPASS_BOW_CONTINUE_EXIT_LRUN_CL_DLYM;
            break;
        }
        linear_velocity = 0;
        angular_velocity = -turn_vel;
        break;
    case MORE_TRY_BREAK_BYPASS_COLLISION_LRUN_CL_DLYM:
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
            collision_left_rightrun_step_status = MORE_TRY_BREAK_BYPASS_LRUN_CL_DLYM;
            turn_start_update = 0;
            break;
        }
        break;

    case COMPLETE_LRUN_CL_DLYM:
        collision_left_rightrun_step_status = 0;
        complete_flag = 1;
        break;
    case DIR_LEFT_YAW_LESS_ABS90_CLLRS:
        linear_velocity = 0;
        angular_velocity = 0;
        last_position_y = current_pose->y;
        collision_left_rightrun_step_status = TURN_CCLOCK_TARGET_YAW_ABS27_LRUN_CL_DLYL;
        break;
    case TURN_CCLOCK_TARGET_YAW_ABS27_LRUN_CL_DLYL:
        linear_velocity = 0;
        angular_velocity = turn_vel;
        if (my_abs(Yaw / 100) > 27)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            collision_left_rightrun_step_status = GOSTR_YAW_EQUAL_ABS27_LRUN_CL_DLYL;
            break;
        }
        if (obstacleSignal != none_obstacle)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            collision_left_rightrun_step_status = TURN_CCLOCK_TAEGET_YAW_ABS27_COLLISION_LRUN_CL_DLYL;
            break;
        }
        break;
    case TURN_CCLOCK_TAEGET_YAW_ABS27_COLLISION_LRUN_CL_DLYL:
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
            collision_left_rightrun_step_status = TURN_CCLOCK_TARGET_YAW_ABS27_LRUN_CL_DLYL;
            break;
        }
        break;
    case GOSTR_YAW_EQUAL_ABS27_LRUN_CL_DLYL:
        linear_velocity = long_stra_vel;
        angular_velocity = 0;
        if (obstacleSignal != none_obstacle)
        {
            collision_left_rightrun_step_status = TURN_CCLOCK_TARGET_YAW_ABS57_LRUN_CL_DLYL;
            break;
        }
        if (my_abs(last_position_y - current_pose->y) > lateral_move_distance / 3)
        {
            collision_left_rightrun_step_status = TURN_CCLOCK_TARGET_YAW_ABS57_LRUN_CL_DLYL;
            break;
        }
        break;
    case TURN_CCLOCK_TARGET_YAW_ABS57_LRUN_CL_DLYL:
        linear_velocity = 0;
        angular_velocity = turn_vel;
        if (Yaw / 100 > 57)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            collision_left_rightrun_step_status = GOSTR_YAW_EQUAL_ABS57_LRUN_CL_DLYL;
            break;
        }
        if (obstacleSignal != none_obstacle)
        {
            collision_left_rightrun_step_status = TURN_CCLOCK_TARGET_YAW_ABS57_COLLISION_LRUN_CL_DLYL;
            break;
        }
        break;
    case TURN_CCLOCK_TARGET_YAW_ABS57_COLLISION_LRUN_CL_DLYL:
        if (turn_start_update == 0)
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
    case GOSTR_YAW_EQUAL_ABS57_LRUN_CL_DLYL:
        linear_velocity = long_stra_vel;
        angular_velocity = 0;
        if (obstacleSignal == front_obstacle || obstacleSignal == left_obstacle || obstacleSignal == right_obstacle)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            collision_left_rightrun_step_status = TURN_CCLOCK_TARGET_YAW_ABS87_LRUN_CL_DLYL;
            break;
        }
        if (my_abs(last_position_y - current_pose->y) > (2 * lateral_move_distance) / 3)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            collision_left_rightrun_step_status = TURN_CCLOCK_TARGET_YAW_ABS87_LRUN_CL_DLYL;
            break;
        }
        break;
    case TURN_CCLOCK_TARGET_YAW_ABS87_LRUN_CL_DLYL:
        linear_velocity = 0;
        angular_velocity = turn_vel;
        if (Yaw / 100 > 87)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            collision_left_rightrun_step_status = GOSTR_YAW_EQUAL_ABS87_LRUN_CL_DLYL;
            break;
        }
        if (obstacleSignal != none_obstacle)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            collision_left_rightrun_step_status = TURN_CCLOCK_TARGET_YAW_ABS87_COLLISION_LRUN_CL_DLYL;
            break;
        }
        break;
    case TURN_CCLOCK_TARGET_YAW_ABS87_COLLISION_LRUN_CL_DLYL:
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
            collision_left_rightrun_step_status = TURN_CCLOCK_TARGET_YAW_ABS87_LRUN_CL_DLYL;
            break;
        }
        break;
    case GOSTR_YAW_EQUAL_ABS87_LRUN_CL_DLYL:
        linear_velocity = long_stra_vel;
        angular_velocity = 0;
        if (obstacleSignal != none_obstacle)
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
            collision_left_rightrun_step_status = TURN_CCLOCK_TARGET_YAW_ABS173_LRUN_CL_DLYL;
            break;
        }
        break;
    case LEFT_WALK_EDGE_LRUN_CL_DLYL:
        collision_left_rightrun_step_status = 0;
        complete_flag = 2;
        break;
    case TURN_CCLOCK_TARGET_YAW_ABS173_LRUN_CL_DLYL:
        linear_velocity = 0;
        angular_velocity = turn_vel;
        if (my_abs(Yaw / 100) > 175)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            collision_left_rightrun_step_status = COMPLETE_LRUN_CL_DLYL;
            break;
        }
        if (obstacleSignal != none_obstacle)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            collision_left_rightrun_step_status = TURN_CCLOCK_TARGET_YAW_ABS173_COLLISION_LRUN_CL_DLYL;
            break;
        }
        break;
    case TURN_CCLOCK_TARGET_YAW_ABS173_COLLISION_LRUN_CL_DLYL:
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
            collision_left_rightrun_step_status = TURN_CCLOCK_TARGET_YAW_ABS173_LRUN_CL_DLYL;
            break;
        }
        break;
    case COMPLETE_LRUN_CL_DLYL:
        collision_left_rightrun_step_status = 0;
        complete_flag = 1;
        break;
    }
    return complete_flag;
}

unsigned char  CollisionFrontLeftRunStep(POSE *current_pose, unsigned char obstacleSignal)
{
    int Yaw;
    unsigned char complete_flag = 0;
    Yaw = current_pose->orientation;
    switch (collision_front_rightrun_step_status)
    {
    case 0:
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

        if (obstacleSignal != none_obstacle)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            if (my_abs(Yaw / 100) < 90)
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
        if (my_abs(turn_start_x - current_pose->x) > front_backward_distance)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            if (my_abs(Yaw / 100) < 90)
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
        if ((Yaw / 100) > 60)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            collision_front_rightrun_step_status = GOSTR_YAW_ABS60_LRUN_CF_DLYL;
            break;
        }
        if (obstacleSignal != none_obstacle)
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
        if (obstacleSignal != none_obstacle)
        {
            collision_front_rightrun_step_status = TURN_CCLOCK_TARGET_YAW_ABS82_LRUN_CF_DLYL;
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
        if (Yaw / 100 > 82)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            collision_front_rightrun_step_status = GOSTR_YAW_ABS82_LRUN_CF_DLYL;
            break;
        }
        if (obstacleSignal != none_obstacle)
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
        if (obstacleSignal != none_obstacle)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            collision_front_rightrun_step_status = LEFT_WALK_EDGE_LRUN_CF_DLYL;
            break;
        }
        if (my_abs(last_position_y - current_pose->y) > (lateral_move_distance))
        {
            linear_velocity = 0;
            angular_velocity = 0;
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
        if (my_abs(Yaw / 100) > 175)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            collision_front_rightrun_step_status = COMPLETE_LRUN_CF_DLYL;
            break;
        }
        if (obstacleSignal != none_obstacle)
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
        if (my_abs(Yaw / 100) < 120)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            collision_front_rightrun_step_status = GOSTR_YAW_EQUAL_ABS120_LRUN_CF_CF_DLYM;
            break;
        }
        if (obstacleSignal != none_obstacle)
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
        if (obstacleSignal != none_obstacle)
        {
            collision_front_rightrun_step_status = TURN_CLOCK_TARGET_YAW_ABS98_LRUN_CF_DLYM;
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
        if (my_abs(Yaw / 100) < 93)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            collision_front_rightrun_step_status = GOSTR_YAW_EQUAL_ABS98_LRUN_CF_DLYM;
            break;
        }
        if (obstacleSignal != none_obstacle)
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
        if (obstacleSignal != none_obstacle)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            collision_front_rightrun_step_status = LEFT_REVERSE_WALK_EDGE_LRUN_CF_DLYM;
            break;
        }
        if (my_abs(last_position_y - current_pose->y) > (lateral_move_distance))
        {
            linear_velocity = 0;
            angular_velocity = 0;
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
        if (my_abs(Yaw / 100) < 5)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            collision_front_rightrun_step_status = COMPLETE_LRUN_CF_DLYM;
            break;
        }
        if (obstacleSignal != none_obstacle)
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

unsigned char  LeftWalkEdge(POSE *current_pose, unsigned char obstacleSignal)
{
    int Yaw;
    unsigned char complete_flag = 0;
    Yaw = current_pose->orientation;
    switch (right_walk_edge_status)
    {
    case 0:
        right_walk_edge_status = LEFT_GOBACK_WALK_EDGE;
        break;
    case LEFT_GOBACK_WALK_EDGE:
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
            right_walk_edge_status = LEFT_EDGE_TURN_CCLOCK_TARGET_YAW_MORE_ABS117_WE;
            break;
        }
        break;
    case LEFT_EDGE_TURN_CCLOCK_TARGET_YAW_MORE_ABS117_WE:
        linear_velocity = 0;
        angular_velocity = turn_vel;
        if (my_abs(Yaw / 100) > 117)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            right_walk_edge_status = LEFT_EDGE_READY_GOSTR_BYPASS_WE;
            break;
        }
        if (obstacleSignal != none_obstacle)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            right_walk_edge_status = LEFT_EDGE_TURN_CCLOCK_TARGET_YAW_MORE_ABS117_COLLISION_WE;
            break;
        }
        break;
    case LEFT_EDGE_TURN_CCLOCK_TARGET_YAW_MORE_ABS117_COLLISION_WE:
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
            right_walk_edge_status = LEFT_EDGE_TURN_CCLOCK_TARGET_YAW_MORE_ABS117_WE;
            break;
        }
        break;
    case LEFT_EDGE_READY_GOSTR_BYPASS_WE:
        edge_length_start = current_pose->x + half_map_wide;
        returnorigin = false;
        b_last_position_yy = false;
        last_position_y = current_pose->y;
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
 
    case LEFT_EDGE_GOSTR_BYPASS_WE:
        linear_velocity = long_stra_vel;
        angular_velocity = 0;
        if (obstacleSignal != none_obstacle)
        {
            right_walk_edge_status = LEFT_EDGE_COLLISION_BYPASS_WE;
            break;
        }
        if (my_abs(last_position_x - current_pose->x) > lateral_move_distance)
        {
            right_walk_edge_status = LEFT_EDGE_GOSTR_X_MORE_LATERALDIS_BYPASS_WE;
            break;
        }
        if ((current_pose->y - last_position_y) > temporary_close_edge)
        {
            right_walk_edge_status = LEFT_EDGE_BOW_CONTINUE_WE;
            break;
        }
        if (b_last_position_yy == true && my_abs(last_position_yy - current_pose->y) > 3 * lateral_move_distance)
        {
            right_walk_edge_status = LEFT_EDGE_BOW_CONTINUE_WE;
            break;
        }
        if (my_abs(Yaw / 100) < 120 && (Yaw / 100 < 0) && (b_last_position_yy == false))
        {
            last_position_yy = current_pose->y;
            b_last_position_yy = true;
        }
        break;

    case LEFT_EDGE_REBACK_GOSTR_BYPASS_CHECK_WE:
        if (my_abs(Yaw / 100) < 135 && (Yaw / 100 < 0))
        {
            right_walk_edge_status = LEFT_EDGE_TARGET_YAW_LESS_ABS105_LESS_0_BYPASS_WE;
            break;
        }
        if ((my_abs(current_pose->x + half_map_wide - edge_length_start) > Edge_length() / 2) && (my_abs(current_pose->x + half_map_wide - edge_length_start) > 500))
        {
            right_walk_edge_status = LEFT_EDGE_DELTA_X_MORE_ONE_FOURTH_CLEANED_MAP_WIDTH_WE;
            break;
        }
        right_walk_edge_status = LEFT_EDGE_GOSTR_BYPASS_WE_X;
        break;
    case LEFT_EDGE_COLLISION_BYPASS_WE:
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
            temporary_yaw = Yaw / 100;
            right_walk_edge_status = LEFT_EDGE_TURN_CCLOCK_YAW_ADD_ABS15_WE;
            break;
        }
        break;
    case LEFT_EDGE_TURN_CCLOCK_YAW_ADD_ABS15_WE:
        linear_velocity = 0;
        angular_velocity = turn_vel;
        if (my_abs(Yaw / 100 - temporary_yaw) > 15)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            right_walk_edge_status = LEFT_EDGE_REBACK_GOSTR_BYPASS_CHECK_WE;
            break;
        }
        if (obstacleSignal != none_obstacle)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            right_walk_edge_status = LEFT_EDGE_TURN_CCLOCK_YAW_ADD_ABS15_COLLISION_WE;
            break;
        }
        break;
    case LEFT_EDGE_TURN_CCLOCK_YAW_ADD_ABS15_COLLISION_WE:
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
            right_walk_edge_status = LEFT_EDGE_TURN_CCLOCK_YAW_ADD_ABS15_WE;
            break;
        }
        break;
    case LEFT_EDGE_TARGET_YAW_LESS_ABS105_LESS_0_BYPASS_WE:
        linear_velocity = 0;
        angular_velocity = 0;
        if (!detection_close_edge)
        {
            detection_close_edge = true;
        }
        else
        {
            detection_close_edge = false;
            if (detection_close == true)
            {
                right_walk_edge_status = LEFT_EDGE_TURN_CCLOCK_TARGET_YAW_LESS_ABS3_WE;
            }
            else
            {
                right_walk_edge_status = LEFT_EDGE_RETURN_ORIGIN_WE;
            }
        }
        break;
    case LEFT_EDGE_DELTA_X_MORE_ONE_FOURTH_CLEANED_MAP_WIDTH_WE:
        linear_velocity = 0;
        angular_velocity = 0;
        if (!detection_close_edge)
        {
            detection_close_edge = true;
        }
        else
        {
            detection_close_edge = false;
            if (detection_close == true)
            {
                temporary_yaw = Yaw / 100;
                right_walk_edge_status = CLOSE_EDGE_MAP_LEFT_WALK_TURN_CCLOCK_YAW_ADD_ABS15_WE;
            }
            else
            {
                right_walk_edge_status = LEFT_EDGE_RETURN_ORIGIN_WE;
            }
        }
        break;

    case CLOSE_EDGE_MAP_LEFT_WALK_TURN_CCLOCK_YAW_ADD_ABS15_WE:
        linear_velocity = 0;
        angular_velocity = turn_vel;
        if (my_abs(Yaw / 100 - temporary_yaw) > 15)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            last_position_x = current_pose->x;
            right_walk_edge_status = LEFT_EDGE_GOSTR_BYPASS_WE;
            break;
        }
        if (obstacleSignal != none_obstacle)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            right_walk_edge_status = COLLISION_CLOSE_EDGE_MAP_LEFT_WALK_TURN_CCLOCK_YAW_ADD_ABS15_WE;
            break;
        }
        break;
    case COLLISION_CLOSE_EDGE_MAP_LEFT_WALK_TURN_CCLOCK_YAW_ADD_ABS15_WE:
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
            right_walk_edge_status = CLOSE_EDGE_MAP_LEFT_WALK_TURN_CCLOCK_YAW_ADD_ABS15_WE;
            break;
        }
        break;

    case LEFT_EDGE_TURN_CCLOCK_TARGET_YAW_LESS_ABS3_WE:
        if (my_abs(Yaw / 100) < 5)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            right_walk_edge_status = LEFT_EDGE_LEFT_EDGE_DILEMMA_WE;
            break;
        }

        if (obstacleSignal != none_obstacle)
        {
            right_walk_edge_status = LEFT_EDGE_TURN_CCLOCK_TARGET_YAW_LESS_ABS3_COLLISION_WE;
            break;
        }
        linear_velocity = 0;
        angular_velocity = turn_vel;
        break;

    case LEFT_EDGE_TURN_CCLOCK_TARGET_YAW_LESS_ABS3_COLLISION_WE:
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
            right_walk_edge_status = LEFT_EDGE_TURN_CCLOCK_TARGET_YAW_LESS_ABS3_WE;
        }
        break;

    case LEFT_EDGE_GOSTR_X_MORE_LATERALDIS_BYPASS_WE:
        temporary_yaw = Yaw / 100;
        if (my_abs(Yaw / 100) < 150 && (Yaw / 100 < 0))
        {
            right_walk_edge_status = LEFT_EDGE_TURN_CLOCK_TARGET_YAW_MORE_0_WE;
            break;
        }
        else if (my_abs(Yaw / 100) >= 150)
        {
            right_walk_edge_status = LEFT_EDGE_TURN_CLOCK_TARGET_YAW_LESS_135_DETAL_YAW_MORE_30_WE;
            break;
        }
        else
        {
            right_walk_edge_status = LEFT_EDGE_GOSTR_BYPASS_WE_X;
            break;
        }
    case LEFT_EDGE_TURN_CLOCK_TARGET_YAW_MORE_0_WE:
        linear_velocity = 0;
        angular_velocity = -turn_vel;
        if (Yaw / 100 > 0)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            right_walk_edge_status = LEFT_EDGE_GOSTR_BYPASS_WE_X;
            break;
        }
        if (obstacleSignal != none_obstacle)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            right_walk_edge_status = LEFT_EDGE_TURN_CLOCK_TARGET_YAW_MORE_0_COLLISION_WE;
            break;
        }
        break;
    case LEFT_EDGE_TURN_CLOCK_TARGET_YAW_MORE_0_COLLISION_WE:
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
            right_walk_edge_status = LEFT_EDGE_TURN_CLOCK_TARGET_YAW_MORE_0_WE;
            break;
        }
        break;
    case LEFT_EDGE_TURN_CLOCK_TARGET_YAW_LESS_135_DETAL_YAW_MORE_30_WE:
        linear_velocity = 0;
        angular_velocity = -turn_vel;
        if (my_abs(temporary_yaw - Yaw / 100) > 30 && my_abs(Yaw / 100) < 150)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            right_walk_edge_status = LEFT_EDGE_GOSTR_BYPASS_WE_X;
            break;
        }
        if (obstacleSignal != none_obstacle)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            right_walk_edge_status = LEFT_EDGE_TURN_CLOCK_TARGET_YAW_LESS_135_DETAL_YAW_MORE_30_COLLISION_WE;
            break;
        }
        break;
    case LEFT_EDGE_TURN_CLOCK_TARGET_YAW_LESS_135_DETAL_YAW_MORE_30_COLLISION_WE:
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
            right_walk_edge_status = LEFT_EDGE_TURN_CLOCK_TARGET_YAW_LESS_135_DETAL_YAW_MORE_30_WE;
            break;
        }
        break;
    case LEFT_EDGE_BOW_CONTINUE_WE:
        linear_velocity = 0;
        angular_velocity = 0;
        if (!detection_close_edge)
        {
            detection_close_edge = true;
        }
        else
        {
            detection_close_edge = false;
            if (detection_close == true)
            {
                right_walk_edge_status = 0;
                complete_flag = 1;
            }
            else
            {
				if(my_abs(current_pose->x + half_map_wide - edge_length_start) > Edge_length() / 2){
                right_walk_edge_status = LEFT_EDGE_RETURN_ORIGIN_WE;
				}else{
				right_walk_edge_status = 0;
                complete_flag = 1;
				}
            }
        }
        break;
    case LEFT_EDGE_LEFT_EDGE_DILEMMA_WE:
        complete_flag = 2;
        right_walk_edge_status = 0;
        break;
    case LEFT_EDGE_RETURN_ORIGIN_WE:
        linear_velocity = long_stra_vel;
        angular_velocity = 0;
        complete_flag = 3;
        right_walk_edge_status = 0;
        break;
    }
    return complete_flag;
}

unsigned char  LeftReverseWalkEdge(POSE *current_pose, unsigned char obstacleSignal)
{
    int Yaw;
    unsigned char complete_flag = 0;
    Yaw = current_pose->orientation;
    switch (right_reverse_walk_edge_status)
    {
    case 0:
        right_reverse_walk_edge_status = LEFT_GOBACK_REVERSE_WALK_EDGE;
        break;
    case LEFT_GOBACK_REVERSE_WALK_EDGE:
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
            right_reverse_walk_edge_status = LEFT_REVERSE_EDGE_TURN_CLOCK_TARGET_YAW_LESS_ABS63_RWE;
            break;
        }
        break;
    case LEFT_REVERSE_EDGE_TURN_CLOCK_TARGET_YAW_LESS_ABS63_RWE:
        linear_velocity = 0;
        angular_velocity = -turn_vel;
        if (my_abs(Yaw / 100) < 63)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            right_reverse_walk_edge_status = LEFT_REVERSE_EDGE_READY_GOSTR_BYPASS_RWE;
            break;
        }
        if (obstacleSignal != none_obstacle)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            right_reverse_walk_edge_status = LEFT_REVERSE_EDGE_TURN_CLOCK_TARGET_YAW_LESS_ABS63_COLLISION_RWE;
            break;
        }
        break;
    case LEFT_REVERSE_EDGE_TURN_CLOCK_TARGET_YAW_LESS_ABS63_COLLISION_RWE:
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
            right_reverse_walk_edge_status = LEFT_REVERSE_EDGE_TURN_CLOCK_TARGET_YAW_LESS_ABS63_RWE;
            break;
        }
        break;
    case LEFT_REVERSE_EDGE_READY_GOSTR_BYPASS_RWE:
        edge_length_start = current_pose->x + half_map_wide;
        returnorigin = false;
        b_last_position_yy = false;
        last_position_y = current_pose->y;
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
        linear_velocity = long_stra_vel;
        angular_velocity = 0;
        if (obstacleSignal != none_obstacle)
        {
            right_reverse_walk_edge_status = LEFT_REVERSE_EDGE_COLLISION_BYPASS_RWE;
            break;
        }
        if (my_abs(last_position_x - current_pose->x) > lateral_move_distance)
        {
            right_reverse_walk_edge_status = LEFT_REVERSE_EDGE_GOSTR_X_MORE_LATERALDIS_BYPASS_RWE;
            break;
        }
        if ((current_pose->y - last_position_y) > temporary_close_edge)
        {
            right_reverse_walk_edge_status = LEFT_REVERSE_EDGE_BOW_CONTINUE_RWE;
            break;
        }
        if (b_last_position_yy == true && my_abs(last_position_yy - current_pose->y) > 3 * lateral_move_distance)
        {
            right_reverse_walk_edge_status = LEFT_REVERSE_EDGE_BOW_CONTINUE_RWE;
            break;
        }
        if (my_abs(Yaw / 100) > 60 && (Yaw / 100 < 0) && (b_last_position_yy == false))
        {
            last_position_yy = current_pose->y;
            b_last_position_yy = true;
        }
        break;
    case LEFT_REVERSE_EDGE_REBACK_GOSTR_BYPASS_CHECK_RWE:
        if (my_abs(Yaw / 100) > 45 && (Yaw / 100 < 0))
        {
            right_reverse_walk_edge_status = LEFT_REVERSE_EDGE_TARGET_YAW_MORE_ABS75_LESS_ABS0_RWE;
            break;
        }
        if ((my_abs(current_pose->x + half_map_wide - edge_length_start) > Edge_length() / 2) && (my_abs(current_pose->x + half_map_wide - edge_length_start) > 500))
        {
            right_reverse_walk_edge_status = LEFT_REVERSE_EDGE_DELTA_X_MORE_ONE_FOURTH_CLEANED_MAP_WIDTH_RWE;
            break;
        }
        right_reverse_walk_edge_status = LEFT_REVERSE_EDGE_GOSTR_BYPASS_RWE_X;
        break;
    case LEFT_REVERSE_EDGE_COLLISION_BYPASS_RWE:
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
            temporary_yaw = Yaw / 100;
            right_reverse_walk_edge_status = LEFT_REVERSE_EDGE_TURN_CLOCK_YAW_ADD_ABS15_RWE;
            break;
        }
        break;
    case LEFT_REVERSE_EDGE_TURN_CLOCK_YAW_ADD_ABS15_RWE:
        linear_velocity = 0;
        angular_velocity = -turn_vel;
        if (my_abs(Yaw / 100 - temporary_yaw) > 15)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            right_reverse_walk_edge_status = LEFT_REVERSE_EDGE_REBACK_GOSTR_BYPASS_CHECK_RWE;
            break;
        }
        if (obstacleSignal != none_obstacle)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            right_reverse_walk_edge_status = LEFT_REVERSE_EDGE_TURN_CLOCK_YAW_ADD_ABS15_COLLISION_RWE;
            break;
        }
        break;
    case LEFT_REVERSE_EDGE_TURN_CLOCK_YAW_ADD_ABS15_COLLISION_RWE:
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
            right_reverse_walk_edge_status = LEFT_REVERSE_EDGE_TURN_CLOCK_YAW_ADD_ABS15_RWE;
            break;
        }
        break;
    case LEFT_REVERSE_EDGE_TARGET_YAW_MORE_ABS75_LESS_ABS0_RWE:
        linear_velocity = 0;
        angular_velocity = 0;
        if (!detection_close_edge)
        {
            detection_close_edge = true;
            break;
        }
        else
        {
            detection_close_edge = false;
            if (detection_close == true)
            {
                right_reverse_walk_edge_status = LEFT_REVERSE_EDGE_TURN_CCLOCK_YAW_LESS_ABS8_RWE;
            }
            else
            {
                right_reverse_walk_edge_status = LEFT_REVERSE_EDGE_RETURN_ORIGIN_RWE;
            }
            break;
        }
    case LEFT_REVERSE_EDGE_DELTA_X_MORE_ONE_FOURTH_CLEANED_MAP_WIDTH_RWE:
        linear_velocity = 0;
        angular_velocity = 0;
        if (!detection_close_edge)
        {
            detection_close_edge = true;
        }
        else
        {
            detection_close_edge = false;
            if (detection_close == true)
            {
                temporary_yaw = Yaw / 100;
                right_reverse_walk_edge_status = CLOSE_EDGE_MAP_LEFT_REVERSE_WALK_TURN_CLOCK_YAW_ADD_ABS15_RWE;
            }
            else
            {
                right_reverse_walk_edge_status = LEFT_REVERSE_EDGE_RETURN_ORIGIN_RWE;
            }
        }
        break;

    case CLOSE_EDGE_MAP_LEFT_REVERSE_WALK_TURN_CLOCK_YAW_ADD_ABS15_RWE:
        linear_velocity = 0;
        angular_velocity = -turn_vel;
        if (my_abs(Yaw / 100 - temporary_yaw) > 15)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            last_position_x = current_pose->x;
            right_reverse_walk_edge_status = LEFT_REVERSE_EDGE_GOSTR_BYPASS_RWE;
            break;
        }
        if (obstacleSignal != none_obstacle)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            right_reverse_walk_edge_status = COLLISION_CLOSE_EDGE_MAP_LEFT_REVERSE_WALK_TURN_CLOCK_YAW_ADD_ABS15_RWE;
            break;
        }
        break;
    case COLLISION_CLOSE_EDGE_MAP_LEFT_REVERSE_WALK_TURN_CLOCK_YAW_ADD_ABS15_RWE:
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
            right_reverse_walk_edge_status = CLOSE_EDGE_MAP_LEFT_REVERSE_WALK_TURN_CLOCK_YAW_ADD_ABS15_RWE;
            break;
        }
        break;
    case LEFT_REVERSE_EDGE_TURN_CCLOCK_YAW_LESS_ABS8_RWE:
        if (my_abs(Yaw / 100) > 175)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            right_reverse_walk_edge_status = LEFT_REVERSE_EDGE_LEFT_EDGE_DILEMMA_RWE;
            break;
        }
        if (obstacleSignal != none_obstacle)
        {
            right_reverse_walk_edge_status = LEFT_REVERSE_EDGE_TURN_CCLOCK_YAW_LESS_ABS8_COLLISION_RWE;
            break;
        }
        linear_velocity = 0;
        angular_velocity = -turn_vel;
        break;
    case LEFT_REVERSE_EDGE_TURN_CCLOCK_YAW_LESS_ABS8_COLLISION_RWE:
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
            right_reverse_walk_edge_status = LEFT_REVERSE_EDGE_TURN_CCLOCK_YAW_LESS_ABS8_RWE;
            break;
        }
        break;

    case LEFT_REVERSE_EDGE_GOSTR_X_MORE_LATERALDIS_BYPASS_RWE:
        temporary_yaw = Yaw / 100;
        if (my_abs(Yaw / 100) > 30 && (Yaw / 100 < 0))
        {
            right_reverse_walk_edge_status = LEFT_REVERSE_EDGE_TURN_CCLOCK_TARGET_YAW_MORE_0_RWE;

            break;
        }
        else if (my_abs(Yaw / 100) <= 30)
        {
            right_reverse_walk_edge_status = LEFT_REVERSE_EDGE_TURN_CCLOCK_TARGET_YAW_MORE_ABS45_DETAL_YAW_MORE_30_RWE;
            break;
        }
        else
        {
            right_reverse_walk_edge_status = LEFT_REVERSE_EDGE_GOSTR_BYPASS_RWE_X;
            break;
        }
    case LEFT_REVERSE_EDGE_TURN_CCLOCK_TARGET_YAW_MORE_0_RWE:
        linear_velocity = 0;
        angular_velocity = turn_vel;
        if (Yaw / 100 > 0)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            right_reverse_walk_edge_status = LEFT_REVERSE_EDGE_GOSTR_BYPASS_RWE_X;
            break;
        }
        if (obstacleSignal != none_obstacle)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            right_reverse_walk_edge_status = LEFT_REVERSE_EDGE_TURN_CCLOCK_TARGET_YAW_MORE_0_COLLISION_RWE;
            break;
        }
        break;
    case LEFT_REVERSE_EDGE_TURN_CCLOCK_TARGET_YAW_MORE_0_COLLISION_RWE:
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
            right_reverse_walk_edge_status = LEFT_REVERSE_EDGE_TURN_CCLOCK_TARGET_YAW_MORE_0_RWE;
            break;
        }
        break;
    case LEFT_REVERSE_EDGE_TURN_CCLOCK_TARGET_YAW_MORE_ABS45_DETAL_YAW_MORE_30_RWE:
        linear_velocity = 0;
        angular_velocity = turn_vel;
        if (my_abs(temporary_yaw - Yaw / 100) > 30 && my_abs(Yaw / 100) > 30)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            right_reverse_walk_edge_status = LEFT_REVERSE_EDGE_GOSTR_BYPASS_RWE_X;
            break;
        }
        if (obstacleSignal != none_obstacle)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            right_reverse_walk_edge_status = LEFT_REVERSE_EDGE_TURN_CCLOCK_TARGET_YAW_MORE_ABS45_DETAL_YAW_MORE_30_COLLISION_RWE;
            break;
        }
        break;
    case LEFT_REVERSE_EDGE_TURN_CCLOCK_TARGET_YAW_MORE_ABS45_DETAL_YAW_MORE_30_COLLISION_RWE:
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
            right_reverse_walk_edge_status = LEFT_REVERSE_EDGE_TURN_CCLOCK_TARGET_YAW_MORE_ABS45_DETAL_YAW_MORE_30_RWE;
            break;
        }
        break;
    case LEFT_REVERSE_EDGE_BOW_CONTINUE_RWE:
        linear_velocity = 0;
        angular_velocity = 0;
        if (!detection_close_edge)
        {
            detection_close_edge = true;
        }
        else
        {
            detection_close_edge = false;
            if (detection_close == true)
            {
                right_reverse_walk_edge_status = 0;
                complete_flag = 1;
            }
            else
            {
				if(my_abs(current_pose->x + half_map_wide - edge_length_start) > Edge_length() / 2){
                right_reverse_walk_edge_status = LEFT_REVERSE_EDGE_RETURN_ORIGIN_RWE;
				}else{
				right_reverse_walk_edge_status = 0;
                complete_flag = 1;
				}
            }
        }
        break;
    case LEFT_REVERSE_EDGE_LEFT_EDGE_DILEMMA_RWE:
        complete_flag = 2;
        right_reverse_walk_edge_status = 0;
        break;
    case LEFT_REVERSE_EDGE_RETURN_ORIGIN_RWE:
        linear_velocity = long_stra_vel;
        angular_velocity = 0;
        complete_flag = 3;
        right_reverse_walk_edge_status = 0;
        break;
    }
    return complete_flag;
}

unsigned char  LeftEdgeDilemma(POSE *current_pose, unsigned char obstacleSignal)
{
    int Yaw;
    unsigned char complete_flag = 0;
    Yaw = current_pose->orientation;
    switch (right_edge_dilemma_status)
    {
    case 0:
        last_position_x = current_pose->x + half_map_wide;
        right_edge_dilemma_status = LEFT_DILEMMA_DELTA_X_MORE_ONE_THIRD_CLEANED_MAP_WIDTH_DILEMMA;
        break;
    case LEFT_DILEMMA_DELTA_X_MORE_ONE_THIRD_CLEANED_MAP_WIDTH_DILEMMA:
        if (my_abs(last_position_x - (current_pose->x + half_map_wide)) > Edge_length() / 3)
        {
            complete_flag = 1;
            right_edge_dilemma_status = 0;
            DelimmaNumber = 0;
            break;
        }
        right_edge_dilemma_status = LEFT_DILEMMA_LOOP_TEN_NUM_DILEMMA;
        break;
    case LEFT_DILEMMA_LOOP_TEN_NUM_DILEMMA:
        if (DelimmaNumber > 4)
        {
            complete_flag = 1;
            right_edge_dilemma_status = 0;
            DelimmaNumber = 0;
            break;
        }
        right_edge_dilemma_status = LEFT_DILEMMA_YAW_MORE_ABS90_LESS_ABS176_CCLOCK_LESS_ABS90_MORE_ABS4_CLOCK_DILEMMA;
        break;
    case LEFT_DILEMMA_YAW_MORE_ABS90_LESS_ABS176_CCLOCK_LESS_ABS90_MORE_ABS4_CLOCK_DILEMMA:
        if (my_abs(Yaw / 100) > 90 && my_abs(Yaw / 100) < 175)
        {
            if (Yaw > 0)
            {
                linear_velocity = 0;
                angular_velocity = 5 * turn_vel / 6;
            }
            else
            {
                linear_velocity = 0;
                angular_velocity = -5 * turn_vel / 6;
            }
        }
        else if (my_abs(Yaw / 100) < 90 && my_abs(Yaw / 100) > 5)
        {
            if (Yaw > 0)
            {
                linear_velocity = 0;
                angular_velocity = -5 * turn_vel / 6;
            }
            else
            {
                linear_velocity = 0;
                angular_velocity = 5 * turn_vel / 6;
            }
        }
        else
        {
            linear_velocity = long_stra_vel;
            angular_velocity = 0;
        }
        right_edge_dilemma_status = LEFT_DILEMMA_GOSTR_DILEMMA;
        break;
    case LEFT_DILEMMA_GOSTR_DILEMMA:
        if (obstacleSignal != none_obstacle)
        {
            right_edge_dilemma_status = LEFT_DILEMMA_GOSTR_COLLISION_DILEMMA;
            break;
        }
        right_edge_dilemma_status = LEFT_DILEMMA_DELTA_X_MORE_ONE_THIRD_CLEANED_MAP_WIDTH_DILEMMA;
        break;
    case LEFT_DILEMMA_GOSTR_COLLISION_DILEMMA:
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
            if (my_abs(Yaw / 100) > 90)
            {
                right_edge_dilemma_status = LEFT_DILEMMA_COLLISION_YAW_MORE_ABS90_DILEMMA;
                break;
            }
            else
            {
                right_edge_dilemma_status = LEFT_DILEMMA_COLLISION_YAW_LESS_ABS90_DILEMMA;
                break;
            }
        }
        break;

    case LEFT_DILEMMA_COLLISION_YAW_MORE_ABS90_DILEMMA:
        right_edge_dilemma_status = LEFT_DILEMMA_CCLOCK_TARGET_YAW_LESS_ABS90_DILEMMA;
        break;
    case LEFT_DILEMMA_CCLOCK_TARGET_YAW_LESS_ABS90_DILEMMA:
        if (my_abs(Yaw / 100) < 90)
        {
            last_position_y = current_pose->y;
            last_position_x = current_pose->x + half_map_wide;
            right_edge_dilemma_status = LEFT_DILEMMA_GOSTR_CYM_DILEMMA;
            break;
        }
        if (obstacleSignal != none_obstacle)
        {
            right_edge_dilemma_status = LEFT_DILEMMA_CCLOCK_TARGET_YAW_LESS_ABS90_COLLISION_DILEMMA;
        }

        linear_velocity = 0;
        angular_velocity = turn_vel;
        break;
    case LEFT_DILEMMA_CCLOCK_TARGET_YAW_LESS_ABS90_COLLISION_DILEMMA:
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
            right_edge_dilemma_status = LEFT_DILEMMA_CCLOCK_TARGET_YAW_LESS_ABS90_DILEMMA;
        }
        break;
    case LEFT_DILEMMA_GOSTR_CYM_DILEMMA:
        if (obstacleSignal != none_obstacle)
        {
            DelimmaNumber++;
            right_edge_dilemma_status = LEFT_DILEMMA_GOSTR_COLLISION_CYM_DILEMMA;
            break;
        }
        if (my_abs(last_position_y - current_pose->y) > lateral_move_distance / 3)
        {
            right_edge_dilemma_status = LEFT_DILEMMA_GOSTR_DELTA_Y_MORE_LATERAL_DIS_CYM_DILEMMA;
            break;
        }
        linear_velocity = long_stra_vel;
        angular_velocity = 0;
        break;
    case LEFT_DILEMMA_GOSTR_COLLISION_CYM_DILEMMA:
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
            right_edge_dilemma_status = LEFT_DILEMMA_CCLOCK_TARGET_YAW_LESS_ABS3_DILEMMA;
            break;
        }
        break;

    case LEFT_DILEMMA_GOSTR_DELTA_Y_MORE_LATERAL_DIS_CYM_DILEMMA:
        right_edge_dilemma_status = LEFT_DILEMMA_CCLOCK_TARGET_YAW_LESS_ABS3_DILEMMA;
        break;
    case LEFT_DILEMMA_CCLOCK_TARGET_YAW_LESS_ABS3_DILEMMA:
        if (my_abs(Yaw / 100) < 5)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            right_edge_dilemma_status = LEFT_DILEMMA_DELTA_X_MORE_ONE_THIRD_CLEANED_MAP_WIDTH_DILEMMA;
            break;
        }
        if (obstacleSignal != none_obstacle)
        {
            right_edge_dilemma_status = LEFT_DILEMMA_CCLOCK_TARGET_YAW_LESS_ABS3_COLLISION_DILEMMA;
            break;
        }
        linear_velocity = 0;
        angular_velocity = turn_vel;
        break;
    case LEFT_DILEMMA_CCLOCK_TARGET_YAW_LESS_ABS3_COLLISION_DILEMMA:
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
            right_edge_dilemma_status = LEFT_DILEMMA_CCLOCK_TARGET_YAW_LESS_ABS3_DILEMMA;
        }
        break;

    case LEFT_DILEMMA_COLLISION_YAW_LESS_ABS90_DILEMMA:
        right_edge_dilemma_status = LEFT_DILEMMA_CLOCK_TARGET_YAW_MORE_ABS87_DILEMMA;
        break;
    case LEFT_DILEMMA_CLOCK_TARGET_YAW_MORE_ABS87_DILEMMA:
        if (my_abs(Yaw / 100) > 87)
        {
            last_position_y = current_pose->y;
            last_position_x = current_pose->x + half_map_wide;
            right_edge_dilemma_status = LEFT_DILEMMA_GOSTR_CYL_DILEMMA;
            break;
        }
        if (obstacleSignal != none_obstacle)
        {
            right_edge_dilemma_status = LEFT_DILEMMA_CLOCK_TARGET_YAW_MORE_ABS87_COLLISION_DILEMMA;
            break;
        }

        linear_velocity = 0;
        angular_velocity = -turn_vel;
        break;
    case LEFT_DILEMMA_CLOCK_TARGET_YAW_MORE_ABS87_COLLISION_DILEMMA:
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
            right_edge_dilemma_status = LEFT_DILEMMA_CLOCK_TARGET_YAW_MORE_ABS87_DILEMMA;
        }
        break;
    case LEFT_DILEMMA_GOSTR_CYL_DILEMMA:
        if (obstacleSignal != none_obstacle)
        {
            DelimmaNumber++;
            right_edge_dilemma_status = LEFT_DILEMMA_GOSTR_COLLISION_CYL_DILEMMA;
            break;
        }
        if (my_abs(last_position_y - current_pose->y) > lateral_move_distance)
        {
            right_edge_dilemma_status = LEFT_DILEMMA_GOSTR_GOSTR_DELTA_Y_MORE_LATERAL_DIS_CYL_DILEMMA;
            break;
        }
        linear_velocity = long_stra_vel;
        angular_velocity = 0;
        break;
    case LEFT_DILEMMA_GOSTR_COLLISION_CYL_DILEMMA:
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
            right_edge_dilemma_status = LEFT_DILEMMA_GOSTR_CYL_DILEMMA;
            break;
        }
        break;
    case LEFT_DILEMMA_GOSTR_GOSTR_DELTA_Y_MORE_LATERAL_DIS_CYL_DILEMMA:
        right_edge_dilemma_status = LEFT_DILEMMA_CLOCK_TARGET_YAW_MORE_ABS173_DILEMMA;
        break;
    case LEFT_DILEMMA_CLOCK_TARGET_YAW_MORE_ABS173_DILEMMA:
        if (my_abs(Yaw / 100) > 175)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            right_edge_dilemma_status = LEFT_DILEMMA_DELTA_X_MORE_ONE_THIRD_CLEANED_MAP_WIDTH_DILEMMA;
            break;
        }
        if (obstacleSignal != none_obstacle)
        {
            right_edge_dilemma_status = LEFT_DILEMMA_CLOCK_TARGET_YAW_MORE_ABS173_COLLISION_DILEMMA;
            break;
        }
        linear_velocity = 0;
        angular_velocity = -turn_vel;
        break;
    case LEFT_DILEMMA_CLOCK_TARGET_YAW_MORE_ABS173_COLLISION_DILEMMA:
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
            right_edge_dilemma_status = LEFT_DILEMMA_CLOCK_TARGET_YAW_MORE_ABS173_DILEMMA;
        }
        break;
    }
    return complete_flag;
}

unsigned char  LeftReadyLeakingSweep(POSE *current_pose, unsigned char obstacleSignal)
{
    int Yaw;
    unsigned char complete_flag = 0;
    Yaw = current_pose->orientation;
    switch (right_ready_leaking_sweep_status)
    {
    case 0:
        right_ready_leaking_sweep_status = LEFT_LEAKING_SWEEP_COLLISION;
        break;
    case LEFT_LEAKING_SWEEP_COLLISION:
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
            if (my_abs(Yaw / 100) > 90)
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
        if (my_abs(Yaw / 100) < 90)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            last_position_y = current_pose->y;
            right_ready_leaking_sweep_status = LEFT_LEAKING_SWEEP_GOSTRAIGHT_MORE;
            break;
        }
        if (obstacleSignal != none_obstacle)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            right_ready_leaking_sweep_status = LEFT_LEAKING_SWEEP_CCLOCK_TARGER_YAW_LESS_ABS90_COLLISION;
        }
        linear_velocity = 0;
        angular_velocity = turn_vel;
        break;
    case LEFT_LEAKING_SWEEP_CCLOCK_TARGER_YAW_LESS_ABS90_COLLISION:
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
            right_ready_leaking_sweep_status = LEFT_LEAKING_SWEEP_CCLOCK_TARGET_YAW_LESS_ABS90;
            break;
        }
        break;
    case LEFT_LEAKING_SWEEP_GOSTRAIGHT_MORE:
        linear_velocity = long_stra_vel;
        angular_velocity = 0;
        if (obstacleSignal != none_obstacle)
        {
            right_ready_leaking_sweep_status = LEFT_LEAKING_SWEEP_CCLOCK_TARGER_YAW_LESS_ABS8;
            break;
        }
        if (my_abs(last_position_y - current_pose->y) > leakingsweep)
        {
            right_ready_leaking_sweep_status = LEFT_LEAKING_SWEEP_CCLOCK_TARGER_YAW_LESS_ABS8;
            break;
        }
        break;
    case LEFT_LEAKING_SWEEP_CCLOCK_TARGER_YAW_LESS_ABS8:
        if (my_abs(Yaw / 100) < 5)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            right_ready_leaking_sweep_status = LEFT_LEAKING_SWEEP_COMPLETE;
            break;
        }
        if (obstacleSignal != none_obstacle)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            right_ready_leaking_sweep_status = LEFT_LEAKING_SWEEP_CCLOCK_TARGER_YAW_LESS_ABS8_COLLISION;
        }
        linear_velocity = 0;
        angular_velocity = turn_vel;
        break;
    case LEFT_LEAKING_SWEEP_CCLOCK_TARGER_YAW_LESS_ABS8_COLLISION:
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
            right_ready_leaking_sweep_status = LEFT_LEAKING_SWEEP_CCLOCK_TARGER_YAW_LESS_ABS8;
            break;
        }
        break;
    case LEFT_LEAKING_SWEEP_YAW_OTHER:
        right_ready_leaking_sweep_status = LEFT_LEAKING_SWEEP_CLOCK_TARGET_YAW_MORE_ABS90;
        break;
    case LEFT_LEAKING_SWEEP_CLOCK_TARGET_YAW_MORE_ABS90:
        if (my_abs(Yaw / 100) > 90)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            last_position_y = current_pose->y;
            right_ready_leaking_sweep_status = LEFT_LEAKING_SWEEP_GOSTRAIGHT_OTHER;
            break;
        }
        if (obstacleSignal != none_obstacle)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            right_ready_leaking_sweep_status = LEFT_LEAKING_SWEEP_CLOCK_TARGET_YAW_MORE_ABS90_COLLISION;
        }
        linear_velocity = 0;
        angular_velocity = -turn_vel;
        break;
    case LEFT_LEAKING_SWEEP_CLOCK_TARGET_YAW_MORE_ABS90_COLLISION:
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
            right_ready_leaking_sweep_status = LEFT_LEAKING_SWEEP_CLOCK_TARGET_YAW_MORE_ABS90;
            break;
        }
        break;
    case LEFT_LEAKING_SWEEP_GOSTRAIGHT_OTHER:
        linear_velocity = long_stra_vel;
        angular_velocity = 0;
        if (obstacleSignal != none_obstacle)
        {
            right_ready_leaking_sweep_status = LEFT_LEAKING_SWEEP_CLOCK_TARGET_YAW_MORE_ABS173;
            break;
        }
        if (my_abs(last_position_y - current_pose->y) > leakingsweep)
        {
            right_ready_leaking_sweep_status = LEFT_LEAKING_SWEEP_CLOCK_TARGET_YAW_MORE_ABS173;
            break;
        }
        break;
    case LEFT_LEAKING_SWEEP_CLOCK_TARGET_YAW_MORE_ABS173:
        if (my_abs(Yaw / 100) > 175)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            right_ready_leaking_sweep_status = LEFT_LEAKING_SWEEP_COMPLETE;
            break;
        }
        if (obstacleSignal != none_obstacle)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            right_ready_leaking_sweep_status = LEFT_LEAKING_SWEEP_CLOCK_TARGET_YAW_MOEE_ABS173_COLLISIION;
        }
        linear_velocity = 0;
        angular_velocity = -turn_vel;
        break;
    case LEFT_LEAKING_SWEEP_CLOCK_TARGET_YAW_MOEE_ABS173_COLLISIION:
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


////////////////////////////////////////////////////////////////////////////////////////////////////////


//#################################################################################
//##############A* return origin function define###################################
//#################################################################################

unsigned char ForceReturnOrigin(POSE *current_pose, unsigned char obstacleSignal)
{
    int Yaw;
    unsigned char complete_flag = 0;
    Yaw = current_pose->orientation;

    switch (return_origin_step_status)
    {
    case 0:
        return_origin_step_status = GOSTR_RETURN_ORIGIN_STEP;
        break;
    case GOSTR_RETURN_ORIGIN_STEP:
        log_debug("return origin");
        if (my_abs(current_pose->x) > return_origin_distance || my_abs(current_pose->y) > return_origin_distance)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            log_debug("return origin");
            if (my_abs(current_pose->y) > return_origin_distance)
            {
                if (current_pose->y > 0)
                {
                    return_origin_step_status = DIR_Y_MORE_POSITIVE_200;
                }
                else
                {
                    return_origin_step_status = DIR_Y_LESS_NEGATIVE_200;
                }
            }
            else
            {
                if (current_pose->x > 0)
                {
                    return_origin_step_status = DIR_X_MORE_POSITIVE_200;
                }
                else
                {
                    return_origin_step_status = DIR_X_LESS_NEGATIVE_200;
                }
            }
        }
        else
        {
            log_debug("Has returned to the origin");
            linear_velocity = 0;
            angular_velocity = 0;
            complete_flag = 1;
        }
        break;
    case DIR_Y_MORE_POSITIVE_200:
        if (obstacleSignal != none_obstacle)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            return_origin_step_status = TURN_CLOCK_DIR_Y_MORE_POSITIVE_200;
            break;
        }
        if (my_abs(Yaw / 100) <= 90)
        {
            angular_velocity = -turn_vel;
            linear_velocity = 0;
        }
        else
        {
            angular_velocity = turn_vel;
            linear_velocity = 0;
        }
        if ((-95 < Yaw / 100) && (Yaw / 100 < -85))
        {
            linear_velocity = 0;
            angular_velocity = 0;
            return_origin_step_status = TURN_CLOCK_TARGET_YAW_NEGATIVE_90_RETURN_ORIGIN;
        }
        break;
    case TURN_CLOCK_DIR_Y_MORE_POSITIVE_200:
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
            return_origin_step_status = DIR_Y_MORE_POSITIVE_200;
        }
        break;
    case DIR_Y_LESS_NEGATIVE_200:
        if (obstacleSignal != none_obstacle)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            return_origin_step_status = TURN_CLOCK_DIR_Y_LESS_NEGATIVE_200;
            break;
        }
        if (my_abs(Yaw / 100) <= 90)
        {
            angular_velocity = turn_vel;
            linear_velocity = 0;
        }
        else
        {
            angular_velocity = -turn_vel;
            linear_velocity = 0;
        }
        if ((95 > Yaw / 100) && (Yaw / 100 > 85))
        {
            linear_velocity = 0;
            angular_velocity = 0;
            return_origin_step_status = TURN_CLOCK_TARGET_YAW_POSITIVE_90_RETURN_ORIGIN;
            break;
        }
        break;
    case TURN_CLOCK_DIR_Y_LESS_NEGATIVE_200:
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
            return_origin_step_status = DIR_Y_LESS_NEGATIVE_200;
        }
        break;
    case DIR_X_MORE_POSITIVE_200:
        if (obstacleSignal != none_obstacle)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            return_origin_step_status = TURN_CLOCK_DIR_X_MORE_POSITIVE_200;
            break;
        }
        if (Yaw > 0)
        {
            angular_velocity = turn_vel;
            linear_velocity = 0;
        }
        else
        {
            angular_velocity = -turn_vel;
            linear_velocity = 0;
        }
        if (my_abs(Yaw / 100) > 175)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            return_origin_step_status = TURN_CLOCK_TARGET_YAW_180_RETURN_ORIGIN;
        }
        break;
    case TURN_CLOCK_DIR_X_MORE_POSITIVE_200:
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
            return_origin_step_status = DIR_X_MORE_POSITIVE_200;
        }
        break;
    case DIR_X_LESS_NEGATIVE_200:
        if (obstacleSignal != none_obstacle)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            return_origin_step_status = TURN_CLOCK_DIR_X_LESS_NEGATIVE_200;
            break;
        }
        if (Yaw > 0)
        {
            angular_velocity = -turn_vel;
            linear_velocity = 0;
        }
        else
        {
            angular_velocity = turn_vel;
            linear_velocity = 0;
        }
        if (my_abs(Yaw / 100) < 5)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            return_origin_step_status = TURN_CLOCK_TARGET_YAW_5_RETURN_ORIGIN;
        }
        break;
    case TURN_CLOCK_DIR_X_LESS_NEGATIVE_200:
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
            return_origin_step_status = DIR_X_LESS_NEGATIVE_200;
        }
        break;
    case TURN_CLOCK_TARGET_YAW_POSITIVE_90_RETURN_ORIGIN:
        if (obstacleSignal != none_obstacle)
        {
            return_origin_step_status = COLLISION_Y_POSITIVE_90_RETURN_ORIGIN;
            break;
        }
        if (my_abs(current_pose->y) < return_origin_distance)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            return_origin_step_status = GOSTR_RETURN_ORIGIN_STEP;
            break;
        }
        linear_velocity = long_stra_vel;
        angular_velocity = 0;
        break;
    case COLLISION_Y_POSITIVE_90_RETURN_ORIGIN:
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
            temporary_yaw = Yaw / 100;
            return_origin_step_status = TURN_CLOCK_Y_POSITIVE_90_RETURN_ORIGIN;
        }
        break;
    case TURN_CLOCK_Y_POSITIVE_90_RETURN_ORIGIN:
        if (obstacleSignal != none_obstacle)
        {
            return_origin_step_status = TURN_CLOCK_COLLISION_Y_POSITIVE_90_RETURN_ORIGIN;
            break;
        }
        if (my_abs(Yaw / 100 - temporary_yaw) > 10)
        {
            return_origin_step_status = BYPASS_OBSTACLES_Y_POSITIVE_90_RETURN_ORIGIN;
            break;
        }
        if (current_pose->x > 0)
        {
            return_origin_positive_start = 1;
        }
        else
        {
            return_origin_positive_start = -1;
        }
        linear_velocity = 0;
        angular_velocity = return_origin_positive_start * turn_vel;
        break;
    case TURN_CLOCK_COLLISION_Y_POSITIVE_90_RETURN_ORIGIN:
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
            return_origin_step_status = TURN_CLOCK_Y_POSITIVE_90_RETURN_ORIGIN;
        }
        break;
    case BYPASS_OBSTACLES_Y_POSITIVE_90_RETURN_ORIGIN:
        last_position_y = current_pose->y;
        last_position_x = current_pose->x;
        return_origin_step_status = LOOP_BYPASS_OBSTACLES_Y_POSITIVE_90_RETURN_ORIGIN;
        break;
    case LOOP_BYPASS_OBSTACLES_Y_POSITIVE_90_RETURN_ORIGIN:
        if (obstacleSignal != none_obstacle)
        {
            temporary_yaw = Yaw / 100;
            return_origin_step_status = COLLISION_LOOP_BYPASS_OBSTACLES_Y_POSITIVE_90_RETURN_ORIGIN;
            break;
        }
        if (my_abs(last_position_y - current_pose->y) > lateral_move_distance || my_abs(last_position_x - current_pose->x) > lateral_move_distance)
        {
            temporary_yaw = Yaw / 100;
            return_origin_step_status = Y_MORE_LOOP_BYPASS_OBSTACLES_Y_POSITIVE_90_RETURN_ORIGIN;
            break;
        }
        if (((100 > Yaw / 100) && (Yaw / 100 > 80)) || my_abs(current_pose->y) < return_origin_distance)
        {
            return_origin_step_status = GOSTR_RETURN_ORIGIN_STEP;
            break;
        }
        linear_velocity = long_stra_vel;
        angular_velocity = 0;
        break;
    case COLLISION_LOOP_BYPASS_OBSTACLES_Y_POSITIVE_90_RETURN_ORIGIN:
        if (obstacleSignal != none_obstacle)
        {
            return_origin_step_status = TURN_CLOCK_LOOP_COLLISION_Y_POSITIVE_90_RETURN_ORIGIN;
            break;
        }
        if (my_abs(Yaw / 100 - temporary_yaw) > 10)
        {
            return_origin_step_status = BYPASS_OBSTACLES_Y_POSITIVE_90_RETURN_ORIGIN;
            break;
        }
        linear_velocity = 0;
        angular_velocity = return_origin_positive_start * turn_vel;
        break;
    case TURN_CLOCK_LOOP_COLLISION_Y_POSITIVE_90_RETURN_ORIGIN:
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
            return_origin_step_status = COLLISION_LOOP_BYPASS_OBSTACLES_Y_POSITIVE_90_RETURN_ORIGIN;
        }
        break;
    case Y_MORE_LOOP_BYPASS_OBSTACLES_Y_POSITIVE_90_RETURN_ORIGIN:
        if (obstacleSignal != none_obstacle)
        {
            return_origin_step_status = COLLISION_LOOP_BYPASS_OBSTACLES_Y_POSITIVE_90_RETURN_ORIGIN;
            break;
        }
        if (((100 > Yaw / 100) && (Yaw / 100 > 80)) || my_abs(Yaw / 100 - temporary_yaw) > 30)
        {
            return_origin_step_status = BYPASS_OBSTACLES_Y_POSITIVE_90_RETURN_ORIGIN;
            break;
        }
        linear_velocity = 0;
        angular_velocity = -return_origin_positive_start * turn_vel;
        break;
    case TURN_CLOCK_TARGET_YAW_NEGATIVE_90_RETURN_ORIGIN:
        if (obstacleSignal != none_obstacle)
        {
            return_origin_step_status = COLLISION_Y_NEGATIVE_90_RETURN_ORIGIN;
            break;
        }
        if (my_abs(current_pose->y) < return_origin_distance)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            return_origin_step_status = GOSTR_RETURN_ORIGIN_STEP;
            break;
        }
        linear_velocity = long_stra_vel;
        angular_velocity = 0;
        break;
    case COLLISION_Y_NEGATIVE_90_RETURN_ORIGIN:
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
            temporary_yaw = Yaw / 100;
            return_origin_step_status = TURN_CLOCK_Y_NEGATIVE_90_RETURN_ORIGIN;
        }
        break;
    case TURN_CLOCK_Y_NEGATIVE_90_RETURN_ORIGIN:
        if (obstacleSignal != none_obstacle)
        {
            return_origin_step_status = TURN_CLOCK_COLLISION_Y_NEGATIVE_90_RETURN_ORIGIN;
            break;
        }
        if (my_abs(Yaw / 100 - temporary_yaw) > 10)
        {
            return_origin_step_status = BYPASS_OBSTACLES_Y_NEGATIVE_90_RETURN_ORIGIN;
            break;
        }
        if (current_pose->x > 0)
        {
            return_origin_positive_start = 1;
        }
        else
        {
            return_origin_positive_start = -1;
        }
        linear_velocity = 0;
        angular_velocity = -return_origin_positive_start * turn_vel;
        break;
    case TURN_CLOCK_COLLISION_Y_NEGATIVE_90_RETURN_ORIGIN:
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
            return_origin_step_status = TURN_CLOCK_Y_NEGATIVE_90_RETURN_ORIGIN;
        }
        break;
    case BYPASS_OBSTACLES_Y_NEGATIVE_90_RETURN_ORIGIN:
        last_position_y = current_pose->y;
        last_position_x = current_pose->x;
        return_origin_step_status = LOOP_BYPASS_OBSTACLES_Y_NEGATIVE_90_RETURN_ORIGIN;
        break;
    case LOOP_BYPASS_OBSTACLES_Y_NEGATIVE_90_RETURN_ORIGIN:
        if (obstacleSignal != none_obstacle)
        {
            temporary_yaw = Yaw / 100;
            return_origin_step_status = COLLISION_LOOP_BYPASS_OBSTACLES_Y_NEGATIVE_90_RETURN_ORIGIN;
            break;
        }
        if (my_abs(last_position_y - current_pose->y) > lateral_move_distance || my_abs(last_position_x - current_pose->x) > lateral_move_distance)
        {
            temporary_yaw = Yaw / 100;
            return_origin_step_status = Y_MORE_LOOP_BYPASS_OBSTACLES_Y_NEGATIVE_90_RETURN_ORIGIN;
            break;
        }
        if (((-100 < Yaw / 100) && (Yaw / 100 < -80)) || my_abs(current_pose->y) < return_origin_distance)
        {
            return_origin_step_status = GOSTR_RETURN_ORIGIN_STEP;
            break;
        }
        linear_velocity = long_stra_vel;
        angular_velocity = 0;
        break;
    case COLLISION_LOOP_BYPASS_OBSTACLES_Y_NEGATIVE_90_RETURN_ORIGIN:
        if (obstacleSignal != none_obstacle)
        {
            return_origin_step_status = TURN_CLOCK_LOOP_COLLISION_Y_NEGATIVE_90_RETURN_ORIGIN;
            break;
        }
        if (my_abs(Yaw / 100 - temporary_yaw) > 10)
        {
            return_origin_step_status = BYPASS_OBSTACLES_Y_NEGATIVE_90_RETURN_ORIGIN;
            break;
        }
        linear_velocity = 0;
        angular_velocity = -return_origin_positive_start * turn_vel;
        break;
    case TURN_CLOCK_LOOP_COLLISION_Y_NEGATIVE_90_RETURN_ORIGIN:
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
            return_origin_step_status = COLLISION_LOOP_BYPASS_OBSTACLES_Y_NEGATIVE_90_RETURN_ORIGIN;
        }
        break;
    case Y_MORE_LOOP_BYPASS_OBSTACLES_Y_NEGATIVE_90_RETURN_ORIGIN:
        if (obstacleSignal != none_obstacle)
        {
            return_origin_step_status = COLLISION_LOOP_BYPASS_OBSTACLES_Y_NEGATIVE_90_RETURN_ORIGIN;
            break;
        }
        if ((-100 < Yaw / 100) && (Yaw / 100 < -80) || my_abs(Yaw / 100 - temporary_yaw) > 30)
        {
            return_origin_step_status = BYPASS_OBSTACLES_Y_NEGATIVE_90_RETURN_ORIGIN;
            break;
        }
        linear_velocity = 0;
        angular_velocity = return_origin_positive_start * turn_vel;
        break;
    case TURN_CLOCK_TARGET_YAW_180_RETURN_ORIGIN:
        if (obstacleSignal != none_obstacle)
        {
            return_origin_step_status = COLLISION_X_POSITIVE_180_RETURN_ORIGIN;
            break;
        }
        if (my_abs(current_pose->x) < return_origin_distance)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            return_origin_step_status = GOSTR_RETURN_ORIGIN_STEP;
            break;
        }
        linear_velocity = long_stra_vel;
        angular_velocity = 0;
        break;
    case COLLISION_X_POSITIVE_180_RETURN_ORIGIN:
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
            temporary_yaw = Yaw / 100;
            return_origin_step_status = TURN_CLOCK_X_POSITIVE_180_RETURN_ORIGIN;
        }
        break;
    case TURN_CLOCK_X_POSITIVE_180_RETURN_ORIGIN:
        if (obstacleSignal != none_obstacle)
        {
            return_origin_step_status = TURN_CLOCK_COLLISION_X_POSITIVE_180_RETURN_ORIGIN;
            break;
        }
        if (my_abs(Yaw / 100 - temporary_yaw) > 10)
        {
            return_origin_step_status = BYPASS_OBSTACLES_X_POSITIVE_180_RETURN_ORIGIN;
            break;
        }
        if (current_pose->y > 0)
        {
            return_origin_positive_start = 1;
        }
        else
        {
            return_origin_positive_start = -1;
        }
        linear_velocity = 0;
        angular_velocity = -return_origin_positive_start * turn_vel;
        break;
    case TURN_CLOCK_COLLISION_X_POSITIVE_180_RETURN_ORIGIN:
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
            return_origin_step_status = TURN_CLOCK_X_POSITIVE_180_RETURN_ORIGIN;
        }
        break;
    case BYPASS_OBSTACLES_X_POSITIVE_180_RETURN_ORIGIN:
        last_position_y = current_pose->y;
        last_position_x = current_pose->x;
        return_origin_step_status = LOOP_BYPASS_OBSTACLES_X_POSITIVE_180_RETURN_ORIGIN;
        break;
    case LOOP_BYPASS_OBSTACLES_X_POSITIVE_180_RETURN_ORIGIN:
        if (obstacleSignal != none_obstacle)
        {
            temporary_yaw = Yaw / 100;
            return_origin_step_status = COLLISION_LOOP_BYPASS_OBSTACLES_X_POSITIVE_180_RETURN_ORIGIN;
            break;
        }
        if (my_abs(last_position_y - current_pose->y) > lateral_move_distance || my_abs(last_position_x - current_pose->x) > lateral_move_distance )
        {
            temporary_yaw = Yaw / 100;
            return_origin_step_status = X_MORE_LOOP_BYPASS_OBSTACLES_X_POSITIVE_180_RETURN_ORIGIN;
            break;
        }
        if (my_abs(Yaw / 100) > 170 || my_abs(current_pose->x) < return_origin_distance)
        {
            return_origin_step_status = GOSTR_RETURN_ORIGIN_STEP;
            break;
        }
        linear_velocity = long_stra_vel;
        angular_velocity = 0;
        break;
    case COLLISION_LOOP_BYPASS_OBSTACLES_X_POSITIVE_180_RETURN_ORIGIN:
        if (obstacleSignal != none_obstacle)
        {
            return_origin_step_status = TURN_CLOCK_LOOP_COLLISION_X_POSITIVE_180_RETURN_ORIGIN;
            break;
        }
        if (my_abs(Yaw / 100 - temporary_yaw) > 10)
        {
            return_origin_step_status = BYPASS_OBSTACLES_X_POSITIVE_180_RETURN_ORIGIN;
            break;
        }
        linear_velocity = 0;
        angular_velocity = -return_origin_positive_start * turn_vel;
        break;
    case TURN_CLOCK_LOOP_COLLISION_X_POSITIVE_180_RETURN_ORIGIN:
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
            return_origin_step_status = COLLISION_LOOP_BYPASS_OBSTACLES_X_POSITIVE_180_RETURN_ORIGIN;
        }
        break;
    case X_MORE_LOOP_BYPASS_OBSTACLES_X_POSITIVE_180_RETURN_ORIGIN:
        if (obstacleSignal != none_obstacle)
        {
            return_origin_step_status = COLLISION_LOOP_BYPASS_OBSTACLES_X_NEGATIVE_5_RETURN_ORIGIN;
            break;
        }
        if (my_abs(Yaw / 100) > 170 || my_abs(Yaw / 100 - temporary_yaw) > 30)
        {
            return_origin_step_status = BYPASS_OBSTACLES_X_POSITIVE_180_RETURN_ORIGIN;
            break;
        }
        linear_velocity = 0;
        angular_velocity = return_origin_positive_start * turn_vel;
        break;

    case TURN_CLOCK_TARGET_YAW_5_RETURN_ORIGIN:
        if (obstacleSignal != none_obstacle)
        {
            return_origin_step_status = COLLISION_X_NEGATIVE_5_RETURN_ORIGIN;
            break;
        }
        if (my_abs(current_pose->x) < return_origin_distance)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            return_origin_step_status = GOSTR_RETURN_ORIGIN_STEP;
            break;
        }
        linear_velocity = long_stra_vel;
        angular_velocity = 0;
        break;
    case COLLISION_X_NEGATIVE_5_RETURN_ORIGIN:
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
            temporary_yaw = Yaw / 100;
            return_origin_step_status = TURN_CLOCK_X_NEGATIVE_5_RETURN_ORIGIN;
        }
        break;
    case TURN_CLOCK_X_NEGATIVE_5_RETURN_ORIGIN:
        if (obstacleSignal != none_obstacle)
        {
            return_origin_step_status = TURN_CLOCK_COLLISION_X_NEGATIVE_5_RETURN_ORIGIN;
            break;
        }
        if (my_abs(Yaw / 100 - temporary_yaw) > 10)
        {
            return_origin_step_status = BYPASS_OBSTACLES_X_NEGATIVE_5_RETURN_ORIGIN;
            break;
        }
        if (current_pose->y > 0)
        {
            return_origin_positive_start = 1;
        }
        else
        {
            return_origin_positive_start = -1;
        }
        linear_velocity = 0;
        angular_velocity = -return_origin_positive_start * turn_vel;
        break;
    case TURN_CLOCK_COLLISION_X_NEGATIVE_5_RETURN_ORIGIN:
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
            return_origin_step_status = TURN_CLOCK_X_NEGATIVE_5_RETURN_ORIGIN;
        }
        break;
    case BYPASS_OBSTACLES_X_NEGATIVE_5_RETURN_ORIGIN:
        last_position_y = current_pose->y;
        last_position_x = current_pose->x;
        return_origin_step_status = LOOP_BYPASS_OBSTACLES_X_NEGATIVE_5_RETURN_ORIGIN;
        break;
    case LOOP_BYPASS_OBSTACLES_X_NEGATIVE_5_RETURN_ORIGIN:
        if (obstacleSignal != none_obstacle)
        {
            temporary_yaw = Yaw / 100;
            return_origin_step_status = COLLISION_LOOP_BYPASS_OBSTACLES_X_NEGATIVE_5_RETURN_ORIGIN;
            break;
        }
        if (my_abs(last_position_y - current_pose->y) > lateral_move_distance || my_abs(last_position_x - current_pose->x) > lateral_move_distance )
        {
            temporary_yaw = Yaw / 100;
            return_origin_step_status = X_MORE_LOOP_BYPASS_OBSTACLES_X_NEGATIVE_5_RETURN_ORIGIN;
            break;
        }
        if (my_abs(Yaw / 100) < 10 || my_abs(current_pose->x) < return_origin_distance)
        {
            return_origin_step_status = GOSTR_RETURN_ORIGIN_STEP;
            break;
        }
        linear_velocity = long_stra_vel;
        angular_velocity = 0;
        break;
    case COLLISION_LOOP_BYPASS_OBSTACLES_X_NEGATIVE_5_RETURN_ORIGIN:
        if (obstacleSignal != none_obstacle)
        {
            return_origin_step_status = TURN_CLOCK_LOOP_COLLISION_X_NEGATIVE_5_RETURN_ORIGIN;
            break;
        }
        if (my_abs(Yaw / 100 - temporary_yaw) > 10)
        {
            return_origin_step_status = BYPASS_OBSTACLES_X_NEGATIVE_5_RETURN_ORIGIN;
            break;
        }
        linear_velocity = 0;
        angular_velocity = -return_origin_positive_start * turn_vel;
        break;
    case TURN_CLOCK_LOOP_COLLISION_X_NEGATIVE_5_RETURN_ORIGIN:
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
            return_origin_step_status = COLLISION_LOOP_BYPASS_OBSTACLES_X_NEGATIVE_5_RETURN_ORIGIN;
        }
        break;
    case X_MORE_LOOP_BYPASS_OBSTACLES_X_NEGATIVE_5_RETURN_ORIGIN:
        if (obstacleSignal != none_obstacle)
        {
            return_origin_step_status = COLLISION_LOOP_BYPASS_OBSTACLES_X_NEGATIVE_5_RETURN_ORIGIN;
            break;
        }
        if (my_abs(Yaw / 100) < 10 || my_abs(Yaw / 100 - temporary_yaw) > 30)
        {
            return_origin_step_status = BYPASS_OBSTACLES_X_NEGATIVE_5_RETURN_ORIGIN;
            break;
        }
        linear_velocity = 0;
        angular_velocity = return_origin_positive_start * turn_vel;
        break;
    }
    return complete_flag;
}

void  initOpen(AStar_Open *q)
{
    q->length = 0;
}

void  push(AStar_Open *q, AStar_Close cls[AStar_Height][AStar_Width], unsigned char x, unsigned char y, short g)
{
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

AStar_Close * shift(AStar_Open *q)
{
    return q->Array[--q->length];
}

void  initClose(AStar_Close cls[AStar_Height][AStar_Width], unsigned char sx, unsigned char sy, unsigned char dx, unsigned char dy)
{
    unsigned char i, j;
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

void  initGraph(bool maze[AStar_Height][AStar_Width], unsigned char sx, unsigned char sy, unsigned char dx, unsigned char dy)
{
    unsigned char i, j;
    AStar_srcX = sx;
    AStar_srcY = sy;
    AStar_dstX = dx;
    AStar_dstY = dy;
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

bool  astar()
{
    unsigned char i, curX, curY, surX, surY;
    float surG;
    AStar_Open q;
    AStar_Close *p;
    initOpen(&q);
    initClose(astar_close, AStar_srcX, AStar_srcY, AStar_dstX, AStar_dstY);
    astar_close[AStar_srcX][AStar_srcY].vis = 1;
    push(&q, astar_close, AStar_srcX, AStar_srcY, 0);
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

AStar_Close * getShortest()
{
    bool result = astar();
    AStar_Close *p, *t, *q = NULL;
    switch (result)
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

unsigned char  printShortest()
{
    AStar_Close *p;
    step = 0;
    p = getShortest();
    AStar_start = p;
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
            if (motionSteps > 1)
            {
               //cout << plansteps[motionSteps - 2] << endl;
               //cout << plansteps[motionSteps - 1] << endl;
            }
            if (motionSteps > 95)
            {
                return step;
            }
           //cout << '(' << (short)p->cur->x << ',' << (short)p->cur->y << ')' << endl;
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

unsigned char  AStarReturnOrigin(POSE *current_pose, unsigned char obstacleSignal)
{
    unsigned char complete_flag = 0;
    bool canmove = true;
    short goal_robot_x = 0;
    short goal_robot_y = 0;
    short firsttrap = 0;
    short secondtrap = 0;
    short thirdtrap = 0;
    short forthtrap = 0;
    short astar_underboundary;
    short astar_onboundary;
    short leftboundary = 0;
    short rightboundary = 0;
    bool end_x = false;
    short a_map_x = (current_pose->x + MAPHEIGHT / 2) / (ZoomMultiple * GRIDWIDTH);
    short a_map_y = (current_pose->y + MAPHEIGHT / 2) / (ZoomMultiple * GRIDWIDTH);
    short goal_map_x = (goal_robot_x + MAPHEIGHT / 2) / (ZoomMultiple * GRIDWIDTH);
    short goal_map_y = (goal_robot_y + MAPHEIGHT / 2) / (ZoomMultiple * GRIDWIDTH);
    short i, j, k, ij, jk, ik;
    complete_flag = 0;
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
                                if (gridmap.map[ZoomMultiple * ij + ik + i][ZoomMultiple * k + ik] == 0)
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

unsigned char  AStarNotMotionReturnOrigin(POSE *current_pose, unsigned char obstacleSignal)
{
    int Yaw;
    unsigned char complete_flag = 0;
    Yaw = current_pose->orientation;
    switch (a_star_not_motion_status)
    {
    case 0:
        temporary_wheel_pulse_r = wheel_pulse_r;
        a_star_not_motion_status = START_PLAN_ASTAR_MOTION_GOSTR_RETURN;
        break;
    case START_PLAN_ASTAR_MOTION_GOSTR_RETURN:
        if (obstacleSignal != none_obstacle)
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
        if (my_abs(turn_start_x - current_pose->x) > collision_backward_distance || my_abs(turn_start_y - current_pose->y) > collision_backward_distance)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            turn_start_update = 0;
            temporary_yaw = Yaw / 100;
            a_star_not_motion_status = TURN_CLOCK_START_PLAN_ASTAR_MOTION_GOSTR_RETURN;
        }
        break;
    case TURN_CLOCK_START_PLAN_ASTAR_MOTION_GOSTR_RETURN:
        if (my_abs(temporary_yaw - Yaw / 100) > 15)
        {
            linear_velocity = 0;
            angular_velocity = 0;
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
        if (my_abs(turn_start_x - current_pose->x) > collision_backward_distance || my_abs(turn_start_y - current_pose->y) > collision_backward_distance)
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

unsigned char  AStarMotionReturnOrigin(POSE *current_pose, unsigned char obstacleSignal)
{
    int Yaw;
    unsigned char complete_flag = 0;
    Yaw = current_pose->orientation;
    switch (a_star_motion_return_origin_status)
    {
    case 0:
        a_star_motion_return_origin_status = ASTAR_MOTION_GOSTR_RETURN;
        break;
    case ASTAR_MOTION_GOSTR_RETURN:
        if (my_abs(current_pose->x) > return_origin_distance || my_abs(current_pose->y) > return_origin_distance)
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
            if (AStarMotionNumber > 10)
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
        if ((my_abs(current_pose->x) < return_origin_distance && my_abs(current_pose->y) < return_origin_distance) || startMotionStep >= motionSteps)
        {
            a_star_motion_return_origin_status = A_STAR_COMPLETED;
            break;
        }
        if (motionSteps > 90)
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
        if (my_abs(Yaw / 100) <= 45 || (Yaw / 100 > 45 && Yaw / 100 < 130))
        {
            a_star_motion_return_origin_status = LESS_45_A_DIRECT_START_PLAN_ASTAR_MOTION_GOSTR_RETURN;
        }
        else if (my_abs(Yaw / 100) > 140 || (Yaw / 100 >= -140 && Yaw / 100 < -45))
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
        if (Yaw / 100 < 138 && Yaw / 100 > 132)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            temporary_wheel_pulse_l = wheel_pulse_l;
            a_star_motion_return_origin_status = GO_A_DIRECT_START_PLAN_ASTAR_MOTION_GOSTR_RETURN;
            break;
        }
        if (obstacleSignal != none_obstacle)
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
        if (Yaw / 100 < 138 && Yaw / 100 > 132)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            temporary_wheel_pulse_l = wheel_pulse_l;
            a_star_motion_return_origin_status = GO_A_DIRECT_START_PLAN_ASTAR_MOTION_GOSTR_RETURN;
            break;
        }
        if (obstacleSignal != none_obstacle)
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
        if (obstacleSignal != none_obstacle)
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
        if (my_abs(Yaw / 100) < 175 && Yaw / 100 >= 0)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            a_star_motion_return_origin_status = LESS_45_B_DIRECT_START_PLAN_ASTAR_MOTION_GOSTR_RETURN;
        }
        else if (my_abs(Yaw / 100) < 175 && Yaw / 100 < 0)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            a_star_motion_return_origin_status = MORE_45_B_DIRECT_START_PLAN_ASTAR_MOTION_GOSTR_RETURN;
        }
        else
        {
            temporary_wheel_pulse_l = wheel_pulse_l;
            a_star_motion_return_origin_status = GO_B_DIRECT_START_PLAN_ASTAR_MOTION_GOSTR_RETURN;
        }
        break;
    case LESS_45_B_DIRECT_START_PLAN_ASTAR_MOTION_GOSTR_RETURN:
        if (my_abs(Yaw / 100) > 178)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            temporary_wheel_pulse_l = wheel_pulse_l;
            a_star_motion_return_origin_status = GO_B_DIRECT_START_PLAN_ASTAR_MOTION_GOSTR_RETURN;
            break;
        }
        if (obstacleSignal != none_obstacle)
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
        if (my_abs(Yaw / 100) > 178)
        {
            linear_velocity = 0;
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
        if (obstacleSignal != none_obstacle)
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
        if (my_abs(Yaw / 100) > 140 || (Yaw / 100 > 45 && Yaw / 100 <= 140))
        {
            linear_velocity = 0;
            angular_velocity = 0;
            a_star_motion_return_origin_status = LESS_45_C_DIRECT_START_PLAN_ASTAR_MOTION_GOSTR_RETURN;
        }
        else if (my_abs(Yaw / 100) <= 45 || (Yaw / 100 > -130 && Yaw / 100 < -45))
        {
            linear_velocity = 0;
            angular_velocity = 0;
            a_star_motion_return_origin_status = MORE_45_C_DIRECT_START_PLAN_ASTAR_MOTION_GOSTR_RETURN;
        }
        else
        {
            temporary_wheel_pulse_l = wheel_pulse_l;
            a_star_motion_return_origin_status = GO_A_DIRECT_START_PLAN_ASTAR_MOTION_GOSTR_RETURN;
        }
        break;
    case LESS_45_C_DIRECT_START_PLAN_ASTAR_MOTION_GOSTR_RETURN:
        if (Yaw / 100 > -138 && Yaw / 100 < -132)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            temporary_wheel_pulse_l = wheel_pulse_l;
            a_star_motion_return_origin_status = GO_A_DIRECT_START_PLAN_ASTAR_MOTION_GOSTR_RETURN;
            break;
        }
        if (obstacleSignal != none_obstacle)
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
        if (Yaw / 100 > -138 && Yaw / 100 < -132)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            temporary_wheel_pulse_l = wheel_pulse_l;
            a_star_motion_return_origin_status = GO_A_DIRECT_START_PLAN_ASTAR_MOTION_GOSTR_RETURN;
            break;
        }
        if (obstacleSignal != none_obstacle)
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
        if (my_abs(Yaw / 100) > 95)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            a_star_motion_return_origin_status = LESS_45_D_DIRECT_START_PLAN_ASTAR_MOTION_GOSTR_RETURN;
        }
        else if (my_abs(Yaw / 100) < 85 || (Yaw / 100 >= 85 && Yaw / 100 <= 95))
        {
            linear_velocity = 0;
            angular_velocity = 0;
            a_star_motion_return_origin_status = MORE_45_D_DIRECT_START_PLAN_ASTAR_MOTION_GOSTR_RETURN;
        }
        else
        {
            temporary_wheel_pulse_l = wheel_pulse_l;
            a_star_motion_return_origin_status = GO_B_DIRECT_START_PLAN_ASTAR_MOTION_GOSTR_RETURN;
        }
        break;
    case LESS_45_D_DIRECT_START_PLAN_ASTAR_MOTION_GOSTR_RETURN:
        if (Yaw / 100 > -138 && Yaw / 100 < -132)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            temporary_wheel_pulse_l = wheel_pulse_l;
            a_star_motion_return_origin_status = GO_B_DIRECT_START_PLAN_ASTAR_MOTION_GOSTR_RETURN;
            break;
        }
        if (obstacleSignal != none_obstacle)
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
        if (Yaw / 100 > -138 && Yaw / 100 < -132)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            temporary_wheel_pulse_l = wheel_pulse_l;
            a_star_motion_return_origin_status = GO_B_DIRECT_START_PLAN_ASTAR_MOTION_GOSTR_RETURN;
            break;
        }
        if (obstacleSignal != none_obstacle)
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
        if (my_abs(Yaw / 100) >= 135 || (Yaw / 100 < -135 && Yaw / 100 > -50))
        {
            linear_velocity = 0;
            angular_velocity = 0;
            a_star_motion_return_origin_status = LESS_45_E_DIRECT_START_PLAN_ASTAR_MOTION_GOSTR_RETURN;
        }
        else if (my_abs(Yaw / 100) < 40 || (Yaw / 100 >= 40 && Yaw / 100 < 135))
        {
            linear_velocity = 0;
            angular_velocity = 0;
            a_star_motion_return_origin_status = MORE_45_E_DIRECT_START_PLAN_ASTAR_MOTION_GOSTR_RETURN;
        }
        else
        {
            temporary_wheel_pulse_l = wheel_pulse_l;
            a_star_motion_return_origin_status = GO_A_DIRECT_START_PLAN_ASTAR_MOTION_GOSTR_RETURN;
        }
        break;
    case LESS_45_E_DIRECT_START_PLAN_ASTAR_MOTION_GOSTR_RETURN:
        if (Yaw / 100 < -42 && Yaw / 100 > -48)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            temporary_wheel_pulse_l = wheel_pulse_l;
            a_star_motion_return_origin_status = GO_A_DIRECT_START_PLAN_ASTAR_MOTION_GOSTR_RETURN;
            break;
        }
        if (obstacleSignal != none_obstacle)
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
        if (Yaw / 100 < -42 && Yaw / 100 > -48)
        {
            linear_velocity = 0;
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
        if (my_abs(Yaw / 100) > 5 && Yaw / 100 < 0)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            a_star_motion_return_origin_status = LESS_45_F_DIRECT_START_PLAN_ASTAR_MOTION_GOSTR_RETURN;
        }
        else if (my_abs(Yaw / 100) > 5 && Yaw / 100 > 0)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            a_star_motion_return_origin_status = MORE_45_F_DIRECT_START_PLAN_ASTAR_MOTION_GOSTR_RETURN;
        }
        else
        {
            temporary_wheel_pulse_l = wheel_pulse_l;
            a_star_motion_return_origin_status = GO_B_DIRECT_START_PLAN_ASTAR_MOTION_GOSTR_RETURN;
        }
        break;
    case LESS_45_F_DIRECT_START_PLAN_ASTAR_MOTION_GOSTR_RETURN:
        if (my_abs(Yaw / 100) < 3)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            temporary_wheel_pulse_l = wheel_pulse_l;
            a_star_motion_return_origin_status = GO_B_DIRECT_START_PLAN_ASTAR_MOTION_GOSTR_RETURN;
            break;
        }
        if (obstacleSignal != none_obstacle)
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
        if (my_abs(Yaw / 100) < 3)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            temporary_wheel_pulse_l = wheel_pulse_l;
            a_star_motion_return_origin_status = GO_B_DIRECT_START_PLAN_ASTAR_MOTION_GOSTR_RETURN;
            break;
        }
        if (obstacleSignal != none_obstacle)
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
        if (my_abs(Yaw / 100) <= 40 || (Yaw / 100 < -40 && Yaw / 100 > -135))
        {
            linear_velocity = 0;
            angular_velocity = 0;
            a_star_motion_return_origin_status = LESS_45_G_DIRECT_START_PLAN_ASTAR_MOTION_GOSTR_RETURN;
        }
        else if (my_abs(Yaw / 100) >= 135 || (Yaw / 100 > 50 && Yaw / 100 < 135))
        {
            linear_velocity = 0;
            angular_velocity = 0;
            a_star_motion_return_origin_status = MORE_45_G_DIRECT_START_PLAN_ASTAR_MOTION_GOSTR_RETURN;
        }
        else
        {
            temporary_wheel_pulse_l = wheel_pulse_l;
            a_star_motion_return_origin_status = GO_A_DIRECT_START_PLAN_ASTAR_MOTION_GOSTR_RETURN;
        }
        break;
    case LESS_45_G_DIRECT_START_PLAN_ASTAR_MOTION_GOSTR_RETURN:
        if (Yaw / 100 < 48 && Yaw / 100 > 42)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            temporary_wheel_pulse_l = wheel_pulse_l;
            a_star_motion_return_origin_status = GO_A_DIRECT_START_PLAN_ASTAR_MOTION_GOSTR_RETURN;
            break;
        }
        if (obstacleSignal != none_obstacle)
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
        if (Yaw / 100 < 48 && Yaw / 100 > 42)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            temporary_wheel_pulse_l = wheel_pulse_l;
            a_star_motion_return_origin_status = GO_A_DIRECT_START_PLAN_ASTAR_MOTION_GOSTR_RETURN;
            break;
        }
        if (obstacleSignal != none_obstacle)
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
        if (my_abs(Yaw / 100) < 85)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            a_star_motion_return_origin_status = LESS_45_H_DIRECT_START_PLAN_ASTAR_MOTION_GOSTR_RETURN;
        }
        else if (my_abs(Yaw / 100) > 95 || (Yaw / 100 >= -95 && Yaw / 100 <= -85))
        {
            linear_velocity = 0;
            angular_velocity = 0;
            a_star_motion_return_origin_status = MORE_45_H_DIRECT_START_PLAN_ASTAR_MOTION_GOSTR_RETURN;
        }
        else
        {
            temporary_wheel_pulse_l = wheel_pulse_l;
            a_star_motion_return_origin_status = GO_B_DIRECT_START_PLAN_ASTAR_MOTION_GOSTR_RETURN;
        }
        break;
    case LESS_45_H_DIRECT_START_PLAN_ASTAR_MOTION_GOSTR_RETURN:
        if (Yaw / 100 > 87 && Yaw / 100 < 93)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            temporary_wheel_pulse_l = wheel_pulse_l;
            a_star_motion_return_origin_status = GO_B_DIRECT_START_PLAN_ASTAR_MOTION_GOSTR_RETURN;
            break;
        }
        if (obstacleSignal != none_obstacle)
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
        if (Yaw / 100 > 87 && Yaw / 100 < 93)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            temporary_wheel_pulse_l = wheel_pulse_l;
            a_star_motion_return_origin_status = GO_B_DIRECT_START_PLAN_ASTAR_MOTION_GOSTR_RETURN;
            break;
        }
        if (obstacleSignal != none_obstacle)
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

unsigned char  AStarCollision(POSE *current_pose, unsigned char obstacleSignal)
{
    int Yaw;
    unsigned char complete_flag = 0;
    Yaw = current_pose->orientation;
    switch (a_star_collision_status)
    {
    case 0:
        a_star_collision_status = A_STAR_COLLISION;
        break;
    case A_STAR_COLLISION:
        log_debug("A_STAR_COLLISION_GOSTR");
        if (my_abs(current_pose->x) > return_origin_distance || my_abs(current_pose->y) > return_origin_distance)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            a_star_collision_status = START_A_STAR_COLLISION;
        }
        else
        {
            log_debug("A_STAR_COLLISION_COMPLETED");
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
        if (my_abs(turn_start_x - current_pose->x) > star_collision_backward || my_abs(turn_start_y - current_pose->y) > star_collision_backward)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            turn_start_update = 0;
            temporary_yaw = Yaw / 100;
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
        if (my_abs(temporary_yaw - Yaw / 100) > 30)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            a_star_collision_status = GO_LEFT_OBSTACLE_START_A_STAR_COLLISION;
            break;
        }
        if (obstacleSignal != none_obstacle)
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
        if (obstacleSignal != none_obstacle)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            turn_start_update = 0;
            a_star_collision_total++;
            a_star_collision_status = BACK_START_A_STAR_COLLISION;
            break;
        }
        if (a_star_collision_total > 10)
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
        if (my_abs(temporary_yaw - Yaw / 100) > 30)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            a_star_collision_status = GO_RIGHT_OBSTACLE_START_A_STAR_COLLISION;
            break;
        }
        if (obstacleSignal != none_obstacle)
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
        if (obstacleSignal != none_obstacle)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            turn_start_update = 0;
            a_star_collision_total++;
            a_star_collision_status = BACK_START_A_STAR_COLLISION;
            break;
        }
        if (a_star_collision_total > 10)
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
        if (my_abs(temporary_yaw - Yaw / 100) > 45)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            a_star_collision_status = GO_FRONT_OBSTACLE_START_A_STAR_COLLISION;
            break;
        }
        if (obstacleSignal != none_obstacle)
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
        if (obstacleSignal != none_obstacle)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            turn_start_update = 0;
            a_star_collision_total++;
            a_star_collision_status = BACK_START_A_STAR_COLLISION;
            break;
        }
        if (a_star_collision_total > 5)
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

unsigned char  CliffRuningWorkStep(POSE *current_pose, CLIFFADCVALUE *cliff_value, unsigned char obstacleSignal)
{

    int Yaw;
    unsigned char complete_flag = 0;
    Yaw = current_pose->orientation;
    log_debug("CliffRuningWorkStep_cliff_running_step_status");
    switch (cliff_running_step_status)
    {
    case 0:
        cliff_running_step_status = GOSTR_CLIFF_RUNNING_STEP;
        linear_velocity = 0;
        angular_velocity = 0;
        break;
    case GOSTR_CLIFF_RUNNING_STEP:
        log_debug("cliff_running_step_status");
        linear_velocity = 0;
        angular_velocity = 0;
        totalCDN++;
        if (cliff_value->cliffValue1 == left_cliff && cliff_value->cliffValue3 == right_cliff)
        {
            if (my_abs(Yaw / 100) <= 90 && selectside == 'R')
            {
                cliff_running_step_status = LESS_90_R_RIGHT_CLIFF_RUNNING_STEP;
                break;
            }
            else if (my_abs(Yaw / 100) > 90 && selectside == 'R')
            {
                cliff_running_step_status = GREATER_90_R_LEFT_CLIFF_RUNNING_STEP;
                break;
            }
            else if (my_abs(Yaw / 100) > 90 && selectside == 'L')
            {
                cliff_running_step_status = GREATER_90_L_RIGHT_CLIFF_RUNNING_STEP;
                break;
            }
            else if (my_abs(Yaw / 100) <= 90 && selectside == 'L')
            {
                cliff_running_step_status = LESS_90_L_LEFT_CLIFF_RUNNING_STEP;
                break;
            }
            else
            {
                cliff_running_step_status = ORIGIN_FRONT_CLIFF_RUNNING_STEP;
            }
            break;
        }
        else if (cliff_value->cliffValue3 == right_cliff)
        {
            if (my_abs(Yaw / 100) <= 90 && selectside == 'R')
            {
                cliff_running_step_status = LESS_90_R_RIGHT_CLIFF_RUNNING_STEP;
                break;
            }
            else if (my_abs(Yaw / 100) > 90 && selectside == 'R')
            {
                cliff_running_step_status = GREATER_90_R_FRONT_CLIFF_RUNNING_STEP;
                break;
            }
            else if (my_abs(Yaw / 100) > 90 && selectside == 'L')
            {
                cliff_running_step_status = GREATER_90_L_RIGHT_CLIFF_RUNNING_STEP;
                break;
            }
            else if (my_abs(Yaw / 100) <= 90 && selectside == 'L')
            {
                cliff_running_step_status = LESS_90_L_FRONT_CLIFF_RUNNING_STEP;
                break;
            }
            else
            {
                cliff_running_step_status = ORIGIN_FRONT_CLIFF_RUNNING_STEP;
            }
            break;
        }
        else if (cliff_value->cliffValue1 == left_cliff)
        {
            if (my_abs(Yaw / 100) <= 90 && selectside == 'R')
            {
                cliff_running_step_status = LESS_90_R_FRONT_CLIFF_RUNNING_STEP;
                break;
            }
            else if (my_abs(Yaw / 100) > 90 && selectside == 'R')
            {
                cliff_running_step_status = GREATER_90_R_LEFT_CLIFF_RUNNING_STEP;
                break;
            }
            else if (my_abs(Yaw / 100) > 90 && selectside == 'L')
            {
                cliff_running_step_status = GREATER_90_L_FRONT_CLIFF_RUNNING_STEP;
                break;
            }
            else if (my_abs(Yaw / 100) <= 90 && selectside == 'L')
            {
                cliff_running_step_status = LESS_90_L_LEFT_CLIFF_RUNNING_STEP;
                break;
            }
            else
            {
                cliff_running_step_status = ORIGIN_FRONT_CLIFF_RUNNING_STEP;
            }
            break;
        }
        else if (cliff_value->cliffValue2 == front_cliff)
        {
            if (my_abs(Yaw / 100) <= 90 && selectside == 'R')
            {
                cliff_running_step_status = LESS_90_R_FRONT_CLIFF_RUNNING_STEP;
                break;
            }
            else if (my_abs(Yaw / 100) > 90 && selectside == 'R')
            {
                cliff_running_step_status = GREATER_90_R_FRONT_CLIFF_RUNNING_STEP;
                break;
            }
            else if (my_abs(Yaw / 100) > 90 && selectside == 'L')
            {
                cliff_running_step_status = GREATER_90_L_FRONT_CLIFF_RUNNING_STEP;
                break;
            }
            else if (my_abs(Yaw / 100) <= 90 && selectside == 'L')
            {
                cliff_running_step_status = LESS_90_L_FRONT_CLIFF_RUNNING_STEP;
                break;
            }
            else
            {
                cliff_running_step_status = ORIGIN_FRONT_CLIFF_RUNNING_STEP;
            }
            break;
        }
        else
        {
            cliff_running_step_status = CLIFF_COMPLETE;
        }
        break;
    case LESS_90_R_FRONT_CLIFF_RUNNING_STEP:
        if (clill_start_update == 0)
        {
            turn_start_x = current_pose->x;
            turn_start_y = current_pose->y;
            clill_start_update = 1;
        }
        linear_velocity = -long_stra_vel;
        angular_velocity = 0;
        if ((my_abs(turn_start_x - current_pose->x) > lateral_move_distance / 4 || my_abs(turn_start_y - current_pose->y) > lateral_move_distance / 4) && cliff_value->cliffValue0 == 0)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            temporary_yaw = Yaw / 100;
            cliff_running_step_status = TURN_LESS_90_R_FRONT_CLIFF_RUNNING_STEP;
            break;
        }
        break;
    case TURN_LESS_90_R_FRONT_CLIFF_RUNNING_STEP:
        linear_velocity = 0;
        angular_velocity = -turn_vel;
        if (CliffDelimmaNumber > 4)
        {
            cliff_running_step_status = INTERRUPT_AND_STOP;
            break;
        }
        if (cliff_value->cliffValue0 == 1)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            cliff_running_step_status = LESS_90_R_FRONT_CLIFF_RUNNING_STEP;
            break;
        }
        if (my_abs(temporary_yaw - Yaw / 100) > 45)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            CliffDelimmaNumber++;
            last_position_x = current_pose->x;
            last_position_y = current_pose->y;
            cliff_running_step_status = GOSTR_LESS_90_R_FRONT_CLIFF_RUNNING_STEP;
            break;
        }
        if (obstacleSignal != none_obstacle)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            cliff_running_step_status = COLLISION_TURN_LESS_90_R_FRONT_CLIFF_RUNNING_STEP;
            break;
        }
        break;
    case COLLISION_TURN_LESS_90_R_FRONT_CLIFF_RUNNING_STEP:
        if (clill_start_update == 0)
        {
            turn_start_x = current_pose->x;
            turn_start_y = current_pose->y;
            clill_start_update = 1;
        }
        linear_velocity = -long_stra_vel;
        angular_velocity = 0;
        if (my_abs(turn_start_x - current_pose->x) > turn_backward_distance || my_abs(turn_start_y - current_pose->y) > turn_backward_distance)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            clill_start_update = 0;
            cliff_running_step_status = TURN_LESS_90_R_FRONT_CLIFF_RUNNING_STEP;
            break;
        }
        break;
    case GOSTR_LESS_90_R_FRONT_CLIFF_RUNNING_STEP:
        linear_velocity = long_stra_vel;
        angular_velocity = 0;
        if (obstacleSignal != none_obstacle)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            cliff_running_step_status = COLLISION_GOSTR_LESS_90_R_FRONT_CLIFF_RUNNING_STEP;
            break;
        }
        if (cliff_value->cliffValue0 == 1)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            cliff_running_step_status = LESS_90_R_FRONT_CLIFF_RUNNING_STEP;
            break;
        }
        if (my_abs(last_position_x - current_pose->x) > lateral_move_distance / 4 || my_abs(last_position_y - current_pose->y) > lateral_move_distance / 4)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            cliff_running_step_status = CLIFF_COMPLETE;
            break;
        }
        break;
    case COLLISION_GOSTR_LESS_90_R_FRONT_CLIFF_RUNNING_STEP:
        if (clill_start_update == 0)
        {
            turn_start_x = current_pose->x;
            turn_start_y = current_pose->y;
            clill_start_update = 1;
        }
        linear_velocity = -long_stra_vel;
        angular_velocity = 0;
        if (my_abs(turn_start_x - current_pose->x) > turn_backward_distance || my_abs(turn_start_y - current_pose->y) > turn_backward_distance)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            clill_start_update = 0;
            temporary_yaw = Yaw / 100;
            cliff_running_step_status = TURN_GOSTR_LESS_90_R_FRONT_CLIFF_RUNNING_STEP;
            break;
        }
        break;
    case TURN_GOSTR_LESS_90_R_FRONT_CLIFF_RUNNING_STEP:
        linear_velocity = 0;
        angular_velocity = -turn_vel;
        if (my_abs(temporary_yaw - Yaw / 100) > 15)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            cliff_running_step_status = GOSTR_LESS_90_R_FRONT_CLIFF_RUNNING_STEP;
            break;
        }
        if (obstacleSignal != none_obstacle)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            cliff_running_step_status = COLLISION_TURN_GOSTR_LESS_90_R_FRONT_CLIFF_RUNNING_STEP;
            break;
        }
        if (cliff_value->cliffValue0 == 1)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            cliff_running_step_status = LESS_90_R_FRONT_CLIFF_RUNNING_STEP;
            break;
        }
        break;
    case COLLISION_TURN_GOSTR_LESS_90_R_FRONT_CLIFF_RUNNING_STEP:
        if (clill_start_update == 0)
        {
            turn_start_x = current_pose->x;
            turn_start_y = current_pose->y;
            clill_start_update = 1;
        }
        linear_velocity = -long_stra_vel;
        angular_velocity = 0;
        if (my_abs(turn_start_x - current_pose->x) > turn_backward_distance || my_abs(turn_start_y - current_pose->y) > turn_backward_distance)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            clill_start_update = 0;
            cliff_running_step_status = TURN_GOSTR_LESS_90_R_FRONT_CLIFF_RUNNING_STEP;
            break;
        }
        break;

    case GREATER_90_R_FRONT_CLIFF_RUNNING_STEP:
        if (clill_start_update == 0)
        {
            turn_start_x = current_pose->x;
            turn_start_y = current_pose->y;
            clill_start_update = 1;
        }
        linear_velocity = -long_stra_vel;
        angular_velocity = 0;
        if ((my_abs(turn_start_x - current_pose->x) > lateral_move_distance / 4 || my_abs(turn_start_y - current_pose->y) > lateral_move_distance / 4) &&
            cliff_value->cliffValue0 == 0)
			
        {
            linear_velocity = 0;
            angular_velocity = 0;
            temporary_yaw = Yaw / 100;
            cliff_running_step_status = TURN_GREATER_90_R_FRONT_CLIFF_RUNNING_STEP;
            break;
        }
        break;
    case TURN_GREATER_90_R_FRONT_CLIFF_RUNNING_STEP:
        linear_velocity = 0;
        angular_velocity = turn_vel;
        if (CliffDelimmaNumber > 4)
        {
            cliff_running_step_status = INTERRUPT_AND_STOP;
            break;
        }
        if (cliff_value->cliffValue0 == 1)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            cliff_running_step_status = GREATER_90_R_FRONT_CLIFF_RUNNING_STEP;
            break;
        }
        if (obstacleSignal != none_obstacle)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            cliff_running_step_status = COLLISION_TURN_GREATER_90_R_FRONT_CLIFF_RUNNING_STEP;
            break;
        }
        if (my_abs(temporary_yaw - Yaw / 100) > 45)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            CliffDelimmaNumber++;
            last_position_x = current_pose->x;
            last_position_y = current_pose->y;
            cliff_running_step_status = GOSTR_GREATER_90_R_FRONT_CLIFF_RUNNING_STEP;
            break;
        }
        break;
    case COLLISION_TURN_GREATER_90_R_FRONT_CLIFF_RUNNING_STEP:
        if (clill_start_update == 0)
        {
            turn_start_x = current_pose->x;
            turn_start_y = current_pose->y;
            clill_start_update = 1;
        }
        linear_velocity = -long_stra_vel;
        angular_velocity = 0;
        if (my_abs(turn_start_x - current_pose->x) > turn_backward_distance || my_abs(turn_start_y - current_pose->y) > turn_backward_distance)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            clill_start_update = 0;
            cliff_running_step_status = TURN_GREATER_90_R_FRONT_CLIFF_RUNNING_STEP;
            break;
        }
        break;
    case GOSTR_GREATER_90_R_FRONT_CLIFF_RUNNING_STEP:
        linear_velocity = long_stra_vel;
        angular_velocity = 0;
        if (cliff_value->cliffValue0 == 1)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            cliff_running_step_status = GREATER_90_R_FRONT_CLIFF_RUNNING_STEP;
            break;
        }
        if (obstacleSignal != none_obstacle)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            cliff_running_step_status = COLLISION_GOSTR_GREATER_90_R_FRONT_CLIFF_RUNNING_STEP;
            break;
        }
        if (my_abs(last_position_x - current_pose->x) > lateral_move_distance / 4 || my_abs(last_position_y - current_pose->y) > lateral_move_distance / 4)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            cliff_running_step_status = CLIFF_COMPLETE;
            break;
        }
        break;
    case COLLISION_GOSTR_GREATER_90_R_FRONT_CLIFF_RUNNING_STEP:
        if (clill_start_update == 0)
        {
            turn_start_x = current_pose->x;
            turn_start_y = current_pose->y;
            clill_start_update = 1;
        }
        linear_velocity = -long_stra_vel;
        angular_velocity = 0;
        if (my_abs(turn_start_x - current_pose->x) > turn_backward_distance || my_abs(turn_start_y - current_pose->y) > turn_backward_distance)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            clill_start_update = 0;
            temporary_yaw = Yaw / 100;
            cliff_running_step_status = TURN_GOSTR_GREATER_90_R_FRONT_CLIFF_RUNNING_STEP;
            break;
        }
        break;
    case TURN_GOSTR_GREATER_90_R_FRONT_CLIFF_RUNNING_STEP:
        linear_velocity = 0;
        angular_velocity = turn_vel;
        if (my_abs(temporary_yaw - Yaw / 100) > 15)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            cliff_running_step_status = GOSTR_GREATER_90_R_FRONT_CLIFF_RUNNING_STEP;
            break;
        }
        if (obstacleSignal != none_obstacle)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            cliff_running_step_status = COLLISION_TURN_GOSTR_GREATER_90_R_FRONT_CLIFF_RUNNING_STEP;
            break;
        }
        if (cliff_value->cliffValue0 == 1)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            cliff_running_step_status = GREATER_90_R_FRONT_CLIFF_RUNNING_STEP;
            break;
        }
        break;
    case COLLISION_TURN_GOSTR_GREATER_90_R_FRONT_CLIFF_RUNNING_STEP:
        if (clill_start_update == 0)
        {
            turn_start_x = current_pose->x;
            turn_start_y = current_pose->y;
            clill_start_update = 1;
        }
        linear_velocity = -long_stra_vel;
        angular_velocity = 0;
        if (my_abs(turn_start_x - current_pose->x) > turn_backward_distance || my_abs(turn_start_y - current_pose->y) > turn_backward_distance)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            clill_start_update = 0;
            cliff_running_step_status = TURN_GOSTR_GREATER_90_R_FRONT_CLIFF_RUNNING_STEP;
            break;
        }
        break;

    case GREATER_90_L_FRONT_CLIFF_RUNNING_STEP:
        if (clill_start_update == 0)
        {
            turn_start_x = current_pose->x;
            turn_start_y = current_pose->y;
            clill_start_update = 1;
        }
        linear_velocity = -long_stra_vel;
        angular_velocity = 0;
        if ((my_abs(turn_start_x - current_pose->x) > lateral_move_distance / 4 || my_abs(turn_start_y - current_pose->y) > lateral_move_distance / 4) 
			&&
            cliff_value->cliffValue0 == 0
		)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            temporary_yaw = Yaw / 100;
            cliff_running_step_status = TURN_GREATER_90_L_FRONT_CLIFF_RUNNING_STEP;
            break;
        }
        break;
    case TURN_GREATER_90_L_FRONT_CLIFF_RUNNING_STEP:
        linear_velocity = 0;
        angular_velocity = -turn_vel;
        if (CliffDelimmaNumber > 4)
        {
            cliff_running_step_status = INTERRUPT_AND_STOP;
            break;
        }
        if (cliff_value->cliffValue0 == 1)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            cliff_running_step_status = GREATER_90_L_FRONT_CLIFF_RUNNING_STEP;
            break;
        }
        if (obstacleSignal != none_obstacle)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            cliff_running_step_status = COLLISION_TURN_GREATER_90_L_FRONT_CLIFF_RUNNING_STEP;
            break;
        }
        if (my_abs(temporary_yaw - Yaw / 100) > 45)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            CliffDelimmaNumber++;
            last_position_x = current_pose->x;
            last_position_y = current_pose->y;
            cliff_running_step_status = GOSTR_GREATER_90_L_FRONT_CLIFF_RUNNING_STEP;
            break;
        }
        break;
    case COLLISION_TURN_GREATER_90_L_FRONT_CLIFF_RUNNING_STEP:
        if (clill_start_update == 0)
        {
            turn_start_x = current_pose->x;
            turn_start_y = current_pose->y;
            clill_start_update = 1;
        }
        linear_velocity = -long_stra_vel;
        angular_velocity = 0;
        if (my_abs(turn_start_x - current_pose->x) > turn_backward_distance || my_abs(turn_start_y - current_pose->y) > turn_backward_distance)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            clill_start_update = 0;
            cliff_running_step_status = TURN_GREATER_90_L_FRONT_CLIFF_RUNNING_STEP;
            break;
        }
        break;
    case GOSTR_GREATER_90_L_FRONT_CLIFF_RUNNING_STEP:
        linear_velocity = long_stra_vel;
        angular_velocity = 0;
        if (cliff_value->cliffValue0 == 1)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            cliff_running_step_status = GREATER_90_L_FRONT_CLIFF_RUNNING_STEP;
            break;
        }
        if (obstacleSignal != none_obstacle)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            cliff_running_step_status = COLLISION_GOSTR_GREATER_90_L_FRONT_CLIFF_RUNNING_STEP;
            break;
        }
        if (my_abs(last_position_x - current_pose->x) > lateral_move_distance / 4 || my_abs(last_position_y - current_pose->y) > lateral_move_distance / 4)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            cliff_running_step_status = CLIFF_COMPLETE;
            break;
        }
        break;
    case COLLISION_GOSTR_GREATER_90_L_FRONT_CLIFF_RUNNING_STEP:
        if (clill_start_update == 0)
        {
            turn_start_x = current_pose->x;
            turn_start_y = current_pose->y;
            clill_start_update = 1;
        }
        linear_velocity = -long_stra_vel;
        angular_velocity = 0;
        if (my_abs(turn_start_x - current_pose->x) > turn_backward_distance || my_abs(turn_start_y - current_pose->y) > turn_backward_distance)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            clill_start_update = 0;
            temporary_yaw = Yaw / 100;
            cliff_running_step_status = TURN_GOSTR_GREATER_90_L_FRONT_CLIFF_RUNNING_STEP;
            break;
        }
        break;
    case TURN_GOSTR_GREATER_90_L_FRONT_CLIFF_RUNNING_STEP:
        linear_velocity = 0;
        angular_velocity = -turn_vel;
        if (my_abs(temporary_yaw - Yaw / 100) > 15)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            cliff_running_step_status = GOSTR_GREATER_90_L_FRONT_CLIFF_RUNNING_STEP;
            break;
        }
        if (obstacleSignal != none_obstacle)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            cliff_running_step_status = COLLISION_TURN_GOSTR_GREATER_90_L_FRONT_CLIFF_RUNNING_STEP;
            break;
        }
        if (cliff_value->cliffValue0 == 1)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            cliff_running_step_status = GREATER_90_L_FRONT_CLIFF_RUNNING_STEP;
            break;
        }
        break;
    case COLLISION_TURN_GOSTR_GREATER_90_L_FRONT_CLIFF_RUNNING_STEP:
        if (clill_start_update == 0)
        {
            turn_start_x = current_pose->x;
            turn_start_y = current_pose->y;
            clill_start_update = 1;
        }
        linear_velocity = -long_stra_vel;
        angular_velocity = 0;
        if (my_abs(turn_start_x - current_pose->x) > turn_backward_distance || my_abs(turn_start_y - current_pose->y) > turn_backward_distance)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            clill_start_update = 0;
            cliff_running_step_status = TURN_GOSTR_GREATER_90_L_FRONT_CLIFF_RUNNING_STEP;
            break;
        }
        break;

    case LESS_90_L_FRONT_CLIFF_RUNNING_STEP:
        if (clill_start_update == 0)
        {
            turn_start_x = current_pose->x;
            turn_start_y = current_pose->y;
            clill_start_update = 1;
        }
        linear_velocity = -long_stra_vel;
        angular_velocity = 0;
        if ((my_abs(turn_start_x - current_pose->x) > lateral_move_distance / 4 || my_abs(turn_start_y - current_pose->y) > lateral_move_distance / 4) 
			&&
            cliff_value->cliffValue0 == 0
		)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            temporary_yaw = Yaw / 100;
            cliff_running_step_status = TURN_LESS_90_L_FRONT_CLIFF_RUNNING_STEP;
            break;
        }
        break;
    case TURN_LESS_90_L_FRONT_CLIFF_RUNNING_STEP:
        linear_velocity = 0;
        angular_velocity = turn_vel;
        if (CliffDelimmaNumber > 4)
        {
            cliff_running_step_status = INTERRUPT_AND_STOP;
            break;
        }
        if (cliff_value->cliffValue0 == 1)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            cliff_running_step_status = LESS_90_L_FRONT_CLIFF_RUNNING_STEP;
            break;
        }
        if (obstacleSignal != none_obstacle)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            cliff_running_step_status = COLLISION_TURN_LESS_90_L_FRONT_CLIFF_RUNNING_STEP;
            break;
        }
        if (my_abs(temporary_yaw - Yaw / 100) > 45)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            CliffDelimmaNumber++;
            last_position_x = current_pose->x;
            last_position_y = current_pose->y;
            cliff_running_step_status = GOSTR_LESS_90_L_FRONT_CLIFF_RUNNING_STEP;
            break;
        }
        break;
    case COLLISION_TURN_LESS_90_L_FRONT_CLIFF_RUNNING_STEP:
        if (clill_start_update == 0)
        {
            turn_start_x = current_pose->x;
            turn_start_y = current_pose->y;
            clill_start_update = 1;
        }
        linear_velocity = -long_stra_vel;
        angular_velocity = 0;
        if (my_abs(turn_start_x - current_pose->x) > turn_backward_distance || my_abs(turn_start_y - current_pose->y) > turn_backward_distance)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            clill_start_update = 0;
            cliff_running_step_status = TURN_LESS_90_L_FRONT_CLIFF_RUNNING_STEP;
            break;
        }
        break;
    case GOSTR_LESS_90_L_FRONT_CLIFF_RUNNING_STEP:
        linear_velocity = long_stra_vel;
        angular_velocity = 0;
        if (cliff_value->cliffValue0 == 1)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            cliff_running_step_status = LESS_90_L_FRONT_CLIFF_RUNNING_STEP;
            break;
        }
        if (obstacleSignal != none_obstacle)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            cliff_running_step_status = COLLISION_GOSTR_LESS_90_L_FRONT_CLIFF_RUNNING_STEP;
            break;
        }

        if (my_abs(last_position_x - current_pose->x) > lateral_move_distance / 4 || my_abs(last_position_y - current_pose->y) > lateral_move_distance / 4)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            cliff_running_step_status = CLIFF_COMPLETE;
            break;
        }
        break;
    case COLLISION_GOSTR_LESS_90_L_FRONT_CLIFF_RUNNING_STEP:
        if (clill_start_update == 0)
        {
            turn_start_x = current_pose->x;
            turn_start_y = current_pose->y;
            clill_start_update = 1;
        }
        linear_velocity = -long_stra_vel;
        angular_velocity = 0;
        if (my_abs(turn_start_x - current_pose->x) > turn_backward_distance || my_abs(turn_start_y - current_pose->y) > turn_backward_distance)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            clill_start_update = 0;
            temporary_yaw = Yaw / 100;
            cliff_running_step_status = TURN_GOSTR_LESS_90_L_FRONT_CLIFF_RUNNING_STEP;
            break;
        }
        break;
    case TURN_GOSTR_LESS_90_L_FRONT_CLIFF_RUNNING_STEP:
        linear_velocity = 0;
        angular_velocity = turn_vel;
        if (my_abs(temporary_yaw - Yaw / 100) > 15)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            cliff_running_step_status = GOSTR_LESS_90_L_FRONT_CLIFF_RUNNING_STEP;
            break;
        }
        if (obstacleSignal != none_obstacle)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            cliff_running_step_status = COLLISION_TURN_GOSTR_LESS_90_L_FRONT_CLIFF_RUNNING_STEP;
            break;
        }
        if (cliff_value->cliffValue0 == 1)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            cliff_running_step_status = LESS_90_L_FRONT_CLIFF_RUNNING_STEP;
            break;
        }
        break;
    case COLLISION_TURN_GOSTR_LESS_90_L_FRONT_CLIFF_RUNNING_STEP:
        if (clill_start_update == 0)
        {
            turn_start_x = current_pose->x;
            turn_start_y = current_pose->y;
            clill_start_update = 1;
        }
        linear_velocity = -long_stra_vel;
        angular_velocity = 0;
        if (my_abs(turn_start_x - current_pose->x) > turn_backward_distance || my_abs(turn_start_y - current_pose->y) > turn_backward_distance)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            clill_start_update = 0;
            cliff_running_step_status = TURN_GOSTR_LESS_90_L_FRONT_CLIFF_RUNNING_STEP;
            break;
        }
        break;

    case ORIGIN_FRONT_CLIFF_RUNNING_STEP:
        if (clill_start_update == 0)
        {
            turn_start_x = current_pose->x;
            turn_start_y = current_pose->y;
            clill_start_update = 1;
        }
        linear_velocity = -long_stra_vel;
        angular_velocity = 0;
        if ((my_abs(turn_start_x - current_pose->x) > lateral_move_distance / 4 || my_abs(turn_start_y - current_pose->y) > lateral_move_distance / 4) 
			&&
            cliff_value->cliffValue0 == 0
		)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            temporary_yaw = Yaw / 100;
            cliff_running_step_status = TURN_ORIGIN_FRONT_CLIFF_RUNNING_STEP;
            break;
        }
        break;
    case TURN_ORIGIN_FRONT_CLIFF_RUNNING_STEP:
        linear_velocity = 0;
        angular_velocity = turn_vel;
        if (CliffDelimmaNumber > 4)
        {
            cliff_running_step_status = INTERRUPT_AND_STOP;
            break;
        }
        if (cliff_value->cliffValue0 == 1)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            cliff_running_step_status = ORIGIN_FRONT_CLIFF_RUNNING_STEP;
            break;
        }
        if (obstacleSignal != none_obstacle)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            cliff_running_step_status = COLLISION_TURN_ORIGIN_FRONT_CLIFF_RUNNING_STEP;
            break;
        }
        if (my_abs(temporary_yaw - Yaw / 100) > 45)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            CliffDelimmaNumber++;
            last_position_x = current_pose->x;
            last_position_y = current_pose->y;
            cliff_running_step_status = GOSTR_ORIGIN_FRONT_CLIFF_RUNNING_STEP;
            break;
        }
        break;
    case COLLISION_TURN_ORIGIN_FRONT_CLIFF_RUNNING_STEP:
        if (clill_start_update == 0)
        {
            turn_start_x = current_pose->x;
            turn_start_y = current_pose->y;
            clill_start_update = 1;
        }
        linear_velocity = -long_stra_vel;
        angular_velocity = 0;
        if (my_abs(turn_start_x - current_pose->x) > turn_backward_distance || my_abs(turn_start_y - current_pose->y) > turn_backward_distance)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            clill_start_update = 0;
            cliff_running_step_status = TURN_ORIGIN_FRONT_CLIFF_RUNNING_STEP;
            break;
        }
        break;
    case GOSTR_ORIGIN_FRONT_CLIFF_RUNNING_STEP:
        linear_velocity = long_stra_vel;
        angular_velocity = 0;
        if (cliff_value->cliffValue0 == 1)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            cliff_running_step_status = ORIGIN_FRONT_CLIFF_RUNNING_STEP;
            break;
        }
        if (obstacleSignal != none_obstacle)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            cliff_running_step_status = COLLISION_GOSTR_ORIGIN_FRONT_CLIFF_RUNNING_STEP;
            break;
        }

        if (my_abs(last_position_x - current_pose->x) > lateral_move_distance / 2 || my_abs(last_position_y - current_pose->y) > lateral_move_distance / 2)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            cliff_running_step_status = CLIFF_COMPLETE;
            break;
        }
        break;
    case COLLISION_GOSTR_ORIGIN_FRONT_CLIFF_RUNNING_STEP:
        if (clill_start_update == 0)
        {
            turn_start_x = current_pose->x;
            turn_start_y = current_pose->y;
            clill_start_update = 1;
        }
        linear_velocity = -long_stra_vel;
        angular_velocity = 0;
        if (my_abs(turn_start_x - current_pose->x) > turn_backward_distance || my_abs(turn_start_y - current_pose->y) > turn_backward_distance)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            clill_start_update = 0;
            temporary_yaw = Yaw / 100;
            cliff_running_step_status = TURN_GOSTR_ORIGIN_FRONT_CLIFF_RUNNING_STEP;
            break;
        }
        break;
    case TURN_GOSTR_ORIGIN_FRONT_CLIFF_RUNNING_STEP:
        linear_velocity = 0;
        angular_velocity = turn_vel;
        if (my_abs(temporary_yaw - Yaw / 100) > 15)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            cliff_running_step_status = GOSTR_ORIGIN_FRONT_CLIFF_RUNNING_STEP;
            break;
        }
        if (obstacleSignal != none_obstacle)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            cliff_running_step_status = COLLISION_TURN_GOSTR_ORIGIN_FRONT_CLIFF_RUNNING_STEP;
            break;
        }
        if (cliff_value->cliffValue0 == 1)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            cliff_running_step_status = ORIGIN_FRONT_CLIFF_RUNNING_STEP;
            break;
        }
        break;
    case COLLISION_TURN_GOSTR_ORIGIN_FRONT_CLIFF_RUNNING_STEP:
        if (clill_start_update == 0)
        {
            turn_start_x = current_pose->x;
            turn_start_y = current_pose->y;
            clill_start_update = 1;
        }
        linear_velocity = -long_stra_vel;
        angular_velocity = 0;
        if (my_abs(turn_start_x - current_pose->x) > turn_backward_distance || my_abs(turn_start_y - current_pose->y) > turn_backward_distance)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            clill_start_update = 0;
            cliff_running_step_status = TURN_GOSTR_ORIGIN_FRONT_CLIFF_RUNNING_STEP;
            break;
        }
        break;

    case LESS_90_R_RIGHT_CLIFF_RUNNING_STEP:
        if (clill_start_update == 0)
        {
            turn_start_x = current_pose->x;
            turn_start_y = current_pose->y;
            clill_start_update = 1;
        }
        linear_velocity = -long_stra_vel;
        angular_velocity = 0;
        if ((my_abs(turn_start_x - current_pose->x) > lateral_move_distance / 4 || my_abs(turn_start_y - current_pose->y) > lateral_move_distance / 4) 
			&&
            cliff_value->cliffValue0 == 0
		)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            temporary_yaw = Yaw / 100;
            cliff_running_step_status = TURN_LESS_90_R_RIGHT_CLIFF_RUNNING_STEP;
            break;
        }
        break;
    case TURN_LESS_90_R_RIGHT_CLIFF_RUNNING_STEP:
        linear_velocity = 0;
        angular_velocity = -turn_vel;
        if (CliffDelimmaNumber > 4)
        {
            cliff_running_step_status = INTERRUPT_AND_STOP;
            break;
        }
        if (cliff_value->cliffValue0 == 1)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            cliff_running_step_status = LESS_90_R_RIGHT_CLIFF_RUNNING_STEP;
            break;
        }
        if (obstacleSignal != none_obstacle)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            cliff_running_step_status = COLLISION_TURN_LESS_90_R_RIGHT_CLIFF_RUNNING_STEP;
            break;
        }
        if (my_abs(temporary_yaw - Yaw / 100) > 90)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            CliffDelimmaNumber++;
            last_position_x = current_pose->x;
            last_position_y = current_pose->y;
            cliff_running_step_status = GOSTR_LESS_90_R_RIGHT_CLIFF_RUNNING_STEP;
            break;
        }
        break;
    case COLLISION_TURN_LESS_90_R_RIGHT_CLIFF_RUNNING_STEP:
        if (clill_start_update == 0)
        {
            turn_start_x = current_pose->x;
            turn_start_y = current_pose->y;
            clill_start_update = 1;
        }
        linear_velocity = -long_stra_vel;
        angular_velocity = 0;
        if (my_abs(turn_start_x - current_pose->x) > turn_backward_distance || my_abs(turn_start_y - current_pose->y) > turn_backward_distance)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            clill_start_update = 0;
            cliff_running_step_status = TURN_LESS_90_R_RIGHT_CLIFF_RUNNING_STEP;
            break;
        }
        break;
    case GOSTR_LESS_90_R_RIGHT_CLIFF_RUNNING_STEP:
        linear_velocity = long_stra_vel;
        angular_velocity = 0;
        if (cliff_value->cliffValue0 == 1)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            cliff_running_step_status = LESS_90_R_RIGHT_CLIFF_RUNNING_STEP;
            break;
        }
        if (obstacleSignal != none_obstacle)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            cliff_running_step_status = COLLISION_GOSTR_LESS_90_R_RIGHT_CLIFF_RUNNING_STEP;
            break;
        }
        if (my_abs(last_position_x - current_pose->x) > lateral_move_distance / 4 || my_abs(last_position_y - current_pose->y) > lateral_move_distance / 4)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            cliff_running_step_status = CLIFF_COMPLETE;
            break;
        }
        break;
    case COLLISION_GOSTR_LESS_90_R_RIGHT_CLIFF_RUNNING_STEP:
        if (clill_start_update == 0)
        {
            turn_start_x = current_pose->x;
            turn_start_y = current_pose->y;
            clill_start_update = 1;
        }
        linear_velocity = -long_stra_vel;
        angular_velocity = 0;
        if (my_abs(turn_start_x - current_pose->x) > turn_backward_distance || my_abs(turn_start_y - current_pose->y) > turn_backward_distance)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            clill_start_update = 0;
            temporary_yaw = Yaw / 100;
            cliff_running_step_status = TURN_GOSTR_LESS_90_R_RIGHT_CLIFF_RUNNING_STEP;
            break;
        }
        break;
    case TURN_GOSTR_LESS_90_R_RIGHT_CLIFF_RUNNING_STEP:
        linear_velocity = 0;
        angular_velocity = -turn_vel;
        if (my_abs(temporary_yaw - Yaw / 100) > 15)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            cliff_running_step_status = GOSTR_LESS_90_R_RIGHT_CLIFF_RUNNING_STEP;
            break;
        }
        if (obstacleSignal != none_obstacle)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            cliff_running_step_status = COLLISION_TURN_GOSTR_LESS_90_R_RIGHT_CLIFF_RUNNING_STEP;
            break;
        }
        if (cliff_value->cliffValue0 == 1)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            cliff_running_step_status = LESS_90_R_RIGHT_CLIFF_RUNNING_STEP;
            break;
        }
        break;
    case COLLISION_TURN_GOSTR_LESS_90_R_RIGHT_CLIFF_RUNNING_STEP:
        if (clill_start_update == 0)
        {
            turn_start_x = current_pose->x;
            turn_start_y = current_pose->y;
            clill_start_update = 1;
        }
        linear_velocity = -long_stra_vel;
        angular_velocity = 0;
        if (my_abs(turn_start_x - current_pose->x) > turn_backward_distance || my_abs(turn_start_y - current_pose->y) > turn_backward_distance)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            clill_start_update = 0;
            cliff_running_step_status = TURN_GOSTR_LESS_90_R_RIGHT_CLIFF_RUNNING_STEP;
            break;
        }
        break;

    case GREATER_90_L_RIGHT_CLIFF_RUNNING_STEP:
        if (clill_start_update == 0)
        {
            turn_start_x = current_pose->x;
            turn_start_y = current_pose->y;
            clill_start_update = 1;
        }
        linear_velocity = -long_stra_vel;
        angular_velocity = 0;
        if ((my_abs(turn_start_x - current_pose->x) > lateral_move_distance / 4 || my_abs(turn_start_y - current_pose->y) > lateral_move_distance / 4) 
			&&
            cliff_value->cliffValue0 == 0
		)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            temporary_yaw = Yaw / 100;
            cliff_running_step_status = TURN_GREATER_90_L_RIGHT_CLIFF_RUNNING_STEP;
            break;
        }
        break;
    case TURN_GREATER_90_L_RIGHT_CLIFF_RUNNING_STEP:
        linear_velocity = 0;
        angular_velocity = -turn_vel;
        if (CliffDelimmaNumber > 4)
        {
            cliff_running_step_status = INTERRUPT_AND_STOP;
            break;
        }
        if (cliff_value->cliffValue0 == 1)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            cliff_running_step_status = GREATER_90_L_RIGHT_CLIFF_RUNNING_STEP;
            break;
        }
        if (obstacleSignal != none_obstacle)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            cliff_running_step_status = COLLISION_TURN_GREATER_90_L_RIGHT_CLIFF_RUNNING_STEP;
            break;
        }
        if (my_abs(temporary_yaw - Yaw / 100) > 90)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            CliffDelimmaNumber++;
            last_position_x = current_pose->x;
            last_position_y = current_pose->y;
            cliff_running_step_status = GOSTR_GREATER_90_L_RIGHT_CLIFF_RUNNING_STEP;
            break;
        }
        break;
    case COLLISION_TURN_GREATER_90_L_RIGHT_CLIFF_RUNNING_STEP:
        if (clill_start_update == 0)
        {
            turn_start_x = current_pose->x;
            turn_start_y = current_pose->y;
            clill_start_update = 1;
        }
        linear_velocity = -long_stra_vel;
        angular_velocity = 0;
        if (my_abs(turn_start_x - current_pose->x) > turn_backward_distance || my_abs(turn_start_y - current_pose->y) > turn_backward_distance)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            clill_start_update = 0;
            cliff_running_step_status = TURN_GREATER_90_L_RIGHT_CLIFF_RUNNING_STEP;
            break;
        }
        break;
    case GOSTR_GREATER_90_L_RIGHT_CLIFF_RUNNING_STEP:
        linear_velocity = long_stra_vel;
        angular_velocity = 0;
        if (cliff_value->cliffValue0 == 1)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            cliff_running_step_status = GREATER_90_L_RIGHT_CLIFF_RUNNING_STEP;
            break;
        }
        if (obstacleSignal != none_obstacle)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            cliff_running_step_status = COLLISION_GOSTR_GREATER_90_L_RIGHT_CLIFF_RUNNING_STEP;
            break;
        }

        if (my_abs(last_position_x - current_pose->x) > lateral_move_distance / 2 || my_abs(last_position_y - current_pose->y) > lateral_move_distance / 2)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            cliff_running_step_status = CLIFF_COMPLETE;
            break;
        }
        break;
    case COLLISION_GOSTR_GREATER_90_L_RIGHT_CLIFF_RUNNING_STEP:
        if (clill_start_update == 0)
        {
            turn_start_x = current_pose->x;
            turn_start_y = current_pose->y;
            clill_start_update = 1;
        }
        linear_velocity = -long_stra_vel;
        angular_velocity = 0;
        if (my_abs(turn_start_x - current_pose->x) > turn_backward_distance || my_abs(turn_start_y - current_pose->y) > turn_backward_distance)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            clill_start_update = 0;
            temporary_yaw = Yaw / 100;
            cliff_running_step_status = TURN_GOSTR_GREATER_90_L_RIGHT_CLIFF_RUNNING_STEP;
            break;
        }
        break;
    case TURN_GOSTR_GREATER_90_L_RIGHT_CLIFF_RUNNING_STEP:
        linear_velocity = 0;
        angular_velocity = -turn_vel;
        if (my_abs(temporary_yaw - Yaw / 100) > 15)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            cliff_running_step_status = GOSTR_GREATER_90_L_RIGHT_CLIFF_RUNNING_STEP;
            break;
        }
        if (obstacleSignal != none_obstacle)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            cliff_running_step_status = COLLISION_TURN_GOSTR_GREATER_90_L_RIGHT_CLIFF_RUNNING_STEP;
            break;
        }
        if (cliff_value->cliffValue0 == 1)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            cliff_running_step_status = GREATER_90_L_RIGHT_CLIFF_RUNNING_STEP;
            break;
        }
        break;
    case COLLISION_TURN_GOSTR_GREATER_90_L_RIGHT_CLIFF_RUNNING_STEP:
        if (clill_start_update == 0)
        {
            turn_start_x = current_pose->x;
            turn_start_y = current_pose->y;
            clill_start_update = 1;
        }
        linear_velocity = -long_stra_vel;
        angular_velocity = 0;
        if (my_abs(turn_start_x - current_pose->x) > turn_backward_distance || my_abs(turn_start_y - current_pose->y) > turn_backward_distance)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            clill_start_update = 0;
            cliff_running_step_status = TURN_GOSTR_GREATER_90_L_RIGHT_CLIFF_RUNNING_STEP;
            break;
        }
        break;

    case GREATER_90_R_LEFT_CLIFF_RUNNING_STEP:
        if (clill_start_update == 0)
        {
            turn_start_x = current_pose->x;
            turn_start_y = current_pose->y;
            clill_start_update = 1;
        }
        linear_velocity = -long_stra_vel;
        angular_velocity = 0;
        if ((my_abs(turn_start_x - current_pose->x) > lateral_move_distance / 4 || my_abs(turn_start_y - current_pose->y) > lateral_move_distance / 4) 
			&&
            cliff_value->cliffValue0 == 0
		)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            temporary_yaw = Yaw / 100;
            cliff_running_step_status = TURN_GREATER_90_R_LEFT_CLIFF_RUNNING_STEP;
            break;
        }
        break;
    case TURN_GREATER_90_R_LEFT_CLIFF_RUNNING_STEP:
        linear_velocity = 0;
        angular_velocity = turn_vel;
        if (CliffDelimmaNumber > 4)
        {
            cliff_running_step_status = INTERRUPT_AND_STOP;
            break;
        }
        if (cliff_value->cliffValue0 == 1)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            cliff_running_step_status = GREATER_90_R_LEFT_CLIFF_RUNNING_STEP;
            break;
        }
        if (obstacleSignal != none_obstacle)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            cliff_running_step_status = COLLISION_TURN_GREATER_90_R_LEFT_CLIFF_RUNNING_STEP;
            break;
        }
        if (my_abs(temporary_yaw - Yaw / 100) > 90)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            CliffDelimmaNumber++;
            last_position_x = current_pose->x;
            last_position_y = current_pose->y;
            cliff_running_step_status = GOSTR_GREATER_90_R_LEFT_CLIFF_RUNNING_STEP;
            break;
        }
        break;
    case COLLISION_TURN_GREATER_90_R_LEFT_CLIFF_RUNNING_STEP:
        if (clill_start_update == 0)
        {
            turn_start_x = current_pose->x;
            turn_start_y = current_pose->y;
            clill_start_update = 1;
        }
        linear_velocity = -long_stra_vel;
        angular_velocity = 0;
        if (my_abs(turn_start_x - current_pose->x) > turn_backward_distance || my_abs(turn_start_y - current_pose->y) > turn_backward_distance)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            clill_start_update = 0;
            cliff_running_step_status = TURN_GREATER_90_R_LEFT_CLIFF_RUNNING_STEP;
            break;
        }
        break;
    case GOSTR_GREATER_90_R_LEFT_CLIFF_RUNNING_STEP:
        linear_velocity = long_stra_vel;
        angular_velocity = 0;
        if (cliff_value->cliffValue0 == 1)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            cliff_running_step_status = GREATER_90_R_LEFT_CLIFF_RUNNING_STEP;
            break;
        }
        if (obstacleSignal != none_obstacle)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            cliff_running_step_status = COLLISION_GOSTR_GREATER_90_R_LEFT_CLIFF_RUNNING_STEP;
            break;
        }

        if (my_abs(last_position_x - current_pose->x) > lateral_move_distance / 2 || my_abs(last_position_y - current_pose->y) > lateral_move_distance / 2)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            cliff_running_step_status = CLIFF_COMPLETE;
            break;
        }
        break;
    case COLLISION_GOSTR_GREATER_90_R_LEFT_CLIFF_RUNNING_STEP:
        if (clill_start_update == 0)
        {
            turn_start_x = current_pose->x;
            turn_start_y = current_pose->y;
            clill_start_update = 1;
        }
        linear_velocity = -long_stra_vel;
        angular_velocity = 0;
        if (my_abs(turn_start_x - current_pose->x) > turn_backward_distance || my_abs(turn_start_y - current_pose->y) > turn_backward_distance)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            clill_start_update = 0;
            temporary_yaw = Yaw / 100;
            cliff_running_step_status = TURN_GOSTR_GREATER_90_R_LEFT_CLIFF_RUNNING_STEP;
            break;
        }
        break;
    case TURN_GOSTR_GREATER_90_R_LEFT_CLIFF_RUNNING_STEP:
        linear_velocity = 0;
        angular_velocity = turn_vel;
        if (my_abs(temporary_yaw - Yaw / 100) > 15)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            cliff_running_step_status = GOSTR_GREATER_90_R_LEFT_CLIFF_RUNNING_STEP;
            break;
        }
        if (obstacleSignal != none_obstacle)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            cliff_running_step_status = COLLISION_TURN_GOSTR_GREATER_90_R_LEFT_CLIFF_RUNNING_STEP;
            break;
        }
        if (cliff_value->cliffValue0 == 1)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            cliff_running_step_status = GREATER_90_R_LEFT_CLIFF_RUNNING_STEP;
            break;
        }
        break;
    case COLLISION_TURN_GOSTR_GREATER_90_R_LEFT_CLIFF_RUNNING_STEP:
        if (clill_start_update == 0)
        {
            turn_start_x = current_pose->x;
            turn_start_y = current_pose->y;
            clill_start_update = 1;
        }
        linear_velocity = -long_stra_vel;
        angular_velocity = 0;
        if (my_abs(turn_start_x - current_pose->x) > turn_backward_distance || my_abs(turn_start_y - current_pose->y) > turn_backward_distance)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            clill_start_update = 0;
            cliff_running_step_status = TURN_GOSTR_GREATER_90_R_LEFT_CLIFF_RUNNING_STEP;
            break;
        }
        break;

    case LESS_90_L_LEFT_CLIFF_RUNNING_STEP:
        if (clill_start_update == 0)
        {
            turn_start_x = current_pose->x;
            turn_start_y = current_pose->y;
            clill_start_update = 1;
        }
        linear_velocity = -long_stra_vel;
        angular_velocity = 0;
        if ((my_abs(turn_start_x - current_pose->x) > lateral_move_distance / 4 || my_abs(turn_start_y - current_pose->y) > lateral_move_distance / 4) 
			&&
            cliff_value->cliffValue0 == 0
		)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            temporary_yaw = Yaw / 100;
            cliff_running_step_status = TURN_LESS_90_L_LEFT_CLIFF_RUNNING_STEP;
            break;
        }
        break;
    case TURN_LESS_90_L_LEFT_CLIFF_RUNNING_STEP:
        linear_velocity = 0;
        angular_velocity = turn_vel;
        if (CliffDelimmaNumber > 4)
        {
            cliff_running_step_status = INTERRUPT_AND_STOP;
            break;
        }
        if (cliff_value->cliffValue0 == 1)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            cliff_running_step_status = LESS_90_L_LEFT_CLIFF_RUNNING_STEP;
            break;
        }
        if (obstacleSignal != none_obstacle)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            cliff_running_step_status = COLLISION_TURN_LESS_90_L_LEFT_CLIFF_RUNNING_STEP;
            break;
        }
        if (my_abs(temporary_yaw - Yaw / 100) > 90)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            CliffDelimmaNumber++;
            last_position_x = current_pose->x;
            last_position_y = current_pose->y;
            cliff_running_step_status = GOSTR_LESS_90_L_LEFT_CLIFF_RUNNING_STEP;
            break;
        }
        break;
    case COLLISION_TURN_LESS_90_L_LEFT_CLIFF_RUNNING_STEP:
        if (clill_start_update == 0)
        {
            turn_start_x = current_pose->x;
            turn_start_y = current_pose->y;
            clill_start_update = 1;
        }
        linear_velocity = -long_stra_vel;
        angular_velocity = 0;
        if (my_abs(turn_start_x - current_pose->x) > turn_backward_distance || my_abs(turn_start_y - current_pose->y) > turn_backward_distance)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            clill_start_update = 0;
            cliff_running_step_status = TURN_LESS_90_L_LEFT_CLIFF_RUNNING_STEP;
            break;
        }
        break;
    case GOSTR_LESS_90_L_LEFT_CLIFF_RUNNING_STEP:
        linear_velocity = long_stra_vel;
        angular_velocity = 0;
        if (cliff_value->cliffValue0 == 1)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            cliff_running_step_status = LESS_90_L_LEFT_CLIFF_RUNNING_STEP;
            break;
        }
        if (obstacleSignal != none_obstacle)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            cliff_running_step_status = COLLISION_GOSTR_LESS_90_L_LEFT_CLIFF_RUNNING_STEP;
            break;
        }

        if (my_abs(last_position_x - current_pose->x) > lateral_move_distance / 2 || my_abs(last_position_y - current_pose->y) > lateral_move_distance / 2)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            cliff_running_step_status = CLIFF_COMPLETE;
            break;
        }
        break;
    case COLLISION_GOSTR_LESS_90_L_LEFT_CLIFF_RUNNING_STEP:
        if (clill_start_update == 0)
        {
            turn_start_x = current_pose->x;
            turn_start_y = current_pose->y;
            clill_start_update = 1;
        }
        linear_velocity = -long_stra_vel;
        angular_velocity = 0;
        if (my_abs(turn_start_x - current_pose->x) > turn_backward_distance || my_abs(turn_start_y - current_pose->y) > turn_backward_distance)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            clill_start_update = 0;
            temporary_yaw = Yaw / 100;
            cliff_running_step_status = TURN_GOSTR_LESS_90_L_LEFT_CLIFF_RUNNING_STEP;
            break;
        }
        break;
    case TURN_GOSTR_LESS_90_L_LEFT_CLIFF_RUNNING_STEP:
        linear_velocity = 0;
        angular_velocity = turn_vel;
        if (my_abs(temporary_yaw - Yaw / 100) > 15)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            cliff_running_step_status = GOSTR_LESS_90_L_LEFT_CLIFF_RUNNING_STEP;
            break;
        }
        if (obstacleSignal != none_obstacle)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            cliff_running_step_status = COLLISION_TURN_GOSTR_LESS_90_L_LEFT_CLIFF_RUNNING_STEP;
            break;
        }
        if (cliff_value->cliffValue0 == 1)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            cliff_running_step_status = LESS_90_L_LEFT_CLIFF_RUNNING_STEP;
            break;
        }
        break;
    case COLLISION_TURN_GOSTR_LESS_90_L_LEFT_CLIFF_RUNNING_STEP:
        if (clill_start_update == 0)
        {
            turn_start_x = current_pose->x;
            turn_start_y = current_pose->y;
            clill_start_update = 1;
        }
        linear_velocity = -long_stra_vel;
        angular_velocity = 0;
        if (my_abs(turn_start_x - current_pose->x) > turn_backward_distance || my_abs(turn_start_y - current_pose->y) > turn_backward_distance)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            clill_start_update = 0;
            cliff_running_step_status = TURN_GOSTR_LESS_90_L_LEFT_CLIFF_RUNNING_STEP;
            break;
        }
        break;
    case CLIFF_COMPLETE:
        complete_flag = 1;
        CliffDelimmaNumber = 0;
        cliff_running_step_status = 0;
        break;
    case INTERRUPT_AND_STOP:
        complete_flag = 2;
        linear_velocity = 0;
        angular_velocity = 0;
        CliffDelimmaNumber = 0;
        cliff_running_step_status = 0;
        break;
    default:
        break;
    }
    return complete_flag;
}






unsigned char  CloseEdgedMap(POSE *current_pose, CLIFFADCVALUE *cliff_value, unsigned char obstacleSignal)
{
    short Yaw, map_robot_x, map_robot_y;
    int8_t i, j, complete_flag = 0;
    int8_t k, ij;
    Yaw = current_pose->orientation / 100;
    map_robot_x = (current_pose->x + half_map_wide) / GRIDWIDTH;
    map_robot_y = (current_pose->y + half_map_long) / GRIDHEIGHT;
    if (Yaw < 0)
    {
        Yaw = -Yaw;
    }
    if (Yaw <= 90 && selectside == 'R')
    {
        if ((map_robot_x + 4 >= MAPWIDECELLS) || (map_robot_y - 2 < 0))
        {
            return complete_flag;
        }
        else
        {
            k = (map_robot_x + 9) < MAPWIDECELLS ? (map_robot_x + 9) : MAPWIDECELLS;
            ij = (map_robot_y - 5) >= 0 ? (map_robot_y - 5) : 0;
            for (i = map_robot_x + 4; i < k; i++)
            {
                for (j = map_robot_y - 2; j > ij; j--)
                {
                    if (gridmap.map[i][j] == 0)
                    {
                        complete_flag = 1;
                        return complete_flag;
                    }
                }
            }
        }
    }
    else if (Yaw > 90 && selectside == 'R')
    {
        if (map_robot_x - 4 < 0 || map_robot_y - 2 < 0)
        {
            return complete_flag;
        }
        else
        {
            k = map_robot_x - 10 >= 0 ? (map_robot_x - 11) : 0;
            ij = map_robot_y - 5 >= 0 ? (map_robot_y - 5) : 0;
            for (i = map_robot_x - 4; i < k; i--)
            {
                for (j = map_robot_y - 2; j > ij; j--)
                {
                    if (gridmap.map[i][j] == 0)
                    {
                        complete_flag = 1;
                        return complete_flag;
                    }
                }
            }
        }
    }
    else if (Yaw <= 90 && selectside == 'L')
    {
        if (map_robot_x + 4 >= MAPWIDECELLS || map_robot_y + 2 >= MAPLONGCELLS)
        {
            return complete_flag;
        }
        else
        {
            k = (map_robot_x + 11) > MAPWIDECELLS ? MAPWIDECELLS : (map_robot_x + 11);
            ij = (map_robot_y + 5) < MAPLONGCELLS ? (map_robot_y + 5) : MAPLONGCELLS;
            for (i = map_robot_x + 4; i < k; i++)
            {
                for (j = map_robot_y + 2; j < ij; j++)
                {
                    if (gridmap.map[i][j] == 0)
                    {
                        complete_flag = 1;
                        return complete_flag;
                    }
                }
            }
        }
    }
    else if (Yaw > 90 && selectside == 'L')
    {
        if (map_robot_x - 4 < 0 || map_robot_y + 2 >= MAPLONGCELLS)
        {
            return complete_flag;
        }
        else
        {
            k = (map_robot_x - 11) >= 0 ? (map_robot_x - 11) : 0;
            ij = (map_robot_y + 5) < MAPLONGCELLS ? (map_robot_y + 5) : MAPLONGCELLS;
            for (i = map_robot_x - 4; i < k; i--)
            {
                for (j = map_robot_y + 2; j < ij; j++)
                {
                    if (gridmap.map[i][j] == 0)
                    {
                        complete_flag = 1;
                        return complete_flag;
                    }
                }
            }
        }
    }
    else
    {
    }
    return complete_flag;
}
void  DetectionCloseEdge()
{
    int8_t i, j, k;
    bool end_x = false;
    if (selectside == 'R')
    {
       //cout << "R" << endl;
        for (i = 0; i < GRIDHEIGHT; i++)
        {
            for (j = 0; j < GRIDHEIGHT; j++)
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
        }
        for (i = GRIDHEIGHT - 1; i > 0; i--)
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
        }
        while (close_edge_max_x - close_edge_min_x > 4)
        {
            for (k = close_edge_max_y; k < close_edge_max_y + 4; k++)
            {
                for (i = close_edge_max_x - 4; i < close_edge_max_x; i++)
                {
                    for (j = 0; j < GRIDHEIGHT; j++)
                    {
                        if (gridmap.map[i][j] == 0)
                        {
                            if (my_abs(close_edge_max_y - j) < 4)
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
                if (end_x == true)
                {
                    break;
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
            for (j = GRIDHEIGHT - 1; j > GRIDHEIGHT / 2; j--)
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
        }
        for (i = GRIDHEIGHT - 1; i > 0; i--)
        {
            for (j = GRIDHEIGHT - 1; j > GRIDHEIGHT / 2; j--)
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
        }
        while (close_edge_max_x - close_edge_min_x > 4)
        {
            for (k = close_edge_max_y; k < close_edge_max_y+4; k++)
            {
                for (i = close_edge_max_x - 4; i < close_edge_max_x; i++)
                {
                    for (j = GRIDHEIGHT - 1; j > 0; j--)
                    {
                        if (gridmap.map[i][j] == 0)
                        {
                            if (my_abs(close_edge_max_y - j) < 4)
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
                if (end_x == true)
                {
                    break;
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
    if (close_edge_max_x - close_edge_min_x > 4)
    {
        detection_close = true;
    }
    else
    {
        detection_close = false;
    }
}
unsigned char  CliffCloseEdge(POSE *current_pose)
{
    short map_robot_x, map_robot_y;
    int8_t i, j, complete_flag = 0;
    int8_t k;
    map_robot_x = (current_pose->x + half_map_wide) / GRIDWIDTH;
    map_robot_y = (current_pose->y + half_map_long) / GRIDHEIGHT;
    bool end_x = false;
    for (i = map_robot_x; i < GRIDHEIGHT; i++)
    {
        for (j = map_robot_y - 1; j <= map_robot_y + 1; j++)
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
    }
    for (i = map_robot_x; i >= 0; i--)
    {
        for (j = map_robot_y - 1; j <= map_robot_y + 1; j++)
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
    }
    for (j = map_robot_y; j >= 0; j--)
    {
        for (i = map_robot_x - 1; i <= map_robot_x + 1; i++)
        {
            if (gridmap.map[i][j] == 0)
            {
                close_r_edge_min_x = i;
                close_r_edge_min_y = j;
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
    for (j = map_robot_y; j < GRIDHEIGHT; j++)
    {
        for (i = map_robot_x - 1; i <= map_robot_x + 1; i++)
        {
            if (gridmap.map[i][j] == 0)
            {
                close_l_edge_max_x = i;
                close_l_edge_max_y = j;
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
    if (selectside == 'R')
    {
        while (close_edge_max_x - close_edge_min_x > 2)
        {
            for (k = close_edge_max_y; k <= close_edge_max_y + 2; k++)
            {
                for (i = close_edge_max_x - 2; i <= close_edge_max_x; i++)
                {
                    for (j = GRIDHEIGHT - 1; j >= 0; j--)
                    {
                        if (gridmap.map[i][j] == 0)
                        {
                            if (my_abs(close_edge_max_y - j) <= 2)
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
                if (end_x == true)
                {
                    break;
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
       //cout << "R" << endl;
        if (close_edge_max_x - close_edge_min_x <= 2)
        {
            complete_flag = 2;
			return complete_flag;
        }
        else
        {
			complete_flag = 0;
            return complete_flag;
        }
    }
    else
    {
        while (close_edge_max_x - close_edge_min_x > 2)
        {
            for (k = close_edge_max_y; k >= close_edge_max_y - 2; k--)
            {
                for (i = close_edge_max_x - 2; i <= close_edge_max_x; i++)
                {
                    for (j = 0; j < GRIDHEIGHT; j++)
                    {
                        if (gridmap.map[i][j] == 0)
                        {
                            if (my_abs(close_edge_max_y - j) <= 2)
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
                if (end_x == true)
                {
                    break;
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
       //cout << "L" << endl;
        if (close_edge_max_x - close_edge_min_x <= 2)
        {
            return complete_flag = 1;
        }
        else
        {
            return complete_flag = 0;
        }
    }
}


//#################################################################################
//#############  return origin function define  ###################################
//#################################################################################



	
