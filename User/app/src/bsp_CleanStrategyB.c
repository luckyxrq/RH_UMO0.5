#include "bsp.h"
#include <math.h>


#define INT_COOR_X 250
#define INT_COOR_Y 250
#define ALL_CLEAN_COMPLETE 6
#define LEFT_CLEAN_WORK_TIME 10*60*1000
#define RIGHT_CLEAN_WORK_TIME 30*1000

#define ZoomMultiple 4
#define compression_map_x 25
#define compression_map_y 25




int right_running_step_status = 0;
int collision_right_rightrun_step_status = 0;
int collision_left_rightrun_step_status = 0;
int collision_front_rightrun_step_status = 0;
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
//for  walk edge
int right_walk_edge_status = 0;
int right_reverse_walk_edge_status = 0;
int right_edge_dilemma_status = 0;
int number = 0;
int right_forward_boundary_status = 0;
int right_ready_leaking_sweep_status = 0;
int leakingsweep = 0;
int leakingsweep_x = 0;
int leakingsweep_y = 0;
short leakingsweep_X_interval = 200;
short leakingsweep_Y_interval = 100;


int left_running_step_status = 0;
int collision_right_leftrun_step_status = 0;
int collision_left_leftrun_step_status = 0;
int collision_front_leftrun_step_status = 0;
int left_walk_edge_status = 0;
int left_reverse_walk_edge_status = 0;
int left_edge_dilemma_status = 0;
int left_forward_boundary_status = 0;
int left_ready_leaking_sweep_status = 0;

int FunctionStatus=0;
unsigned int LastCleanTimeStamp = 0;
unsigned int CurrentCleanTimeStamp  = 0;

//**************return origin value************************************
int x_more_number = 0;
int x_less_number = 0;
int y_more_number = 0;
int y_less_number = 0;

char return_origin_positive_start = 1;
int return_origin_step_status = 0;
char plansteps[100];
int motionSteps = 0;
int a_star_not_motion = 0;

AStar_MapNode AStar_graph[AStar_Height][AStar_Width];
short AStar_srcX, AStar_srcY, AStar_dstX, AStar_dstY;
AStar_Close astar_close[AStar_Height][AStar_Width];
AStar_Close *AStar_start;
int AStar_shortestep;
bool maze[compression_map_x][compression_map_y];

int a_star_motion_return_origin = 0;
int startMotionStep = 0;
int AStarMotionNumber = 0;
bool return_origin_collision;
int temporary_wheel_pulse_r;
int temporary_wheel_pulse_l;
int wheel_pulse_l;
int wheel_pulse_r;
bool astar_coll = false;

int temporary_returnorigin = 0;
char selectside;
int a_star_collision = 0;
int a_star_collision_total = 0;
int markingobstacle = 0;

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
int OVERALL_CLEANING_STRATEGY = 0;
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
static int Yaw;
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
//right running
	right_running_step_status = 0;
	collision_right_rightrun_step_status = 0;
	collision_left_rightrun_step_status = 0;
	collision_front_rightrun_step_status = 0;
//left running	
	left_running_step_status = 0;
	collision_right_leftrun_step_status = 0;
	collision_left_leftrun_step_status = 0;
	collision_front_leftrun_step_status = 0;
//for collision step
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
//for right walk edge
	right_walk_edge_status = 0;
	right_reverse_walk_edge_status = 0;
	right_edge_dilemma_status = 0;
	number = 0;
	right_forward_boundary_status = 0;
	right_ready_leaking_sweep_status = 0;
	leakingsweep = 0;
	leakingsweep_x = 0;
	leakingsweep_y = 0;
//for left walk edge
	left_walk_edge_status = 0;
	left_reverse_walk_edge_status = 0;
	left_edge_dilemma_status = 0;
	left_forward_boundary_status = 0;
	left_ready_leaking_sweep_status = 0;
	
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
	
	
	
}



void bsp_CleanStrategyUpdateB(int robotX,int robotY,double robotTheta, unsigned char obstacleSignal, int current_wheel_pulse_l, int current_wheel_pulse_r, unsigned char IRSensorData[])
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
		if(clean_strategy(&current_pose,obstacleSignal) != ALL_CLEAN_COMPLETE)
		{
			//nothing...
		}
		else{
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
			if(RightReturnOriginWorkStep(current_pose,obstacleSignal) || ((CurrentCleanTimeStamp - LastCleanTimeStamp) > RIGHT_CLEAN_WORK_TIME))
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
			if(LeftReturnOriginWorkStep(current_pose,obstacleSignal) || ((CurrentCleanTimeStamp - LastCleanTimeStamp) > LEFT_CLEAN_WORK_TIME))
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
	switch (OVERALL_CLEANING_STRATEGY)
    {
        case 0:
            OVERALL_CLEANING_STRATEGY = START_OVERALL_CLEANING_STRATEGY;
            break;
        case START_OVERALL_CLEANING_STRATEGY:
            log_debug("START_OVERALL_CLEANING_STRATEGY");
            OVERALL_CLEANING_STRATEGY = RIGHT_RUNNING_WORKING_OVERALL_CLEANING_STRATEGY;
            break;
        case RIGHT_RUNNING_WORKING_OVERALL_CLEANING_STRATEGY:
            log_debug("RIGHT_RUNNING_WORKING_OVERALL_CLEANING_STRATEGY");
            FunctionStatus = RightRunningWorkStep(current_pose, obstacleSignal);
            if (1 == FunctionStatus)
            {
                selectside = 'L';
                OVERALL_CLEANING_STRATEGY = A_STAR_RETURN_ORIGIN_WORKING_OVERALL_CLEANING_STRATEGY;
                right_running_step_status = 0;
                FunctionStatus = 0;
                break;
            }
            break;

        case A_STAR_RETURN_ORIGIN_WORKING_OVERALL_CLEANING_STRATEGY:
            log_debug("A_STAR_RETURN_ORIGIN_WORKING_OVERALL_CLEANING_STRATEGY");
            AStarReturnOrigin(current_pose, obstacleSignal);
            OVERALL_CLEANING_STRATEGY = A_STAR_MOTION_RETURN_ORIGIN_WORKING_OVERALL_CLEANING_STRATEGY;
            break;

        case A_STAR_MOTION_RETURN_ORIGIN_WORKING_OVERALL_CLEANING_STRATEGY:
            log_debug("A_STAR_MOTION_RETURN_ORIGIN_WORKING_OVERALL_CLEANING_STRATEGY");
            FunctionStatus = AStarMotionReturnOrigin(current_pose, obstacleSignal);
            if (1 == FunctionStatus)
            {
                a_star_motion_return_origin = 0;
                OVERALL_CLEANING_STRATEGY = RETURN_ORIGIN_WORKING_OVERALL_CLEANING_STRATEGY;
                FunctionStatus = 0;
                break;
            }
            if (2 == FunctionStatus)
            {
                OVERALL_CLEANING_STRATEGY = A_STAR_COLLISION_RETURN_ORIGIN_WORKING_OVERALL_CLEANING_STRATEGY;
                FunctionStatus = 0;
                break;
            }
            if (3 == FunctionStatus)
            {
                a_star_motion_return_origin = 0;
                OVERALL_CLEANING_STRATEGY = A_STAR_NOT_RETURN_ORIGIN_WORKING_OVERALL_CLEANING_STRATEGY;
                FunctionStatus = 0;
                break;
            }
            if (4 == FunctionStatus)
            {
                OVERALL_CLEANING_STRATEGY = RETURN_ORIGIN_WORKING_OVERALL_CLEANING_STRATEGY;
                a_star_motion_return_origin = 0;
                FunctionStatus = 0;
                break;
            }
            break;
        case A_STAR_NOT_RETURN_ORIGIN_WORKING_OVERALL_CLEANING_STRATEGY:
            log_debug("A_STAR_NOT_RETURN_ORIGIN_WORKING_OVERALL_CLEANING_STRATEGY");
            FunctionStatus = AStarNotMotionReturnOrigin(current_pose, obstacleSignal);
            if (1 == FunctionStatus)
            {
                //a_star_motion_return_origin=0;
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
            FunctionStatus = RightReturnOrigin(current_pose, obstacleSignal);
            if (1 == FunctionStatus)
            {
                if (selectside == 'L')
                {
                    OVERALL_CLEANING_STRATEGY = LEFT_RUNNING_WORKING_OVERALL_CLEANING_STRATEGY;
                }
                if (selectside == 'R')
                {
                    OVERALL_CLEANING_STRATEGY = RIGHT_RUNNING_WORKING_OVERALL_CLEANING_STRATEGY;
                }
                return_origin_step_status = 0;
                FunctionStatus = 0;
                break;
            }
            break;
        case LEFT_RUNNING_WORKING_OVERALL_CLEANING_STRATEGY:
            log_debug("LEFT_RUNNING_WORKING_OVERALL_CLEANING_STRATEGY");
            FunctionStatus = LeftRunningWorkStep(current_pose, obstacleSignal);
            if (1 == FunctionStatus)
            {
                selectside = 'R';
                OVERALL_CLEANING_STRATEGY = A_STAR_RETURN_ORIGIN_WORKING_OVERALL_CLEANING_STRATEGY;
                left_running_step_status = 0;
                FunctionStatus = 0;
                break;
            }
            break;
        }
	
		return 0;
}



//#################################################################################
//####################################################           RIGHT        #####    
//#################################################################################
////////////////////////////////////////////////////////////////////////////////////////////////////////
unsigned char RightRunningWorkStep(POSE *current_pose,unsigned char obstacleSignal)
{
	unsigned char complete_flag = 0;
    Yaw = current_pose->orientation;
    switch(right_running_step_status)
    {
    case 0:
        right_running_step_status = GOSTR_RIGHTRUN_STEP;
        break;
    case GOSTR_RIGHTRUN_STEP:
        log_debug("gostraight right run step!");
        if (my_abs(Yaw / 100) > 90 && my_abs(Yaw / 100) < 175)
        {
            log_debug("backaward Corrected heading angle !");
            if (Yaw > 0)
            {
                linear_velocity = 0;
                angular_velocity = 5*turn_vel/6;
                break;
            }
            else
            {
                linear_velocity = 0;
                angular_velocity = -5*turn_vel/6;
                break;
            }
        }
        else if (my_abs(Yaw / 100) < 90 && my_abs(Yaw / 100) > 5)
        {
            log_debug("gostraight Corrected heading angle !");
            if (Yaw > 0)
            {
                linear_velocity = 0;
                angular_velocity = -5*turn_vel/6;
                break;
            }
            else
            {
                linear_velocity = 0;
                angular_velocity = 5*turn_vel/6;
                break;
            }
        }
        else if(my_abs(current_pose->x) > W)
        {
            log_debug("current pose x arrived width max!");
            right_running_step_status = FORWARD_BOUNDARY_RIGHTRUN_STEP;
            break;
        }
        else if(my_abs(current_pose->y) > W)
        {
            log_debug("current pose y arrived width max!");
            complete_flag = 1;
            break;
        }
        else if (right_obstacle == obstacleSignal)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            //leakingsweep = gridmap.ReturnExtreme_point(Yaw / 100, obstacleSignal);
			//leakingsweep = bsp_Right_ReturnExtreme_point(current_pose->x,current_pose->y,current_pose->orientation,obstacleSignal);
			if(my_abs(leakingsweep_x-current_pose->x)>leakingsweep_X_interval&&my_abs(leakingsweep_y-current_pose->y)>leakingsweep_Y_interval){
                
				//sendvelocity(&linear_velocity,&angular_velocity);
				leakingsweep_x=current_pose->x;
                leakingsweep_y=current_pose->y;
                leakingsweep =bsp_Right_ReturnExtreme_point(current_pose->x,current_pose->y,current_pose->orientation,obstacleSignal);
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
            //leakingsweep = gridmap.ReturnExtreme_point(Yaw / 100, obstacleSignal);
            //leakingsweep = bsp_Right_ReturnExtreme_point(current_pose->x,current_pose->y,current_pose->orientation,obstacleSignal);
			if(my_abs(leakingsweep_x-current_pose->x)>leakingsweep_X_interval&&my_abs(leakingsweep_y-current_pose->y)>leakingsweep_Y_interval){
                
				//sendvelocity(&linear_velocity,&angular_velocity);
				leakingsweep_x=current_pose->x;
                leakingsweep_y=current_pose->y;
                leakingsweep =bsp_Right_ReturnExtreme_point(current_pose->x,current_pose->y,current_pose->orientation,obstacleSignal);
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
            //leakingsweep = gridmap.ReturnExtreme_point(Yaw / 100, obstacleSignal);
            //leakingsweep = bsp_Right_ReturnExtreme_point(current_pose->x,current_pose->y,current_pose->orientation,obstacleSignal);
			if(my_abs(leakingsweep_x-current_pose->x)>leakingsweep_X_interval&&my_abs(leakingsweep_y-current_pose->y)>leakingsweep_Y_interval){
                
				//sendvelocity(&linear_velocity,&angular_velocity);
				leakingsweep_x=current_pose->x;
                leakingsweep_y=current_pose->y;
                leakingsweep =bsp_Right_ReturnExtreme_point(current_pose->x,current_pose->y,current_pose->orientation,obstacleSignal);
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
        if(ForwardBoundaryRightRunStep(current_pose,obstacleSignal))
        {
            right_running_step_status = GOSTR_RIGHTRUN_STEP;
            right_forward_boundary_status=0;
        }
        break;

    case COLLISION_RIGHT_RIGHTRUN_STEP:
        log_debug("COLLISION right right run step!");
        FunctionStatus=CollisionRightRightRunStep(current_pose,obstacleSignal);
        if(1==FunctionStatus){
            right_running_step_status = GOSTR_RIGHTRUN_STEP;
            collision_right_rightrun_step_status = 0;
            FunctionStatus=0;
            break;
        }
        if(2==FunctionStatus){
            right_running_step_status =RIGHTWALKEDGE;
            collision_right_rightrun_step_status = 0;
            FunctionStatus=0;
            break;
        }
        break;
    case RIGHTWALKEDGE:
        log_debug("RIGHTWALKEDGE");
        FunctionStatus=RightWalkEdge(current_pose,obstacleSignal);
        if(1==FunctionStatus){
            right_running_step_status = GOSTR_RIGHTRUN_STEP;
            right_walk_edge_status = 0;
            FunctionStatus=0;
            break;
        }
        if(2==FunctionStatus){
            right_running_step_status =RIGHTEDGEDILEMMA;
            right_walk_edge_status = 0;
            FunctionStatus=0;
            break;
        }
		if(3==FunctionStatus){
            right_running_step_status =0;
            right_walk_edge_status = 0;
            FunctionStatus=0;
			complete_flag = 1;
            break;
        }
        break;
    case RIGHTEDGEDILEMMA:
        log_debug("RIGHTEDGEDILEMMA!");
        if(RightEdgeDilemma(current_pose,obstacleSignal))
        {
            right_running_step_status = GOSTR_RIGHTRUN_STEP;
            right_edge_dilemma_status=0;
        }
        break;



    case COLLISION_LEFT_RIGHTRUN_STEP:
        log_debug("COLLISION left right run step!");
        FunctionStatus=CollisionLeftRightRunStep(current_pose,obstacleSignal);
        if(1==FunctionStatus){
            right_running_step_status = GOSTR_RIGHTRUN_STEP;
            collision_right_rightrun_step_status = 0;
            FunctionStatus=0;
            break;
        }
        if(2==FunctionStatus){
            right_running_step_status =RIGHTREVERSEWALKEDGE;
            collision_right_rightrun_step_status = 0;
            FunctionStatus=0;
            break;
        }
        break;
    case RIGHTREVERSEWALKEDGE:
        log_debug("RIGHTREVERSEWALKEDGE");
        FunctionStatus=RightReverseWalkEdge(current_pose,obstacleSignal);
        if(1==FunctionStatus){
            right_running_step_status = GOSTR_RIGHTRUN_STEP;
            right_reverse_walk_edge_status= 0;
            FunctionStatus=0;
            break;
        }
        if(2==FunctionStatus){
            right_running_step_status =RIGHTEDGEDILEMMA;
            right_reverse_walk_edge_status = 0;
            FunctionStatus=0;
            break;
        }
		if(3==FunctionStatus){
            right_running_step_status =0;
            right_reverse_walk_edge_status = 0;
            FunctionStatus=0;
			complete_flag = 1;
            break;
        }
        break;
    case COLLISION_FRONT_RIGHTRUN_STEP:
        log_debug("COLLISION front right run step!");   
        FunctionStatus=CollisionFrontRightRunStep(current_pose,obstacleSignal);
        if(1==FunctionStatus){
            right_running_step_status = GOSTR_RIGHTRUN_STEP;
            collision_right_rightrun_step_status = 0;
            FunctionStatus=0;
            break;
        }
        if(2==FunctionStatus){
            right_running_step_status =RIGHTWALKEDGE;
            collision_right_rightrun_step_status = 0;
            FunctionStatus=0;
            break;
        }
        if(3==FunctionStatus){
            right_running_step_status =RIGHTREVERSEWALKEDGE;
            collision_right_rightrun_step_status = 0;
            FunctionStatus=0;
            break;
        }
        break;
    case LEAKING_SWEEP_RIGHTRUN_STEP:
        log_debug("leaking sweep right run step!");
        if(RightReadyLeakingSweep(current_pose,obstacleSignal))
        {
            right_running_step_status = GOSTR_RIGHTRUN_STEP;
            right_ready_leaking_sweep_status=0;
        }
        break;
    default:
        break;
    }
    sendvelocity(&linear_velocity,&angular_velocity);
    return complete_flag;
}
unsigned char CollisionRightRightRunStep(POSE *current_pose,unsigned char obstacleSignal)
{
    ////cout<<"CollisionRighttemporary_current_pose.x................============>>>>>>>>>>>"<<temporary_current_pose.x<<endl;
    //int Yaw;
    unsigned char complete_flag = 0;
    Yaw = current_pose->orientation;
    switch(collision_right_rightrun_step_status)
    {
    case 0:
//        if(turn_start_update == 0)
//        {
//            turn_start_x = current_pose->x;
//            turn_start_y = current_pose->y;
//            turn_start_update = 1;
//        }
//        linear_velocity = -long_stra_vel;
//        angular_velocity = 0;
//        if (my_abs(turn_start_x - current_pose->x) > collision_backward_distance || my_abs(turn_start_y - current_pose->y) > collision_backward_distance)
//        {
//            collision_right_rightrun_step_status = GOBACK_DISTANCE_CRRRS;
//            turn_start_update = 0;
//            break;
//        }
		collision_right_rightrun_step_status = GOBACK_DISTANCE_CRRRS;
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

        if (obstacleSignal == front_obstacle || obstacleSignal == left_obstacle || obstacleSignal == right_obstacle)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            if (my_abs(Yaw / 100) < 90)
            {
                collision_right_rightrun_step_status  = DIR_RIGHT_YAW_LESS_ABS90_CRRRS;
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
            collision_right_rightrun_step_status = TURN_CLOCK_TARGET_YAW_NEG27_CR_DRYL;
            break;
        }
        break;
    case GOSTR_YAW_EQUAL_NEG27_CR_DRYL:
        linear_velocity = long_stra_vel;
        angular_velocity = 0;
        if (obstacleSignal == front_obstacle || obstacleSignal == left_obstacle || obstacleSignal == right_obstacle)
        {
            collision_right_rightrun_step_status  = TURN_CLOCK_TARGET_YAW_NEG57_CR_DRYL;
            break;
        }
        if (my_abs(last_position_y - current_pose->y) > lateral_move_distance / 3)
        {
            collision_right_rightrun_step_status  = TURN_CLOCK_TARGET_YAW_NEG57_CR_DRYL;
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
            collision_right_rightrun_step_status  = GOSTR_YAW_EQUAL_NEG57_CR_DRYL;
            break;
        }
        if (obstacleSignal == front_obstacle || obstacleSignal == left_obstacle || obstacleSignal == right_obstacle)
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
        maintain_bow_distance = current_pose->y;
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
        maintain_bow_distance = current_pose->y;
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
        linear_velocity = bypass_velocity;
        angular_velocity = 0;
        if(cnt_update > 4&&my_abs(last_position_xx - current_pose->x)<20&&my_abs(last_position_y - current_pose->y)<20)
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
        if (my_abs(current_pose->x) > W)
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

        if (my_abs(turn_start_x - current_pose->x) > collision_backward_distance || my_abs(turn_start_y - current_pose->y) > collision_backward_distance)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            turn_start_update = 0;
            temporary_yaw = Yaw / 100;
            if (my_abs(Yaw / 100) > 165) collision_right_rightrun_step_status = TURN_CCLOCK_TARGET_YAW_MORE_ABS165_CR_DRYM;
            else collision_right_rightrun_step_status = TURN_CCLOCK_TARGET_YAW_LESS_ABS165_CR_DRYM;
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
        if (my_abs(Yaw / 100) > 150)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            temporary_yaw = Yaw / 100;
            collision_right_rightrun_step_status = TURN_CLOCK_TARGET_YAW_ABS150_CR_DRYM;
            break;
        }
        else if (my_abs(Yaw / 100) < 150 && Yaw / 100 < 0)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            temporary_yaw = Yaw / 100;
            collision_right_rightrun_step_status = TURN_CLOCK_YAW_ADD_ABS30_CR_DRYM;
            break;
        }
        else{
            linear_velocity = 0;
            angular_velocity = 0;
            collision_right_rightrun_step_status = GOSTR_BYPASS_CR_DRYM;
            break;
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


unsigned char CollisionLeftRightRunStep(POSE *current_pose,unsigned char obstacleSignal)
{
    //int Yaw;
    unsigned char complete_flag = 0;
    Yaw = current_pose->orientation;
    ////cout<<"collision_left_rightrun_step_status................============>>>>>>>>>>>"<<collision_left_rightrun_step_status<<endl;
    switch(collision_left_rightrun_step_status)
    {
    case 0:
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
            collision_left_rightrun_step_status = GOBACK_DISTANCE_CLRRS;
            turn_start_update = 0;
            break;
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

        if (obstacleSignal == front_obstacle || obstacleSignal == left_obstacle || obstacleSignal == right_obstacle)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            if (my_abs(Yaw / 100) < 90)
            {
                collision_left_rightrun_step_status  = DIR_RIGHT_YAW_LESS_ABS90_CLRRS;
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
    case  DIR_RIGHT_YAW_LESS_ABS90_CLRRS:
        linear_velocity = 0;
        angular_velocity = 0;
        collision_left_rightrun_step_status = TURN_CLOCK_TARGET_YAW_ABS30_CL_DRYL;
        break;
    case  TURN_CLOCK_TARGET_YAW_ABS30_CL_DRYL:
        linear_velocity = 0;
        angular_velocity = -turn_vel;
        if (my_abs(Yaw / 100) >30)
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
        linear_velocity = bypass_velocity;
        angular_velocity = 0;
        if(cnt_update > 4&&my_abs(last_position_xx - current_pose->x)<20&&my_abs(last_position_y - current_pose->y)<20)
        {
            cnt_update = 0;
            collision_left_rightrun_step_status = MORE_TRY_BREAK_BYPASS_CL_DRYL;
            break;
        }
        collision_left_rightrun_step_status = GOSTR_BYPASS_LOOP_CL_DRYL;
        break;
    case  GOSTR_BYPASS_LOOP_CL_DRYL:
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
        if (my_abs(current_pose->x) > W)
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

    case  GOSTR_BYPASS_BOW_CONTINUE_CL_DRYL:
        if (obstacleSignal == front_obstacle || obstacleSignal == right_obstacle || obstacleSignal == left_obstacle)
        {
            collision_left_rightrun_step_status = GOSTR_BYPASS_BOW_CONTINUE_COLLISION_CL_DRYL;
            break;
        }
        if (my_abs(Yaw / 100) > 175 )
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
            if (my_abs(Yaw / 100) <15){
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
    case  TURN_CLCOK_TARGET_YAW_LESS_ABS15_CL_DRYL:
        if (obstacleSignal == front_obstacle || obstacleSignal == right_obstacle || obstacleSignal == left_obstacle)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            collision_left_rightrun_step_status = TURN_CLCOK_TARGET_YAW_LESS_ABS15_COLLISION_CL_DRYL;
            break;
        }
        if (my_abs(Yaw / 100) >15)
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
        else{
            linear_velocity = 0;
            angular_velocity = 0;
            collision_left_rightrun_step_status = GOSTR_BYPASS_CL_DRYL;
            break;
        }
        break;
    case  TURN_CCLOCK_TARGET_YAW_MORE_AB30_CL_DRYL:
        if (my_abs(Yaw / 100) >30)
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
            collision_left_rightrun_step_status = TURN_CCLOCK_TARGET_YAW_MORE_AB30_CL_DRYL;
            turn_start_update = 0;
            break;
        }
        break;
    case  TURN_CCLOCK_YAW_ADD_ABS30_CL_DRYL:
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
            collision_left_rightrun_step_status = TURN_CCLOCK_YAW_ADD_ABS30_CL_DRYL;
            turn_start_update = 0;
            break;
        }
        break;
    case  MORE_TRY_BREAK_BYPASS_CL_DRYL:
        if (obstacleSignal == front_obstacle || obstacleSignal == right_obstacle || obstacleSignal == left_obstacle)
        {
            collision_left_rightrun_step_status = GOSTR_BYPASS_BOW_CONTINUE_COLLISION_CL_DRYL;
            bow_continue = true;
            break;
        }
        if (my_abs(Yaw / 100) > 173)
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
        collision_left_rightrun_step_status = 0;
        complete_flag = 1;
        break;
    case  DIR_RIGHT_YAW_MORE_ABS90_CLRRS:
        linear_velocity = 0;
        angular_velocity = 0;

        collision_left_rightrun_step_status = TURN_CCLOCK_TARGET_YAW_ABS153_CL_DRYM;
        break;
    case  TURN_CCLOCK_TARGET_YAW_ABS153_CL_DRYM:
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
        if (obstacleSignal == front_obstacle || obstacleSignal == left_obstacle || obstacleSignal == right_obstacle)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            collision_left_rightrun_step_status = TURN_CCLOCK_TAEGET_YAW_ABS153_COLLISION_CL_DRYM;
            break;
        }
        break;
    case  TURN_CCLOCK_TAEGET_YAW_ABS153_COLLISION_CL_DRYM:
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
            collision_left_rightrun_step_status = TURN_CCLOCK_TARGET_YAW_ABS153_CL_DRYM;
            break;
        }
        break;
    case  GOSTR_YAW_EQUAL_ABS153_CL_DRYM:
        linear_velocity = long_stra_vel;
        angular_velocity = 0;
        if (obstacleSignal == front_obstacle || obstacleSignal == left_obstacle || obstacleSignal == right_obstacle)
        {
            collision_left_rightrun_step_status  = TURN_CCLOCK_TARGET_YAW_NEG123_CL_DRYM;
            break;
        }
        if (my_abs(last_position_y - current_pose->y) > lateral_move_distance / 3)
        {
            collision_left_rightrun_step_status  = TURN_CCLOCK_TARGET_YAW_NEG123_CL_DRYM;
            break;
        }
        break;
    case  TURN_CCLOCK_TARGET_YAW_NEG123_CL_DRYM:
        linear_velocity = 0;
        angular_velocity = turn_vel;
        if (Yaw / 100 > -123)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            collision_left_rightrun_step_status  = GOSTR_YAW_EQUAL_NEG123_CL_DRYM;
            break;
        }
        if (obstacleSignal == front_obstacle || obstacleSignal == left_obstacle || obstacleSignal == right_obstacle)
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
        linear_velocity = long_stra_vel;
        angular_velocity = 0;
        if (obstacleSignal == front_obstacle || obstacleSignal == left_obstacle || obstacleSignal == right_obstacle)
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
    case  TURN_CCLOCK_TARGET_YAW_NEG98_CL_DRYM:
        linear_velocity = 0;
        angular_velocity = turn_vel;
        if (Yaw / 100 > -98)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            collision_left_rightrun_step_status = GOSTR_YAW_EQUAL_NEG98_CL_DRYM;
            break;
        }
        if (obstacleSignal == front_obstacle || obstacleSignal == left_obstacle || obstacleSignal == right_obstacle)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            collision_left_rightrun_step_status = TURN_CCLOCK_TARGET_YAW_NEG98_COLLISION_CL_DRYM;
            break;
        }
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
        if (obstacleSignal == front_obstacle || obstacleSignal == left_obstacle || obstacleSignal == right_obstacle)
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
        collision_left_rightrun_step_status = 0;
        complete_flag = 2;
        break;

    case  TURN_CCLOCK_TARGET_YAW_ABS8_CL_DRYM:
        linear_velocity = 0;
        angular_velocity = turn_vel;
        if (my_abs(Yaw / 100) < 8)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            collision_left_rightrun_step_status = COMPLETE_CL_DRYM;
            break;
        }
        if (obstacleSignal == front_obstacle || obstacleSignal == left_obstacle || obstacleSignal == right_obstacle)
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
        maintain_bow_distance = current_pose->y;
        collision_left_rightrun_step_status = 0;
        complete_flag = 1;
        break;
    }
    return complete_flag;
}


unsigned char CollisionFrontRightRunStep(POSE *current_pose,unsigned char obstacleSignal)
{
    //Callport();
    //int Yaw;
    unsigned char complete_flag = 0;
    Yaw = current_pose->orientation;
    switch(collision_front_rightrun_step_status)
    {
    case 0:
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
            collision_front_rightrun_step_status = GOBACK_DISTANCE_CFRRS;
            turn_start_update = 0;
            break;
        }
        break;
    case GOBACK_DISTANCE_CFRRS:
        if(turn_start_update == 0)
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
                collision_front_rightrun_step_status  = DIR_RIGHT_YAW_LESS_ABS90_CFRRS;
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
        if (obstacleSignal == front_obstacle || obstacleSignal == left_obstacle || obstacleSignal == right_obstacle)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            collision_front_rightrun_step_status = TURN_CLOCK_TARGET_YAW_NEG60_COLLISION_CF_DRYL;
            break;
        }
        break;
    case TURN_CLOCK_TARGET_YAW_NEG60_COLLISION_CF_DRYL:
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
            collision_front_rightrun_step_status = TURN_CLOCK_TARGET_YAW_NEG60_CF_DRYL;
            break;
        }
        break;
    case GOSTR_YAW_EQUAL_NEG60_CF_CF_DRYL:
        linear_velocity = long_stra_vel;
        angular_velocity = 0;
        if (obstacleSignal == front_obstacle || obstacleSignal == left_obstacle || obstacleSignal == right_obstacle)
        {
            collision_front_rightrun_step_status  = TURN_CLOCK_TARGET_YAW_NEG82_CF_DRYL;
            break;
        }
        if (my_abs(last_position_y - current_pose->y) > lateral_move_distance)
        {
            collision_front_rightrun_step_status  = TURN_CLOCK_TARGET_YAW_NEG82_CF_DRYL;
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
            collision_front_rightrun_step_status  = GOSTR_YAW_EQUAL_NEG82_CF_DRYL;
            break;
        }
        if (obstacleSignal == front_obstacle || obstacleSignal == left_obstacle || obstacleSignal == right_obstacle)
        {
            collision_front_rightrun_step_status  = TURN_CLOCK_TARGET_YAW_NEG82_COLLISION_CF_DRYL;
            break;
        }
        break;
    case TURN_CLOCK_TARGET_YAW_NEG82_COLLISION_CF_DRYL:
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
            collision_front_rightrun_step_status = TURN_CLOCK_TARGET_YAW_NEG82_CF_DRYL;
            break;
        }
        break;
    case GOSTR_YAW_EQUAL_NEG82_CF_DRYL:
        linear_velocity = long_stra_vel;
        angular_velocity = 0;
        if (obstacleSignal == front_obstacle || obstacleSignal == left_obstacle || obstacleSignal == right_obstacle)
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
        collision_front_rightrun_step_status=0;
        complete_flag=2;
        break;
    case TURN_CLOCK_TARGET_YAW_ABS173_CF_DRYL:
        linear_velocity = 0;
        angular_velocity = -turn_vel;
        if (my_abs(Yaw / 100) > 173 )
        {
            linear_velocity = 0;
            angular_velocity = 0;
            collision_front_rightrun_step_status  = COMPLETE_CF_DRYL;
            break;
        }
        if (obstacleSignal == front_obstacle || obstacleSignal == left_obstacle || obstacleSignal == right_obstacle)
        {
            collision_front_rightrun_step_status  = TURN_CLOCK_TARGET_YAW_ABS173_COLLISION_CF_DRYL;
            break;
        }
        break;
    case TURN_CLOCK_TARGET_YAW_ABS173_COLLISION_CF_DRYL:
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
        if (obstacleSignal == front_obstacle || obstacleSignal == left_obstacle || obstacleSignal == right_obstacle)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            collision_front_rightrun_step_status = TURN_CCLOCK_TARGET_YAW_ABS120_COLLISION_CF_DRYM;
            break;
        }
        break;
    case TURN_CCLOCK_TARGET_YAW_ABS120_COLLISION_CF_DRYM:
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
            collision_front_rightrun_step_status = TURN_CCLOCK_TARGET_YAW_ABS120_CF_DRYM;
            break;
        }
        break;
    case GOSTR_YAW_ABS120_CF_DRYM:
        linear_velocity = long_stra_vel;
        angular_velocity = 0;
        if (obstacleSignal == front_obstacle || obstacleSignal == left_obstacle || obstacleSignal == right_obstacle)
        {
            collision_front_rightrun_step_status  = TURN_CCLOCK_TARGET_YAW_ABS93_CF_DRYM;
            break;
        }
        if (my_abs(last_position_y - current_pose->y) > lateral_move_distance)
        {
            collision_front_rightrun_step_status  = TURN_CCLOCK_TARGET_YAW_ABS93_CF_DRYM;
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
            collision_front_rightrun_step_status  = GOSTR_YAW_ABS93_CF_DRYM;
            break;
        }
        if (obstacleSignal == front_obstacle || obstacleSignal == left_obstacle || obstacleSignal == right_obstacle)
        {
            collision_front_rightrun_step_status  = TURN_CCLOCK_TARGET_YAW_ABS93_COLLISION_CF_DRYM;
            break;
        }
        break;
    case TURN_CCLOCK_TARGET_YAW_ABS93_COLLISION_CF_DRYM:
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
            collision_front_rightrun_step_status = TURN_CCLOCK_TARGET_YAW_ABS93_CF_DRYM;
            break;
        }
        break;
    case GOSTR_YAW_ABS93_CF_DRYM:
        linear_velocity = long_stra_vel;
        angular_velocity = 0;
        if (obstacleSignal == front_obstacle || obstacleSignal == left_obstacle || obstacleSignal == right_obstacle)
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
        if (my_abs(Yaw / 100)< 10 )
        {
            linear_velocity = 0;
            angular_velocity = 0;
            collision_front_rightrun_step_status  = COMPLETE_CF_DRYL;
            break;
        }
        if (obstacleSignal == front_obstacle || obstacleSignal == left_obstacle || obstacleSignal == right_obstacle)
        {
            collision_front_rightrun_step_status  = TURN_CCLOCK_TARGET_YAW_ABS10_COLLISION_CF_DRYM;
            break;
        }

        break;
    case TURN_CCLOCK_TARGET_YAW_ABS10_COLLISION_CF_DRYM:
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
unsigned char RightEdgeDilemma(POSE *current_pose,unsigned char obstacleSignal)
{
    unsigned char complete_flag = 0;
    Yaw = current_pose->orientation;
    switch(right_edge_dilemma_status)
    {
        case 0:
            last_position_x = current_pose->x + W;
            right_edge_dilemma_status = DELTA_X_MORE_ONE_THIRD_CLEANED_MAP_WIDTH_DILEMMA;
            break;
        case DELTA_X_MORE_ONE_THIRD_CLEANED_MAP_WIDTH_DILEMMA:
            //if (my_abs(last_position_x - (current_pose->x + W)) > gridmap.Edge_length() / 3)
            if (my_abs(last_position_x - (current_pose->x + W)) > 1000)
            {
                complete_flag = 1;
                right_edge_dilemma_status = COMPLETE_DEILEMMA;
                number = 0;
                break;
            }
            right_edge_dilemma_status = LOOP_TEN_NUM_DILEMMA;
            break;
        case LOOP_TEN_NUM_DILEMMA:
            if(number >4)
            {
                complete_flag = 1;
                right_edge_dilemma_status = COMPLETE_DEILEMMA;
                number = 0;
                break;
            }
            right_edge_dilemma_status = YAW_MORE_ABS90_LESS_ABS176_CCLOCK_LESS_ABS90_MORE_ABS4_CLOCK_DILEMMA;
            break;
		case COMPLETE_DEILEMMA:
			complete_flag = 1;
			right_edge_dilemma_status = 0;
			number = 0;
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
                linear_velocity = 200;
                angular_velocity = 0;
            }
            right_edge_dilemma_status = GOSTR_DILEMMA;
            break;
        case GOSTR_DILEMMA:
            if (right_obstacle == obstacleSignal || front_obstacle == obstacleSignal || left_obstacle == obstacleSignal)
            {
                right_edge_dilemma_status = GOSTR_COLLISION_DILEMMA;
                break;
            }
            right_edge_dilemma_status = DELTA_X_MORE_ONE_THIRD_CLEANED_MAP_WIDTH_DILEMMA;
            break;
        case GOSTR_COLLISION_DILEMMA:
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
                last_position_x = current_pose->x + W;
                right_edge_dilemma_status = GOSTR_CYM_DILEMMA;
                break;
            }
            if (obstacleSignal == front_obstacle || obstacleSignal == left_obstacle || obstacleSignal == right_obstacle)
            {
                right_edge_dilemma_status =  CLOCK_TARGET_YAW_LESS_ABS90_COLLISION_DILEMMA;
            }
            linear_velocity = 0;
            angular_velocity = -turn_vel;
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
            }
            break;
        case GOSTR_CYM_DILEMMA:
            if (obstacleSignal == front_obstacle || obstacleSignal == right_obstacle || obstacleSignal == left_obstacle)
            {
                number++;
                right_edge_dilemma_status  = GOSTR_COLLISION_CYM_DILEMMA;
                break;
            }
            if (my_abs(last_position_y - current_pose->y) > lateral_move_distance / 3)
            {
                right_edge_dilemma_status  = GOSTR_DELTA_Y_MORE_LATERAL_DIS_CYM_DILEMMA;
                break;
            }
            linear_velocity = long_stra_vel;
            angular_velocity = 0;
            break;
        case GOSTR_COLLISION_CYM_DILEMMA:
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
                right_edge_dilemma_status  = CLOCK_TARGET_YAW_LESS_ABS3_DILEMMA;
                break;
            }
            break;

        case GOSTR_DELTA_Y_MORE_LATERAL_DIS_CYM_DILEMMA:
            right_edge_dilemma_status  = CLOCK_TARGET_YAW_LESS_ABS3_DILEMMA;
            break;
        case CLOCK_TARGET_YAW_LESS_ABS3_DILEMMA:
            if (my_abs(Yaw / 100) < 5)
            {
                linear_velocity = 0;
                angular_velocity = 0;
                right_edge_dilemma_status = DELTA_X_MORE_ONE_THIRD_CLEANED_MAP_WIDTH_DILEMMA;
                break;
            }
            if (right_obstacle == obstacleSignal || front_obstacle == obstacleSignal || left_obstacle == obstacleSignal)
            {
                right_edge_dilemma_status = CLOCK_TARGET_YAW_LESS_ABS3_COLLISION_DILEMMA;
                break;
            }
            linear_velocity = 0;
            angular_velocity = -turn_vel;
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
                last_position_x = current_pose->x + W;
                right_edge_dilemma_status = GOSTR_CYL_DILEMMA;
                break;
            }
            if (obstacleSignal == front_obstacle || obstacleSignal == left_obstacle || obstacleSignal == right_obstacle)
            {
                right_edge_dilemma_status =  CCLOCK_TARGET_YAW_MORE_ABS90_COLLISION_DILEMMA;
                break;
            }

            linear_velocity = 0;
            angular_velocity = turn_vel;
            break;
        case CCLOCK_TARGET_YAW_MORE_ABS90_COLLISION_DILEMMA:
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
                right_edge_dilemma_status = CCLOCK_TARGET_YAW_MORE_ABS90_DILEMMA;
            }
            break;
        case GOSTR_CYL_DILEMMA:
            if (obstacleSignal == front_obstacle || obstacleSignal == right_obstacle || obstacleSignal == left_obstacle)
            {
                number++;
                right_edge_dilemma_status  = GOSTR_COLLISION_CYL_DILEMMA;
                break;
            }
            if (my_abs(last_position_y - current_pose->y) > lateral_move_distance / 3)
            {
                right_edge_dilemma_status  = GOSTR_GOSTR_DELTA_Y_MORE_LATERAL_DIS_CYL_DILEMMA;
                break;
            }
            linear_velocity = long_stra_vel;
            angular_velocity = 0;
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
                right_edge_dilemma_status = CCLOCK_TARGET_YAW_MORE_ABS178_DILEMMA;
                break;
            }
            break;
        case GOSTR_GOSTR_DELTA_Y_MORE_LATERAL_DIS_CYL_DILEMMA:
            right_edge_dilemma_status  = CCLOCK_TARGET_YAW_MORE_ABS178_DILEMMA;
            break;
        case CCLOCK_TARGET_YAW_MORE_ABS178_DILEMMA:
            if (my_abs(Yaw / 100) >175)
            {
                linear_velocity = 0;
                angular_velocity = 0;
                right_edge_dilemma_status = DELTA_X_MORE_ONE_THIRD_CLEANED_MAP_WIDTH_DILEMMA;
                break;
            }
            if (right_obstacle == obstacleSignal || front_obstacle == obstacleSignal || left_obstacle == obstacleSignal)
            {
                right_edge_dilemma_status = CCLOCK_TARGET_YAW_MORE_ABS178_COLLISION_DILEMMA;
                break;
            }
            linear_velocity = 0;
            angular_velocity = turn_vel;
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
                right_edge_dilemma_status = CCLOCK_TARGET_YAW_MORE_ABS178_DILEMMA;
            }
            break;
    }
    return complete_flag;
}


unsigned char RightWalkEdge(POSE *current_pose,unsigned char obstacleSignal)
{
    //int Yaw;
    unsigned char complete_flag = 0;
    Yaw = current_pose->orientation;
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
        if (my_abs(Yaw / 100) > 117)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            right_walk_edge_status = READY_GOSTR_BYPASS_WE;
            break;
        }
        if (obstacleSignal == front_obstacle || obstacleSignal == left_obstacle || obstacleSignal == right_obstacle)
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
        edge_length_start = current_pose->x + W;
        returnorigin = false;
        b_last_position_yy = false;
        last_position_y = current_pose->y;
        last_position_yy = 0;
        temporary_close_edge = close_edge;
        right_walk_edge_status = GOSTR_BYPASS_WE_X;
        break;

    case GOSTR_BYPASS_WE_X:
        last_position_x = current_pose->x;
        right_walk_edge_status = GOSTR_BYPASS_WE;
        break;

    case GOSTR_BYPASS_WE:
        linear_velocity = long_stra_vel;
        angular_velocity =0;
        if (obstacleSignal == front_obstacle || obstacleSignal == right_obstacle || obstacleSignal == left_obstacle)
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
        if (my_abs(Yaw / 100) < 105 && (Yaw / 100 > 0))
        {
            right_walk_edge_status = TARGET_YAW_LESS_ABS105_MORE_0_BYPASS_WE;
            break;
        }




        // *********************************************************************************************************************//
        //if ((my_abs(current_pose->x + W - edge_length_start) > 3 * gridmap.Edge_length() / 4))
        if ((my_abs(current_pose->x + W - edge_length_start) > 3000))
        {
            right_walk_edge_status = DELTA_X_MORE_ONE_FOURTH_CLEANED_MAP_WIDTH_WE;
            break;
        }
        right_walk_edge_status = GOSTR_BYPASS_WE_X;
        break;
    case COLLISION_BYPASS_WE:
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
        if (obstacleSignal == front_obstacle || obstacleSignal == left_obstacle || obstacleSignal == right_obstacle)
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
        //  *********************************************************************************************//
    case TARGET_YAW_LESS_ABS105_MORE_0_BYPASS_WE:
        //if (my_abs(current_pose->x + W - edge_length_start) > (gridmap.Edge_length() / 4))
        if (my_abs(current_pose->x + W - edge_length_start) > 2000)
        {
            right_walk_edge_status = RETURN_ORIGIN_WE;
            break;
        }
        else
        {
            right_walk_edge_status = TURN_CLOCK_TARGET_YAW_LESS_ABS3_WE;
            break;
        }
    case DELTA_X_MORE_ONE_FOURTH_CLEANED_MAP_WIDTH_WE:
        right_walk_edge_status = RETURN_ORIGIN_WE;
        break;
    case TURN_CLOCK_TARGET_YAW_LESS_ABS3_WE:
        if (my_abs(Yaw / 100) < 5)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            right_walk_edge_status = RIGHT_EDGE_DILEMMA_WE;
            break;
        }

        //   ***********************************************************************************************/////////////
        if (obstacleSignal == front_obstacle || obstacleSignal == left_obstacle || obstacleSignal == right_obstacle)
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

    case RIGHT_EDGE_DILEMMA_WE:
        linear_velocity = long_stra_vel;
        angular_velocity = 0;
        complete_flag = 2;
        right_walk_edge_status = 0;
        break;

    case GOSTR_X_MORE_LATERALDIS_BYPASS_WE:
        temporary_yaw = Yaw / 100;
        if (my_abs(Yaw / 100) < 135 && (Yaw / 100 > 0))
        {
            right_walk_edge_status = TURN_CCLOCK_TARGET_YAW_LESS_0_WE;
            break;
        }
        else if (my_abs(Yaw / 100) > 135)
        {
            right_walk_edge_status = TURN_CCLOCK_TARGET_YAW_LESS_135_DETAL_YAW_MORE_30_WE;
            break;
        }
        else{
            right_walk_edge_status = GOSTR_BYPASS_WE_X;
            break;
        }
       break;
    case TURN_CCLOCK_TARGET_YAW_LESS_0_WE:
        linear_velocity = 0;
        angular_velocity = turn_vel;
        if (Yaw / 100  < 0 )
        {
            linear_velocity = 0;
            angular_velocity = 0;
            right_walk_edge_status = GOSTR_BYPASS_WE_X;
            break;
        }
        if (obstacleSignal == front_obstacle || obstacleSignal == left_obstacle || obstacleSignal == right_obstacle)
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
        if (obstacleSignal == front_obstacle || obstacleSignal == left_obstacle || obstacleSignal == right_obstacle)
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
            right_walk_edge_status = TURN_CCLOCK_TARGET_YAW_LESS_135_DETAL_YAW_MORE_30_WE;
            break;
        }
        break;
    case BOW_CONTINUE_WE:
        linear_velocity = 0;
        angular_velocity = 0;
        complete_flag = 1;
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



unsigned char RightReverseWalkEdge(POSE *current_pose,unsigned char obstacleSignal)
{
    //int Yaw;
    unsigned char complete_flag = 0;
    Yaw = current_pose->orientation;
    switch(right_reverse_walk_edge_status)
    {
    case 0:
        right_reverse_walk_edge_status = GOBACK_REVERSE_WALK_EDGE;
    case GOBACK_REVERSE_WALK_EDGE                                       :
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
    case TURN_CCLOCK_TARGET_YAW_LESS_ABS63_RWE                          :
        linear_velocity = 0;
        angular_velocity = turn_vel;
        if (my_abs(Yaw / 100) < 63)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            right_reverse_walk_edge_status = READY_GOSTR_BYPASS_RWE;
            break;
        }
        if (obstacleSignal == front_obstacle || obstacleSignal == left_obstacle || obstacleSignal == right_obstacle)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            right_reverse_walk_edge_status = TURN_CCLOCK_TARGET_YAW_LESS_ABS63_COLLISION_RWE;
            break;
        }
        break;
    case TURN_CCLOCK_TARGET_YAW_LESS_ABS63_COLLISION_RWE                 :
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
    case READY_GOSTR_BYPASS_RWE                                         :
        edge_length_start = current_pose->x + W;
        returnorigin = false;
        b_last_position_yy = false;
        last_position_y = current_pose->y;
        last_position_yy = 0;
        temporary_close_edge = close_edge;
        right_reverse_walk_edge_status = GOSTR_BYPASS_RWE_X;
        break;
    case GOSTR_BYPASS_RWE_X:
        last_position_x = current_pose->x;
        right_reverse_walk_edge_status = GOSTR_BYPASS_RWE;
        break;
    case GOSTR_BYPASS_RWE                                               :
        linear_velocity = long_stra_vel;
        angular_velocity =0;
        if (obstacleSignal == front_obstacle || obstacleSignal == right_obstacle || obstacleSignal == left_obstacle)
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
        if (my_abs(Yaw / 100) > 75 && (Yaw / 100 > 0))
        {
            right_reverse_walk_edge_status = TARGET_YAW_MORE_ABS75_MORE_ABS0_RWE;
            break;
        }


        //   ****************************************************************//
        //if ((my_abs(current_pose->x + W - edge_length_start) > 3 * gridmap.Edge_length() / 4))
        if ((my_abs(current_pose->x + W - edge_length_start) > 3000))
        {
            right_reverse_walk_edge_status = DELTA_X_MORE_ONE_FOURTH_CLEANED_MAP_WIDTH_RWE;
            break;
        }
        right_reverse_walk_edge_status = GOSTR_BYPASS_RWE_X;
        break;
    case COLLISION_BYPASS_RWE                                            :
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
            temporary_yaw = Yaw / 100;
            right_reverse_walk_edge_status = TURN_CCLOCK_YAW_ADD_ABS15_RWE;
            break;
        }
        break;
    case TURN_CCLOCK_YAW_ADD_ABS15_RWE                                  :
        linear_velocity = 0;
        angular_velocity = turn_vel;
        if (my_abs(Yaw / 100 - temporary_yaw) > 15)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            right_reverse_walk_edge_status = REBACK_GOSTR_BYPASS_CHECK_RWE;
            break;
        }
        if (obstacleSignal == front_obstacle || obstacleSignal == left_obstacle || obstacleSignal == right_obstacle)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            right_reverse_walk_edge_status = TURN_CCLOCK_YAW_ADD_ABS15_COLLISION_RWE;
            break;
        }
        break;
    case TURN_CCLOCK_YAW_ADD_ABS15_COLLISION_RWE                         :
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
    case TARGET_YAW_MORE_ABS75_MORE_ABS0_RWE                            :

        if (my_abs(current_pose->x + W - edge_length_start) >2000)//if (my_abs(current_pose->x + W - edge_length_start) > (gridmap.Edge_length() / 4))
        {
            right_reverse_walk_edge_status = RETURN_ORIGIN_RWE;
            break;
        }
        else
        {
            right_reverse_walk_edge_status = TURN_CCLOCK_YAW_MORE_178ABS_RWE;
            break;
        }
        break;
    case DELTA_X_MORE_ONE_FOURTH_CLEANED_MAP_WIDTH_RWE                  :
        right_reverse_walk_edge_status = RETURN_ORIGIN_RWE;
        break;
    case TURN_CCLOCK_YAW_MORE_178ABS_RWE                                :
        if (my_abs(Yaw / 100) >175)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            right_reverse_walk_edge_status = RIGHT_EDGE_DILEMMA_RWE;
            break;
        }
        if (obstacleSignal == front_obstacle || obstacleSignal == left_obstacle || obstacleSignal == right_obstacle)
        {
            right_reverse_walk_edge_status = TURN_CCLOCK_YAW_MORE_178ABS_COLLISION_RWE;
            break;
        }
        linear_velocity = 0;
        angular_velocity = turn_vel;
        break;
    case TURN_CCLOCK_YAW_MORE_178ABS_COLLISION_RWE                       :
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
    case RIGHT_EDGE_DILEMMA_RWE:
        linear_velocity = long_stra_vel;
        angular_velocity = 0;
        complete_flag = 2;
        right_reverse_walk_edge_status = 0;
        break;

    case GOSTR_X_MORE_LATERALDIS_BYPASS_RWE                             :
        temporary_yaw = Yaw / 100;
        if (my_abs(Yaw / 100) > 45 && (Yaw / 100 > 0))
        {
            right_reverse_walk_edge_status = TURN_CLOCK_TARGET_YAW_LESS_0_RWE;
            break;
        }
        else if (my_abs(Yaw / 100) <= 45 )
        {
            right_reverse_walk_edge_status = TURN_CLOCK_TARGET_YAW_LESS_ABS45_DETAL_YAW_MORE_30_RWE;
            break;
        }
        else{
            right_reverse_walk_edge_status = GOSTR_BYPASS_RWE_X;
            break;
        }
        break;

    case TURN_CLOCK_TARGET_YAW_LESS_0_RWE                 :
        linear_velocity = 0;
        angular_velocity = -turn_vel;
        if (Yaw / 100 < 0 )
        {
            linear_velocity = 0;
            angular_velocity = 0;
            right_reverse_walk_edge_status = GOSTR_BYPASS_RWE_X;
            break;
        }
        if (obstacleSignal == front_obstacle || obstacleSignal == left_obstacle || obstacleSignal == right_obstacle)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            right_reverse_walk_edge_status = TURN_CLOCK_TARGET_YAW_LESS_0_COLLISION_RWE;
            break;
        }
        break;
    case TURN_CLOCK_TARGET_YAW_LESS_0_COLLISION_RWE      :
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
            right_reverse_walk_edge_status = TURN_CLOCK_TARGET_YAW_LESS_0_RWE;
            break;
        }
        break;
    case TURN_CLOCK_TARGET_YAW_LESS_ABS45_DETAL_YAW_MORE_30_RWE         :
        linear_velocity = 0;
        angular_velocity = -turn_vel;
        if (my_abs(temporary_yaw - Yaw / 100) > 30 && my_abs(Yaw / 100) > 45)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            right_reverse_walk_edge_status = GOSTR_BYPASS_RWE_X;
            break;
        }
        if (obstacleSignal == front_obstacle || obstacleSignal == left_obstacle || obstacleSignal == right_obstacle)
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
            right_reverse_walk_edge_status = TURN_CLOCK_TARGET_YAW_LESS_ABS45_DETAL_YAW_MORE_30_RWE;
            break;
        }
        break;
    case BOW_CONTINUE_RWE                                               :
        linear_velocity = 0;
        angular_velocity = 0;
        complete_flag = 1;
        right_reverse_walk_edge_status = 0;
        break;
    case RETURN_ORIGIN_RWE                                              :
        linear_velocity = 0;
        angular_velocity = 0;
        complete_flag = 3;
        right_reverse_walk_edge_status = 0;
        break;
    }
    return complete_flag;
}

unsigned char ForwardBoundaryRightRunStep(POSE *current_pose,unsigned char obstacleSignal)
{
    //int Yaw;
    unsigned char complete_flag = 0;
    Yaw = current_pose->orientation;
    switch(right_forward_boundary_status)
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
        if (my_abs(Yaw / 100) >175)
        {
            right_forward_boundary_status = FORWARDBOUNDARY_GOSTRAIGHT;
            break;
        }
        if (obstacleSignal == front_obstacle || obstacleSignal == left_obstacle || obstacleSignal == right_obstacle)
        {
            right_forward_boundary_status = FORWARDBOUNDARY_CLOCK_TARGET_YAW_MORE_ABS178_COLLISION;
            break;
        }
        break;
    case FORWARDBOUNDARY_CLOCK_TARGET_YAW_MORE_ABS178_COLLISION:
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
        if (obstacleSignal == front_obstacle || obstacleSignal == left_obstacle || obstacleSignal == right_obstacle)
        {
            right_forward_boundary_status = FORWARDBOUNDARY_CCLOCK_TARGET_YAW_LESS_ABS3_COLLISION;
            break;
        }

        break;
    case FORWARDBOUNDARY_CCLOCK_TARGET_YAW_LESS_ABS3_COLLISION:
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
            right_forward_boundary_status = FORWARDBOUNDARY_CCLOCK_TARGET_YAW_LESS_ABS3;
            break;
        }
        break;
    case FORWARDBOUNDARY_GOSTRAIGHT:
        if(turn_start_update == 0)
        {
            turn_start_x = current_pose->x;
            turn_start_y = current_pose->y;
            turn_start_update = 1;
        }
        if ((my_abs(turn_start_x - current_pose->x) > 50 || my_abs(turn_start_y - current_pose->y) >50)&&my_abs(Yaw / 100) < 90&&current_pose->x>0)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            turn_start_update = 0;
            right_forward_boundary_status = FORWARDBOUNDARY_COMPLETE;
            break;
        }
        if ((my_abs(turn_start_x - current_pose->x) > 50 || my_abs(turn_start_y - current_pose->y) >50)&&my_abs(Yaw / 100) >90&&current_pose->x<0)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            turn_start_update = 0;
            right_forward_boundary_status = FORWARDBOUNDARY_COMPLETE;
            break;
        }

        linear_velocity = long_stra_vel;
        angular_velocity = 0;
        if (obstacleSignal == front_obstacle || obstacleSignal == left_obstacle || obstacleSignal == right_obstacle)
        {
            right_forward_boundary_status = FORWARDBOUNDARY_COMPLETE;
            break;
        }
        if (my_abs(current_pose->x)<W)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            right_forward_boundary_status = FORWARDBOUNDARY_COMPLETE;
            break;
        }
        break;
    case FORWARDBOUNDARY_COMPLETE:
        right_forward_boundary_status = 0;
        complete_flag = 1;
        maintain_bow_distance = current_pose->y;
        break;
    }
    return complete_flag;
}
unsigned char RightReadyLeakingSweep(POSE *current_pose,unsigned char obstacleSignal)
{
    //int Yaw;
    unsigned char complete_flag = 0;
    Yaw = current_pose->orientation;
    switch(right_ready_leaking_sweep_status)
    {

    case 0:
        right_ready_leaking_sweep_status = RIGHT_LEAKING_SWEEP_COLLISION;
        break;
    case RIGHT_LEAKING_SWEEP_COLLISION                              :
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

            if(my_abs(Yaw/100)>90)
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
    case RIGHT_LEAKING_SWEEP_YAW_MORE_ABS90                         :
        right_ready_leaking_sweep_status = RIGHT_LEAKING_SWEEP_CLOCK_TARGET_YAW_LESS_ABS90;
        break;
    case RIGHT_LEAKING_SWEEP_CLOCK_TARGET_YAW_LESS_ABS90            :
        if(my_abs(Yaw/100)<90){
            linear_velocity=0;
            angular_velocity=0;
            last_position_y=current_pose->y;
            right_ready_leaking_sweep_status = RIGHT_LEAKING_SWEEP_GOSTRAIGHT_MORE;
            break;
        }
        if (obstacleSignal == front_obstacle || obstacleSignal == left_obstacle || obstacleSignal == right_obstacle)
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
    case RIGHT_LEAKING_SWEEP_GOSTRAIGHT_MORE                        :
        linear_velocity=long_stra_vel;
        angular_velocity=0;
        if(obstacleSignal==front_obstacle||obstacleSignal==left_obstacle||obstacleSignal==right_obstacle){
            right_ready_leaking_sweep_status = RIGHT_LEAKING_SWEEP_CLOCK_TARGER_YAW_LESS_ABS3;
            break;
        }
        if(my_abs(last_position_y-current_pose->y)>leakingsweep){
            right_ready_leaking_sweep_status = RIGHT_LEAKING_SWEEP_CLOCK_TARGER_YAW_LESS_ABS3;
            break;
        }
        break;
    case RIGHT_LEAKING_SWEEP_CLOCK_TARGER_YAW_LESS_ABS3             :
        if(my_abs(Yaw/100)<5){
            linear_velocity=0;
            angular_velocity=0;
            right_ready_leaking_sweep_status = RIGHT_LEAKING_SWEEP_COMPLETE;
            break;
        }
        if (obstacleSignal == front_obstacle || obstacleSignal == left_obstacle || obstacleSignal == right_obstacle)
        {
            linear_velocity=0;
            angular_velocity=0;
            right_ready_leaking_sweep_status = RIGHT_LEAKING_SWEEP_CLOCK_TARGER_YAW_LESS_ABS3_COLLISION;
        }
        linear_velocity=0;
        angular_velocity=-turn_vel;
        break;
    case RIGHT_LEAKING_SWEEP_CLOCK_TARGER_YAW_LESS_ABS3_COLLISION   :
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
    case RIGHT_LEAKING_SWEEP_YAW_OTHER                              :
        right_ready_leaking_sweep_status = RIGHT_LEAKING_SWEEP_CCLOCK_TARGET_YAW_MORE_ABS90;
        break;
    case RIGHT_LEAKING_SWEEP_CCLOCK_TARGET_YAW_MORE_ABS90           :
        if(my_abs(Yaw/100)>90){
            linear_velocity=0;
            angular_velocity=0;
            last_position_y=current_pose->y;
            right_ready_leaking_sweep_status = RIGHT_LEAKING_SWEEP_GOSTRAIGHT_OTHER;
            break;
        }
        if (obstacleSignal == front_obstacle || obstacleSignal == left_obstacle || obstacleSignal == right_obstacle)
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
        break;
    case RIGHT_LEAKING_SWEEP_GOSTRAIGHT_OTHER                       :
        linear_velocity=long_stra_vel;
        angular_velocity=0;
        if(obstacleSignal==front_obstacle||obstacleSignal==left_obstacle||obstacleSignal==right_obstacle){
            right_ready_leaking_sweep_status = RIGHT_LEAKING_SWEEP_CCLOCK_TARGET_YAW_MORE_ABS178;
            break;
        }
        if(my_abs(last_position_y-current_pose->y)>leakingsweep){
            right_ready_leaking_sweep_status = RIGHT_LEAKING_SWEEP_CCLOCK_TARGET_YAW_MORE_ABS178;
            break;
        }
        break;
    case RIGHT_LEAKING_SWEEP_CCLOCK_TARGET_YAW_MORE_ABS178           :
        if(my_abs(Yaw/100)>175){
            linear_velocity=0;
            angular_velocity=0;
            right_ready_leaking_sweep_status = RIGHT_LEAKING_SWEEP_COMPLETE;
            break;
        }
        if (obstacleSignal == front_obstacle || obstacleSignal == left_obstacle || obstacleSignal == right_obstacle)
        {
            linear_velocity=0;
            angular_velocity=0;
            right_ready_leaking_sweep_status = RIGHT_LEAKING_SWEEP_CCLOCK_TARGET_YAW_MOEE_ABS178_COLLISIION;
        }
        linear_velocity=0;
        angular_velocity=turn_vel;
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
    case RIGHT_LEAKING_SWEEP_COMPLETE                               :
        right_ready_leaking_sweep_status = 0;
        complete_flag = 1;
        break;
    }
    return complete_flag;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////
unsigned char RightReturnOriginWorkStep(POSE *current_pose,unsigned char obstacleSignal)
{
	return 1;
}
	
	
//#################################################################################
//##########           LEFT             ###########################################	
//#################################################################################	
////////////////////////////////////////////////////////////////////////////////////////////////////////	
unsigned char LeftRunningWorkStep(POSE *current_pose,unsigned char obstacleSignal)
{
	unsigned char complete_flag = 0;    
    Yaw = current_pose->orientation;

    switch(left_running_step_status)
    {
        case 0:
            left_running_step_status = GOSTR_LEFTRUN_STEP;
            break;
        case GOSTR_LEFTRUN_STEP:
            log_debug("gostraight right run step!");
            if (my_abs(Yaw / 100) > 90 && my_abs(Yaw / 100) < 175)
            {
                log_debug("backaward Corrected heading angle !");
                if (Yaw > 0)
                {
                    linear_velocity = 0;
                    angular_velocity = 5*turn_vel / 6;
                    break;
                }
                else
                {
                    linear_velocity = 0;
                    angular_velocity = -5*turn_vel / 6;
                    break;
                }
            }
            else if (my_abs(Yaw / 100) < 90 && my_abs(Yaw / 100) > 5)
            {
                log_debug("gostraight Corrected heading angle !");
                if (Yaw > 0)
                {
                    linear_velocity = 0;
                    angular_velocity = -5*turn_vel / 6;
                    break;
                }
                else
                {
                    linear_velocity = 0;
                    angular_velocity = 5*turn_vel / 6;
                    break;
                }
            }
            else if(my_abs(current_pose->x) > W)
            {
                log_debug("current pose x arrived width max!");
                left_running_step_status = FORWARD_BOUNDARY_LEFTRUN_STEP;
                break;
            }
            else if(my_abs(current_pose->y) > W)
            {
                log_debug("current pose y arrived width max!");
                complete_flag = 1;
                break;
            }
            else if (right_obstacle == obstacleSignal)
            {
                linear_velocity = 0;
                angular_velocity = 0;
                //leakingsweep = gridmap.Left_Return_Extreme_point(Yaw / 100, obstacleSignal);
				//leakingsweep = bsp_Left_ReturnExtreme_point(current_pose->x,current_pose->y,current_pose->orientation,obstacleSignal);
				if(my_abs(leakingsweep_x-current_pose->x)>leakingsweep_X_interval&&my_abs(leakingsweep_y-current_pose->y)>leakingsweep_Y_interval){
					
					//sendvelocity(&linear_velocity,&angular_velocity);
					leakingsweep_x=current_pose->x;
					leakingsweep_y=current_pose->y;
					leakingsweep =bsp_Left_ReturnExtreme_point(current_pose->x,current_pose->y,current_pose->orientation,obstacleSignal);
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
                    collision_right_leftrun_step_status = 0;
                    left_running_step_status = COLLISION_RIGHT_LEFTRUN_STEP;
                    break;
                }
            }
            else if (front_obstacle == obstacleSignal)
            {
                linear_velocity = 0;
                angular_velocity = 0;
                //leakingsweep = gridmap.Left_Return_Extreme_point(Yaw / 100, obstacleSignal);
                //leakingsweep = bsp_Left_ReturnExtreme_point(current_pose->x,current_pose->y,current_pose->orientation,obstacleSignal);
				if(my_abs(leakingsweep_x-current_pose->x)>leakingsweep_X_interval&&my_abs(leakingsweep_y-current_pose->y)>leakingsweep_Y_interval){
					
					//sendvelocity(&linear_velocity,&angular_velocity);
					leakingsweep_x=current_pose->x;
					leakingsweep_y=current_pose->y;
					leakingsweep =bsp_Left_ReturnExtreme_point(current_pose->x,current_pose->y,current_pose->orientation,obstacleSignal);
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
                    collision_front_leftrun_step_status = 0;
                    left_running_step_status = COLLISION_FRONT_LEFTRUN_STEP;
                    break;
                }
            }
            else if (left_obstacle == obstacleSignal)
            {
                linear_velocity = 0;
                angular_velocity = 0;
                //leakingsweep = gridmap.Left_Return_Extreme_point(Yaw / 100, obstacleSignal);
                //leakingsweep = bsp_Left_ReturnExtreme_point(current_pose->x,current_pose->y,current_pose->orientation,obstacleSignal);
				if(my_abs(leakingsweep_x-current_pose->x)>leakingsweep_X_interval&&my_abs(leakingsweep_y-current_pose->y)>leakingsweep_Y_interval){
					
					//sendvelocity(&linear_velocity,&angular_velocity);
					leakingsweep_x=current_pose->x;
					leakingsweep_y=current_pose->y;
					leakingsweep =bsp_Left_ReturnExtreme_point(current_pose->x,current_pose->y,current_pose->orientation,obstacleSignal);
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
                    collision_left_leftrun_step_status = 0;
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
            if( ForwardBoundaryLeftRunStep(current_pose,obstacleSignal)){
                left_running_step_status = GOSTR_LEFTRUN_STEP;
                left_forward_boundary_status=0;
            }
            break;

        case COLLISION_RIGHT_LEFTRUN_STEP:
            log_debug("collsion right right run step!");
            FunctionStatus=CollisionRightLeftRunStep(current_pose,obstacleSignal);
            if(1==FunctionStatus){
                left_running_step_status = GOSTR_LEFTRUN_STEP;
                collision_right_leftrun_step_status = 0;
                FunctionStatus=0;
                break;
            }
            if(2==FunctionStatus){
                left_running_step_status =LEFTREVERSEWALKEDGE;
                collision_right_leftrun_step_status = 0;
                FunctionStatus=0;
                break;
            }
            break;
        case LEFTREVERSEWALKEDGE:
            log_debug("LEFTREVERSEWALKEDGE");
            FunctionStatus=LeftReverseWalkEdge(current_pose,obstacleSignal);
            if(1==FunctionStatus){
                left_running_step_status = GOSTR_LEFTRUN_STEP;
                left_reverse_walk_edge_status = 0;
                FunctionStatus=0;
                break;
            }
            if(2==FunctionStatus){
                left_running_step_status =LEFTEDGEDILEMMA;
                left_reverse_walk_edge_status = 0;
                FunctionStatus=0;
                break;
            }
			if(3==FunctionStatus){
				left_running_step_status =0;
				left_reverse_walk_edge_status = 0;
				FunctionStatus=0;
				complete_flag = 1;
				break;
			}
            break;
        case LEFTEDGEDILEMMA:
            log_debug("LEFTEDGEDILEMMA");
            if(LeftEdgeDilemma(current_pose,obstacleSignal))
            {
                left_running_step_status = GOSTR_LEFTRUN_STEP;
                left_edge_dilemma_status=0;
            }
            break;
        case COLLISION_LEFT_LEFTRUN_STEP:
            log_debug("collsion left left run step!");
            FunctionStatus=CollisionLeftLeftRunStep(current_pose,obstacleSignal);
            if(1==FunctionStatus){
                left_running_step_status = GOSTR_LEFTRUN_STEP;
                collision_left_leftrun_step_status= 0;
                FunctionStatus=0;
                break;
            }
            if(2==FunctionStatus){
                left_running_step_status =LEFTWALKEDGE;
                collision_left_leftrun_step_status = 0;
                FunctionStatus=0;
                break;
            }
            break;
        case LEFTWALKEDGE:
            log_debug("LEFTWALKEDGE");
            FunctionStatus=LeftWalkEdge(current_pose,obstacleSignal);
            if(1==FunctionStatus){
                left_running_step_status = GOSTR_LEFTRUN_STEP;
                left_walk_edge_status = 0;
                FunctionStatus=0;
                break;
            }
            if(2==FunctionStatus){
                left_running_step_status =LEFTEDGEDILEMMA;
                left_walk_edge_status = 0;
                FunctionStatus=0;
                break;
            }
			if(3==FunctionStatus){
				left_running_step_status =0;
				left_walk_edge_status = 0;
				FunctionStatus=0;
				complete_flag = 1;
				break;
			}
            break;
        case COLLISION_FRONT_LEFTRUN_STEP:
            log_debug("collsion front left run step!");
            FunctionStatus=CollisionFrontLeftRunStep(current_pose,obstacleSignal);
            if(1==FunctionStatus){
                left_running_step_status = GOSTR_LEFTRUN_STEP;
                collision_front_leftrun_step_status=0;
                FunctionStatus=0;
                break;
            }
            if(2==FunctionStatus){
                left_running_step_status =LEFTWALKEDGE;
                collision_front_leftrun_step_status=0;
                FunctionStatus=0;
                break;
            }
            if(3==FunctionStatus){
            left_running_step_status =LEFTREVERSEWALKEDGE;
                collision_front_leftrun_step_status=0;
                FunctionStatus=0;
                break;
            }
            break;
        case LEAKING_SWEEP_LEFTRUN_STEP:
            log_debug("leaking sweep left run step!");
            if(LeftReadyLeakingSweep(current_pose,obstacleSignal))
            {
                left_running_step_status = GOSTR_LEFTRUN_STEP;
            }
            break;
        default:
            break;
    }
	
    sendvelocity(&linear_velocity,&angular_velocity);
	return complete_flag;
}

unsigned char ForwardBoundaryLeftRunStep(POSE *current_pose,unsigned char obstacleSignal)
{
    unsigned char complete_flag = 0;
    Yaw = current_pose->orientation;
	switch(left_forward_boundary_status)
	{
		case 0:
			if (my_abs(Yaw / 100) < 90)
			{
				left_forward_boundary_status = LEFT_FORWARDBOUNDARY_YAW_LESS_ABS10;
			}
			else
			{
				left_forward_boundary_status = LEFT_FORWARDBOUNDARY_YAW_OTHER;
			}
			break;
		case LEFT_FORWARDBOUNDARY_YAW_LESS_ABS10:
			left_forward_boundary_status = LEFT_FORWARDBOUNDARY_CCLOCK_TARGET_YAW_MORE_ABS178;
			break;
		case LEFT_FORWARDBOUNDARY_CCLOCK_TARGET_YAW_MORE_ABS178:
			linear_velocity = 100;
			angular_velocity = 57;
			if (my_abs(Yaw / 100) > 175)
			{
				left_forward_boundary_status = LEFT_FORWARDBOUNDARY_GOSTRAIGHT;
				break;
			}
			if (obstacleSignal == front_obstacle || obstacleSignal == left_obstacle || obstacleSignal == right_obstacle)
			{
				left_forward_boundary_status = LEFT_FORWARDBOUNDARY_CCLOCK_TARGET_YAW_MORE_ABS178_COLLISION;
				break;
			}
			break;
		case LEFT_FORWARDBOUNDARY_CCLOCK_TARGET_YAW_MORE_ABS178_COLLISION:
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
				left_forward_boundary_status = LEFT_FORWARDBOUNDARY_CCLOCK_TARGET_YAW_MORE_ABS178;
				break;
			}
			break;
		case LEFT_FORWARDBOUNDARY_YAW_OTHER:
			left_forward_boundary_status = LEFT_FORWARDBOUNDARY_CLOCK_TARGET_YAW_LESS_ABS3;
			break;
		case LEFT_FORWARDBOUNDARY_CLOCK_TARGET_YAW_LESS_ABS3:
			linear_velocity = 100;
			angular_velocity = -57;
			if (my_abs(Yaw / 100) < 5)
			{
				left_forward_boundary_status = LEFT_FORWARDBOUNDARY_GOSTRAIGHT;
				break;
			}
			if (obstacleSignal == front_obstacle || obstacleSignal == left_obstacle || obstacleSignal == right_obstacle)
			{
				left_forward_boundary_status = LEFT_FORWARDBOUNDARY_CLOCK_TARGET_YAW_LESS_ABS3_COLLISION;
				break;
			}
			break;
		case LEFT_FORWARDBOUNDARY_CLOCK_TARGET_YAW_LESS_ABS3_COLLISION:
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
				left_forward_boundary_status = LEFT_FORWARDBOUNDARY_CLOCK_TARGET_YAW_LESS_ABS3;
				break;
			}
			break;
		case LEFT_FORWARDBOUNDARY_GOSTRAIGHT:
			if(turn_start_update == 0)
			{
				turn_start_x = current_pose->x;
				turn_start_y = current_pose->y;
				turn_start_update = 1;
			}
			linear_velocity = long_stra_vel;
			angular_velocity = 0;
			if ((my_abs(turn_start_x - current_pose->x) > 50 || my_abs(turn_start_y - current_pose->y) >50)&&my_abs(Yaw / 100) < 90&&current_pose->x>0)
			{
				linear_velocity = 0;
				angular_velocity = 0;
				turn_start_update = 0;
				left_forward_boundary_status = LEFT_FORWARDBOUNDARY_COMPLETE;
				break;
			}

			if ((my_abs(turn_start_x - current_pose->x) > 50 || my_abs(turn_start_y - current_pose->y) >50)&&my_abs(Yaw / 100) > 90&&current_pose->x<0)
			{
				linear_velocity = 0;
				angular_velocity = 0;
				turn_start_update = 0;
				left_forward_boundary_status = LEFT_FORWARDBOUNDARY_COMPLETE;
				break;
			}
			linear_velocity = long_stra_vel;
			angular_velocity = 0;
			if (obstacleSignal == front_obstacle || obstacleSignal == left_obstacle || obstacleSignal == right_obstacle)
			{
				left_forward_boundary_status = LEFT_FORWARDBOUNDARY_COMPLETE;
				break;
			}
			if (my_abs(current_pose->x)<W)
			{
				linear_velocity = 0;
				angular_velocity = 0;
				left_forward_boundary_status = FORWARDBOUNDARY_COMPLETE;
				break;
			}
			break;
		case LEFT_FORWARDBOUNDARY_COMPLETE:
			left_forward_boundary_status = 0;
			complete_flag = 1;
			maintain_bow_distance = current_pose->y;
			break;
	}
	return complete_flag;
}

unsigned char CollisionRightLeftRunStep(POSE *current_pose,unsigned char obstacleSignal)
{
    unsigned char complete_flag = 0;
    Yaw =current_pose->orientation;
    switch(collision_right_leftrun_step_status)
    {
        case 0:
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
                collision_right_leftrun_step_status = GOBACK_DISTANCE_CRLRS;
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

            if (obstacleSignal == front_obstacle || obstacleSignal == left_obstacle || obstacleSignal == right_obstacle)
            {
                linear_velocity = 0;
                angular_velocity = 0;
                if (my_abs(Yaw / 100) > 90)
                {
                    collision_right_leftrun_step_status  = DIR_LEFT_YAW_MORE_ABS90_CRLRS;
                }
                else
                {
                    collision_right_leftrun_step_status = DIR_LEFT_YAW_LESS_ABS90_CRLRS;
                }
                turn_start_update = 0;
                break;
            }
            if (my_abs(turn_start_x - current_pose->x) > side_backward_distance||my_abs(turn_start_y - current_pose->y) > side_backward_distance)
            {
                linear_velocity = 0;
                angular_velocity = 0;
                if (my_abs(Yaw / 100) > 90)
                {
                    collision_right_leftrun_step_status = DIR_LEFT_YAW_MORE_ABS90_CRLRS;
                }
                else
                {
                    collision_right_leftrun_step_status = DIR_LEFT_YAW_LESS_ABS90_CRLRS;
                }
                turn_start_update = 0;
                break;
            }
            break;
        case DIR_LEFT_YAW_MORE_ABS90_CRLRS:
            linear_velocity = 0;
            angular_velocity = 0;
            last_position_y = current_pose->y;
            collision_right_leftrun_step_status = TURN_CLOCK_TARGET_YAW_ABS153_LRUN_CR_DLYM;
            break;
        case TURN_CLOCK_TARGET_YAW_ABS153_LRUN_CR_DLYM:
            linear_velocity = 0;
            angular_velocity = -turn_vel;
            if (my_abs(Yaw / 100) < 153)
            {
                linear_velocity = 0;
                angular_velocity = 0;
                collision_right_leftrun_step_status = GOSTR_YAW_EQUAL_ABS153_LRUN_CR_DLYM;
                break;
            }
            if (obstacleSignal == front_obstacle || obstacleSignal == left_obstacle || obstacleSignal == right_obstacle)
            {
                linear_velocity = 0;
                angular_velocity = 0;
                collision_right_leftrun_step_status = TURN_CLOCK_TARGET_YAW_ABS153_COLLISION_LRUN_CR_DLYM;
                break;
            }
            break;
        case TURN_CLOCK_TARGET_YAW_ABS153_COLLISION_LRUN_CR_DLYM:
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
                collision_right_leftrun_step_status = TURN_CLOCK_TARGET_YAW_ABS153_LRUN_CR_DLYM;
                break;
            }
            break;
        case GOSTR_YAW_EQUAL_ABS153_LRUN_CR_DLYM:
            linear_velocity = long_stra_vel;
            angular_velocity = 0;
            if (obstacleSignal == front_obstacle || obstacleSignal == left_obstacle || obstacleSignal == right_obstacle)
            {
                collision_right_leftrun_step_status  = TURN_CLOCK_TARGET_YAW_POS123_LRUN_CR_DLYM;
                break;
            }
            if (my_abs(last_position_y - current_pose->y) > lateral_move_distance / 3)
            {
                collision_right_leftrun_step_status  = TURN_CLOCK_TARGET_YAW_POS123_LRUN_CR_DLYM;
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
                collision_right_leftrun_step_status  = GOSTR_YAW_EQUAL_POS123_LRUN_CR_DLYM;
                break;
            }
            if (obstacleSignal == front_obstacle || obstacleSignal == left_obstacle || obstacleSignal == right_obstacle)
            {
                collision_right_leftrun_step_status  = TURN_CLOCK_TARGET_YAW_POS123_COLLISION_LRUN_CR_DLYM;
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
                collision_right_leftrun_step_status = TURN_CLOCK_TARGET_YAW_POS123_LRUN_CR_DLYM;
                break;
            }
            break;
        case GOSTR_YAW_EQUAL_POS123_LRUN_CR_DLYM:
            linear_velocity = long_stra_vel;
            angular_velocity = 0;
            if (obstacleSignal == front_obstacle || obstacleSignal == left_obstacle || obstacleSignal == right_obstacle)
            {
                linear_velocity = 0;
                angular_velocity = 0;
                collision_right_leftrun_step_status = TURN_CLOCK_TARGET_YAW_POS98_LRUN_CR_DLYM;
                break;
            }
            if (my_abs(last_position_y - current_pose->y) > (2 * lateral_move_distance) / 3)
            {
                linear_velocity = 0;
                angular_velocity = 0;
                collision_right_leftrun_step_status = TURN_CLOCK_TARGET_YAW_POS98_LRUN_CR_DLYM;
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
                collision_right_leftrun_step_status = GOSTR_YAW_EQUAL_POS98_LRUN_CR_DLYM;
                break;
            }
            if (obstacleSignal == front_obstacle || obstacleSignal == left_obstacle || obstacleSignal == right_obstacle)
            {
                linear_velocity = 0;
                angular_velocity = 0;
                collision_right_leftrun_step_status = TURN_CLOCK_TARGET_YAW_POS98_COLLISION_LRUN_CR_DLYM;
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
                collision_right_leftrun_step_status = TURN_CLOCK_TARGET_YAW_POS98_LRUN_CR_DLYM;
                break;
            }
            break;
        case GOSTR_YAW_EQUAL_POS98_LRUN_CR_DLYM:
            linear_velocity = long_stra_vel;
            angular_velocity = 0;
            if (obstacleSignal == front_obstacle || obstacleSignal == left_obstacle || obstacleSignal == right_obstacle)
            {
                linear_velocity = 0;
                angular_velocity = 0;
                collision_right_leftrun_step_status = LEFT_REVERSE_WALK_EDGE_LRUN_CR_DLYM;
                break;
            }
            if (my_abs(last_position_y - current_pose->y) > lateral_move_distance)
            {
                linear_velocity = 0;
                angular_velocity = 0;
                collision_right_leftrun_step_status = TURN_CLOCK_TARGET_YAW_ABS3_LRUN_CR_DLYM;
                break;
            }
            break;
        case LEFT_REVERSE_WALK_EDGE_LRUN_CR_DLYM:
			collision_right_leftrun_step_status = 0;
			complete_flag = 2;
			break;
        case TURN_CLOCK_TARGET_YAW_ABS3_LRUN_CR_DLYM:
            linear_velocity = 0;
            angular_velocity = -turn_vel;
            if (my_abs(Yaw / 100) < 3)
            {
                linear_velocity = 0;
                angular_velocity = 0;
                collision_right_leftrun_step_status = COMPLETE_LRUN_CR_DLYM;
                break;
            }
            if (obstacleSignal == front_obstacle || obstacleSignal == left_obstacle || obstacleSignal == right_obstacle)
            {
                linear_velocity = 0;
                angular_velocity = 0;
                collision_right_leftrun_step_status = TURN_CLOCK_TARGET_YAW_ABS3_COLLISION_LRUN_CR_DLYM;
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
                collision_right_leftrun_step_status = TURN_CLOCK_TARGET_YAW_ABS3_LRUN_CR_DLYM;
                break;
            }
            break;
        case COMPLETE_LRUN_CR_DLYM:
            maintain_bow_distance = current_pose->y;
            collision_right_leftrun_step_status = 0;
            complete_flag = 1;
            break;

        case DIR_LEFT_YAW_LESS_ABS90_CRLRS:
            linear_velocity = 0;
            angular_velocity = 0;
            collision_right_leftrun_step_status = TURN_CCLCOK_TARGET_YAW_ABS27_LRUN_CR_DLYL;
            break;
        case TURN_CCLCOK_TARGET_YAW_ABS27_LRUN_CR_DLYL:
            linear_velocity = 0;
            angular_velocity = turn_vel;
            if (my_abs(Yaw / 100) > 27)
            {
                linear_velocity = 0;
                angular_velocity = 0;
                collision_right_leftrun_step_status = GOSTR_YAW_EQUAL_ABS27_LRUN_CR_DLYL;
                break;
            }
            if (obstacleSignal == front_obstacle || obstacleSignal == left_obstacle || obstacleSignal == right_obstacle)
            {
                linear_velocity = 0;
                angular_velocity = 0;
                collision_right_leftrun_step_status = TURN_CCLCOK_TARGET_YAW_ABS27_COLLISION_LRUN_CR_DLYL;
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
                collision_right_leftrun_step_status = TURN_CCLCOK_TARGET_YAW_ABS27_LRUN_CR_DLYL;
                break;
            }
            break;
        case GOSTR_YAW_EQUAL_ABS27_LRUN_CR_DLYL:
            cnt_update = 0;
            last_position_y = current_pose->y;
            last_position_xx = current_pose->x;
            collision_right_leftrun_step_status = GOSTR_BYPASS_LRUN_CR_DLYL;
            break;
        case GOSTR_BYPASS_LRUN_CR_DLYL:
            last_position_x = current_pose->x;
            cnt_update +=1;
            linear_velocity = bypass_velocity;
            angular_velocity = 0;
            if(cnt_update > 4&&my_abs(last_position_xx - current_pose->x)<20&&my_abs(last_position_y - current_pose->y)<20)
            {
                cnt_update = 0;
                collision_right_leftrun_step_status = MORE_TRY_BREAK_BYPASS_LRUN_CR_DLYL;
                break;
            }
            collision_right_leftrun_step_status = GOSTR_BYPASS_LOOP_LRUN_CR_DLYL;
            break;
        case GOSTR_BYPASS_LOOP_LRUN_CR_DLYL:
            if (obstacleSignal == right_obstacle)
            {
                collision_right_leftrun_step_status = RIGHT_COLLISION_BYPASS_LRUN_CR_DLYL;
                break;
            }
            if (my_abs(last_position_x - current_pose->x) > lateral_move_distance / 3)
            {
                collision_right_leftrun_step_status = GOSTR_X_MORE_ONE_THIRD_LATERALDIS_BYPASS_LRUN_CR_DLYL;
                break;
            }
            if ((current_pose->y - last_position_y) > close_edge || obstacleSignal == front_obstacle || obstacleSignal == left_obstacle)
            {
                collision_right_leftrun_step_status = GOSTR_BYPASS_BOW_CONTINUE_LRUN_CR_DLYL;
                break;
            }
            if (last_position_y > current_pose->y + 1)
            {
                linear_velocity = 0;
                angular_velocity = 0;
                old_bow_continue = true;
                collision_right_leftrun_step_status = GOSTR_BYPASS_BOW_CONTINUE_TARGET_YAW_ABS3_LRUN_CR_DLYL;
                break;
            }
            if (my_abs(current_pose->x) > W)
            {
                collision_right_leftrun_step_status = COMPLETE_LRUN_CR_DLYL;
                break;
            }
            if (my_abs(Yaw / 100) >90)
            {
                collision_right_leftrun_step_status = TURN_CCLOCK_TARGET_YAW_MORE_AB173_LRUN_CR_DLYL;
                break;
            }
            break;
        case GOSTR_BYPASS_BOW_CONTINUE_TARGET_YAW_ABS3_LRUN_CR_DLYL:
            linear_velocity = 0;
            angular_velocity = turn_vel;
            if (obstacleSignal == front_obstacle || obstacleSignal == right_obstacle || obstacleSignal == left_obstacle)
            {
                linear_velocity = 0;
                angular_velocity = 0;
                collision_right_leftrun_step_status =  GOSTR_BYPASS_BOW_CONTINUE_TARGET_YAW_ABS3_COLLISION_LRUN_CR_DLYL;
                break;
            }
            if (my_abs(Yaw / 100) < 5)
            {
                linear_velocity = 0;
                angular_velocity = 0;
                collision_right_leftrun_step_status =  GOSTR_BYPASS_BOW_CONTINUE_EXIT_LRUN_CR_DLYL;
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
                collision_right_leftrun_step_status = GOSTR_BYPASS_BOW_CONTINUE_TARGET_YAW_ABS3_LRUN_CR_DLYL;
                break;
            }
            break;

        case GOSTR_BYPASS_BOW_CONTINUE_LRUN_CR_DLYL:
            if (obstacleSignal == front_obstacle || obstacleSignal == right_obstacle || obstacleSignal == left_obstacle)
            {
                collision_right_leftrun_step_status = GOSTR_BYPASS_BOW_CONTINUE_COLLISION_LRUN_CR_DLYL;
                break;
            }
            if (my_abs(Yaw / 100) > 175)
            {
                linear_velocity = 0;
                angular_velocity = 0;
                bow_continue = true;
                collision_right_leftrun_step_status = GOSTR_BYPASS_BOW_CONTINUE_EXIT_LRUN_CR_DLYL;
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
                collision_right_leftrun_step_status = GOSTR_BYPASS_BOW_CONTINUE_LRUN_CR_DLYL;
                turn_start_update = 0;
                break;
            }
            break;
        case GOSTR_BYPASS_BOW_CONTINUE_EXIT_LRUN_CR_DLYL:
            maintain_bow_distance = current_pose->y;
            collision_right_leftrun_step_status = COMPLETE_LRUN_CR_DLYL;
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
                temporary_yaw = Yaw / 100;
                if (my_abs(Yaw / 100) < 15) collision_right_leftrun_step_status = TURN_CCLOCK_TARGET_YAW_LESS_ABS15_LRUN_CR_DLYL;
                else collision_right_leftrun_step_status = TURN_CCLOCK_TARGET_YAW_MORE_ABS15_LRUN_CR_DLYL;
                break;
            }
            break;
        case TURN_CCLOCK_TARGET_YAW_LESS_ABS15_LRUN_CR_DLYL:
            if (obstacleSignal == front_obstacle || obstacleSignal == right_obstacle || obstacleSignal == left_obstacle)
            {
                linear_velocity = 0;
                angular_velocity = 0;
                collision_right_leftrun_step_status = TURN_CCLOCK_TARGET_YAW_LESS_ABS15_COLLISION_LRUN_CR_DLYL;
                break;
            }
            if (my_abs(Yaw / 100) > 15)
            {
                linear_velocity = 0;
                angular_velocity = 0;
                collision_right_leftrun_step_status = GOSTR_BYPASS_LRUN_CR_DLYL;
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
				collision_right_leftrun_step_status = TURN_CCLOCK_TARGET_YAW_LESS_ABS15_LRUN_CR_DLYL;
				break;
			}
			break;

        case TURN_CCLOCK_TARGET_YAW_MORE_ABS15_LRUN_CR_DLYL:
            if (obstacleSignal == front_obstacle || obstacleSignal == right_obstacle || obstacleSignal == left_obstacle)
            {
                collision_right_leftrun_step_status = TURN_CCLOCK_TARGET_YAW_MORE_ABS15_COLLISION_LRUN_CR_DLYL;
                break;
            }
            if (my_abs(Yaw / 100 - temporary_yaw) > 15)
            {
                collision_right_leftrun_step_status = GOSTR_BYPASS_LRUN_CR_DLYL;
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
				collision_right_leftrun_step_status = TURN_CCLOCK_TARGET_YAW_MORE_ABS15_LRUN_CR_DLYL;
				break;
			}
			break;

        case GOSTR_X_MORE_ONE_THIRD_LATERALDIS_BYPASS_LRUN_CR_DLYL:
            if (my_abs(Yaw / 100) <=30)
            {
                linear_velocity = 0;
                angular_velocity = 0;
                temporary_yaw = Yaw / 100;
                collision_right_leftrun_step_status = TURN_CLOCK_TARGET_YAW_ABS30_LRUN_CR_DLYL;
                break;
            }
            else if (my_abs(Yaw / 100) > 30 && Yaw / 100 > 0)
            {
                linear_velocity = 0;
                angular_velocity = 0;
                temporary_yaw = Yaw / 100;
                collision_right_leftrun_step_status = TURN_CLOCK_YAW_ADD_ABS30_LRUN_CR_DLYL;
                break;
            }
            else{
                collision_right_leftrun_step_status = GOSTR_BYPASS_LRUN_CR_DLYL;
                break;
            }
        case TURN_CLOCK_TARGET_YAW_ABS30_LRUN_CR_DLYL:
            if (my_abs(Yaw / 100) > 30)
            {
                linear_velocity = 0;
                angular_velocity = 0;
                collision_right_leftrun_step_status = GOSTR_BYPASS_LRUN_CR_DLYL;
                break;
            }
            if (obstacleSignal == front_obstacle || obstacleSignal == left_obstacle || obstacleSignal == right_obstacle)
            {
                collision_right_leftrun_step_status = TURN_CLOCK_TARGET_YAW_ABS30_COLLISION_LRUN_CR_DLYL;
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
                collision_right_leftrun_step_status = TURN_CLOCK_TARGET_YAW_ABS30_LRUN_CR_DLYL;
                turn_start_update = 0;
                break;
            }
            break;
        case TURN_CLOCK_YAW_ADD_ABS30_LRUN_CR_DLYL:
            if (my_abs(Yaw / 100 - temporary_yaw) > 30)
            {
                linear_velocity = 0;
                angular_velocity = 0;
                collision_right_leftrun_step_status = GOSTR_BYPASS_LRUN_CR_DLYL;
                break;
            }

            if (obstacleSignal == front_obstacle || obstacleSignal == left_obstacle || obstacleSignal == right_obstacle)
            {
                collision_right_leftrun_step_status = TURN_CLOCK_YAW_ADD_ABS30_COLLISION_LRUN_CR_DLYL;
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
                collision_right_leftrun_step_status = TURN_CLOCK_YAW_ADD_ABS30_LRUN_CR_DLYL;
                turn_start_update = 0;
                break;
            }
            break;

        case TURN_CCLOCK_TARGET_YAW_MORE_AB173_LRUN_CR_DLYL:
            if (obstacleSignal == front_obstacle || obstacleSignal == right_obstacle || obstacleSignal == left_obstacle)
            {
                collision_right_leftrun_step_status = TURN_CCLOCK_TARGET_YAW_MORE_AB173_COLLISION_LRUN_CR_DLYL;
                break;
            }
            if (my_abs(Yaw / 100) > 173)
            {
                linear_velocity = 0;
                angular_velocity = 0;
                collision_right_leftrun_step_status = COMPLETE_LRUN_CR_DLYL;
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
                collision_right_leftrun_step_status = COMPLETE_LRUN_CR_DLYL;
                turn_start_update = 0;
                break;
            }
            break;
        case MORE_TRY_BREAK_BYPASS_LRUN_CR_DLYL:
            if (obstacleSignal == front_obstacle || obstacleSignal == right_obstacle || obstacleSignal == left_obstacle)
            {
                collision_right_leftrun_step_status = MORE_TRY_BREAK_BYPASS_COLLISION_LRUN_CR_DLYL;
                bow_continue = true;
                break;
            }
            if (my_abs(Yaw / 100) > 173)
            {
                linear_velocity = 0;
                angular_velocity = 0;
                bow_continue = true;
                collision_right_leftrun_step_status = GOSTR_BYPASS_BOW_CONTINUE_EXIT_LRUN_CR_DLYL;
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
				collision_right_leftrun_step_status = MORE_TRY_BREAK_BYPASS_LRUN_CR_DLYL;
				turn_start_update = 0;
				break;
			}
			break;
        case COMPLETE_LRUN_CR_DLYL:
            collision_right_leftrun_step_status = 0;
            complete_flag = 1;
            break;

    }
    return complete_flag;
}

unsigned char CollisionLeftLeftRunStep(POSE *current_pose,unsigned char obstacleSignal)
{
    unsigned char complete_flag = 0;
    Yaw = current_pose->orientation;
    switch(collision_left_leftrun_step_status)
    {
        case 0:
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
                collision_left_leftrun_step_status = GOBACK_DISTANCE_CLLRS;
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

            if (obstacleSignal == front_obstacle || obstacleSignal == left_obstacle || obstacleSignal == right_obstacle)
            {
                linear_velocity = 0;
                angular_velocity = 0;
                if (my_abs(Yaw / 100) > 90)
                {
                    collision_left_leftrun_step_status  = DIR_LEFT_YAW_MORE_ABS90_CLLRS;
                }
                else
                {
                    collision_left_leftrun_step_status = DIR_LEFT_YAW_LESS_ABS90_CLLRS;
                }
                turn_start_update = 0;
                break;
            }
            if (my_abs(turn_start_x - current_pose->x) > side_backward_distance||my_abs(turn_start_y - current_pose->y) > side_backward_distance)
            {
                linear_velocity = 0;
                angular_velocity = 0;
                if (my_abs(Yaw / 100) > 90)
                {
                    collision_left_leftrun_step_status = DIR_LEFT_YAW_MORE_ABS90_CLLRS;
                }
                else
                {
                    collision_left_leftrun_step_status = DIR_LEFT_YAW_LESS_ABS90_CLLRS;
                }
                turn_start_update = 0;
                break;
            }
            break;
        case  DIR_LEFT_YAW_MORE_ABS90_CLLRS:
            linear_velocity = 0;
            angular_velocity = 0;
            collision_left_leftrun_step_status = TURN_CLOCK_TARGET_YAW_ABS150_LRUN_CL_DLYM;
            break;
        case  TURN_CLOCK_TARGET_YAW_ABS150_LRUN_CL_DLYM:
            linear_velocity = 0;
            angular_velocity = -turn_vel;
            if (my_abs(Yaw / 100) < 150)
            {
                linear_velocity = 0;
                angular_velocity = 0;
                collision_left_leftrun_step_status = GOSTR_YAW_EQUAL_ABS150_LRUN_CL_DLYM;
                break;
            }
            if (obstacleSignal == front_obstacle || obstacleSignal == left_obstacle || obstacleSignal == right_obstacle)
            {
                linear_velocity = 0;
                angular_velocity = 0;
                collision_left_leftrun_step_status = TURN_CLOCK_TARGET_YAW_ABS150_COLLISION_LRUN_CL_DLYM;
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
                collision_left_leftrun_step_status = TURN_CLOCK_TARGET_YAW_ABS150_LRUN_CL_DLYM;
                break;
            }
            break;
        case  GOSTR_YAW_EQUAL_ABS150_LRUN_CL_DLYM:
            cnt_update = 0;
            last_position_y = current_pose->y;
            last_position_xx = current_pose->x;
            collision_left_leftrun_step_status = GOSTR_BYPASS_LRUN_CL_DLYM;
            break;
        case  GOSTR_BYPASS_LRUN_CL_DLYM:
            last_position_x = current_pose->x;
            cnt_update +=1;
            linear_velocity = bypass_velocity;
            angular_velocity = 0;
            if(cnt_update > 4&&my_abs(last_position_xx - current_pose->x)<20&&my_abs(last_position_y - current_pose->y)<20)
            {
                cnt_update = 0;
                collision_left_leftrun_step_status = MORE_TRY_BREAK_BYPASS_LRUN_CL_DLYM;
                break;
            }
            collision_left_leftrun_step_status = GOSTR_BYPASS_LOOP_LRUN_CL_DLYM;
            break;
        case  GOSTR_BYPASS_LOOP_LRUN_CL_DLYM:
            if (obstacleSignal == left_obstacle)
            {
                collision_left_leftrun_step_status = LEFT_COLLISION_BYPASS_LRUN_CL_DLYM;
                break;
            }
            if (my_abs(last_position_x - current_pose->x) > lateral_move_distance / 3)
            {
                collision_left_leftrun_step_status = GOSTR_X_MORE_ONE_THIRD_LATERALDIS_BYPASS_LRUN_CL_DLYM;
                break;
            }
            if (my_abs(last_position_y - current_pose->y) > close_edge || obstacleSignal == front_obstacle || obstacleSignal == right_obstacle)
            {
                collision_left_leftrun_step_status = GOSTR_BYPASS_BOW_CONTINUE_LRUN_CL_DLYM;
                break;
            }
            if (last_position_y > current_pose->y+1)
            {
                linear_velocity = 0;
                angular_velocity = 0;
                collision_left_leftrun_step_status = GOSTR_BYPASS_OLD_BOW_CONTINUE_LRUN_CL_DLYM;
                break;
            }
            if (my_abs(current_pose->x) > W)
            {
                collision_left_leftrun_step_status = COMPLETE_LRUN_CL_DLYM;
                break;
            }
            if (my_abs(Yaw / 100) < 90)
            {
                collision_left_leftrun_step_status = TURN_CCLOCK_TARGET_YAW_LESS_ABS3_LRUN_CL_DLYM;
                break;
            }
            break;

        case  GOSTR_BYPASS_OLD_BOW_CONTINUE_LRUN_CL_DLYM:
            if (obstacleSignal == front_obstacle || obstacleSignal == right_obstacle || obstacleSignal == left_obstacle)
            {
                collision_left_leftrun_step_status = GOSTR_BYPASS_OLD_BOW_CONTINUE_COLLISION_LRUN_CL_DLYM;
                break;
            }
            if (my_abs(Yaw / 100) < 175 )
            {
                linear_velocity = 0;
                angular_velocity = 0;
                bow_continue = true;
                collision_left_leftrun_step_status = COMPLETE_LRUN_CL_DLYM;
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
                collision_left_leftrun_step_status = GOSTR_BYPASS_OLD_BOW_CONTINUE_LRUN_CL_DLYM;
                turn_start_update = 0;
                break;
            }
            break;
        case  TURN_CCLOCK_TARGET_YAW_LESS_ABS3_LRUN_CL_DLYM:
            if (obstacleSignal == front_obstacle || obstacleSignal == right_obstacle || obstacleSignal == left_obstacle)
            {
                collision_left_leftrun_step_status = TURN_CCLOCK_TARGET_YAW_LESS_ABS3_COLLISION_LRUN_CL_DLYM;
                break;
            }
            if (my_abs(Yaw / 100) < 5 )
            {
                linear_velocity = 0;
                angular_velocity = 0;
                bow_continue = true;
                collision_left_leftrun_step_status = COMPLETE_LRUN_CL_DLYM;
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
                collision_left_leftrun_step_status = TURN_CCLOCK_TARGET_YAW_LESS_ABS3_LRUN_CL_DLYM;
                turn_start_update = 0;
                break;
            }
            break;

        case  GOSTR_BYPASS_BOW_CONTINUE_LRUN_CL_DLYM:
            if (obstacleSignal == front_obstacle || obstacleSignal == right_obstacle || obstacleSignal == left_obstacle)
            {
                collision_left_leftrun_step_status = GOSTR_BYPASS_BOW_CONTINUE_COLLISION_LRUN_CL_DLYM;
                break;
            }
            if (my_abs(Yaw / 100) < 5 )
            {
                linear_velocity = 0;
                angular_velocity = 0;
                bow_continue = true;
                collision_left_leftrun_step_status = GOSTR_BYPASS_BOW_CONTINUE_EXIT_LRUN_CL_DLYM;
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
                collision_left_leftrun_step_status = GOSTR_BYPASS_BOW_CONTINUE_LRUN_CL_DLYM;
                turn_start_update = 0;
                break;
            }
            break;
        case  GOSTR_BYPASS_BOW_CONTINUE_EXIT_LRUN_CL_DLYM:
            maintain_bow_distance = current_pose->y;
            collision_left_leftrun_step_status = COMPLETE_LRUN_CL_DLYM;
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
                temporary_yaw = Yaw / 100;
                if (my_abs(Yaw / 100) > 150) collision_left_leftrun_step_status = TURN_CLCOK_TARGET_YAW_MORE_ABS150_LRUN_CL_DLYM;
                else collision_left_leftrun_step_status = TURN_CLCOK_TARGET_YAW_LESS_ABS150_LRUN_CL_DLYM;
                break;
            }
            break;
        case  TURN_CLCOK_TARGET_YAW_MORE_ABS150_LRUN_CL_DLYM:
            if (obstacleSignal == front_obstacle || obstacleSignal == right_obstacle || obstacleSignal == left_obstacle)
            {
                linear_velocity = 0;
                angular_velocity = 0;
                collision_left_leftrun_step_status = TURN_CLCOK_TARGET_YAW_MORE_ABS150_COLLISION_LRUN_CL_DLYM;
                break;
            }
            if (my_abs(Yaw / 100) < 150)
            {
                linear_velocity = 0;
                angular_velocity = 0;
                collision_left_leftrun_step_status = GOSTR_BYPASS_LRUN_CL_DLYM;
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
				 collision_left_leftrun_step_status = TURN_CLCOK_TARGET_YAW_MORE_ABS150_LRUN_CL_DLYM;
				break;
			}
			break;

        case  TURN_CLCOK_TARGET_YAW_LESS_ABS150_LRUN_CL_DLYM:
            if (obstacleSignal == front_obstacle || obstacleSignal == right_obstacle || obstacleSignal == left_obstacle)
            {
                collision_left_leftrun_step_status = TURN_CLCOK_TARGET_YAW_LESS_ABS150_COLLISION_LRUN_CL_DLYM;
                break;
            }
            if (my_abs(Yaw / 100 - temporary_yaw) > 15)
            {
                collision_left_leftrun_step_status = GOSTR_BYPASS_LRUN_CL_DLYM;
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
				 collision_left_leftrun_step_status = TURN_CLCOK_TARGET_YAW_LESS_ABS150_LRUN_CL_DLYM;
				break;
			}
			break;
        case  GOSTR_X_MORE_ONE_THIRD_LATERALDIS_BYPASS_LRUN_CL_DLYM:
            if (my_abs(Yaw / 100) > 150 || Yaw / 100 > 0)
            {
                linear_velocity = 0;
                angular_velocity = 0;
                if(Yaw/100>0&&my_abs(Yaw / 100) > 150){
                    turn_start_update=1;
                }
                temporary_yaw = Yaw / 100;
                collision_left_leftrun_step_status = TURN_CCLOCK_YAW_ADD_ABS30_LRUN_CL_DLYM;
                break;
            }
            collision_left_leftrun_step_status = GOSTR_BYPASS_LRUN_CL_DLYM;
            break;
        case  TURN_CCLOCK_YAW_ADD_ABS30_LRUN_CL_DLYM:
			if(turn_start_update==1){
				if(my_abs(Yaw / 100)<150)
				{
					turn_start_update = 0;
					linear_velocity = 0;
					angular_velocity = 0;
					collision_left_leftrun_step_status = GOSTR_BYPASS_LRUN_CL_DLYM;
					break;
				}
			}
			else{
				if (my_abs(Yaw / 100 - temporary_yaw) > 30)
				{
					linear_velocity = 0;
					angular_velocity = 0;
					collision_left_leftrun_step_status = GOSTR_BYPASS_LRUN_CL_DLYM;
					break;
				}
			}
            if (obstacleSignal == front_obstacle || obstacleSignal == left_obstacle || obstacleSignal == right_obstacle)
            {
                collision_left_leftrun_step_status = TURN_CCLOCK_YAW_ADD_ABS30_COLLISION_LRUN_CL_DLYM;
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
                collision_left_leftrun_step_status = TURN_CCLOCK_YAW_ADD_ABS30_LRUN_CL_DLYM;
                turn_start_update = 0;
                break;
            }
            break;
        case  MORE_TRY_BREAK_BYPASS_LRUN_CL_DLYM:
            if (obstacleSignal == front_obstacle || obstacleSignal == right_obstacle || obstacleSignal == left_obstacle)
            {
                collision_left_leftrun_step_status = MORE_TRY_BREAK_BYPASS_COLLISION_LRUN_CL_DLYM;
                bow_continue = true;
                break;
            }
            if (my_abs(Yaw / 100) < 5)
            {
                linear_velocity = 0;
                angular_velocity = 0;
                bow_continue = true;
                collision_left_leftrun_step_status = GOSTR_BYPASS_BOW_CONTINUE_EXIT_LRUN_CL_DLYM;
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
				collision_left_leftrun_step_status = MORE_TRY_BREAK_BYPASS_LRUN_CL_DLYM;
				turn_start_update = 0;
				break;
			}
			break;

        case  COMPLETE_LRUN_CL_DLYM:
            collision_left_leftrun_step_status = 0;
            complete_flag = 1;
            break;
        case  DIR_LEFT_YAW_LESS_ABS90_CLLRS:
            linear_velocity = 0;
            angular_velocity = 0;
            last_position_y = current_pose->y;
            collision_left_leftrun_step_status = TURN_CCLOCK_TARGET_YAW_ABS27_LRUN_CL_DLYL;
            break;
        case  TURN_CCLOCK_TARGET_YAW_ABS27_LRUN_CL_DLYL:
            linear_velocity = 0;
            angular_velocity = turn_vel;
            if (my_abs(Yaw / 100) > 27)
            {
                linear_velocity = 0;
                angular_velocity = 0;
                collision_left_leftrun_step_status = GOSTR_YAW_EQUAL_ABS27_LRUN_CL_DLYL;
                break;
            }
            if (obstacleSignal == front_obstacle || obstacleSignal == left_obstacle || obstacleSignal == right_obstacle)
            {
                linear_velocity = 0;
                angular_velocity = 0;
                collision_left_leftrun_step_status = TURN_CCLOCK_TAEGET_YAW_ABS27_COLLISION_LRUN_CL_DLYL;
                break;
            }
            break;
        case  TURN_CCLOCK_TAEGET_YAW_ABS27_COLLISION_LRUN_CL_DLYL:
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
                collision_left_leftrun_step_status = TURN_CCLOCK_TARGET_YAW_ABS27_LRUN_CL_DLYL;
                break;
            }
            break;
        case  GOSTR_YAW_EQUAL_ABS27_LRUN_CL_DLYL:
            linear_velocity = long_stra_vel;
            angular_velocity = 0;
            if (obstacleSignal == front_obstacle || obstacleSignal == left_obstacle || obstacleSignal == right_obstacle)
            {
                collision_left_leftrun_step_status  = TURN_CCLOCK_TARGET_YAW_ABS57_LRUN_CL_DLYL;
                break;
            }
            if (my_abs(last_position_y - current_pose->y) > lateral_move_distance / 3)
            {
                collision_left_leftrun_step_status  = TURN_CCLOCK_TARGET_YAW_ABS57_LRUN_CL_DLYL;
                break;
            }
            break;
        case  TURN_CCLOCK_TARGET_YAW_ABS57_LRUN_CL_DLYL:
            linear_velocity = 0;
            angular_velocity = turn_vel;
            if (Yaw / 100 > 57)
            {
                linear_velocity = 0;
                angular_velocity = 0;
                collision_left_leftrun_step_status  = GOSTR_YAW_EQUAL_ABS57_LRUN_CL_DLYL;
                break;
            }
            if (obstacleSignal == front_obstacle || obstacleSignal == left_obstacle || obstacleSignal == right_obstacle)
            {
                collision_left_leftrun_step_status  = TURN_CCLOCK_TARGET_YAW_ABS57_COLLISION_LRUN_CL_DLYL;
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
                collision_left_leftrun_step_status = TURN_CCLOCK_TARGET_YAW_ABS57_LRUN_CL_DLYL;
                break;
            }
            break;
        case  GOSTR_YAW_EQUAL_ABS57_LRUN_CL_DLYL:
            linear_velocity = long_stra_vel;
            angular_velocity = 0;
            if (obstacleSignal == front_obstacle || obstacleSignal == left_obstacle || obstacleSignal == right_obstacle)
            {
                linear_velocity = 0;
                angular_velocity = 0;
                collision_left_leftrun_step_status = TURN_CCLOCK_TARGET_YAW_ABS87_LRUN_CL_DLYL;
                break;
            }
            if (my_abs(last_position_y - current_pose->y) > (2 * lateral_move_distance) / 3)
            {
                linear_velocity = 0;
                angular_velocity = 0;
                collision_left_leftrun_step_status = TURN_CCLOCK_TARGET_YAW_ABS87_LRUN_CL_DLYL;
                break;
            }
            break;
        case  TURN_CCLOCK_TARGET_YAW_ABS87_LRUN_CL_DLYL:
            linear_velocity = 0;
            angular_velocity = turn_vel;
            if (Yaw / 100 > 87)
            {
                linear_velocity = 0;
                angular_velocity = 0;
                collision_left_leftrun_step_status = GOSTR_YAW_EQUAL_ABS87_LRUN_CL_DLYL;
                break;
            }
            if (obstacleSignal == front_obstacle || obstacleSignal == left_obstacle || obstacleSignal == right_obstacle)
            {
                linear_velocity = 0;
                angular_velocity = 0;
                collision_left_leftrun_step_status = TURN_CCLOCK_TARGET_YAW_ABS87_COLLISION_LRUN_CL_DLYL;
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
                collision_left_leftrun_step_status = TURN_CCLOCK_TARGET_YAW_ABS87_LRUN_CL_DLYL;
                break;
            }
            break;
        case  GOSTR_YAW_EQUAL_ABS87_LRUN_CL_DLYL:
            linear_velocity = long_stra_vel;
            angular_velocity = 0;
            if (obstacleSignal == front_obstacle || obstacleSignal == left_obstacle || obstacleSignal == right_obstacle)
            {
                linear_velocity = 0;
                angular_velocity = 0;
                collision_left_leftrun_step_status = LEFT_WALK_EDGE_LRUN_CL_DLYL;
                break;
            }
            if (my_abs(last_position_y - current_pose->y) > lateral_move_distance)
            {
                linear_velocity = 0;
                angular_velocity = 0;
                collision_left_leftrun_step_status = TURN_CCLOCK_TARGET_YAW_ABS173_LRUN_CL_DLYL;
                break;
            }
            break;
        case  LEFT_WALK_EDGE_LRUN_CL_DLYL:
			collision_left_leftrun_step_status = 0;
			complete_flag = 2;
			break;
        case  TURN_CCLOCK_TARGET_YAW_ABS173_LRUN_CL_DLYL:
            linear_velocity = 0;
            angular_velocity = turn_vel;
            if (my_abs(Yaw / 100) >175)
            {
                linear_velocity = 0;
                angular_velocity = 0;
                collision_left_leftrun_step_status = COMPLETE_LRUN_CL_DLYL;
                break;
            }
            if (obstacleSignal == front_obstacle || obstacleSignal == left_obstacle || obstacleSignal == right_obstacle)
            {
                linear_velocity = 0;
                angular_velocity = 0;
                collision_left_leftrun_step_status = TURN_CCLOCK_TARGET_YAW_ABS173_COLLISION_LRUN_CL_DLYL;
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
                collision_left_leftrun_step_status = TURN_CCLOCK_TARGET_YAW_ABS173_LRUN_CL_DLYL;
                break;
            }
            break;
        case  COMPLETE_LRUN_CL_DLYL:
            maintain_bow_distance = current_pose->y;
            collision_left_leftrun_step_status = 0;
            complete_flag = 1;
            break;
    }
    return complete_flag;
}

unsigned char CollisionFrontLeftRunStep(POSE *current_pose,unsigned char obstacleSignal)
{
   
    unsigned char complete_flag = 0;
    Yaw = current_pose->orientation;
    switch(collision_front_leftrun_step_status)
    {
        case 0:
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
                collision_front_leftrun_step_status = GOBACK_DISTANCE_CFLRS;
                turn_start_update = 0;
                break;
            }
            break;
        case GOBACK_DISTANCE_CFLRS:
            if(turn_start_update == 0)
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
                    collision_front_leftrun_step_status  = DIR_LEFT_YAW_LESS_ABS90_CFRLS;
                }
                else
                {
                    collision_front_leftrun_step_status = DIR_LEFT_YAW_MORE_ABS90_CFRLS;
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
                    collision_front_leftrun_step_status = DIR_LEFT_YAW_LESS_ABS90_CFRLS;
                }
                else
                {
                    collision_front_leftrun_step_status = DIR_LEFT_YAW_MORE_ABS90_CFRLS;
                }
                turn_start_update = 0;
                break;
            }
            break;
        case DIR_LEFT_YAW_LESS_ABS90_CFRLS:
            linear_velocity = 0;
            angular_velocity = 0;
            last_position_y = current_pose->y;
            collision_front_leftrun_step_status = TURN_CCLOCK_TARGET_YAW_ABS60_LRUN_CF_DLYL;
            break;
        case TURN_CCLOCK_TARGET_YAW_ABS60_LRUN_CF_DLYL:
            linear_velocity = 0;
            angular_velocity = turn_vel;
            if ((Yaw / 100)  > 60)
            {
                linear_velocity = 0;
                angular_velocity = 0;
                collision_front_leftrun_step_status = GOSTR_YAW_ABS60_LRUN_CF_DLYL;
                break;
            }
            if (obstacleSignal == front_obstacle || obstacleSignal == left_obstacle || obstacleSignal == right_obstacle)
            {
                linear_velocity = 0;
                angular_velocity = 0;
                collision_front_leftrun_step_status = TURN_CCLOCK_TARGET_YAW_ABS60_COLLISION_LRUN_CF_DLYL;
                break;
            }
            break;
        case TURN_CCLOCK_TARGET_YAW_ABS60_COLLISION_LRUN_CF_DLYL:
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
                collision_front_leftrun_step_status = TURN_CCLOCK_TARGET_YAW_ABS60_LRUN_CF_DLYL;
                break;
            }
            break;
        case GOSTR_YAW_ABS60_LRUN_CF_DLYL:
            linear_velocity = long_stra_vel;
            angular_velocity = 0;
            if (obstacleSignal == front_obstacle || obstacleSignal == left_obstacle || obstacleSignal == right_obstacle)
            {
                collision_front_leftrun_step_status  = TURN_CCLOCK_TARGET_YAW_ABS82_LRUN_CF_DLYL;
                break;
            }
            if (my_abs(last_position_y - current_pose->y) > lateral_move_distance)
            {
                collision_front_leftrun_step_status  = TURN_CCLOCK_TARGET_YAW_ABS82_LRUN_CF_DLYL;
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
                collision_front_leftrun_step_status  = GOSTR_YAW_ABS82_LRUN_CF_DLYL;
                break;
            }
            if (obstacleSignal == front_obstacle || obstacleSignal == left_obstacle || obstacleSignal == right_obstacle)
            {
                collision_front_leftrun_step_status  = TURN_CCLOCK_TARGET_YAW_ABS82_COLLISION_LRUN_CF_DLYL;
                break;
            }
            break;
        case TURN_CCLOCK_TARGET_YAW_ABS82_COLLISION_LRUN_CF_DLYL:
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
                collision_front_leftrun_step_status = TURN_CCLOCK_TARGET_YAW_ABS82_LRUN_CF_DLYL;
                break;
            }
            break;
        case GOSTR_YAW_ABS82_LRUN_CF_DLYL:
            linear_velocity = long_stra_vel;
            angular_velocity = 0;
            if (obstacleSignal == front_obstacle || obstacleSignal == left_obstacle || obstacleSignal == right_obstacle)
            {
                linear_velocity = 0;
                angular_velocity = 0;
                collision_front_leftrun_step_status = LEFT_WALK_EDGE_LRUN_CF_DLYL;
                break;
            }
            if (my_abs(last_position_y - current_pose->y) > (lateral_move_distance))
            {
                linear_velocity = 0;
                angular_velocity = 0;
                collision_front_leftrun_step_status = TURN_CCLOCK_TARGET_YAW_ABS173_LRUN_CF_DLYL;
                break;
            }
            break;
        case LEFT_WALK_EDGE_LRUN_CF_DLYL:
			collision_front_leftrun_step_status = 0;
			complete_flag = 2;
			break;
        case TURN_CCLOCK_TARGET_YAW_ABS173_LRUN_CF_DLYL:
            linear_velocity = 0;
            angular_velocity = turn_vel;
            if (my_abs(Yaw / 100) > 175 )
            {
                linear_velocity = 0;
                angular_velocity = 0;
                collision_front_leftrun_step_status  = COMPLETE_LRUN_CF_DLYL;
                break;
            }
            if (obstacleSignal == front_obstacle || obstacleSignal == left_obstacle || obstacleSignal == right_obstacle)
            {
                collision_front_leftrun_step_status  = TURN_CCLOCK_TARGET_YAW_ABS173_COLLISION_LRUN_CF_DLYL;
                break;
            }
            break;
        case TURN_CCLOCK_TARGET_YAW_ABS173_COLLISION_LRUN_CF_DLYL:
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
                collision_front_leftrun_step_status = TURN_CCLOCK_TARGET_YAW_ABS173_LRUN_CF_DLYL;
                break;
            }
            break;
        case COMPLETE_LRUN_CF_DLYL:
            collision_front_leftrun_step_status = 0;
            complete_flag = 1;
            break;
        case DIR_LEFT_YAW_MORE_ABS90_CFRLS:
            linear_velocity = 0;
            angular_velocity = 0;
            last_position_y = current_pose->y;
            collision_front_leftrun_step_status = TURN_CLOCK_TARGET_YAW_ABS120_LRUN_CF_DLYM;
            break;
        case TURN_CLOCK_TARGET_YAW_ABS120_LRUN_CF_DLYM:
            linear_velocity = 0;
            angular_velocity = -turn_vel;
            if (my_abs(Yaw / 100) < 120)
            {
                linear_velocity = 0;
                angular_velocity = 0;
                collision_front_leftrun_step_status = GOSTR_YAW_EQUAL_ABS120_LRUN_CF_CF_DLYM;
                break;
            }
            if (obstacleSignal == front_obstacle || obstacleSignal == left_obstacle || obstacleSignal == right_obstacle)
            {
                linear_velocity = 0;
                angular_velocity = 0;
                collision_front_leftrun_step_status = TURN_CLOCK_TARGET_YAW_ABS120_COLLISION_LRUN_CF_DLYM;
                break;
            }
            break;
        case TURN_CLOCK_TARGET_YAW_ABS120_COLLISION_LRUN_CF_DLYM:
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
                collision_front_leftrun_step_status = TURN_CLOCK_TARGET_YAW_ABS120_LRUN_CF_DLYM;
                break;
            }
            break;
        case GOSTR_YAW_EQUAL_ABS120_LRUN_CF_CF_DLYM:
            linear_velocity = long_stra_vel;
            angular_velocity = 0;
            if (obstacleSignal == front_obstacle || obstacleSignal == left_obstacle || obstacleSignal == right_obstacle)
            {
                collision_front_leftrun_step_status  = TURN_CLOCK_TARGET_YAW_ABS98_LRUN_CF_DLYM;
                break;
            }
            if (my_abs(last_position_y - current_pose->y) > lateral_move_distance)
            {
                collision_front_leftrun_step_status  = TURN_CLOCK_TARGET_YAW_ABS98_LRUN_CF_DLYM;
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
                collision_front_leftrun_step_status  = GOSTR_YAW_EQUAL_ABS98_LRUN_CF_DLYM;
                break;
            }
            if (obstacleSignal == front_obstacle || obstacleSignal == left_obstacle || obstacleSignal == right_obstacle)
            {
                collision_front_leftrun_step_status  = TURN_CLOCK_TARGET_YAW_ABS98_COLLISION_LRUN_CF_DLYM;
                break;
            }
            break;
        case TURN_CLOCK_TARGET_YAW_ABS98_COLLISION_LRUN_CF_DLYM:
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
                collision_front_leftrun_step_status = TURN_CLOCK_TARGET_YAW_ABS98_LRUN_CF_DLYM;
                break;
            }
            break;
        case GOSTR_YAW_EQUAL_ABS98_LRUN_CF_DLYM:
            linear_velocity = long_stra_vel;
            angular_velocity = 0;
            if (obstacleSignal == front_obstacle || obstacleSignal == left_obstacle || obstacleSignal == right_obstacle)
            {
                linear_velocity = 0;
                angular_velocity = 0;
                collision_front_leftrun_step_status = LEFT_REVERSE_WALK_EDGE_LRUN_CF_DLYM;
                break;
            }
            if (my_abs(last_position_y - current_pose->y) > (lateral_move_distance))
            {
                linear_velocity = 0;
                angular_velocity = 0;
                collision_front_leftrun_step_status = TURN_CLOCK_TARGET_YAW_ABS8_LRUN_CF_DLYM;
                break;
            }
            break;
        case LEFT_REVERSE_WALK_EDGE_LRUN_CF_DLYM:
			collision_front_leftrun_step_status = 0;
			complete_flag = 3;
			break;
        case TURN_CLOCK_TARGET_YAW_ABS8_LRUN_CF_DLYM:
            linear_velocity = 0;
            angular_velocity = -turn_vel;
            if (my_abs(Yaw / 100)< 5 )
            {
                linear_velocity = 0;
                angular_velocity = 0;
                collision_front_leftrun_step_status  = COMPLETE_LRUN_CF_DLYM;
                break;
            }
            if (obstacleSignal == front_obstacle || obstacleSignal == left_obstacle || obstacleSignal == right_obstacle)
            {
                collision_front_leftrun_step_status  = TURN_CLOCK_TARGET_YAW_ABS8_COLLISION_LRUN_CF_DLYM;
                break;
            }

            break;
        case TURN_CLOCK_TARGET_YAW_ABS8_COLLISION_LRUN_CF_DLYM:
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
                collision_front_leftrun_step_status = TURN_CLOCK_TARGET_YAW_ABS8_LRUN_CF_DLYM;
                break;
            }
            break;
        case COMPLETE_LRUN_CF_DLYM:
            collision_front_leftrun_step_status = 0;
            complete_flag = 1;
            break;
    }
    return complete_flag;

}

unsigned char LeftWalkEdge(POSE *current_pose,unsigned char obstacleSignal)
{
    
    unsigned char complete_flag = 0;
    Yaw = current_pose->orientation;
	
    switch(left_walk_edge_status)
    {
        case 0:
            left_walk_edge_status = LEFT_GOBACK_WALK_EDGE;
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
                left_walk_edge_status = LEFT_EDGE_TURN_CCLOCK_TARGET_YAW_MORE_ABS117_WE;
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
                left_walk_edge_status = LEFT_EDGE_READY_GOSTR_BYPASS_WE;
                break;
            }
            if (obstacleSignal == front_obstacle || obstacleSignal == left_obstacle || obstacleSignal == right_obstacle)
            {
                linear_velocity = 0;
                angular_velocity = 0;
                left_walk_edge_status = LEFT_EDGE_TURN_CCLOCK_TARGET_YAW_MORE_ABS117_COLLISION_WE;
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
                left_walk_edge_status = LEFT_EDGE_TURN_CCLOCK_TARGET_YAW_MORE_ABS117_WE;
                break;
            }
            break;
        case LEFT_EDGE_READY_GOSTR_BYPASS_WE:
            edge_length_start = current_pose->x + W;
            returnorigin = false;
            b_last_position_yy = false;
            last_position_y = current_pose->y;
            last_position_yy = 0;
            temporary_close_edge = close_edge;
            left_walk_edge_status = LEFT_EDGE_GOSTR_BYPASS_WE_X;
            break;
        case LEFT_EDGE_GOSTR_BYPASS_WE_X:
			last_position_x = current_pose->x;
			left_walk_edge_status = LEFT_EDGE_GOSTR_BYPASS_WE;
			break;
        case LEFT_EDGE_GOSTR_BYPASS_WE:
            linear_velocity = long_stra_vel;
            angular_velocity =0;
            if (obstacleSignal == front_obstacle || obstacleSignal == right_obstacle || obstacleSignal == left_obstacle)
            {
                left_walk_edge_status = LEFT_EDGE_COLLISION_BYPASS_WE;
                break;
            }
            if (my_abs(last_position_x - current_pose->x) > lateral_move_distance)
            {
                left_walk_edge_status = LEFT_EDGE_GOSTR_X_MORE_LATERALDIS_BYPASS_WE;
                break;
            }
            if ((current_pose->y - last_position_y)  > temporary_close_edge)
            {
                left_walk_edge_status = LEFT_EDGE_BOW_CONTINUE_WE;
                break;
            }
            if (b_last_position_yy == true && my_abs(last_position_yy - current_pose->y) > 3 * lateral_move_distance)
            {
                left_walk_edge_status = LEFT_EDGE_BOW_CONTINUE_WE;
                break;
            }
            if (my_abs(Yaw / 100) < 120 && (Yaw / 100 < 0) && (b_last_position_yy == false))
            {
                last_position_yy = current_pose->y;
                b_last_position_yy = true;
            }

            break;

        case LEFT_EDGE_REBACK_GOSTR_BYPASS_CHECK_WE:
            if (my_abs(Yaw / 100) < 105 && (Yaw / 100 < 0))
            {
                left_walk_edge_status = LEFT_EDGE_TARGET_YAW_LESS_ABS105_LESS_0_BYPASS_WE;
                break;
            }
            left_walk_edge_status = LEFT_EDGE_GOSTR_BYPASS_WE_X;
            break;
        case LEFT_EDGE_COLLISION_BYPASS_WE:
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
                temporary_yaw = Yaw / 100;
                left_walk_edge_status = LEFT_EDGE_TURN_CCLOCK_YAW_ADD_ABS15_WE;
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
                left_walk_edge_status = LEFT_EDGE_REBACK_GOSTR_BYPASS_CHECK_WE;
                break;
            }
            if (obstacleSignal == front_obstacle || obstacleSignal == left_obstacle || obstacleSignal == right_obstacle)
            {
                linear_velocity = 0;
                angular_velocity = 0;
                left_walk_edge_status = LEFT_EDGE_TURN_CCLOCK_YAW_ADD_ABS15_COLLISION_WE;
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
                left_walk_edge_status = LEFT_EDGE_TURN_CCLOCK_YAW_ADD_ABS15_WE;
                break;
            }
            break;
        case LEFT_EDGE_TARGET_YAW_LESS_ABS105_LESS_0_BYPASS_WE:
            if (my_abs(current_pose->x + W - edge_length_start) > 2000) // if (my_abs(current_pose->x + W - edge_length_start) > (gridmap.Edge_length() / 4))
            {
                left_walk_edge_status = LEFT_EDGE_DELTA_X_MORE_ONE_FOURTH_CLEANED_MAP_WIDTH_WE;
                break;
            }
            else
            {
                left_walk_edge_status = LEFT_EDGE_TURN_CCLOCK_TARGET_YAW_LESS_ABS3_WE;
                break;
            }
        case LEFT_EDGE_DELTA_X_MORE_ONE_FOURTH_CLEANED_MAP_WIDTH_WE:
            left_walk_edge_status = LEFT_EDGE_RETURN_ORIGIN_WE;
            break;
        case LEFT_EDGE_TURN_CCLOCK_TARGET_YAW_LESS_ABS3_WE:
            if (my_abs(Yaw / 100) < 5)
            {
                linear_velocity = 0;
                angular_velocity = 0;
                left_walk_edge_status = LEFT_EDGE_LEFT_EDGE_DILEMMA_WE;
                break;
            }

            if (obstacleSignal == front_obstacle || obstacleSignal == left_obstacle || obstacleSignal == right_obstacle)
            {
                left_walk_edge_status = LEFT_EDGE_TURN_CCLOCK_TARGET_YAW_LESS_ABS3_COLLISION_WE;
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
                left_walk_edge_status = LEFT_EDGE_TURN_CCLOCK_TARGET_YAW_LESS_ABS3_WE;
                break;
            }

            break;
        case LEFT_EDGE_LEFT_EDGE_DILEMMA_WE:
			complete_flag = 2;
			left_walk_edge_status = 0;
			break;
        case LEFT_EDGE_GOSTR_X_MORE_LATERALDIS_BYPASS_WE:
            temporary_yaw = Yaw / 100;
            if (my_abs(Yaw / 100) < 150 && (Yaw / 100 < 0))
            {
                left_walk_edge_status = LEFT_EDGE_TURN_CLOCK_TARGET_YAW_MORE_0_WE;
                break;
            }
            else if (my_abs(Yaw / 100) >= 150)
            {
                left_walk_edge_status = LEFT_EDGE_TURN_CLOCK_TARGET_YAW_LESS_135_DETAL_YAW_MORE_30_WE;
                break;
            }
            else{
                left_walk_edge_status = LEFT_EDGE_GOSTR_BYPASS_WE_X;
                break;
            }
        case LEFT_EDGE_TURN_CLOCK_TARGET_YAW_MORE_0_WE:
            linear_velocity = 0;
            angular_velocity = -turn_vel;
            if (Yaw / 100 > 0 )
            {
                linear_velocity = 0;
                angular_velocity = 0;
                left_walk_edge_status = LEFT_EDGE_GOSTR_BYPASS_WE_X;
                break;
            }
            if (obstacleSignal == front_obstacle || obstacleSignal == left_obstacle || obstacleSignal == right_obstacle)
            {
                linear_velocity = 0;
                angular_velocity = 0;
                left_walk_edge_status = LEFT_EDGE_TURN_CLOCK_TARGET_YAW_MORE_0_COLLISION_WE;
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
                left_walk_edge_status = LEFT_EDGE_TURN_CLOCK_TARGET_YAW_MORE_0_WE;
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
                left_walk_edge_status = LEFT_EDGE_GOSTR_BYPASS_WE_X;
                break;
            }
            if (obstacleSignal == front_obstacle || obstacleSignal == left_obstacle || obstacleSignal == right_obstacle)
            {
                linear_velocity = 0;
                angular_velocity = 0;
                left_walk_edge_status = LEFT_EDGE_TURN_CLOCK_TARGET_YAW_LESS_135_DETAL_YAW_MORE_30_COLLISION_WE;
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
                left_walk_edge_status = LEFT_EDGE_TURN_CLOCK_TARGET_YAW_LESS_135_DETAL_YAW_MORE_30_WE;
                break;
            }
            break;
        case LEFT_EDGE_BOW_CONTINUE_WE:
            linear_velocity = 0;
            angular_velocity = 0;
            complete_flag = 1;
            left_walk_edge_status = 0;
            break;
        case LEFT_EDGE_RETURN_ORIGIN_WE:
            linear_velocity = 0;
            angular_velocity = 0;
            complete_flag = 3;
            left_walk_edge_status = 0;
            break;
    }
    return complete_flag;                             
}

unsigned char LeftReverseWalkEdge(POSE *current_pose,unsigned char obstacleSignal)
{
    
    unsigned char complete_flag = 0;
    Yaw = current_pose->orientation;
    switch(left_reverse_walk_edge_status)
    {
        case 0:
            left_reverse_walk_edge_status = LEFT_GOBACK_REVERSE_WALK_EDGE;
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
                left_reverse_walk_edge_status = LEFT_REVERSE_EDGE_TURN_CLOCK_TARGET_YAW_LESS_ABS63_RWE;
                break;
            }
            break;
        case LEFT_REVERSE_EDGE_TURN_CLOCK_TARGET_YAW_LESS_ABS63_RWE                          :
            linear_velocity = 0;
            angular_velocity = -turn_vel;
            if (my_abs(Yaw / 100) < 63)
            {
                linear_velocity = 0;
                angular_velocity = 0;
                left_reverse_walk_edge_status = LEFT_REVERSE_EDGE_READY_GOSTR_BYPASS_RWE;
                break;
            }
            if (obstacleSignal == front_obstacle || obstacleSignal == left_obstacle || obstacleSignal == right_obstacle)
            {
                linear_velocity = 0;
                angular_velocity = 0;
                left_reverse_walk_edge_status = LEFT_REVERSE_EDGE_TURN_CLOCK_TARGET_YAW_LESS_ABS63_COLLISION_RWE;
                break;
            }
            break;
        case LEFT_REVERSE_EDGE_TURN_CLOCK_TARGET_YAW_LESS_ABS63_COLLISION_RWE                 :
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
                left_reverse_walk_edge_status = LEFT_REVERSE_EDGE_TURN_CLOCK_TARGET_YAW_LESS_ABS63_RWE;
                break;
            }
            break;
        case LEFT_REVERSE_EDGE_READY_GOSTR_BYPASS_RWE                                         :
            edge_length_start = current_pose->x + W;
            returnorigin = false;
            b_last_position_yy = false;
            last_position_y = current_pose->y;
            last_position_yy = 0;
            temporary_close_edge = close_edge;
            left_reverse_walk_edge_status = LEFT_REVERSE_EDGE_GOSTR_BYPASS_RWE_X;
            break;

        case LEFT_REVERSE_EDGE_GOSTR_BYPASS_RWE_X:
            last_position_x = current_pose->x;
            left_reverse_walk_edge_status = LEFT_REVERSE_EDGE_GOSTR_BYPASS_RWE;
            break;

        case LEFT_REVERSE_EDGE_GOSTR_BYPASS_RWE                                               :
            linear_velocity = long_stra_vel;
            angular_velocity =0;
            if (obstacleSignal == front_obstacle || obstacleSignal == right_obstacle || obstacleSignal == left_obstacle)
            {
                left_reverse_walk_edge_status = LEFT_REVERSE_EDGE_COLLISION_BYPASS_RWE;
                break;
            }
            if (my_abs(last_position_x - current_pose->x) > lateral_move_distance)
            {
                left_reverse_walk_edge_status = LEFT_REVERSE_EDGE_GOSTR_X_MORE_LATERALDIS_BYPASS_RWE;
                break;
            }
            if (current_pose->y - last_position_y > temporary_close_edge)
            {
                left_reverse_walk_edge_status = LEFT_REVERSE_EDGE_BOW_CONTINUE_RWE;
                break;
            }
            if (b_last_position_yy == true && my_abs(last_position_yy - current_pose->y) > 3 * lateral_move_distance)
            {
                left_reverse_walk_edge_status = LEFT_REVERSE_EDGE_BOW_CONTINUE_RWE;
                break;
            }
            if (my_abs(Yaw / 100) > 60 && (Yaw / 100 < 0) && (b_last_position_yy == false))
            {
                last_position_yy = current_pose->y;
                b_last_position_yy = true;
            }
            break;
        case LEFT_REVERSE_EDGE_REBACK_GOSTR_BYPASS_CHECK_RWE:
            if (my_abs(Yaw / 100) > 75 && (Yaw / 100 < 0))
            {
                left_reverse_walk_edge_status = LEFT_REVERSE_EDGE_TARGET_YAW_MORE_ABS75_LESS_ABS0_RWE;
                break;
            }
            if (my_abs(current_pose->x + W - edge_length_start) > 3000) //  if ((my_abs(current_pose->x + W - edge_length_start) > 3 * gridmap.Edge_length() / 4))
            {
                left_reverse_walk_edge_status = LEFT_REVERSE_EDGE_DELTA_X_MORE_ONE_FOURTH_CLEANED_MAP_WIDTH_RWE;
                break;
            }
            left_reverse_walk_edge_status = LEFT_REVERSE_EDGE_GOSTR_BYPASS_RWE_X;
            break;
        case LEFT_REVERSE_EDGE_COLLISION_BYPASS_RWE                                            :
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
                temporary_yaw = Yaw / 100;
                left_reverse_walk_edge_status = LEFT_REVERSE_EDGE_TURN_CLOCK_YAW_ADD_ABS15_RWE;
                break;
            }
            break;
        case LEFT_REVERSE_EDGE_TURN_CLOCK_YAW_ADD_ABS15_RWE                                  :
            linear_velocity = 0;
            angular_velocity = -turn_vel;
            if (my_abs(Yaw / 100 - temporary_yaw) > 15)
            {
                linear_velocity = 0;
                angular_velocity = 0;
                left_reverse_walk_edge_status = LEFT_REVERSE_EDGE_REBACK_GOSTR_BYPASS_CHECK_RWE;
                break;
            }
            if (obstacleSignal == front_obstacle || obstacleSignal == left_obstacle || obstacleSignal == right_obstacle)
            {
                linear_velocity = 0;
                angular_velocity = 0;
                left_reverse_walk_edge_status = LEFT_REVERSE_EDGE_TURN_CLOCK_YAW_ADD_ABS15_COLLISION_RWE;
                break;
            }
            break;
        case LEFT_REVERSE_EDGE_TURN_CLOCK_YAW_ADD_ABS15_COLLISION_RWE                         :
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
                left_reverse_walk_edge_status = LEFT_REVERSE_EDGE_TURN_CLOCK_YAW_ADD_ABS15_RWE;
                break;
            }
            break;
        case LEFT_REVERSE_EDGE_TARGET_YAW_MORE_ABS75_LESS_ABS0_RWE:
            if (my_abs(current_pose->x + W - edge_length_start) > 2000)//if (my_abs(current_pose->x + W - edge_length_start) > (gridmap.Edge_length() / 4))
            {
                left_reverse_walk_edge_status = LEFT_REVERSE_EDGE_RETURN_ORIGIN_RWE;
                break;
            }
            else
            {
                left_reverse_walk_edge_status = LEFT_REVERSE_EDGE_TURN_CCLOCK_YAW_LESS_ABS8_RWE;
                break;
            }
            break;
        case LEFT_REVERSE_EDGE_DELTA_X_MORE_ONE_FOURTH_CLEANED_MAP_WIDTH_RWE                  :
            left_reverse_walk_edge_status = LEFT_REVERSE_EDGE_RETURN_ORIGIN_RWE;
            break;
        case LEFT_REVERSE_EDGE_TURN_CCLOCK_YAW_LESS_ABS8_RWE                                :
            if (my_abs(Yaw / 100) > 175)
            {
                linear_velocity = 0;
                angular_velocity = 0;
                left_reverse_walk_edge_status = LEFT_REVERSE_EDGE_LEFT_EDGE_DILEMMA_RWE;
                break;
            }
            if (obstacleSignal == front_obstacle || obstacleSignal == left_obstacle || obstacleSignal == right_obstacle)
            {
                left_reverse_walk_edge_status = LEFT_REVERSE_EDGE_TURN_CCLOCK_YAW_LESS_ABS8_COLLISION_RWE;
                break;
            }
            linear_velocity = 0;
            angular_velocity = -turn_vel;
            break;
        case LEFT_REVERSE_EDGE_TURN_CCLOCK_YAW_LESS_ABS8_COLLISION_RWE                       :
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
                left_reverse_walk_edge_status = LEFT_REVERSE_EDGE_TURN_CCLOCK_YAW_LESS_ABS8_RWE;
                break;
            }
            break;
        case LEFT_REVERSE_EDGE_LEFT_EDGE_DILEMMA_RWE:
            complete_flag = 2;
            left_reverse_walk_edge_status = 0;
            break;
        case LEFT_REVERSE_EDGE_GOSTR_X_MORE_LATERALDIS_BYPASS_RWE                             :
            temporary_yaw = Yaw / 100;
            if (my_abs(Yaw / 100) > 30 && (Yaw / 100 < 0))
            {
                left_reverse_walk_edge_status = LEFT_REVERSE_EDGE_TURN_CCLOCK_TARGET_YAW_MORE_0_RWE;

                break;
            }
            else if (my_abs(Yaw / 100) <= 30 )
            {
                left_reverse_walk_edge_status = LEFT_REVERSE_EDGE_TURN_CCLOCK_TARGET_YAW_MORE_ABS45_DETAL_YAW_MORE_30_RWE;
                break;
            }
            else{
                left_reverse_walk_edge_status = LEFT_REVERSE_EDGE_GOSTR_BYPASS_RWE_X;
                break;
            }
        case LEFT_REVERSE_EDGE_TURN_CCLOCK_TARGET_YAW_MORE_0_RWE                 :
            linear_velocity = 0;
            angular_velocity = turn_vel;
            if (Yaw / 100 > 0 )
            {
                linear_velocity = 0;
                angular_velocity = 0;
                left_reverse_walk_edge_status = LEFT_REVERSE_EDGE_GOSTR_BYPASS_RWE_X;
                break;
            }
            if (obstacleSignal == front_obstacle || obstacleSignal == left_obstacle || obstacleSignal == right_obstacle)
            {
                linear_velocity = 0;
                angular_velocity = 0;
                left_reverse_walk_edge_status = LEFT_REVERSE_EDGE_TURN_CCLOCK_TARGET_YAW_MORE_0_COLLISION_RWE;
                break;
            }
            break;
        case LEFT_REVERSE_EDGE_TURN_CCLOCK_TARGET_YAW_MORE_0_COLLISION_RWE      :
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
                left_reverse_walk_edge_status = LEFT_REVERSE_EDGE_TURN_CCLOCK_TARGET_YAW_MORE_0_RWE;
                break;
            }
            break;
        case LEFT_REVERSE_EDGE_TURN_CCLOCK_TARGET_YAW_MORE_ABS45_DETAL_YAW_MORE_30_RWE         :
            linear_velocity = 0;
            angular_velocity = turn_vel;
            if (my_abs(temporary_yaw - Yaw / 100) > 30 && my_abs(Yaw / 100) > 30)
            {
                linear_velocity = 0;
                angular_velocity = 0;
                left_reverse_walk_edge_status = LEFT_REVERSE_EDGE_GOSTR_BYPASS_RWE_X;
                break;
            }
            if (obstacleSignal == front_obstacle || obstacleSignal == left_obstacle || obstacleSignal == right_obstacle)
            {
                linear_velocity = 0;
                angular_velocity = 0;
                left_reverse_walk_edge_status = LEFT_REVERSE_EDGE_TURN_CCLOCK_TARGET_YAW_MORE_ABS45_DETAL_YAW_MORE_30_COLLISION_RWE;
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
                left_reverse_walk_edge_status = LEFT_REVERSE_EDGE_TURN_CCLOCK_TARGET_YAW_MORE_ABS45_DETAL_YAW_MORE_30_RWE;
                break;
            }
            break;
        case LEFT_REVERSE_EDGE_BOW_CONTINUE_RWE                                               :
            linear_velocity = 0;
            angular_velocity = 0;
            complete_flag = 1;
            left_reverse_walk_edge_status = 0;
            break;
        case LEFT_REVERSE_EDGE_RETURN_ORIGIN_RWE                                              :
            linear_velocity = 0;
            angular_velocity = 0;
            complete_flag = 3;
            left_reverse_walk_edge_status = 0;
            break;
    }
    return complete_flag;
}

unsigned char LeftEdgeDilemma(POSE *current_pose,unsigned char obstacleSignal)
{
    
    unsigned char complete_flag = 0;
    Yaw = current_pose->orientation;
    
    switch(left_edge_dilemma_status)
    {
        case 0:
            last_position_x = current_pose->x + W;
            left_edge_dilemma_status = LEFT_DILEMMA_DELTA_X_MORE_ONE_THIRD_CLEANED_MAP_WIDTH_DILEMMA;
            break;
        case LEFT_DILEMMA_DELTA_X_MORE_ONE_THIRD_CLEANED_MAP_WIDTH_DILEMMA:
			if (my_abs(last_position_x - (current_pose->x + W)) > 1000) //if (my_abs(last_position_x - (current_pose->x + W)) > gridmap.Edge_length() / 3)
			{
				complete_flag = 1;
				left_edge_dilemma_status = 0;
				number = 0;
				break;
			}
			left_edge_dilemma_status = LEFT_DILEMMA_LOOP_TEN_NUM_DILEMMA;
			break;
        case LEFT_DILEMMA_LOOP_TEN_NUM_DILEMMA:
            if(number >4)
            {
                complete_flag = 1;
                left_edge_dilemma_status = 0;
                 number = 0;
                break;
            }
            left_edge_dilemma_status = LEFT_DILEMMA_YAW_MORE_ABS90_LESS_ABS176_CCLOCK_LESS_ABS90_MORE_ABS4_CLOCK_DILEMMA;
            break;
        case LEFT_DILEMMA_YAW_MORE_ABS90_LESS_ABS176_CCLOCK_LESS_ABS90_MORE_ABS4_CLOCK_DILEMMA:
            if (my_abs(Yaw / 100) > 90 && my_abs(Yaw / 100) < 175)
            {
                if (Yaw > 0)
                {
                    linear_velocity = 0;
                    angular_velocity = 5*turn_vel / 6;
                }
                else
                {
                    linear_velocity = 0;
                    angular_velocity = -5*turn_vel /6;
                }
            }
            else if (my_abs(Yaw / 100) < 90 && my_abs(Yaw / 100) > 5)
            {
                if (Yaw > 0)
                {
                    linear_velocity = 0;
                    angular_velocity = -5*turn_vel/6;
                }
                else
                {
                    linear_velocity = 0;
                    angular_velocity = 5*turn_vel /6;
                }
            }
            else
            {
                linear_velocity = 200; // leave dilemma speed
                angular_velocity = 0;
            }
            left_edge_dilemma_status = LEFT_DILEMMA_GOSTR_DILEMMA;
            break;
        case LEFT_DILEMMA_GOSTR_DILEMMA:
            if (right_obstacle == obstacleSignal || front_obstacle == obstacleSignal || left_obstacle == obstacleSignal)
            {
                left_edge_dilemma_status = LEFT_DILEMMA_GOSTR_COLLISION_DILEMMA;
                break;
            }
            left_edge_dilemma_status = LEFT_DILEMMA_DELTA_X_MORE_ONE_THIRD_CLEANED_MAP_WIDTH_DILEMMA;
            break;
        case LEFT_DILEMMA_GOSTR_COLLISION_DILEMMA:
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
                if (my_abs(Yaw / 100) > 90)
                {
                    left_edge_dilemma_status = LEFT_DILEMMA_COLLISION_YAW_MORE_ABS90_DILEMMA;
                    break;
                }
                else
                {
                    left_edge_dilemma_status = LEFT_DILEMMA_COLLISION_YAW_LESS_ABS90_DILEMMA;
                    break;
                }
            }
            break;

        case LEFT_DILEMMA_COLLISION_YAW_MORE_ABS90_DILEMMA:
            left_edge_dilemma_status = LEFT_DILEMMA_CCLOCK_TARGET_YAW_LESS_ABS90_DILEMMA;
            break;
        case LEFT_DILEMMA_CCLOCK_TARGET_YAW_LESS_ABS90_DILEMMA:
            if (my_abs(Yaw / 100) < 90)
            {
                last_position_y = current_pose->y;
                last_position_x = current_pose->x + W;
                left_edge_dilemma_status = LEFT_DILEMMA_GOSTR_CYM_DILEMMA;
                break;
            }
            if (obstacleSignal == front_obstacle || obstacleSignal == left_obstacle || obstacleSignal == right_obstacle)
            {
                left_edge_dilemma_status =  LEFT_DILEMMA_CCLOCK_TARGET_YAW_LESS_ABS90_COLLISION_DILEMMA;
            }

            linear_velocity = 0;
            angular_velocity = turn_vel;
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
                left_edge_dilemma_status = LEFT_DILEMMA_CCLOCK_TARGET_YAW_LESS_ABS90_DILEMMA;
            }
            break;
        case LEFT_DILEMMA_GOSTR_CYM_DILEMMA:
            if (obstacleSignal == front_obstacle || obstacleSignal == right_obstacle || obstacleSignal == left_obstacle)
            {
                number++;
                left_edge_dilemma_status  = LEFT_DILEMMA_GOSTR_COLLISION_CYM_DILEMMA;
                break;
            }
            if (my_abs(last_position_y - current_pose->y) > lateral_move_distance / 3)
            {
                left_edge_dilemma_status  = LEFT_DILEMMA_GOSTR_DELTA_Y_MORE_LATERAL_DIS_CYM_DILEMMA;
                break;
            }
            linear_velocity = long_stra_vel;
            angular_velocity = 0;
            break;
        case LEFT_DILEMMA_GOSTR_COLLISION_CYM_DILEMMA:
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
                left_edge_dilemma_status  = LEFT_DILEMMA_CCLOCK_TARGET_YAW_LESS_ABS3_DILEMMA;
                break;
            }
            break;

        case LEFT_DILEMMA_GOSTR_DELTA_Y_MORE_LATERAL_DIS_CYM_DILEMMA:
            left_edge_dilemma_status  = LEFT_DILEMMA_CCLOCK_TARGET_YAW_LESS_ABS3_DILEMMA;
            break;
        case LEFT_DILEMMA_CCLOCK_TARGET_YAW_LESS_ABS3_DILEMMA:
            if (my_abs(Yaw / 100) < 5)
            {
                linear_velocity = 0;
                angular_velocity = 0;
                left_edge_dilemma_status = LEFT_DILEMMA_DELTA_X_MORE_ONE_THIRD_CLEANED_MAP_WIDTH_DILEMMA;
                break;
            }
            if (right_obstacle == obstacleSignal || front_obstacle == obstacleSignal || left_obstacle == obstacleSignal)
            {
                left_edge_dilemma_status = LEFT_DILEMMA_CCLOCK_TARGET_YAW_LESS_ABS3_COLLISION_DILEMMA;
                break;
            }
            linear_velocity = 0;
            angular_velocity = turn_vel;
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
                left_edge_dilemma_status = LEFT_DILEMMA_CCLOCK_TARGET_YAW_LESS_ABS3_DILEMMA;
            }
            break;

        case LEFT_DILEMMA_COLLISION_YAW_LESS_ABS90_DILEMMA:
            left_edge_dilemma_status = LEFT_DILEMMA_CLOCK_TARGET_YAW_MORE_ABS87_DILEMMA;
            break;
        case LEFT_DILEMMA_CLOCK_TARGET_YAW_MORE_ABS87_DILEMMA:
            if (my_abs(Yaw / 100) > 87)
            {
                last_position_y = current_pose->y;
                last_position_x = current_pose->x + W;
                left_edge_dilemma_status = LEFT_DILEMMA_GOSTR_CYL_DILEMMA;
                break;
            }
            if (obstacleSignal == front_obstacle || obstacleSignal == left_obstacle || obstacleSignal == right_obstacle)
            {
                left_edge_dilemma_status =  LEFT_DILEMMA_CLOCK_TARGET_YAW_MORE_ABS87_COLLISION_DILEMMA;
                break;
            }

            linear_velocity = 0;
            angular_velocity = -turn_vel;
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
                left_edge_dilemma_status = LEFT_DILEMMA_CLOCK_TARGET_YAW_MORE_ABS87_DILEMMA;
            }
            break;
        case LEFT_DILEMMA_GOSTR_CYL_DILEMMA:
            if (obstacleSignal == front_obstacle || obstacleSignal == right_obstacle || obstacleSignal == left_obstacle)
            {
                number++;
                left_edge_dilemma_status  = LEFT_DILEMMA_GOSTR_COLLISION_CYL_DILEMMA;
                break;
            }
            if (my_abs(last_position_y - current_pose->y) > lateral_move_distance / 3)
            {
                left_edge_dilemma_status  = LEFT_DILEMMA_GOSTR_GOSTR_DELTA_Y_MORE_LATERAL_DIS_CYL_DILEMMA;
                break;
            }
            linear_velocity = long_stra_vel;
            angular_velocity = 0;
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
                left_edge_dilemma_status = LEFT_DILEMMA_GOSTR_CYL_DILEMMA;
                break;
            }
            break;
        case LEFT_DILEMMA_GOSTR_GOSTR_DELTA_Y_MORE_LATERAL_DIS_CYL_DILEMMA:
            left_edge_dilemma_status  = LEFT_DILEMMA_CLOCK_TARGET_YAW_MORE_ABS173_DILEMMA;
            break;
        case LEFT_DILEMMA_CLOCK_TARGET_YAW_MORE_ABS173_DILEMMA:
            if (my_abs(Yaw / 100) > 175)
            {
                linear_velocity = 0;
                angular_velocity = 0;
                left_edge_dilemma_status = LEFT_DILEMMA_DELTA_X_MORE_ONE_THIRD_CLEANED_MAP_WIDTH_DILEMMA;
                break;
            }
            if (right_obstacle == obstacleSignal || front_obstacle == obstacleSignal || left_obstacle == obstacleSignal)
            {
                left_edge_dilemma_status = LEFT_DILEMMA_CLOCK_TARGET_YAW_MORE_ABS173_COLLISION_DILEMMA;
                break;
            }
            linear_velocity = 0;
            angular_velocity = -turn_vel;
            break;
        case LEFT_DILEMMA_CLOCK_TARGET_YAW_MORE_ABS173_COLLISION_DILEMMA:
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
                left_edge_dilemma_status = LEFT_DILEMMA_CLOCK_TARGET_YAW_MORE_ABS173_DILEMMA;
            }
            break;
    }
    return complete_flag;
}

unsigned char LeftReadyLeakingSweep(POSE *current_pose,unsigned char obstacleSignal)
{
   
    unsigned char complete_flag = 0;
    Yaw = current_pose->orientation;
	
    switch(left_ready_leaking_sweep_status)
    {
        case 0:
            left_ready_leaking_sweep_status = LEFT_LEAKING_SWEEP_COLLISION;
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
				if(my_abs(Yaw/100)>90)
				{
					left_ready_leaking_sweep_status = LEFT_LEAKING_SWEEP_YAW_MORE_ABS90;
					break;
				}
				else
				{
					left_ready_leaking_sweep_status = LEFT_LEAKING_SWEEP_YAW_OTHER;
					break;
				}
			}
			break;
        case LEFT_LEAKING_SWEEP_YAW_MORE_ABS90:
            left_ready_leaking_sweep_status = LEFT_LEAKING_SWEEP_CCLOCK_TARGET_YAW_LESS_ABS90;
            break;
        case LEFT_LEAKING_SWEEP_CCLOCK_TARGET_YAW_LESS_ABS90:
            if(my_abs(Yaw/100)<90){
                linear_velocity=0;
                angular_velocity=0;
                last_position_y=current_pose->y;
                left_ready_leaking_sweep_status = LEFT_LEAKING_SWEEP_GOSTRAIGHT_MORE;
                break;
            }
            if (obstacleSignal == front_obstacle || obstacleSignal == left_obstacle || obstacleSignal == right_obstacle)
            {
                linear_velocity=0;
                angular_velocity=0;
                left_ready_leaking_sweep_status = LEFT_LEAKING_SWEEP_CCLOCK_TARGER_YAW_LESS_ABS90_COLLISION;
            }
            linear_velocity=0;
            angular_velocity=turn_vel;
            break;
        case LEFT_LEAKING_SWEEP_CCLOCK_TARGER_YAW_LESS_ABS90_COLLISION  :
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
                left_ready_leaking_sweep_status = LEFT_LEAKING_SWEEP_CCLOCK_TARGET_YAW_LESS_ABS90;
                break;
            }
            break;
        case LEFT_LEAKING_SWEEP_GOSTRAIGHT_MORE                        :
            linear_velocity=long_stra_vel;
            angular_velocity=0;
            if(obstacleSignal==front_obstacle||obstacleSignal==left_obstacle||obstacleSignal==right_obstacle){
                left_ready_leaking_sweep_status = LEFT_LEAKING_SWEEP_CCLOCK_TARGER_YAW_LESS_ABS8;
                break;
            }
            if(my_abs(last_position_y-current_pose->y)>leakingsweep){
                left_ready_leaking_sweep_status = LEFT_LEAKING_SWEEP_CCLOCK_TARGER_YAW_LESS_ABS8;
                break;
            }
            break;
        case LEFT_LEAKING_SWEEP_CCLOCK_TARGER_YAW_LESS_ABS8:
            if(my_abs(Yaw/100)<5){
                linear_velocity=0;
                angular_velocity=0;
                left_ready_leaking_sweep_status = LEFT_LEAKING_SWEEP_COMPLETE;
                break;
            }
            if (obstacleSignal == front_obstacle || obstacleSignal == left_obstacle || obstacleSignal == right_obstacle)
            {
                linear_velocity=0;
                angular_velocity=0;
                left_ready_leaking_sweep_status = LEFT_LEAKING_SWEEP_CCLOCK_TARGER_YAW_LESS_ABS8_COLLISION;
            }
            linear_velocity=0;
            angular_velocity=turn_vel;
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
                left_ready_leaking_sweep_status = LEFT_LEAKING_SWEEP_CCLOCK_TARGER_YAW_LESS_ABS8;
                break;
            }
            break;
        case LEFT_LEAKING_SWEEP_YAW_OTHER:
            left_ready_leaking_sweep_status = LEFT_LEAKING_SWEEP_CLOCK_TARGET_YAW_MORE_ABS90;
            break;
        case LEFT_LEAKING_SWEEP_CLOCK_TARGET_YAW_MORE_ABS90:
            if(my_abs(Yaw/100)>90){
                linear_velocity=0;
                angular_velocity=0;
                last_position_y=current_pose->y;
                left_ready_leaking_sweep_status = LEFT_LEAKING_SWEEP_GOSTRAIGHT_OTHER;
                break;
            }
            if (obstacleSignal == front_obstacle || obstacleSignal == left_obstacle || obstacleSignal == right_obstacle)
            {
                linear_velocity=0;
                angular_velocity=0;
                left_ready_leaking_sweep_status = LEFT_LEAKING_SWEEP_CLOCK_TARGET_YAW_MORE_ABS90_COLLISION;
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
                left_ready_leaking_sweep_status = LEFT_LEAKING_SWEEP_CLOCK_TARGET_YAW_MORE_ABS90;
                break;
            }
            break;
        case LEFT_LEAKING_SWEEP_GOSTRAIGHT_OTHER:
            linear_velocity=long_stra_vel;
            angular_velocity=0;
            if(obstacleSignal==front_obstacle||obstacleSignal==left_obstacle||obstacleSignal==right_obstacle){
                left_ready_leaking_sweep_status = LEFT_LEAKING_SWEEP_CLOCK_TARGET_YAW_MORE_ABS173;
                break;
            }
            if(my_abs(last_position_y-current_pose->y)>leakingsweep){
                left_ready_leaking_sweep_status = LEFT_LEAKING_SWEEP_CLOCK_TARGET_YAW_MORE_ABS173;
                break;
            }
            break;
        case LEFT_LEAKING_SWEEP_CLOCK_TARGET_YAW_MORE_ABS173:
            if(my_abs(Yaw/100)>175){
                linear_velocity=0;
                angular_velocity=0;
                left_ready_leaking_sweep_status = LEFT_LEAKING_SWEEP_COMPLETE;
                break;
            }
            if (obstacleSignal == front_obstacle || obstacleSignal == left_obstacle || obstacleSignal == right_obstacle)
            {
                linear_velocity=0;
                angular_velocity=0;
                left_ready_leaking_sweep_status = LEFT_LEAKING_SWEEP_CLOCK_TARGET_YAW_MOEE_ABS173_COLLISIION;
            }
            linear_velocity=0;
            angular_velocity=-turn_vel;
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
                left_ready_leaking_sweep_status = LEFT_LEAKING_SWEEP_CLOCK_TARGET_YAW_MORE_ABS173;
                break;
            }
            break;
        case LEFT_LEAKING_SWEEP_COMPLETE                               :
            left_ready_leaking_sweep_status = 0;
            complete_flag = 1;
            break;
    }
    return complete_flag;
}


////////////////////////////////////////////////////////////////////////////////////////////////////////
unsigned char LeftReturnOriginWorkStep(POSE *current_pose,unsigned char obstacleSignal)
{
	return 1;
}



//#################################################################################
//##############A* return origin function define###################################
//#################################################################################
unsigned char within(short x, short y)
{
    return (x >= 0 && y >= 0 && x < AStar_Height && y < AStar_Width);
}

void initOpen(AStar_Open *q)
{
    q->length = 0;
}

void push(AStar_Open *q, AStar_Close cls[AStar_Height][AStar_Width], short x, short y, short g)
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

AStar_Close *shift(AStar_Open *q)
{
    return q->Array[--q->length];
}

void initClose(AStar_Close cls[AStar_Height][AStar_Width], short sx, short sy, short dx, short dy)
{
    short i, j;
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

void initGraph(bool maze[AStar_Height][AStar_Width], short sx, short sy, short dx, short dy)
{
    short i, j;
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
            AStar_graph[i][j].value = maze[i][j];
            AStar_graph[i][j].reachable = (AStar_graph[i][j].value == AStar_Reachable);
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

short astar(void)
{
    short i, curX, curY, surX, surY;
    short surG;
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

AStar_Close *getShortest(void)
{
    short result = astar();
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

short printShortest(void)
{
    AStar_Close *p;
    short step = 0;
	short previous_x = -3, previous_y = -3;
    p = getShortest();
    AStar_start = p;
    if (!p)
    {
        return 0;
    }
    else
    {
        
        while (p->from)
        {
            AStar_graph[p->cur->x][p->cur->y].value = AStar_Pass;
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
            //cout << plansteps[motionSteps - 1] << endl;
            //cout << '(' << p->cur->x << ',' << p->cur->y << ')' << endl;
            maze[p->cur->x][p->cur->y] = 1;
            previous_x = p->cur->x;
            previous_y = p->cur->y;
            p = p->from;
            step++;
        }
        AStar_graph[AStar_srcX][AStar_srcY].value = AStar_Source;
        AStar_graph[AStar_dstX][AStar_dstY].value = AStar_Destination;
        return step;
    }
}

unsigned char AStarReturnOrigin(POSE *current_pose, unsigned char obstacleSignal)
{
    unsigned char complete_flag = 0;
    int current_pose_x = current_pose->x;
    int current_pose_y = current_pose->y;
    bool canmove = true;
    int robot_x = current_pose_x;
    int robot_y = current_pose_y;
    int goal_robot_x = 0;
    int goal_robot_y = 0;
    short jk = 0,ij=0;
	short i = 0,j=0,k=0;
    short firsttrap = 0;
    short secondtrap = 0;
    short thirdtrap = 0;
    short forthtrap = 0;
    short astar_underboundary;
    short astar_onboundary;
    short leftboundary = 0;
    short rightboundary = 0;
    bool end_x = false;
    short a_map_x = (robot_x + MAPHEIGHT / 2) / (ZoomMultiple * GRIDWIDTH);
    short a_map_y = (robot_y + MAPHEIGHT / 2) / (ZoomMultiple * GRIDWIDTH);
    short goal_map_x = (goal_robot_x + MAPHEIGHT / 2) / (ZoomMultiple * GRIDWIDTH);
    short goal_map_y = (goal_robot_y + MAPHEIGHT / 2) / (ZoomMultiple * GRIDWIDTH);

    for (i = 0; i < 4; i++)
    {
        jk = i > 0 ? 1 : 0;
        for (int j = 0; j < GRIDWIDTH; j++)
        {
            for (int k = 0; k < GRIDWIDTH; k++)
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

        for (int j = GRIDWIDTH - 1; j >= 0; j--)
        {
            for (int k = 0; k < GRIDWIDTH; k++)
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
            for (int k = 0; k < GRIDWIDTH; k++)
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
                            for ( jk = 0; jk < ZoomMultiple; jk++)
                            {
                                if (gridmap.map[ZoomMultiple * ij + jk + i][ZoomMultiple * k + jk] == 0)
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
    initGraph(maze, a_map_x, a_map_x, goal_map_x, goal_map_x);
    AStar_shortestep = printShortest();
    return complete_flag;
}

unsigned char AStarNotMotionReturnOrigin(POSE *current_pose, unsigned char obstacleSignal)
{
    unsigned char complete_flag = 0;
    Yaw = current_pose->orientation;
    switch (a_star_not_motion)
    {
    case 0:
        temporary_wheel_pulse_r = wheel_pulse_r;
        a_star_not_motion = START_PLAN_ASTAR_MOTION_GOSTR_RETURN;
        break;
    case START_PLAN_ASTAR_MOTION_GOSTR_RETURN:
        if (obstacleSignal != none_obstacle)
        {
            a_star_not_motion = COLLISION_START_PLAN_ASTAR_MOTION_GOSTR_RETURN;
            break;
        }
        if (my_abs(wheel_pulse_r - temporary_wheel_pulse_r) > 958)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            a_star_not_motion = 0;
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
            a_star_not_motion = TURN_CLOCK_START_PLAN_ASTAR_MOTION_GOSTR_RETURN;
        }
        break;
    case TURN_CLOCK_START_PLAN_ASTAR_MOTION_GOSTR_RETURN:
        if (my_abs(temporary_yaw - Yaw / 100) > 15)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            a_star_not_motion = START_PLAN_ASTAR_MOTION_GOSTR_RETURN;
            break;
        }
        if (obstacleSignal != none_obstacle)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            a_star_not_motion = COLLISION_TURN_CLOCK_START_PLAN_ASTAR_MOTION_GOSTR_RETURN;
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
            a_star_not_motion = TURN_CLOCK_START_PLAN_ASTAR_MOTION_GOSTR_RETURN;
        }
        break;
    }
    return complete_flag;
}

unsigned char AStarMotionReturnOrigin(POSE *current_pose, unsigned char obstacleSignal)
{
    unsigned char complete_flag = 0;
    Yaw = current_pose->orientation;
    //cout << "AStarMotionReturnOrigin................." << endl;
    switch (a_star_motion_return_origin)
    {
    case 0:
        a_star_motion_return_origin = ASTAR_MOTION_GOSTR_RETURN;
        break;
    case ASTAR_MOTION_GOSTR_RETURN:
        if (my_abs(current_pose->x) > return_origin_distance || my_abs(current_pose->y) > return_origin_distance)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            a_star_motion_return_origin = PLAN_ASTAR_MOTION_GOSTR_RETURN;
            break;
        }
        else
        {
            linear_velocity = 0;
            angular_velocity = 0;
            complete_flag = 1;
            a_star_motion_return_origin = A_STAR_COMPLETED;
            break;
        }
    case PLAN_ASTAR_MOTION_GOSTR_RETURN:
        if (motionSteps == 0)
        {
            AStarMotionNumber++;
            if (AStarMotionNumber > 10)
            {
                linear_velocity = 0;
                angular_velocity = 0;
                complete_flag = 4;
            }
            else
            {
                complete_flag = 3;
            }
            break;
        }
        if (motionSteps > 0)
        {
            AStarMotionNumber = 0;
            a_star_motion_return_origin = DIRECT_START_PLAN_ASTAR_MOTION_GOSTR_RETURN;
            break;
        }
        break;
    case DIRECT_START_PLAN_ASTAR_MOTION_GOSTR_RETURN:
        if (my_abs(current_pose->x) < return_origin_distance && my_abs(current_pose->y) < return_origin_distance)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            a_star_motion_return_origin = A_STAR_COMPLETED;
            complete_flag = 1;
            break;
        }
        if (startMotionStep >= motionSteps)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            a_star_motion_return_origin = A_STAR_COMPLETED;
            complete_flag = 1;
            break;
        }
        if (plansteps[startMotionStep] == 'a')
        {
            a_star_motion_return_origin = A_DIRECT_START_PLAN_ASTAR_MOTION_GOSTR_RETURN;
        }
        else if (plansteps[startMotionStep] == 'b')
        {
            a_star_motion_return_origin = B_DIRECT_START_PLAN_ASTAR_MOTION_GOSTR_RETURN;
        }
        else if (plansteps[startMotionStep] == 'c')
        {
            a_star_motion_return_origin = C_DIRECT_START_PLAN_ASTAR_MOTION_GOSTR_RETURN;
        }
        else if (plansteps[startMotionStep] == 'd')
        {
            a_star_motion_return_origin = D_DIRECT_START_PLAN_ASTAR_MOTION_GOSTR_RETURN;
        }
        else if (plansteps[startMotionStep] == 'e')
        {
            a_star_motion_return_origin = E_DIRECT_START_PLAN_ASTAR_MOTION_GOSTR_RETURN;
        }
        else if (plansteps[startMotionStep] == 'f')
        {
            a_star_motion_return_origin = F_DIRECT_START_PLAN_ASTAR_MOTION_GOSTR_RETURN;
        }
        else if (plansteps[startMotionStep] == 'g')
        {
            a_star_motion_return_origin = G_DIRECT_START_PLAN_ASTAR_MOTION_GOSTR_RETURN;
        }
        else if (plansteps[startMotionStep] == 'h')
        {
            a_star_motion_return_origin = H_DIRECT_START_PLAN_ASTAR_MOTION_GOSTR_RETURN;
        }
        else
        {
        }
        linear_velocity = long_stra_vel;
        angular_velocity = 0;
        startMotionStep++;
        break;
    case A_DIRECT_START_PLAN_ASTAR_MOTION_GOSTR_RETURN:
        if (my_abs(Yaw / 100) <= 45 || (Yaw / 100 > 45 && Yaw / 100 < 130))
        {
            linear_velocity = 0;
            angular_velocity = 0;
            a_star_motion_return_origin = LESS_45_A_DIRECT_START_PLAN_ASTAR_MOTION_GOSTR_RETURN;
        }
        else if (my_abs(Yaw / 100) > 140 || (Yaw / 100 >= -140 && Yaw / 100 < -45))
        {
            linear_velocity = 0;
            angular_velocity = 0;
            a_star_motion_return_origin = MORE_45_A_DIRECT_START_PLAN_ASTAR_MOTION_GOSTR_RETURN;
        }
        else
        {
            temporary_wheel_pulse_l = wheel_pulse_l;
            a_star_motion_return_origin = GO_A_DIRECT_START_PLAN_ASTAR_MOTION_GOSTR_RETURN;
        }
        break;
    case LESS_45_A_DIRECT_START_PLAN_ASTAR_MOTION_GOSTR_RETURN:
        if (Yaw / 100 < 138 && Yaw / 100 > 132)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            temporary_wheel_pulse_l = wheel_pulse_l;
            a_star_motion_return_origin = GO_A_DIRECT_START_PLAN_ASTAR_MOTION_GOSTR_RETURN;
            break;
        }
        if (obstacleSignal != none_obstacle)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            a_star_motion_return_origin = COLLISION_LESS_45_A_DIRECT_START_PLAN_ASTAR_MOTION_GOSTR_RETURN;
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
            a_star_motion_return_origin = LESS_45_A_DIRECT_START_PLAN_ASTAR_MOTION_GOSTR_RETURN;
        }
        break;
    case MORE_45_A_DIRECT_START_PLAN_ASTAR_MOTION_GOSTR_RETURN:
        if (Yaw / 100 < 138 && Yaw / 100 > 132)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            temporary_wheel_pulse_l = wheel_pulse_l;
            a_star_motion_return_origin = GO_A_DIRECT_START_PLAN_ASTAR_MOTION_GOSTR_RETURN;
            break;
        }
        if (obstacleSignal != none_obstacle)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            a_star_motion_return_origin = COLLISION_MORE_45_A_DIRECT_START_PLAN_ASTAR_MOTION_GOSTR_RETURN;
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
            a_star_motion_return_origin = MORE_45_A_DIRECT_START_PLAN_ASTAR_MOTION_GOSTR_RETURN;
        }
        break;
    case GO_A_DIRECT_START_PLAN_ASTAR_MOTION_GOSTR_RETURN:
        if (obstacleSignal != none_obstacle)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            a_star_motion_return_origin = GO_A_DIRECT_START_PLAN_ASTAR_MOTION_GOSTR_RETURN;
            complete_flag = 2;
            break;
        }
        if (wheel_pulse_l - temporary_wheel_pulse_l > 1255)
        {
            a_star_motion_return_origin = DIRECT_START_PLAN_ASTAR_MOTION_GOSTR_RETURN;
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
            a_star_motion_return_origin = LESS_45_B_DIRECT_START_PLAN_ASTAR_MOTION_GOSTR_RETURN;
        }
        else if (my_abs(Yaw / 100) < 175 && Yaw / 100 < 0)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            a_star_motion_return_origin = MORE_45_B_DIRECT_START_PLAN_ASTAR_MOTION_GOSTR_RETURN;
        }
        else
        {
            temporary_wheel_pulse_l = wheel_pulse_l;
            a_star_motion_return_origin = GO_B_DIRECT_START_PLAN_ASTAR_MOTION_GOSTR_RETURN;
        }
        break;
    case LESS_45_B_DIRECT_START_PLAN_ASTAR_MOTION_GOSTR_RETURN:
        if (my_abs(Yaw / 100) > 178)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            temporary_wheel_pulse_l = wheel_pulse_l;
            a_star_motion_return_origin = GO_B_DIRECT_START_PLAN_ASTAR_MOTION_GOSTR_RETURN;
            break;
        }
        if (obstacleSignal != none_obstacle)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            a_star_motion_return_origin = COLLISION_LESS_45_B_DIRECT_START_PLAN_ASTAR_MOTION_GOSTR_RETURN;
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
            a_star_motion_return_origin = LESS_45_B_DIRECT_START_PLAN_ASTAR_MOTION_GOSTR_RETURN;
        }
        break;
    case MORE_45_B_DIRECT_START_PLAN_ASTAR_MOTION_GOSTR_RETURN:
        if (my_abs(Yaw / 100) > 178)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            temporary_wheel_pulse_l = wheel_pulse_l;
            a_star_motion_return_origin = GO_B_DIRECT_START_PLAN_ASTAR_MOTION_GOSTR_RETURN;
            break;
        }
        if (obstacleSignal != none_obstacle)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            a_star_motion_return_origin = COLLISION_MORE_45_B_DIRECT_START_PLAN_ASTAR_MOTION_GOSTR_RETURN;
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
            a_star_motion_return_origin = MORE_45_B_DIRECT_START_PLAN_ASTAR_MOTION_GOSTR_RETURN;
        }
        break;
    case GO_B_DIRECT_START_PLAN_ASTAR_MOTION_GOSTR_RETURN:
        if (obstacleSignal != none_obstacle)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            a_star_motion_return_origin = GO_B_DIRECT_START_PLAN_ASTAR_MOTION_GOSTR_RETURN;
            complete_flag = 2;
            break;
        }
        if (wheel_pulse_l - temporary_wheel_pulse_l > 908)
        {
            a_star_motion_return_origin = DIRECT_START_PLAN_ASTAR_MOTION_GOSTR_RETURN;
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
            a_star_motion_return_origin = LESS_45_C_DIRECT_START_PLAN_ASTAR_MOTION_GOSTR_RETURN;
        }
        else if (my_abs(Yaw / 100) <= 45 || (Yaw / 100 > -130 && Yaw / 100 < -45))
        {
            linear_velocity = 0;
            angular_velocity = 0;
            a_star_motion_return_origin = MORE_45_C_DIRECT_START_PLAN_ASTAR_MOTION_GOSTR_RETURN;
        }
        else
        {
            temporary_wheel_pulse_l = wheel_pulse_l;
            a_star_motion_return_origin = GO_B_DIRECT_START_PLAN_ASTAR_MOTION_GOSTR_RETURN;
        }
        break;
    case LESS_45_C_DIRECT_START_PLAN_ASTAR_MOTION_GOSTR_RETURN:
        if (Yaw / 100 > -138 && Yaw / 100 < -132)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            temporary_wheel_pulse_l = wheel_pulse_l;
            a_star_motion_return_origin = GO_C_DIRECT_START_PLAN_ASTAR_MOTION_GOSTR_RETURN;
            break;
        }
        if (obstacleSignal != none_obstacle)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            a_star_motion_return_origin = COLLISION_LESS_45_C_DIRECT_START_PLAN_ASTAR_MOTION_GOSTR_RETURN;
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
            a_star_motion_return_origin = LESS_45_C_DIRECT_START_PLAN_ASTAR_MOTION_GOSTR_RETURN;
        }
        break;
    case MORE_45_C_DIRECT_START_PLAN_ASTAR_MOTION_GOSTR_RETURN:
        if (Yaw / 100 > -138 && Yaw / 100 < -132)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            temporary_wheel_pulse_l = wheel_pulse_l;
            a_star_motion_return_origin = GO_C_DIRECT_START_PLAN_ASTAR_MOTION_GOSTR_RETURN;
            break;
        }
        if (obstacleSignal != none_obstacle)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            a_star_motion_return_origin = COLLISION_MORE_45_C_DIRECT_START_PLAN_ASTAR_MOTION_GOSTR_RETURN;
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
            a_star_motion_return_origin = MORE_45_C_DIRECT_START_PLAN_ASTAR_MOTION_GOSTR_RETURN;
        }
        break;
    case GO_C_DIRECT_START_PLAN_ASTAR_MOTION_GOSTR_RETURN:
        if (obstacleSignal != none_obstacle)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            a_star_motion_return_origin = GO_C_DIRECT_START_PLAN_ASTAR_MOTION_GOSTR_RETURN;
            complete_flag = 2;
            break;
        }
        if (wheel_pulse_l - temporary_wheel_pulse_l > 1255)
        {
            a_star_motion_return_origin = DIRECT_START_PLAN_ASTAR_MOTION_GOSTR_RETURN;
            break;
        }
        linear_velocity = long_stra_vel;
        angular_velocity = 0;
        break;
    case D_DIRECT_START_PLAN_ASTAR_MOTION_GOSTR_RETURN:
        if (my_abs(Yaw / 100) > 95)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            a_star_motion_return_origin = LESS_45_D_DIRECT_START_PLAN_ASTAR_MOTION_GOSTR_RETURN;
        }
        else if (my_abs(Yaw / 100) < 85 || (Yaw / 100 >= 85 && Yaw / 100 <= 95))
        {
            linear_velocity = 0;
            angular_velocity = 0;
            a_star_motion_return_origin = MORE_45_D_DIRECT_START_PLAN_ASTAR_MOTION_GOSTR_RETURN;
        }
        else
        {
            temporary_wheel_pulse_l = wheel_pulse_l;
            a_star_motion_return_origin = GO_D_DIRECT_START_PLAN_ASTAR_MOTION_GOSTR_RETURN;
        }
        break;
    case LESS_45_D_DIRECT_START_PLAN_ASTAR_MOTION_GOSTR_RETURN:
        if (Yaw / 100 > -138 && Yaw / 100 < -132)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            temporary_wheel_pulse_l = wheel_pulse_l;
            a_star_motion_return_origin = GO_D_DIRECT_START_PLAN_ASTAR_MOTION_GOSTR_RETURN;
            break;
        }
        if (obstacleSignal != none_obstacle)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            a_star_motion_return_origin = COLLISION_LESS_45_D_DIRECT_START_PLAN_ASTAR_MOTION_GOSTR_RETURN;
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
            a_star_motion_return_origin = LESS_45_D_DIRECT_START_PLAN_ASTAR_MOTION_GOSTR_RETURN;
        }
        break;
    case MORE_45_D_DIRECT_START_PLAN_ASTAR_MOTION_GOSTR_RETURN:
        if (Yaw / 100 > -138 && Yaw / 100 < -132)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            temporary_wheel_pulse_l = wheel_pulse_l;
            a_star_motion_return_origin = GO_D_DIRECT_START_PLAN_ASTAR_MOTION_GOSTR_RETURN;
            break;
        }
        if (obstacleSignal != none_obstacle)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            a_star_motion_return_origin = COLLISION_MORE_45_D_DIRECT_START_PLAN_ASTAR_MOTION_GOSTR_RETURN;
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
            a_star_motion_return_origin = MORE_45_D_DIRECT_START_PLAN_ASTAR_MOTION_GOSTR_RETURN;
        }
        break;
    case GO_D_DIRECT_START_PLAN_ASTAR_MOTION_GOSTR_RETURN:
        if (obstacleSignal != none_obstacle)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            a_star_motion_return_origin = GO_D_DIRECT_START_PLAN_ASTAR_MOTION_GOSTR_RETURN;
            complete_flag = 2;
            break;
        }
        if (wheel_pulse_l - temporary_wheel_pulse_l > 908)
        {
            a_star_motion_return_origin = DIRECT_START_PLAN_ASTAR_MOTION_GOSTR_RETURN;
            break;
        }
        linear_velocity = long_stra_vel;
        angular_velocity = 0;
        break;
    case E_DIRECT_START_PLAN_ASTAR_MOTION_GOSTR_RETURN:
        if (my_abs(Yaw / 100) >= 135 || (Yaw / 100 < -135 && Yaw / 100 > -50))
        {
            linear_velocity = 0;
            angular_velocity = 0;
            a_star_motion_return_origin = LESS_45_E_DIRECT_START_PLAN_ASTAR_MOTION_GOSTR_RETURN;
        }
        else if (my_abs(Yaw / 100) < 40 || (Yaw / 100 >= 40 && Yaw / 100 < 135))
        {
            linear_velocity = 0;
            angular_velocity = 0;
            a_star_motion_return_origin = MORE_45_E_DIRECT_START_PLAN_ASTAR_MOTION_GOSTR_RETURN;
        }
        else
        {
            temporary_wheel_pulse_l = wheel_pulse_l;
            a_star_motion_return_origin = GO_E_DIRECT_START_PLAN_ASTAR_MOTION_GOSTR_RETURN;
        }
        break;
    case LESS_45_E_DIRECT_START_PLAN_ASTAR_MOTION_GOSTR_RETURN:
        if (Yaw / 100 < -42 && Yaw / 100 > -48)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            temporary_wheel_pulse_l = wheel_pulse_l;
            a_star_motion_return_origin = GO_E_DIRECT_START_PLAN_ASTAR_MOTION_GOSTR_RETURN;
            break;
        }
        if (obstacleSignal != none_obstacle)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            a_star_motion_return_origin = COLLISION_LESS_45_E_DIRECT_START_PLAN_ASTAR_MOTION_GOSTR_RETURN;
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
            a_star_motion_return_origin = LESS_45_E_DIRECT_START_PLAN_ASTAR_MOTION_GOSTR_RETURN;
        }
        break;
    case MORE_45_E_DIRECT_START_PLAN_ASTAR_MOTION_GOSTR_RETURN:
        if (Yaw / 100 < -42 && Yaw / 100 > -48)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            temporary_wheel_pulse_l = wheel_pulse_l;
            a_star_motion_return_origin = GO_E_DIRECT_START_PLAN_ASTAR_MOTION_GOSTR_RETURN;
            break;
        }
        if (obstacleSignal != none_obstacle)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            a_star_motion_return_origin = COLLISION_MORE_45_E_DIRECT_START_PLAN_ASTAR_MOTION_GOSTR_RETURN;
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
            a_star_motion_return_origin = MORE_45_E_DIRECT_START_PLAN_ASTAR_MOTION_GOSTR_RETURN;
        }
        break;
    case GO_E_DIRECT_START_PLAN_ASTAR_MOTION_GOSTR_RETURN:
        if (obstacleSignal != none_obstacle)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            a_star_motion_return_origin = GO_E_DIRECT_START_PLAN_ASTAR_MOTION_GOSTR_RETURN;
            complete_flag = 2;
            break;
        }
        if (wheel_pulse_l - temporary_wheel_pulse_l > 1255)
        {
            a_star_motion_return_origin = DIRECT_START_PLAN_ASTAR_MOTION_GOSTR_RETURN;
            break;
        }
        linear_velocity = long_stra_vel;
        angular_velocity = 0;
        break;

    case F_DIRECT_START_PLAN_ASTAR_MOTION_GOSTR_RETURN:
        if (my_abs(Yaw / 100) > 5 && Yaw / 100 < 0)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            a_star_motion_return_origin = LESS_45_F_DIRECT_START_PLAN_ASTAR_MOTION_GOSTR_RETURN;
        }
        else if (my_abs(Yaw / 100) > 5 && Yaw / 100 > 0)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            a_star_motion_return_origin = MORE_45_F_DIRECT_START_PLAN_ASTAR_MOTION_GOSTR_RETURN;
        }
        else
        {
            temporary_wheel_pulse_l = wheel_pulse_l;
            a_star_motion_return_origin = GO_F_DIRECT_START_PLAN_ASTAR_MOTION_GOSTR_RETURN;
        }
        break;
    case LESS_45_F_DIRECT_START_PLAN_ASTAR_MOTION_GOSTR_RETURN:
        if (my_abs(Yaw / 100) < 3)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            temporary_wheel_pulse_l = wheel_pulse_l;
            a_star_motion_return_origin = GO_F_DIRECT_START_PLAN_ASTAR_MOTION_GOSTR_RETURN;
            break;
        }
        if (obstacleSignal != none_obstacle)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            a_star_motion_return_origin = COLLISION_LESS_45_F_DIRECT_START_PLAN_ASTAR_MOTION_GOSTR_RETURN;
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
            a_star_motion_return_origin = LESS_45_F_DIRECT_START_PLAN_ASTAR_MOTION_GOSTR_RETURN;
        }
        break;
    case MORE_45_F_DIRECT_START_PLAN_ASTAR_MOTION_GOSTR_RETURN:
        if (my_abs(Yaw / 100) < 3)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            temporary_wheel_pulse_l = wheel_pulse_l;
            a_star_motion_return_origin = GO_F_DIRECT_START_PLAN_ASTAR_MOTION_GOSTR_RETURN;
            break;
        }
        if (obstacleSignal != none_obstacle)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            a_star_motion_return_origin = COLLISION_MORE_45_F_DIRECT_START_PLAN_ASTAR_MOTION_GOSTR_RETURN;
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
            a_star_motion_return_origin = MORE_45_F_DIRECT_START_PLAN_ASTAR_MOTION_GOSTR_RETURN;
        }
        break;
    case GO_F_DIRECT_START_PLAN_ASTAR_MOTION_GOSTR_RETURN:
        if (obstacleSignal != none_obstacle)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            a_star_motion_return_origin = GO_F_DIRECT_START_PLAN_ASTAR_MOTION_GOSTR_RETURN;
            complete_flag = 2;
            break;
        }
        if (wheel_pulse_l - temporary_wheel_pulse_l > 908)
        {
            a_star_motion_return_origin = DIRECT_START_PLAN_ASTAR_MOTION_GOSTR_RETURN;
            break;
        }
        linear_velocity = long_stra_vel;
        angular_velocity = 0;
        break;
    case G_DIRECT_START_PLAN_ASTAR_MOTION_GOSTR_RETURN:
        if (my_abs(Yaw / 100) <= 40 || (Yaw / 100 < -40 && Yaw / 100 > -135))
        {
            linear_velocity = 0;
            angular_velocity = 0;
            a_star_motion_return_origin = LESS_45_G_DIRECT_START_PLAN_ASTAR_MOTION_GOSTR_RETURN;
        }
        else if (my_abs(Yaw / 100) >= 135 || (Yaw / 100 > 50 && Yaw / 100 < 135))
        {
            linear_velocity = 0;
            angular_velocity = 0;
            a_star_motion_return_origin = MORE_45_G_DIRECT_START_PLAN_ASTAR_MOTION_GOSTR_RETURN;
        }
        else
        {
            temporary_wheel_pulse_l = wheel_pulse_l;
            a_star_motion_return_origin = GO_G_DIRECT_START_PLAN_ASTAR_MOTION_GOSTR_RETURN;
        }
        break;
    case LESS_45_G_DIRECT_START_PLAN_ASTAR_MOTION_GOSTR_RETURN:
        if (Yaw / 100 < 48 && Yaw / 100 > 42)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            temporary_wheel_pulse_l = wheel_pulse_l;
            a_star_motion_return_origin = GO_G_DIRECT_START_PLAN_ASTAR_MOTION_GOSTR_RETURN;
            break;
        }
        if (obstacleSignal != none_obstacle)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            a_star_motion_return_origin = COLLISION_LESS_45_G_DIRECT_START_PLAN_ASTAR_MOTION_GOSTR_RETURN;
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
            a_star_motion_return_origin = LESS_45_G_DIRECT_START_PLAN_ASTAR_MOTION_GOSTR_RETURN;
        }
        break;
    case MORE_45_G_DIRECT_START_PLAN_ASTAR_MOTION_GOSTR_RETURN:
        if (Yaw / 100 < 48 && Yaw / 100 > 42)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            temporary_wheel_pulse_l = wheel_pulse_l;
            a_star_motion_return_origin = GO_G_DIRECT_START_PLAN_ASTAR_MOTION_GOSTR_RETURN;
            break;
        }
        if (obstacleSignal != none_obstacle)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            a_star_motion_return_origin = COLLISION_MORE_45_G_DIRECT_START_PLAN_ASTAR_MOTION_GOSTR_RETURN;
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
            a_star_motion_return_origin = MORE_45_G_DIRECT_START_PLAN_ASTAR_MOTION_GOSTR_RETURN;
        }
        break;
    case GO_G_DIRECT_START_PLAN_ASTAR_MOTION_GOSTR_RETURN:
        if (obstacleSignal != none_obstacle)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            a_star_motion_return_origin = GO_G_DIRECT_START_PLAN_ASTAR_MOTION_GOSTR_RETURN;
            complete_flag = 2;
            break;
        }
        if (wheel_pulse_l - temporary_wheel_pulse_l > 1255)
        {
            a_star_motion_return_origin = DIRECT_START_PLAN_ASTAR_MOTION_GOSTR_RETURN;
            break;
        }
        linear_velocity = long_stra_vel;
        angular_velocity = 0;
        break;
    case H_DIRECT_START_PLAN_ASTAR_MOTION_GOSTR_RETURN:
        if (my_abs(Yaw / 100) < 85)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            a_star_motion_return_origin = LESS_45_H_DIRECT_START_PLAN_ASTAR_MOTION_GOSTR_RETURN;
        }
        else if (my_abs(Yaw / 100) > 95 || (Yaw / 100 >= -95 && Yaw / 100 <= -85))
        {
            linear_velocity = 0;
            angular_velocity = 0;
            a_star_motion_return_origin = MORE_45_H_DIRECT_START_PLAN_ASTAR_MOTION_GOSTR_RETURN;
        }
        else
        {
            temporary_wheel_pulse_l = wheel_pulse_l;
            a_star_motion_return_origin = GO_H_DIRECT_START_PLAN_ASTAR_MOTION_GOSTR_RETURN;
        }
        break;
    case LESS_45_H_DIRECT_START_PLAN_ASTAR_MOTION_GOSTR_RETURN:
        if (Yaw / 100 > 87 && Yaw / 100 < 93)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            temporary_wheel_pulse_l = wheel_pulse_l;
            a_star_motion_return_origin = GO_H_DIRECT_START_PLAN_ASTAR_MOTION_GOSTR_RETURN;
            break;
        }
        if (obstacleSignal != none_obstacle)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            a_star_motion_return_origin = COLLISION_LESS_45_H_DIRECT_START_PLAN_ASTAR_MOTION_GOSTR_RETURN;
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
            a_star_motion_return_origin = LESS_45_H_DIRECT_START_PLAN_ASTAR_MOTION_GOSTR_RETURN;
        }
        break;
    case MORE_45_H_DIRECT_START_PLAN_ASTAR_MOTION_GOSTR_RETURN:
        if (Yaw / 100 > 87 && Yaw / 100 < 93)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            temporary_wheel_pulse_l = wheel_pulse_l;
            a_star_motion_return_origin = GO_H_DIRECT_START_PLAN_ASTAR_MOTION_GOSTR_RETURN;
            break;
        }
        if (obstacleSignal != none_obstacle)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            a_star_motion_return_origin = COLLISION_MORE_45_H_DIRECT_START_PLAN_ASTAR_MOTION_GOSTR_RETURN;
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
            a_star_motion_return_origin = MORE_45_H_DIRECT_START_PLAN_ASTAR_MOTION_GOSTR_RETURN;
        }
        break;
    case GO_H_DIRECT_START_PLAN_ASTAR_MOTION_GOSTR_RETURN:
        if (obstacleSignal != none_obstacle)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            a_star_motion_return_origin = GO_H_DIRECT_START_PLAN_ASTAR_MOTION_GOSTR_RETURN;
            complete_flag = 2;
            break;
        }
        if (wheel_pulse_l - temporary_wheel_pulse_l > 908)
        {
            a_star_motion_return_origin = DIRECT_START_PLAN_ASTAR_MOTION_GOSTR_RETURN;
            break;
        }
        linear_velocity = long_stra_vel;
        angular_velocity = 0;
        break;
    case A_STAR_COMPLETED:
        linear_velocity = 0;
        angular_velocity = 0;
        startMotionStep = 0;
        motionSteps = 0;
        AStarMotionNumber = 0;
        a_star_motion_return_origin = 0;
        break;
    }
    return complete_flag;
}

unsigned char AStarCollision(POSE *current_pose, unsigned char obstacleSignal)
{
    unsigned char complete_flag = 0;
    Yaw = current_pose->orientation;
    switch (a_star_collision)
    {
    case 0:
        a_star_collision = A_STAR_COLLISION;
        break;
    case A_STAR_COLLISION:
        log_debug("A_STAR_COLLISION_GOSTR");
        if (my_abs(current_pose->x) > return_origin_distance || my_abs(current_pose->y) > return_origin_distance)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            a_star_collision = START_A_STAR_COLLISION;
        }
        else
        {
            log_debug("A_STAR_COLLISION_COMPLETED");
            linear_velocity = 0;
            angular_velocity = 0;
            complete_flag = 2;
            a_star_collision = A_STAR_COLLISION_COMPLETED;
        }
        break;
    case START_A_STAR_COLLISION:
        if (obstacleSignal == left_obstacle)
        {
            markingobstacle = left_obstacle;
        }
        else if (obstacleSignal == right_obstacle)
        {
            markingobstacle = right_obstacle;
        }
        else
        {
            markingobstacle = front_obstacle;
        }
        a_star_collision = BACK_START_A_STAR_COLLISION;
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
            temporary_yaw = Yaw / 100;
            if (markingobstacle == left_obstacle)
            {
                a_star_collision = LEFT_OBSTACLE_START_A_STAR_COLLISION;
            }
            else if (markingobstacle == right_obstacle)
            {
                a_star_collision = RIGHT_OBSTACLE_START_A_STAR_COLLISION;
            }
            else
            {
                a_star_collision = FRONT_OBSTACLE_START_A_STAR_COLLISION;
            }
        }
        break;
    case LEFT_OBSTACLE_START_A_STAR_COLLISION:
        if (my_abs(temporary_yaw - Yaw / 100) > 5)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            a_star_collision = GO_LEFT_OBSTACLE_START_A_STAR_COLLISION;
            break;
        }
        if (obstacleSignal != none_obstacle)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            a_star_collision = COLLISION_LEFT_OBSTACLE_START_A_STAR_COLLISION;
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
            a_star_collision = LEFT_OBSTACLE_START_A_STAR_COLLISION;
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
            a_star_collision = BACK_START_A_STAR_COLLISION;
            break;
        }
        if (a_star_collision_total > 10)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            turn_start_update = 0;
            a_star_collision = RECALCULATE_A_STAR_COLLISION_COMPLETED;
            break;
        }
        if (my_abs(turn_start_x - current_pose->x) > side_backward_distance || my_abs(turn_start_y - current_pose->y) > side_backward_distance)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            turn_start_update = 0;
            a_star_collision = A_STAR_COLLISION_COMPLETED;
        }
        linear_velocity = long_stra_vel;
        angular_velocity = 0;
        break;
    case RIGHT_OBSTACLE_START_A_STAR_COLLISION:
        if (my_abs(temporary_yaw - Yaw / 100) > 5)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            a_star_collision = GO_RIGHT_OBSTACLE_START_A_STAR_COLLISION;
            break;
        }
        if (obstacleSignal != none_obstacle)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            a_star_collision = COLLISION_RIGHT_OBSTACLE_START_A_STAR_COLLISION;
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
            a_star_collision = RIGHT_OBSTACLE_START_A_STAR_COLLISION;
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
            a_star_collision = BACK_START_A_STAR_COLLISION;
            break;
        }
        if (a_star_collision_total > 10)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            turn_start_update = 0;
            a_star_collision = RECALCULATE_A_STAR_COLLISION_COMPLETED;
            break;
        }
        if (my_abs(turn_start_x - current_pose->x) > side_backward_distance || my_abs(turn_start_y - current_pose->y) > side_backward_distance)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            turn_start_update = 0;
            a_star_collision = A_STAR_COLLISION_COMPLETED;
        }
        break;
    case FRONT_OBSTACLE_START_A_STAR_COLLISION:
        if (my_abs(temporary_yaw - Yaw / 100) > 15)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            a_star_collision = GO_FRONT_OBSTACLE_START_A_STAR_COLLISION;
            break;
        }
        if (obstacleSignal != none_obstacle)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            a_star_collision = COLLISION_FRONT_OBSTACLE_START_A_STAR_COLLISION;
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
            a_star_collision = FRONT_OBSTACLE_START_A_STAR_COLLISION;
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
            a_star_collision = BACK_START_A_STAR_COLLISION;
            break;
        }
        if (a_star_collision_total > 5)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            turn_start_update = 0;
            a_star_collision = RECALCULATE_A_STAR_COLLISION_COMPLETED;
            break;
        }
        if (my_abs(turn_start_x - current_pose->x) > side_backward_distance || my_abs(turn_start_y - current_pose->y) > side_backward_distance)
        {
            linear_velocity = 0;
            angular_velocity = 0;
            turn_start_update = 0;
            a_star_collision = A_STAR_COLLISION_COMPLETED;
        }
        break;
    case RECALCULATE_A_STAR_COLLISION_COMPLETED:
        linear_velocity = 0;
        angular_velocity = 0;
        a_star_collision = 0;
        a_star_collision_total = 0;
        complete_flag = 1;
        break;
    case A_STAR_COLLISION_COMPLETED:
        linear_velocity = 0;
        angular_velocity = 0;
        a_star_collision = 0;
        a_star_collision_total = 0;
        complete_flag = 2;
        break;
    }
    return complete_flag;
}

///////////////////////////////////////////////////////////////////////////

//#################################################################################
//#############  return origin function define  ###################################
//#################################################################################
unsigned char RightReturnOrigin(POSE *current_pose, unsigned char obstacleSignal)
{
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
        if (my_abs(last_position_y - current_pose->y) > lateral_move_distance / 2 || my_abs(last_position_x - current_pose->x) > lateral_move_distance / 2)
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
        if (my_abs(last_position_y - current_pose->y) > lateral_move_distance / 2 || my_abs(last_position_x - current_pose->x) > lateral_move_distance / 2)
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
        if (my_abs(last_position_y - current_pose->y) > 2 * lateral_move_distance / 3 || my_abs(last_position_x - current_pose->x) > 2 * lateral_move_distance / 3)
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
        if (my_abs(last_position_y - current_pose->y) > lateral_move_distance / 2 || my_abs(last_position_x - current_pose->x) > lateral_move_distance / 2)
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
	
///////////////////////////////////////////////////////////////////////////






	
