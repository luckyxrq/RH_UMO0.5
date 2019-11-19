#include "bsp.h"
#include <math.h>


#define INT_COOR_X 250
#define INT_COOR_Y 250
#define ALL_CLEAN_COMPLETE 6

int right_running_step_status = 0;
int collision_right_rightrun_step_status = 0;
int collision_left_rightrun_step_status = 0;
int collision_front_rightrun_step_status = 0;
double linear_velocity = 0,angular_velocity = 0;
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


static CleanStrategyB cleanstrategy;

static double my_abs(double x){
    if (x<0){
        x= -x;
    }
    return x;}

	
static void log_debug(char* str)
{
    //########################
    DEBUG("%s",str);
    //########################
}
static void sendvelocity(double* linear_velocity,double* angular_velocity)
{
    short leftVelocity,rightVelocity;
    leftVelocity = (short)((0.5*(2**linear_velocity*0.001 - Deg2Rad(*angular_velocity)*WHEEL_LENGTH))* 1000);
    rightVelocity = (short)((0.5*(2**linear_velocity*0.001 + Deg2Rad(*angular_velocity)*WHEEL_LENGTH))* 1000);
    /*设定速度*/
    bsp_SetMotorSpeed(MotorLeft,bsp_MotorSpeedMM2Pulse(leftVelocity));
    bsp_SetMotorSpeed(MotorRight,bsp_MotorSpeedMM2Pulse(rightVelocity));

}


void bsp_StartUpdateCleanStrategyB(void)
{
	cleanstrategy.work_step_status = 0;
	cleanstrategy.right_running_complete  = 0;
	cleanstrategy.right_return_origin_complete = 0;
	cleanstrategy.left_running_complete = 0;
	cleanstrategy.left_return_origin_complete = 0;
	
	cleanstrategy.action = 0 ;
	cleanstrategy.delay = 0 ;
	cleanstrategy.isRunning = true;
	
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
	
}



void bsp_CleanStrategyUpdateB(int robotX,int robotY,double robotTheta, unsigned char obstacleSignal, int wheel_pulse_l, int wheel_pulse_r, unsigned char IRSensorData[])
{
	POSE current_pose;
	
	current_pose.x = INT_COOR_X + robotX;
	current_pose.y = INT_COOR_Y + robotY;
	current_pose.orientation = robotTheta;
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


////////////////////////////////////////////////////////////////////////////////////////////////////////
uint8_t clean_strategy(POSE *current_pose,unsigned char obstacleSignal)
{
    switch (cleanstrategy.work_step_status)
    {
        case RIGHTRUNNING_WORK_SETP:
            if (cleanstrategy.right_running_complete)
            {
                cleanstrategy.work_step_status  = RIGHTRETURN_ORIGIN_WORK_SETP;
            }else
            {
                 RightRuningWorkStep(current_pose,obstacleSignal);
            }
            break;

        case RIGHTRETURN_ORIGIN_WORK_SETP:
            if (cleanstrategy.right_return_origin_complete)
            {
                cleanstrategy.work_step_status  = LEFTRUNNING_WORK_SETP;
            }else
            {
                RightReturnOriginWorkStep(current_pose, obstacleSignal);
            }
            break;

        case LEFTRUNNING_WORK_SETP:
            if(cleanstrategy.left_running_complete)
            {
                cleanstrategy.work_step_status = LEFTRETURN_ORIGIN_WORK_SETP;
            }else
            {
                LeftRunningWorkStep(current_pose,obstacleSignal);
            }
            break;
        
        case LEFTRETURN_ORIGIN_WORK_SETP:
            if(cleanstrategy.left_return_origin_complete)
            {
                cleanstrategy.work_step_status = ALL_FINSHED_WORK_SETP;
            }else
            {
                LeftReturnOriginWorkStep(current_pose,obstacleSignal); 
            }
            break;
        case ALL_FINSHED_WORK_SETP:
            return ALL_CLEAN_COMPLETE;//" clean complete"
        default:
            break;
        return 0;
        
    }
}





////////////////////////////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////////////////////////////
void RightRuningWorkStep(POSE *current_pose,unsigned char obstacleSignal)
{
    double Yaw;
	
    Yaw = Rad2Deg(current_pose->orientation);

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
                    linear_velocity = long_stra_vel / 2;
                    angular_velocity = turn_vel / 2;
                    break;
                }
                else
                {
                    linear_velocity = long_stra_vel / 2;
                    angular_velocity = -turn_vel / 2;
                    break;
                }
            }
            else if (my_abs(Yaw / 100) < 90 && my_abs(Yaw / 100) > 5)
            {
                log_debug("gostraight Corrected heading angle !");
                if (Yaw > 0)
                {
                    linear_velocity = long_stra_vel / 2;
                    angular_velocity = -turn_vel / 2;
                    break;
                }
                else
                {
                    linear_velocity = long_stra_vel / 2;
                    angular_velocity = turn_vel / 2;
                    break;
                }
            }
            else if(my_abs(current_pose->x) > W)
            {
                log_debug("current pose x arrived width max!");
                log_debug("ready goto FORWARD_BOUNDARY_RIGHTRUN_STEP");
                right_running_step_status = FORWARD_BOUNDARY_RIGHTRUN_STEP;

                break;
            }
            else if(my_abs(current_pose->y) > W)
            {
                log_debug("current pose y arrived width max!");

                break;
            }
            else if (right_obstacle == obstacleSignal)
            {
                linear_velocity = 0;
                angular_velocity = 0;
                //leakingsweep = gridmap.ReturnExtreme_point(Yaw / 100, obstacleSignal);
				leakingsweep = bsp_Right_ReturnExtreme_point(current_pose->x,current_pose->y,current_pose->orientation,obstacleSignal);
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
				leakingsweep = bsp_Right_ReturnExtreme_point(current_pose->x,current_pose->y,current_pose->orientation,obstacleSignal);
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
				leakingsweep = bsp_Right_ReturnExtreme_point(current_pose->x,current_pose->y,current_pose->orientation,obstacleSignal);
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
            ForwardBoundaryRightRunStep(current_pose,obstacleSignal);
            break;

        case COLLISION_RIGHT_RIGHTRUN_STEP:
            log_debug("collsion right right run step!");
            if(CollisionRightRightRunStep(current_pose,obstacleSignal))
            {
                 right_running_step_status = GOSTR_RIGHTRUN_STEP;
                 collision_right_rightrun_step_status = 0;
            }
            break;
        case COLLISION_LEFT_RIGHTRUN_STEP:
            log_debug("collsion right left run step!");
            if(CollisionLeftRightRunStep(current_pose,obstacleSignal))
            {
                right_running_step_status = GOSTR_RIGHTRUN_STEP;
            }
            break;
        case COLLISION_FRONT_RIGHTRUN_STEP:
            log_debug("collsion front right run step!");
            if(CollisionFrontRightRunStep(current_pose,obstacleSignal))
            {
                right_running_step_status = GOSTR_RIGHTRUN_STEP;
            }
            break;
        case LEAKING_SWEEP_RIGHTRUN_STEP:
            log_debug("leaking sweep right run step!");
            if(RightReadyLeakingSweep(current_pose,obstacleSignal))
            {
                right_running_step_status = GOSTR_RIGHTRUN_STEP;
            }
            break;
        default:
            break;
    }
    sendvelocity(&linear_velocity,&angular_velocity);
}
////////////////////////////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////////////////////////////
unsigned char CollisionRightRightRunStep(POSE *current_pose,unsigned char obstacleSignal)
{
    double Yaw;
    unsigned char complete_flag = 0;

    Yaw = Rad2Deg(current_pose->orientation);
    switch(collision_right_rightrun_step_status)
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
                collision_right_rightrun_step_status = TURN_CLOCK_TARGET_YAW_NEG27_COLLSION_CR_DRYL;
                break;
            }
            break;
        case TURN_CLOCK_TARGET_YAW_NEG27_COLLSION_CR_DRYL:
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
                collision_right_rightrun_step_status  = TURN_CLOCK_TARGET_YAW_NEG57_COLLSION_CR_DRYL;
                break;
            }
            break;
        case TURN_CLOCK_TARGET_YAW_NEG57_COLLSION_CR_DRYL:
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
                collision_right_rightrun_step_status = TURN_CLOCK_TARGET_YAW_NEG82_COLLSION_CR_DRYL;
                break;
            }
            break;
        case TURN_CLOCK_TARGET_YAW_NEG82_COLLSION_CR_DRYL:
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
            if(RightWalkEdge(current_pose,obstacleSignal))
            {
                collision_right_rightrun_step_status = COMPLETE_CR_DRYL;
                break;
            }
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
                collision_right_rightrun_step_status = TURN_CLOCK_TARGET_YAW_ABS173_COLLSION_CR_DRYL;
                break;
            }
            break;
        case TURN_CLOCK_TARGET_YAW_ABS173_COLLSION_CR_DRYL:
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
                collision_right_rightrun_step_status = TURN_CCLCOK_TARGET_YAW_ABS153_COLLSION_CR_DRYM;
                break;
            }
            break;
        case TURN_CCLCOK_TARGET_YAW_ABS153_COLLSION_CR_DRYM:
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
                collision_right_rightrun_step_status = GOSTR_YAW_EQUAL_ABS153_CR_DRYM;
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
            if(cnt_update > 4)
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
                collision_right_rightrun_step_status = GOSTR_BYPASS_BOW_CONTINUE_EXIT_CR_DRYM;
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
                collision_right_rightrun_step_status = GOSTR_BYPASS_CR_DRYM;
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
        case TURN_CCLOCK_TARGET_YAW_LESS_ABS165_CR_DRYM:
            if (obstacleSignal == front_obstacle || obstacleSignal == right_obstacle || obstacleSignal == left_obstacle)
            {
                collision_right_rightrun_step_status = GOSTR_BYPASS_CR_DRYM;
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
                //second_turn_collsion();
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

////////////////////////////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////////////////////////////
unsigned char CollisionLeftRightRunStep(POSE *current_pose,unsigned char obstacleSignal)
{
    double Yaw;
    unsigned char complete_flag = 0;
    Yaw = Rad2Deg(current_pose->orientation);
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
                collision_left_rightrun_step_status = TURN_CLOCK_TARGET_YAW_ABS30_COLLSION_CL_DRYL;
                break;
            }
            break;
        case  TURN_CLOCK_TARGET_YAW_ABS30_COLLSION_CL_DRYL:
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
                collision_left_rightrun_step_status = GOSTR_YAW_MORE_ABS30_CL_DRYL;
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
            if(cnt_update > 4)
            {
                cnt_update = 0;
                collision_left_rightrun_step_status = MORE_TRY_BREAK_BYPASS_CR_DRYM;  
                break;     
            }
            collision_left_rightrun_step_status = GOSTR_BYPASS_LOOP_CL_DRYL;  
            break;
        case  GOSTR_BYPASS_LOOP_CL_DRYL:
            if (obstacleSignal == right_obstacle)
            {
                collision_left_rightrun_step_status = LEFT_COLLISION_BYPASS_CL_DRYL;   
                break;
            }
            if (my_abs(last_position_x - current_pose->x) > lateral_move_distance / 3)
            {
                collision_left_rightrun_step_status = GOSTR_X_MORE_ONE_THIRD_LATERALDIS_BYPASS_CL_DRYL; 
                break;
            }
            if (my_abs(last_position_y - current_pose->y) > close_edge || obstacleSignal == front_obstacle || obstacleSignal == left_obstacle)
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
            break;

        case  GOSTR_BYPASS_BOW_CONTINUE_CL_DRYL:
            if (obstacleSignal == front_obstacle || obstacleSignal == right_obstacle || obstacleSignal == left_obstacle)
            {
                collision_left_rightrun_step_status = GOSTR_BYPASS_BOW_CONTINUE_COLLISION_CL_DRYL; 
                break;
            }
            if (my_abs(Yaw / 100) > 178 )
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
                bow_continue = true;
                collision_left_rightrun_step_status = GOSTR_BYPASS_BOW_CONTINUE_EXIT_CL_DRYL;
                turn_start_update = 0;
                break;
            }
            break;
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
                temporary_yaw = Yaw / 100;
                if (my_abs(Yaw / 100) <15) collision_left_rightrun_step_status = TURN_CLCOK_TARGET_YAW_LESS_ABS15_CL_DRYL;
                else collision_left_rightrun_step_status = TURN_CLCOK_TARGET_YAW_MORE_ABS15_CL_DRYL;
                break;
            }
            break;
        case  TURN_CLCOK_TARGET_YAW_LESS_ABS15_CL_DRYL:
            if (obstacleSignal == front_obstacle || obstacleSignal == right_obstacle || obstacleSignal == left_obstacle)
            {
                linear_velocity = 0;
                angular_velocity = 0;
                collision_left_rightrun_step_status = GOSTR_BYPASS_CL_DRYL;
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
        case  TURN_CLCOK_TARGET_YAW_MORE_ABS15_CL_DRYL:
            if (obstacleSignal == front_obstacle || obstacleSignal == right_obstacle || obstacleSignal == left_obstacle)
            {
                collision_left_rightrun_step_status = GOSTR_BYPASS_CL_DRYL;
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
                collision_left_rightrun_step_status = GOSTR_BYPASS_CR_DRYM;
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
                //second_turn_collsion();
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
            last_position_yy = current_pose->x;
            collision_left_rightrun_step_status = TURN_CCLOCK_TARGET_YAW_ABS153_CL_DRYM;
            break;
        case  TURN_CCLOCK_TARGET_YAW_ABS153_CL_DRYM:
            linear_velocity = 0;
            angular_velocity = turn_vel;
            if (my_abs(Yaw / 100) < 153)
            {
                linear_velocity = 0;
                angular_velocity = 0;
                collision_left_rightrun_step_status = GOSTR_YAW_EQUAL_ABS153_CL_DRYM;
                break;
            }
            if (obstacleSignal == front_obstacle || obstacleSignal == left_obstacle || obstacleSignal == right_obstacle)
            {
                linear_velocity = 0;
                angular_velocity = 0;
                collision_left_rightrun_step_status = TURN_CCLOCK_TAEGET_YAW_ABS153_COLLSION_CL_DRYM;
                break;
            }
            break;
        case  TURN_CCLOCK_TAEGET_YAW_ABS153_COLLSION_CL_DRYM:
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
                collision_left_rightrun_step_status  = TURN_CCLOCK_TARGET_YAW_NEG123_COLLSION_CL_DRYM;
                break;
            }
            break;
        case  TURN_CCLOCK_TARGET_YAW_NEG123_COLLSION_CL_DRYM:
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
                collision_left_rightrun_step_status = TURN_CCLOCK_TARGET_YAW_NEG98_COLLSION_CL_DRYM;
                break;
            }
            break;
        case  TURN_CCLOCK_TARGET_YAW_NEG98_COLLSION_CL_DRYM:
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
            if(RightReverseWalkEdge(current_pose,obstacleSignal))
            {
                collision_left_rightrun_step_status = COMPLETE_CL_DRYM;
                break;
            }
            break;
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
                collision_left_rightrun_step_status = TURN_CCLOCK_TARGET_YAW_ABS8_COLLSION_CL_DRYM;
                break;
            }
            break;
        case  TURN_CCLOCK_TARGET_YAW_ABS8_COLLSION_CL_DRYM:
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

////////////////////////////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////////////////////////////
unsigned char CollisionFrontRightRunStep(POSE *current_pose,unsigned char obstacleSignal)
{
    double Yaw;
    unsigned char complete_flag = 0;
    Yaw = Rad2Deg(current_pose->orientation);
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
                collision_front_rightrun_step_status = TURN_CLOCK_TARGET_YAW_NEG60_COLLSION_CF_DRYL;
                break;
            }
            break;             
        case TURN_CLOCK_TARGET_YAW_NEG60_COLLSION_CF_DRYL:
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
                collision_front_rightrun_step_status  = TURN_CLOCK_TARGET_YAW_NEG82_COLLSION_CF_DRYL;
                break;
            }
            break;         
        case TURN_CLOCK_TARGET_YAW_NEG82_COLLSION_CF_DRYL:
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
            if(RightWalkEdge(current_pose,obstacleSignal))
            {
                collision_front_rightrun_step_status = COMPLETE_CR_DRYL;
                break;
            }
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
                collision_front_rightrun_step_status  = TURN_CLOCK_TARGET_YAW_ABS173_COLLSION_CF_DRYL;
                break;
            }
            break;        
        case TURN_CLOCK_TARGET_YAW_ABS173_COLLSION_CF_DRYL:
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
                collision_front_rightrun_step_status = TURN_CCLOCK_TARGET_YAW_ABS120_COLLSION_CF_DRYM;
                break;
            }
            break;            
        case TURN_CCLOCK_TARGET_YAW_ABS120_COLLSION_CF_DRYM:
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
                collision_front_rightrun_step_status  = TURN_CCLOCK_TARGET_YAW_ABS93_COLLSION_CF_DRYM;
                break;
            }
            break;            
        case TURN_CCLOCK_TARGET_YAW_ABS93_COLLSION_CF_DRYM:
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
            if(RightReverseWalkEdge(current_pose,obstacleSignal))
            {
                collision_front_rightrun_step_status = COMPLETE_CR_DRYL;
                break;
            }
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
                collision_front_rightrun_step_status  = TURN_CCLOCK_TARGET_YAW_ABS10_COLLSION_CF_DRYM;
                break;
            }

            break;       
        case TURN_CCLOCK_TARGET_YAW_ABS10_COLLSION_CF_DRYM:
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
            complete_flag = 1;
            break;  
    }
    return complete_flag;

}

////////////////////////////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////////////////////////////
unsigned char RightEdgeDilemma(POSE *current_pose,unsigned char obstacleSignal)
{
    double Yaw;
    unsigned char complete_flag = 0;
    Yaw = Rad2Deg(current_pose->orientation);
    
    switch(right_edge_dilemma_status)
    {
        case 0:
            last_position_x = current_pose->x + W;
            right_edge_dilemma_status = DELTA_X_MORE_ONE_THIRD_CLEANED_MAP_WIDTH_DILEMMA;
            break;
        case DELTA_X_MORE_ONE_THIRD_CLEANED_MAP_WIDTH_DILEMMA:
            if (my_abs(last_position_x - (current_pose->x + W)) > bsp_Edge_length() / 3)
            {
                complete_flag = 1;
                right_edge_dilemma_status = 0;
                break;
            }
            right_edge_dilemma_status = LOOP_TEN_NUM_DILEMMA;
            break; 
        case LOOP_TEN_NUM_DILEMMA:
            if(number >10)
            {
                complete_flag = 1;
                right_edge_dilemma_status = 0;
                break;
            }
            right_edge_dilemma_status = YAW_MORE_ABS90_LESS_ABS176_CCLOCK_LESS_ABS90_MORE_ABS4_CLOCK_DILEMMA;
            break; 
        case YAW_MORE_ABS90_LESS_ABS176_CCLOCK_LESS_ABS90_MORE_ABS4_CLOCK_DILEMMA:
            if (my_abs(Yaw / 100) > 90 && my_abs(Yaw / 100) < 176)
            {
                if (Yaw > 0)
                {
                    linear_velocity = long_stra_vel / 4;
                    angular_velocity = turn_vel / 2;
                }
                else
                {
                    linear_velocity = long_stra_vel / 4;
                    angular_velocity = -turn_vel / 2;
                }
            }
            else if (my_abs(Yaw / 100) < 90 && my_abs(Yaw / 100) > 4)
            {
                if (Yaw > 0)
                {
                    linear_velocity = long_stra_vel / 4;
                    angular_velocity = -turn_vel / 2;
                }
                else
                {
                    linear_velocity = long_stra_vel / 4;
                    angular_velocity = turn_vel / 2;
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
            if (right_obstacle == obstacleSignal || front_obstacle == obstacleSignal || left_obstacle == obstacleSignal)
            {
                right_edge_dilemma_status = GOSTR_COLLSION_DILEMMA;
                break;  
            }
            right_edge_dilemma_status = DELTA_X_MORE_ONE_THIRD_CLEANED_MAP_WIDTH_DILEMMA;
            break;
        case GOSTR_COLLSION_DILEMMA:
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
                    right_edge_dilemma_status = COLLSION_YAW_MORE_ABS90_DILEMMA;
                    break;
                }
                else
                {
                    right_edge_dilemma_status = COLLSION_YAW_LESS_ABS90_DILEMMA;
                    break;
                }
            }
            break;

        case COLLSION_YAW_MORE_ABS90_DILEMMA:
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
                right_edge_dilemma_status  = GOSTR_COLLSION_CYM_DILEMMA;
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
        case GOSTR_COLLSION_CYM_DILEMMA:
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
            if (my_abs(Yaw / 100) < 3)
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

        case COLLSION_YAW_LESS_ABS90_DILEMMA:
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
                right_edge_dilemma_status  = GOSTR_COLLSION_CYL_DILEMMA;
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
        case GOSTR_COLLSION_CYL_DILEMMA:
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
            if (my_abs(Yaw / 100) > 178)
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

////////////////////////////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////////////////////////////
unsigned char RightWalkEdge(POSE *current_pose,unsigned char obstacleSignal)
{
    double Yaw;
    unsigned char complete_flag = 0;
    Yaw = Rad2Deg(current_pose->orientation);

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
                right_walk_edge_status = TURN_CLOCK_TARGET_YAW_MORE_ABS117_COLLSION_WE;
                break;
            }
            break;                                                      
        case TURN_CLOCK_TARGET_YAW_MORE_ABS117_COLLSION_WE:
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
            if (last_position_y - current_pose->y > temporary_close_edge++)
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

            last_position_x = current_pose->x;
            break;

        case REBACK_GOSTR_BYPASS_CHECK_WE:
            if (my_abs(Yaw / 100) < 105 && (Yaw / 100 > 0))
            {
                right_walk_edge_status = TARGET_YAW_LESS_ABS105_MORE_0_BYPASS_WE;
                break;
            }
            if ((my_abs(current_pose->x + W - edge_length_start) > 3 * bsp_Edge_length() / 4))
            {
                right_walk_edge_status = DELTA_X_MORE_ONE_FOURTH_CLEANED_MAP_WIDTH_WE;
                break; 
            }
            right_walk_edge_status = GOSTR_BYPASS_WE;
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

            //right_walk_edge_status = REBACK_GOSTR_BYPASS_CHECK_WE;
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
                right_walk_edge_status = TURN_CLOCK_YAW_ADD_ABS15_COLLSION_WE;
                break;
            }
            break;                                            
        case TURN_CLOCK_YAW_ADD_ABS15_COLLSION_WE:
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
        case TARGET_YAW_LESS_ABS105_MORE_0_BYPASS_WE:
            if (my_abs(current_pose->x + W - edge_length_start) > (bsp_Edge_length() / 4))
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
            if (my_abs(Yaw / 100) < 3)
            {
                linear_velocity = 0;
                angular_velocity = 0;
                right_walk_edge_status = RIGHT_EDGE_DILEMMA_WE;
                break;
            }

            if (obstacleSignal == front_obstacle || obstacleSignal == left_obstacle || obstacleSignal == right_obstacle)
            {
                right_walk_edge_status = TURN_CLOCK_TARGET_YAW_LESS_ABS3_COLLSION_WE;
            }
            linear_velocity = 0;
            angular_velocity = -turn_vel;

        case TURN_CLOCK_TARGET_YAW_LESS_ABS3_COLLSION_WE:
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
            if(RightEdgeDilemma(current_pose,obstacleSignal))
            {
                right_walk_edge_status = BOW_CONTINUE_WE;
            }
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
            right_walk_edge_status = GOSTR_BYPASS_WE;
            break;                                                  
        case TURN_CCLOCK_TARGET_YAW_LESS_0_WE:
            linear_velocity = 0;
            angular_velocity = turn_vel;
            if (my_abs(Yaw / 100) < 0 )
            {
                linear_velocity = 0;
                angular_velocity = 0;
                right_walk_edge_status = GOSTR_BYPASS_WE;
                break;
            }
            if (obstacleSignal == front_obstacle || obstacleSignal == left_obstacle || obstacleSignal == right_obstacle)
            {
                linear_velocity = 0;
                angular_velocity = 0;
                right_walk_edge_status = TURN_CCLOCK_TARGET_YAW_LESS_0_COLLSION_WE;
                break;
            }
            break;                                                 
        case TURN_CCLOCK_TARGET_YAW_LESS_0_COLLSION_WE:
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
                right_walk_edge_status = GOSTR_BYPASS_WE;
                break;
            }
            if (obstacleSignal == front_obstacle || obstacleSignal == left_obstacle || obstacleSignal == right_obstacle)
            {
                linear_velocity = 0;
                angular_velocity = 0;
                right_walk_edge_status = TURN_CLOCK_TARGET_YAW_MORE_ABS117_COLLSION_WE;
                break;
            }
            break; 
        case TURN_CCLOCK_TARGET_YAW_LESS_135_DETAL_YAW_MORE_30_COLLSION_WE:
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
            linear_velocity = long_stra_vel;
            angular_velocity = 0;
            complete_flag = 1;
            right_walk_edge_status = 0;
            break;                               
        case RETURN_ORIGIN_WE:
            linear_velocity = long_stra_vel;
            angular_velocity = 0;
            complete_flag = 2;
            right_walk_edge_status = 0;
            break;
    }   
    return complete_flag;                              
}

////////////////////////////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////////////////////////////
unsigned char RightReverseWalkEdge(POSE *current_pose,unsigned char obstacleSignal)
{
    double Yaw;
    unsigned char complete_flag = 0;
    Yaw = Rad2Deg(current_pose->orientation);
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
            if (my_abs(turn_start_x - current_pose->x) > turn_backward_distance || my_abs(turn_start_y - current_pose->y) > turn_backward_distance)
            {
                linear_velocity = 0;
                angular_velocity = 0;
                turn_start_update = 0;
                right_walk_edge_status = TURN_CCLOCK_TARGET_YAW_LESS_ABS63_RWE;
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
                right_walk_edge_status = READY_GOSTR_BYPASS_RWE;
                break;
            }
            if (obstacleSignal == front_obstacle || obstacleSignal == left_obstacle || obstacleSignal == right_obstacle)
            {
                linear_velocity = 0;
                angular_velocity = 0;
                right_walk_edge_status = TURN_CCLOCK_TARGET_YAW_LESS_ABS63_COLLSION_RWE;
                break;
            }
            break;                                                
        case TURN_CCLOCK_TARGET_YAW_LESS_ABS63_COLLSION_RWE                 :
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
                right_walk_edge_status = TURN_CCLOCK_TARGET_YAW_LESS_ABS63_RWE;
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
            right_walk_edge_status = GOSTR_BYPASS_WE;
            break;  
        case GOSTR_BYPASS_RWE                                               :
            linear_velocity = long_stra_vel;
            angular_velocity =0;
            if (obstacleSignal == front_obstacle || obstacleSignal == right_obstacle || obstacleSignal == left_obstacle)
            {
                right_walk_edge_status = COLLSION_BYPASS_RWE;
                break;
            }
            if (my_abs(last_position_x - current_pose->x) > lateral_move_distance)
            {
                right_walk_edge_status = GOSTR_X_MORE_LATERALDIS_BYPASS_RWE;
                break;
            }
            if (last_position_y - current_pose->y > temporary_close_edge++)
            {
                right_walk_edge_status = BOW_CONTINUE_RWE;
                break;
            }
            if (b_last_position_yy == true && my_abs(last_position_yy - current_pose->y) > 3 * lateral_move_distance)
            {
                right_walk_edge_status = BOW_CONTINUE_RWE;
                break;
            }
            if (my_abs(Yaw / 100) < 120 && (Yaw / 100 > 0) && (b_last_position_yy == false))
            {
                last_position_yy = current_pose->y;
                b_last_position_yy = true;
            }
            last_position_x = current_pose->x;
            break;
        case REBACK_GOSTR_BYPASS_CHECK_RWE:
            if (my_abs(Yaw / 100) > 75 && (Yaw / 100 > 0))
            {
                right_walk_edge_status = TARGET_YAW_MORE_ABS75_MORE_ABS0_RWE;
                break;
            }
            if ((my_abs(current_pose->x + W - edge_length_start) > 3 * bsp_Edge_length() / 4))
            {
                right_walk_edge_status = DELTA_X_MORE_ONE_FOURTH_CLEANED_MAP_WIDTH_RWE;
                break; 
            }
            right_walk_edge_status = GOSTR_BYPASS_RWE;
            break;                           
        case COLLSION_BYPASS_RWE                                            :
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
                right_walk_edge_status = TURN_CCLOCK_YAW_ADD_ABS15_RWE;
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
                right_walk_edge_status = REBACK_GOSTR_BYPASS_CHECK_RWE;
                break;
            }
            if (obstacleSignal == front_obstacle || obstacleSignal == left_obstacle || obstacleSignal == right_obstacle)
            {
                linear_velocity = 0;
                angular_velocity = 0;
                right_walk_edge_status = TURN_CCLOCK_YAW_ADD_ABS15_COLLSION_RWE;
                break;
            }
            break;                                        
        case TURN_CCLOCK_YAW_ADD_ABS15_COLLSION_RWE                         :
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
                right_walk_edge_status = TURN_CCLOCK_YAW_ADD_ABS15_RWE;
                break;
            }
            break;                                                 
        case TARGET_YAW_MORE_ABS75_MORE_ABS0_RWE                            :
            if (my_abs(current_pose->x + W - edge_length_start) > (bsp_Edge_length() / 4))
            {
                right_walk_edge_status = RETURN_ORIGIN_RWE;
                break;
            }
            else
            {
                right_walk_edge_status = TURN_CCLOCK_YAW_MORE_178ABS_RWE;
                break;
            }
            break;                                              
        case DELTA_X_MORE_ONE_FOURTH_CLEANED_MAP_WIDTH_RWE                  :
            right_walk_edge_status = RETURN_ORIGIN_RWE;
            break;                                                       
        case TURN_CCLOCK_YAW_MORE_178ABS_RWE                                :
            if (my_abs(Yaw / 100) > 178)
            {
                linear_velocity = 0;
                angular_velocity = 0;
                right_walk_edge_status = RIGHT_EDGE_DILEMMA_RWE;
                break;
            }
            if (obstacleSignal == front_obstacle || obstacleSignal == left_obstacle || obstacleSignal == right_obstacle)
            {
                right_walk_edge_status = TURN_CCLOCK_YAW_MORE_178ABS_COLLSION_RWE;
            }
            linear_velocity = 0;
            angular_velocity = -turn_vel;
            break;                                          
        case TURN_CCLOCK_YAW_MORE_178ABS_COLLSION_RWE                       :
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
                right_walk_edge_status = TURN_CCLOCK_YAW_MORE_178ABS_RWE;
                break;
            }
            break;                                                   
        case RIGHT_EDGE_DILEMMA_RWE                                         :
            if(RightEdgeDilemma(current_pose,obstacleSignal))
            {
                right_walk_edge_status = BOW_CONTINUE_RWE;
            }
            break;                       
        case GOSTR_X_MORE_LATERALDIS_BYPASS_RWE                             :
            temporary_yaw = Yaw / 100;
            if (my_abs(Yaw / 100) > 45 && (Yaw / 100 > 0))
            {
                right_walk_edge_status = TURN_CLOCK_TARGET_YAW_LESS_0_RWE;
                
                break;
            }
            else if (my_abs(Yaw / 100) <= 45 )
            {
                right_walk_edge_status = TURN_CLOCK_TARGET_YAW_LESS_ABS45_DETAL_YAW_MORE_30_RWE;
                break;
            }
            right_walk_edge_status = GOSTR_BYPASS_RWE;
            break;                                         
        case TURN_CLOCK_TARGET_YAW_LESS_0_RWE                 :
            linear_velocity = 0;
            angular_velocity = -turn_vel;
            if (my_abs(Yaw / 100) < 0 )
            {
                linear_velocity = 0;
                angular_velocity = 0;
                right_walk_edge_status = GOSTR_BYPASS_RWE;
                break;
            }
            if (obstacleSignal == front_obstacle || obstacleSignal == left_obstacle || obstacleSignal == right_obstacle)
            {
                linear_velocity = 0;
                angular_velocity = 0;
                right_walk_edge_status = TURN_CLOCK_TARGET_YAW_LESS_0_COLLSION_RWE;
                break;
            }
            break;                                                         
        case TURN_CLOCK_TARGET_YAW_LESS_0_COLLSION_RWE      :
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
                right_walk_edge_status = TURN_CLOCK_TARGET_YAW_LESS_0_RWE;
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
                right_walk_edge_status = GOSTR_BYPASS_RWE;
                break;
            }
            if (obstacleSignal == front_obstacle || obstacleSignal == left_obstacle || obstacleSignal == right_obstacle)
            {
                linear_velocity = 0;
                angular_velocity = 0;
                right_walk_edge_status = TURN_CLOCK_TARGET_YAW_LESS_ABS45_DETAL_YAW_MORE_30_COLLSION_RWE;
                break;
            }
            break;                                                                 
        case TURN_CLOCK_TARGET_YAW_LESS_ABS45_DETAL_YAW_MORE_30_COLLSION_RWE:
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
                right_walk_edge_status = TURN_CLOCK_TARGET_YAW_LESS_ABS45_DETAL_YAW_MORE_30_RWE;
                break;
            }
            break;  
        case BOW_CONTINUE_RWE                                               :
            linear_velocity = long_stra_vel;
            angular_velocity = 0;
            complete_flag = 1;
            right_walk_edge_status = 0;
            break;                          
        case RETURN_ORIGIN_RWE                                              :
            linear_velocity = long_stra_vel;
            angular_velocity = 0;
            complete_flag = 2;
            right_walk_edge_status = 0;
            break;  
    }   
    return complete_flag;     
}

////////////////////////////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////////////////////////////
unsigned char ForwardBoundaryRightRunStep(POSE *current_pose,unsigned char obstacleSignal)
{
    double Yaw;
    unsigned char complete_flag = 0;
    Yaw = Rad2Deg(current_pose->orientation);
    switch(right_forward_boundary_status) 
    {
        case 0:
            if (my_abs(Yaw / 100) < 10)
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
            linear_velocity = long_stra_vel / 2;
            angular_velocity = -57;
            if (my_abs(Yaw / 100) > 178)
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
            if (my_abs(turn_start_x - current_pose->x) > turn_backward_distance || my_abs(turn_start_y - current_pose->y) > turn_backward_distance)
            {
                linear_velocity = 0;
                angular_velocity = 0;
                turn_start_update = 0;
                right_walk_edge_status = FORWARDBOUNDARY_CLOCK_TARGET_YAW_MORE_ABS178;
                break;
            }
            break;
        case FORWARDBOUNDARY_YAW_OTHER:
            right_forward_boundary_status = FORWARDBOUNDARY_CCLOCK_TARGET_YAW_LESS_ABS3;
            break;
        case FORWARDBOUNDARY_CCLOCK_TARGET_YAW_LESS_ABS3:
            linear_velocity = long_stra_vel / 2;
            angular_velocity = 57;
            if (my_abs(Yaw / 100) < 3)
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
            linear_velocity = long_stra_vel;
            angular_velocity = 0;
            if (obstacleSignal == front_obstacle || obstacleSignal == left_obstacle || obstacleSignal == right_obstacle)
            {
                right_forward_boundary_status = FORWARDBOUNDARY_COMPLETE;
                break;
            }
            break;
        case FORWARDBOUNDARY_COMPLETE:
            right_forward_boundary_status = 0;
            complete_flag = 1;
            break;

    }
    return complete_flag;
}
////////////////////////////////////////////////////////////////////////////////////////////////////////
unsigned char RightReadyLeakingSweep(POSE *current_pose,unsigned char obstacleSignal)
{
    double Yaw;
    unsigned char complete_flag = 0;
    Yaw = Rad2Deg(current_pose->orientation);
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
                    right_forward_boundary_status = RIGHT_LEAKING_SWEEP_YAW_MORE_ABS90;
                    break;
                }
                else
                {
                    right_forward_boundary_status = RIGHT_LEAKING_SWEEP_YAW_OTHER;
                    break;
                }
            }  
            break;
        case RIGHT_LEAKING_SWEEP_YAW_MORE_ABS90                         :
            right_forward_boundary_status = RIGHT_LEAKING_SWEEP_CLOCK_TARGET_YAW_LESS_ABS90;
            break;
        case RIGHT_LEAKING_SWEEP_CLOCK_TARGET_YAW_LESS_ABS90            :
            if(my_abs(Yaw/100)<90){
                linear_velocity=0;
                angular_velocity=0;
                last_position_y=current_pose->y;
                right_forward_boundary_status = RIGHT_LEAKING_SWEEP_GOSTRAIGHT_MORE;
                break;
            }
            if (obstacleSignal == front_obstacle || obstacleSignal == left_obstacle || obstacleSignal == right_obstacle)
            {
                linear_velocity=0;
                angular_velocity=0;
                right_forward_boundary_status = RIGHT_LEAKING_SWEEP_CLOCK_TARGER_YAW_LESS_ABS90_COLLISION;
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
                right_forward_boundary_status = RIGHT_LEAKING_SWEEP_CLOCK_TARGET_YAW_LESS_ABS90;
                break;
            }  
            break;
        case RIGHT_LEAKING_SWEEP_GOSTRAIGHT_MORE                        :
            linear_velocity=long_stra_vel;
            angular_velocity=0;
            if(obstacleSignal==front_obstacle||obstacleSignal==left_obstacle||obstacleSignal==right_obstacle){
                right_forward_boundary_status = RIGHT_LEAKING_SWEEP_CLOCK_TARGER_YAW_LESS_ABS3;
                break;
            }
            if(my_abs(last_position_y-current_pose->y)>leakingsweep){
                right_forward_boundary_status = RIGHT_LEAKING_SWEEP_CLOCK_TARGER_YAW_LESS_ABS3;
                break;
            }
            break;
        case RIGHT_LEAKING_SWEEP_CLOCK_TARGER_YAW_LESS_ABS3             :
            if(my_abs(Yaw/100)<3){
                linear_velocity=0;
                angular_velocity=0;
                right_forward_boundary_status = RIGHT_LEAKING_SWEEP_COMPLETE;
                break;
            }
            if (obstacleSignal == front_obstacle || obstacleSignal == left_obstacle || obstacleSignal == right_obstacle)
            {
                linear_velocity=0;
                angular_velocity=0;
                right_forward_boundary_status = RIGHT_LEAKING_SWEEP_CLOCK_TARGER_YAW_LESS_ABS3_COLLISION;
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
                right_forward_boundary_status = RIGHT_LEAKING_SWEEP_CLOCK_TARGER_YAW_LESS_ABS3;
                break;
            }  
            break;
        case RIGHT_LEAKING_SWEEP_YAW_OTHER                              :
            right_forward_boundary_status = RIGHT_LEAKING_SWEEP_CCLOCK_TARGET_YAW_MORE_ABS90;
            break;
        case RIGHT_LEAKING_SWEEP_CCLOCK_TARGET_YAW_MORE_ABS90           :
            if(my_abs(Yaw/100)>90){
                linear_velocity=0;
                angular_velocity=0;
                last_position_y=current_pose->y;
                right_forward_boundary_status = RIGHT_LEAKING_SWEEP_GOSTRAIGHT_OTHER;
                break;
            }
            if (obstacleSignal == front_obstacle || obstacleSignal == left_obstacle || obstacleSignal == right_obstacle)
            {
                linear_velocity=0;
                angular_velocity=0;
                right_forward_boundary_status = RIGHT_LEAKING_SWEEP_CCLOCK_TARGET_YAW_MORE_ABS90_COLLISION;
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
                right_forward_boundary_status = RIGHT_LEAKING_SWEEP_CCLOCK_TARGET_YAW_MORE_ABS90;
                break;
            }  
            break;
            break;
        case RIGHT_LEAKING_SWEEP_GOSTRAIGHT_OTHER                       :
            linear_velocity=long_stra_vel;
            angular_velocity=0;
            if(obstacleSignal==front_obstacle||obstacleSignal==left_obstacle||obstacleSignal==right_obstacle){
                right_forward_boundary_status = RIGHT_LEAKING_SWEEP_CCLOCK_TARGET_YAW_MORE_ABS178;
                break;
            }
            if(my_abs(last_position_y-current_pose->y)>leakingsweep){
                right_forward_boundary_status = RIGHT_LEAKING_SWEEP_CCLOCK_TARGET_YAW_MORE_ABS178;
                break;
            }
            break;
        case RIGHT_LEAKING_SWEEP_CCLOCK_TARGET_YAW_MORE_ABS178           :
            if(my_abs(Yaw/100)>178){
                linear_velocity=0;
                angular_velocity=0;
                right_forward_boundary_status = RIGHT_LEAKING_SWEEP_COMPLETE;
                break;
            }
            if (obstacleSignal == front_obstacle || obstacleSignal == left_obstacle || obstacleSignal == right_obstacle)
            {
                linear_velocity=0;
                angular_velocity=0;
                right_forward_boundary_status = RIGHT_LEAKING_SWEEP_CCLOCK_TARGET_YAW_MOEE_ABS178_COLLISIION;
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
                right_forward_boundary_status = RIGHT_LEAKING_SWEEP_CCLOCK_TARGET_YAW_MORE_ABS178;
                break;
            }  
            break;
        case RIGHT_LEAKING_SWEEP_COMPLETE                               :
            right_forward_boundary_status = 0;
            complete_flag = 1;
            break;
    }
    return complete_flag;
}
////////////////////////////////////////////////////////////////////////////////////////////////////////

void RightReturnOriginWorkStep(POSE *current_pose,unsigned char obstacleSignal)
{}
void LeftRunningWorkStep(POSE *current_pose,unsigned char obstacleSignal)
{}
void LeftReturnOriginWorkStep(POSE *current_pose,unsigned char obstacleSignal)
{}


unsigned char LeftReadyLeakingSweep(POSE *current_pose,unsigned char obstacleSignal)
{}
	
	
	
	
