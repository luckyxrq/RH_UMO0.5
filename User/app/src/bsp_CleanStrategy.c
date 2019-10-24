#include "bsp.h"
#include <math.h>


#define DEFAULT_STRAIGHT_VEL 200.0f //150		//60//30 //50// 30cm/s
#define DEFAULT_STRAIGHT_VEL_HALF 100.0f
#define GO_BACK_VEL   150.0f
#define TURN_VEL      20.0f 	//80 100 50		//25 // 15cm/s 
#define TURN_VEL_HALF 10.0f
#define TURN_RADIUS   20.0f //80 //100 117

#define NONE_OBSTACLE_TURN_VEL  80.0f
#define NONE_OBSTACLE_TURN_R  80.0f

#define TURN_ANGULAR_VEL 50.0f

#define LEVEL_180 175
#define LEVEL_90 85
#define LEVEL_45 40 
#define LEVEL_0 5
#define SECOND_LEVEL_45 50
#define SECOND_LEVEL_90 95

#define SLOWDOWN_DISTANCE 150 		//15cm
#define MOVEBACK_DISTANCE 25 		//25mm 8cm 10cm
#define SLOWDOWN_ANGLE 0.785 		//45deg
#define FRONT_DECISION_ANGLE 0.17 	//10deg

#define DELT_Y_DISTANCE 200 // 160mm
#define OFF_DELT_Y_DISTANCE 10 // 10mm

#define WHEEL_PULSE_R 0.2085f 	        // c/pulse (pi*d/1024)

#define MAP_WIDTH_W    4000 //4000 // Unit: mm
#define MAP_HEIGHT_H   4000

#define INT_COOR_X 250
#define INT_COOR_Y 250

#define DegToRad Deg2Rad
#define RadToDeg Rad2Deg

static CleanStrategy cleanstrategy;

static void WheelBrake()
{
	cleanstrategy.linear_velocity = 0;
	cleanstrategy.angular_velocity = 0;
	
	bsp_SetMotorSpeed(MotorLeft, 0);
	bsp_SetMotorSpeed(MotorRight,0);
}

static void WheelControl(short linear_velocity,short angular_velocity)
{
	
	/*计算出速度，单位MM/S */
	int16_t leftVelocity = (int16_t)((0.5*(2*linear_velocity*0.001 - Deg2Rad(angular_velocity)*WHEEL_LENGTH))* 1000);
	int16_t rightVelocity = (int16_t)((0.5*(2*linear_velocity*0.001 + Deg2Rad(angular_velocity)*WHEEL_LENGTH))* 1000);
	
	/*设定速度*/
	bsp_SetMotorSpeed(MotorLeft,bsp_MotorSpeedMM2Pulse(leftVelocity));
	bsp_SetMotorSpeed(MotorRight,bsp_MotorSpeedMM2Pulse(rightVelocity));
}
static void StraightSlowDowm()
{
	cleanstrategy.linear_velocity = DEFAULT_STRAIGHT_VEL_HALF;
	cleanstrategy.angular_velocity = 0;
	WheelControl((short)(DEFAULT_STRAIGHT_VEL_HALF),0);
}
static void CalculateDistanceByWheelPulse(int wheel_pulse_l, int wheel_pulse_r, int last_wheel_pulse_l, int last_wheel_pulse_r, int* wheel_distance)
{
	int delt_wheel_pulse_l = wheel_pulse_l - last_wheel_pulse_l;
	int delt_wheel_pulse_r = wheel_pulse_r - last_wheel_pulse_r;
	int delt_wheel_pulse = (delt_wheel_pulse_l + delt_wheel_pulse_r) / 2;
	*wheel_distance = (int)(delt_wheel_pulse * WHEEL_PULSE_R); //mm
}
static void MoveBack()
{
	cleanstrategy.linear_velocity = (GO_BACK_VEL)*-1;
	cleanstrategy.angular_velocity = 0;
	
	WheelControl((short)(GO_BACK_VEL)*-1,0);
}






static void ObstacleAvoidanceStrategy(int ir_adc_x_sensordata, uint8_t obstacleSignal, int ir_adc315_signal, POSE current_pose, int wheel_pulse_l, int wheel_pulse_r, int Symbol_dir)
{
	int wheel_distance = 0;
	int delt_y_distance = 0;
	int off_delt_y = 0;
	
	
	UNUSED(off_delt_y);
	
	switch (cleanstrategy.obstacle_avoidance_case)
	{
	case 0:
	{
		if (ir_adc_x_sensordata == 0)
		{
			//WheelBrake(linear_velocity,angular_velocity);
			cleanstrategy.obstacle_avoidance_case = 3;
			break;
		}
		else
		{
			cleanstrategy.obstacle_avoidance_case = 1;
		}
		break;
	}
	case 1:
	{
		if (ir_adc_x_sensordata == 0)
		{
			WheelBrake();
			cleanstrategy.obstacle_avoidance_case = 2;
			break;
		}
		else
		{
			cleanstrategy.linear_velocity = 0;
			cleanstrategy.angular_velocity = Symbol_dir * TURN_ANGULAR_VEL; //50;
		}
		break;
	}
	// Added by 2019.09.25
	case 2:
	{
		if (ir_adc_x_sensordata == 1)
		{
			WheelBrake();
			cleanstrategy.obstacle_avoidance_case = 3;
			break;
		}
		else
		{
			cleanstrategy.linear_velocity = 0;
			cleanstrategy.angular_velocity = -(Symbol_dir * TURN_ANGULAR_VEL); //-50;
		}
		break;
	}
	case 3:
	{
		delt_y_distance = current_pose.y - cleanstrategy.last_adjustment_pose.y;
		off_delt_y = delt_y_distance - DELT_Y_DISTANCE;
		if(delt_y_distance >= DELT_Y_DISTANCE)
		//if (abs(off_delt_y) <= OFF_DELT_Y_DISTANCE)
		{
			WheelBrake();
			cleanstrategy.obstacle_avoidance_case = 4;
		}
		// Added by 2019.10.10 17:36
		else if ((obstacleSignal == FRONT_OBSTACLE_SIGNAL) || (obstacleSignal == RIGHT_OBSTACLE_SIGNAL) || (obstacleSignal == LEFT_OBSTACLE_SIGNAL))
		{
			WheelBrake(); // Slow down
			cleanstrategy.slowdown_pose = current_pose;
			cleanstrategy.last_wheel_pulse_l = wheel_pulse_l;
			cleanstrategy.last_wheel_pulse_r = wheel_pulse_r;
			cleanstrategy.obstacle_avoidance_case = 40;
		}
		else
		{
			cleanstrategy.linear_velocity = DEFAULT_STRAIGHT_VEL;
			cleanstrategy.angular_velocity = 0;
		}
		break;
	}
	case 40:
	{
		
		MoveBack(); // 10cm
		CalculateDistanceByWheelPulse(wheel_pulse_l, wheel_pulse_r,\
			cleanstrategy.last_wheel_pulse_l, cleanstrategy.last_wheel_pulse_r, &wheel_distance);
		if (abs(wheel_distance) >= MOVEBACK_DISTANCE)
		{
			WheelBrake();
			cleanstrategy.obstacle_avoidance_case = 4;
		}
		else
			;
		break;
	}
	case 4:
	{
		if (((abs((int)RadToDeg(current_pose.orientation)) > LEVEL_180) && (Symbol_dir == 1)) || ((abs((int)RadToDeg(current_pose.orientation)) < LEVEL_0) && (Symbol_dir == -1)))
		{
			WheelBrake();
			cleanstrategy.obstacle_avoidance_case = 0;
		}
		else
		{
			cleanstrategy.linear_velocity = 0;
			cleanstrategy.angular_velocity = Symbol_dir * TURN_ANGULAR_VEL; //50;
		}
		break;
	}
	default:
	{
		cleanstrategy.obstacle_avoidance_case = 0;
	}
	}
}





void bsp_StartUpdateCleanStrategy(void)
{
	cleanstrategy.route_case = 0; 
	cleanstrategy.adjustment_pose_case = 0;
	cleanstrategy.obstacle_avoidance_case = 0;
	
	cleanstrategy.slowdown_pose.x = 0;
	cleanstrategy.slowdown_pose.y = 0;
	cleanstrategy.slowdown_pose.orientation = 0;
	
	cleanstrategy.last_adjustment_pose.x=0;
	cleanstrategy.last_adjustment_pose.y=0;
	cleanstrategy.last_adjustment_pose.orientation=0;
	
	cleanstrategy.last_pose.x=0;
	cleanstrategy.last_pose.y=0;
	cleanstrategy.last_pose.orientation=0;
	
	cleanstrategy.last_wheel_pulse_l = 0;
	cleanstrategy.last_wheel_pulse_r = 0;
	
	cleanstrategy.linear_velocity = 0;
	cleanstrategy.angular_velocity = 0;
	
	cleanstrategy.set_turn_velocity = NONE_OBSTACLE_TURN_VEL;
	cleanstrategy.set_turn_radius = NONE_OBSTACLE_TURN_R;
	cleanstrategy.left_gostright_task = 0;
	cleanstrategy.is_one_task = 0;

	cleanstrategy.action = 0 ;
	cleanstrategy.delay = 0 ;
	cleanstrategy.isRunning = true;
	
}



void bsp_StopUpdateCleanStrategy(void)
{

	cleanstrategy.action = 0 ;
	cleanstrategy.delay = 0 ;
	cleanstrategy.isRunning = false;
	
}







void bsp_CleanStrategyUpdate(int robotX,int robotY,double robotTheta, unsigned char obstacleSignal, int wheel_pulse_l, int wheel_pulse_r, unsigned char IRSensorData[])
{
	int ir_adc315_signal = 0;
	int ir_adc_x_sensordata = 0;
	int wheel_distance = 0;
	float current_distance;
	int i = 0;
	POSE current_pose;
	
	ir_adc315_signal = IRSensorData[3] || IRSensorData[1] || IRSensorData[5];
	
	current_pose.x = INT_COOR_X + robotX;
	current_pose.y = INT_COOR_Y + robotY;
	current_pose.orientation = robotTheta;
	
	
	UNUSED(current_distance);
	
	//loop route
	switch (cleanstrategy.route_case)
	{
	case 0:
	{
		//------- step0 go straight ------------//
		cleanstrategy.linear_velocity = DEFAULT_STRAIGHT_VEL;
		cleanstrategy.angular_velocity = 0;
		if ((obstacleSignal == FRONT_OBSTACLE_SIGNAL) || (obstacleSignal == RIGHT_OBSTACLE_SIGNAL) || (obstacleSignal == LEFT_OBSTACLE_SIGNAL))
		{												   // if obstacle
			WheelBrake();
			cleanstrategy.slowdown_pose = current_pose;
			cleanstrategy.last_wheel_pulse_l = wheel_pulse_l;
			cleanstrategy.last_wheel_pulse_r = wheel_pulse_r;
			cleanstrategy.set_turn_velocity = TURN_VEL;
			cleanstrategy.set_turn_radius = TURN_RADIUS;
			
			if (cleanstrategy.is_one_task == 0)
			{ 
				cleanstrategy.last_adjustment_pose = current_pose;
			}
			if (obstacleSignal == RIGHT_OBSTACLE_SIGNAL)
			{
				cleanstrategy.route_case = 2;
			}else{ 
				cleanstrategy.route_case = 20;
			}
		}
		else if (ir_adc315_signal == 1)
		{
			cleanstrategy.slowdown_pose = current_pose;
			cleanstrategy.last_wheel_pulse_l = wheel_pulse_l;
			cleanstrategy.last_wheel_pulse_r = wheel_pulse_r;
			cleanstrategy.route_case = 1;
		}
		else if (current_pose.x > MAP_WIDTH_W)
		{
			WheelBrake(); // Slow down
			cleanstrategy.set_turn_velocity = NONE_OBSTACLE_TURN_VEL;
			cleanstrategy.set_turn_radius = NONE_OBSTACLE_TURN_R;
			cleanstrategy.route_case = 4;
			//last_adjustment_pose = current_pose;
			if (cleanstrategy.is_one_task == 0)
			{
				cleanstrategy.last_adjustment_pose = current_pose;
			}
			else
				;
		}
		else
			;
		break;
	}
	
	
	case 1:
	{
		StraightSlowDowm();
		CalculateDistanceByWheelPulse(wheel_pulse_l, wheel_pulse_r, cleanstrategy.last_wheel_pulse_l, cleanstrategy.last_wheel_pulse_r, &wheel_distance);
		
		if ((obstacleSignal == FRONT_OBSTACLE_SIGNAL) || (obstacleSignal == RIGHT_OBSTACLE_SIGNAL) || (obstacleSignal == LEFT_OBSTACLE_SIGNAL))
		{ // if obstacle
			WheelBrake();
			
			cleanstrategy.slowdown_pose = current_pose;
			cleanstrategy.last_wheel_pulse_l = wheel_pulse_l;
			cleanstrategy.last_wheel_pulse_r = wheel_pulse_r;
			cleanstrategy.set_turn_velocity = TURN_VEL;
			cleanstrategy.set_turn_radius = TURN_RADIUS;
		
			if (cleanstrategy.is_one_task == 0)
			{
				cleanstrategy.last_adjustment_pose = current_pose;
			}
			else
				;
			if (obstacleSignal == RIGHT_OBSTACLE_SIGNAL)
				cleanstrategy.route_case = 2;
			else
				cleanstrategy.route_case = 20;
		}
		else if (abs((int)current_pose.x) > MAP_WIDTH_W)
		{
			WheelBrake();
			cleanstrategy.set_turn_velocity = NONE_OBSTACLE_TURN_VEL;
			cleanstrategy.set_turn_radius = NONE_OBSTACLE_TURN_R;
			cleanstrategy.route_case = 4;
			//last_adjustment_pose = current_pose;
			if (cleanstrategy.is_one_task == 0)
			{
				cleanstrategy.last_adjustment_pose = current_pose;
			}
			else
				;
		}
		else if (abs(wheel_distance) >= SLOWDOWN_DISTANCE)
		{
			cleanstrategy.route_case = 0;
		}
		else
			;
		break;
	}
	case 20: // Front and left
	{
		MoveBack();
		CalculateDistanceByWheelPulse(wheel_pulse_l, wheel_pulse_r, cleanstrategy.last_wheel_pulse_l, cleanstrategy.last_wheel_pulse_r, &wheel_distance);
		if (abs(wheel_distance) >= MOVEBACK_DISTANCE)
		{
			WheelBrake();
			cleanstrategy.route_case = 30;
		}
		else
			;
		break;
	}
	case 2: // Right
	{
		MoveBack(); // 10cm
		CalculateDistanceByWheelPulse(wheel_pulse_l, wheel_pulse_r, cleanstrategy.last_wheel_pulse_l, cleanstrategy.last_wheel_pulse_r, &wheel_distance);
		if (abs(wheel_distance) >= MOVEBACK_DISTANCE)
		{
			WheelBrake();
			cleanstrategy.route_case = 3;
		}
		else
			;
		break;
	}
	case 30: // Front and left
	{
		if ((IRSensorData[9] == 1) && (abs((int)RadToDeg(current_pose.orientation)) > LEVEL_90))
		{
			WheelBrake();
			cleanstrategy.obstacle_avoidance_case = 0;
			cleanstrategy.route_case = 5;
		}
		else if ((abs((int)RadToDeg(current_pose.orientation)) > LEVEL_90) && (IRSensorData[9] == 0))
		{
			WheelBrake();
			cleanstrategy.obstacle_avoidance_case = 0;
			cleanstrategy.route_case = 5;
		}
		else
		{
			//-------- step1 turn back in the right -------------//
			//linear_velocity = turn_vel;//35;//0 //40//turn_vel;
			//double angular_velocity_rad = (double)linear_velocity/TURN_R;
			
			cleanstrategy.linear_velocity = cleanstrategy.set_turn_velocity; //turn_vel;
			cleanstrategy.angular_velocity = RadToDeg((cleanstrategy.linear_velocity) /(cleanstrategy.set_turn_radius)); //linear_velocity/l;
			
			
			if ((obstacleSignal == FRONT_OBSTACLE_SIGNAL) || (obstacleSignal == RIGHT_OBSTACLE_SIGNAL) || (obstacleSignal == LEFT_OBSTACLE_SIGNAL))
			{
				WheelBrake();
				cleanstrategy.slowdown_pose = current_pose;
				cleanstrategy.last_wheel_pulse_l = wheel_pulse_l;
				cleanstrategy.last_wheel_pulse_r = wheel_pulse_r;
				cleanstrategy.set_turn_velocity = TURN_VEL;
				cleanstrategy.set_turn_radius = TURN_RADIUS;
				cleanstrategy.route_case = 2;
			}
			else if ((abs((int)RadToDeg(current_pose.orientation)) > LEVEL_180))
			{
				WheelBrake();
				cleanstrategy.route_case = 6;
			}
		}
		break;
	}
	case 3:
	{
		if (IRSensorData[9] == 1)
		{
			WheelBrake();
			cleanstrategy.obstacle_avoidance_case = 0;
			cleanstrategy.route_case = 5;
		}
		else if ((abs((int)RadToDeg(current_pose.orientation)) > LEVEL_90) && (IRSensorData[9] == 0))
		{
			WheelBrake();
			cleanstrategy.obstacle_avoidance_case = 0;
			cleanstrategy.route_case = 5;
		}
		else
		{
			//-------- step1 turn back in the right -------------//
			//linear_velocity = turn_vel;//35;//0 //40//turn_vel;
			//double angular_velocity_rad = (double)linear_velocity/TURN_R;
			cleanstrategy.linear_velocity = cleanstrategy.set_turn_velocity; //turn_vel;
			cleanstrategy.angular_velocity = RadToDeg((cleanstrategy.linear_velocity) / (cleanstrategy.set_turn_radius)); //linear_velocity/l;
			
			if ((obstacleSignal == FRONT_OBSTACLE_SIGNAL) || (obstacleSignal == RIGHT_OBSTACLE_SIGNAL) || (obstacleSignal == LEFT_OBSTACLE_SIGNAL))
			{
				WheelBrake();
				cleanstrategy.slowdown_pose = current_pose;
				cleanstrategy.last_wheel_pulse_l = wheel_pulse_l;
				cleanstrategy.last_wheel_pulse_r = wheel_pulse_r;
				cleanstrategy.set_turn_velocity = TURN_VEL;
				cleanstrategy.set_turn_radius = TURN_RADIUS;
				cleanstrategy.route_case = 2;
			}
			else if ((abs((int)RadToDeg(current_pose.orientation)) > LEVEL_180))
			{
				WheelBrake();
				cleanstrategy.route_case = 6;
			}
		}
		break;
	}
	case 4:
	{
		cleanstrategy.linear_velocity = cleanstrategy.set_turn_velocity; //turn_vel;
		cleanstrategy.angular_velocity = RadToDeg((cleanstrategy.linear_velocity) / (cleanstrategy.set_turn_radius)); //linear_velocity/l;
		
		if ((obstacleSignal == FRONT_OBSTACLE_SIGNAL) || (obstacleSignal == RIGHT_OBSTACLE_SIGNAL) || (obstacleSignal == LEFT_OBSTACLE_SIGNAL))
		{
			WheelBrake();
			cleanstrategy.slowdown_pose = current_pose;
			cleanstrategy.last_wheel_pulse_l = wheel_pulse_l;
			cleanstrategy.last_wheel_pulse_r = wheel_pulse_r;
			cleanstrategy.set_turn_velocity = TURN_VEL;
			cleanstrategy.set_turn_radius = TURN_RADIUS;
			cleanstrategy.route_case = 2;
		}
		else if ((abs((int)RadToDeg(current_pose.orientation)) > LEVEL_180))
		{
			WheelBrake();
			cleanstrategy.route_case = 6;
		}
		break;
	}

	case 5:
	{
		if ((abs((int)RadToDeg(current_pose.orientation)) > LEVEL_180))
		{
			WheelBrake();
			cleanstrategy.route_case = 6;
			cleanstrategy.is_one_task = 0;
		}
		//------------- problem -------------------------//
		else if (abs((int)RadToDeg(current_pose.orientation)) < LEVEL_0)
		{
			WheelBrake();
			cleanstrategy.route_case = 0;
			cleanstrategy.is_one_task = 1;
		}
		else
		{
			ir_adc_x_sensordata = IRSensorData[9];
			ObstacleAvoidanceStrategy(ir_adc_x_sensordata,\
				obstacleSignal, ir_adc315_signal, current_pose, wheel_pulse_l, wheel_pulse_r, 1);

		}
		break;
	}
	case 6:
	{
		
		cleanstrategy.is_one_task = 0;
		//-------- step2 go straight -----------------------//
		cleanstrategy.linear_velocity = DEFAULT_STRAIGHT_VEL;
		cleanstrategy.angular_velocity = 0;
		if ((obstacleSignal == FRONT_OBSTACLE_SIGNAL) || (obstacleSignal == RIGHT_OBSTACLE_SIGNAL) || (obstacleSignal == LEFT_OBSTACLE_SIGNAL))
		{
			WheelBrake(); // Slow down
			cleanstrategy.slowdown_pose = current_pose;
			cleanstrategy.last_wheel_pulse_l = wheel_pulse_l;
			cleanstrategy.last_wheel_pulse_r = wheel_pulse_r;
			cleanstrategy.set_turn_velocity = TURN_VEL;
			cleanstrategy.set_turn_radius = TURN_RADIUS;
			cleanstrategy.route_case = 8;
			//last_adjustment_pose = current_pose;
			if (cleanstrategy.is_one_task == 0)
			{
				cleanstrategy.last_adjustment_pose = current_pose;
			}
			else
				;
		}
		else if (ir_adc315_signal == 1)
		{
			cleanstrategy.slowdown_pose = current_pose;
			cleanstrategy.last_wheel_pulse_l = wheel_pulse_l;
			cleanstrategy.last_wheel_pulse_r = wheel_pulse_r;
			cleanstrategy.route_case = 7;
		}
		else if (abs((int)current_pose.x) > MAP_WIDTH_W) //250 initial point: (25cm,25cm)
		{
			WheelBrake();
			cleanstrategy.set_turn_velocity = NONE_OBSTACLE_TURN_VEL;
			cleanstrategy.set_turn_radius = NONE_OBSTACLE_TURN_R;
			cleanstrategy.route_case = 10; //11
			//last_adjustment_pose = current_pose;
			if (cleanstrategy.is_one_task == 0)
			{
				cleanstrategy.last_adjustment_pose = current_pose;
			}
			else
				;
		}
		else
			;
		break;
	}
	case 7:
	{
		StraightSlowDowm();
		CalculateDistanceByWheelPulse(wheel_pulse_l, wheel_pulse_r, \
			cleanstrategy.last_wheel_pulse_l, cleanstrategy.last_wheel_pulse_r, &wheel_distance);
		if ((obstacleSignal == FRONT_OBSTACLE_SIGNAL) || (obstacleSignal == RIGHT_OBSTACLE_SIGNAL) || (obstacleSignal == LEFT_OBSTACLE_SIGNAL))
		{ // if obstacle
			WheelBrake();
			cleanstrategy.slowdown_pose = current_pose;
			cleanstrategy.last_wheel_pulse_l = wheel_pulse_l;
			cleanstrategy.last_wheel_pulse_r = wheel_pulse_r;
			cleanstrategy.set_turn_velocity = TURN_VEL;
			cleanstrategy.set_turn_radius = TURN_RADIUS;
			cleanstrategy.route_case = 8;
			if (cleanstrategy.is_one_task == 0)
			{
				cleanstrategy.last_adjustment_pose = current_pose;
			}
			else
				;
		}
		else if (abs((int)current_pose.x) > MAP_WIDTH_W)
		{
			WheelBrake(); // Slow down
			cleanstrategy.set_turn_velocity = NONE_OBSTACLE_TURN_VEL;
			cleanstrategy.set_turn_radius = NONE_OBSTACLE_TURN_R;
			cleanstrategy.route_case = 10;
			//last_adjustment_pose = current_pose;
			if (cleanstrategy.is_one_task == 0)
			{
				cleanstrategy.last_adjustment_pose = current_pose;
			}
			else
				;
		}
		else if (abs(wheel_distance) >= SLOWDOWN_DISTANCE)
			cleanstrategy.route_case = 6;
		else
			;
		break;
	}
	//--------------------------------------------------//
	case 8:
	{
		MoveBack(); // 10cm
		CalculateDistanceByWheelPulse(wheel_pulse_l, wheel_pulse_r,\
			cleanstrategy.last_wheel_pulse_l, cleanstrategy.last_wheel_pulse_r, &wheel_distance);
		if (abs(wheel_distance) >= MOVEBACK_DISTANCE)
		{
			WheelBrake();
			cleanstrategy.route_case = 9;
		}
		else
			;
		break;
	}
	case 9:
	{
		if (IRSensorData[8] == 1)
		//if((SensorData[8] == 1)&&(abs(RadToDeg(current_pose.orientation)) < SECOND_LEVEL_45))
		//if((SensorData[8] == 1)&&(abs(RadToDeg(current_pose.orientation)) < SECOND_LEVEL_90))
		{
			WheelBrake();
			cleanstrategy.obstacle_avoidance_case = 0;
			cleanstrategy.route_case = 12;
			break;
		}
		else if ((abs((int)RadToDeg(current_pose.orientation)) < SECOND_LEVEL_90) && (IRSensorData[8] == 0))
		{
			WheelBrake();
			cleanstrategy.obstacle_avoidance_case = 0;
			cleanstrategy.route_case = 11;
		}
		else
		{
			
			cleanstrategy.linear_velocity = cleanstrategy.set_turn_velocity; //turn_vel;
			cleanstrategy.angular_velocity = -RadToDeg((cleanstrategy.linear_velocity) / (cleanstrategy.set_turn_radius)); //linear_velocity/l;
			
			if ((obstacleSignal == FRONT_OBSTACLE_SIGNAL) || (obstacleSignal == RIGHT_OBSTACLE_SIGNAL) || (obstacleSignal == LEFT_OBSTACLE_SIGNAL))
			{
				WheelBrake(); // Slow down
				cleanstrategy.slowdown_pose = current_pose;
				cleanstrategy.last_wheel_pulse_l = wheel_pulse_l;
				cleanstrategy.last_wheel_pulse_r = wheel_pulse_r;
				cleanstrategy.route_case = 8;
			}
			else if (abs((int)RadToDeg(current_pose.orientation)) < LEVEL_0)
			{
				WheelBrake();
				cleanstrategy.route_case = 0;
			}
		}
		break;
	}
	case 10:
	{
		cleanstrategy.linear_velocity = cleanstrategy.set_turn_velocity; //turn_vel;
		cleanstrategy.angular_velocity = -RadToDeg((cleanstrategy.linear_velocity) / (cleanstrategy.set_turn_radius)); //linear_velocity/l;
			
		if ((obstacleSignal == FRONT_OBSTACLE_SIGNAL) || (obstacleSignal == RIGHT_OBSTACLE_SIGNAL) || (obstacleSignal == LEFT_OBSTACLE_SIGNAL))
		{
			WheelBrake(); // Slow down
			cleanstrategy.slowdown_pose = current_pose;
			cleanstrategy.last_wheel_pulse_l = wheel_pulse_l;
			cleanstrategy.last_wheel_pulse_r = wheel_pulse_r;
			cleanstrategy.set_turn_velocity = TURN_VEL;
			cleanstrategy.set_turn_radius = TURN_RADIUS;
			cleanstrategy.route_case = 8;
		}
		else if (abs((int)RadToDeg(current_pose.orientation)) < LEVEL_0)
		{
			WheelBrake();
			cleanstrategy.route_case = 0;
		}
		break;
	}
	
	case 11:
	{
		//cout << " case 11 " << endl;
		double delt_y_distance = current_pose.y - cleanstrategy.last_adjustment_pose.y;
		//double off_delt_y = delt_y_distance - DELT_Y_DISTANCE;
		if(delt_y_distance >= DELT_Y_DISTANCE)
		//if (abs((int)off_delt_y) <= OFF_DELT_Y_DISTANCE)
		{
			WheelBrake();
			cleanstrategy.obstacle_avoidance_case = 0;
			cleanstrategy.route_case = 12;
		}
		else
		{
			cleanstrategy.linear_velocity = DEFAULT_STRAIGHT_VEL;
			cleanstrategy.angular_velocity = 0;
		}
		break;
	}
	case 12:
	{
		//cout << "-------------- case 12 ------------" << endl;
		if (abs((int)RadToDeg(current_pose.orientation)) < LEVEL_0)
		{
			WheelBrake();
			cleanstrategy.route_case = 0;
			cleanstrategy.is_one_task = 0;
		}
		if ((abs((int)RadToDeg(current_pose.orientation)) > LEVEL_180))
		{
			WheelBrake();
			cleanstrategy.route_case = 6;
			// if the same task : 2019.10.09
			cleanstrategy.is_one_task = 1;
		}
		else
		{
			ir_adc_x_sensordata = IRSensorData[8];
			ObstacleAvoidanceStrategy(ir_adc_x_sensordata, obstacleSignal, ir_adc315_signal, current_pose, wheel_pulse_l, wheel_pulse_r, -1);
			//SecondObstacleAvoidanceStrategy( linear_velocity, angular_velocity, ObstacleSensorData, obstacleSignal, Obstacle_adc_Signal,  current_pose, wheel_pulse_l, wheel_pulse_r,-1);
		}
		break;
	}
	default:
	{
		cleanstrategy.route_case = 0;
		cleanstrategy.is_one_task = 0;
		//cout << "Unknown  Type " << endl;
	}
		//cout << " i = " << i << endl;
		i = i + 1;
	}
	
	WheelControl((short)cleanstrategy.linear_velocity,(short)cleanstrategy.angular_velocity);
	
	cleanstrategy.last_pose = current_pose;
	
}


