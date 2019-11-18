#ifdef A
#ifndef __BSP_CLEANSTRATEGY_H
#define __BSP_CLEANSTRATEGY_H

#include <stdbool.h>

typedef struct POSE{
    float x;
    float y;
    float orientation;}POSE;


typedef struct
{
	int route_case ;//= 0; // global variable -- 0, 1, 2,
	int adjustment_pose_case;// = 0;
	int obstacle_avoidance_case;// = 0;
	struct POSE slowdown_pose;
	struct POSE last_adjustment_pose;
	struct POSE last_pose;
	double last_wheel_pulse_l;// = 0;
	double last_wheel_pulse_r;// = 0;
	float linear_velocity;
	float angular_velocity;
	int set_turn_velocity;// = NONE_OBSTACLE_TURN_VEL;
	int set_turn_radius;// = NONE_OBSTACLE_TURN_R;
	int left_gostright_task;// = 0;
	int is_one_task;// = 0;
	
	/*×´Ì¬»ú*/
	volatile unsigned char action ;
	volatile bool isRunning ;
	volatile unsigned int delay ;
}CleanStrategy;

void bsp_StartUpdateCleanStrategy(void);

void bsp_StopUpdateCleanStrategy(void);

void bsp_CleanStrategyUpdate(int robotX,int robotY,double robotTheta,unsigned char obstacleSignal, int wheel_pulse_l, int wheel_pulse_r, unsigned char IRSensorData[]);

#endif

#endif
	
