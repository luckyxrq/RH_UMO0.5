#ifndef __BSP_GRIDMAP_H
#define __BSP_GRIDMAP_H

#include <stdbool.h>

/** Grid Size (mm) */
// 10cm * 10cm
#define GRIDWIDTH  100
#define GRIDHEIGHT 100
/** Map Size (mm) */ 
// 10m * 10m
#define MAPWIDTH  10000
#define MAPHEIGHT 10000

#define ROBOTXOFFSET   MAPWIDTH/2
#define ROBOTYOFFSET   MAPHEIGHT/2

#define FRONT_OBSTACLE_SIGNAL 2
#define LEFT_OBSTACLE_SIGNAL 0
#define RIGHT_OBSTACLE_SIGNAL 1
#define NONE_OBSTACLE_SIGNAL 3

#define REFRESH_ZONE_SIZE 3



#define OBSTACLE_INFRARED_ADC_THRESHOLD_VALUE_FROM07 250 
#define OBSTACLE_INFRARED_ADC_THRESHOLD_VALUE_FROM89 50



#define CUR_POS             (uint8_t)0x00    /*当前点*/
#define OBSTACLE_POS        (uint8_t)0x01    /*障碍物*/
#define CLEANED_POS         (uint8_t)0x02    /*已清扫*/
#define CHARGING_PILE_POS   (uint8_t)0x03    /*充电桩*/
#define RESERVE_POS         (uint8_t)0x04    /*保留*/


#pragma pack(1)
typedef struct 
{
	unsigned char x ; 
	unsigned char y ;
	unsigned char posInfo;
}MapInfo;
#pragma pack()

// Initialize GridMapping
//l0,  locc,  lfree,  alpha,  alpha1,  beta,  Zmax,  Zmin,  sensorType
//l0     :  Init default grid map data   0.5
//locc   :  Occupancy grid map data      1
//lfree  :  free grid map data           0
//alpha  :  obstacle distance from robot center to draw the grid map   170mm
//alpha1 :  free zone from robot center to draw the grid map           140mm
//beta   :  sensor installation angle on robot  45°
//Zmax   :  refresh zone max radius  300mm
//Zmin   :  refresh zone min radius  170mm
//sensorType: collision or  infrared


typedef struct
{
	unsigned char grid_default;
	unsigned char grid_occupancy;
	unsigned char grid_half_occupancy;
	unsigned char grid_free;
	short obstacle_distance_from_robot_center;
	short free_zone_from_robot_center;
	//short collision_sensor_installation_angle_on_robot;
	double collision_sensor_installation_angle_on_robot;
	short refresh_zone_max_radius;
	short refresh_zone_min_radius;
	short sensor_type;
	volatile unsigned char map[MAPWIDTH/GRIDWIDTH][MAPHEIGHT/GRIDHEIGHT];
	/*状态机*/
	volatile unsigned char action ;
	volatile bool isRunning ;
	volatile unsigned int delay ;
}GridMap;

void bsp_StartUpdateGridMap(void);
void bsp_StopUpdateGridMap(void);
void bsp_GridMapUpdate(int robotX,int robotY,double robotTheta, unsigned char obstacleSignal,unsigned char IRSensorData[]);
unsigned char* bsp_GetIRSensorData(void);
//int* bsp_GetGridMap(void);

int bsp_Edge_length(void);

int bsp_Right_ReturnExtreme_point(int robotX,int robotY,int robotTheta,unsigned char obstacleSignal);
int bsp_Left_ReturnExtreme_point(int robotX,int robotY,int robotTheta,unsigned char obstacleSignal);
int bsp_Leakingsweep(void);

const unsigned char*  bsp_Get_GridMap(int robotX,int robotY);




#endif



