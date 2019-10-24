#include "bsp.h"
#include <math.h>

static GridMap gridmap;

static unsigned long mysqrt(unsigned long x);

static unsigned char inverseSensorModel(int robotXY_from_gridXY_dist,int robotX,int robotY,double robotTheta,int grid_real_center_x,int grid_real_center_y,unsigned char obstacleSignal,unsigned char* IRSensorData)
{

	double thetaK;
	double sensorTheta;
	
	int SensorData_signal = 0;
	int SensorData_signal_flag = 0;
	
	int r = robotXY_from_gridXY_dist;
	double phi = atan2(grid_real_center_y - robotY, grid_real_center_x - robotX) - robotTheta;
	unsigned char IR_index;
	 

	switch (gridmap.sensor_type)
	{
		case 0:
			
			if (obstacleSignal == NONE_OBSTACLE_SIGNAL) // Free
			{
				sensorTheta = 0;
			}
			else if (obstacleSignal == RIGHT_OBSTACLE_SIGNAL) // The barriers in right front
			{
				sensorTheta = -gridmap.collision_sensor_installation_angle_on_robot; //-45;
			}
			else if (obstacleSignal == LEFT_OBSTACLE_SIGNAL) // The barriers in left front
			{
				sensorTheta = gridmap.collision_sensor_installation_angle_on_robot; //45;
			}
			else if (obstacleSignal == FRONT_OBSTACLE_SIGNAL) // The barriers in front
			{
				sensorTheta = 0;
			}
			if ((obstacleSignal == FRONT_OBSTACLE_SIGNAL) &&\
				(r < gridmap.refresh_zone_max_radius && r >= gridmap.obstacle_distance_from_robot_center) &&\
				(fabs(phi - sensorTheta) <= gridmap.collision_sensor_installation_angle_on_robot))
			{ 
				return gridmap.grid_occcupancy; // Front
			}
			else if ((obstacleSignal == LEFT_OBSTACLE_SIGNAL) &&\
				(r < gridmap.refresh_zone_max_radius && r >= gridmap.obstacle_distance_from_robot_center) &&\
				(fabs(phi - sensorTheta) <= gridmap.collision_sensor_installation_angle_on_robot))
			{ 
				return gridmap.grid_occcupancy; // Left
			}
			else if ((obstacleSignal == RIGHT_OBSTACLE_SIGNAL) &&\
				(r < gridmap.refresh_zone_max_radius && r >= gridmap.obstacle_distance_from_robot_center) &&\
				(fabs(phi - sensorTheta) <= gridmap.collision_sensor_installation_angle_on_robot))
			{
				return gridmap.grid_occcupancy; // Right
			}
			else if ((obstacleSignal == NONE_OBSTACLE_SIGNAL) && (r < gridmap.free_zone_from_robot_center))
			{
				return gridmap.grid_free;
			}
			else
			{
				return gridmap.grid_default;
			}
			break;
		case 1: // IR Laser
			/**  65.5 52.5 35.7 17.3 0 0 -17.3 -35.7 -52.5 -65.5  */
			for (IR_index = 0; IR_index < 10; IR_index++)
			{
				if (IR_index == 0)
				{
					sensorTheta = Deg2Rad(52.5);
					SensorData_signal = IRSensorData[0];
				}
				else if (IR_index == 1)
				{
					sensorTheta = Deg2Rad(35.7);
					SensorData_signal = IRSensorData[1];
				}
				else if (IR_index == 2)
				{
					sensorTheta = Deg2Rad(17.3);
					SensorData_signal = IRSensorData[2];
				}
				else if (IR_index == 3)
				{
					sensorTheta = Deg2Rad(0);
					SensorData_signal = IRSensorData[3];
				}
				else if (IR_index == 4)
				{
					sensorTheta = Deg2Rad(-17.3);
					SensorData_signal = IRSensorData[4];
				}
				else if (IR_index == 5)
				{
					sensorTheta = Deg2Rad(-35.7);
					SensorData_signal = IRSensorData[5];
				}
				else if (IR_index == 6)
				{
					sensorTheta = Deg2Rad(-52.5);
					SensorData_signal = IRSensorData[6];
				}
				else if (IR_index == 7)
				{ // Head
					sensorTheta = Deg2Rad(0);
					SensorData_signal = IRSensorData[7];
				}
				else if (IR_index == 8)
				{
					sensorTheta = Deg2Rad(65.5);
					SensorData_signal = IRSensorData[8];
				}
				else if (IR_index == 9)
				{
					sensorTheta = Deg2Rad(-65.5);
					SensorData_signal = IRSensorData[9];
				}

				//if (fabs(phi - sensorTheta) < minDelta || minDelta == -1) {
				if (fabs(phi - sensorTheta) < gridmap.collision_sensor_installation_angle_on_robot)
				{ //deg:10(Rad 0.17)
					//Zk = SensorData_signal;//SensorData[i];
					SensorData_signal_flag = SensorData_signal;
					thetaK = sensorTheta;
					//minDelta = fabs(phi - sensorTheta);
				}
			}
			if (SensorData_signal_flag == 1)
			{
				if ((r < gridmap.refresh_zone_max_radius && r >= gridmap.obstacle_distance_from_robot_center) &&\
					(fabs(phi - thetaK) <= gridmap.collision_sensor_installation_angle_on_robot))
				{ //deg:10(Rad 0.17)
					return gridmap.grid_occcupancy;
				}
				else if (r < gridmap.obstacle_distance_from_robot_center)
				{
					return gridmap.grid_free;
				}
			}
			else if (r < gridmap.free_zone_from_robot_center)
			{
				return gridmap.grid_free; // -2.2
			}
			else
			{
				return gridmap.grid_default;
			}
			break;
		default:
			DEBUG(" Unknown Sensor Type ");
	}
}


static void GridToXY( int* x_point,  int* y_point, int* x_grid,  int* y_grid) {
    
    *x_point = *x_grid * GRIDWIDTH + GRIDWIDTH / 2 - ROBOTXOFFSET;
    *y_point = -(*y_grid * GRIDHEIGHT + GRIDHEIGHT / 2 ) + ROBOTXOFFSET;

}
static void XYToGrid( int* x_point,  int* y_point, int* x_grid,  int* y_grid) {
    
    *x_grid = (*x_point + ROBOTXOFFSET - GRIDWIDTH / 2) / GRIDWIDTH;
    *y_grid = (-*y_point + ROBOTXOFFSET - GRIDHEIGHT / 2) / GRIDHEIGHT;
    
}



/*
*********************************************************************************************************
*	函 数 名: bsp_StartUpdateMap
*	功能说明: 开启周期性的更新地图
*	形    参: 无
*	返 回 值: 无
*********************************************************************************************************
*/
void bsp_StartUpdateGridMap(void)
{
	int grid_index_x,grid_index_y;
	gridmap.grid_default = 1;
	gridmap.grid_occcupancy = 2;
	gridmap.grid_free = 0;
	gridmap.obstacle_distance_from_robot_center=170;
	gridmap.free_zone_from_robot_center=140;
	gridmap.collision_sensor_installation_angle_on_robot=Deg2Rad(45);
	gridmap.refresh_zone_max_radius=300;
	gridmap.refresh_zone_min_radius=170;
	gridmap.sensor_type=0;
	
	for (grid_index_x = 0; grid_index_x < MAPWIDTH / GRIDWIDTH; grid_index_x++)
	{ // The number of cells
		for (grid_index_y = 0; grid_index_y < MAPHEIGHT / GRIDHEIGHT; grid_index_y++)
		{
			gridmap.map[grid_index_x][grid_index_y] = gridmap.grid_default; // initialization
		}
	}
	
	gridmap.action = 0 ;
	gridmap.delay = 0 ;
	gridmap.isRunning = true;
	
	
}


/*
*********************************************************************************************************
*	函 数 名: bsp_StartUpdateMap
*	功能说明: 关闭周期性的更新地图
*	形    参: 无
*	返 回 值: 无
*********************************************************************************************************
*/
void bsp_StopUpdateGridMap(void)
{
	gridmap.isRunning = false;
	gridmap.action = 0 ;
	gridmap.delay = 0 ;
}
/*
*********************************************************************************************************
*	函 数 名: bsp_MapUpdate
*	功能说明: 周期性的更新地图
*	形    参: 无
*	返 回 值: 无
*********************************************************************************************************
*/
void bsp_GridMapUpdate(int robotX,int robotY,double robotTheta, unsigned char obstacleSignal,unsigned char IRSensorData[])
{
	int grid_real_center_x, grid_real_center_y;
	int grid_index_x,grid_index_y;
	int robotXY_from_gridXY_dist;
	unsigned char grid_status;
	int min_x,min_y,max_x,max_y;
	
	
	UNUSED(min_y);
	
	if(!gridmap.isRunning)
	{
		return ;
	}
	
	switch(gridmap.action)
	{
		case 0:
		{
			robotX += X_BIAS;
			robotY += Y_BIAS;
			
			XYToGrid(&robotX, &robotY,&grid_index_x, &grid_index_y);
			
			min_x = grid_index_x - REFRESH_ZONE_SIZE;
			min_y = grid_index_y - REFRESH_ZONE_SIZE;
			max_x = grid_index_x + REFRESH_ZONE_SIZE;
			max_y = grid_index_y + REFRESH_ZONE_SIZE;
			
			
			for ( grid_index_x = min_x; grid_index_x < max_x; grid_index_x++)
			{
				for ( grid_index_y = min_x; grid_index_y < max_y; grid_index_y++)
				{
					GridToXY( &grid_real_center_x,  &grid_real_center_y, &grid_index_x, &grid_index_y);
//					grid_real_center_x = grid_index_x * GRIDWIDTH + GRIDWIDTH / 2 - ROBOTXOFFSET;
//					grid_real_center_y = -(grid_index_y * GRIDHEIGHT + GRIDHEIGHT / 2) + ROBOTYOFFSET;
					
					robotXY_from_gridXY_dist = mysqrt(pow(grid_real_center_x - robotX, 2) + pow(grid_real_center_y - robotY, 2));
					
					if (robotXY_from_gridXY_dist <= gridmap.refresh_zone_max_radius)
					{
						grid_status = inverseSensorModel(robotXY_from_gridXY_dist,robotX, robotY, robotTheta, grid_real_center_x, grid_real_center_y, obstacleSignal, IRSensorData);
						
						if (grid_status == gridmap.grid_default)
						{
							gridmap.map[grid_index_x][grid_index_y] = gridmap.map[grid_index_x][grid_index_y];
						}
						else
							gridmap.map[grid_index_x][grid_index_y] = grid_status;
					}
				}
			}
					

		};
		case 1:
		{

		}break;
		
		case 2:
		{

		}break;
	}
	
 
}

unsigned char* bsp_GetIRSensorData(void)
{
	
	static unsigned char IRSensorData[10] = {0};
	
	((bsp_GetInfraRedAdcVoltage(IR0)*100 - OBSTACLE_INFRARED_ADC_THRESHOLD_VALUE_FROM07) >=0)?(IRSensorData[0]=1):(IRSensorData[0]=0);
	((bsp_GetInfraRedAdcVoltage(IR1)*100 - OBSTACLE_INFRARED_ADC_THRESHOLD_VALUE_FROM07) >=0)?(IRSensorData[1]=1):(IRSensorData[1]=0);
	((bsp_GetInfraRedAdcVoltage(IR2)*100 - OBSTACLE_INFRARED_ADC_THRESHOLD_VALUE_FROM07) >=0)?(IRSensorData[2]=1):(IRSensorData[2]=0);
	((bsp_GetInfraRedAdcVoltage(IR3)*100 - OBSTACLE_INFRARED_ADC_THRESHOLD_VALUE_FROM07) >=0)?(IRSensorData[3]=1):(IRSensorData[3]=0);
	((bsp_GetInfraRedAdcVoltage(IR4)*100 - OBSTACLE_INFRARED_ADC_THRESHOLD_VALUE_FROM07) >=0)?(IRSensorData[4]=1):(IRSensorData[4]=0);
	((bsp_GetInfraRedAdcVoltage(IR5)*100 - OBSTACLE_INFRARED_ADC_THRESHOLD_VALUE_FROM07) >=0)?(IRSensorData[5]=1):(IRSensorData[5]=0);
	((bsp_GetInfraRedAdcVoltage(IR6)*100 - OBSTACLE_INFRARED_ADC_THRESHOLD_VALUE_FROM07) >=0)?(IRSensorData[6]=1):(IRSensorData[6]=0);
    ((bsp_GetInfraRedAdcVoltage(IR7)*100 - OBSTACLE_INFRARED_ADC_THRESHOLD_VALUE_FROM07) >=0)?(IRSensorData[7]=1):(IRSensorData[7]=0);
    ((bsp_GetInfraRedAdcVoltage(IR8)*100 - OBSTACLE_INFRARED_ADC_THRESHOLD_VALUE_FROM89) >=0)?(IRSensorData[8]=1):(IRSensorData[8]=0);
	((bsp_GetInfraRedAdcVoltage(IR9)*100 - OBSTACLE_INFRARED_ADC_THRESHOLD_VALUE_FROM89) >=0)?(IRSensorData[9]=1):(IRSensorData[9]=0);
	
	return IRSensorData;

}


/*! \brief Square root routine.
 *
 * sqrt routine 'grupe', from comp.sys.ibm.pc.programmer
 * Subject: Summary: SQRT(int) algorithm (with profiling)
 *    From: warwick@cs.uq.oz.au (Warwick Allison)
 *    Date: Tue Oct 8 09:16:35 1991
 *
 *  \param x  Value to find square root of.
 *  \return  Square root of x.
 */
static unsigned long mysqrt(unsigned long x)
{
  register unsigned long xr;  // result register
  register unsigned long q2;  // scan-bit register
  register unsigned char f;   // flag (one bit)

  xr = 0;                     // clear result
  q2 = 0x40000000L;           // higest possible result bit
  do
  {
    if((xr + q2) <= x)
    {
      x -= xr + q2;
      f = 1;                  // set flag
    }
    else{
      f = 0;                  // clear flag
    }
    xr >>= 1;
    if(f){
      xr += q2;               // test flag
    }
  } while(q2 >>= 2);          // shift twice
  if(xr < x){
    return xr +1;             // add for rounding
  }
  else{
    return xr;
  }
}
