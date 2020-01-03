#include "bsp.h"
#include <math.h>

static int map_last_robotX = 0, map_last_robotY = 0;
static char map_update = 0;
static GridMap gridmap;
static MapInfo TuYa_map[81] = {0};

static double my_abs(double x){
    if (x<0){
        x= -x;
    }
    return x;
}

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
			
			//if (obstacleSignal == NONE_OBSTACLE_SIGNAL) // Free
			//{
			//	sensorTheta = 0;
			//}
			if (obstacleSignal == RIGHT_OBSTACLE_SIGNAL) // The barriers in right front
			{
				sensorTheta = -gridmap.collision_sensor_installation_angle_on_robot; //-45;
			}
			else if (obstacleSignal == LEFT_OBSTACLE_SIGNAL) // The barriers in left front
			{
				sensorTheta = gridmap.collision_sensor_installation_angle_on_robot; //45;
			}
			//else if (obstacleSignal == FRONT_OBSTACLE_SIGNAL) // The barriers in front
			//{
			//	sensorTheta = 0;
			//}
			else 
			{
				sensorTheta = 0;
			}
			
			if(r < gridmap.free_zone_from_robot_center)
			{
				return gridmap.grid_free;
			}
			else if ((obstacleSignal == FRONT_OBSTACLE_SIGNAL) &&\
				(r < gridmap.refresh_zone_max_radius && r >= gridmap.obstacle_distance_from_robot_center) &&\
				(fabs(phi - sensorTheta) <= gridmap.collision_sensor_installation_angle_on_robot))
			{ 
				return gridmap.grid_occupancy; // Front
			}
			else if ((obstacleSignal == LEFT_OBSTACLE_SIGNAL) &&\
				(r < gridmap.refresh_zone_max_radius && r >= gridmap.obstacle_distance_from_robot_center) &&\
				(fabs(phi - sensorTheta) <= gridmap.collision_sensor_installation_angle_on_robot))
			{ 
				return gridmap.grid_occupancy; // Left
			}
			else if ((obstacleSignal == RIGHT_OBSTACLE_SIGNAL) &&\
				(r < gridmap.refresh_zone_max_radius && r >= gridmap.obstacle_distance_from_robot_center) &&\
				(fabs(phi - sensorTheta) <= gridmap.collision_sensor_installation_angle_on_robot))
			{
				return gridmap.grid_occupancy; // Right
			}
			//else if ((obstacleSignal == NONE_OBSTACLE_SIGNAL) && (r < gridmap.free_zone_from_robot_center))
			//{
			//	return gridmap.grid_free;
			//}
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
					return gridmap.grid_occupancy;
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
			//DEBUG(" Unknown Sensor Type ");
			break;
	}
	return gridmap.grid_default;
}



static unsigned char inverseSensorModelB(int  grid_x,int  grid_y,double x,double y,double theta, double xi, double yi,int obstacleSignal,double grid_dist) {
    double o_x = x;
    double o_y = y;
    double r=grid_dist;
    double phi;
    double temporary_phi;
    double temporary_theta;
    double phi_temporaryphi;
    double theta_phi = atan2(xi - o_x,yi - o_y);
    theta_phi =theta_phi*180/3.14;
    if(theta_phi<0){
        temporary_phi=theta_phi+360;
    }
    else{
        temporary_phi=theta_phi;
    }
    if(temporary_phi<270){
        temporary_phi=temporary_phi+90;
    }
    else{
        temporary_phi=temporary_phi-270;
    }
    if(theta<0){
        temporary_theta=theta+360;
    }
    else{
        temporary_theta=theta;
    }
    phi_temporaryphi=temporary_phi-temporary_theta;
	
    if(my_abs(phi_temporaryphi)>180){
        if(phi_temporaryphi>0){
            phi=phi_temporaryphi-360;
        }
        else{
            phi=phi_temporaryphi+360;
        }
    }
    else{
        phi=phi_temporaryphi;
    }
    if(phi>=0){
        phi=180-phi;
    }
    else{
        phi=-180-phi;
    }
    if(my_abs(phi)>=75){
        return 250;
    }
    else{
        if((obstacleSignal == FRONT_OBSTACLE_SIGNAL )&&(my_abs(phi)<=10)){
            return 0;
        }
        else if((obstacleSignal == LEFT_OBSTACLE_SIGNAL)&&(10<phi)&&(phi<50)){
            return 0;
        }
        else if((obstacleSignal == LEFT_OBSTACLE_SIGNAL)&&(r<260)&&(50<phi)&&(phi<75)){
            return 0;
        }
        else if((obstacleSignal == RIGHT_OBSTACLE_SIGNAL)&&(-50<phi)&&(phi<-10)){
            return 0;
        }
        else if((obstacleSignal == RIGHT_OBSTACLE_SIGNAL)&&(r<260)&&(-75<phi)&&(phi<-50)){
            return 0;
        }
        else{
            if(gridmap.map[grid_x][grid_y]==250){
                return  250;
            }
            else{
                return 125;
            }
        }
    }
}





static void GridToXY( int* x_point,  int* y_point, int* x_grid,  int* y_grid) {
    
    *x_point = *x_grid * GRIDWIDTH + GRIDWIDTH / 2;
    *y_point = *y_grid * GRIDHEIGHT + GRIDHEIGHT / 2;

}
static void XYToGrid( int* x_point,  int* y_point, int* x_grid,  int* y_grid) {
    
    *x_grid = (*x_point  + GRIDWIDTH / 2) / GRIDWIDTH;
    *y_grid = (*y_point  + GRIDHEIGHT / 2) / GRIDHEIGHT;
    
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
	gridmap.grid_default = 125;
	gridmap.grid_occupancy = 0;
	gridmap.grid_half_occupancy = 1;
	gridmap.grid_free = 250;
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
	int map_robot_x,map_robot_y;
	int robotXY_from_gridXY_dist;
	unsigned char grid_status;
	int min_x,min_y,max_x,max_y;
	
	
	if ((abs(map_last_robotX - robotX) >100 || abs(map_last_robotY - robotY) >100) || obstacleSignal!=3)
	{
		gridmap.action = 0;
		map_last_robotX = robotX;
		map_last_robotY = robotY;
	}
	else 
	{
		gridmap.action = 1;
	}
	
	if(map_update==0)
	{
		gridmap.action = 0;
		map_update = 1;
	}
	
	UNUSED(min_y);
	
	if(!gridmap.isRunning)
	{
		return ;
	}
	
	switch(gridmap.action)
	{
		case 0:
		{
			map_robot_x  = robotX + ROBOTXOFFSET;
			map_robot_y  = robotY + ROBOTYOFFSET;
			
			//XYToGrid(&map_robot_x, &map_robot_y,&grid_index_x, &grid_index_y);
			grid_index_x = (map_robot_x  + GRIDWIDTH / 2) / GRIDWIDTH;
			grid_index_y = (map_robot_y  + GRIDWIDTH / 2) / GRIDWIDTH;
			
			min_x = grid_index_x - REFRESH_ZONE_SIZE;
			min_y = grid_index_y - REFRESH_ZONE_SIZE;
			max_x = grid_index_x + REFRESH_ZONE_SIZE;
			max_y = grid_index_y + REFRESH_ZONE_SIZE;
			
			if(min_x < 0) min_x =0;
			if(min_y < 0) min_y =0;
			
			if(max_x > 99) max_x =99;
			if(max_y > 99) max_y =99;
			
			DEBUG("______obstacleSignal____________%d________________________________\r\n",obstacleSignal);
			//DEBUG("map_robot_x:%d, &map_robot_y:%d,&grid_index_x:%d, &grid_index_y:%d",map_robot_x,map_robot_y,grid_index_x, grid_index_y);
			//DEBUG("____________________________________________________________________\r\n");
			
			
			for ( grid_index_x = min_x; grid_index_x <= max_x; grid_index_x++)
			{
				for ( grid_index_y = min_y; grid_index_y <= max_y; grid_index_y++)
				{
					//GridToXY( &grid_real_center_x,  &grid_real_center_y, &grid_index_x, &grid_index_y);
					grid_real_center_x = grid_index_x * GRIDWIDTH + GRIDWIDTH / 2;
					grid_real_center_y = grid_index_y * GRIDWIDTH + GRIDWIDTH / 2;
					
					robotXY_from_gridXY_dist = mysqrt(pow(grid_real_center_x - map_robot_x, 2) + pow(grid_real_center_y - map_robot_y, 2));
					
					if(obstacleSignal==NONE_OBSTACLE_SIGNAL)
					{
						if(robotXY_from_gridXY_dist<=gridmap.free_zone_from_robot_center){
							gridmap.map[grid_index_x][grid_index_y]=gridmap.grid_free;
						}
					}else{
						if(robotXY_from_gridXY_dist<=gridmap.free_zone_from_robot_center){
							gridmap.map[grid_index_x][grid_index_y]=gridmap.grid_free;
						}
						if(robotXY_from_gridXY_dist <= gridmap.refresh_zone_max_radius && robotXY_from_gridXY_dist>gridmap.free_zone_from_robot_center)
						{
							if (gridmap.map[grid_index_x][grid_index_y] == gridmap.grid_occupancy)
							{
								;
							}
							else
							{
								//grid_status = inverseSensorModel(robotXY_from_gridXY_dist,map_robot_x, map_robot_y, robotTheta, grid_real_center_x, grid_real_center_y, obstacleSignal, IRSensorData);
								
								grid_status = inverseSensorModelB(grid_index_x,grid_index_y,map_robot_x,map_robot_y,Rad2Deg(robotTheta), grid_real_center_x, grid_real_center_y,obstacleSignal,robotXY_from_gridXY_dist);   
								
								gridmap.map[grid_index_x][grid_index_y] = grid_status;
							}
						}
					}
				}
			}
			
			//gridmap.map[(map_robot_x  + GRIDWIDTH / 2) / GRIDWIDTH][(map_robot_y  + GRIDWIDTH / 2) / GRIDWIDTH] = 6;
			
			for ( grid_index_x = 0; grid_index_x < MAPWIDTH/GRIDWIDTH; grid_index_x++)
			{
				for ( grid_index_y = 0; grid_index_y < MAPHEIGHT/GRIDHEIGHT; grid_index_y++)
				{
						if(gridmap.map[grid_index_x][grid_index_y] == gridmap.grid_default) DEBUG("-");
						if(gridmap.map[grid_index_x][grid_index_y] == gridmap.grid_occupancy) DEBUG("■");
						if(gridmap.map[grid_index_x][grid_index_y] == gridmap.grid_free) DEBUG("□");
						//if(gridmap.map[grid_index_x][grid_index_y] == 6) DEBUG("▲");
				}
				DEBUG("\r\n");
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





const unsigned char*  bsp_Get_GridMap(int robotX,int robotY)
{
	int grid_index_x,grid_index_y;
	int map_robot_x,map_robot_y;
	int min_x,min_y,max_x,max_y;
	int i = 0;
	
	map_robot_x  = robotX + ROBOTXOFFSET;
	map_robot_y  = robotY + ROBOTYOFFSET;
	
	grid_index_x = (map_robot_x  + GRIDWIDTH / 2) / GRIDWIDTH;
	grid_index_y = (map_robot_y  + GRIDWIDTH / 2) / GRIDWIDTH;
	
	min_x = grid_index_x - REFRESH_ZONE_SIZE;
	min_y = grid_index_y - REFRESH_ZONE_SIZE;
	max_x = grid_index_x + REFRESH_ZONE_SIZE;
	max_y = grid_index_y + REFRESH_ZONE_SIZE;
	
	if(min_x < 0) min_x =0;
	if(min_y < 0) min_y =0;
	
	if(max_x > 99) max_x =99;
	if(max_y > 99) max_y =99;
	
	for ( grid_index_x = min_x; grid_index_x <= max_x; grid_index_x++)
	{
		for ( grid_index_y = min_y; grid_index_y <= max_y; grid_index_y++)
		{
			TuYa_map[i].x = grid_index_x;
			TuYa_map[i].y = grid_index_y;
			if(gridmap.map[grid_index_x][grid_index_y] == gridmap.grid_default) TuYa_map[i].posInfo = RESERVE_POS;
			else if (gridmap.map[grid_index_x][grid_index_y] == gridmap.grid_occupancy) TuYa_map[i].posInfo = OBSTACLE_POS;
			else if (gridmap.map[grid_index_x][grid_index_y] == gridmap.grid_free) TuYa_map[i].posInfo = CLEANED_POS;
			i++;
		}
	}
	
	TuYa_map[40].x = (map_robot_x  + GRIDWIDTH / 2) / GRIDWIDTH;;
	TuYa_map[40].y = (map_robot_y  + GRIDWIDTH / 2) / GRIDWIDTH;;
	TuYa_map[40].posInfo = CUR_POS;
	
	return (unsigned char*)TuYa_map;
	
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



int bsp_Edge_length(void)
{
	bool end_x=false;
	int firsttrap;
	int edgelength;
	int i=0,j=0;
	
	for (i=0;i<MAPHEIGHT;i++) {
		for(j=0;j<MAPWIDTH;j++){
			if(gridmap.map[i][j]==250||gridmap.map[i][j]==0){
				firsttrap=i;
				end_x=true;
				break;
			}
		}
		if(end_x==true){
			end_x=false;
			break;
		}
	}
	
	for (i=MAPHEIGHT-1;i>0;i--){
		for(j=0;j<MAPWIDTH;j++){
			if(gridmap.map[i][j]==250||gridmap.map[i][j]==0){
				edgelength=i-firsttrap;
				edgelength=100*edgelength;
				end_x=true;
				break;
			}
		}
		if(end_x==true){
			end_x=false;
			break;
		}
	}
	
	return edgelength;
}




int bsp_Right_ReturnExtreme_point(int robotX,int robotY,int robotTheta,unsigned char obstacleSignal)
{
    int y_boundary;
    int x_boundary;
    bool end_x=false;
    bool firsttrap=false;
    int Extreme_point = 0;
    unsigned char Under_extreme_point_x[50] = {0};
    unsigned char Under_extreme_point_x_index = 0;
    
    unsigned char Under_extreme_point_y[50]={0};
    unsigned char Under_extreme_point_y_index=0;
    
    unsigned char On_extreme_point_x[50] = {0};
    unsigned char On_extreme_point_x_index = 0;
    
    unsigned char On_extreme_point_y[50] = {0};
    unsigned char On_extreme_point_y_index = 0;
    
    int i=0,j=0;
    int t=0,z=0;
    
    int Leaksweep=0;
    bool leak;
    y_boundary=(robotY+5000)/100;
    x_boundary=(robotX+5000)/100;
    if(my_abs(robotTheta/100)>170){
        for ( i=0;i<100;i++) {
            for( j=y_boundary;j<=50;j++){
                if(gridmap.map[i][j]==250||gridmap.map[i][j]==0){
                    Extreme_point=i;
                    end_x=true;
                    break;
                }
            }
            if(end_x==true){
                end_x=false;
                break;
            }
        }
        if((y_boundary<48)&&(Extreme_point!=0)){
            for ( i=1;i<100;i++) {
                for( j=47;j>=y_boundary;j--){
                    if(gridmap.map[i][j]==250||gridmap.map[i][j]==0){
                        if(Extreme_point-i<-2){
                            if(x_boundary-i<-2){
                                if((j-y_boundary)>1){
                                    for( z=i-1;z>0;z--){
                                        if(gridmap.map[z][j]==125){
                                            firsttrap=true;
                                        }
                                        else{
                                            firsttrap=false;
                                            break;
                                        }
                                    }
                                    if(firsttrap==true){
                                        Extreme_point=0;
                                        Leaksweep=100*(j-y_boundary+1);
                                        if(Under_extreme_point_x[0]==0){
                                            Under_extreme_point_x[0]=j;
                                            Under_extreme_point_x_index++;
                                            Under_extreme_point_y[0]=x_boundary;
                                            Under_extreme_point_y_index++;
                                            return  Leaksweep;
                                        }
                                        else{
                                            leak=true;
                                            for( t=0;t<Under_extreme_point_x_index;t++){
                                                if(Under_extreme_point_x[t]==(j-2)||Under_extreme_point_x[t]==(j-1)||Under_extreme_point_x[t]==j||
                                                        Under_extreme_point_x[t]==(j+1)||Under_extreme_point_x[t]==(j+2)){
                                                    if (Under_extreme_point_y[t]-x_boundary>5)
                                                    {
                                                        break;
                                                    }
                                                    else
                                                    {
                                                        leak=false;
                                                        break;
                                                    }
                                                }
                                            }
                                            if(leak==true){
                                                Under_extreme_point_x[Under_extreme_point_x_index]=j;
                                                Under_extreme_point_x_index++;
                                                Under_extreme_point_y[Under_extreme_point_y_index]=x_boundary;
                                                Under_extreme_point_y_index++;
                                                return Leaksweep;
                                            }
                                        }
                                    }
                                }
                            }
                        }
                    }
                }
            }
        }
    }
    else if(my_abs(robotTheta/100)<10){
        for( i=99;i>0;i--){
            for( j=47;j>=y_boundary;j--){
                if(gridmap.map[i][j]==250||gridmap.map[i][j]==0){
                    Extreme_point=i;
                    end_x=true;
                    break;
                }
            }
            if(end_x==true){
                end_x=false;
                break;
            }
        }
        if((y_boundary<47)&&(Extreme_point!=0)){
            for( i=99;i>0;i--){
                for( j=46;j>=y_boundary;j--){
                    if(gridmap.map[i][j]==250||gridmap.map[i][j]==0){
                        if(Extreme_point-i>2){
                            if(x_boundary-i>2){
                                if((j-y_boundary)>1){
                                    for( z=i+1;z<99;z++){
                                        if(gridmap.map[z][j]==125){
                                            firsttrap=true;
                                        }
                                        else{
                                            firsttrap=false;
                                            break;
                                        }
                                    }
                                    if(firsttrap==true){
                                        Extreme_point=0;
                                        Leaksweep=100*(j-y_boundary+1);
                                        if(On_extreme_point_x[0]==0){
                                            On_extreme_point_x[0]=j;
                                            On_extreme_point_x_index++;
                                            
                                            On_extreme_point_y[0] = x_boundary;
                                            On_extreme_point_y_index++;
                                            
                                            return Leaksweep;
                                        }
                                        else{
                                            leak=true;
                                            for( t=0;t<On_extreme_point_x_index;t++){
                                                if(On_extreme_point_x[t]==(j-2)||On_extreme_point_x[t]==(j-1)||On_extreme_point_x[t]==j
                                                        ||On_extreme_point_x[t]==(j+1)||On_extreme_point_x[t]==(j+2)){
                                                    if (On_extreme_point_y[t]-x_boundary>5)
                                                    {
                                                        break;
                                                    }
                                                    else
                                                    {
                                                        leak=false;
                                                        break;
                                                    }
                                                }
                                            }
                                            if(leak==true){
                                                On_extreme_point_x[On_extreme_point_x_index]=j;
                                                On_extreme_point_x_index++;
                                                
                                                On_extreme_point_y[Under_extreme_point_y_index]=x_boundary;
                                                On_extreme_point_y_index++;
                                                
                                                return Leaksweep;
                                            }
                                        }
                                    }
                                }
                            }
                        }
                    }
                }
            }
        }
    }
    else{
        return 0;
    }
    return 0;
}

int bsp_Left_ReturnExtreme_point(int robotX,int robotY,int robotTheta,unsigned char obstacleSignal)
{
    int y_boundary;
    int x_boundary;
    bool end_x=false;
    bool firsttrap=false;
    int Extreme_point = 0;
    
    unsigned char Left_Under_extreme_point_x[50] = {0};
    unsigned char Left_Under_extreme_point_x_index = 0;
    
    unsigned char Left_On_extreme_point_x[50] = {0};
    unsigned char Left_On_extreme_point_x_index = 0;
    
    unsigned char Left_Under_extreme_point_y[50]={0};
    unsigned char Left_Under_extreme_point_y_index;
    
    unsigned char Left_On_extreme_point_y[50] = {0};
    unsigned char Left_On_extreme_point_y_index = 0;
    
    int i=0,j=0;
    int t=0,z=0;
    
    int Leaksweep;
    bool leak;
    y_boundary=(robotY+5000)/100;
    x_boundary=(robotX+5000)/100;
    if(my_abs(robotTheta/100)>170){
        for ( i=0;i<100;i++) {
            for( j=51;j<=y_boundary;j++){
                if(gridmap.map[i][j]==250||gridmap.map[i][j]==0){
                    Extreme_point=i;
                    end_x=true;
                    break;
                }
            }
            if(end_x==true){
                end_x=false;
                break;
            }
        }
        if((y_boundary>51)&&(Extreme_point!=0)){
            for ( i=1;i<100;i++) {
                for( j=52;j<=y_boundary;j++){
                    if(gridmap.map[i][j]==250||gridmap.map[i][j]==0){
                        if(Extreme_point-i<-2){
                            if(x_boundary-i<-2){
                                if((y_boundary-j)>1){
                                    for( z=i-1;z>0;z--){
                                        if(gridmap.map[z][j]==125){
                                            firsttrap=true;
                                        }
                                        else{
                                            firsttrap=false;
                                            break;
                                        }
                                    }
                                    if(firsttrap==true){
                                        Extreme_point=0;
                                        Leaksweep=100*(y_boundary-j+1);
                                        if(Left_Under_extreme_point_x[0]==0){
                                            Left_Under_extreme_point_x[0] = j;
                                            Left_Under_extreme_point_x_index++;
                                            
                                            Left_Under_extreme_point_y[0] = x_boundary;
                                            Left_Under_extreme_point_y_index++;
                                            
                                            
                                            return  Leaksweep;
                                        }
                                        else{
                                            leak=true;
                                            for( t=0;t<Left_Under_extreme_point_x_index;t++){
                                                if(Left_Under_extreme_point_x[t]==(j-2)||Left_Under_extreme_point_x[t]==(j-1)||Left_Under_extreme_point_x[t]==j||
                                                        Left_Under_extreme_point_x[t]==(j+1)||Left_Under_extreme_point_x[t]==(j+2)){
                                                    if (Left_Under_extreme_point_y[t]-x_boundary>5)
                                                    {
                                                        break;
                                                    }
                                                    else
                                                    {
                                                        leak=false;
                                                        break;
                                                    }
                                                }
                                            }
                                            if(leak==true){
                                                Left_Under_extreme_point_x[Left_Under_extreme_point_x_index] = j;
                                                Left_Under_extreme_point_x_index++;
                                                
                                                Left_Under_extreme_point_y[Left_Under_extreme_point_y_index] = x_boundary;
                                                Left_Under_extreme_point_y_index++;
                                                return Leaksweep;
                                            }
                                        }
                                    }
                                }
                            }
                        }
                    }
                }
            }
        }
    }
    else if(my_abs(robotTheta/100)<10){
        for( i=99;i>=0;i--){
            for( j=50;j<=y_boundary;j++){
                if(gridmap.map[i][j]==250||gridmap.map[i][j]==0){
                    Extreme_point=i;
                    end_x=true;
                    break;
                }
            }
            if(end_x==true){
                end_x=false;
                break;
            }
        }
        if((y_boundary>51)&&(Extreme_point!=0)){
            for( i=99;i>=0;i--){
                for( j=52;j<=y_boundary;j++){
                    if(gridmap.map[i][j]==250||gridmap.map[i][j]==0){
                        if(Extreme_point-i>2){
                            if(x_boundary-i>2){
                                if((y_boundary-j)>1){
                                    for( z=i+1;z<99;z++){
                                        if(gridmap.map[z][j]==125){
                                            firsttrap=true;
                                        }
                                        else{
                                            firsttrap=false;
                                            break;
                                        }
                                    }
                                    if(firsttrap==true){
                                        Extreme_point=0;
                                        Leaksweep=100*(y_boundary-j+1);
                                        if(Left_On_extreme_point_x[0]==0){
                                            Left_On_extreme_point_x[0] = j;
                                            
                                            Left_On_extreme_point_y[0] = x_boundary;
                                            Left_On_extreme_point_y_index++;
                                            
                                            return Leaksweep;
                                        }
                                        else{
                                            leak=true;
                                            for( t=0;t<Left_On_extreme_point_x_index;t++){
                                                if(Left_On_extreme_point_x[t]==(j-2)||Left_On_extreme_point_x[t]==(j-1)||Left_On_extreme_point_x[t]==j
                                                        ||Left_On_extreme_point_x[t]==(j+1)||Left_On_extreme_point_x[t]==(j+2)){
                                                    if (Left_On_extreme_point_y[t]-x_boundary>5)
                                                    {
                                                        break;
                                                    }
                                                    else
                                                    {
                                                        leak=false;
                                                        break;
                                                    }
                                                }
                                            }
                                            if(leak==true){
                                                Left_On_extreme_point_x[Left_On_extreme_point_x_index] = j;
                                                Left_On_extreme_point_x_index++;
                                                Left_On_extreme_point_y[Left_On_extreme_point_y_index] = x_boundary;
                                                Left_On_extreme_point_y_index++;
                                                return Leaksweep;
                                            }
                                        }
                                    }
                                }
                            }
                        }
                    }
                }
            }
        }
    }
    else{
        return 0;
    }
    return 0;
}








//------------------------------------------------------------------------------------+++#-##--##-----
//---------------------------------------------------------------------------#----+++++++++++#-+++----
//---------------------------------------------------------------------------++--+++++++++++++++++----
//--------------------------------------------------------------------------++++-+++#+++++++++++++----
//--------------------------------------------------------------------------++++-+++#-++++++++++++----
//--------------------------------------------------------------------------++++-+++##++++++++++++----
//-------------------------------------------------------------------------++++++++++-++++++++++++----
//-------------------------------------------------------------------------+++++++++++++++++++++++----
//-------------------------------------------------------------------------+++++++++++++++++++++++----
//-------------------------------------------------------------------------+++++++++++++++++++++++----
//-------------------------------------------------------------------------+++++++++++++++++++++++----
//-------------------------------------------------------------------------+++++++++++++++++++++++----
//--------------------------------------------------------------##---------+++++++++++++++++++++++----
//--------------------------------------------------##----#++--+++---------++++++++-+##+++++++++++----
//--------------------------------------------------++#---+++-+++++--------+++++++#####+++++++++++----
//-------------------------------------------------+++---++++-+++++--------+++-+++-----+++++++++++----
//-------------------------------------------------++++--++++++++++--------++#####-----+++++++++++----
//-------------------------------------------------++++--++++++++++--------++--#-------+++++++++++----
//-------------------------------------------------++++--++++++++++--------++----------+++++++++++----
//-----------------------------------------------++++++--++++++++++--------++-----------+++-++++++----
//----------------------------------------------+++++++--++++++++++-------+++---------------++++++----
//----------------------------------------------++++++++-++++++++++-------+++---------------++++++----
//----------------------------------------------+++++++++++++++++++-------+++---------------++++++----
//----------------------------------------------+++++++++++++++++++-------+++---------------++++++----
//----------------------------------------------+++++++++++++++++++-------+++---------------++++++----
//----------------------------------------------+++++++++++++++++++-------+++---------------+++++-----
//----------------------------------------------+++++++++++++++++++-------+++---------------+++++-----
//----------------------------------------------+++++++++++++++++++-------+++---------------+++++-----
//----------------------------------------------+++++++++++++++++++-------+++---------------+++++-----
//----------------------------------------------+++++++++++++++++++-------+++---------------+++++-----
//----------------------------------------------+++++++++++++++++++------#+++---------------+++++-----
//----------------------------------------------++++-++++++++++++++----##++++---------------+++++-----
//----------------------------------------------++++-++++++++++++++---##+++++---------------+++++-----
//----------------------------------------------++++--++++--+++-+++##-#++++++---------------++++#-----
//----------------------------------------------++++--++++###+#-+++####++++++---------------++++##----
//----------------------------------------------++++--++++-###--+++++#++++++----------------+++++#----
//----------------------------------------------++++--++++-------+++++++++++--------------##+++++#----
//----------------------------------------------++++--++++-----####+++++++++--------------##++++##----
//-----------------------------------------------++#--++++----------++++++++---------------##++-#-----
//-----------------------------------------------##---++++----------+++#####---------------##++#------
//-----------------------------------------------------++-----------++++###-----------------####------
//---------------------------------------------------####------------++##-----------------------------
//------------------------------------------------------------------####------------------------------






