#include "bsp.h"
#include <math.h>

#define STRATEGY_DEBUG	 0 //0 关闭串口调试

#if STRATEGY_DEBUG	
#define gridmap_debug(format, ...) printf (format, ##__VA_ARGS__)
#else
#define gridmap_debug(format, ...) 
#endif

CLIFFADCVALUE cliff_valueB;
GridMap gridmap;
static int map_last_robotX = 0, map_last_robotY = 0;
static int8_t map_update = 0;

static MapInfo TuYa_map[PER_UPLOAD_POINT_CNT] = {0};

static double my_abs(double x){
    if (x<0){
        x= -x;
    }
    return x;
}

static bool obstacle_cliff_status_cls_flag = false;

static signed char Under_extreme_point_x[10] = {0};
static signed char Under_extreme_point_x_index = 0;
static signed char Under_extreme_point_y[10]={0};
static signed char Under_extreme_point_y_index=0;
static signed char On_extreme_point_x[10] = {0};
static signed char On_extreme_point_x_index = 0;
static signed char On_extreme_point_y[10] = {0};
static signed char On_extreme_point_y_index = 0;
static signed char Left_Under_extreme_point_x[10] = {0};
static signed char Left_Under_extreme_point_x_index = 0;
static signed char Left_On_extreme_point_x[10] = {0};
static signed char Left_On_extreme_point_x_index = 0;
static signed char Left_Under_extreme_point_y[10]={0};
static signed char Left_Under_extreme_point_y_index;
static signed char Left_On_extreme_point_y[10] = {0};
static signed char Left_On_extreme_point_y_index = 0;

static unsigned char inverseSensorModelB(unsigned char grid_x,unsigned char grid_y,int x,int y,short theta,int xi,int yi,unsigned char obstacleSignal,int grid_dist,CLIFFADCVALUE *cliff_value)
{
    int o_x = x;
    int o_y = y;
    int r=grid_dist;
    short phi= (short)(180*(atan2(yi - o_y,xi - o_x)/3.1415926));
    short theta_phi=phi-theta;
	
	UNUSED(r);
	
	
    if(my_abs(theta_phi)>180){
        if(theta_phi>180){
            theta_phi=theta_phi-360;
        }else{
            theta_phi=theta_phi+360;
        }
    }
    if(my_abs(theta_phi)>65){
        return gridmap.map[grid_x%MAPMAXCELLS][grid_y%MAPMAXCELLS];
    }
    else{
        if(cliff_value->cliffValue0==1){
            if((cliff_value->cliffValue2==1)&&(my_abs(theta_phi)<45)){
                //cout << "----------------- The front is cliff -------------------------" << endl;
                return 0;
            }
            else if((cliff_value->cliffValue1==1&&cliff_value->cliffValue3==1)&&(my_abs(theta_phi)<65)){
                //cout << "----------------- The left and the right is cliff -------------------------" << endl;
                return 0;
            }
            else if((cliff_value->cliffValue1==1)&&(0<theta_phi)&&(theta_phi<65)){
                //cout << "----------------- The left is cliff -------------------------" << endl;
                return 0;
            }
//            else if((cliff_value->cliffValue1==1)&&(r<260)&&(45<theta_phi)&&(theta_phi<=65)){
                //cout << "----------------- The left is cliff -------------------------" << endl;
//                return 0;
//            }
            else if((cliff_value->cliffValue3==1)&&(-65<theta_phi)&&(theta_phi<0)){
                //cout << "----------------- The right is cliff -------------------------" << endl;
                return 0;
            }
//            else if((cliff_value->cliffValue3==1)&&(r<260)&&(-65<=theta_phi)&&(theta_phi<-45)){
                //cout << "----------------- The right is cliff -------------------------" << endl;
//                return 0;
//            }
            else{
                return gridmap.map[grid_x%MAPMAXCELLS][grid_y%MAPMAXCELLS];
            }
        }
        else{
            if((obstacleSignal == front_obstacle)&&(my_abs(theta_phi)<45)){
                //cout << "----------------- The front is locc -------------------------" << endl;
                return 0;// Front
            }
            else if((obstacleSignal == left_obstacle)&&(0<theta_phi)&&(theta_phi<=65)){
                //cout << "----------------- The left is locc -------------------------" << endl;
                return 0; // Left
            }
//            else if((obstacleSignal == left_obstacle)&&(r<260)&&(45<theta_phi)&&(theta_phi<=65)){
                //cout << "----------------- The left is locc -------------------------" << endl;
//                return 0; // Left
//            }
            else if((obstacleSignal == right_obstacle)&&(-65<=theta_phi)&&(theta_phi<0)){
                //cout << "----------------- The right is locc -------------------------" << endl;
                return 0;   //right
            }
//            else if((obstacleSignal == right_obstacle)&&(r<260)&&(-65<=theta_phi)&&(theta_phi<-45)){
                //cout << "----------------- The right is locc -------------------------" << endl;
//                return 0;   //right
//            }
            else{
                return gridmap.map[grid_x%MAPMAXCELLS][grid_y%MAPMAXCELLS];
            }
        }
    }
}








//static void GridToXY( int* x_point,  int* y_point, int* x_grid,  int* y_grid) {
//    
//    *x_point = *x_grid * GRIDWIDTH + GRIDWIDTH / 2;
//    *y_point = *y_grid * GRIDHEIGHT + GRIDHEIGHT / 2;

//}
//static void XYToGrid( int* x_point,  int* y_point, int* x_grid,  int* y_grid) {
//    
//    *x_grid = (*x_point  + GRIDWIDTH / 2) / GRIDWIDTH;
//    *y_grid = (*y_point  + GRIDHEIGHT / 2) / GRIDHEIGHT;
//    
//}







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
	unsigned char grid_index_x,grid_index_y;
	gridmap.grid_default = 125;
	gridmap.grid_occupancy = 0;
	gridmap.grid_half_occupancy = 1;
	gridmap.grid_free = 250;
	gridmap.obstacle_distance_from_robot_center=120;
	gridmap.free_zone_from_robot_center=120;
	gridmap.collision_sensor_installation_angle_on_robot=Deg2Rad(45);
	gridmap.refresh_zone_max_radius=300;
	gridmap.refresh_zone_min_radius=120;
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

void bsp_GridMapUpdate(int robotX,int robotY, double robotTheta, unsigned char obstacleSignal,unsigned char IRSensorData[],CLIFFADCVALUE * cliff_value) {
    
	//int grid_dist;
    //unsigned char temporary_x,temporary_y,x,y;
    //int map_robot_x,map_robot_y,xi, yi;
	//int grid_index_x,grid_index_y;
	short grid_dist;
	//signed char temporary_x,temporary_y,x,y;
	int8_t map_update_x_range_max_index,map_update_y_range_max_index;
	int8_t map_update_x_range_min_index,map_update_y_range_min_index;
	int8_t x,y;
	int map_robot_x,map_robot_y,xi, yi;
		
	
	if(obstacleSignal==3||cliff_value->cliffValue0 == 0){
		obstacle_cliff_status_cls_flag = true;
	}
	
	if ((abs(map_last_robotX - robotX) >150 || abs(map_last_robotY - robotY) >100) || ((obstacleSignal!=3 || cliff_value->cliffValue0 == 1)&&obstacle_cliff_status_cls_flag)){
		gridmap.action = 0;
		map_last_robotX = robotX;
		map_last_robotY = robotY;
		gridmap_debug("map_last_robotX - robotX) >100 ,ready update map!  \n");
		gridmap_debug("###########################################################################!  \n");
		gridmap_debug("robotX：%d   robotY：%d   robotTheta：%d  \n",robotX,robotY,(int)Rad2Deg(robotTheta));
		gridmap_debug("###########################################################################!  \n");
		obstacle_cliff_status_cls_flag = false;
	}else 
	{
		gridmap.action = 1;
		//gridmap_debug("gridmap.action = 1  !  \n");
	}

	
	if(map_update==0)
	{
		//gridmap_debug(" fisrt update!  \n");
		gridmap.action = 0;
		map_update = 1;
	}
	
	if(!gridmap.isRunning)
	{
		//gridmap_debug(" gridmap is not Running !  \n");
		return ;
	}

	
	switch(gridmap.action)
	{
		case 0:
		{ 
			//gridmap_debug(" gridmap.action == 0 !  \n");
			map_last_robotX=robotX;
			map_last_robotY=robotY;
			map_robot_x=robotX+half_map_wide;
			map_robot_y=robotY+half_map_long;
			if(map_robot_x/GRIDWIDTH>96){
				map_update_x_range_max_index=MAPWIDECELLS;
			}
			else{
				map_update_x_range_max_index=map_robot_x/GRIDWIDTH+4;
			}
			if(map_robot_y/GRIDHEIGHT>96){
				map_update_y_range_max_index=MAPLONGCELLS;
			}
			else{
				map_update_y_range_max_index=map_robot_y/GRIDHEIGHT+4;
			}
			
			if(map_robot_x/GRIDWIDTH >=3) map_update_x_range_min_index=map_robot_x/GRIDWIDTH-3;
			else map_update_x_range_min_index =0;
			if(map_robot_y/GRIDHEIGHT >=3) map_update_y_range_min_index=map_robot_y/GRIDWIDTH-3;
			else map_update_y_range_min_index =0;
			
			for (x=map_update_x_range_min_index; x<map_update_x_range_max_index; x++){
				for (y=map_update_y_range_min_index; y<map_update_y_range_max_index; y++){
					xi = x * GRIDWIDTH+50;
					yi = y * GRIDHEIGHT+50;
					grid_dist = sqrt(pow(map_robot_x - xi, 2) + pow(map_robot_y - yi, 2));
					if(obstacleSignal==none_obstacle&&cliff_value->cliffValue0==0){
						if(grid_dist<=map_robot_radius){
							gridmap.map[x%MAPMAXCELLS][y%MAPMAXCELLS]=250;
						}
					}
					else{
						if(grid_dist<=map_robot_radius){
							gridmap.map[x%MAPMAXCELLS][y%MAPMAXCELLS]=250;
						}
						if((grid_dist <= 300)&&(grid_dist>map_robot_radius)){
							if(gridmap.map[x%MAPMAXCELLS][y%MAPMAXCELLS]==0){
							}
							else{
								gridmap_debug(" inverseSensorModelB()!  \n");
								gridmap.map[x%MAPMAXCELLS][y%MAPMAXCELLS] = inverseSensorModelB(x,y,map_robot_x,\
								map_robot_y,Rad2Deg(robotTheta), xi, yi, obstacleSignal,grid_dist,cliff_value);
							}
						}
					}
				}
			}
			
			for ( x = 0; x < MAPWIDTH/GRIDWIDTH; x++)
            {
                for ( y = 0; y < MAPHEIGHT/GRIDHEIGHT; y++)
                {
                        if(gridmap.map[x][y] == gridmap.grid_default) gridmap_debug("-");
                        if(gridmap.map[x][y] == gridmap.grid_occupancy) gridmap_debug("*");
                        if(gridmap.map[x][y] == gridmap.grid_free) gridmap_debug("#");
                        //if(gridmap.map[grid_index_x][grid_index_y] == 6) DEBUG("??");
                }
                gridmap_debug("\r\n");
            }
			break;
		}
		case 1:
			break;
		default:
			break;
	}
}


const unsigned char*  bsp_Get_GridMap(int robotX,int robotY)
{
	int grid_index_x,grid_index_y;
	int map_robot_x,map_robot_y;
	int min_x,min_y,max_x,max_y;
	int i = 0;
	//unsigned char current_x = 0,current_y = 0;
	
	map_robot_x  = robotX + ROBOTXOFFSET;
	map_robot_y  = robotY + ROBOTYOFFSET;
	
	grid_index_x = (map_robot_x  + GRIDWIDTH / 2) / GRIDWIDTH;
	grid_index_y = (map_robot_y  + GRIDWIDTH / 2) / GRIDWIDTH;
	
	
	min_x = grid_index_x - 4;
	min_y = grid_index_y - 4;
	max_x = grid_index_x + 4;
	max_y = grid_index_y + 4;
	
	if(min_x < 0) min_x =0;
	if(min_y < 0) min_y =0;
	
	if(max_x > 99) max_x =99;
	if(max_y > 99) max_y =99;
	
//	current_x = grid_index_x - 1;
//	current_y = grid_index_y - 2;
//	TuYa_map[0].x = current_x;
//	TuYa_map[0].y = current_y;
//	if(gridmap.map[current_x][current_y] == gridmap.grid_default) TuYa_map[0].posInfo = RESERVE_POS;
//	else if (gridmap.map[current_x][current_y] == gridmap.grid_occupancy) TuYa_map[0].posInfo = OBSTACLE_POS;
//	else if (gridmap.map[current_x][current_y] == gridmap.grid_free) TuYa_map[0].posInfo = CLEANED_POS;
//	
//	current_x = grid_index_x - 2;
//	current_y = grid_index_y - 1;
//	TuYa_map[1].x = current_x;
//	TuYa_map[1].y = current_y;
//	if(gridmap.map[current_x][current_y] == gridmap.grid_default) TuYa_map[1].posInfo = RESERVE_POS;
//	else if (gridmap.map[current_x][current_y] == gridmap.grid_occupancy) TuYa_map[1].posInfo = OBSTACLE_POS;
//	else if (gridmap.map[current_x][current_y] == gridmap.grid_free) TuYa_map[1].posInfo = CLEANED_POS;

//	current_x = grid_index_x - 2;
//	current_y = grid_index_y ;
//	TuYa_map[2].x = current_x;
//	TuYa_map[2].y = current_y;
//	if(gridmap.map[current_x][current_y] == gridmap.grid_default) TuYa_map[2].posInfo = RESERVE_POS;
//	else if (gridmap.map[current_x][current_y] == gridmap.grid_occupancy) TuYa_map[2].posInfo = OBSTACLE_POS;
//	else if (gridmap.map[current_x][current_y] == gridmap.grid_free) TuYa_map[2].posInfo = CLEANED_POS;
//	
//	current_x = grid_index_x - 2;
//	current_y = grid_index_y + 1;
//	TuYa_map[3].x = current_x;
//	TuYa_map[3].y = current_y;
//	if(gridmap.map[current_x][current_y] == gridmap.grid_default) TuYa_map[3].posInfo = RESERVE_POS;
//	else if (gridmap.map[current_x][current_y] == gridmap.grid_occupancy) TuYa_map[3].posInfo = OBSTACLE_POS;
//	else if (gridmap.map[current_x][current_y] == gridmap.grid_free) TuYa_map[3].posInfo = CLEANED_POS;

//	current_x = grid_index_x - 1;
//	current_y = grid_index_y + 2;
//	TuYa_map[4].x = current_x;
//	TuYa_map[4].y = current_y;
//	if(gridmap.map[current_x][current_y] == gridmap.grid_default) TuYa_map[4].posInfo = RESERVE_POS;
//	else if (gridmap.map[current_x][current_y] == gridmap.grid_occupancy) TuYa_map[4].posInfo = OBSTACLE_POS;
//	else if (gridmap.map[current_x][current_y] == gridmap.grid_free) TuYa_map[4].posInfo = CLEANED_POS;

//	current_x = grid_index_x ;
//	current_y = grid_index_y - 1;
//	TuYa_map[5].x = current_x;
//	TuYa_map[5].y = current_y;
//	if(gridmap.map[current_x][current_y] == gridmap.grid_default) TuYa_map[5].posInfo = RESERVE_POS;
//	else if (gridmap.map[current_x][current_y] == gridmap.grid_occupancy) TuYa_map[5].posInfo = OBSTACLE_POS;
//	else if (gridmap.map[current_x][current_y] == gridmap.grid_free) TuYa_map[5].posInfo = CLEANED_POS;
//	
//	current_x = grid_index_x ;
//	current_y = grid_index_y + 1;
//	TuYa_map[6].x = current_x;
//	TuYa_map[6].y = current_y;
//	if(gridmap.map[current_x][current_y] == gridmap.grid_default) TuYa_map[6].posInfo = RESERVE_POS;
//	else if (gridmap.map[current_x][current_y] == gridmap.grid_occupancy) TuYa_map[6].posInfo = OBSTACLE_POS;
//	else if (gridmap.map[current_x][current_y] == gridmap.grid_free) TuYa_map[6].posInfo = CLEANED_POS;
//	
//	current_x = grid_index_x ;
//	current_y = grid_index_y ;
//	TuYa_map[7].x = current_x;
//	TuYa_map[7].y = current_y;
//	TuYa_map[7].posInfo = CUR_POS;
	
	
	
	
	for ( grid_index_x = min_x; grid_index_x <= max_x; grid_index_x++)
	{
		for ( grid_index_y = min_y; grid_index_y <= max_y; grid_index_y++)
		{
			TuYa_map[i].x = grid_index_x;
			TuYa_map[i].y = grid_index_y;
			if(gridmap.map[grid_index_x][grid_index_y] == gridmap.grid_default) TuYa_map[i].posInfo = RESERVE_POS;
			else if (gridmap.map[grid_index_x][grid_index_y] == gridmap.grid_occupancy) TuYa_map[i].posInfo = OBSTACLE_POS;
			else if (gridmap.map[grid_index_x][grid_index_y] == gridmap.grid_free) TuYa_map[i].posInfo = OBSTACLE_POS;//CLEANED_POS;
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
	
	
	bsp_GetAllIrIsObstacle(IRSensorData);
	
//	((bsp_GetInfraRedAdcVoltage(IR0)*100 - OBSTACLE_INFRARED_ADC_THRESHOLD_VALUE_FROM07) >=0)?(IRSensorData[0]=1):(IRSensorData[0]=0);
//	((bsp_GetInfraRedAdcVoltage(IR1)*100 - OBSTACLE_INFRARED_ADC_THRESHOLD_VALUE_FROM07) >=0)?(IRSensorData[1]=1):(IRSensorData[1]=0);
//	((bsp_GetInfraRedAdcVoltage(IR2)*100 - OBSTACLE_INFRARED_ADC_THRESHOLD_VALUE_FROM07) >=0)?(IRSensorData[2]=1):(IRSensorData[2]=0);
//	((bsp_GetInfraRedAdcVoltage(IR3)*100 - OBSTACLE_INFRARED_ADC_THRESHOLD_VALUE_FROM07) >=0)?(IRSensorData[3]=1):(IRSensorData[3]=0);
//	((bsp_GetInfraRedAdcVoltage(IR4)*100 - OBSTACLE_INFRARED_ADC_THRESHOLD_VALUE_FROM07) >=0)?(IRSensorData[4]=1):(IRSensorData[4]=0);
//	((bsp_GetInfraRedAdcVoltage(IR5)*100 - OBSTACLE_INFRARED_ADC_THRESHOLD_VALUE_FROM07) >=0)?(IRSensorData[5]=1):(IRSensorData[5]=0);
//	((bsp_GetInfraRedAdcVoltage(IR6)*100 - OBSTACLE_INFRARED_ADC_THRESHOLD_VALUE_FROM07) >=0)?(IRSensorData[6]=1):(IRSensorData[6]=0);
//  ((bsp_GetInfraRedAdcVoltage(IR7)*100 - OBSTACLE_INFRARED_ADC_THRESHOLD_VALUE_FROM07) >=0)?(IRSensorData[7]=1):(IRSensorData[7]=0);
//  ((bsp_GetInfraRedAdcVoltage(IR8)*100 - OBSTACLE_INFRARED_ADC_THRESHOLD_VALUE_FROM89) >=0)?(IRSensorData[8]=1):(IRSensorData[8]=0);
//	((bsp_GetInfraRedAdcVoltage(IR9)*100 - OBSTACLE_INFRARED_ADC_THRESHOLD_VALUE_FROM89) >=0)?(IRSensorData[9]=1):(IRSensorData[9]=0);
//	
	return IRSensorData;

}

CLIFFADCVALUE* bsp_GetCliffSensorData(void)
{
	cliff_valueB.cliffValue0 =0;
	if(bsp_CliffIsDangerous(CliffLeft))  
	{	
		cliff_valueB.cliffValue0 =1;
		cliff_valueB.cliffValue1 = 1;
	}else cliff_valueB.cliffValue1 = 0;
	if(bsp_CliffIsDangerous(CliffMiddle))  
	{	
		cliff_valueB.cliffValue0 =1;
		cliff_valueB.cliffValue2 = 1;
	}else cliff_valueB.cliffValue2 = 0;
	if(bsp_CliffIsDangerous(CliffRight))  
	{	
		cliff_valueB.cliffValue0 =1;
		cliff_valueB.cliffValue3 = 1;
	}else cliff_valueB.cliffValue3 = 0;
	
	return &cliff_valueB;

}






///*! \brief Square root routine.
// *
// * sqrt routine 'grupe', from comp.sys.ibm.pc.programmer
// * Subject: Summary: SQRT(int) algorithm (with profiling)
// *    From: warwick@cs.uq.oz.au (Warwick Allison)
// *    Date: Tue Oct 8 09:16:35 1991
// *
// *  \param x  Value to find square root of.
// *  \return  Square root of x.
// */
//static unsigned long mysqrt(unsigned long x)
//{
//  register unsigned long xr;  // result register
//  register unsigned long q2;  // scan-bit register
//  register unsigned char f;   // flag (one bit)

//  xr = 0;                     // clear result
//  q2 = 0x40000000L;           // higest possible result bit
//  do
//  {
//    if((xr + q2) <= x)
//    {
//      x -= xr + q2;
//      f = 1;                  // set flag
//    }
//    else{
//      f = 0;                  // clear flag
//    }
//    xr >>= 1;
//    if(f){
//      xr += q2;               // test flag
//    }
//  } while(q2 >>= 2);          // shift twice
//  if(xr < x){
//    return xr +1;             // add for rounding
//  }
//  else{
//    return xr;
//  }
//}



//
short Edge_length(void){
    bool end_x=false;
    short edgelength=0;
    int8_t i,j,firsttrap=0;
    for (i=0;i<GRIDWIDTH;i++) {
        for(j=0;j<GRIDHEIGHT;j++){
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
    for (i=GRIDWIDTH-1;i>=0;i--){
        for(j=0;j<GRIDHEIGHT;j++){
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

short __bsp_Right_ReturnExtreme_point(int robotX,int robotY,int robotTheta,unsigned char obstacleSignal)
{
    short y_boundary;
    short x_boundary;
    bool end_x=false;
    bool firsttrap=false;
    short Extreme_point = 0;
    short i=0,j=0;
    short t=0,z=0;

    short Leaksweep=0;
    bool leak;
    y_boundary=(robotY+half_map_wide)/100 ;
    x_boundary=(robotX+half_map_long)/100;
	
	y_boundary = y_boundary%100;
	x_boundary = x_boundary%100;
	
    if(my_abs(robotTheta)>170){
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
        if((y_boundary<47)&&(Extreme_point!=0)){
            for ( i=1;i<100;i++) {
                for( j=46;j>=y_boundary;j--){
                    if(gridmap.map[i][j]==250||gridmap.map[i][j]==0){
                        if(Extreme_point-i<-2){
                            if(x_boundary-i<-2){
                                if((j-y_boundary)>3){
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
                                                    if (Under_extreme_point_y[t]-x_boundary>10)
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
    else if(my_abs(robotTheta)<10){
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
                                if((j-y_boundary)>3){
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
                                                    if (On_extreme_point_y[t]-x_boundary>10)
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

short __bsp_Left_ReturnExtreme_point(int robotX,int robotY,int robotTheta,unsigned char obstacleSignal)
{
    short y_boundary;
    short x_boundary;
    bool end_x=false;
    bool firsttrap=false;
    short Extreme_point = 0;
    short i=0,j=0;
    short t=0,z=0;

    short Leaksweep;
    bool leak;
    y_boundary=(robotY+half_map_wide)/100;
    x_boundary=(robotX+half_map_long)/100;
	
	y_boundary = y_boundary%5000;
	x_boundary = x_boundary%5000;
	
    if(my_abs(robotTheta)>170){
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
                for( j=1;j<=y_boundary;j++){
                    if(gridmap.map[i][j]==250||gridmap.map[i][j]==0){
                        if(Extreme_point-i<-2){
                            if(x_boundary-i<-2){
                                if((y_boundary-j)>3){
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
                                                    if (Left_Under_extreme_point_y[t]-x_boundary>10)
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
    else if(my_abs(robotTheta)<10){
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
                for( j=1;j<=y_boundary;j++){
                    if(gridmap.map[i][j]==250||gridmap.map[i][j]==0){
                        if(Extreme_point-i>2){
                            if(x_boundary-i>2){
                                if((y_boundary-j)>3){
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
                                                    if (Left_On_extreme_point_y[t]-x_boundary>10)
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







short bsp_Right_ReturnExtreme_point(int robotX,int robotY,int robotTheta,unsigned char obstacleSignal)
{
	  unsigned char y_boundary=0;
    unsigned char x_boundary=0;
    bool end_x=false;
    bool firsttrap=false;
    unsigned char Extreme_point = 0;
    unsigned char i=0,j=0;
    unsigned char t=0,z=0;
    short Leaksweep=0;
    bool leak;
	  y_boundary=(robotY+half_map_wide)/100;
    x_boundary=(robotX+half_map_long)/100;
	  if(y_boundary<3||x_boundary<3||y_boundary>96||x_boundary>96){
			return 0;
		}
    if(my_abs(robotTheta)>170){
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
                        if(Extreme_point-i<-4){
                            if(x_boundary-i<-4){
                                if((j-y_boundary)>2){
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
                                        if(Under_extreme_point_y[0]==0){
                                            Under_extreme_point_y[0]=j;
                                            Under_extreme_point_y_index++;
                                            Under_extreme_point_x[0]=x_boundary;
                                            Under_extreme_point_x_index++;
                                            for (z=y_boundary;z<j;z++) {
                                                gridmap.map[x_boundary-2][z]=0;
                                            }
                                            return  Leaksweep;
                                        }
                                        else{
                                            leak=true;
                                            for( t=0;t<Under_extreme_point_y_index;t++){
                                                if(Under_extreme_point_y[t]==(j-2)||Under_extreme_point_y[t]==(j-1)||Under_extreme_point_y[t]==j||
                                                        Under_extreme_point_y[t]==(j+1)||Under_extreme_point_y[t]==(j+2)){
                                                    if (Under_extreme_point_x[t]-x_boundary>10)
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
                                                Under_extreme_point_y[Under_extreme_point_y_index]=j;
                                                Under_extreme_point_y_index++;
                                                Under_extreme_point_x[Under_extreme_point_x_index]=x_boundary;
                                                Under_extreme_point_x_index++;
                                                for (z=y_boundary;z<j;z++) {
                                                    gridmap.map[x_boundary-2][z]=0;
                                                }
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
    else if(my_abs(robotTheta)<10){
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
                        if(Extreme_point-i>4){
                            if(x_boundary-i>4){
                                if((j-y_boundary)>2){
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
                                        if(On_extreme_point_y[0]==0){
                                            On_extreme_point_y[0]=j;
                                            On_extreme_point_y_index++;
                                            On_extreme_point_x[0] = x_boundary;
                                            On_extreme_point_x_index++;
                                            for (z=y_boundary;z<j;z++) {
                                                gridmap.map[x_boundary+2][z]=0;
                                            }
                                            return Leaksweep;
                                        }
                                        else{
                                            leak=true;
                                            for( t=0;t<On_extreme_point_y_index;t++){
                                                if(On_extreme_point_y[t]==(j-2)||On_extreme_point_y[t]==(j-1)||On_extreme_point_y[t]==j
                                                        ||On_extreme_point_y[t]==(j+1)||On_extreme_point_y[t]==(j+2)){
                                                    if (x_boundary-On_extreme_point_x[t]>10)
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
                                                On_extreme_point_y[On_extreme_point_y_index]=j;
                                                On_extreme_point_y_index++;
                                                On_extreme_point_x[Under_extreme_point_x_index]=x_boundary;
                                                On_extreme_point_x_index++;
                                                for (z=y_boundary;z<j;z++) {
                                                    gridmap.map[x_boundary+2][z]=0;
                                                }
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
	
	
//	short y_boundary;
//    short x_boundary;
//    bool end_x=false;
//    bool firsttrap=false;
//    short Extreme_point = 0;
//    short i=0,j=0;
//    short t=0,z=0;

//    short Leaksweep=0;
//    bool leak;
//	
//	
//	
//    y_boundary=(robotY+half_map_wide)/100;
//    x_boundary=(robotX+half_map_long)/100;
//	y_boundary = y_boundary%100 ;
//	x_boundary = x_boundary%100 ;
//    if(my_abs(robotTheta)>170){
//        for ( i=0;i<100;i++) {
//            for( j=y_boundary;j<=50;j++){
//                if(gridmap.map[i][j]==250||gridmap.map[i][j]==0){
//                    Extreme_point=i;
//                    end_x=true;
//                    break;
//                }
//            }
//            if(end_x==true){
//                end_x=false;
//                break;
//            }
//        }
//        if((y_boundary<47)&&(Extreme_point!=0)){
//            for ( i=1;i<100;i++) {
//                for( j=46;j>=y_boundary;j--){
//                    if(gridmap.map[i][j]==250||gridmap.map[i][j]==0){
//                        if(Extreme_point-i<-4){
//                            if(x_boundary-i<-4){
//                                if((j-y_boundary)>5){
//                                    for( z=i-1;z>0;z--){
//                                        if(gridmap.map[z][j]==125){
//                                            firsttrap=true;
//                                        }
//                                        else{
//                                            firsttrap=false;
//                                            break;
//                                        }
//                                    }
//                                    if(firsttrap==true){
//                                        Extreme_point=0;
//                                        Leaksweep=100*(j-y_boundary+1);
//                                        if(Under_extreme_point_y[0]==0){
//                                            Under_extreme_point_y[0]=j;
//                                            Under_extreme_point_y_index++;
//                                            Under_extreme_point_x[0]=x_boundary;
//                                            Under_extreme_point_x_index++;
//                                            return  Leaksweep;
//                                        }
//                                        else{
//                                            leak=true;
//                                            for( t=0;t<Under_extreme_point_y_index;t++){
//                                                if(Under_extreme_point_y[t]==(j-2)||Under_extreme_point_y[t]==(j-1)||Under_extreme_point_y[t]==j||
//                                                        Under_extreme_point_y[t]==(j+1)||Under_extreme_point_y[t]==(j+2)){
//                                                    if (Under_extreme_point_x[t]-x_boundary>10)
//                                                    {
//                                                        break;
//                                                    }
//                                                    else
//                                                    {
//                                                        leak=false;
//                                                        break;
//                                                    }
//                                                }
//                                            }
//                                            if(leak==true){
//                                                Under_extreme_point_y[Under_extreme_point_y_index]=j;
//                                                Under_extreme_point_y_index++;
//                                                Under_extreme_point_x[Under_extreme_point_x_index]=x_boundary;
//                                                Under_extreme_point_x_index++;
//                                                return Leaksweep;
//                                            }
//                                        }
//                                    }
//                                }
//                            }
//                        }
//                    }
//                }
//            }
//        }
//    }
//    else if(my_abs(robotTheta)<10){
//        for( i=99;i>0;i--){
//            for( j=47;j>=y_boundary;j--){
//                if(gridmap.map[i][j]==250||gridmap.map[i][j]==0){
//                    Extreme_point=i;
//                    end_x=true;
//                    break;
//                }
//            }
//            if(end_x==true){
//                end_x=false;
//                break;
//            }
//        }
//        if((y_boundary<47)&&(Extreme_point!=0)){
//            for( i=99;i>0;i--){
//                for( j=46;j>=y_boundary;j--){
//                    if(gridmap.map[i][j]==250||gridmap.map[i][j]==0){
//                        if(Extreme_point-i>4){
//                            if(x_boundary-i>4){
//                                if((j-y_boundary)>5){
//                                    for( z=i+1;z<99;z++){
//                                        if(gridmap.map[z][j]==125){
//                                            firsttrap=true;
//                                        }
//                                        else{
//                                            firsttrap=false;
//                                            break;
//                                        }
//                                    }
//                                    if(firsttrap==true){
//                                        Extreme_point=0;
//                                        Leaksweep=100*(j-y_boundary+1);
//                                        if(On_extreme_point_y[0]==0){
//                                            On_extreme_point_y[0]=j;
//                                            On_extreme_point_y_index++;
//                                            On_extreme_point_x[0] = x_boundary;
//                                            On_extreme_point_x_index++;
//                                            return Leaksweep;
//                                        }
//                                        else{
//                                            leak=true;
//                                            for( t=0;t<On_extreme_point_y_index;t++){
//                                                if(On_extreme_point_y[t]==(j-2)||On_extreme_point_y[t]==(j-1)||On_extreme_point_y[t]==j
//                                                        ||On_extreme_point_y[t]==(j+1)||On_extreme_point_y[t]==(j+2)){
//                                                    if (x_boundary-On_extreme_point_x[t]>10)
//                                                    {
//                                                        break;
//                                                    }
//                                                    else
//                                                    {
//                                                        leak=false;
//                                                        break;
//                                                    }
//                                                }
//                                            }
//                                            if(leak==true){
//                                                On_extreme_point_y[On_extreme_point_y_index]=j;
//                                                On_extreme_point_y_index++;
//                                                On_extreme_point_x[Under_extreme_point_x_index]=x_boundary;
//                                                On_extreme_point_x_index++;
//                                                return Leaksweep;
//                                            }
//                                        }
//                                    }
//                                }
//                            }
//                        }
//                    }
//                }
//            }
//        }
//    }
//    else{
//        return 0;
//    }
//    return 0;
//}
//
short bsp_Left_ReturnExtreme_point(int robotX,int robotY,int robotTheta,unsigned char obstacleSignal)
{
		unsigned char y_boundary;
    unsigned char x_boundary;
    bool end_x=false;
    bool firsttrap=false;
    unsigned char Extreme_point = 0;
    unsigned char i=0,j=0;
    unsigned char t=0,z=0;
    short Leaksweep=0;
    bool leak;
    y_boundary=(robotY+half_map_wide)/100;
    x_boundary=(robotX+half_map_long)/100;
	  if(y_boundary<3||x_boundary<3||y_boundary>96||x_boundary>96){
			return 0;
		}
    if(my_abs(robotTheta)>170){
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
                for( j=rightmapmin;j<=y_boundary;j++){
                    if(gridmap.map[i][j]==250||gridmap.map[i][j]==0){
                        if(Extreme_point-i<-4){
                            if(x_boundary-i<-4){
                                if((y_boundary-j)>2){
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
                                        if(Left_Under_extreme_point_y[0]==0){
                                            Left_Under_extreme_point_y[0] = j;
                                            Left_Under_extreme_point_y_index++;
                                            Left_Under_extreme_point_x[0] = x_boundary;
                                            Left_Under_extreme_point_x_index++;
                                            for (z=j;z<y_boundary;z++) {
                                                gridmap.map[x_boundary-2][z]=0;
                                            }
                                            return  Leaksweep;
                                        }
                                        else{
                                            leak=true;
                                            for( t=0;t<Left_Under_extreme_point_y_index;t++){
                                                if(Left_Under_extreme_point_y[t]==(j-2)||Left_Under_extreme_point_y[t]==(j-1)||Left_Under_extreme_point_y[t]==j||
                                                        Left_Under_extreme_point_y[t]==(j+1)||Left_Under_extreme_point_y[t]==(j+2)||Left_Under_extreme_point_y[t]==(j+3)||Left_Under_extreme_point_y[t]==(j+4)){
                                                    if (Left_Under_extreme_point_x[t]-x_boundary>10)
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
                                                Left_Under_extreme_point_y[Left_Under_extreme_point_y_index] = j;
                                                Left_Under_extreme_point_y_index++;
                                                Left_Under_extreme_point_x[Left_Under_extreme_point_x_index] = x_boundary;
                                                Left_Under_extreme_point_x_index++;
                                                for (z=j;z<y_boundary;z++) {
                                                    gridmap.map[x_boundary-2][z]=0;
                                                }
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
    else if(my_abs(robotTheta)<10){
        for( i=99;i>0;i--){
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
            for( i=99;i>0;i--){
                for( j=rightmapmax;j<=y_boundary;j++){
                    if(gridmap.map[i][j]==250||gridmap.map[i][j]==0){
                        if(Extreme_point-i>4){
                            if(x_boundary-i>4){
                                if((y_boundary-j)>2){
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
                                        if(Left_On_extreme_point_y[0]==0){
                                            Left_On_extreme_point_y[0] = j;
                                            Left_On_extreme_point_x[0] = x_boundary;
                                            Left_On_extreme_point_x_index++;
                                            for (z=j;z<y_boundary;z++) {
                                                gridmap.map[x_boundary+2][z]=0;
                                            }
                                            return Leaksweep;
                                        }
                                        else{
                                            leak=true;
                                            for( t=0;t<Left_On_extreme_point_y_index;t++){
                                                if(Left_On_extreme_point_y[t]==(j-2)||Left_On_extreme_point_y[t]==(j-1)||Left_On_extreme_point_y[t]==j
                                                        ||Left_On_extreme_point_y[t]==(j+1)||Left_On_extreme_point_y[t]==(j+2)||Left_On_extreme_point_y[t]==(j+3)||Left_On_extreme_point_y[t]==(j+4)){
                                                    if (x_boundary-Left_On_extreme_point_x[t]>10)
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
                                                Left_On_extreme_point_y[Left_On_extreme_point_y_index] = j;
                                                Left_On_extreme_point_y_index++;
                                                Left_On_extreme_point_x[Left_On_extreme_point_x_index] = x_boundary;
                                                Left_On_extreme_point_x_index++;
                                                for (z=j;z<y_boundary;z++) {
                                                    gridmap.map[x_boundary+2][z]=0;
                                                }
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
		
	
//    short y_boundary;
//    short x_boundary;
//    bool end_x=false;
//    bool firsttrap=false;
//    short Extreme_point = 0;
//    short i=0,j=0;
//    short t=0,z=0;
//    short Leaksweep;
//    bool leak;
//	
//	
//	
//    y_boundary=(robotY+half_map_wide)/100;
//    x_boundary=(robotX+half_map_long)/100;
//	y_boundary = y_boundary%100 ;
//	x_boundary = x_boundary%100 ;
//    if(my_abs(robotTheta)>170){
//        for ( i=0;i<100;i++) {
//            for( j=51;j<=y_boundary;j++){
//                if(gridmap.map[i][j]==250||gridmap.map[i][j]==0){
//                    Extreme_point=i;
//                    end_x=true;
//                    break;
//                }
//            }
//            if(end_x==true){
//                end_x=false;
//                break;
//            }
//        }
//        if((y_boundary>51)&&(Extreme_point!=0)){
//            for ( i=1;i<100;i++) {
//                for( j=1;j<=y_boundary;j++){
//                    if(gridmap.map[i][j]==250||gridmap.map[i][j]==0){
//                        if(Extreme_point-i<-4){
//                            if(x_boundary-i<-4){
//                                if((y_boundary-j)>5){
//                                    for( z=i-1;z>0;z--){
//                                        if(gridmap.map[z][j]==125){
//                                            firsttrap=true;
//                                        }
//                                        else{
//                                            firsttrap=false;
//                                            break;
//                                        }
//                                    }
//                                    if(firsttrap==true){
//                                        Extreme_point=0;
//                                        Leaksweep=100*(y_boundary-j+1);
//                                        if(Left_Under_extreme_point_y[0]==0){
//                                            Left_Under_extreme_point_y[0] = j;
//                                            Left_Under_extreme_point_y_index++;
//                                            Left_Under_extreme_point_x[0] = x_boundary;
//                                            Left_Under_extreme_point_x_index++;
//                                            return  Leaksweep;
//                                        }
//                                        else{
//                                            leak=true;
//                                            for( t=0;t<Left_Under_extreme_point_y_index;t++){
//                                                if(Left_Under_extreme_point_y[t]==(j-2)||Left_Under_extreme_point_y[t]==(j-1)||Left_Under_extreme_point_y[t]==j||
//                                                        Left_Under_extreme_point_y[t]==(j+1)||Left_Under_extreme_point_y[t]==(j+2)||
//												Left_Under_extreme_point_y[t]==(j+3)||Left_Under_extreme_point_y[t]==(j+4)){
//                                                    if (Left_Under_extreme_point_x[t]-x_boundary>10)
//                                                    {
//                                                        break;
//                                                    }
//                                                    else
//                                                    {
//                                                        leak=false;
//                                                        break;
//                                                    }
//                                                }
//                                            }
//                                            if(leak==true){
//                                                Left_Under_extreme_point_y[Left_Under_extreme_point_y_index] = j;
//                                                Left_Under_extreme_point_y_index++;
//                                                Left_Under_extreme_point_x[Left_Under_extreme_point_x_index] = x_boundary;
//                                                Left_Under_extreme_point_x_index++;
//                                                return Leaksweep;
//                                            }
//                                        }
//                                    }
//                                }
//                            }
//                        }
//                    }
//                }
//            }
//        }
//    }
//    else if(my_abs(robotTheta)<10){
//        for( i=99;i>=0;i--){
//            for( j=50;j<=y_boundary;j++){
//                if(gridmap.map[i][j]==250||gridmap.map[i][j]==0){
//                    Extreme_point=i;
//                    end_x=true;
//                    break;
//                }
//            }
//            if(end_x==true){
//                end_x=false;
//                break;
//            }
//        }
//        if((y_boundary>51)&&(Extreme_point!=0)){
//            for( i=99;i>=0;i--){
//                for( j=1;j<=y_boundary;j++){
//                    if(gridmap.map[i][j]==250||gridmap.map[i][j]==0){
//                        if(Extreme_point-i>4){
//                            if(x_boundary-i>4){
//                                if((y_boundary-j)>5){
//                                    for( z=i+1;z<99;z++){
//                                        if(gridmap.map[z][j]==125){
//                                            firsttrap=true;
//                                        }
//                                        else{
//                                            firsttrap=false;
//                                            break;
//                                        }
//                                    }
//                                    if(firsttrap==true){
//                                        Extreme_point=0;
//                                        Leaksweep=100*(y_boundary-j+1);
//                                        if(Left_On_extreme_point_y[0]==0){
//                                            Left_On_extreme_point_y[0] = j;
//                                            Left_On_extreme_point_x[0] = x_boundary;
//                                            Left_On_extreme_point_x_index++;
//                                            return Leaksweep;
//                                        }
//                                        else{
//                                            leak=true;
//                                            for( t=0;t<Left_On_extreme_point_y_index;t++){
//                                                if(Left_On_extreme_point_y[t]==(j-2)||Left_On_extreme_point_y[t]==(j-1)||Left_On_extreme_point_y[t]==j
//                                                        ||Left_On_extreme_point_y[t]==(j+1)||Left_On_extreme_point_y[t]==(j+2)||Left_On_extreme_point_y[t]==(j+3)||Left_On_extreme_point_y[t]==(j+4)){
//                                                    if (x_boundary-Left_On_extreme_point_x[t]>10)
//                                                    {
//                                                        break;
//                                                    }
//                                                    else
//                                                    {
//                                                        leak=false;
//                                                        break;
//                                                    }
//                                                }
//                                            }
//                                            if(leak==true){
//                                                Left_On_extreme_point_y[Left_On_extreme_point_y_index] = j;
//                                                Left_On_extreme_point_y_index++;
//                                                Left_On_extreme_point_x[Left_On_extreme_point_x_index] = x_boundary;
//                                                Left_On_extreme_point_x_index++;
//                                                return Leaksweep;
//                                            }
//                                        }
//                                    }
//                                }
//                            }
//                        }
//                    }
//                }
//            }
//        }
//    }
//    else{
//        return 0;
//    }
//    return 0;
//}








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







