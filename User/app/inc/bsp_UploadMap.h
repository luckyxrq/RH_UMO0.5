#ifndef __BSP_UPLOADMAP_H
#define __BSP_UPLOADMAP_H

#define CUR_POS             (uint8_t)0x00    /*当前点*/
#define OBSTACLE_POS        (uint8_t)0x01    /*障碍物*/
#define CLEANED_POS         (uint8_t)0x02    /*已清扫*/
#define CHARGING_PILE_POS   (uint8_t)0x03    /*充电桩*/
#define RESERVE_POS         (uint8_t)0x04    /*保留*/


#define UPLOAD_MAP_INTERVAL      500 /*传输地图的时间间隔*/
#define PER_UPLOAD_POINT_CNT     81   /*每次上传点的个数*/


#pragma pack(1)
typedef struct
{
	uint8_t x ; 
	uint8_t y ;
	uint8_t posInfo;
}MapInfo;
#pragma pack()

typedef struct
{
	volatile bool isRunning;
	volatile uint32_t delay;
	volatile uint8_t action;
	
	uint16_t id;
	uint32_t offset;
	
}UploadMap;

void bsp_StartUploadMap(void);
void bsp_StopUploadMap(void);
void bsp_UploadMap(void);

#endif
