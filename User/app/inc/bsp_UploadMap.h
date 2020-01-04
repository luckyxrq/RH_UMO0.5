#ifndef __BSP_UPLOADMAP_H
#define __BSP_UPLOADMAP_H

#define CUR_POS             (uint8_t)0x00    /*��ǰ��*/
#define OBSTACLE_POS        (uint8_t)0x01    /*�ϰ���*/
#define CLEANED_POS         (uint8_t)0x02    /*����ɨ*/
#define CHARGING_PILE_POS   (uint8_t)0x03    /*���׮*/
#define RESERVE_POS         (uint8_t)0x04    /*����*/


#define UPLOAD_MAP_INTERVAL      500 /*�����ͼ��ʱ����*/
#define PER_UPLOAD_POINT_CNT     81   /*ÿ���ϴ���ĸ���*/


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
