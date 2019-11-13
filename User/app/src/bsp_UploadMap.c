#include "bsp.h"


static UploadMap uploadMap;
static MapInfo mapInfo[PER_UPLOAD_POINT_CNT];

/*
*********************************************************************************************************
*	函 数 名: bsp_GetMapInfoPt
*	功能说明: 返回地图的指针，这个指针有  PER_UPLOAD_POINT_CNT  个元素
*	形    参: 无
*	返 回 值: 无
*********************************************************************************************************
*/
MapInfo* bsp_GetMapInfoPt(void)
{
	return mapInfo;
}


void bsp_FillMapInfo(void)
{
	/*下面填充地图数据 mapInfo */
	
	
}

/*
*********************************************************************************************************
*	函 数 名: bsp_StartUploadMap
*	功能说明: 开启上报地图
*	形    参: 无
*	返 回 值: 无
*********************************************************************************************************
*/
void bsp_StartUploadMap(void)
{
	uint8_t i = 0 ;
	
	/*将点信息全部初始化为保留信息*/
	for(i=0;i<PER_UPLOAD_POINT_CNT;i++)
	{
		mapInfo[i].posInfo = CLEANED_POS;
	}
	
	uploadMap.action = 0 ;
	uploadMap.delay = 0 ;
	uploadMap.isRunning = true;
}

/*
*********************************************************************************************************
*	函 数 名: bsp_StopUploadMap
*	功能说明: 关闭上报地图
*	形    参: 无
*	返 回 值: 无
*********************************************************************************************************
*/
void bsp_StopUploadMap(void)
{
	uploadMap.isRunning = false;
	uploadMap.action = 0 ;
	uploadMap.delay = 0 ;
	
}

/*
*********************************************************************************************************
*	函 数 名: bsp_UploadMap
*	功能说明: 上报地图
*	形    参: 无
*	返 回 值: 无
*********************************************************************************************************
*/
void bsp_UploadMap(void)
{
	if(!uploadMap.isRunning)
		return;
	
	switch(uploadMap.action)
	{
		case 0:
		{
			/*已经连接到云服务器了*/
			if(mcu_get_wifi_work_state() == WIFI_CONN_CLOUD)
			{
				bsp_OpenStreamService();
			    DEBUG("开启流服务\r\n");
				
				uploadMap.action++;
			}
		}break;
		
		case 1:
		{
			/*传输地图数据，1字节对齐的结构体数组 转unsigned char*  */
			stream_trans(uploadMap.id, uploadMap.offset++ , (unsigned char*)mapInfo, PER_UPLOAD_POINT_CNT*3);
			uploadMap.delay = xTaskGetTickCount();
			uploadMap.action++;
		}break;
		
		case 2:
		{
			if(xTaskGetTickCount() - uploadMap.delay >= UPLOAD_MAP_INTERVAL)
			{
				uploadMap.action = 1 ;
			}
		}break;
	}		
}

