#include "bsp.h"


static UploadMap uploadMap;
static MapInfo mapInfo[PER_UPLOAD_POINT_CNT];

/*
*********************************************************************************************************
*	�� �� ��: bsp_GetMapInfoPt
*	����˵��: ���ص�ͼ��ָ�룬���ָ����  PER_UPLOAD_POINT_CNT  ��Ԫ��
*	��    ��: ��
*	�� �� ֵ: ��
*********************************************************************************************************
*/
MapInfo* bsp_GetMapInfoPt(void)
{
	return mapInfo;
}


void bsp_FillMapInfo(void)
{
	/*��������ͼ���� mapInfo */
	
	
}

/*
*********************************************************************************************************
*	�� �� ��: bsp_StartUploadMap
*	����˵��: �����ϱ���ͼ
*	��    ��: ��
*	�� �� ֵ: ��
*********************************************************************************************************
*/
void bsp_StartUploadMap(void)
{
	uint8_t i = 0 ;
	
	/*������Ϣȫ����ʼ��Ϊ������Ϣ*/
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
*	�� �� ��: bsp_StopUploadMap
*	����˵��: �ر��ϱ���ͼ
*	��    ��: ��
*	�� �� ֵ: ��
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
*	�� �� ��: bsp_UploadMap
*	����˵��: �ϱ���ͼ
*	��    ��: ��
*	�� �� ֵ: ��
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
			/*�Ѿ����ӵ��Ʒ�������*/
			if(mcu_get_wifi_work_state() == WIFI_CONN_CLOUD)
			{
				bsp_OpenStreamService();
			    DEBUG("����������\r\n");
				
				uploadMap.action++;
			}
		}break;
		
		case 1:
		{
			/*�����ͼ���ݣ�1�ֽڶ���Ľṹ������ תunsigned char*  */
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

