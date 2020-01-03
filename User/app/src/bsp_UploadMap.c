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

void bsp_SetCurPos(uint8_t i)
{
	if(i >= 1 && i <= 80)
	mapInfo[i].posInfo = CUR_POS;
	mapInfo[i-1].posInfo = CLEANED_POS;
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
		mapInfo[i].x = i;
		mapInfo[i].y = i;
		mapInfo[i].posInfo = RESERVE_POS;
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
				
				uploadMap.delay = xTaskGetTickCount();
				uploadMap.action++;
			}
		}break;
		
		
		case 1:
		{
			if(xTaskGetTickCount() - uploadMap.delay >= UPLOAD_MAP_INTERVAL)
			{
				bsp_StreamTransOpen(uploadMap.id);
				uploadMap.delay = xTaskGetTickCount();
				uploadMap.action++ ;
			}
		}break;
		
		
		case 2:
		{
			if(xTaskGetTickCount() - uploadMap.delay >= UPLOAD_MAP_INTERVAL)
			{
				uploadMap.action++ ;
			}
		}break;
		
		case 3:
		{
			bsp_SetCurPos(uploadMap.offset + 1);
			printf("���͵�ͼ����\r\n");
			/*�����ͼ���ݣ�1�ֽڶ���Ľṹ������ תunsigned char*  */
			//stream_trans(uploadMap.id, uploadMap.offset++ , (unsigned char*)bsp_Get_GridMap(bsp_GetCurrentPosX(),bsp_GetCurrentPosY()), PER_UPLOAD_POINT_CNT*3);
			stream_trans(uploadMap.id, uploadMap.offset++ , (unsigned char*)mapInfo, PER_UPLOAD_POINT_CNT*3);
			uploadMap.delay = xTaskGetTickCount();
			uploadMap.action++;
		}break;
		
		case 4:
		{
			if(xTaskGetTickCount() - uploadMap.delay >= UPLOAD_MAP_INTERVAL)
			{
				uploadMap.action = 3 ;
			}
		}break;
	}		
}


// 1 ����������
// 2 ����������
// 3 �����ͼ����
///4 ������ͼ���ݴ���
