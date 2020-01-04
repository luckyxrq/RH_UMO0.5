#include "bsp.h"

/*
*********************************************************************************************************
*	�� �� ��: bsp_WifiStateProc
*	����˵��: WIFI״̬�����ı䣬��Ӧ�Ĵ�����
*	��    �Σ���
*	�� �� ֵ: ��
*********************************************************************************************************
*/
void bsp_WifiStateProc(void)
{
	static bool isStartConnectWIFI = true;
	static bool isStartConnectClound = true;
	static bool isOpenUploadMap = false;
	
	switch(mcu_get_wifi_work_state())
	{
		case SMART_CONFIG_STATE:
		//smart config ����״̬ LED���� ��led��˸���û����
		DEBUG("smart config ����״̬ LED���� ��led��˸���û����\r\n");
		break;
		
		case AP_STATE:
		//AP����״̬ LED����
		DEBUG("AP����״̬ LED����\r\n");
		break;
		
		case WIFI_NOT_CONNECTED:
		//WIFI������ɣ���������·������LED����
		DEBUG("WIFI������ɣ���������·������LED����\r\n");
		if(isStartConnectWIFI)
		{
			isStartConnectWIFI = false;
			bsp_SperkerPlay(Song29);
		}
		
		break;
		
		case WIFI_CONNECTED:
		//·�������ӳɹ� LED����
		DEBUG("·�������ӳɹ� LED����\r\n");
		break;
		
		case WIFI_CONN_CLOUD:
		//·�������ӳɹ� LED����
		DEBUG("�Ѿ��������Ʒ�����\r\n");
		if(isStartConnectClound)
		{
			isStartConnectClound = false;
			bsp_SperkerPlay(Song27);
			
			if(isOpenUploadMap == false)
			{
				isOpenUploadMap = true;
				bsp_StartUploadMap();
			    bsp_PutKey(KEY_3_LONG);
			}
		}
		
		break;
		
		case WIFI_LOW_POWER:
		//·�������ӳɹ� LED����
		DEBUG("���ڵ͹���ģʽ\r\n");
		break;
		
		case WIFI_SATE_UNKNOW:
		//·�������ӳɹ� LED����
		DEBUG("δ֪WIFI״̬\r\n");
	    isStartConnectWIFI = true;
	    isStartConnectClound = true;
		break;
		
		default:break;
	}
}

