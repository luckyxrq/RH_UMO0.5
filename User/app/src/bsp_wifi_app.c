#include "bsp.h"




#define DETECT_SMART_CONFIG_PULSE_MS      1000

typedef enum
{
	eSMART_CONFIG_STATE    =      SMART_CONFIG_STATE ,
	eAP_STATE              =      AP_STATE ,
	eWIFI_NOT_CONNECTED    =      WIFI_NOT_CONNECTED ,
	eWIFI_CONNECTED        =      WIFI_CONNECTED ,
	eWIFI_CONN_CLOUD	   =	  WIFI_CONN_CLOUD ,
	eWIFI_LOW_POWER		   =	  WIFI_LOW_POWER ,
	eWIFI_SATE_UNKNOW      =      WIFI_SATE_UNKNOW ,
	
}WIFI_STATE;

	
typedef struct
{
	WIFI_STATE state;
	
	bool isRunning;
	uint8_t action;
	uint32_t delay;
}WIFI_CTR;

typedef struct
{
	
	bool isRunning;
	uint8_t action;
	uint32_t delay;
}WIFI_2_SMART_CONFIG;


static WIFI_CTR wifi_ctr;
static WIFI_2_SMART_CONFIG wifi_2_smart_config;


/*
*********************************************************************************************************
*	�� �� ��: bsp_StartChangeWifi2SmartConfigState
*	����˵��: ��WIFI�л�ΪSmartConfig״̬
*	��    �Σ���
*	�� �� ֵ: ��
*********************************************************************************************************
*/
void bsp_StartChangeWifi2SmartConfigState(void)
{
	wifi_2_smart_config.action = 0 ;
	wifi_2_smart_config.delay = 0 ;
	wifi_2_smart_config.isRunning = true;
}


/*
*********************************************************************************************************
*	�� �� ��: bsp_StartChangeWifi2SmartConfigState
*	����˵��: ֹͣ��WIFI�л�ΪSmartConfig״̬��״̬��
*	��    �Σ���
*	�� �� ֵ: ��
*********************************************************************************************************
*/
void bsp_StopChangeWifi2SmartConfigState(void)
{
	wifi_2_smart_config.isRunning = false;
	wifi_2_smart_config.action = 0 ;
	wifi_2_smart_config.delay = 0 ;
	
}

/*
*********************************************************************************************************
*	�� �� ��: bsp_WifiStateProc
*	����˵��: ��WIFI�л�ΪSmartConfig״̬
*	��    �Σ���
*	�� �� ֵ: ��
*********************************************************************************************************
*/
void bsp_ChangeWifi2SmartConfigStateProc(void)
{
	if(!wifi_2_smart_config.isRunning)
		return;
	
	switch(wifi_2_smart_config.action)
	{
		case 0:
		{
			if(wifi_ctr.state != eSMART_CONFIG_STATE)
			{
				mcu_reset_wifi();
				wifi_2_smart_config.delay = xTaskGetTickCount();
				wifi_2_smart_config.action++;
			}
			else /*�Ѿ���SMART CONFIGģʽ*/
			{
				bsp_StopChangeWifi2SmartConfigState();
			}
		}break;
		
		case 1:
		{
			if(xTaskGetTickCount() - wifi_2_smart_config.delay >= DETECT_SMART_CONFIG_PULSE_MS)
			{
				wifi_ctr.action = 0 ;
			};
		}break;
	}
}




/*
*********************************************************************************************************
*	�� �� ��: bsp_WifiStateProc
*	����˵��:�˺���ר���ڻ�ȡWIFI״̬�������Եı����ã��������������������
*	��    �Σ���
*	�� �� ֵ: ��
*********************************************************************************************************
*/
void bsp_WifiStateProc(void)
{
	switch(mcu_get_wifi_work_state())
	{
		case SMART_CONFIG_STATE:
		WIFI_DEBUG("smart config ����״̬ LED���� ��led��˸���û����\r\n");
		wifi_ctr.state = eSMART_CONFIG_STATE;
		break;
		
		case AP_STATE:
		WIFI_DEBUG("AP����״̬ LED����\r\n");
		wifi_ctr.state = eAP_STATE;
		break;
		
		case WIFI_NOT_CONNECTED:
		WIFI_DEBUG("WIFI������ɣ���������·������LED����\r\n");
		wifi_ctr.state = eWIFI_NOT_CONNECTED;
		break;
		
		case WIFI_CONNECTED:
		WIFI_DEBUG("·�������ӳɹ� LED����\r\n");
		wifi_ctr.state = eWIFI_CONNECTED;
		break;
		
		case WIFI_CONN_CLOUD:
		WIFI_DEBUG("�Ѿ��������Ʒ�����\r\n");
		wifi_ctr.state = eWIFI_CONN_CLOUD;
		break;
		
		case WIFI_LOW_POWER:
		WIFI_DEBUG("���ڵ͹���ģʽ\r\n");
		wifi_ctr.state = eWIFI_LOW_POWER;
		break;
		
		case WIFI_SATE_UNKNOW:
		WIFI_DEBUG("δ֪WIFI״̬\r\n");
		wifi_ctr.state = eWIFI_SATE_UNKNOW;
		break;
		
		default:break;
	}
}



