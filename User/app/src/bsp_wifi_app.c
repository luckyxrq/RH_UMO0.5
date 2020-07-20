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
*	函 数 名: bsp_StartChangeWifi2SmartConfigState
*	功能说明: 将WIFI切换为SmartConfig状态
*	形    参：无
*	返 回 值: 无
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
*	函 数 名: bsp_StartChangeWifi2SmartConfigState
*	功能说明: 停止将WIFI切换为SmartConfig状态的状态机
*	形    参：无
*	返 回 值: 无
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
*	函 数 名: bsp_WifiStateProc
*	功能说明: 将WIFI切换为SmartConfig状态
*	形    参：无
*	返 回 值: 无
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
			else /*已经是SMART CONFIG模式*/
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
*	函 数 名: bsp_WifiStateProc
*	功能说明:此函数专用于获取WIFI状态，周期性的被调用，无需再里面添加内容了
*	形    参：无
*	返 回 值: 无
*********************************************************************************************************
*/
void bsp_WifiStateProc(void)
{
	switch(mcu_get_wifi_work_state())
	{
		case SMART_CONFIG_STATE:
		WIFI_DEBUG("smart config 配置状态 LED快闪 ，led闪烁请用户完成\r\n");
		wifi_ctr.state = eSMART_CONFIG_STATE;
		break;
		
		case AP_STATE:
		WIFI_DEBUG("AP配置状态 LED慢闪\r\n");
		wifi_ctr.state = eAP_STATE;
		break;
		
		case WIFI_NOT_CONNECTED:
		WIFI_DEBUG("WIFI配置完成，正在连接路由器，LED常暗\r\n");
		wifi_ctr.state = eWIFI_NOT_CONNECTED;
		break;
		
		case WIFI_CONNECTED:
		WIFI_DEBUG("路由器连接成功 LED常亮\r\n");
		wifi_ctr.state = eWIFI_CONNECTED;
		break;
		
		case WIFI_CONN_CLOUD:
		WIFI_DEBUG("已经连接上云服务器\r\n");
		wifi_ctr.state = eWIFI_CONN_CLOUD;
		break;
		
		case WIFI_LOW_POWER:
		WIFI_DEBUG("处于低功耗模式\r\n");
		wifi_ctr.state = eWIFI_LOW_POWER;
		break;
		
		case WIFI_SATE_UNKNOW:
		WIFI_DEBUG("未知WIFI状态\r\n");
		wifi_ctr.state = eWIFI_SATE_UNKNOW;
		break;
		
		default:break;
	}
}



