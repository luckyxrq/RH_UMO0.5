#include "bsp.h"

/*
*********************************************************************************************************
*	函 数 名: bsp_WifiStateProc
*	功能说明: WIFI状态发生改变，对应的处理函数
*	形    参：无
*	返 回 值: 无
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
		//smart config 配置状态 LED快闪 ，led闪烁请用户完成
		DEBUG("smart config 配置状态 LED快闪 ，led闪烁请用户完成\r\n");
		break;
		
		case AP_STATE:
		//AP配置状态 LED慢闪
		DEBUG("AP配置状态 LED慢闪\r\n");
		break;
		
		case WIFI_NOT_CONNECTED:
		//WIFI配置完成，正在连接路由器，LED常暗
		DEBUG("WIFI配置完成，正在连接路由器，LED常暗\r\n");
		if(isStartConnectWIFI)
		{
			isStartConnectWIFI = false;
			bsp_SperkerPlay(Song29);
		}
		
		break;
		
		case WIFI_CONNECTED:
		//路由器连接成功 LED常亮
		DEBUG("路由器连接成功 LED常亮\r\n");
		break;
		
		case WIFI_CONN_CLOUD:
		//路由器连接成功 LED常亮
		DEBUG("已经连接上云服务器\r\n");
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
		//路由器连接成功 LED常亮
		DEBUG("处于低功耗模式\r\n");
		break;
		
		case WIFI_SATE_UNKNOW:
		//路由器连接成功 LED常亮
		DEBUG("未知WIFI状态\r\n");
	    isStartConnectWIFI = true;
	    isStartConnectClound = true;
		break;
		
		default:break;
	}
}

