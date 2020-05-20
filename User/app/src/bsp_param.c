#include "bsp.h"




/* 全局参数 */
PARAM_T param;



/*
*********************************************************************************************************
*	函 数 名: bsp_LoadParam
*	功能说明: 从Flash读参数到g_tParam，注意长度的除2，因为每次都是读半字
*	形    参：无
*	返 回 值: 无
*********************************************************************************************************
*/
void bsp_LoadParam(void)
{
	bsp_FlashReadPage(PARAM_SAVE_PAGE,(uint16_t*)&param,sizeof(param)/2);
	
	/* 填充缺省参数 */
	if (param.ParamVer != PARAM_VER)
	{
		param.ParamVer = PARAM_VER;

		param.data1 = 1500;
		param.data2 = 1500;
		param.data3 = 1500;
		param.data4 = 1500;

		bsp_SaveParam();							/* 将新参数写入Flash */
	}
}

/*
*********************************************************************************************************
*	函 数 名: bsp_SaveParam
*	功能说明: 将全局变量g_tParam 写入到CPU内部Flash，注意长度的除2，因为每次都是写半字
*	形    参: 无
*	返 回 值: 无
*********************************************************************************************************
*/
void bsp_SaveParam(void)
{
	bsp_FlashWritePage(PARAM_SAVE_PAGE,(uint16_t*)&param,sizeof(param)/2);
}


/*
*********************************************************************************************************
*	函 数 名: bsp_ParamUpdateTest
*	功能说明: 更新内部FLASH数据
*	形    参: 无
*	返 回 值: 无
*********************************************************************************************************
*/
void bsp_ParamUpdateTest(void)
{
	memset(&param,0,sizeof(param));
	
	bsp_LoadParam();
	DEBUG("param.ParamVer:%03X\r\n",param.ParamVer);
	DEBUG("param.data1:%d\r\n",param.data1);
	DEBUG("param.data2:%d\r\n",param.data2);
	DEBUG("param.data3:%d\r\n",param.data3);
	DEBUG("param.data4:%d\r\n",param.data4);

	
}


