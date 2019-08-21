#include "bsp.h"

#define PARAM_VER			 0x00000104      /* 参数版本 */
#define PARAM_SAVE_PAGE      255             /* 保存参数的页序号 */


/*按照2字节对齐，便于存储到内部FLASH，内部FLASH每次必须写2字节*/
#pragma pack(2)
typedef struct
{
	uint32_t ParamVer;			/* 参数区版本控制（可用于程序升级时，决定是否对参数区进行升级） */

	
}
PARAM_T;
#pragma pack()


/* 全局参数 */
static PARAM_T param;



/*
*********************************************************************************************************
*	函 数 名: bsp_LoadParam
*	功能说明: 从Flash读参数到g_tParam
*	形    参：无
*	返 回 值: 无
*********************************************************************************************************
*/
void bsp_LoadParam(void)
{
	bsp_FlashReadPage(PARAM_SAVE_PAGE,(uint16_t*)&param,sizeof(param));
	
	/* 填充缺省参数 */
	if (param.ParamVer != PARAM_VER)
	{
		param.ParamVer = PARAM_VER;


		bsp_SaveParam();							/* 将新参数写入Flash */
	}
}

/*
*********************************************************************************************************
*	函 数 名: bsp_SaveParam
*	功能说明: 将全局变量g_tParam 写入到CPU内部Flash
*	形    参: 无
*	返 回 值: 无
*********************************************************************************************************
*/
void bsp_SaveParam(void)
{
	bsp_FlashWritePage(PARAM_SAVE_PAGE,(uint16_t*)&param,sizeof(param));
}


