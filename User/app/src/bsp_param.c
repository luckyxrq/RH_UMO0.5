#include "bsp.h"

#define PARAM_VER			 0x00000102      /* 参数版本 */
#define PARAM_SAVE_PAGE      255             /* 保存参数的页序号 */


/*
 按照2字节对齐，便于存储到内部FLASH，内部FLASH每次必须写2字节
 切记！！虽然可以在结构体中使用float和double，但是存储的时候后面的小数不准，所以建议浮点数全部扩大倍数后再存储
*/
#pragma pack(2)
typedef struct
{
	uint32_t ParamVer;			/* 参数区版本控制（可用于程序升级时，决定是否对参数区进行升级） */

	uint8_t data1;
	uint32_t data2;
	uint16_t data3;
	uint8_t data4;
}
PARAM_T;
#pragma pack()


/* 全局参数 */
static PARAM_T param;



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

		param.data1 = 101;
		param.data2 = 305;
		param.data3 = 409;
		param.data4 = 200;

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
	
	
	DEBUG("sizeof(param):%d\r\n",sizeof(param));
	
	bsp_LoadParam();
	DEBUG("param.ParamVer:%03X\r\n",param.ParamVer);
	DEBUG("param.data1:%d\r\n",param.data1);
	DEBUG("param.data2:%d\r\n",param.data2);
	DEBUG("param.data3:%d\r\n",param.data3);
	DEBUG("param.data4:%d\r\n",param.data4);

	
}


