#include "bsp.h"

#define PARAM_VER			 0x00000105      /* 参数版本 */
#define PARAM_SAVE_PAGE      255             /* 保存参数的页序号 */


/*按照2字节对齐，便于存储到内部FLASH，内部FLASH每次必须写2字节*/
#pragma pack(1)
typedef struct
{
	uint32_t ParamVer;			/* 参数区版本控制（可用于程序升级时，决定是否对参数区进行升级） */

	float data1;
	uint8_t data2;
	float data3;
	uint16_t data4;
	float data5;
	uint32_t data6;
	float data7;
	double data8;
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

//		param.data1 = 3.141F;
//		param.data2 = 66;
//		param.data3 = 666.123F;
//		param.data4 = 2345;
//		param.data5 = 888.632F;
//		param.data6 = 99;
//		param.data7 = 99.8F;
//		param.data8 = 3.141592654;

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
//	bsp_LoadParam();
//	DEBUG("param.ParamVer:%03X\r\n",param.ParamVer);
//	DEBUG("param.data1:%f\r\n",param.data1);
//	DEBUG("param.data2:%d\r\n",param.data2);
//	DEBUG("param.data3:%f\r\n",param.data3);
//	DEBUG("param.data4:%d\r\n",param.data4);
//	DEBUG("param.data5:%f\r\n",param.data5);
//	DEBUG("param.data6:%d\r\n",param.data6);
//	DEBUG("param.data7:%f\r\n",param.data7);
//	DEBUG("param.data8:%f\r\n",param.data8);
	
}


