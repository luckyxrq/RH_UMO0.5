#include "bsp.h"

#define PARAM_VER			 0x00000103      /* 参数版本 */
#define PARAM_SAVE_PAGE      255             /* 保存参数的页序号 */







/*
 按照2字节对齐，便于存储到内部FLASH，内部FLASH每次必须写2字节
 切记！！虽然可以在结构体中使用float和double，但是存储的时候后面的小数不准，所以建议浮点数全部扩大倍数后再存储
*/
#pragma pack(2)
typedef struct
{
	uint32_t ParamVer;			/* 参数区版本控制（可用于程序升级时，决定是否对参数区进行升级） */

	uint8_t VacuumPowerGrade;   /* VACUUM_STRENGTH    VACUUM_NORMAL    VACUUM_QUIET*/
	
	uint32_t collsion_cnt[4];
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

		param.VacuumPowerGrade = VACUUM_NORMAL;

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
void bsp_ParamReadAtPowerOn(void)
{
	memset(&param,0,sizeof(param));
	
	bsp_LoadParam();
	RTT("param.ParamVer:%03X\r\n",param.ParamVer);

	RTT("param.VacuumPowerGrade:%d\r\n",param.VacuumPowerGrade);
}


/********************************************************************************************************


									      下面都是 SET GET


********************************************************************************************************/




/*
*********************************************************************************************************
*	函 数 名: bsp_ParamUpdateTest
*	功能说明: 风机吸力
*	形    参: VACUUM_STRENGTH    VACUUM_NORMAL    VACUUM_QUIET
*	返 回 值: 无
*********************************************************************************************************
*/
void bsp_SetVacuumPowerGrade(uint8_t grade)
{
	param.VacuumPowerGrade = grade;
	
	bsp_SaveParam();
}

void bsp_SetCollisonCnt(uint32_t* collison_buf)
{
	param.collsion_cnt[0] = collison_buf[0]; //left_cnt;
	param.collsion_cnt[1] = collison_buf[1];//right_cnt;
	param.collsion_cnt[2] = collison_buf[2];//all_cnt;
	param.collsion_cnt[3] = collison_buf[3];//none_cnt;
	
	bsp_SaveParam();
}

/*
*********************************************************************************************************
*	函 数 名: bsp_ParamUpdateTest
*	功能说明: 更新内部FLASH数据
*	形    参: 无
*	返 回 值: VACUUM_STRENGTH    VACUUM_NORMAL    VACUUM_QUIET
*********************************************************************************************************
*/
uint8_t bsp_GetVacuumPowerGrade(void)
{
	return param.VacuumPowerGrade;
}

const uint32_t* bsp_GetCollisonCnt(void)
{
	return param.collsion_cnt;
}
