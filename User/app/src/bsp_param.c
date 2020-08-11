#include "bsp.h"



/*
 按照2字节对齐，便于存储到内部FLASH，内部FLASH每次必须写2字节
 切记！！虽然可以在结构体中使用float和double，但是存储的时候后面的小数不准，所以建议浮点数全部扩大倍数后再存储
*/
#pragma pack(2)
typedef struct
{
	uint32_t ParamVer;			/* 参数区版本控制（可用于程序升级时，决定是否对参数区进行升级） */

	uint8_t VacuumPowerGrade;   /* VACUUM_STRENGTH    VACUUM_NORMAL    VACUUM_QUIET*/
	
	uint32_t Cliff_L;       /*跳崖传感器阈值左*/ 
	uint32_t Cliff_M;       /*跳崖传感器阈值中*/ 
	uint32_t Cliff_R;       /*跳崖传感器阈值右*/ 
	                        
	uint32_t Edge_L;        /*沿边传感器阈值左*/
	uint32_t Edge_R;        /*沿边传感器阈值右*/
	
	uint32_t ErLangShen;    /*二郎神阈值*/
	
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
		
		param.Cliff_L = 30 ;
		param.Cliff_M = 30 ;
		param.Cliff_R = 30 ;
		
		param.Edge_L = 100 ;
		param.Edge_R = 100 ;
		
		param.ErLangShen = 100;

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



/************************************************SET接口********************************************************/

void bsp_SetVacuumPowerGrade(uint8_t grade)
{
	param.VacuumPowerGrade = grade;
	
	bsp_SaveParam();
}

void bsp_SetParaCliff_L(uint32_t val)
{
	param.Cliff_L = val;
	
	bsp_SaveParam();
}

void bsp_SetParaCliff_M(uint32_t val)
{
	param.Cliff_M = val;
	
	bsp_SaveParam();
}


void bsp_SetParaCliff_R(uint32_t val)
{
	param.Cliff_R = val;
	
	bsp_SaveParam();
}


void bsp_SetParaEdge_L(uint32_t val)
{
	param.Edge_L = val;
	
	bsp_SaveParam();
}


void bsp_SetParaEdge_R(uint32_t val)
{
	param.Edge_R = val;
	
	bsp_SaveParam();
}


void bsp_SetParaErLangShen(uint32_t val)
{
	param.ErLangShen = val;
	
	bsp_SaveParam();
}


/************************************************GET接口********************************************************/

uint8_t bsp_GetVacuumPowerGrade(void)
{
	return param.VacuumPowerGrade;
}

uint32_t bsp_GetParaCliff_L(void)
{
	return param.Cliff_L;
}

uint32_t bsp_GetParaCliff_M(void)
{
	return param.Cliff_M;
}


uint32_t bsp_GetParaCliff_R(void)
{
	return param.Cliff_R;
}


uint32_t bsp_GetParaEdge_L(void)
{
	return param.Edge_L;
}


uint32_t bsp_GetParaEdge_R(void)
{
	return param.Edge_R;
}


uint32_t bsp_GetParaErLangShen(void)
{
	return param.ErLangShen;
}



