#include "bsp.h"



/*
 ����2�ֽڶ��룬���ڴ洢���ڲ�FLASH���ڲ�FLASHÿ�α���д2�ֽ�
 �мǣ�����Ȼ�����ڽṹ����ʹ��float��double�����Ǵ洢��ʱ������С����׼�����Խ��鸡����ȫ�����������ٴ洢
*/
#pragma pack(2)
typedef struct
{
	uint32_t ParamVer;			/* �������汾���ƣ������ڳ�������ʱ�������Ƿ�Բ��������������� */

	uint8_t VacuumPowerGrade;   /* VACUUM_STRENGTH    VACUUM_NORMAL    VACUUM_QUIET*/
	
	uint32_t Cliff_L;       /*���´�������ֵ��*/ 
	uint32_t Cliff_M;       /*���´�������ֵ��*/ 
	uint32_t Cliff_R;       /*���´�������ֵ��*/ 
	                        
	uint32_t Edge_L;        /*�رߴ�������ֵ��*/
	uint32_t Edge_R;        /*�رߴ�������ֵ��*/
	
	uint32_t ErLangShen;    /*��������ֵ*/
	
}
PARAM_T;
#pragma pack()


/* ȫ�ֲ��� */
static PARAM_T param;



/*
*********************************************************************************************************
*	�� �� ��: bsp_LoadParam
*	����˵��: ��Flash��������g_tParam��ע�ⳤ�ȵĳ�2����Ϊÿ�ζ��Ƕ�����
*	��    �Σ���
*	�� �� ֵ: ��
*********************************************************************************************************
*/
void bsp_LoadParam(void)
{
	bsp_FlashReadPage(PARAM_SAVE_PAGE,(uint16_t*)&param,sizeof(param)/2);
	
	/* ���ȱʡ���� */
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

		bsp_SaveParam();							/* ���²���д��Flash */
	}
}

/*
*********************************************************************************************************
*	�� �� ��: bsp_SaveParam
*	����˵��: ��ȫ�ֱ���g_tParam д�뵽CPU�ڲ�Flash��ע�ⳤ�ȵĳ�2����Ϊÿ�ζ���д����
*	��    ��: ��
*	�� �� ֵ: ��
*********************************************************************************************************
*/
void bsp_SaveParam(void)
{
	bsp_FlashWritePage(PARAM_SAVE_PAGE,(uint16_t*)&param,sizeof(param)/2);
}


/*
*********************************************************************************************************
*	�� �� ��: bsp_ParamUpdateTest
*	����˵��: �����ڲ�FLASH����
*	��    ��: ��
*	�� �� ֵ: ��
*********************************************************************************************************
*/
void bsp_ParamReadAtPowerOn(void)
{
	memset(&param,0,sizeof(param));
	
	bsp_LoadParam();
	RTT("param.ParamVer:%03X\r\n",param.ParamVer);

	RTT("param.VacuumPowerGrade:%d\r\n",param.VacuumPowerGrade);
}



/************************************************SET�ӿ�********************************************************/

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


/************************************************GET�ӿ�********************************************************/

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



