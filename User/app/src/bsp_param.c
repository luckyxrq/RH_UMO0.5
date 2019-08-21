#include "bsp.h"

#define PARAM_VER			 0x00000104      /* �����汾 */
#define PARAM_SAVE_PAGE      255             /* ���������ҳ��� */


/*����2�ֽڶ��룬���ڴ洢���ڲ�FLASH���ڲ�FLASHÿ�α���д2�ֽ�*/
#pragma pack(2)
typedef struct
{
	uint32_t ParamVer;			/* �������汾���ƣ������ڳ�������ʱ�������Ƿ�Բ��������������� */

	
}
PARAM_T;
#pragma pack()


/* ȫ�ֲ��� */
static PARAM_T param;



/*
*********************************************************************************************************
*	�� �� ��: bsp_LoadParam
*	����˵��: ��Flash��������g_tParam
*	��    �Σ���
*	�� �� ֵ: ��
*********************************************************************************************************
*/
void bsp_LoadParam(void)
{
	bsp_FlashReadPage(PARAM_SAVE_PAGE,(uint16_t*)&param,sizeof(param));
	
	/* ���ȱʡ���� */
	if (param.ParamVer != PARAM_VER)
	{
		param.ParamVer = PARAM_VER;


		bsp_SaveParam();							/* ���²���д��Flash */
	}
}

/*
*********************************************************************************************************
*	�� �� ��: bsp_SaveParam
*	����˵��: ��ȫ�ֱ���g_tParam д�뵽CPU�ڲ�Flash
*	��    ��: ��
*	�� �� ֵ: ��
*********************************************************************************************************
*/
void bsp_SaveParam(void)
{
	bsp_FlashWritePage(PARAM_SAVE_PAGE,(uint16_t*)&param,sizeof(param));
}


