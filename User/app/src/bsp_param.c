#include "bsp.h"

#define PARAM_VER			 0x00000105      /* �����汾 */
#define PARAM_SAVE_PAGE      255             /* ���������ҳ��� */


/*����2�ֽڶ��룬���ڴ洢���ڲ�FLASH���ڲ�FLASHÿ�α���д2�ֽ�*/
#pragma pack(1)
typedef struct
{
	uint32_t ParamVer;			/* �������汾���ƣ������ڳ�������ʱ�������Ƿ�Բ��������������� */

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

//		param.data1 = 3.141F;
//		param.data2 = 66;
//		param.data3 = 666.123F;
//		param.data4 = 2345;
//		param.data5 = 888.632F;
//		param.data6 = 99;
//		param.data7 = 99.8F;
//		param.data8 = 3.141592654;

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


/*
*********************************************************************************************************
*	�� �� ��: bsp_ParamUpdateTest
*	����˵��: �����ڲ�FLASH����
*	��    ��: ��
*	�� �� ֵ: ��
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


