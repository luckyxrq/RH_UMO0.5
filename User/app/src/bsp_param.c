#include "bsp.h"




/* ȫ�ֲ��� */
PARAM_T param;



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

		param.data1 = 1500;
		param.data2 = 1500;
		param.data3 = 1500;
		param.data4 = 1500;

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


