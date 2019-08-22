#include "bsp.h"

#define PARAM_VER			 0x00000102      /* �����汾 */
#define PARAM_SAVE_PAGE      255             /* ���������ҳ��� */


/*
 ����2�ֽڶ��룬���ڴ洢���ڲ�FLASH���ڲ�FLASHÿ�α���д2�ֽ�
 �мǣ�����Ȼ�����ڽṹ����ʹ��float��double�����Ǵ洢��ʱ������С����׼�����Խ��鸡����ȫ�����������ٴ洢
*/
#pragma pack(2)
typedef struct
{
	uint32_t ParamVer;			/* �������汾���ƣ������ڳ�������ʱ�������Ƿ�Բ��������������� */

	uint8_t data1;
	uint32_t data2;
	uint16_t data3;
	uint8_t data4;
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

		param.data1 = 101;
		param.data2 = 305;
		param.data3 = 409;
		param.data4 = 200;

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
	
	
	DEBUG("sizeof(param):%d\r\n",sizeof(param));
	
	bsp_LoadParam();
	DEBUG("param.ParamVer:%03X\r\n",param.ParamVer);
	DEBUG("param.data1:%d\r\n",param.data1);
	DEBUG("param.data2:%d\r\n",param.data2);
	DEBUG("param.data3:%d\r\n",param.data3);
	DEBUG("param.data4:%d\r\n",param.data4);

	
}


