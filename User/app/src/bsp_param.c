#include "bsp.h"

#define PARAM_VER			 0x00000103      /* �����汾 */
#define PARAM_SAVE_PAGE      255             /* ���������ҳ��� */







/*
 ����2�ֽڶ��룬���ڴ洢���ڲ�FLASH���ڲ�FLASHÿ�α���д2�ֽ�
 �мǣ�����Ȼ�����ڽṹ����ʹ��float��double�����Ǵ洢��ʱ������С����׼�����Խ��鸡����ȫ�����������ٴ洢
*/
#pragma pack(2)
typedef struct
{
	uint32_t ParamVer;			/* �������汾���ƣ������ڳ�������ʱ�������Ƿ�Բ��������������� */

	uint8_t VacuumPowerGrade;   /* VACUUM_STRENGTH    VACUUM_NORMAL    VACUUM_QUIET*/
	
	uint32_t collsion_cnt[4];
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


/********************************************************************************************************


									      ���涼�� SET GET


********************************************************************************************************/




/*
*********************************************************************************************************
*	�� �� ��: bsp_ParamUpdateTest
*	����˵��: �������
*	��    ��: VACUUM_STRENGTH    VACUUM_NORMAL    VACUUM_QUIET
*	�� �� ֵ: ��
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
*	�� �� ��: bsp_ParamUpdateTest
*	����˵��: �����ڲ�FLASH����
*	��    ��: ��
*	�� �� ֵ: VACUUM_STRENGTH    VACUUM_NORMAL    VACUUM_QUIET
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
