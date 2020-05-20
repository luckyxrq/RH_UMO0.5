#ifndef __BSP_PARAM_H
#define __BSP_PARAM_H

#define PARAM_VER			 0x106      /* �����汾 */
#define PARAM_SAVE_PAGE      250             /* ���������ҳ��� */


/*
 ����2�ֽڶ��룬���ڴ洢���ڲ�FLASH���ڲ�FLASHÿ�α���д2�ֽ�
 �мǣ�����Ȼ�����ڽṹ����ʹ��float��double�����Ǵ洢��ʱ������С����׼�����Խ��鸡����ȫ�����������ٴ洢
*/
#pragma pack(2)
typedef struct
{
	uint32_t ParamVer;			/* �������汾���ƣ������ڳ�������ʱ�������Ƿ�Բ��������������� */

	uint32_t data1;
	uint32_t data2;
	uint32_t data3;
	uint32_t data4;
}
PARAM_T;
#pragma pack()


void bsp_LoadParam(void);
void bsp_SaveParam(void);
void bsp_ParamUpdateTest(void);

#endif

