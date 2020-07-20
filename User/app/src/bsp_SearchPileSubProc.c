#include "bsp.h"

typedef struct
{
	__IO bool isRunning;
	__IO uint32_t action;
	__IO uint32_t delay;
	
	float angle;
}CCWRotationDrawArc;

static CCWRotationDrawArc ccwRotationDrawArc;  /*����ʱ����ת �� ������*/

/*
*********************************************************************************************************
*	�� �� ��: bsp_IsCCWRotationDrawArc
*	����˵��: �Ƿ�����ִ�У�����ʱ����ת �� �����ߣ�
*	��    ��: ��
*	�� �� ֵ: ��
*********************************************************************************************************
*/
bool bsp_IsCCWRotationDrawArc(void)
{
	return ccwRotationDrawArc.isRunning;
}


/*
*********************************************************************************************************
*	�� �� ��: bsp_CCWRotationDrawArcProc
*	����˵��: ����ʱ����ת �� ������
*	��    ��: ��
*	�� �� ֵ: ��
*********************************************************************************************************
*/
void bsp_CCWRotationDrawArcProc(void)
{
	if(!ccwRotationDrawArc.isRunning)
		return;
	
	switch(ccwRotationDrawArc.action)
	{
		case 0:
		{
			
		}break;
	}
}



