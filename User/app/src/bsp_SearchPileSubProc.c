#include "bsp.h"

typedef struct
{
	__IO bool isRunning;
	__IO uint32_t action;
	__IO uint32_t delay;
	
	float angle;
}CCWRotationDrawArc;

static CCWRotationDrawArc ccwRotationDrawArc;  /*先逆时针旋转 再 画弧线*/

/*
*********************************************************************************************************
*	函 数 名: bsp_IsCCWRotationDrawArc
*	功能说明: 是否正在执行（先逆时针旋转 再 画弧线）
*	形    参: 无
*	返 回 值: 无
*********************************************************************************************************
*/
bool bsp_IsCCWRotationDrawArc(void)
{
	return ccwRotationDrawArc.isRunning;
}


/*
*********************************************************************************************************
*	函 数 名: bsp_CCWRotationDrawArcProc
*	功能说明: 先逆时针旋转 再 画弧线
*	形    参: 无
*	返 回 值: 无
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



