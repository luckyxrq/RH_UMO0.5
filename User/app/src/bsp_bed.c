#include "bsp.h"

typedef struct
{
	bool isRunning;
	uint32_t action;
	uint32_t delay;
	
	
	uint32_t pulse;
	uint32_t erLangGodCnt;
}Bed;

static Bed bed;




void bsp_BedStart(void)
{
	bed.action = 0 ;
	bed.delay = 0 ;
	bed.pulse = 0 ;
	bed.erLangGodCnt = 0 ;
	
	bed.isRunning = true;
}

void bsp_BedStop(void)
{	
	bed.isRunning = false;
	
	bed.action = 0 ;
	bed.delay = 0 ;
	bed.pulse = 0 ;
	bed.erLangGodCnt = 0 ;
}

void bsp_BedProc(void)
{
	if(!bed.isRunning)
		return;
	
	switch(bed.action)
	{
		case 0: /* 原地左转*/
		{
			bed.pulse = bsp_GetCurrentBothPulse();
			
			bsp_SetMotorSpeed(MotorLeft, -6);
			bsp_SetMotorSpeed(MotorRight,6);
			++bed.action;
		}break;
		
		case 1: /* 转90度*/
		{
			if(bsp_GetCurrentBothPulse() - bed.pulse >= CCW_90_PULSE)
			{
				bsp_SetMotorSpeed(MotorLeft, 0);
				bsp_SetMotorSpeed(MotorRight,0);
				++bed.action;
			}
		}break;
		
		case 2: /*开启左沿边*/
		{
				bed.pulse = bsp_GetCurrentBothPulse();
			
				bsp_SetEdgeLeftRight(Dir_left);
				bsp_StartEdgewiseRun();
			
				++bed.action;
		}break;
		
		case 3: /* 记脉冲数 用于停止左沿边*/
		{
			if(bsp_GetCurrentBothPulse() - bed.pulse >= 1700*3)
			{
				
				bsp_StopEdgewiseRun();
				
				bsp_SetMotorSpeed(MotorLeft, 0);
				bsp_SetMotorSpeed(MotorRight,0);
				++bed.action;
			}
		}break;
		
		case 4: /* 原地右转 */
		{
			bed.pulse = bsp_GetCurrentBothPulse();
			
			bsp_SetMotorSpeed(MotorLeft, 6);
			bsp_SetMotorSpeed(MotorRight,-6);
			++bed.action;
		}break;
		
		case 5: /* 转90度 */
		{
			if(bsp_GetCurrentBothPulse() - bed.pulse >= CCW_90_PULSE)
			{
				bsp_SetMotorSpeed(MotorLeft, 0);
				bsp_SetMotorSpeed(MotorRight,0);
				++bed.action;
			}
		}break;
		
		
		case 6: /* 向前走几步 */
		{
			bed.pulse = bsp_GetCurrentBothPulse();
			
			bsp_SetMotorSpeed(MotorLeft, 6);
			bsp_SetMotorSpeed(MotorRight,6);
			
			++bed.action;

		}break;
		
		case 7: /* 停下 此时平行于  二郎板 */
		{
			
			if(bsp_GetCurrentBothPulse() - bed.pulse >= 1700*0.7)
			{
				bsp_SetMotorSpeed(MotorLeft, 0);
				bsp_SetMotorSpeed(MotorRight,0);
				++bed.action;
			}

		}break;

		case 8: /* 原地左转*/
		{
			bed.pulse = bsp_GetCurrentBothPulse();
			
			bsp_SetMotorSpeed(MotorLeft, -6);
			bsp_SetMotorSpeed(MotorRight,6);
			++bed.action;
		}break;
		
		case 9: /* 转90度*/
		{
			if(bsp_GetCurrentBothPulse() - bed.pulse >= CCW_90_PULSE)
			{
				bsp_SetMotorSpeed(MotorLeft, 0);
				bsp_SetMotorSpeed(MotorRight,0);
				++bed.action;
			}
		}break;
		
		case 10: /* 直走  等到遇到 二郎板 */
		{
			if(bsp_GetCurrentBothPulse() - bed.pulse >= CCW_90_PULSE)
			{
				bsp_SetMotorSpeed(MotorLeft, 10);
				bsp_SetMotorSpeed(MotorRight,10);
				++bed.action;
			}
		}break;
		
		case 11:
		{
			/* 如果二郎神被遮挡*/
			if(bsp_GetInfraRedAdcVoltage(IR7) >= 250)
			{
				++bed.erLangGodCnt;
			}
			
			/* 判断二郎神被遮挡次数  减速缓行*/
			if(bed.erLangGodCnt >= 16)
			{
				bed.pulse = bsp_GetCurrentBothPulse();
				
				bsp_SetMotorSpeed(MotorLeft, 2);
				bsp_SetMotorSpeed(MotorRight,2);
				
				++bed.action;
			}
		}
		
		case 12: /* 缓行一定距离 ，开启右沿边*/
		{
			if(bsp_GetCurrentBothPulse() - bed.pulse >= 1700)
			{
				bed.pulse = bsp_GetCurrentBothPulse();
				
				bsp_SetEdgeLeftRight(Dir_right);
				bsp_StartEdgewiseRun();
				
				++bed.action;
			}
		}break;
		
		
		case 13: /* 记脉冲数 用于停止右沿边*/
		{
			if(bsp_GetCurrentBothPulse() - bed.pulse >= 1700*4.2)
			{
				
				bsp_StopEdgewiseRun();
				
				bsp_SetMotorSpeed(MotorLeft, 0);
				bsp_SetMotorSpeed(MotorRight,0);
				++bed.action;
			}
		}break;
		
		
		case 14: /* 直走*/
		{
			bsp_SetMotorSpeed(MotorLeft, 8);
			bsp_SetMotorSpeed(MotorRight,8);
			
			++bed.action;
		}break;
		
		case 15: /* 等待遇到悬崖 停止  理论此次遇到 前面悬崖  后退*/
		{
			if(bsp_CliffIsDangerous(CliffMiddle))
			{
				bed.pulse = bsp_GetCurrentBothPulse();
				
				bsp_SetMotorSpeed(MotorLeft, -6);
				bsp_SetMotorSpeed(MotorRight,-6);
				
				++bed.action;
			}
		}break;
		
		case 16: /* 后退固定脉冲数 再原地左转*/
		{
			if(bsp_GetCurrentBothPulse() - bed.pulse >= 800)
			{
				bed.pulse = bsp_GetCurrentBothPulse();
				
				bsp_SetMotorSpeed(MotorLeft, -6);
				bsp_SetMotorSpeed(MotorRight,6);
				++bed.action;
			}
		}break;
		
		case 17: /* 转完直行  准备测试右边跳崖*/
		{
			if(bsp_GetCurrentBothPulse() - bed.pulse >= CCW_45_PULSE)
			{
				bed.pulse = bsp_GetCurrentBothPulse();
				
				bsp_SetMotorSpeed(MotorLeft, 6);
				bsp_SetMotorSpeed(MotorRight,6);
				++bed.action;
			}
		}break;
		
		case 18: /* 等待遇到悬崖 停止  理论此次遇到 右面悬崖  后退*/
		{
			if(bsp_CliffIsDangerous(CliffRight))
			{
				bed.pulse = bsp_GetCurrentBothPulse();
				
				bsp_SetMotorSpeed(MotorLeft, -6);
				bsp_SetMotorSpeed(MotorRight,-6);
				
				++bed.action;
			}
		}break;
		
		case 19: /* 后退固定脉冲数 再原地右转*/
		{
			if(bsp_GetCurrentBothPulse() - bed.pulse >= 400)
			{
				bed.pulse = bsp_GetCurrentBothPulse();
				
				bsp_SetMotorSpeed(MotorLeft, 6);
				bsp_SetMotorSpeed(MotorRight,-6);
				++bed.action;
			}
		}break;
		
		case 20: /* 转完直行  准备测试左边跳崖*/
		{
			if(bsp_GetCurrentBothPulse() - bed.pulse >= CCW_90_PULSE)
			{
				bsp_SetMotorSpeed(MotorLeft, 6);
				bsp_SetMotorSpeed(MotorRight,6);
				++bed.action;
			}
		}break;
		
		case 21: /* 等待遇到悬崖 停止  理论此次遇到 左面悬崖  后退*/
		{
			if(bsp_CliffIsDangerous(CliffLeft))
			{
				bed.pulse = bsp_GetCurrentBothPulse();
				
				bsp_SetMotorSpeed(MotorLeft, -6);
				bsp_SetMotorSpeed(MotorRight,-6);
				
				++bed.action;
			}
		}break;
		
		case 22: /* 后退固定脉冲数*/
		{
			if(bsp_GetCurrentBothPulse() - bed.pulse >= 800)
			{
				bed.pulse = bsp_GetCurrentBothPulse();
				
				bsp_SetMotorSpeed(MotorLeft, 0);
				bsp_SetMotorSpeed(MotorRight,0);
				++bed.action;
			}
		}break;

		default: break;
	}
}


