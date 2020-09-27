#include "bsp.h"

#define RTT_BED_EN  0

#if RTT_BED_EN
#define RTT_BED(format, ...) SEGGER_RTT_printf (0,format, ##__VA_ARGS__)
#else
#define RTT_BED(format, ...) {}
#endif	


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

bool bsp_IsBedProcRunning(void)
{
	return bed.isRunning;
}



void bsp_BedProc(void)
{
	if(!bed.isRunning)
		return;
	
	switch(bed.action)
	{
		case 0: /* 原地左转*/
		{
			RTT_BED("bed %d\r\n",0);
			
			bed.pulse = bsp_GetCurrentBothPulse();
			
			bsp_SetMotorSpeed(MotorLeft, -6);
			bsp_SetMotorSpeed(MotorRight,6);
			++bed.action;
		}break;
		
		case 1: /* 转90度*/
		{
			RTT_BED("bed %d\r\n",1);
			
			if(bsp_GetCurrentBothPulse() - bed.pulse >= CCW_90_PULSE)
			{
				bsp_SetMotorSpeed(MotorLeft, 0);
				bsp_SetMotorSpeed(MotorRight,0);
				++bed.action;
			}
		}break;
		
		case 2: /*开启左沿边*/
		{
			RTT_BED("bed %d\r\n",2);
			
			bed.pulse = bsp_GetCurrentBothPulse();
		
			bsp_SetEdgeLeftRight(Dir_left);
			bsp_StartEdgewiseRun();
		
			++bed.action;
		}break;
		
		case 3: /* 记脉冲数 用于停止左沿边*/
		{
			RTT_BED("bed %d\r\n",3);
			
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
			RTT_BED("bed %d\r\n",4);
			
			bed.pulse = bsp_GetCurrentBothPulse();
			
			bsp_SetMotorSpeed(MotorLeft, 6);
			bsp_SetMotorSpeed(MotorRight,-6);
			++bed.action;
		}break;
		
		case 5: /* 转90度 */
		{
			RTT_BED("bed %d\r\n",5);
			
			if(bsp_GetCurrentBothPulse() - bed.pulse >= CCW_90_PULSE)
			{
				bsp_SetMotorSpeed(MotorLeft, 0);
				bsp_SetMotorSpeed(MotorRight,0);
				++bed.action;
			}
		}break;
		
		
		case 6: /* 向前走几步 */
		{
			RTT_BED("bed %d\r\n",6);
			
			bed.pulse = bsp_GetCurrentBothPulse();
			
			bsp_SetMotorSpeed(MotorLeft, 6);
			bsp_SetMotorSpeed(MotorRight,6);
			
			++bed.action;

		}break;
		
		case 7: /* 停下 此时平行于  二郎板 */
		{
			RTT_BED("bed %d\r\n",7);
			
			if(bsp_GetCurrentBothPulse() - bed.pulse >= 1700*0.7)
			{
				bsp_SetMotorSpeed(MotorLeft, 0);
				bsp_SetMotorSpeed(MotorRight,0);
				++bed.action;
			}

		}break;

		case 8: /* 原地左转*/
		{
			RTT_BED("bed %d\r\n",8);
			
			bed.pulse = bsp_GetCurrentBothPulse();
			
			bsp_SetMotorSpeed(MotorLeft, -6);
			bsp_SetMotorSpeed(MotorRight,6);
			++bed.action;
		}break;
		
		case 9: /* 转90度*/
		{
			RTT_BED("bed %d\r\n",9);
			
			if(bsp_GetCurrentBothPulse() - bed.pulse >= CCW_90_PULSE)
			{
				bsp_SetMotorSpeed(MotorLeft, 0);
				bsp_SetMotorSpeed(MotorRight,0);
				++bed.action;
			}
		}break;
		
		case 10: /* 直走  等到遇到 二郎板 */
		{
			RTT_BED("bed %d\r\n",10);
			
			bsp_SetMotorSpeed(MotorLeft, 8);
			bsp_SetMotorSpeed(MotorRight,8);
			
			bed.erLangGodCnt = 0 ;
			
			++bed.action;
		}break;
		
		case 11:
		{
			RTT_BED("bed %d\r\n",11);
			
			/* 如果二郎神被遮挡*/
			if(bsp_GetInfraRedAdcVoltage(IR7) >= bsp_GetParaErLangShen())
			{
				RTT("IR7:%d\r\n",(int)bsp_GetInfraRedAdcVoltage(IR7));
				
				
				++bed.erLangGodCnt;
				
				/* 判断二郎神被遮挡次数  减速缓行*/
				if(bed.erLangGodCnt >= 10)
				{
					bsp_SetMotorSpeed(MotorLeft, 0);
					bsp_SetMotorSpeed(MotorRight,0);
					
					bed.delay = xTaskGetTickCount();
					
					++bed.action;
				}
			}
		}break;
		
		case 12:
		{
			RTT_BED("bed %d\r\n",12);
			
			/* 在二郎板子底下减速运行*/
			if(xTaskGetTickCount() - bed.delay >= 200)
			{
				bed.pulse = bsp_GetCurrentBothPulse();
				bsp_SetMotorSpeed(MotorLeft, 3);
				bsp_SetMotorSpeed(MotorRight,3);

				++bed.action;
			}
		}break;

		case 13: /* 缓行一定距离 ，开启右沿边*/
		{
			RTT_BED("bed %d\r\n",13);
			
			if(bsp_GetCurrentBothPulse() - bed.pulse >= 1700 * 0.6)
			{
				bed.pulse = bsp_GetCurrentBothPulse();
				
				bsp_SetEdgeLeftRight(Dir_right);
				bsp_StartEdgewiseRun();
				
				++bed.action;
			}
		}break;
		
		case 14: /* 记脉冲数 用于停止右沿边*/
		{
			RTT_BED("bed %d\r\n",14);
			
			if(bsp_GetCurrentBothPulse() - bed.pulse >= 1700*4.2)
			{
				
				bsp_StopEdgewiseRun();
				
				bsp_SetMotorSpeed(MotorLeft, 0);
				bsp_SetMotorSpeed(MotorRight,0);
				++bed.action;
			}
		}break;
		
		
		case 15: /* 直走*/
		{
			RTT_BED("bed %d\r\n",15);
			
			bsp_SetMotorSpeed(MotorLeft, 8);
			bsp_SetMotorSpeed(MotorRight,8);
			
			++bed.action;
		}break;
		
		case 16: /* 等待遇到悬崖 停止  理论此次遇到 前面悬崖  后退*/
		{
			RTT_BED("bed %d\r\n",16);
			
			if(bsp_CliffIsDangerous(CliffMiddle))
			{
				bed.pulse = bsp_GetCurrentBothPulse();
				
				bsp_SetMotorSpeed(MotorLeft, -6);
				bsp_SetMotorSpeed(MotorRight,-6);
				
				++bed.action;
			}
		}break;
		
		case 17: /* 后退固定脉冲数 再原地左转*/
		{
			RTT_BED("bed %d\r\n",17);
			
			if(bsp_GetCurrentBothPulse() - bed.pulse >= 800)
			{
				bed.pulse = bsp_GetCurrentBothPulse();
				
				bsp_SetMotorSpeed(MotorLeft, -6);
				bsp_SetMotorSpeed(MotorRight,6);
				++bed.action;
			}
		}break;
		
		case 18: /* 转完直行  准备测试右边跳崖*/
		{
			RTT_BED("bed %d\r\n",18);
			
			if(bsp_GetCurrentBothPulse() - bed.pulse >= CCW_45_PULSE)
			{
				bed.pulse = bsp_GetCurrentBothPulse();
				
				bsp_SetMotorSpeed(MotorLeft, 6);
				bsp_SetMotorSpeed(MotorRight,6);
				++bed.action;
			}
		}break;
		
		case 19: /* 等待遇到悬崖 停止  理论此次遇到 右面悬崖  后退*/
		{
			RTT_BED("bed %d\r\n",19);
			
			if(bsp_CliffIsDangerous(CliffRight))
			{
				bed.pulse = bsp_GetCurrentBothPulse();
				
				bsp_SetMotorSpeed(MotorLeft, -6);
				bsp_SetMotorSpeed(MotorRight,-6);
				
				++bed.action;
			}
		}break;
		
		case 20: /* 后退固定脉冲数 再原地右转*/
		{
			RTT_BED("bed %d\r\n",20);
			
			if(bsp_GetCurrentBothPulse() - bed.pulse >= 400)
			{
				bed.pulse = bsp_GetCurrentBothPulse();
				
				bsp_SetMotorSpeed(MotorLeft, 6);
				bsp_SetMotorSpeed(MotorRight,-6);
				++bed.action;
			}
		}break;
		
		case 21: /* 转完直行  准备测试左边跳崖*/
		{
			RTT_BED("bed %d\r\n",21);
			
			if(bsp_GetCurrentBothPulse() - bed.pulse >= CCW_90_PULSE)
			{
				bsp_SetMotorSpeed(MotorLeft, 6);
				bsp_SetMotorSpeed(MotorRight,6);
				++bed.action;
			}
		}break;
		
		case 22: /* 等待遇到悬崖 停止  理论此次遇到 左面悬崖  后退*/
		{
			RTT_BED("bed %d\r\n",22);
			
			if(bsp_CliffIsDangerous(CliffLeft))
			{
				bed.pulse = bsp_GetCurrentBothPulse();
				
				bsp_SetMotorSpeed(MotorLeft, -6);
				bsp_SetMotorSpeed(MotorRight,-6);
				
				++bed.action;
			}
		}break;
		
		case 23: /* 后退固定脉冲数*/
		{
			RTT_BED("bed %d\r\n",23);
			
			if(bsp_GetCurrentBothPulse() - bed.pulse >= 1200)
			{
				bed.pulse = bsp_GetCurrentBothPulse();
				
				bsp_SetMotorSpeed(MotorLeft, 0);
				bsp_SetMotorSpeed(MotorRight,0);
				++bed.action;
			}
		}break;
		
		
		case 24: /* 原地旋转  起始角度是 0   旋转到180度  验证陀螺仪*/
		{
			RTT_BED("bed %d\r\n",24);
			
			bsp_SetMotorSpeed(MotorLeft, -3);
			bsp_SetMotorSpeed(MotorRight, 3);
			++bed.action;	
			
		}break;
		
		case 25: /* 停在180 度*/
		{
			RTT_BED("bed %d\r\n",25);
			
			float yaw = ABS(bsp_AngleRead());	
			
			if( yaw >= 170 && yaw <= 180 )
			{
				bsp_SetMotorSpeed(MotorLeft, 0);
				bsp_SetMotorSpeed(MotorRight,0);
				
				bed.delay = xTaskGetTickCount();
				
				++bed.action;
			}
		}break;
		
		
		case 26: /*停几百MS*/
		{
			RTT_BED("bed %d\r\n",26);
			
			if(xTaskGetTickCount() - bed.delay >= 800)
			{
				bed.pulse = bsp_GetCurrentBothPulse();
				
				bsp_SetMotorSpeed(MotorLeft, 8);
				bsp_SetMotorSpeed(MotorRight,8);
				
				++bed.action;
			}
		}break;
		
		case 27:
		{

			if(bsp_GetCurrentBothPulse() - bed.pulse >= 1700)
			{

				bsp_SetMotorSpeed(MotorLeft, 0);
				bsp_SetMotorSpeed(MotorRight,0);
				
				++bed.action;
			}
		}break;

		case 28: /*停几百MS*/
		{
			/*关闭电机*/
			bsp_MotorCleanSetPWM(MotorRollingBrush, CW , 0);
			bsp_StopVacuum();
			
			/*LED*/
			bsp_OpenThreeWhileLed();
			
			bsp_BedStop();
			
			/*返回充电*/
			bsp_PutKey(KEY_LONG_CHARGE);
		}break;
		
		

		default: break;
	}
}



// LED_LOGO_CLEAN,LED_LOGO_POWER,LED_LOGO_CHARGE,


#define LED_ARR_SZIE  3

static LED_SN ledArr[LED_ARR_SZIE] = {LED_COLOR_YELLOW,LED_COLOR_GREEN,LED_COLOR_RED};


void bsp_LedBedTurns(void)
{
	static uint8_t toggleIndex = 0 ;
	
	bsp_LedOff(LED_LOGO_CLEAN);
	bsp_LedOff(LED_LOGO_POWER);
	bsp_LedOff(LED_LOGO_CHARGE);
	
	bsp_LedToggle(ledArr[toggleIndex % LED_ARR_SZIE]);
	
	if(++toggleIndex >= LED_ARR_SZIE)
	{
		toggleIndex = 0;
	}
}
