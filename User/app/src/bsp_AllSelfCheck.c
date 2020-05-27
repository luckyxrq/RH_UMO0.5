#include "bsp.h"

#define IR_RX_CH1()      GPIO_ReadInputDataBit(GPIOC,GPIO_Pin_6)
#define IR_RX_CH2()      GPIO_ReadInputDataBit(GPIOC,GPIO_Pin_7)
#define IR_RX_CH3()      GPIO_ReadInputDataBit(GPIOC,GPIO_Pin_8)
#define IR_RX_CH4()      GPIO_ReadInputDataBit(GPIOC,GPIO_Pin_9)


#define IR_KEY_CLEAN()       GPIO_ReadInputDataBit(GPIO_PORT_K2,GPIO_PIN_K2)
#define IR_KEY_POWER()       GPIO_ReadInputDataBit(GPIO_PORT_K3,GPIO_PIN_K3)
#define IR_KEY_CHARGE()      GPIO_ReadInputDataBit(GPIO_PORT_K1,GPIO_PIN_K1)


typedef struct
{
	__IO bool isRunning ;
	__IO uint32_t action;
	__IO uint32_t delay;
	
	__IO bool isIMU_OK;
	__IO bool isWIFI_OK;
	
	
	__IO bool isIR_InitOK;
	__IO bool isIR_ADC_OK;
	
	
	__IO bool isIR_RX_CH1_OK;
	__IO bool isIR_RX_CH2_OK;
	__IO bool isIR_RX_CH3_OK;
	__IO bool isIR_RX_CH4_OK;
	
	__IO bool isOffsiteL_OK;
	__IO bool isOffsiteR_OK;
	
	__IO bool isCliffL_OK;
	__IO bool isCliffM_OK;
	__IO bool isCliffR_OK;
	
	__IO bool isDustBox_OK;
	
	
	__IO bool isKEY_CLEAN_OK;
	__IO bool isKEY_POWER_OK;
	__IO bool isKEY_CHARGE_OK;
	
	__IO bool isAllOK;
	
	
}AllSelfCheck;

static AllSelfCheck allSelfCheck;

void bsp_StartAllSelfCheck(void)
{
	allSelfCheck.action = 0 ;
	allSelfCheck.delay = 0 ;
	
	allSelfCheck.isIMU_OK = false;
	allSelfCheck.isWIFI_OK = false;
	allSelfCheck.isAllOK = true;
	
	allSelfCheck.isRunning = true;
	
}

void bsp_StopAllSelfCheck(void)
{
	allSelfCheck.isRunning = false;
	allSelfCheck.action = 0 ;
	allSelfCheck.delay = 0 ;
	
	allSelfCheck.isIMU_OK = false;
	allSelfCheck.isWIFI_OK = false;
	
}


void bsp_SetIMU_OK(void)
{
	allSelfCheck.isIMU_OK = true;
}

void bsp_SetWIFI_OK(void)
{
	allSelfCheck.isWIFI_OK = true;
}


bool bsp_IsRunningAllSelfCheck(void)
{
	return allSelfCheck.isRunning;
}


/**************************************************************************************************
*  {
*  	    uint8_t data[4] = {1,2,3,4};
*  	    bsp_AllSelfCheckSendFrame(2 , 1 , 1 , 1 , data , 4);
*  	    
*  	    
*  	    发送（实测）
*  	    AA AA 00 14 FF EB 00 02 00 01 00 01 00 01 01 02 03 04 2B A1 
*  	
*  }
***************************************************************************************************/


void bsp_AllSelfCheckSendFrame(uint16_t tx , uint16_t rx , uint16_t main , uint16_t sub , uint8_t data[] , uint16_t size)
{
	uint16_t i = 0 ;
	uint8_t arr[64] = {0};
	uint16_t data_size = size ;                 /*帧里面的数据的长度*/
	uint16_t frame_size = data_size + 16 ;            /*帧总长度*/
	
	uint16_t tx_addr = tx;
	uint16_t rx_addr = rx;
	uint16_t main_section = main;
	uint16_t sub_section = sub;
	uint16_t crc_ret = 0 ;
	
	/*帧头*/
	arr[0] = 0xAA;
	arr[1] = 0xAA;
	/*帧总长度*/
	arr[2] = (frame_size >> 8 ) & 0x00FF;
	arr[3] =  frame_size        & 0x00FF;
	
	arr[4] = ((~frame_size) >> 8 ) & 0x00FF;
	arr[5] =  (~frame_size)        & 0x00FF;
	/*发送方地址*/
	arr[6] = (tx_addr >> 8 ) & 0x00FF;
	arr[7] =  tx_addr        & 0x00FF;
	/*接收方地址*/
	arr[8] = (rx_addr >> 8 ) & 0x00FF;
	arr[9] =  rx_addr        & 0x00FF;
	/*主功能*/
	arr[10] = (main_section >> 8 ) & 0x00FF;
	arr[11] =  main_section        & 0x00FF;
	/*子功能*/
	arr[12] = (sub_section >> 8 ) & 0x00FF;
	arr[13] =  sub_section        & 0x00FF;
	
	/*数据*/
	for(i=0;i<size;++i)
	{
		arr[14+i] = data[i];
	}
	
	/*校验*/
	crc_ret = CRC16_CALC(arr,frame_size-2);
	arr[frame_size-2] = (crc_ret >> 8 ) & 0x00FF;
	arr[frame_size-1] =  crc_ret        & 0x00FF;
	
	/*发送*/
	comSendBuf(COM4,arr,frame_size);
}



void bsp_AllSelfCheckProc(void)
{
	if(!allSelfCheck.isRunning)
		return;
	
	
	switch(allSelfCheck.action)
	{
		case 0: /*电池电压*/
		{		
			float batteryVoltage = bsp_GetFeedbackVoltage(eBatteryVoltage);
			batteryVoltage = (batteryVoltage * 430 / 66.5) + batteryVoltage + 0.2F; 
			if( batteryVoltage >= 12.0F && batteryVoltage <= 16.5F)
			{
				bsp_SendPassFail(1,1);
			}
			else
			{
				bsp_SendPassFail(1,0);
				
				allSelfCheck.isAllOK = false;
			}
			
			allSelfCheck.delay = xTaskGetTickCount();
			++allSelfCheck.action;
			
		}break;
		
		case 1: /*红外通信*/
		{		
			if(bsp_IsInitAW9523B_OK()) 
			{
				bsp_SendPassFail(2,1);
			}
			else
			{
				bsp_SendPassFail(2,0);
				
				allSelfCheck.isAllOK = false;
			}
			
			allSelfCheck.delay = xTaskGetTickCount();
			++allSelfCheck.action;
			
		}break;

		case 2: /*红外读值*/
		{		
			if(xTaskGetTickCount() - allSelfCheck.delay >= 1000)
			{
				float vol = bsp_GetInfraRedAdcVoltage(IR7);
				
				if(vol >= 1000 && vol<=2000)
				{
					bsp_SendPassFail(3,1);
				}
				else
				{
					bsp_SendPassFail(3,0);
					
					allSelfCheck.isAllOK = false;
				}
				
				allSelfCheck.delay = xTaskGetTickCount();
				++allSelfCheck.action;
			}
		}break;
		
		case 3: /*红外接收*/
		{		
			if(IR_RX_CH1() == 0)
			{
				bsp_SendPassFail(4,1);
			}
			else
			{
				bsp_SendPassFail(4,0);
				
				allSelfCheck.isAllOK = false;
			}
			
			/*CH2*/
			if(IR_RX_CH2() == 0)
			{
				bsp_SendPassFail(5,1);
			}
			else
			{
				bsp_SendPassFail(5,0);
				
				allSelfCheck.isAllOK = false;
			}
			
			/*CH3*/
			if(IR_RX_CH3() == 0)
			{
				bsp_SendPassFail(6,1);
			}
			else
			{
				bsp_SendPassFail(6,0);
				
				allSelfCheck.isAllOK = false;
			}
			
			/*CH4*/
			if(IR_RX_CH4() == 0)
			{
				bsp_SendPassFail(7,1);
			}
			else
			{
				bsp_SendPassFail(7,0);
				
				allSelfCheck.isAllOK = false;
			}
			
			allSelfCheck.delay = xTaskGetTickCount();
			++allSelfCheck.action;
			
		}break;
		
		case 4: /*悬崖*/
		{
			float cliff_arr[3] = {0};
			bsp_GetCliffSub(cliff_arr);
			
			
			//887.00 1364.00  1681.00 
			
			if(cliff_arr[0] >= 600 && cliff_arr[0] <= 1200)
			{
				bsp_SendPassFail(8,1);
			}
			else
			{
				bsp_SendPassFail(8,0);
				
				allSelfCheck.isAllOK = false;
			}
			
			if(cliff_arr[1] >= 1000 && cliff_arr[1] <= 1600)
			{
				bsp_SendPassFail(9,1);
			}
			else
			{
				bsp_SendPassFail(9,0);
				
				allSelfCheck.isAllOK = false;
			}
			
			if(cliff_arr[2] >= 1200 && cliff_arr[2] <= 2000)
			{
				bsp_SendPassFail(10,1);
			}
			else
			{
				bsp_SendPassFail(10,0);
				
				allSelfCheck.isAllOK = false;
			}
			
			++allSelfCheck.action;
		}break;
		
		case 5:
		{
			bsp_StartVacuum();
			bsp_MotorCleanSetPWM(MotorRollingBrush, CCW , CONSTANT_HIGH_PWM*0.9F);
			bsp_MotorCleanSetPWM(MotorSideBrush, CW , CONSTANT_HIGH_PWM*0.7F);
			bsp_SetMotorSpeed(MotorLeft,bsp_MotorSpeedMM2Pulse(250));
			bsp_SetMotorSpeed(MotorRight,bsp_MotorSpeedMM2Pulse(250));
			
			allSelfCheck.delay = xTaskGetTickCount();
			++allSelfCheck.action;
		}break;
		
		case 6: /*左轮电流  右轮电流  左轮转速  右轮转速*/
		{
			if(xTaskGetTickCount() - allSelfCheck.delay >= 2000)
			{
				float wheelL = bsp_GetFeedbackVoltage(eMotorLeft);
				float wheelR = bsp_GetFeedbackVoltage(eMotorRight);
				float roll = bsp_GetFeedbackVoltage(eRollingBrush);
				float vacuum = bsp_GetFeedbackVoltage(eVacuum);
				float sideBrush = bsp_GetFeedbackVoltage(eSideBrush);
				
				int32_t speed_l = bsp_MotorGetSpeed(MotorLeft);
				int32_t speed_r = bsp_MotorGetSpeed(MotorRight);
				
				wheelL = wheelL * 1000.0F * 1000.0F / 33.0F / 50.0F;
				wheelR = wheelR * 1000.0F * 1000.0F / 33.0F / 50.0F;
				roll = roll * 1000.0F * 1000.0F / 33.0F / 50.0F;
				vacuum = vacuum * 1000.0F * 1000.0F / 33.0F / 50.0F;
				sideBrush = sideBrush * 1000.0F * 1000.0F / 100.0F / 50.0F;
				
				
				if(wheelL >= 60 && wheelL <= 200)
				{
					bsp_SendPassFail(11,1);
				}
				else
				{
					bsp_SendPassFail(11,0);
					
					allSelfCheck.isAllOK = false;
				}
				
				
				if(wheelR >= 50 && wheelR <= 150)
				{
					bsp_SendPassFail(12,1);
				}
				else
				{
					bsp_SendPassFail(12,0);
					
					allSelfCheck.isAllOK = false;
				}
				
				if(speed_l >= 220 && speed_l <= 280)
				{
					bsp_SendPassFail(13,1);
				}
				else
				{
					bsp_SendPassFail(13,0);
					
					allSelfCheck.isAllOK = false;
				}
				
				if(speed_r >= 220 && speed_r <= 280)
				{
					bsp_SendPassFail(14,1);
				}
				else
				{
					bsp_SendPassFail(14,0);
					
					allSelfCheck.isAllOK = false;
				}
				
				if(vacuum >= 300 && vacuum <= 600)
				{
					bsp_SendPassFail(15,1);
				}
				else
				{
					bsp_SendPassFail(15,0);
					
					allSelfCheck.isAllOK = false;
				}
				
				if(roll >= 150 && roll <= 300)
				{
					bsp_SendPassFail(16,1);
				}
				else
				{
					bsp_SendPassFail(16,0);
					
					allSelfCheck.isAllOK = false;
				}
				
				if(sideBrush >= 50 && sideBrush <= 200)
				{
					bsp_SendPassFail(17,1);
				}
				else
				{
					bsp_SendPassFail(17,0);
					
					allSelfCheck.isAllOK = false;
				}
				
				bsp_SendPassFail(18,1);
				
				bsp_StopVacuum();
				bsp_MotorCleanSetPWM(MotorRollingBrush, CCW , CONSTANT_HIGH_PWM*0.0F);
				bsp_MotorCleanSetPWM(MotorSideBrush, CW , CONSTANT_HIGH_PWM*0.0F);
				bsp_SetMotorSpeed(MotorLeft,bsp_MotorSpeedMM2Pulse(0));
				bsp_SetMotorSpeed(MotorRight,bsp_MotorSpeedMM2Pulse(0));
				
				++allSelfCheck.action;
			}
		}break;
		
		case 7: /*离地*/
		{
			OffSiteState state = bsp_OffSiteGetState();
			
			if(state == OffSiteBoth)
			{
				bsp_SendPassFail(19,1);
				bsp_SendPassFail(20,1);
			}
			else if(state == OffSiteNone)
			{
				bsp_SendPassFail(19,0);
				bsp_SendPassFail(20,0);
				
				allSelfCheck.isAllOK = false;
			}
			else if(state == OffSiteLeft)
			{
				bsp_SendPassFail(19,1);
				bsp_SendPassFail(20,0);
				
				allSelfCheck.isAllOK = false;
			}
			else if(state == OffSiteRight)
			{
				bsp_SendPassFail(19,0);
				bsp_SendPassFail(20,1);
				
				allSelfCheck.isAllOK = false;
			}
			
			++allSelfCheck.action;
		}break;
		
		case 8: /*陀螺仪*/
		{
			if(allSelfCheck.isIMU_OK)
			{
				bsp_SendPassFail(21,1);
			}
			else
			{
				bsp_SendPassFail(21,0);
				
				allSelfCheck.isAllOK = false;
			}
			
			if(bsp_DustBoxGetState() == DustBoxInside)
			{
				bsp_SendPassFail(27,1);
			}
			else
			{
				bsp_SendPassFail(27,0);
				
				allSelfCheck.isAllOK = false;
			}
			
			++allSelfCheck.action;
		}break;
		
		case 9:
		{
			if(IR_KEY_CLEAN() == 0)
			{
				bsp_SendPassFail(22,1);
				++allSelfCheck.action;
			}
		}break;
		
		case 10:
		{
			if(IR_KEY_POWER() == 0)
			{
				bsp_SendPassFail(23,1);
				++allSelfCheck.action;
			}
		}break;
		
		case 11:
		{
			if(IR_KEY_CHARGE() == 0)
			{
				bsp_SendPassFail(24,1);
				++allSelfCheck.action;
			}
		}break;
		
		case 12:
		{
			if(bsp_CollisionScan() == CollisionLeft)
			{
				bsp_SendPassFail(25,1);
				++allSelfCheck.action;
			}
		}break;
		
		case 13:
		{
			if(bsp_CollisionScan() == CollisionRight)
			{
				bsp_SendPassFail(26,1);
				++allSelfCheck.action;
			}
		}break;
		
		
		case 14:
		{
			if(allSelfCheck.isAllOK)
			{
				bsp_SendPassFail(128,1);
			}
			else
			{
				bsp_SendPassFail(129,1);
			}
			allSelfCheck.delay = xTaskGetTickCount();
			++allSelfCheck.action;
			
		}break;
		
		case 15:
		{
			if(xTaskGetTickCount() - allSelfCheck.delay >= 1000)
			{
				/*进入休眠模式*/
				bsp_SperkerPlay(Song31);
				vTaskDelay(10);	
				while(bsp_SpeakerIsBusy()){}
				
				bsp_ClearKey();
				bsp_EnterStopMODE();
					
				++allSelfCheck.action;
			}
		}break;
		
	}
}





