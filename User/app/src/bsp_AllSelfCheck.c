#include "bsp.h"

#define IR_RX_CH1()      GPIO_ReadInputDataBit(GPIOC,GPIO_Pin_6)
#define IR_RX_CH2()      GPIO_ReadInputDataBit(GPIOC,GPIO_Pin_7)
#define IR_RX_CH3()      GPIO_ReadInputDataBit(GPIOC,GPIO_Pin_8)
#define IR_RX_CH4()      GPIO_ReadInputDataBit(GPIOC,GPIO_Pin_9)

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
	
	
}AllSelfCheck;

static AllSelfCheck allSelfCheck;

void bsp_StartAllSelfCheck(void)
{
	allSelfCheck.action = 0 ;
	allSelfCheck.delay = 0 ;
	
	allSelfCheck.isIMU_OK = false;
	allSelfCheck.isWIFI_OK = false;
	
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
		case 0: /*判断红外条是否能够初始化成功*/
		{		
			if(bsp_IsInitAW9523B_OK()) 
			{
				allSelfCheck.isIR_InitOK = true;
				DEBUG("SELF:红外初始化 成功\r\n");
			}
			else
			{
				allSelfCheck.isIR_InitOK = false;
				DEBUG("SELF:红外初始化 失败\r\n");
			}
			allSelfCheck.delay = xTaskGetTickCount();
			
			++allSelfCheck.action;
			
		}break;
		
		case 1: /*读值红外ADC值*/
		{
			if(xTaskGetTickCount() - allSelfCheck.delay >= 2000)
			{
				float vol = bsp_GetInfraredVoltageRight();
				if(allSelfCheck.isIR_InitOK && vol >= 50 && vol<=200)
				{
					allSelfCheck.isIR_ADC_OK = true;
				}
				else
				{
					allSelfCheck.isIR_ADC_OK = false;
				}
				++allSelfCheck.action;
			}
		}break;
		
		case 2: /*读IO口模拟的红外发射*/
		{
			/*CH1*/
			if(IR_RX_CH1() == 0)
			{
				allSelfCheck.isIR_RX_CH1_OK = true;
			}
			else
			{
				allSelfCheck.isIR_RX_CH1_OK = false;
			}
			
			/*CH2*/
			if(IR_RX_CH2() == 0)
			{
				allSelfCheck.isIR_RX_CH2_OK = true;
			}
			else
			{
				allSelfCheck.isIR_RX_CH2_OK = false;
			}
			
			/*CH3*/
			if(IR_RX_CH3() == 0)
			{
				allSelfCheck.isIR_RX_CH3_OK = true;
			}
			else
			{
				allSelfCheck.isIR_RX_CH3_OK = false;
			}
			
			/*CH4*/
			if(IR_RX_CH4() == 0)
			{
				allSelfCheck.isIR_RX_CH4_OK = true;
			}
			else
			{
				allSelfCheck.isIR_RX_CH4_OK = false;
			}
			
			++allSelfCheck.action;
		}break;
		
		case 3: /*模拟离地*/
		{
			if(bsp_OffSiteGetState() == OffSiteBoth)
			{
				allSelfCheck.isOffsiteL_OK = true;
				allSelfCheck.isOffsiteR_OK = true;
			}
			else if(bsp_OffSiteGetState() == OffSiteNone)
			{
				allSelfCheck.isOffsiteL_OK = false;
				allSelfCheck.isOffsiteR_OK = false;
			}
			else if(bsp_OffSiteGetState() == OffSiteLeft)
			{
				allSelfCheck.isOffsiteL_OK = true;
				allSelfCheck.isOffsiteR_OK = false;
			}
			else if(bsp_OffSiteGetState() == OffSiteRight)
			{
				allSelfCheck.isOffsiteL_OK = false;
				allSelfCheck.isOffsiteR_OK = true;
			}
			++allSelfCheck.action;
		}break;
		
		case 4: /*悬崖*/
		{
			allSelfCheck.isCliffL_OK = true;
			allSelfCheck.isCliffM_OK = true;
			allSelfCheck.isCliffR_OK = true;
			
			++allSelfCheck.action;
		}break;
		
		
		case 5: /*尘盒*/
		{
			if(bsp_DustBoxGetState() == DustBoxInside)
			{
				allSelfCheck.isDustBox_OK = true;
			}
			else
			{
				allSelfCheck.isDustBox_OK = false;
			}
		}break;
		
		case 7: /*机器动起来，然后测试电流*/
		{
			bsp_StartVacuum();
			bsp_MotorCleanSetPWM(MotorRollingBrush, CCW , CONSTANT_HIGH_PWM*0.9F);
			bsp_MotorCleanSetPWM(MotorSideBrush, CW , CONSTANT_HIGH_PWM*0.7F);
			bsp_SetMotorSpeed(MotorLeft,bsp_MotorSpeedMM2Pulse(250));
			bsp_SetMotorSpeed(MotorRight,bsp_MotorSpeedMM2Pulse(250));
			
			allSelfCheck.delay = xTaskGetTickCount();
			
			++allSelfCheck.action;
		}break;
		
		case 8: /*然后测试电流*/
		{
			if(xTaskGetTickCount() - allSelfCheck.delay >= 2000)
			{
				
				
				bsp_StopVacuum();
				bsp_MotorCleanSetPWM(MotorRollingBrush, CCW , CONSTANT_HIGH_PWM*0.0F);
				bsp_MotorCleanSetPWM(MotorSideBrush, CW , CONSTANT_HIGH_PWM*0.0F);
				bsp_SetMotorSpeed(MotorLeft,bsp_MotorSpeedMM2Pulse(0));
				bsp_SetMotorSpeed(MotorRight,bsp_MotorSpeedMM2Pulse(0));
				
				++allSelfCheck.action;
			}
		}break;
		
		case 9: /*判断陀螺仪 WIFI模块*/
		{
			if(allSelfCheck.isIMU_OK)
			{
				
			}
			
			if(allSelfCheck.isWIFI_OK)
			{
				
			}
		}break;

	}
}


