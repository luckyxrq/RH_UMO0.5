#include "bsp.h"

#define MAX_ANALYSIS_LEN	    512                 /*一帧数据的最大长度*/
#define MIN_ANALYSIS_LEN        16                  /*一帧数据的最小长度*/





/*
**********************************************************************************************************
											变量声明
**********************************************************************************************************
*/
static uint8_t analysisBuf[MAX_ANALYSIS_LEN] = {0};    /*用于解析帧数据*/
CMD_FRAME cmd_frame_tx;
CMD_FRAME cmd_frame_rx;

static uint8_t isCmdStartUpload = 0; /* 1 表示开始上传数据到PC */

/*
**********************************************************************************************************
											函数声明
**********************************************************************************************************
*/

static void bsp_SendMCU_Ver(void);
static void bsp_SendAllCalibration(void);


uint8_t GetCmdStartUpload(void)
{
	return isCmdStartUpload;
}


void bsp_ExexCmd(void)
{
	RTT("crc success(%d-%d)\r\n",cmd_frame_rx.main_sec,cmd_frame_rx.sub_sec);
	
	
	if(cmd_frame_rx.main_sec == 2)
	{
		switch(cmd_frame_rx.sub_sec)
		{
			case 1:
			{
				if(cmd_frame_rx.union_para.sw == 1)
				{
					bsp_MotorCleanSetPWM(MotorRollingBrush, CW , CONSTANT_HIGH_PWM*0.7F);
					bsp_MotorCleanSetPWM(MotorSideBrush, CCW , CONSTANT_HIGH_PWM*0.7F);

					bsp_SetMotorSpeed(MotorLeft,bsp_MotorSpeedMM2Pulse(300));
					bsp_SetMotorSpeed(MotorRight,bsp_MotorSpeedMM2Pulse(300));
					bsp_StartVacuum(VACUUM_STRENGTH);
				}
				else
				{
					bsp_MotorCleanSetPWM(MotorRollingBrush, CCW , CONSTANT_HIGH_PWM*0.0F);
					bsp_MotorCleanSetPWM(MotorSideBrush, CW , CONSTANT_HIGH_PWM*0.0F);
					bsp_SetMotorSpeed(MotorLeft,bsp_MotorSpeedMM2Pulse(0));
					bsp_SetMotorSpeed(MotorRight,bsp_MotorSpeedMM2Pulse(0));
					bsp_StopVacuum();
				}
			}break;
			case 2:
			{
				if(cmd_frame_rx.union_para.sw == 1)
				{
					bsp_StartVacuum(VACUUM_STRENGTH);
				}
				else
				{
					bsp_StopVacuum();
				}
			}break;
			case 3:
			{
				if(cmd_frame_rx.union_para.sw == 1)
				{
					bsp_SetMotorSpeed(MotorLeft,bsp_MotorSpeedMM2Pulse(300));
				}
				else
				{
					bsp_SetMotorSpeed(MotorLeft,bsp_MotorSpeedMM2Pulse(0));
				}
			}break;
			case 4:
			{
				if(cmd_frame_rx.union_para.sw == 1)
				{
					bsp_SetMotorSpeed(MotorRight,bsp_MotorSpeedMM2Pulse(300));
				}
				else
				{
					bsp_SetMotorSpeed(MotorRight,bsp_MotorSpeedMM2Pulse(0));
				}
			}break;
			case 5:
			{
				if(cmd_frame_rx.union_para.sw == 1)
				{
					bsp_MotorCleanSetPWM(MotorSideBrush, CCW , CONSTANT_HIGH_PWM*0.7F);
				}
				else
				{
					bsp_MotorCleanSetPWM(MotorSideBrush, CCW , CONSTANT_HIGH_PWM*0.0F);
				}
			}break;
			case 6:
			{
				if(cmd_frame_rx.union_para.sw == 1)
				{
					bsp_MotorCleanSetPWM(MotorRollingBrush, CW , CONSTANT_HIGH_PWM*0.7F);
				}
				else
				{
					bsp_MotorCleanSetPWM(MotorRollingBrush, CW , CONSTANT_HIGH_PWM*0.0F);
				}
			}break;
			case 7:
			{
				if(cmd_frame_rx.union_para.sw == 1)
				{
					isCmdStartUpload = 1 ;
				}
				else
				{
					isCmdStartUpload = 0 ;
				}
			}break;
			default: break;
		}
	}
	else if(cmd_frame_rx.main_sec == 4)
	{
		switch(cmd_frame_rx.sub_sec)
		{
			case 1:
			{
				bsp_SendMCU_Ver();
			}break;
			default: break;
		}
	}
	else if(cmd_frame_rx.main_sec == 5)
	{
		switch(cmd_frame_rx.sub_sec)
		{
			case 7:
			{
				bsp_SendAllCalibration();
			}break;
			
			case 8:
			{
				bsp_SetParaCliff_L(cmd_frame_rx.union_para.calibration.Cliff_L);
				bsp_SetParaCliff_M(cmd_frame_rx.union_para.calibration.Cliff_M);
				bsp_SetParaCliff_R(cmd_frame_rx.union_para.calibration.Cliff_R);
				bsp_SetParaEdge_L(cmd_frame_rx.union_para.calibration.Edge_L);
				bsp_SetParaEdge_R(cmd_frame_rx.union_para.calibration.Edge_R);
				bsp_SetParaErLangShen(cmd_frame_rx.union_para.calibration.ErLangShen);
			}break;
			
			default: break;
		}
	}
}

void bsp_ComAnalysis(void)
{
	uint8_t ch = 0;
	COM_PORT_E port ; 
    /*接收索引，之前不是static 不科学*/
    static uint16_t index = 0 ;
    
    /*帧结构*/
    static uint16_t frame_len = 0 ;
    static uint16_t tx_addr = 0 ;
    static uint16_t rx_addr = 0 ;
    static uint16_t main_sec = 0 ;
    static uint16_t sub_sec = 0 ;
    
    UNUSED(frame_len);
    UNUSED(tx_addr);
    UNUSED(rx_addr);
    UNUSED(main_sec);
    UNUSED(sub_sec);
	
	/*选定串口*/
	port = COM2;
	
    while(comGetChar( port , &ch))   
    {
        //qDebug("%02X ",ch);
        
        analysisBuf[index % MAX_ANALYSIS_LEN] = ch ;
        index++;
        
        if(index == 1)       /*前面2个是帧头*/
        {
            if(ch != 0xAA)
            {
                index = 0 ;
            }
        }
        else if(index == 2)  /*前面2个是帧头*/
        {
            if(ch != 0xAA)
            {
                index = 0 ;
            }
        }
        else if(index == 4)  /*帧长度*/
        {
            frame_len = ((analysisBuf[3] << 8) | analysisBuf[2]) & 0xFFFF;
            
            if(frame_len < MIN_ANALYSIS_LEN || frame_len > MAX_ANALYSIS_LEN)
            {
                DEBUG("err:frame_len %d",frame_len);
                index = 0 ;
            }
            
        }
        else if(index == 6)  /*帧长度 取反*/
        {
            uint16_t reverse = ((analysisBuf[5] << 8) | analysisBuf[4]) & 0xFFFF;
            
            /*这里 &0x00FF 非常有必要！！！*/
            if( ((~reverse)&0xFFFF) != frame_len )
            {
                DEBUG("err:reverse %04X",frame_len);
                index = 0 ;
            }
            
        }
        else if(index == 8)  /*发送方地址*/
        {
            tx_addr = ((analysisBuf[7] << 8) | analysisBuf[6]) & 0xFFFF;
            
        }
        else if(index == 10)  /*接收方地址*/
        {
            rx_addr = ((analysisBuf[9] << 8) | analysisBuf[8]) & 0xFFFF;
            
        }
        else if(index == 12)  /*主功能*/
        {
            main_sec = ((analysisBuf[11] << 8) | analysisBuf[10]) & 0xFFFF;
            
        }
        else if(index == 14)  /*子功能*/
        {
            sub_sec = ((analysisBuf[13] << 8) | analysisBuf[12]) & 0xFFFF;
            
        }
        else if(index >= MIN_ANALYSIS_LEN && index >= frame_len)  /*一帧数据接收完毕了，这里的16很有灵性*/
        {
            uint16_t crc_ret = CRC16_Modbus(analysisBuf,frame_len);
            if(crc_ret == 0)
            {
				memset(&cmd_frame_rx,0,sizeof(CMD_FRAME));
				memcpy(&cmd_frame_rx,analysisBuf,sizeof(CMD_FRAME));

                bsp_ExexCmd();
            }
            else
            {
                RTT("crc faile(%d-%d):%04X\r\n",main_sec,sub_sec,crc_ret);
            }
            index = 0 ;
        }
    }
}



extern unsigned int vTaskMapping_cnt;
void bsp_SendReportFrameWithCRC16(void)
{
	memset(&cmd_frame_tx,0,sizeof(CMD_FRAME));
	
	cmd_frame_tx.union_para.mcu_frame.dustBox = bsp_DustBoxGetState();
	
	cmd_frame_tx.union_para.mcu_frame.wheelSpeedL = bsp_MotorGetSpeed(MotorLeft);
	cmd_frame_tx.union_para.mcu_frame.wheelSpeedR = bsp_MotorGetSpeed(MotorRight);

	cmd_frame_tx.union_para.mcu_frame.wheelPulseL = vTaskMapping_cnt;//bsp_MotorGetPulseVector(MotorLeft);
	cmd_frame_tx.union_para.mcu_frame.wheelPulseR = bsp_MotorGetPulseVector(MotorRight);

	cmd_frame_tx.union_para.mcu_frame.x_pos = bsp_GetCurrentPosX();
	cmd_frame_tx.union_para.mcu_frame.y_pos = bsp_GetCurrentPosY();

	cmd_frame_tx.union_para.mcu_frame.cliffMV_L = bsp_GetCliffRealVal(CliffLeft); 
	cmd_frame_tx.union_para.mcu_frame.cliffMV_M = bsp_GetCliffRealVal(CliffMiddle); 
	cmd_frame_tx.union_para.mcu_frame.cliffMV_R = bsp_GetCliffRealVal(CliffRight); 

	cmd_frame_tx.union_para.mcu_frame.yaw = bsp_IMU_GetData(YAW)*100;
	cmd_frame_tx.union_para.mcu_frame.pitch = bsp_IMU_GetData(PITCH)*100;
	cmd_frame_tx.union_para.mcu_frame.roll = bsp_IMU_GetData(ROLL)*100;

	cmd_frame_tx.union_para.mcu_frame.irMV[0] = bsp_GetInfraRedAdcVoltage(IR0); 
	cmd_frame_tx.union_para.mcu_frame.irMV[1] = bsp_GetInfraRedAdcVoltage(IR1); 
	cmd_frame_tx.union_para.mcu_frame.irMV[2] = bsp_GetInfraRedAdcVoltage(IR2); 
	cmd_frame_tx.union_para.mcu_frame.irMV[3] = bsp_GetInfraRedAdcVoltage(IR3); 
	cmd_frame_tx.union_para.mcu_frame.irMV[4] = bsp_GetInfraRedAdcVoltage(IR4); 
	cmd_frame_tx.union_para.mcu_frame.irMV[5] = bsp_GetInfraRedAdcVoltage(IR5); 
	cmd_frame_tx.union_para.mcu_frame.irMV[6] = bsp_GetInfraRedAdcVoltage(IR6); 
	cmd_frame_tx.union_para.mcu_frame.irMV[7] = bsp_GetInfraRedAdcVoltage(IR7); 
	cmd_frame_tx.union_para.mcu_frame.irMV[8] = bsp_GetInfraRedAdcVoltage(IR8); 
	cmd_frame_tx.union_para.mcu_frame.irMV[9] = bsp_GetInfraRedAdcVoltage(IR9); 

	cmd_frame_tx.union_para.mcu_frame.irRX[0][0] = bsp_IR_GetRev(IR_CH1,IR_TX_SITE_LEFT); 
	cmd_frame_tx.union_para.mcu_frame.irRX[0][1] = bsp_IR_GetRev(IR_CH1,IR_TX_SITE_CENTER); 
	cmd_frame_tx.union_para.mcu_frame.irRX[0][2] = bsp_IR_GetRev(IR_CH1,IR_TX_SITE_RIGHT); 

	cmd_frame_tx.union_para.mcu_frame.irRX[1][0] = bsp_IR_GetRev(IR_CH2,IR_TX_SITE_LEFT); 
	cmd_frame_tx.union_para.mcu_frame.irRX[1][1] = bsp_IR_GetRev(IR_CH2,IR_TX_SITE_CENTER); 
	cmd_frame_tx.union_para.mcu_frame.irRX[1][2] = bsp_IR_GetRev(IR_CH2,IR_TX_SITE_RIGHT);

	cmd_frame_tx.union_para.mcu_frame.irRX[2][0] = bsp_IR_GetRev(IR_CH3,IR_TX_SITE_LEFT); 
	cmd_frame_tx.union_para.mcu_frame.irRX[2][1] = bsp_IR_GetRev(IR_CH3,IR_TX_SITE_CENTER); 
	cmd_frame_tx.union_para.mcu_frame.irRX[2][2] = bsp_IR_GetRev(IR_CH3,IR_TX_SITE_RIGHT);

	cmd_frame_tx.union_para.mcu_frame.irRX[3][0] = bsp_IR_GetRev(IR_CH4,IR_TX_SITE_LEFT); 
	cmd_frame_tx.union_para.mcu_frame.irRX[3][1] = bsp_IR_GetRev(IR_CH4,IR_TX_SITE_CENTER); 
	cmd_frame_tx.union_para.mcu_frame.irRX[3][2] = bsp_IR_GetRev(IR_CH4,IR_TX_SITE_RIGHT);

	cmd_frame_tx.union_para.mcu_frame.offsiteSW = bsp_OffSiteGetState();
	cmd_frame_tx.union_para.mcu_frame.collision = bsp_CollisionScan();

	cmd_frame_tx.union_para.mcu_frame.mA_wheelL           = bsp_GetVoltageAfterFilter(eMotorLeft);
	cmd_frame_tx.union_para.mcu_frame.mA_wheelR           = bsp_GetVoltageAfterFilter(eMotorRight);
	cmd_frame_tx.union_para.mcu_frame.mA_roll             = bsp_GetVoltageAfterFilter(eRollingBrush);
	cmd_frame_tx.union_para.mcu_frame.mA_sideBrush        = bsp_GetVoltageAfterFilter(eSideBrush);
	cmd_frame_tx.union_para.mcu_frame.mA_vacuum           = bsp_GetVoltageAfterFilter(eVacuum);
	cmd_frame_tx.union_para.mcu_frame.v_batteryVoltage    = bsp_GetVoltageAfterFilter(eBatteryVoltage);
	cmd_frame_tx.union_para.mcu_frame.mA_batteryCurrent   = bsp_GetVoltageAfterFilter(eBatteryCurrent);

	/*头文件新增内容*/
	cmd_frame_tx.union_para.mcu_frame.keyPinState = bsp_GetKeyPinState();
	cmd_frame_tx.union_para.mcu_frame.irRxPinState = bsp_GetIrRxPinState();
	cmd_frame_tx.union_para.mcu_frame.isAwIniOK = bsp_IsInitAW9523B_OK();
	cmd_frame_tx.union_para.mcu_frame.offsitePinState = bsp_GetOffsitePinState();
	cmd_frame_tx.union_para.mcu_frame.isAngleInitOk = bsp_IsAngleInitOK();
	
	cmd_frame_tx.union_para.mcu_frame.strategyMajor = bsp_GetStrategy_MajorIndex();
	cmd_frame_tx.union_para.mcu_frame.strategyMinor = bsp_GetStrategy_MinorIndex();

	cmd_frame_tx.head = 0xAAAA;
	cmd_frame_tx.frame_len = sizeof(CMD_FRAME) & 0xFFFF;
	cmd_frame_tx.frame_len_reverse = (~cmd_frame_tx.frame_len) & 0xFFFF;
	
	cmd_frame_tx.tx_addr = 0x02;
	cmd_frame_tx.rx_addr = 0x01;
	
	cmd_frame_tx.main_sec = 3;
	cmd_frame_tx.sub_sec = 1;
	
	uint16_t ret = CRC16_Modbus((uint8_t*)&cmd_frame_tx,sizeof(CMD_FRAME)-2);
	cmd_frame_tx.crc16 = ((ret>>8)&0x00FF)  | ((ret<<8)&0xFF00);
	
	comSendBuf(COM2,(uint8_t*)&cmd_frame_tx,sizeof(CMD_FRAME));
}


static void bsp_SendMCU_Ver(void)
{
	memset(&cmd_frame_tx,0,sizeof(CMD_FRAME));
	
	cmd_frame_tx.union_para.mcu_ver = PARAM_VER;

	cmd_frame_tx.head = 0xAAAA;
	cmd_frame_tx.frame_len = sizeof(CMD_FRAME) & 0xFFFF;
	cmd_frame_tx.frame_len_reverse = (~cmd_frame_tx.frame_len) & 0xFFFF;
	
	cmd_frame_tx.tx_addr = 0x02;
	cmd_frame_tx.rx_addr = 0x01;
	
	cmd_frame_tx.main_sec = 4;
	cmd_frame_tx.sub_sec = 2;
	
	uint16_t ret = CRC16_Modbus((uint8_t*)&cmd_frame_tx,sizeof(CMD_FRAME)-2);
	cmd_frame_tx.crc16 = ((ret>>8)&0x00FF)  | ((ret<<8)&0xFF00);
	
	comSendBuf(COM2,(uint8_t*)&cmd_frame_tx,sizeof(CMD_FRAME));
}



static void bsp_SendAllCalibration(void)
{
	memset(&cmd_frame_tx,0,sizeof(CMD_FRAME));
	
	cmd_frame_tx.union_para.calibration.Cliff_L = bsp_GetParaCliff_L();
	cmd_frame_tx.union_para.calibration.Cliff_M = bsp_GetParaCliff_M();
	cmd_frame_tx.union_para.calibration.Cliff_R = bsp_GetParaCliff_R();
	
	cmd_frame_tx.union_para.calibration.Edge_L = bsp_GetParaEdge_L();
	cmd_frame_tx.union_para.calibration.Edge_R = bsp_GetParaEdge_R();
	
	cmd_frame_tx.union_para.calibration.ErLangShen = bsp_GetParaErLangShen();
	

	cmd_frame_tx.head = 0xAAAA;
	cmd_frame_tx.frame_len = sizeof(CMD_FRAME) & 0xFFFF;
	cmd_frame_tx.frame_len_reverse = (~cmd_frame_tx.frame_len) & 0xFFFF;
	
	cmd_frame_tx.tx_addr = 0x02;
	cmd_frame_tx.rx_addr = 0x01;
	
	cmd_frame_tx.main_sec = 5;
	cmd_frame_tx.sub_sec = 7;
	
	uint16_t ret = CRC16_Modbus((uint8_t*)&cmd_frame_tx,sizeof(CMD_FRAME)-2);
	cmd_frame_tx.crc16 = ((ret>>8)&0x00FF)  | ((ret<<8)&0xFF00);
	
	comSendBuf(COM2,(uint8_t*)&cmd_frame_tx,sizeof(CMD_FRAME));
}

