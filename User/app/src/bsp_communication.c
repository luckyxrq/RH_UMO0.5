#include "bsp.h"

#define MAX_ANALYSIS_LEN	    512                 /*一帧数据的最大长度*/
#define MIN_ANALYSIS_LEN        16                  /*一帧数据的最小长度*/

/*按照1字节对齐，便于存储到uint8_t类型buf*/
#pragma pack(1)
typedef struct
{
	uint16_t head;
	
	uint16_t frame_len;
	uint16_t frame_len_reverse;
	
    uint16_t tx_addr;
    uint16_t rx_addr;
	
    uint16_t main_sec ;
    uint16_t sub_sec ;
	
	/*********数据部分开始***********/
    
	uint8_t isOpen; /*上传数据与否，1：上传，0：不上传*/
    
	/*********数据部分结束***********/
	
	uint16_t crc16;
}CMD_START_UPLOAD;
#pragma pack()


/*按照1字节对齐，便于存储到uint8_t类型buf*/
#pragma pack(1)
typedef struct
{
	uint16_t head;
	
	uint16_t frame_len;
	uint16_t frame_len_reverse;
	
    uint16_t tx_addr;
    uint16_t rx_addr;
	
    uint16_t main_sec ;
    uint16_t sub_sec ;
	
	/*********数据部分开始***********/
    
	uint8_t isOpen; /*上传数据与否，1：上传，0：不上传*/
    
	/*********数据部分结束***********/
	
	uint16_t crc16;
}CMD_START_UPLOAD_FIXTURE;
#pragma pack()


/*按照1字节对齐，便于存储到uint8_t类型buf*/
#pragma pack(1)
typedef struct
{
	uint16_t head;
	
	uint16_t frame_len;
	uint16_t frame_len_reverse;
	
    uint16_t tx_addr;
    uint16_t rx_addr;
	
    uint16_t main_sec ;
    uint16_t sub_sec ;
	
	/*********通用数据部分开始***********/
    
	uint32_t paraNormal;
    
	/*********通用数据部分结束***********/
	
	uint16_t crc16;
}CommunicationNormal;
#pragma pack()

/*
**********************************************************************************************************
											变量声明
**********************************************************************************************************
*/
static uint8_t analysisBuf[MAX_ANALYSIS_LEN] = {0};    /*用于解析帧数据*/
static ReportFrameWithCRC16 reportFrameWithCRC16;

static CMD_START_UPLOAD cmd_START_UPLOAD;
static CMD_START_UPLOAD_FIXTURE cmd_START_UPLOAD_FIXTURE;
static CommunicationNormal communicationNormal;
/*
**********************************************************************************************************
											函数声明
**********************************************************************************************************
*/


uint8_t GetCmdStartUpload(void)
{
	return cmd_START_UPLOAD.isOpen;
}


void bsp_ExexCmd(uint8_t *cmd , uint16_t main_sec , uint16_t sub_sec)
{
    if(main_sec == 2 && sub_sec == 1) /*PC机命令主机上报所有数据*/
	{
		memcpy(&cmd_START_UPLOAD,cmd,sizeof(cmd_START_UPLOAD));
	}
	else if(main_sec == 2 && sub_sec == 2) /*PC机命令治具主板开启测试并上报数据*/
	{
		memcpy(&cmd_START_UPLOAD_FIXTURE,cmd,sizeof(cmd_START_UPLOAD_FIXTURE));
		if(cmd_START_UPLOAD_FIXTURE.isOpen)
		{
			bsp_StartSelfCheck();
		}
	}
	else if(main_sec == 2 && sub_sec == 4) /*PC机命令命令主机执行测试床程序*/
	{
		memcpy(&communicationNormal,cmd,sizeof(communicationNormal));
		if(communicationNormal.paraNormal)
		{
			bsp_StartFunctionTest();
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
                RTT("crc success(%d-%d):%04X\r\n",main_sec,sub_sec,crc_ret);
                bsp_ExexCmd(analysisBuf,main_sec,sub_sec);
            }
            else
            {
                RTT("crc faile(%d-%d):%04X\r\n",main_sec,sub_sec,crc_ret);
            }
            index = 0 ;
        }
    }
}




void bsp_SendReportFrameWithCRC16(void)
{
	/*电压部分*/
	float batteryVoltage = bsp_GetFeedbackVoltage(eBatteryVoltage);
	float batteryCurrent = bsp_GetFeedbackVoltage(eBatteryCurrent);
	float wheelL = bsp_GetFeedbackVoltage(eMotorLeft);
	float wheelR = bsp_GetFeedbackVoltage(eMotorRight);
	float roll = bsp_GetFeedbackVoltage(eRollingBrush);
	float vacuum = bsp_GetFeedbackVoltage(eVacuum);
	float sideBrush = bsp_GetFeedbackVoltage(eSideBrush);

	reportFrameWithCRC16.dustBox = bsp_DustBoxGetState();
	
	reportFrameWithCRC16.wheelSpeedL = bsp_MotorGetSpeed(MotorLeft);
	reportFrameWithCRC16.wheelSpeedR = bsp_MotorGetSpeed(MotorRight);

	reportFrameWithCRC16.wheelPulseL = bsp_MotorGetPulseVector(MotorLeft);
	reportFrameWithCRC16.wheelPulseR = bsp_MotorGetPulseVector(MotorRight);

	reportFrameWithCRC16.x_pos = bsp_GetCurrentPosX();
	reportFrameWithCRC16.y_pos = bsp_GetCurrentPosY();

	reportFrameWithCRC16.cliffMV_L = bsp_GetCliffRealVal(CliffLeft); 
	reportFrameWithCRC16.cliffMV_M = bsp_GetCliffRealVal(CliffMiddle); 
	reportFrameWithCRC16.cliffMV_R = bsp_GetCliffRealVal(CliffRight); 

	reportFrameWithCRC16.yaw = bsp_AngleReadRaw(); 

	reportFrameWithCRC16.irMV[0] = bsp_GetInfraRedAdcVoltage(IR0); 
	reportFrameWithCRC16.irMV[1] = bsp_GetInfraRedAdcVoltage(IR1); 
	reportFrameWithCRC16.irMV[2] = bsp_GetInfraRedAdcVoltage(IR2); 
	reportFrameWithCRC16.irMV[3] = bsp_GetInfraRedAdcVoltage(IR3); 
	reportFrameWithCRC16.irMV[4] = bsp_GetInfraRedAdcVoltage(IR4); 
	reportFrameWithCRC16.irMV[5] = bsp_GetInfraRedAdcVoltage(IR5); 
	reportFrameWithCRC16.irMV[6] = bsp_GetInfraRedAdcVoltage(IR6); 
	reportFrameWithCRC16.irMV[7] = bsp_GetInfraRedAdcVoltage(IR7); 
	reportFrameWithCRC16.irMV[8] = bsp_GetInfraRedAdcVoltage(IR8); 
	reportFrameWithCRC16.irMV[9] = bsp_GetInfraRedAdcVoltage(IR9); 

	reportFrameWithCRC16.irRX[0][0] = bsp_IR_GetRev(IR_CH1,IR_TX_SITE_LEFT); 
	reportFrameWithCRC16.irRX[0][1] = bsp_IR_GetRev(IR_CH1,IR_TX_SITE_CENTER); 
	reportFrameWithCRC16.irRX[0][2] = bsp_IR_GetRev(IR_CH1,IR_TX_SITE_RIGHT); 

	reportFrameWithCRC16.irRX[1][0] = bsp_IR_GetRev(IR_CH2,IR_TX_SITE_LEFT); 
	reportFrameWithCRC16.irRX[1][1] = bsp_IR_GetRev(IR_CH2,IR_TX_SITE_CENTER); 
	reportFrameWithCRC16.irRX[1][2] = bsp_IR_GetRev(IR_CH2,IR_TX_SITE_RIGHT);

	reportFrameWithCRC16.irRX[2][0] = bsp_IR_GetRev(IR_CH3,IR_TX_SITE_LEFT); 
	reportFrameWithCRC16.irRX[2][1] = bsp_IR_GetRev(IR_CH3,IR_TX_SITE_CENTER); 
	reportFrameWithCRC16.irRX[2][2] = bsp_IR_GetRev(IR_CH3,IR_TX_SITE_RIGHT);

	reportFrameWithCRC16.irRX[3][0] = bsp_IR_GetRev(IR_CH4,IR_TX_SITE_LEFT); 
	reportFrameWithCRC16.irRX[3][1] = bsp_IR_GetRev(IR_CH4,IR_TX_SITE_CENTER); 
	reportFrameWithCRC16.irRX[3][2] = bsp_IR_GetRev(IR_CH4,IR_TX_SITE_RIGHT);

	reportFrameWithCRC16.offsiteSW = bsp_OffSiteGetState();
	reportFrameWithCRC16.collision = bsp_CollisionScan();

	reportFrameWithCRC16.mA_wheelL           = wheelL * 1000.0F * 1000.0F / 33.0F / 50.0F;
	reportFrameWithCRC16.mA_wheelR           = wheelR * 1000.0F * 1000.0F / 33.0F / 50.0F;
	reportFrameWithCRC16.mA_roll             = roll * 1000.0F * 1000.0F / 33.0F / 50.0F;
	reportFrameWithCRC16.mA_sideBrush        = sideBrush * 1000.0F * 1000.0F / 100.0F / 50.0F;
	reportFrameWithCRC16.mA_vacuum           = vacuum * 1000.0F * 1000.0F / 33.0F / 50.0F;
	reportFrameWithCRC16.v_batteryVoltage    = ((batteryVoltage * 430 / 66.5) + batteryVoltage + 0.2F)*1000; 
	reportFrameWithCRC16.mA_batteryCurrent   = batteryCurrent*1000.0F * 1000.0F / 10.0F / 50.0F; 


	reportFrameWithCRC16.head = 0xAAAA;
	reportFrameWithCRC16.frame_len = sizeof(ReportFrameWithCRC16) & 0xFFFF;
	reportFrameWithCRC16.frame_len_reverse = (~reportFrameWithCRC16.frame_len) & 0xFFFF;
	
	reportFrameWithCRC16.tx_addr = 0;
	reportFrameWithCRC16.rx_addr = 0;
	
	reportFrameWithCRC16.main_sec = 0;
	reportFrameWithCRC16.sub_sec = 0;
	
	uint16_t ret = CRC16_Modbus((uint8_t*)&reportFrameWithCRC16,sizeof(ReportFrameWithCRC16)-2);
	reportFrameWithCRC16.crc16 = ((ret>>8)&0x00FF)  | ((ret<<8)&0xFF00);
	
	comSendBuf(COM2,(uint8_t*)&reportFrameWithCRC16,sizeof(ReportFrameWithCRC16));
}

