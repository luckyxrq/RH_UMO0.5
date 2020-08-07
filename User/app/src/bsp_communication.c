#include "bsp.h"

#define MAX_ANALYSIS_LEN	    512                 /*һ֡���ݵ���󳤶�*/
#define MIN_ANALYSIS_LEN        16                  /*һ֡���ݵ���С����*/





/*
**********************************************************************************************************
											��������
**********************************************************************************************************
*/
static uint8_t analysisBuf[MAX_ANALYSIS_LEN] = {0};    /*���ڽ���֡����*/
static MCU_FRAME mcu_frame;

/*
**********************************************************************************************************
											��������
**********************************************************************************************************
*/


uint8_t GetCmdStartUpload(void)
{
	return 0;
}


void bsp_ExexCmd(uint8_t *cmd , uint16_t main_sec , uint16_t sub_sec)
{
    if(main_sec == 2 && sub_sec == 1) /*PC�����������ϱ���������*/
	{
		
	}
	else if(main_sec == 2 && sub_sec == 2) /*PC�������ξ����忪�����Բ��ϱ�����*/
	{

	}
	else if(main_sec == 2 && sub_sec == 4) /*PC��������������ִ�в��Դ�����*/
	{

	}
}

void bsp_ComAnalysis(void)
{
	uint8_t ch = 0;
	COM_PORT_E port ; 
    /*����������֮ǰ����static ����ѧ*/
    static uint16_t index = 0 ;
    
    /*֡�ṹ*/
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
	
	/*ѡ������*/
	port = COM2;
	
    while(comGetChar( port , &ch))   
    {
        //qDebug("%02X ",ch);
        
        analysisBuf[index % MAX_ANALYSIS_LEN] = ch ;
        index++;
        
        if(index == 1)       /*ǰ��2����֡ͷ*/
        {
            if(ch != 0xAA)
            {
                index = 0 ;
            }
        }
        else if(index == 2)  /*ǰ��2����֡ͷ*/
        {
            if(ch != 0xAA)
            {
                index = 0 ;
            }
        }
        else if(index == 4)  /*֡����*/
        {
            frame_len = ((analysisBuf[3] << 8) | analysisBuf[2]) & 0xFFFF;
            
            if(frame_len < MIN_ANALYSIS_LEN || frame_len > MAX_ANALYSIS_LEN)
            {
                DEBUG("err:frame_len %d",frame_len);
                index = 0 ;
            }
            
        }
        else if(index == 6)  /*֡���� ȡ��*/
        {
            uint16_t reverse = ((analysisBuf[5] << 8) | analysisBuf[4]) & 0xFFFF;
            
            /*���� &0x00FF �ǳ��б�Ҫ������*/
            if( ((~reverse)&0xFFFF) != frame_len )
            {
                DEBUG("err:reverse %04X",frame_len);
                index = 0 ;
            }
            
        }
        else if(index == 8)  /*���ͷ���ַ*/
        {
            tx_addr = ((analysisBuf[7] << 8) | analysisBuf[6]) & 0xFFFF;
            
        }
        else if(index == 10)  /*���շ���ַ*/
        {
            rx_addr = ((analysisBuf[9] << 8) | analysisBuf[8]) & 0xFFFF;
            
        }
        else if(index == 12)  /*������*/
        {
            main_sec = ((analysisBuf[11] << 8) | analysisBuf[10]) & 0xFFFF;
            
        }
        else if(index == 14)  /*�ӹ���*/
        {
            sub_sec = ((analysisBuf[13] << 8) | analysisBuf[12]) & 0xFFFF;
            
        }
        else if(index >= MIN_ANALYSIS_LEN && index >= frame_len)  /*һ֡���ݽ�������ˣ������16��������*/
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
	mcu_frame.dustBox = bsp_DustBoxGetState();
	
	mcu_frame.wheelSpeedL = bsp_MotorGetSpeed(MotorLeft);
	mcu_frame.wheelSpeedR = bsp_MotorGetSpeed(MotorRight);

	mcu_frame.wheelPulseL = bsp_MotorGetPulseVector(MotorLeft);
	mcu_frame.wheelPulseR = bsp_MotorGetPulseVector(MotorRight);

	mcu_frame.x_pos = bsp_GetCurrentPosX();
	mcu_frame.y_pos = bsp_GetCurrentPosY();

	mcu_frame.cliffMV_L = bsp_GetCliffRealVal(CliffLeft); 
	mcu_frame.cliffMV_M = bsp_GetCliffRealVal(CliffMiddle); 
	mcu_frame.cliffMV_R = bsp_GetCliffRealVal(CliffRight); 

	mcu_frame.yaw = bsp_AngleReadRaw(); 

	mcu_frame.irMV[0] = bsp_GetInfraRedAdcVoltage(IR0); 
	mcu_frame.irMV[1] = bsp_GetInfraRedAdcVoltage(IR1); 
	mcu_frame.irMV[2] = bsp_GetInfraRedAdcVoltage(IR2); 
	mcu_frame.irMV[3] = bsp_GetInfraRedAdcVoltage(IR3); 
	mcu_frame.irMV[4] = bsp_GetInfraRedAdcVoltage(IR4); 
	mcu_frame.irMV[5] = bsp_GetInfraRedAdcVoltage(IR5); 
	mcu_frame.irMV[6] = bsp_GetInfraRedAdcVoltage(IR6); 
	mcu_frame.irMV[7] = bsp_GetInfraRedAdcVoltage(IR7); 
	mcu_frame.irMV[8] = bsp_GetInfraRedAdcVoltage(IR8); 
	mcu_frame.irMV[9] = bsp_GetInfraRedAdcVoltage(IR9); 

	mcu_frame.irRX[0][0] = bsp_IR_GetRev(IR_CH1,IR_TX_SITE_LEFT); 
	mcu_frame.irRX[0][1] = bsp_IR_GetRev(IR_CH1,IR_TX_SITE_CENTER); 
	mcu_frame.irRX[0][2] = bsp_IR_GetRev(IR_CH1,IR_TX_SITE_RIGHT); 

	mcu_frame.irRX[1][0] = bsp_IR_GetRev(IR_CH2,IR_TX_SITE_LEFT); 
	mcu_frame.irRX[1][1] = bsp_IR_GetRev(IR_CH2,IR_TX_SITE_CENTER); 
	mcu_frame.irRX[1][2] = bsp_IR_GetRev(IR_CH2,IR_TX_SITE_RIGHT);

	mcu_frame.irRX[2][0] = bsp_IR_GetRev(IR_CH3,IR_TX_SITE_LEFT); 
	mcu_frame.irRX[2][1] = bsp_IR_GetRev(IR_CH3,IR_TX_SITE_CENTER); 
	mcu_frame.irRX[2][2] = bsp_IR_GetRev(IR_CH3,IR_TX_SITE_RIGHT);

	mcu_frame.irRX[3][0] = bsp_IR_GetRev(IR_CH4,IR_TX_SITE_LEFT); 
	mcu_frame.irRX[3][1] = bsp_IR_GetRev(IR_CH4,IR_TX_SITE_CENTER); 
	mcu_frame.irRX[3][2] = bsp_IR_GetRev(IR_CH4,IR_TX_SITE_RIGHT);

	mcu_frame.offsiteSW = bsp_OffSiteGetState();
	mcu_frame.collision = bsp_CollisionScan();

	mcu_frame.mA_wheelL           = bsp_GetVoltageAfterFilter(eMotorLeft);
	mcu_frame.mA_wheelR           = bsp_GetVoltageAfterFilter(eMotorRight);
	mcu_frame.mA_roll             = bsp_GetVoltageAfterFilter(eRollingBrush);
	mcu_frame.mA_sideBrush        = bsp_GetVoltageAfterFilter(eSideBrush);
	mcu_frame.mA_vacuum           = bsp_GetVoltageAfterFilter(eVacuum);
	mcu_frame.v_batteryVoltage    = bsp_GetVoltageAfterFilter(eBatteryVoltage);
	mcu_frame.mA_batteryCurrent   = bsp_GetVoltageAfterFilter(eBatteryCurrent);


	mcu_frame.head = 0xAAAA;
	mcu_frame.frame_len = sizeof(MCU_FRAME) & 0xFFFF;
	mcu_frame.frame_len_reverse = (~mcu_frame.frame_len) & 0xFFFF;
	
	mcu_frame.tx_addr = 0;
	mcu_frame.rx_addr = 0;
	
	mcu_frame.main_sec = 0;
	mcu_frame.sub_sec = 0;
	
	uint16_t ret = CRC16_Modbus((uint8_t*)&mcu_frame,sizeof(MCU_FRAME)-2);
	mcu_frame.crc16 = ((ret>>8)&0x00FF)  | ((ret<<8)&0xFF00);
	
	comSendBuf(COM2,(uint8_t*)&mcu_frame,sizeof(MCU_FRAME));
}

