#ifndef __BSP_COMMUNICATION_H
#define __BSP_COMMUNICATION_H




/*********************************************Э���ʽ�滻���֣���λ����λ��ͳһ��ֱ��Copy ��ʼ*************************************************/


#include <stdint.h>

/*����1�ֽڶ��룬���ڴ洢��uint8_t����buf*/
#pragma pack(1)
typedef struct
{
	/*********���ݲ��ֿ�ʼ***********/
	uint8_t dustBox;
	
	int16_t wheelSpeedL;
	int16_t wheelSpeedR;
	
	int32_t wheelPulseL;
	int32_t wheelPulseR;
	
	int32_t x_pos;
	int32_t y_pos;
	
	uint8_t offsiteSW;
	uint8_t collision;
	
	uint16_t cliffMV_L;
	uint16_t cliffMV_M;
	uint16_t cliffMV_R;
	
	int16_t yaw;         /*�Ƕ�ֵ*100����ȥ*/
	int16_t pitch;       /*�Ƕ�ֵ*100����ȥ*/
	int16_t roll;        /*�Ƕ�ֵ*100����ȥ*/
	
	uint16_t irMV[10];  /*һȦ����MV*/
	uint8_t  irRX[4][3];  /*�������*/

	
	uint16_t mA_wheelL;
	uint16_t mA_wheelR;
	uint16_t mA_roll;
	uint16_t mA_sideBrush;
	uint16_t mA_vacuum;
	uint16_t v_batteryVoltage;
	uint16_t mA_batteryCurrent;
	
	/*********���ݲ��ֽ���***********/

}MCU_FRAME;
#pragma pack()



/*����ʹ�������壬����һЩ�Զ���Ĳ���*/
typedef union
{
    uint8_t  sw;
    uint32_t val;
    uint32_t arr[16];
	uint32_t mcu_ver;
    MCU_FRAME mcu_frame; 
}UNION_PARA;


/*����1�ֽڶ��룬���ڴ洢��uint8_t����buf*/
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
	
	/*********���ݲ��ֿ�ʼ***********/
    UNION_PARA union_para;

	/*********���ݲ��ֽ���***********/
	
	uint16_t crc16;
}CMD_FRAME;
#pragma pack()

extern CMD_FRAME cmd_frame_tx;
extern CMD_FRAME cmd_frame_rx;


/*********************************************Э���ʽ�滻���֣���λ����λ��ͳһ��ֱ��Copy ����*************************************************/













#define WHEEL_LENGTH  			0.235F
#define M_PI 					3.14F
#define MAX_ANALYSIS_LEN	    512


#define Deg2Rad(deg) (M_PI * deg / 180.0F)
#define Rad2Deg(rad) (180.0F * rad / M_PI)


void bsp_ComAnalysis(void);     /*����֡*/
void bsp_SendReportFrameWithCRC16(void);

/*��Э��*/
uint8_t GetCmdStartUpload(void);

#endif
