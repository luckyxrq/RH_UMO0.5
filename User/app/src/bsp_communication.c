#include "bsp.h"

#pragma pack(1)
typedef struct _s_min_id_environment
{
	uint8_t sof1;                  //0xAA
	uint8_t sof2;                  //0xAA
	uint8_t sof3;                  //0xAA
	uint8_t identifier;	           //0x25
	uint8_t size_of_payload_field; //0x04
	int32_t left_wheel_pulse_count;
	int32_t right_wheel_pulse_count;
	uint8_t button_control_cmd;
	uint16_t distance_of_left_infrared;
	uint16_t distance_of_right_infrared;
	uint16_t distance_of_front_infrared;
	int16_t  angle_deg;
	uint16_t adc_1;
	uint16_t adc_2;
	uint16_t adc_3;
	uint16_t adc_4;
	int16_t acc_x;
	int16_t acc_y;
	int16_t acc_z;
	uint16_t obstacle_signal;
	uint8_t battery_level_soc;
	uint8_t charge_status;
	uint8_t checksum_msb;
	uint8_t checksum_lsb;
	uint8_t end_of_falg;           //0x55
}ReportFrame;


static ReportFrame reportFrame;
static uint16_t bsp_CalcChk(uint8_t *buf, uint8_t len);
static void bsp_FillReportFrame(void);




void bsp_SendReportFrame(void)
{
	uint8_t  buf[256] = {0};       /*用于存储帧*/
	uint32_t len = sizeof(reportFrame);/*帧大小*/
	uint8_t* src = (uint8_t*)&reportFrame;
	uint32_t i = 0 ;
	
	/*填充数据*/
	bsp_FillReportFrame();
	
	/*填充帧*/
	for(i=0;i<len;i++)
	{
		buf[i] = src[i];
	}
	
	comSendBuf(COM4,buf,len);
	
}


/*
*********************************************************************************************************
*	函 数 名: bsp_FillReportFrame
*	功能说明: 填充上报数据帧
*	形    参: 无
*	返 回 值: 无
*********************************************************************************************************
*/
void bsp_FillReportFrame(void)
{
	uint16_t chk = 0 ;
	uint32_t len = sizeof(reportFrame);/*帧大小*/
	uint8_t* src = (uint8_t*)&reportFrame;
	int16_t angle = bsp_AngleReadRaw();                     /*角度*/
	int32_t odometerL = bsp_encoderGetOdometer(MotorLeft);  /*里程计 左*/
	int32_t odometerR = bsp_encoderGetOdometer(MotorRight); /*里程计 右*/
	/*消除编译器警告*/
	UNUSED(reportFrame);
	
	/*大小端转换*/
	reportFrame.sof1 = 0xAA;                   //恒定为0xAA
	reportFrame.sof2 = 0xAA;                   //恒定为0xAA
	reportFrame.sof3 = 0xAA;                   //恒定为0xAA
	reportFrame.identifier = 0x25;             //恒定为0x25
	reportFrame.size_of_payload_field = 0x04;  //恒定为0x04
	reportFrame.left_wheel_pulse_count =  BEBufToUint32((uint8_t*)&odometerL);
	reportFrame.right_wheel_pulse_count = BEBufToUint32((uint8_t*)&odometerR);
	reportFrame.button_control_cmd = 0 ;
	reportFrame.distance_of_left_infrared = 0 ;
	reportFrame.distance_of_right_infrared = 0 ;
	reportFrame.distance_of_front_infrared = 0 ;
	reportFrame.angle_deg = BEBufToUint16((uint8_t*)&angle);
	reportFrame.adc_1 = 0 ;
	reportFrame.adc_2 = 0 ;
	reportFrame.adc_3 = 0 ;
	reportFrame.adc_4 = 0 ;
	reportFrame.acc_x = 0 ;
	reportFrame.acc_y = 0 ;
	reportFrame.acc_z = 0 ;
	reportFrame.obstacle_signal = 0 ;
	reportFrame.battery_level_soc = 0 ;
	reportFrame.charge_status = 0 ;
	reportFrame.checksum_msb = 0 ;
	reportFrame.checksum_lsb = 0 ;
	reportFrame.end_of_falg = 0x55 ;                  /*恒定0x55*/
	
	/*计算校验*/
	chk = bsp_CalcChk(src+3,len-7);
	reportFrame.checksum_msb = chk >> 8;
	reportFrame.checksum_lsb = chk & 0x00FF;
}


uint16_t bsp_CalcChk(uint8_t *buf, uint8_t len)
{
	uint8_t  i;
	uint16_t rx_sum1=0x00FFu;
	uint16_t rx_sum2=0x00FFu;
	
	for(i=0; i<len; i++)
	{
		rx_sum2 += rx_sum1 += buf[i];
	}
	
	rx_sum1 = (rx_sum1&0x00FFu) + (rx_sum1>>8);
	rx_sum2 = (rx_sum2&0x00FFu) + (rx_sum2>>8);
	
	return rx_sum2<<8|rx_sum1;
}


