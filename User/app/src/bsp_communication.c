#include "bsp.h"

#define  MIN_ID_ENVIRONMENT 	0x30	// 环境
#define  MIN_ID_MOTION  		0x25	// 运动
#define  MIN_ID_POSE  			0x20	// 姿势
#define  MIN_ID_MAP 			0x40	// 地图


uint8_t  buf[256] = {0};       /*用于存储帧，使用全局变量，防止频繁开辟栈空间*/



static ReportFrame reportFrame;
static uint16_t bsp_CalcChk(uint8_t *buf, uint8_t len);
static void bsp_FillReportFrame(void);



/*
*********************************************************************************************************
*	函 数 名: bsp_SendReportFrame
*	功能说明: 发送数据帧
*	形    参: 无
*	返 回 值: 无
*********************************************************************************************************
*/
void bsp_SendReportFrame(void)
{
	
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
	int32_t odometerL = 0;//bsp_encoderGetOdometer(MotorLeft);  /*里程计 左*/
	int32_t odometerR = 0;//bsp_encoderGetOdometer(MotorRight); /*里程计 右*/
	
	/*消除编译器警告*/
	UNUSED(reportFrame);
	
	/*大小端转换*/
	reportFrame.sof1 = 0xAA;                     //恒定为0xAA
	reportFrame.sof2 = 0xAA;                     //恒定为0xAA
	reportFrame.sof3 = 0xAA;                     //恒定为0xAA
	reportFrame.identifier = MIN_ID_ENVIRONMENT; //恒定为0x25
	reportFrame.size_of_payload_field = 0x1B+8; 
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
	chk = bsp_CalcChk(src+3,len-6);
	reportFrame.checksum_msb = chk >> 8;
	reportFrame.checksum_lsb = chk & 0x00FF;
}


/*
*********************************************************************************************************
*	函 数 名: bsp_CalcChk
*	功能说明: 计算校验值
*	形    参: 无
*	返 回 值: 无
*********************************************************************************************************
*/
static uint16_t bsp_CalcChk(uint8_t *buf, uint8_t len)
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


