#include "bsp.h"

#define  MIN_ID_ENVIRONMENT 	0x30	// ����
#define  MIN_ID_MOTION  		0x25	// �˶�
#define  MIN_ID_POSE  			0x20	// ����
#define  MIN_ID_MAP 			0x40	// ��ͼ


#define  CMD_ID_SPEED 	        0x25
#define  CMD_ID_DISTANCE  		0x35
#define  CMD_ID_ANGLE  			0x45

#define WHEEL_LENGTH  			0.23F
#define M_PI 					3.14F
#define MAX_ANALYSIS_LEN	    512


#define Deg2Rad(deg) (M_PI * deg / 180.0F)
#define Rad2Deg(rad) (180.0F * rad / M_PI)

typedef struct
{
	int16_t* leftVelocity;              /*�����ٶ�*/
	int16_t* rightVelocity;             /*�����ٶ�*/
	uint16_t len;                       /*���������ݸ���*/
}RouteAnalysis;





uint8_t  buf[256] = {0};                              /*���ڴ洢֡��ʹ��ȫ�ֱ�������ֹƵ������ջ�ռ�*/
static uint8_t analysisBuf[MAX_ANALYSIS_LEN] = {0}; /*���ڽ���֡����*/
static RouteAnalysis routeAnalysis;

static ReportFrame reportFrame;
static uint16_t bsp_CalcChk(uint8_t *buf, uint8_t len);
static void bsp_FillReportFrame(void);




/*
*********************************************************************************************************
*	�� �� ��: bsp_ComAnalysis
*	����˵��: �������ݽ���
*	��    ��: ��
*	�� �� ֵ: ��
*********************************************************************************************************
*/
void bsp_ComAnalysis(void)
{
	COM_PORT_E port ; 
	uint8_t ch = 0 ;
	uint16_t index = 0 ;
	
	/*ѡ������*/
	port = COM4;
	
	while(comGetChar(port, &ch))
	{
		/*���λ�ȡ���ڻ�����ÿ���ֽ�*/
		analysisBuf[index % MAX_ANALYSIS_LEN] = ch ;
		index++;
		
		if(index == 1)    /*ǰ��3����֡ͷ*/
		{
			if(ch != 0xAA)
			{
				index = 0 ;
			}
		}
		else if(index == 2)/*ǰ��3����֡ͷ*/
		{
			if(ch != 0xAA)
			{
				index = 0 ;
			}
		}
		else if(index == 3)/*ǰ��3����֡ͷ*/
		{
			if(ch != 0xAA)
			{
				index = 0 ;
			}
		}
		else if(index == 4)/*��ϢID*/
		{
			if(ch != CMD_ID_SPEED && ch != CMD_ID_DISTANCE && ch != CMD_ID_ANGLE)
			{
				index = 0 ;
			}
		}
		else if(index == 5)/*֡����*/
		{
			routeAnalysis.len = ch;
		}
		else if(index >= (routeAnalysis.len+ 8))/*���ݽ�����ϣ�֡ͷ֡β4������2��ID 1������1��*/
		{
			uint16_t calcChk = bsp_CalcChk(analysisBuf+3,index-6); /*����У�����ݣ�����У������ݲ�����ǰ���3��֡ͷ�ͺ����1��֡β��2��У���ֽ�*/
			uint16_t rxChk   = analysisBuf[index-1-2] << 8 | analysisBuf[index-1-1];
			
			if(analysisBuf[index-1] != 0x55 || calcChk != rxChk)
			{
				index = 0 ;
			}
			else /*�������ȷ�Ľ�������*/
			{
				int16_t linearVelocity  = analysisBuf[5]<<8 | analysisBuf[6];
				int16_t angularVelocity = analysisBuf[7]<<8 | analysisBuf[8];
				
				/*������ٶȣ���λMM/S */
				int16_t leftVelocity = (int16_t)((0.5*(2*linearVelocity*0.001 - Deg2Rad(angularVelocity)*WHEEL_LENGTH))* 1000);
				int16_t rightVelocity = (int16_t)((0.5*(2*linearVelocity*0.001 + Deg2Rad(angularVelocity)*WHEEL_LENGTH))* 1000);
				
				/*�趨�ٶ�*/
				bsp_SetMotorSpeed(MotorLeft,bsp_MotorSpeedMM2Pulse(leftVelocity));
				bsp_SetMotorSpeed(MotorRight,bsp_MotorSpeedMM2Pulse(rightVelocity));
			}
		}
	}
}





/*
*********************************************************************************************************
*	�� �� ��: bsp_SendReportFrame
*	����˵��: ��������֡
*	��    ��: ��
*	�� �� ֵ: ��
*********************************************************************************************************
*/
void bsp_SendReportFrame(void)
{
	
	uint32_t len = sizeof(reportFrame);/*֡��С*/
	uint8_t* src = (uint8_t*)&reportFrame;
	uint32_t i = 0 ;
	
	/*�������*/
	bsp_FillReportFrame();
	
	/*���֡*/
	for(i=0;i<len;i++)
	{
		buf[i] = src[i];
	}
	
	comSendBuf(COM4,buf,len);
	
}


/*
*********************************************************************************************************
*	�� �� ��: bsp_FillReportFrame
*	����˵��: ����ϱ�����֡
*	��    ��: ��
*	�� �� ֵ: ��
*********************************************************************************************************
*/
void bsp_FillReportFrame(void)
{
//	uint16_t chk = 0 ;
//	uint32_t len = sizeof(reportFrame);/*֡��С*/
//	uint8_t* src = (uint8_t*)&reportFrame;
//	int16_t angle = bsp_AngleReadRaw();                     /*�Ƕ�*/
//	int32_t odometerL = 0;//bsp_encoderGetOdometer(MotorLeft);  /*��̼� ��*/
//	int32_t odometerR = 0;//bsp_encoderGetOdometer(MotorRight); /*��̼� ��*/
//	
//	/*��������������*/
//	UNUSED(reportFrame);
//	
//	/*��С��ת��*/
//	reportFrame.sof1 = 0xAA;                     //�㶨Ϊ0xAA
//	reportFrame.sof2 = 0xAA;                     //�㶨Ϊ0xAA
//	reportFrame.sof3 = 0xAA;                     //�㶨Ϊ0xAA
//	reportFrame.identifier = MIN_ID_ENVIRONMENT; //�㶨Ϊ0x25
//	reportFrame.size_of_payload_field = 0x1B+8; 
//	reportFrame.left_wheel_pulse_count =  BEBufToUint32((uint8_t*)&odometerL);
//	reportFrame.right_wheel_pulse_count = BEBufToUint32((uint8_t*)&odometerR);
//	reportFrame.button_control_cmd = 0 ;
//	reportFrame.distance_of_left_infrared = 0 ;
//	reportFrame.distance_of_right_infrared = 0 ;
//	reportFrame.distance_of_front_infrared = 0 ;
//	reportFrame.angle_deg = BEBufToUint16((uint8_t*)&angle);
//	reportFrame.adc_1 = 0 ;
//	reportFrame.adc_2 = 0 ;
//	reportFrame.adc_3 = 0 ;
//	reportFrame.adc_4 = 0 ;
//	reportFrame.acc_x = 0 ;
//	reportFrame.acc_y = 0 ;
//	reportFrame.acc_z = 0 ;
//	reportFrame.obstacle_signal = 0 ;
//	reportFrame.battery_level_soc = 0 ;
//	reportFrame.charge_status = 0 ;
//	reportFrame.checksum_msb = 0 ;
//	reportFrame.checksum_lsb = 0 ;
//	reportFrame.end_of_falg = 0x55 ;                  /*�㶨0x55*/
//	
//	/*����У��*/
//	chk = bsp_CalcChk(src+3,len-6);
//	reportFrame.checksum_msb = chk >> 8;
//	reportFrame.checksum_lsb = chk & 0x00FF;
}


/*
*********************************************************************************************************
*	�� �� ��: bsp_CalcChk
*	����˵��: ����У��ֵ
*	��    ��: ��
*	�� �� ֵ: ��
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




