#include "bsp.h"

/*
**********************************************************************************************************
											��������
**********************************************************************************************************
*/
static uint8_t sendBuf[256] = {0};                     /*���ڴ洢֡��ʹ��ȫ�ֱ�������ֹƵ������ջ�ռ�*/
static uint8_t analysisBuf[MAX_ANALYSIS_LEN] = {0};    /*���ڽ���֡����*/
static RouteAnalysis routeAnalysis;
static ReportFrame reportFrame;

/*
**********************************************************************************************************
											��������
**********************************************************************************************************
*/
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
			routeAnalysis.msgID = ch;
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
				if(routeAnalysis.msgID == CMD_ID_SPEED)
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
				
				/*���¿�ʼ����*/
				index = 0 ;
			}
		}
	}
	
	
	/*ѡ������*/
	port = COM1;
	
	/*ѡ������*/
	port = COM2;
	
	/*ѡ������*/
	port = COM3;
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
		sendBuf[i] = src[i];
	}
	
	comSendBuf(COM4,sendBuf,len);
	
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
	uint16_t chk = 0 ;
	uint32_t len = sizeof(reportFrame);  /*֡��С*/
	uint8_t* src = (uint8_t*)&reportFrame;
	
	int16_t angle = bsp_AngleReadRaw();  /*�Ƕ�*/ 
	uint8_t dataLen = sizeof(reportFrame) - 6;
	int16_t leftSpeedMM = bsp_MotorGetSpeed(MotorLeft);
	int16_t rightSpeedMM = bsp_MotorGetSpeed(MotorRight);
	uint16_t adc1  = bsp_GetInfraRedAdcVoltage(IR0)*100;
	uint16_t adc2  = bsp_GetInfraRedAdcVoltage(IR1)*100;
	uint16_t adc3  = bsp_GetInfraRedAdcVoltage(IR2)*100;
	uint16_t adc4  = bsp_GetInfraRedAdcVoltage(IR3)*100;
	uint16_t adc5  = bsp_GetInfraRedAdcVoltage(IR4)*100;
	uint16_t adc6  = bsp_GetInfraRedAdcVoltage(IR5)*100;
	uint16_t adc7  = bsp_GetInfraRedAdcVoltage(IR6)*100;
	uint16_t adc8  = bsp_GetInfraRedAdcVoltage(IR7)*100;
	uint16_t adc9  = bsp_GetInfraRedAdcVoltage(IR8)*100;
	uint16_t adc10 = bsp_GetInfraRedAdcVoltage(IR9)*100;
	uint32_t timestamp = xTaskGetTickCount();
	int32_t leftPulseVector = bsp_MotorGetPulseVector(MotorLeft);
	int32_t rightPulseVector = bsp_MotorGetPulseVector(MotorRight);
	int32_t x_coordinate = bsp_GetCurrentPosX();
	int32_t y_coordinate = bsp_GetCurrentPosY();
	
	
	
	
	reportFrame.sof1 = 0xAA;                  		                                  //0xAA
	reportFrame.sof2 = 0xAA;                  		                                  //0xAA
	reportFrame.sof3 = 0xAA;                  		                                  //0xAA
	reportFrame.identifier_major = MIN_ID_ENVIRONMENT;                                //��ʶ����
	reportFrame.identifier_subs = 0x00;        	                                      //��ʶ����
	reportFrame.size_of_payload_field = dataLen;   	                                  //���ݳ��ȣ�������ͷ��ʶ3��β��ʶ1��У��2��
	reportFrame.left_wheel_pulse_count = leftPulseVector;   //���ֱ���������
	reportFrame.right_wheel_pluse_count = rightPulseVector; //���ֱ���������
	reportFrame.left_wheel_velocity = leftSpeedMM;  		  //���ֵ���ٶ�
	reportFrame.right_wheel_veloctiy = rightSpeedMM;		  //���ֵ���ٶ�
	reportFrame.x_coordinate = x_coordinate;				  //X����
	reportFrame.y_coordinate = y_coordinate;				  //Y����
	reportFrame.theta_angle_deg = angle;		              //�����
	reportFrame.landoff_button = 0;                                                   //��ؿ���
	reportFrame.collosion_button = bsp_CollisionScan() ; 			                  //��ײ����
	reportFrame.infrared_front_status = 0; 	                                          //ǰ�����״̬ 
	reportFrame.infrared_edge_status = 0;	                                          //�رߺ���״̬
	reportFrame.infrared_adc_value1 =  adc1;                //����ADCֵ1	 
	reportFrame.infrared_adc_value2 =  adc2;                //����ADCֵ2	 
	reportFrame.infrared_adc_value3 =  adc3;                //����ADCֵ3	 
	reportFrame.infrared_adc_value4 =  adc4;                //����ADCֵ4	 
	reportFrame.infrared_adc_value5 =  adc5;                //����ADCֵ5	 
	reportFrame.infrared_adc_value6 =  adc6;                //����ADCֵ6	 
	reportFrame.infrared_adc_value7 =  adc7;                //����ADCֵ7	 
	reportFrame.infrared_adc_value8 =  adc8;                //����ADCֵ8	 
	reportFrame.infrared_adc_value9 =  adc9;                //����ADCֵ9	 
	reportFrame.infrared_adc_value10 = adc10;               //����ADCֵ10
	reportFrame.infrared_cliff_status = 0 ;                                           //���º���״̬
	reportFrame.infrared_cliff_adc_value1 = 0 ;                                       //����ADCֵ1
	reportFrame.infrared_cliff_adc_value2 = 0 ;                                       //����ADCֵ2
	reportFrame.infrared_cliff_adc_value3 = 0 ;                                       //����ADCֵ3
	reportFrame.battery_voltage = 0;                                                  //��ص�ѹ
	reportFrame.charging_status = 0;                                                  //���״̬
	reportFrame.error_code = 0;         	                                          //�쳣״̬
	reportFrame.machine_status = 0;                                                   //����״̬
	reportFrame.timestamp = timestamp;                      //ʱ���
	reportFrame.reserved1 = 0;  	                                                  //����λ1
	reportFrame.reserved2 = 0;				                                          //����λ2
	reportFrame.reserved3 = 0;         	                                              //����λ3
	reportFrame.checksum_msb = 0;                                                     //У��
	reportFrame.checksum_lsb = 0;                                                     //У��
	reportFrame.end_of_falg = 0x55;                                                   //֡��β�㶨Ϊ0x55
	
	/*����У��*/
	chk = bsp_CalcChk(src+3,len-6);
	reportFrame.checksum_msb = chk >> 8;
	reportFrame.checksum_lsb = chk & 0x00FF;
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




