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



	reportFrame.sof1 = 0xAA;                  		                    //0xAA
	reportFrame.sof2 = 0xAA;                  		                    //0xAA
	reportFrame.sof3 = 0xAA;                  		                    //0xAA
	reportFrame.identifier_major = MIN_ID_ENVIRONMENT;                  //��ʶ����
	reportFrame.identifier_subs = 0x00;        	                        //��ʶ����
	reportFrame.size_of_payload_field = sizeof(reportFrame) - 6;   	    //���ݳ��ȣ�������ͷ��ʶ3��β��ʶ1��У��2��
	reportFrame.left_wheel_pulse_count = 0 ;  	                        //���ֱ���������
	reportFrame.right_wheel_pluse_count = 0; 	                        //���ֱ���������
	reportFrame.left_wheel_velocity = 0 ;  		                        //���ֵ���ٶ�
	reportFrame.right_wheel_veloctiy = 0 ;		                        //���ֵ���ٶ�
	reportFrame.x_coordinate = 0 ;				                        //X����
	reportFrame.y_coordinate = 0 ;				                        //Y����
	reportFrame.theta_angle_deg = 0;			                        //�����
	reportFrame.landoff_button = 0;                                     //��ؿ���
	reportFrame.collosion_button = 0 ; 			                        //��ײ����
	reportFrame.infrared_front_status = 0; 	                            //ǰ�����״̬ 
	reportFrame.infrared_edge_status = 0;	                            //�رߺ���״̬
	reportFrame.infrared_adc_value1 = 0;                                //����ADCֵ1	 
	reportFrame.infrared_adc_value2 = 0;                                //����ADCֵ2	 
	reportFrame.infrared_adc_value3 = 0;                                //����ADCֵ3	 
	reportFrame.infrared_adc_value4 = 0;                                //����ADCֵ4	 
	reportFrame.infrared_adc_value5 = 0;                                //����ADCֵ5	 
	reportFrame.infrared_adc_value6 = 0;                                //����ADCֵ6	 
	reportFrame.infrared_adc_value7 = 0;                                //����ADCֵ7	 
	reportFrame.infrared_adc_value8 = 0;                                //����ADCֵ8	 
	reportFrame.infrared_adc_value9 = 0;                                //����ADCֵ9	 
	reportFrame.infrared_adc_value10 = 0;                               //����ADCֵ10
	reportFrame.infrared_cliff_status = 0 ;                             //���º���״̬
	reportFrame.infrared_cliff_adc_value1 = 0 ;                         //����ADCֵ1
	reportFrame.infrared_cliff_adc_value2 = 0 ;                         //����ADCֵ2
	reportFrame.infrared_cliff_adc_value3 = 0 ;                         //����ADCֵ3
	reportFrame.battery_voltage = 0;                                    //��ص�ѹ
	reportFrame.charging_status = 0;                                    //���״̬
	reportFrame.error_code = 0;         	                            //�쳣״̬
	reportFrame.machine_status = 0;                                     //����״̬
	reportFrame.timestamp = 0;                                          //ʱ���
	reportFrame.reserved1 = 0;  	                                    //����λ1
	reportFrame.reserved2 = 0;				                            //����λ2
	reportFrame.reserved3 = 0;         	                                //����λ3
	reportFrame.checksum_msb = 0;
	reportFrame.checksum_lsb = 0;
	reportFrame.end_of_falg = 0;                                        //0x55
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




