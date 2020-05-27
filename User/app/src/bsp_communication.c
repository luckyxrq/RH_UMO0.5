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



void bsp_SendCmdStartSelfCheck_ACK(void)
{
	bsp_AllSelfCheckSendFrame(0x02,0x01,2,1,0,0);
}



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
	uint16_t index = 0 ; /*֮ǰ����static ����ѧ*/
	
	static uint16_t frame_len = 0 ;
	static uint16_t tx_addr = 0 ;
	static uint16_t rx_addr = 0 ;
	static uint16_t main_sec = 0 ;
	static uint16_t sub_sec = 0 ;
	
	/*ѡ������*/
	port = COM4;
	
	while(comGetChar(port, &ch))
	{
		//DEBUG("%02X ",ch);
		//DEBUG("%d\r\n",index);
		/*���λ�ȡ���ڻ�����ÿ���ֽ�*/
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
			frame_len = (analysisBuf[2] << 8) | analysisBuf[3];
		}
		else if(index == 6)  /*֡���� ȡ��*/
		{
			uint16_t reverse = (analysisBuf[4] << 8) | analysisBuf[5];
			
			/*���� &0x00FF �ǳ��б�Ҫ������*/
			if( ((~reverse)&0x00FF) != frame_len )
			{
				index = 0 ;
			}
		}
		else if(index == 8)  /*���ͷ���ַ*/
		{
			tx_addr = (analysisBuf[6] << 8) | analysisBuf[7];
			
		}
		else if(index == 10)  /*���շ���ַ*/
		{
			tx_addr = (analysisBuf[8] << 8) | analysisBuf[9];
			
		}
		else if(index == 12)  /*������*/
		{
			main_sec = (analysisBuf[10] << 8) | analysisBuf[11];
			
		}
		else if(index == 14)  /*�ӹ���*/
		{
			sub_sec = (analysisBuf[12] << 8) | analysisBuf[13];
			
		}
		else if(index == frame_len)  /*һ֡���ݽ��������*/
		{
			if(CRC16_CALC(analysisBuf,frame_len) == 0)
			{
				DEBUG("У��ͨ�� %d %d \r\n",main_sec,sub_sec);
				if(main_sec == 2 && sub_sec == 1)
				{
					bsp_StartAllSelfCheck();
					bsp_SendCmdStartSelfCheck_ACK();
				}
			}
			else
			{
				//DEBUG("У��ʧ��\r\n");
			}
			index = 0 ;
		}

	}
	
	
	/*ѡ������*/
	port = COM1;
	
	/*ѡ������*/
	port = COM4;

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
	
	uint8_t IRC1 = 0,IRC2 = 0,IRC3 = 0;
	uint8_t charge1_status  = 0;
	uint8_t charge2_status  = 0;
	uint8_t charge3_status  = 0;
	uint8_t charge4_status  = 0;
	
	
	
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
	
	
	IRC1 = bsp_IR_GetRev(IR_CH1,IR_TX_SITE_LEFT)?1:0;
	IRC2 = bsp_IR_GetRev(IR_CH1,IR_TX_SITE_CENTER)?1:0;
	IRC3 = bsp_IR_GetRev(IR_CH1,IR_TX_SITE_RIGHT)?1:0;
	charge1_status  = (IRC1)<<3 | (IRC2)<<2 | (IRC3) ;
	
	IRC1 = bsp_IR_GetRev(IR_CH2,IR_TX_SITE_LEFT)?1:0;
	IRC2 = bsp_IR_GetRev(IR_CH2,IR_TX_SITE_CENTER)?1:0;
	IRC3 = bsp_IR_GetRev(IR_CH2,IR_TX_SITE_RIGHT)?1:0;
	charge2_status  =  (IRC1)<<3 | (IRC2)<<2 | (IRC3) ;
	
	IRC1 = bsp_IR_GetRev(IR_CH3,IR_TX_SITE_LEFT)?1:0;
	IRC2 = bsp_IR_GetRev(IR_CH3,IR_TX_SITE_CENTER)?1:0;
	IRC3 = bsp_IR_GetRev(IR_CH3,IR_TX_SITE_RIGHT)?1:0;
	charge3_status |= (IRC1)<<3 | (IRC2)<<2 | (IRC3) ;
	
	IRC1 = bsp_IR_GetRev(IR_CH4,IR_TX_SITE_LEFT)?1:0;
	IRC2 = bsp_IR_GetRev(IR_CH4,IR_TX_SITE_CENTER)?1:0;
	IRC3 = bsp_IR_GetRev(IR_CH4,IR_TX_SITE_RIGHT)?1:0;
	charge4_status |= (IRC1)<<3 | (IRC2)<<2 | (IRC3);
	
	
	
	
	uint8_t  cliffstatus = 1;
	if(bsp_CliffIsDangerous(CliffLeft)) cliffstatus+=2;
	if(bsp_CliffIsDangerous(CliffMiddle)) cliffstatus+=4;
	if(bsp_CliffIsDangerous(CliffRight)) cliffstatus+=8;
	uint16_t adcCliffLeft = bsp_GetCliffVoltage(CliffLeft)*100;
	uint16_t adcCliffMiddle = bsp_GetCliffVoltage(CliffMiddle)*100;
	uint16_t adcCliffRight = bsp_GetCliffVoltage(CliffRight)*100;
	
	uint16_t motorLeftVoltage = bsp_GetFeedbackVoltage(eMotorLeft)*100;
	uint16_t motorRightVoltage = bsp_GetFeedbackVoltage(eMotorRight)*100;
	uint16_t motorVacuumVoltage = bsp_GetFeedbackVoltage(eVacuum)*100;
	uint16_t motorRollingVoltage = bsp_GetFeedbackVoltage(eRollingBrush)*100;
	uint16_t motorSideVoltage = bsp_GetFeedbackVoltage(eSideBrush)*100;
	uint16_t batteryCurrent = bsp_GetFeedbackVoltage(eBatteryCurrent)*100;
	
	uint8_t batteryvoltage = bsp_GetFeedbackVoltage(eBatteryVoltage)*10;
	

 

	reportFrame.sof1 = 0xAA;                  		                                  //0xAA
	reportFrame.sof2 = 0xAA;                  		                                  //0xAA
	reportFrame.sof3 = 0xAA;                  		                                  //0xAA
	reportFrame.identifier_major = MIN_ID_ENVIRONMENT;                                //��ʶ����
	reportFrame.identifier_subs = 0x00;        	                                      //��ʶ����
	reportFrame.size_of_payload_field = dataLen;   	                                  //���ݳ��ȣ�������ͷ��ʶ3��β��ʶ1��У��2��
	reportFrame.left_wheel_pulse_count = leftPulseVector;                             //���ֱ���������
	reportFrame.right_wheel_pluse_count = rightPulseVector;                           //���ֱ���������
	reportFrame.left_wheel_velocity = leftSpeedMM;  		                          //���ֵ���ٶ�
	reportFrame.right_wheel_veloctiy = rightSpeedMM;		                          //���ֵ���ٶ�
	reportFrame.x_coordinate = x_coordinate;				                          //X����
	reportFrame.y_coordinate = y_coordinate;				                          //Y����
	reportFrame.theta_angle_deg = angle;		                                      //�����
	reportFrame.landoff_button = bsp_OffSiteGetState() ;                              //��ؿ���
	reportFrame.collosion_button = bsp_CollisionScan() ; 			                  //��ײ����
	reportFrame.infrared_charge1_status = charge1_status; 	                  //�س����״̬ 
	reportFrame.infrared_charge1_status = charge2_status; 	                  //�س����״̬ 
	reportFrame.infrared_charge1_status = charge3_status; 	                  //�س����״̬ 
	reportFrame.infrared_charge1_status = charge4_status; 	                  //�س����״̬ 
	reportFrame.infrared_adc_value1 =  adc1;                                          //����ADCֵ1	 
	reportFrame.infrared_adc_value2 =  adc2;                                          //����ADCֵ2	 
	reportFrame.infrared_adc_value3 =  adc3;                                          //����ADCֵ3	 
	reportFrame.infrared_adc_value4 =  adc4;                                          //����ADCֵ4	 
	reportFrame.infrared_adc_value5 =  adc5;                                          //����ADCֵ5	 
	reportFrame.infrared_adc_value6 =  adc6;                                          //����ADCֵ6	 
	reportFrame.infrared_adc_value7 =  adc7;                                          //����ADCֵ7	 
	reportFrame.infrared_adc_value8 =  adc8;                                          //����ADCֵ8	 
	reportFrame.infrared_adc_value9 =  adc9;                                          //����ADCֵ9	 
	reportFrame.infrared_adc_value10 = adc10;                                         //����ADCֵ10
	//reportFrame.infrared_cliff_status = cliffstatus;                                  //���º���״̬
	reportFrame.infrared_cliff_adc_value1 = adcCliffLeft;                             //����ADCֵ1
	reportFrame.infrared_cliff_adc_value2 = adcCliffMiddle;                           //����ADCֵ2
	reportFrame.infrared_cliff_adc_value3 = adcCliffRight;                            //����ADCֵ3
	reportFrame.battery_voltage = batteryvoltage;                                     //��ص�ѹ
	reportFrame.dustbox_status = bsp_DustBoxGetState();                               //����״̬
	reportFrame.error_code = 0;         	                                          //�쳣״̬
	reportFrame.machine_status = 0;                                                   //����״̬
	reportFrame.timestamp = timestamp;                                                //ʱ���
	reportFrame.motor_left_voltage    = motorLeftVoltage;     
	reportFrame.motor_right_voltage   = motorRightVoltage;    
	reportFrame.motor_vacuum_voltage  = motorVacuumVoltage;   
	reportFrame.motor_rolling_voltage = motorRollingVoltage;  
	reportFrame.motor_side_voltage    = motorSideVoltage;     
	reportFrame.motor_battery_current = batteryCurrent;  
	reportFrame.checksum_msb = 0;                                                     //У��
	reportFrame.checksum_lsb = 0;                                                     //У��
	reportFrame.end_of_falg = 0x55;                                                   //֡��β�㶨Ϊ0x55
	
	/*����У��*/
	chk = bsp_CalcChk(src+3,len-6);
	reportFrame.checksum_msb = chk >> 8;
	reportFrame.checksum_lsb = chk & 0x00FF;
	
	
	/*��ӡ������Ϣ*/
	#if 1
	{
		float data[10] = {0};
		
		UNUSED(data);
		
		//DEBUG("angle:%.2F\r\n",angle/100.0F);
		//DEBUG("x_coordinate:%d  ",x_coordinate);
		//DEBUG("y_coordinate:%d\r\n",y_coordinate);
		
		data[0] =  angle;                                          //����ADCֵ1	 
		data[1] =  adc2;                                          //����ADCֵ2	 
		data[2] =  adc3;                                          //����ADCֵ3	 
		data[3] =  adc4;                                          //����ADCֵ4	 
		data[4] =  adc5;                                          //����ADCֵ5	 
		data[5] =  adc6;                                          //����ADCֵ6	 
		data[6] =  adc7;                                          //����ADCֵ7	 
		data[7] =  adc8;                                          //����ADCֵ8	 
		data[8] =  adc9;                                          //����ADCֵ9	 
		data[9] =  adc10;                                         //����ADCֵ10
		
		//bsp_ScopeSend(data,1);
	}
	#endif
	
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




