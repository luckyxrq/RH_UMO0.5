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
					/*���ٶȷ�Χ��5~60 ��/��*/
					/*���ٶȷ�Χ��20~250 ����/��*/
					int16_t leftVelocity = (int16_t)((0.5*(2*linearVelocity*0.001 - Deg2Rad(angularVelocity)*WHEEL_LENGTH))* 1000);
					int16_t rightVelocity = (int16_t)((0.5*(2*linearVelocity*0.001 + Deg2Rad(angularVelocity)*WHEEL_LENGTH))* 1000);
					
					const int16_t limitSameDirSpeed = 250 ;
					const int16_t limitDiffrentDirSpeed = 150 ;
					
					if(leftVelocity > limitSameDirSpeed )
						leftVelocity = limitSameDirSpeed;
					
					if(rightVelocity > limitSameDirSpeed )
						rightVelocity = limitSameDirSpeed;
					
					if(leftVelocity < -limitSameDirSpeed )
						leftVelocity = -limitSameDirSpeed;
					
					if(rightVelocity < -limitSameDirSpeed )
						rightVelocity = -limitSameDirSpeed;
					
					if(leftVelocity * rightVelocity < 0)
					{
						if(leftVelocity > limitDiffrentDirSpeed )
						leftVelocity = limitDiffrentDirSpeed;
					
						if(rightVelocity > limitDiffrentDirSpeed )
							rightVelocity = limitDiffrentDirSpeed;
						
						if(leftVelocity < -limitDiffrentDirSpeed )
							leftVelocity = -limitDiffrentDirSpeed;
						
						if(rightVelocity < -limitDiffrentDirSpeed )
							rightVelocity = -limitDiffrentDirSpeed;
					}
					
					/*�趨�ٶ�*/
					bsp_SetMotorSpeed(MotorLeft,bsp_MotorSpeedMM2Pulse(leftVelocity));
					bsp_SetMotorSpeed(MotorRight,bsp_MotorSpeedMM2Pulse(rightVelocity));
					
					/*����*/
					#if 0
					DEBUG("routeAnalysis.msgID:%02X\r\n",routeAnalysis.msgID);
					DEBUG("routeAnalysis.len:%02X\r\n",routeAnalysis.len);

					DEBUG("linearVelocity:%02X %02X\r\n",analysisBuf[5], analysisBuf[6]);
					DEBUG("linearVelocity:%02X %02X\r\n",analysisBuf[7], analysisBuf[8]);
					#endif
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
	while(comGetChar(port, &ch))
	{
		//comSendChar(COM2, ch);
		//SEGGER_RTT_printf(0,"%02X ",ch);
	}
	/*ѡ������*/
	port = COM3;
}




/**************************************************ר���ϴ�����****************************************


	
	
****************************************************************************************************************/
static ReportFrameWithCRC16 reportFrameWithCRC16;

void bsp_SendReportFrameWithCRC16(void)
{
	/*��ѹ����*/
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




