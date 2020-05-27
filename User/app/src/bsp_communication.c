#include "bsp.h"

/*
**********************************************************************************************************
											变量声明
**********************************************************************************************************
*/
static uint8_t sendBuf[256] = {0};                     /*用于存储帧，使用全局变量，防止频繁开辟栈空间*/
static uint8_t analysisBuf[MAX_ANALYSIS_LEN] = {0};    /*用于解析帧数据*/
static RouteAnalysis routeAnalysis;
static ReportFrame reportFrame;

/*
**********************************************************************************************************
											函数声明
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
*	函 数 名: bsp_ComAnalysis
*	功能说明: 串口数据解析
*	形    参: 无
*	返 回 值: 无
*********************************************************************************************************
*/
void bsp_ComAnalysis(void)
{
	COM_PORT_E port ; 
	uint8_t ch = 0 ;
	uint16_t index = 0 ; /*之前不是static 不科学*/
	
	static uint16_t frame_len = 0 ;
	static uint16_t tx_addr = 0 ;
	static uint16_t rx_addr = 0 ;
	static uint16_t main_sec = 0 ;
	static uint16_t sub_sec = 0 ;
	
	/*选定串口*/
	port = COM4;
	
	while(comGetChar(port, &ch))
	{
		//DEBUG("%02X ",ch);
		//DEBUG("%d\r\n",index);
		/*依次获取串口缓冲区每个字节*/
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
			frame_len = (analysisBuf[2] << 8) | analysisBuf[3];
		}
		else if(index == 6)  /*帧长度 取反*/
		{
			uint16_t reverse = (analysisBuf[4] << 8) | analysisBuf[5];
			
			/*这里 &0x00FF 非常有必要！！！*/
			if( ((~reverse)&0x00FF) != frame_len )
			{
				index = 0 ;
			}
		}
		else if(index == 8)  /*发送方地址*/
		{
			tx_addr = (analysisBuf[6] << 8) | analysisBuf[7];
			
		}
		else if(index == 10)  /*接收方地址*/
		{
			tx_addr = (analysisBuf[8] << 8) | analysisBuf[9];
			
		}
		else if(index == 12)  /*主功能*/
		{
			main_sec = (analysisBuf[10] << 8) | analysisBuf[11];
			
		}
		else if(index == 14)  /*子功能*/
		{
			sub_sec = (analysisBuf[12] << 8) | analysisBuf[13];
			
		}
		else if(index == frame_len)  /*一帧数据接收完毕了*/
		{
			if(CRC16_CALC(analysisBuf,frame_len) == 0)
			{
				DEBUG("校验通过 %d %d \r\n",main_sec,sub_sec);
				if(main_sec == 2 && sub_sec == 1)
				{
					bsp_StartAllSelfCheck();
					bsp_SendCmdStartSelfCheck_ACK();
				}
			}
			else
			{
				//DEBUG("校验失败\r\n");
			}
			index = 0 ;
		}

	}
	
	
	/*选定串口*/
	port = COM1;
	
	/*选定串口*/
	port = COM4;

	/*选定串口*/
	port = COM3;
}



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
		sendBuf[i] = src[i];
	}
	
	comSendBuf(COM4,sendBuf,len);
	
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
	
	uint8_t IRC1 = 0,IRC2 = 0,IRC3 = 0;
	uint8_t charge1_status  = 0;
	uint8_t charge2_status  = 0;
	uint8_t charge3_status  = 0;
	uint8_t charge4_status  = 0;
	
	
	
	uint16_t chk = 0 ;
	uint32_t len = sizeof(reportFrame);  /*帧大小*/
	uint8_t* src = (uint8_t*)&reportFrame;
	
	int16_t angle = bsp_AngleReadRaw();  /*角度*/ 
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
	reportFrame.identifier_major = MIN_ID_ENVIRONMENT;                                //主识别码
	reportFrame.identifier_subs = 0x00;        	                                      //子识别码
	reportFrame.size_of_payload_field = dataLen;   	                                  //数据长度（不包括头标识3，尾标识1，校验2）
	reportFrame.left_wheel_pulse_count = leftPulseVector;                             //左轮编码器计数
	reportFrame.right_wheel_pluse_count = rightPulseVector;                           //右轮编码器计数
	reportFrame.left_wheel_velocity = leftSpeedMM;  		                          //左轮电机速度
	reportFrame.right_wheel_veloctiy = rightSpeedMM;		                          //右轮电机速度
	reportFrame.x_coordinate = x_coordinate;				                          //X坐标
	reportFrame.y_coordinate = y_coordinate;				                          //Y坐标
	reportFrame.theta_angle_deg = angle;		                                      //航向角
	reportFrame.landoff_button = bsp_OffSiteGetState() ;                              //离地开关
	reportFrame.collosion_button = bsp_CollisionScan() ; 			                  //碰撞开关
	reportFrame.infrared_charge1_status = charge1_status; 	                  //回充红外状态 
	reportFrame.infrared_charge1_status = charge2_status; 	                  //回充红外状态 
	reportFrame.infrared_charge1_status = charge3_status; 	                  //回充红外状态 
	reportFrame.infrared_charge1_status = charge4_status; 	                  //回充红外状态 
	reportFrame.infrared_adc_value1 =  adc1;                                          //红外ADC值1	 
	reportFrame.infrared_adc_value2 =  adc2;                                          //红外ADC值2	 
	reportFrame.infrared_adc_value3 =  adc3;                                          //红外ADC值3	 
	reportFrame.infrared_adc_value4 =  adc4;                                          //红外ADC值4	 
	reportFrame.infrared_adc_value5 =  adc5;                                          //红外ADC值5	 
	reportFrame.infrared_adc_value6 =  adc6;                                          //红外ADC值6	 
	reportFrame.infrared_adc_value7 =  adc7;                                          //红外ADC值7	 
	reportFrame.infrared_adc_value8 =  adc8;                                          //红外ADC值8	 
	reportFrame.infrared_adc_value9 =  adc9;                                          //红外ADC值9	 
	reportFrame.infrared_adc_value10 = adc10;                                         //红外ADC值10
	//reportFrame.infrared_cliff_status = cliffstatus;                                  //跳崖红外状态
	reportFrame.infrared_cliff_adc_value1 = adcCliffLeft;                             //跳崖ADC值1
	reportFrame.infrared_cliff_adc_value2 = adcCliffMiddle;                           //跳崖ADC值2
	reportFrame.infrared_cliff_adc_value3 = adcCliffRight;                            //跳崖ADC值3
	reportFrame.battery_voltage = batteryvoltage;                                     //电池电压
	reportFrame.dustbox_status = bsp_DustBoxGetState();                               //尘盒状态
	reportFrame.error_code = 0;         	                                          //异常状态
	reportFrame.machine_status = 0;                                                   //机器状态
	reportFrame.timestamp = timestamp;                                                //时间戳
	reportFrame.motor_left_voltage    = motorLeftVoltage;     
	reportFrame.motor_right_voltage   = motorRightVoltage;    
	reportFrame.motor_vacuum_voltage  = motorVacuumVoltage;   
	reportFrame.motor_rolling_voltage = motorRollingVoltage;  
	reportFrame.motor_side_voltage    = motorSideVoltage;     
	reportFrame.motor_battery_current = batteryCurrent;  
	reportFrame.checksum_msb = 0;                                                     //校验
	reportFrame.checksum_lsb = 0;                                                     //校验
	reportFrame.end_of_falg = 0x55;                                                   //帧结尾恒定为0x55
	
	/*计算校验*/
	chk = bsp_CalcChk(src+3,len-6);
	reportFrame.checksum_msb = chk >> 8;
	reportFrame.checksum_lsb = chk & 0x00FF;
	
	
	/*打印调试信息*/
	#if 1
	{
		float data[10] = {0};
		
		UNUSED(data);
		
		//DEBUG("angle:%.2F\r\n",angle/100.0F);
		//DEBUG("x_coordinate:%d  ",x_coordinate);
		//DEBUG("y_coordinate:%d\r\n",y_coordinate);
		
		data[0] =  angle;                                          //红外ADC值1	 
		data[1] =  adc2;                                          //红外ADC值2	 
		data[2] =  adc3;                                          //红外ADC值3	 
		data[3] =  adc4;                                          //红外ADC值4	 
		data[4] =  adc5;                                          //红外ADC值5	 
		data[5] =  adc6;                                          //红外ADC值6	 
		data[6] =  adc7;                                          //红外ADC值7	 
		data[7] =  adc8;                                          //红外ADC值8	 
		data[8] =  adc9;                                          //红外ADC值9	 
		data[9] =  adc10;                                         //红外ADC值10
		
		//bsp_ScopeSend(data,1);
	}
	#endif
	
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




