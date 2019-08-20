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
	uint16_t index = 0 ;
	
	/*选定串口*/
	port = COM4;
	
	while(comGetChar(port, &ch))
	{
		/*依次获取串口缓冲区每个字节*/
		analysisBuf[index % MAX_ANALYSIS_LEN] = ch ;
		index++;
		
		if(index == 1)    /*前面3个是帧头*/
		{
			if(ch != 0xAA)
			{
				index = 0 ;
			}
		}
		else if(index == 2)/*前面3个是帧头*/
		{
			if(ch != 0xAA)
			{
				index = 0 ;
			}
		}
		else if(index == 3)/*前面3个是帧头*/
		{
			if(ch != 0xAA)
			{
				index = 0 ;
			}
		}
		else if(index == 4)/*消息ID*/
		{
			routeAnalysis.msgID = ch;
			if(ch != CMD_ID_SPEED && ch != CMD_ID_DISTANCE && ch != CMD_ID_ANGLE)
			{
				index = 0 ;
			}
		}
		else if(index == 5)/*帧长度*/
		{
			routeAnalysis.len = ch;
		}
		else if(index >= (routeAnalysis.len+ 8))/*数据接收完毕（帧头帧尾4，检验2，ID 1，长度1）*/
		{
			uint16_t calcChk = bsp_CalcChk(analysisBuf+3,index-6); /*计算校验数据，用于校验的数据不包括前面的3个帧头和后面的1个帧尾，2个校验字节*/
			uint16_t rxChk   = analysisBuf[index-1-2] << 8 | analysisBuf[index-1-1];
			
			if(analysisBuf[index-1] != 0x55 || calcChk != rxChk)
			{
				index = 0 ;
			}
			else /*获得了正确的解析数据*/
			{
				if(routeAnalysis.msgID == CMD_ID_SPEED)
				{
					int16_t linearVelocity  = analysisBuf[5]<<8 | analysisBuf[6];
					int16_t angularVelocity = analysisBuf[7]<<8 | analysisBuf[8];
					
					/*计算出速度，单位MM/S */
					int16_t leftVelocity = (int16_t)((0.5*(2*linearVelocity*0.001 - Deg2Rad(angularVelocity)*WHEEL_LENGTH))* 1000);
					int16_t rightVelocity = (int16_t)((0.5*(2*linearVelocity*0.001 + Deg2Rad(angularVelocity)*WHEEL_LENGTH))* 1000);
					
					/*设定速度*/
					bsp_SetMotorSpeed(MotorLeft,bsp_MotorSpeedMM2Pulse(leftVelocity));
					bsp_SetMotorSpeed(MotorRight,bsp_MotorSpeedMM2Pulse(rightVelocity));
				}
				
				/*重新开始计数*/
				index = 0 ;
			}
		}
	}
	
	
	/*选定串口*/
	port = COM1;
	
	/*选定串口*/
	port = COM2;
	
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
	
	
	
	
	reportFrame.sof1 = 0xAA;                  		                                  //0xAA
	reportFrame.sof2 = 0xAA;                  		                                  //0xAA
	reportFrame.sof3 = 0xAA;                  		                                  //0xAA
	reportFrame.identifier_major = MIN_ID_ENVIRONMENT;                                //主识别码
	reportFrame.identifier_subs = 0x00;        	                                      //子识别码
	reportFrame.size_of_payload_field = dataLen;   	                                  //数据长度（不包括头标识3，尾标识1，校验2）
	reportFrame.left_wheel_pulse_count = leftPulseVector;   //左轮编码器计数
	reportFrame.right_wheel_pluse_count = rightPulseVector; //右轮编码器计数
	reportFrame.left_wheel_velocity = leftSpeedMM;  		  //左轮电机速度
	reportFrame.right_wheel_veloctiy = rightSpeedMM;		  //右轮电机速度
	reportFrame.x_coordinate = x_coordinate;				  //X坐标
	reportFrame.y_coordinate = y_coordinate;				  //Y坐标
	reportFrame.theta_angle_deg = angle;		              //航向角
	reportFrame.landoff_button = 0;                                                   //离地开关
	reportFrame.collosion_button = bsp_CollisionScan() ; 			                  //碰撞开关
	reportFrame.infrared_front_status = 0; 	                                          //前向红外状态 
	reportFrame.infrared_edge_status = 0;	                                          //沿边红外状态
	reportFrame.infrared_adc_value1 =  adc1;                //红外ADC值1	 
	reportFrame.infrared_adc_value2 =  adc2;                //红外ADC值2	 
	reportFrame.infrared_adc_value3 =  adc3;                //红外ADC值3	 
	reportFrame.infrared_adc_value4 =  adc4;                //红外ADC值4	 
	reportFrame.infrared_adc_value5 =  adc5;                //红外ADC值5	 
	reportFrame.infrared_adc_value6 =  adc6;                //红外ADC值6	 
	reportFrame.infrared_adc_value7 =  adc7;                //红外ADC值7	 
	reportFrame.infrared_adc_value8 =  adc8;                //红外ADC值8	 
	reportFrame.infrared_adc_value9 =  adc9;                //红外ADC值9	 
	reportFrame.infrared_adc_value10 = adc10;               //红外ADC值10
	reportFrame.infrared_cliff_status = 0 ;                                           //跳崖红外状态
	reportFrame.infrared_cliff_adc_value1 = 0 ;                                       //跳崖ADC值1
	reportFrame.infrared_cliff_adc_value2 = 0 ;                                       //跳崖ADC值2
	reportFrame.infrared_cliff_adc_value3 = 0 ;                                       //跳崖ADC值3
	reportFrame.battery_voltage = 0;                                                  //电池电压
	reportFrame.charging_status = 0;                                                  //充电状态
	reportFrame.error_code = 0;         	                                          //异常状态
	reportFrame.machine_status = 0;                                                   //机器状态
	reportFrame.timestamp = timestamp;                      //时间戳
	reportFrame.reserved1 = 0;  	                                                  //保留位1
	reportFrame.reserved2 = 0;				                                          //保留位2
	reportFrame.reserved3 = 0;         	                                              //保留位3
	reportFrame.checksum_msb = 0;                                                     //校验
	reportFrame.checksum_lsb = 0;                                                     //校验
	reportFrame.end_of_falg = 0x55;                                                   //帧结尾恒定为0x55
	
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




