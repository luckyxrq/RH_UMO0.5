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
//	uint16_t chk = 0 ;
//	uint32_t len = sizeof(reportFrame);/*帧大小*/
//	uint8_t* src = (uint8_t*)&reportFrame;
//	int16_t angle = bsp_AngleReadRaw();                     /*角度*/
//	int32_t odometerL = 0;//bsp_encoderGetOdometer(MotorLeft);  /*里程计 左*/
//	int32_t odometerR = 0;//bsp_encoderGetOdometer(MotorRight); /*里程计 右*/
//	
//	/*消除编译器警告*/
//	UNUSED(reportFrame);
//	
//	/*大小端转换*/
//	reportFrame.sof1 = 0xAA;                     //恒定为0xAA
//	reportFrame.sof2 = 0xAA;                     //恒定为0xAA
//	reportFrame.sof3 = 0xAA;                     //恒定为0xAA
//	reportFrame.identifier = MIN_ID_ENVIRONMENT; //恒定为0x25
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
//	reportFrame.end_of_falg = 0x55 ;                  /*恒定0x55*/
//	
//	/*计算校验*/
//	chk = bsp_CalcChk(src+3,len-6);
//	reportFrame.checksum_msb = chk >> 8;
//	reportFrame.checksum_lsb = chk & 0x00FF;



	reportFrame.sof1 = 0xAA;                  		                    //0xAA
	reportFrame.sof2 = 0xAA;                  		                    //0xAA
	reportFrame.sof3 = 0xAA;                  		                    //0xAA
	reportFrame.identifier_major = MIN_ID_ENVIRONMENT;                  //主识别码
	reportFrame.identifier_subs = 0x00;        	                        //子识别码
	reportFrame.size_of_payload_field = sizeof(reportFrame) - 6;   	    //数据长度（不包括头标识3，尾标识1，校验2）
	reportFrame.left_wheel_pulse_count = 0 ;  	                        //左轮编码器计数
	reportFrame.right_wheel_pluse_count = 0; 	                        //右轮编码器计数
	reportFrame.left_wheel_velocity = 0 ;  		                        //左轮电机速度
	reportFrame.right_wheel_veloctiy = 0 ;		                        //右轮电机速度
	reportFrame.x_coordinate = 0 ;				                        //X坐标
	reportFrame.y_coordinate = 0 ;				                        //Y坐标
	reportFrame.theta_angle_deg = 0;			                        //航向角
	reportFrame.landoff_button = 0;                                     //离地开关
	reportFrame.collosion_button = 0 ; 			                        //碰撞开关
	reportFrame.infrared_front_status = 0; 	                            //前向红外状态 
	reportFrame.infrared_edge_status = 0;	                            //沿边红外状态
	reportFrame.infrared_adc_value1 = 0;                                //红外ADC值1	 
	reportFrame.infrared_adc_value2 = 0;                                //红外ADC值2	 
	reportFrame.infrared_adc_value3 = 0;                                //红外ADC值3	 
	reportFrame.infrared_adc_value4 = 0;                                //红外ADC值4	 
	reportFrame.infrared_adc_value5 = 0;                                //红外ADC值5	 
	reportFrame.infrared_adc_value6 = 0;                                //红外ADC值6	 
	reportFrame.infrared_adc_value7 = 0;                                //红外ADC值7	 
	reportFrame.infrared_adc_value8 = 0;                                //红外ADC值8	 
	reportFrame.infrared_adc_value9 = 0;                                //红外ADC值9	 
	reportFrame.infrared_adc_value10 = 0;                               //红外ADC值10
	reportFrame.infrared_cliff_status = 0 ;                             //跳崖红外状态
	reportFrame.infrared_cliff_adc_value1 = 0 ;                         //跳崖ADC值1
	reportFrame.infrared_cliff_adc_value2 = 0 ;                         //跳崖ADC值2
	reportFrame.infrared_cliff_adc_value3 = 0 ;                         //跳崖ADC值3
	reportFrame.battery_voltage = 0;                                    //电池电压
	reportFrame.charging_status = 0;                                    //充电状态
	reportFrame.error_code = 0;         	                            //异常状态
	reportFrame.machine_status = 0;                                     //机器状态
	reportFrame.timestamp = 0;                                          //时间戳
	reportFrame.reserved1 = 0;  	                                    //保留位1
	reportFrame.reserved2 = 0;				                            //保留位2
	reportFrame.reserved3 = 0;         	                                //保留位3
	reportFrame.checksum_msb = 0;
	reportFrame.checksum_lsb = 0;
	reportFrame.end_of_falg = 0;                                        //0x55
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




