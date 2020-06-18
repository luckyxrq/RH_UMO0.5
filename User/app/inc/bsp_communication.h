#ifndef __BSP_COMMUNICATION_H
#define __BSP_COMMUNICATION_H


#define  MIN_ID_ENVIRONMENT 	0x30	// 环境
#define  MIN_ID_MOTION  		0x25	// 运动
#define  MIN_ID_POSE  			0x20	// 姿势
#define  MIN_ID_MAP 			0x40	// 地图


#define  CMD_ID_SPEED 	        0x25
#define  CMD_ID_DISTANCE  		0x35
#define  CMD_ID_ANGLE  			0x45

#define WHEEL_LENGTH  			0.235F
#define M_PI 					3.14F
#define MAX_ANALYSIS_LEN	    512


#define Deg2Rad(deg) (M_PI * deg / 180.0F)
#define Rad2Deg(rad) (180.0F * rad / M_PI)


/*按照1字节对齐，便于存储到uint8_t类型buf*/
#pragma pack(1)
typedef struct
{
	uint8_t sof1;                  		//0xAA
	uint8_t sof2;                  		//0xAA
	uint8_t sof3;                  		//0xAA
	uint8_t  identifier_major;       	//0     主识别码            u8
	uint8_t  identifier_subs;        	//1		子识别码	        u8
	uint8_t  size_of_payload_field;   	//2		数据长度	        u8
	int32_t  left_wheel_pulse_count;  	//3		左轮编码器计数	    u32
	int32_t  right_wheel_pluse_count; 	//4		右轮编码器计数	    u32
	int16_t  left_wheel_velocity;  		//5		左轮电机速度        int16
	int16_t  right_wheel_veloctiy;		//6		右轮电机速度		int16
	int32_t  x_coordinate;				//7		X坐标				int32
	int32_t  y_coordinate;				//8		Y坐标				int32
	int16_t  theta_angle_deg;			//9		航向角	        	int16
	uint8_t  landoff_button;            //10	离地开关	 		u8
	uint8_t  collosion_button; 			//11 	碰撞开关	 		u8
	uint8_t  infrared_charge1_status;   //12	回充红外状态 		u16
	uint8_t  infrared_charge2_status;					//13	保留 		 		u8
	uint8_t  infrared_charge3_status;					//13	保留 		 		u8
	uint16_t infrared_adc_value1;       //14	红外ADC值1	 		u16
	uint16_t infrared_adc_value2;       //15	红外ADC值2	 		u16
	uint16_t infrared_adc_value3;       //16	红外ADC值3	 		u16
	uint16_t infrared_adc_value4;       //17	红外ADC值4	 		u16
	uint16_t infrared_adc_value5;       //18	红外ADC值5	 		u16
	uint16_t infrared_adc_value6;       //19	红外ADC值6	 		u16
	uint16_t infrared_adc_value7;       //20	红外ADC值7	 		u16
	uint16_t infrared_adc_value8;       //21	红外ADC值8	 		u16
	uint16_t infrared_adc_value9;       //22	红外ADC值9	 		u16
	uint16_t infrared_adc_value10;      //23	红外ADC值10	 		u16
	uint8_t  infrared_charge4_status;     //24    跳崖红外状态 		u8 
	uint16_t infrared_cliff_adc_value1; //25	跳崖ADC值1	 		u16
	uint16_t infrared_cliff_adc_value2; //26	跳崖ADC值2	 		u16
	uint16_t infrared_cliff_adc_value3; //27	跳崖ADC值3	 		u16
	uint8_t  battery_voltage;           //28	电池电压	 		u8
	uint8_t  dustbox_status;            //29	尘盒状态     		u8
	uint8_t  error_code;         	    //30	异常状态 	 		u8
	uint8_t  machine_status;            //31	机器状态	 		u8
	uint32_t timestamp;                 //32	时间戳	     		u32
	uint16_t motor_left_voltage;        //      左轮电机电压        u16
	uint16_t motor_right_voltage;       //      右轮电机电压        u16
	uint16_t motor_vacuum_voltage;      //      吸尘电机电压        u16 
	uint16_t motor_rolling_voltage;     //      滚刷电机电压        u16  
	uint16_t motor_side_voltage;        //      边刷电机电压        u16
	uint16_t motor_battery_current;     //      电池放电电流        u16    
	uint8_t checksum_msb;
	uint8_t checksum_lsb;
	uint8_t end_of_falg;           //0x55
}ReportFrame;
#pragma pack()

/*按照1字节对齐，便于存储到uint8_t类型buf*/
#pragma pack(1)
typedef struct
{
	uint16_t head;
	
	uint16_t frame_len;
	uint16_t frame_len_reverse;
	
    uint16_t tx_addr;
    uint16_t rx_addr;
	
    uint16_t main_sec ;
    uint16_t sub_sec ;
	
	/*********数据部分开始***********/
	uint8_t dustBox;
	
	int16_t wheelSpeedL;
	int16_t wheelSpeedR;
	
	int32_t wheelPulseL;
	int32_t wheelPulseR;
	
	int32_t x_pos;
	int32_t y_pos;
	
	uint8_t offsiteSW;
	uint8_t collision;
	
	uint16_t cliffMV_L;
	uint16_t cliffMV_M;
	uint16_t cliffMV_R;
	
	int16_t yaw;       /*角度值*100传过去*/
	
	uint16_t irMV[10];  /*一圈红外MV*/
	uint8_t  irRX[4][3];  /*红外接收*/

	
	uint16_t mA_wheelL;
	uint16_t mA_wheelR;
	uint16_t mA_roll;
	uint16_t mA_sideBrush;
	uint16_t mA_vacuum;
	uint16_t v_batteryVoltage;
	uint16_t mA_batteryCurrent;
	
	/*********数据部分结束***********/
	
	/*********拓展数据部分开始***********/
	
	/*********拓展数据部分结束***********/
	
	uint16_t crc16;
}ReportFrameWithCRC16;
#pragma pack()

typedef struct
{
	uint16_t len;                       /*缓冲区数据个数*/
	uint8_t msgID;                      /*消息ID*/
}RouteAnalysis;

void bsp_SendReportFrame(void); /*发送帧*/
void bsp_ComAnalysis(void);     /*接收帧*/
void bsp_SendReportFrameWithCRC16(void);

/*新协议*/
uint8_t GetCmdStartUpload(void);

#endif
