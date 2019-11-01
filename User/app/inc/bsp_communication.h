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
	uint16_t infrared_front_status; 	//12	前向红外状态 		u16
	uint8_t  infrared_edge_status;	    //13	沿边红外状态 		u8
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
	uint8_t  infrared_cliff_status;     //24    跳崖红外状态 		u8 
	uint16_t infrared_cliff_adc_value1; //25	跳崖ADC值1	 		u16
	uint16_t infrared_cliff_adc_value2; //26	跳崖ADC值2	 		u16
	uint16_t infrared_cliff_adc_value3; //27	跳崖ADC值3	 		u16
	uint8_t  battery_voltage;           //28	电池电压	 		u8
	uint8_t  charging_status;           //29	充电状态     		u8
	uint8_t  error_code;         	    //30	异常状态 	 		u8
	uint8_t  machine_status;            //31	机器状态	 		u8
	uint32_t timestamp;                 //32	时间戳	     		u32
	uint32_t reserved1;  	            //33	保留位1	     		u32
	uint32_t reserved2;				    //34	保留位2	     		u32
	uint32_t reserved3;         	    //35	保留位3	     		u32
	uint8_t checksum_msb;
	uint8_t checksum_lsb;
	uint8_t end_of_falg;           //0x55
}ReportFrame;
#pragma pack()


typedef struct
{
	uint16_t len;                       /*缓冲区数据个数*/
	uint8_t msgID;                      /*消息ID*/
}RouteAnalysis;

void bsp_SendReportFrame(void); /*发送帧*/
void bsp_ComAnalysis(void);     /*接收帧*/

#endif
