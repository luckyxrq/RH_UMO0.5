#ifndef __BSP_COMMUNICATION_H
#define __BSP_COMMUNICATION_H


#define  MIN_ID_ENVIRONMENT 	0x30	// ����
#define  MIN_ID_MOTION  		0x25	// �˶�
#define  MIN_ID_POSE  			0x20	// ����
#define  MIN_ID_MAP 			0x40	// ��ͼ


#define  CMD_ID_SPEED 	        0x25
#define  CMD_ID_DISTANCE  		0x35
#define  CMD_ID_ANGLE  			0x45

#define WHEEL_LENGTH  			0.235F
#define M_PI 					3.14F
#define MAX_ANALYSIS_LEN	    512


#define Deg2Rad(deg) (M_PI * deg / 180.0F)
#define Rad2Deg(rad) (180.0F * rad / M_PI)


/*����1�ֽڶ��룬���ڴ洢��uint8_t����buf*/
#pragma pack(1)
typedef struct
{
	uint8_t sof1;                  		//0xAA
	uint8_t sof2;                  		//0xAA
	uint8_t sof3;                  		//0xAA
	uint8_t  identifier_major;       	//0     ��ʶ����            u8
	uint8_t  identifier_subs;        	//1		��ʶ����	        u8
	uint8_t  size_of_payload_field;   	//2		���ݳ���	        u8
	int32_t  left_wheel_pulse_count;  	//3		���ֱ���������	    u32
	int32_t  right_wheel_pluse_count; 	//4		���ֱ���������	    u32
	int16_t  left_wheel_velocity;  		//5		���ֵ���ٶ�        int16
	int16_t  right_wheel_veloctiy;		//6		���ֵ���ٶ�		int16
	int32_t  x_coordinate;				//7		X����				int32
	int32_t  y_coordinate;				//8		Y����				int32
	int16_t  theta_angle_deg;			//9		�����	        	int16
	uint8_t  landoff_button;            //10	��ؿ���	 		u8
	uint8_t  collosion_button; 			//11 	��ײ����	 		u8
	uint16_t infrared_front_status; 	//12	ǰ�����״̬ 		u16
	uint8_t  infrared_edge_status;	    //13	�رߺ���״̬ 		u8
	uint16_t infrared_adc_value1;       //14	����ADCֵ1	 		u16
	uint16_t infrared_adc_value2;       //15	����ADCֵ2	 		u16
	uint16_t infrared_adc_value3;       //16	����ADCֵ3	 		u16
	uint16_t infrared_adc_value4;       //17	����ADCֵ4	 		u16
	uint16_t infrared_adc_value5;       //18	����ADCֵ5	 		u16
	uint16_t infrared_adc_value6;       //19	����ADCֵ6	 		u16
	uint16_t infrared_adc_value7;       //20	����ADCֵ7	 		u16
	uint16_t infrared_adc_value8;       //21	����ADCֵ8	 		u16
	uint16_t infrared_adc_value9;       //22	����ADCֵ9	 		u16
	uint16_t infrared_adc_value10;      //23	����ADCֵ10	 		u16
	uint8_t  infrared_cliff_status;     //24    ���º���״̬ 		u8 
	uint16_t infrared_cliff_adc_value1; //25	����ADCֵ1	 		u16
	uint16_t infrared_cliff_adc_value2; //26	����ADCֵ2	 		u16
	uint16_t infrared_cliff_adc_value3; //27	����ADCֵ3	 		u16
	uint8_t  battery_voltage;           //28	��ص�ѹ	 		u8
	uint8_t  charging_status;           //29	���״̬     		u8
	uint8_t  error_code;         	    //30	�쳣״̬ 	 		u8
	uint8_t  machine_status;            //31	����״̬	 		u8
	uint32_t timestamp;                 //32	ʱ���	     		u32
	uint32_t reserved1;  	            //33	����λ1	     		u32
	uint32_t reserved2;				    //34	����λ2	     		u32
	uint32_t reserved3;         	    //35	����λ3	     		u32
	uint8_t checksum_msb;
	uint8_t checksum_lsb;
	uint8_t end_of_falg;           //0x55
}ReportFrame;
#pragma pack()


typedef struct
{
	uint16_t len;                       /*���������ݸ���*/
	uint8_t msgID;                      /*��ϢID*/
}RouteAnalysis;

void bsp_SendReportFrame(void); /*����֡*/
void bsp_ComAnalysis(void);     /*����֡*/

#endif
