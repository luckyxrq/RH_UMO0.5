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
	uint8_t  infrared_charge1_status;   //12	�س����״̬ 		u16
	uint8_t  infrared_charge2_status;					//13	���� 		 		u8
	uint8_t  infrared_charge3_status;					//13	���� 		 		u8
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
	uint8_t  infrared_charge4_status;     //24    ���º���״̬ 		u8 
	uint16_t infrared_cliff_adc_value1; //25	����ADCֵ1	 		u16
	uint16_t infrared_cliff_adc_value2; //26	����ADCֵ2	 		u16
	uint16_t infrared_cliff_adc_value3; //27	����ADCֵ3	 		u16
	uint8_t  battery_voltage;           //28	��ص�ѹ	 		u8
	uint8_t  dustbox_status;            //29	����״̬     		u8
	uint8_t  error_code;         	    //30	�쳣״̬ 	 		u8
	uint8_t  machine_status;            //31	����״̬	 		u8
	uint32_t timestamp;                 //32	ʱ���	     		u32
	uint16_t motor_left_voltage;        //      ���ֵ����ѹ        u16
	uint16_t motor_right_voltage;       //      ���ֵ����ѹ        u16
	uint16_t motor_vacuum_voltage;      //      ���������ѹ        u16 
	uint16_t motor_rolling_voltage;     //      ��ˢ�����ѹ        u16  
	uint16_t motor_side_voltage;        //      ��ˢ�����ѹ        u16
	uint16_t motor_battery_current;     //      ��طŵ����        u16    
	uint8_t checksum_msb;
	uint8_t checksum_lsb;
	uint8_t end_of_falg;           //0x55
}ReportFrame;
#pragma pack()

/*����1�ֽڶ��룬���ڴ洢��uint8_t����buf*/
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
	
	/*********���ݲ��ֿ�ʼ***********/
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
	
	int16_t yaw;       /*�Ƕ�ֵ*100����ȥ*/
	
	uint16_t irMV[10];  /*һȦ����MV*/
	uint8_t  irRX[4][3];  /*�������*/

	
	uint16_t mA_wheelL;
	uint16_t mA_wheelR;
	uint16_t mA_roll;
	uint16_t mA_sideBrush;
	uint16_t mA_vacuum;
	uint16_t v_batteryVoltage;
	uint16_t mA_batteryCurrent;
	
	/*********���ݲ��ֽ���***********/
	
	/*********��չ���ݲ��ֿ�ʼ***********/
	
	/*********��չ���ݲ��ֽ���***********/
	
	uint16_t crc16;
}ReportFrameWithCRC16;
#pragma pack()

typedef struct
{
	uint16_t len;                       /*���������ݸ���*/
	uint8_t msgID;                      /*��ϢID*/
}RouteAnalysis;

void bsp_SendReportFrame(void); /*����֡*/
void bsp_ComAnalysis(void);     /*����֡*/
void bsp_SendReportFrameWithCRC16(void);

/*��Э��*/
uint8_t GetCmdStartUpload(void);

#endif
