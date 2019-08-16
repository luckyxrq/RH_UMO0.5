#include "bsp.h"

#define  MIN_ID_ENVIRONMENT 	0x30	// ����
#define  MIN_ID_MOTION  		0x25	// �˶�
#define  MIN_ID_POSE  			0x20	// ����
#define  MIN_ID_MAP 			0x40	// ��ͼ


uint8_t  buf[256] = {0};       /*���ڴ洢֡��ʹ��ȫ�ֱ�������ֹƵ������ջ�ռ�*/

double DegToRad(double deg) { return M_PI * deg / 180.; }
double RadToDeg(double rad) { return 180. * rad / M_PI; }



#pragma pack(1)
typedef struct
{
	uint8_t sof1;                  //0xAA
	uint8_t sof2;                  //0xAA
	uint8_t sof3;                  //0xAA
	uint8_t identifier;	           //0x25
	uint8_t size_of_payload_field; //0x04
	int32_t left_wheel_pulse_count;
	int32_t right_wheel_pulse_count;
	uint8_t button_control_cmd;
	uint16_t distance_of_left_infrared;
	uint16_t distance_of_right_infrared;
	uint16_t distance_of_front_infrared;
	int16_t  angle_deg;
	uint16_t adc_1;
	uint16_t adc_2;
	uint16_t adc_3;
	uint16_t adc_4;
	int16_t acc_x;
	int16_t acc_y;
	int16_t acc_z;
	uint16_t obstacle_signal;
	uint8_t battery_level_soc;
	uint8_t charge_status;
	uint8_t checksum_msb;
	uint8_t checksum_lsb;
	uint8_t end_of_falg;           //0x55
}ReportFrame;



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
}SensorDataFrame;



typedef struct min_id_motion
{
//	uint8_t sof1;    //0xAA
//	uint8_t sof2;    //0xAA
//	uint8_t sof3;    //0xAA
	uint8_t identifier;	   //0x25
	uint8_t size_of_payload_field; //0x04
	int16_t linear_velocity;
	int16_t angular_velocity;
//	uint8_t checksum_msb;
//	uint8_t checksum_lsb;
//	uint8_t end_of_falg; //0x55
}S_MIN_ID_MOTION;

typedef struct _S_CMD_
{	
	uint8_t  state;
	uint8_t  cmd;
	uint8_t  cnt;
	uint8_t  len;
	uint8_t  lenght;
	uint8_t  *buf;
	uint32_t time;
}S_Cmd;

typedef struct _BUF_
{
	uint16_t  in;
	uint16_t  out;
	uint8_t   *array;
}S_BUF;


static ReportFrame reportFrame;
static uint16_t bsp_CalcChk(uint8_t *buf, uint8_t len);
static void bsp_FillReportFrame(void);
static void bsp_UartAnalysis(uint8_t *buf,int16_t* left_velocity,int16_t* right_velocity);

uint8_t  BB16_Buf[1*1024] = {0};
S_Cmd    BB16_Cmd = {0,0,0,0,0,BB16_Buf,0};



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
		buf[i] = src[i];
	}
	
	comSendBuf(COM4,buf,len);	
}

/*
*********************************************************************************************************
*	�� �� ��: bsp_ReveiceCmdFrame
*	����˵��: ����֡���ս���
*	��    ��: ��
*	�� �� ֵ: ��
*********************************************************************************************************
*/
uint8_t bsp_ReveiceCmdFrame(int16_t* left_velocity,int16_t* right_velocity)
{
	uint8_t ch = 0;
	uint16_t cnt = 0;
	uint16_t i = 0;
	
	uint16_t fletcher16=0;
	uint8_t  BB16_Tmp_Buf[1*1024] = {0};
	uint8_t  vaule = 0;

	
	while(comGetChar(COM4, &ch))
	{
		BB16_Tmp_Buf[cnt] = ch;
		cnt++;
	}
	
	if(cnt<12) return 0;
	
	while(cnt--)
	{
		vaule = BB16_Tmp_Buf[i];
		i++;
		BB16_Cmd.buf[BB16_Cmd.cnt] = vaule; 	
		BB16_Cmd.cnt++;
		
		switch(BB16_Cmd.state)
		{
			case BB16_SOF_1:		// ֡ͷ1
					if(0xAA == vaule)
					{
						BB16_Cmd.state = BB16_SOF_2;
					}
					else
					{
						BB16_Cmd.state = BB16_SOF_1;
						BB16_Cmd.cnt   = 0;
					}
					break;
					
			case BB16_SOF_2:		// ֡ͷ2
					if(0xAA == vaule)
					{
						BB16_Cmd.state = BB16_SOF_3;
					}
					else
					{
						BB16_Cmd.state = BB16_SOF_1;
						BB16_Cmd.cnt   = 0;
					}
					break;
					
			case BB16_SOF_3:		// ֡ͷ3
					if(0xAA == vaule)
					{
						BB16_Cmd.state = BB16_MESSAGE_ID;
					}
					else
					{
						BB16_Cmd.state = BB16_SOF_1;
						BB16_Cmd.cnt   = 0;
					}
					break;
											
			case BB16_MESSAGE_ID:	// ��ϢID
					if(CMD_ID_SPEED == vaule  || CMD_ID_DISTANCE == vaule ||  CMD_ID_ANGLE == vaule)
					{
						BB16_Cmd.state = BB16_LENGHT;		
					}
					else
					{
						BB16_Cmd.state = BB16_SOF_1;
						BB16_Cmd.cnt   = 0;
					}
					break;
					
			case BB16_LENGHT:		// ���ݳ���
					BB16_Cmd.len    = 0;
					BB16_Cmd.lenght = vaule;
					
					if(BB16_Cmd.lenght >= 255)
					{
						BB16_Cmd.cnt   = 0;
						BB16_Cmd.state = BB16_SOF_1;
					}
					else if(0 == BB16_Cmd.lenght)
					{
						BB16_Cmd.state = BB16_CKB;
					}
					else
					{
						BB16_Cmd.state = BB16_PAYLOAD;
					}	
					break;
					
			case BB16_PAYLOAD:		// ��Ч����
					BB16_Cmd.len++;
					if(BB16_Cmd.len == BB16_Cmd.lenght)
					{
						BB16_Cmd.state = BB16_CKB;
					}	
					break;
					
			case BB16_CKB:			// У��B
					fletcher16 = vaule << 8;
					BB16_Cmd.state = BB16_CKA;
					break;
			
			case BB16_CKA:			// У��A
					fletcher16 += vaule;
					BB16_Cmd.state = BB16_END;
					break;
					
			case BB16_END:			// ֡β
				
					if((0x55==vaule) && (fletcher16==bsp_CalcChk(BB16_Cmd.buf+3,BB16_Cmd.cnt-6)))
					{
						bsp_UartAnalysis(BB16_Cmd.buf,left_velocity,right_velocity);
						memset(&BB16_Cmd,0,sizeof(S_Cmd));
						return 1;
					}
					else			// У��δͨ��
					{
						BB16_Cmd.cnt   = 0;
						BB16_Cmd.state = BB16_SOF_1;
									
					}		
		}			
	}
	
	return 0;

}

/*
*********************************************************************************************************
*	�� �� ��: bsp_UartAnalysis
*	����˵��: ����֡��ֵ����
*	��    ��: ��
*	�� �� ֵ: ��
*********************************************************************************************************
*/
void bsp_UartAnalysis(uint8_t *buf,int16_t* left_velocity,int16_t* right_velocity)
{
	int16_t linear_velocity = 0,angular_velocity = 0;
	uint8_t  message_id;
	
	message_id = buf[3];			// ��ϢID
	
	switch (message_id)
	{
		case CMD_ID_SPEED:			// �˶�
			linear_velocity  = (buf[5]<<8) + buf[6];
			angular_velocity = (buf[7]<<8) + buf[8];

			*left_velocity  = (int16_t)((0.5*(2*linear_velocity*0.001 - DegToRad(angular_velocity)*WHEEL_LENGTH))* 1000);
			*right_velocity = (int16_t)((0.5*(2*linear_velocity*0.001 + DegToRad(angular_velocity)*WHEEL_LENGTH))* 1000);
			break;
		case CMD_ID_DISTANCE:
			
			break;
		case CMD_ID_ANGLE:
			
			break;
		default:
			break;
	}

	
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
	uint16_t chk = 0 ;
	uint32_t len = sizeof(reportFrame);/*֡��С*/
	uint8_t* src = (uint8_t*)&reportFrame;
	int16_t angle = bsp_AngleReadRaw()*-1;                     /*�Ƕ�*/
	int32_t odometerL = bsp_encoderGetOdometer(MotorLeft);  /*��̼� ��*/
	int32_t odometerR = bsp_encoderGetOdometer(MotorRight); /*��̼� ��*/
	uint16_t collisionRL = bsp_CollisionScan();
	
	/*��������������*/
	UNUSED(reportFrame);
	
	/*��С��ת��*/
	reportFrame.sof1 = 0xAA;                     //�㶨Ϊ0xAA
	reportFrame.sof2 = 0xAA;                     //�㶨Ϊ0xAA
	reportFrame.sof3 = 0xAA;                     //�㶨Ϊ0xAA
	reportFrame.identifier = MIN_ID_ENVIRONMENT; //�㶨Ϊ0x25
	reportFrame.size_of_payload_field = 0x1B+8; 
	reportFrame.left_wheel_pulse_count =  BEBufToUint32((uint8_t*)&odometerL);
	reportFrame.right_wheel_pulse_count = BEBufToUint32((uint8_t*)&odometerR);
	reportFrame.button_control_cmd = 0 ;
	reportFrame.distance_of_left_infrared = collisionRL;
	reportFrame.distance_of_right_infrared = 0 ;
	reportFrame.distance_of_front_infrared = 0 ;
	reportFrame.angle_deg = BEBufToUint16((uint8_t*)&angle);
	reportFrame.adc_1 = 0 ;
	reportFrame.adc_2 = 0 ;
	reportFrame.adc_3 = 0 ;
	reportFrame.adc_4 = 0 ;
	reportFrame.acc_x = 0 ;
	reportFrame.acc_y = 0 ;
	reportFrame.acc_z = 0 ;
	reportFrame.obstacle_signal = 0 ;
	reportFrame.battery_level_soc = 0 ;
	reportFrame.charge_status = 0 ;
	reportFrame.checksum_msb = 0 ;
	reportFrame.checksum_lsb = 0 ;
	reportFrame.end_of_falg = 0x55 ;                  /*�㶨0x55*/
	
	/*����У��*/
	chk = bsp_CalcChk(src+3,len-6);
	reportFrame.checksum_msb = chk >> 8;
	reportFrame.checksum_lsb = chk & 0x00FF;
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


