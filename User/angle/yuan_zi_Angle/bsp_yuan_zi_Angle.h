

#define RX_BUF_SIZE   96             /*���ڽ��ս������ݵĻ�������С*/
#define RX_BAUD       115200         /*������ͨ������*/
#define SIZE_WITH_ROLL_PITCH      79 /*һ֡�Ƕ���Ϣ�����ֽڴ�С*/

/*������Ϣ������֡�е���ʼλ��*/
typedef enum
{
	POS_ANGLE_CHK = 77,
	POS_HEAD1_CHK = 0,
	POS_HEAD2_CHK = 21,
	POS_HEAD3_CHK = 36,
	POS_HEAD4_CHK = 51,
	POS_TAIL_CHK = 78,
}POS_ANGLE;

typedef enum
{
	ACC_X = 0 ,
	ACC_Y     ,
	ACC_Z     ,
	PITCH     ,
	ROLL      ,
	YAW       ,
}IMU_DATA_TYPE;

typedef struct
{
	volatile bool timeout ;
	volatile uint16_t rxCount;
	volatile uint8_t  buf[RX_BUF_SIZE] ;
	volatile float angle;
	volatile int16_t angleRaw;
	
	float accX;
	float accY;
	float accZ;
	
	float roll;
	float pitch;
	float yaw;
}Angle;

void bsp_IMU_Calibration(void);
float bsp_IMU_GetData(IMU_DATA_TYPE type);
void bsp_AngleRevByte(uint8_t byte);
float bsp_AngleAdd(float angle1 , float angle2);
void bsp_InitAngle(void);
void bsp_AngleRst(void);

/*Ϊ�˼��ݾɰ������ӿڣ����ṩ�˺���*/
float bsp_AngleRead(void);
int16_t bsp_AngleReadRaw(void);



