

#define RX_BUF_SIZE   96             /*���ڽ��ս������ݵĻ�������С*/
#define RX_BAUD       115200         /*������ͨ������*/
#define SIZE_WITH_ROLL_PITCH      18 /*һ֡�Ƕ���Ϣ�����ֽڴ�С*/

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
	
	uint16_t temp_angular_velocity;  /*��ʱ����������У��*/
	uint16_t temp_roll;              /*��ʱ����������У��*/
	uint16_t temp_pitch;             /*��ʱ����������У��*/
	uint16_t temp_yaw;               /*��ʱ����������У��*/
	uint16_t temp_chk;               /*��ʱ����������У��*/
}Angle;


float bsp_IMU_GetData(IMU_DATA_TYPE type);
void bsp_AngleRevByte(uint8_t byte);
float bsp_AngleAdd(float angle1 , float angle2);
void bsp_InitAngle(void);
void bsp_AngleRst(void);
float bsp_AngleRead(void);
int16_t bsp_AngleReadRaw(void);



