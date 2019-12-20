

#define RX_BUF_SIZE   96             /*用于接收解析数据的缓冲区大小*/
#define RX_BAUD       115200         /*陀螺仪通信速率*/
#define SIZE_WITH_ROLL_PITCH      79 /*一帧角度信息的总字节大小*/

/*各个信息在数据帧中的起始位置*/
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

/*为了兼容旧版驱动接口，故提供此函数*/
float bsp_AngleRead(void);
int16_t bsp_AngleReadRaw(void);



