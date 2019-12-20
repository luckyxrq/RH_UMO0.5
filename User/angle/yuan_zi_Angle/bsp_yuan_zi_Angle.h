

#define RX_BUF_SIZE   96             /*用于接收解析数据的缓冲区大小*/
#define RX_BAUD       115200         /*陀螺仪通信速率*/
#define SIZE_WITH_ROLL_PITCH      18 /*一帧角度信息的总字节大小*/

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
	
	uint16_t temp_angular_velocity;  /*临时变量，用作校验*/
	uint16_t temp_roll;              /*临时变量，用作校验*/
	uint16_t temp_pitch;             /*临时变量，用作校验*/
	uint16_t temp_yaw;               /*临时变量，用作校验*/
	uint16_t temp_chk;               /*临时变量，用作校验*/
}Angle;


float bsp_IMU_GetData(IMU_DATA_TYPE type);
void bsp_AngleRevByte(uint8_t byte);
float bsp_AngleAdd(float angle1 , float angle2);
void bsp_InitAngle(void);
void bsp_AngleRst(void);
float bsp_AngleRead(void);
int16_t bsp_AngleReadRaw(void);



