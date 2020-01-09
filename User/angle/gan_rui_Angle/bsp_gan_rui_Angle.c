#include "bsp.h"


#define CALIBRATION_SIZE      8

static Angle angle;

static void bsp_AngleTimeout(void);
static void bsp_AnglePoll(void);
static void bsp_AngleAnalyzeApp(void);
static bool bsp_AngleCheck(void);


/* �����ڶ�Ӧ��RCCʱ�� */
#define RCC_ALL_ANGLE 	(RCC_APB2Periph_GPIOA)

#define GPIO_PORT_RST    GPIOA
#define GPIO_PIN_RST	 GPIO_Pin_1


/*
*********************************************************************************************************
*	�� �� ��: bsp_AngleRead,(Ϊ�˼��ݾɰ������ӿڣ����ṩ�˺���)
*	����˵��: ���ض���
*	��    ��: ��
*	�� �� ֵ: ��
*********************************************************************************************************
*/
float bsp_AngleRead(void)
{
	return bsp_IMU_GetData(YAW);
}

/*
*********************************************************************************************************
*	�� �� ��: bsp_AngleReadRaw,(Ϊ�˼��ݾɰ������ӿڣ����ṩ�˺���)
*	����˵��: ���ض�����ԭʼ������ʵ�ʶ�����100��
*	��    ��: ��
*	�� �� ֵ: ��
*********************************************************************************************************
*/
int16_t bsp_AngleReadRaw(void)
{
	return bsp_IMU_GetData(YAW)*100;
}

/*
*********************************************************************************************************
*	�� �� ��: bsp_IMU_GetData
*	����˵��: ������Ҫ������
*	��    ��: ��
*	�� �� ֵ: ��
*********************************************************************************************************
*/
float bsp_IMU_GetData(IMU_DATA_TYPE type)
{
	float data = 0.0F;
	
	switch(type)
	{
		case ACC_X:
		{
			data = angle.accX;
		}break;
		
		case ACC_Y:
		{
			data = angle.accY;
		}break;
		
		case ACC_Z:
		{
			data = angle.accZ;
		}break;
		
		case PITCH:
		{
			data = angle.pitch;
		}break;
		
		case ROLL:
		{
			data = angle.roll;
		}break;
		
		case YAW:
		{
			data = angle.yaw;
		}break;
	}
	
	return data;
}

/*
*********************************************************************************************************
*	�� �� ��: bsp_InitAngle
*	����˵��: ��ʼ��������ģ��
*	��    ��: ��
*	�� �� ֵ: ��
*********************************************************************************************************
*/
void bsp_InitAngle(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;

	/* ��GPIOʱ�� */
	RCC_APB2PeriphClockCmd(RCC_ALL_ANGLE, ENABLE);

	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_OD;	
	
	GPIO_InitStructure.GPIO_Pin = GPIO_PIN_RST;
	GPIO_Init(GPIO_PORT_RST, &GPIO_InitStructure);

}

/*
*********************************************************************************************************
*	�� �� ��: bsp_AngleRst
*	����˵��: ��λ������ģ��
*	��    ��: ��
*	�� �� ֵ: ��
*********************************************************************************************************
*/
void bsp_AngleRst(void)
{
	GPIO_SetBits(GPIO_PORT_RST,GPIO_PIN_RST);
	bsp_DelayMS(10);
	GPIO_ResetBits(GPIO_PORT_RST,GPIO_PIN_RST);
	bsp_DelayMS(10);
	GPIO_SetBits(GPIO_PORT_RST,GPIO_PIN_RST);
}



/*
*********************************************************************************************************
*	�� �� ��: bsp_AngleRevByte
*	����˵��: ���յ������Ƿ������������ˣ�һ���ֽ�
*	��    ��: ��
*	�� �� ֵ: ��
*********************************************************************************************************
*/
void bsp_AngleRevByte(uint8_t byte)
{
	/*
		3.5���ַ���ʱ������ֻ������RTUģʽ���棬��ΪRTUģʽû�п�ʼ���ͽ�������
		�������ݰ�֮��ֻ�ܿ�ʱ���������֣�Modbus�����ڲ�ͬ�Ĳ������£����ʱ���ǲ�һ���ģ�
		���Ծ���3.5���ַ���ʱ�䣬�����ʸߣ����ʱ������С�������ʵͣ����ʱ������Ӧ�ʹ�

		4800  = 7.297ms
		9600  = 3.646ms
		19200  = 1.771ms
		38400  = 0.885ms
	*/
	uint32_t timeout;

	angle.timeout = false;
	
	timeout = 35000000 / RX_BAUD;		/* ���㳬ʱʱ�䣬��λus 35000000*/
	
	/* Ӳ����ʱ�жϣ���ʱ����us*/
	bsp_StartHardTimer(3, timeout, (void *)bsp_AngleTimeout);

	if (angle.rxCount < RX_BUF_SIZE)
	{
		angle.buf[angle.rxCount++] = byte;
	}
}

/*
*********************************************************************************************************
*	�� �� ��: bsp_AngleTimeout
*	����˵��: ����3.5���ֽ���Ϊ��1֡����
*	��    ��: ��
*	�� �� ֵ: ��
*********************************************************************************************************
*/
static void bsp_AngleTimeout(void)
{
	angle.timeout = true;
	
	bsp_AnglePoll();
}


/*
*********************************************************************************************************
*	�� �� ��: bsp_AnglePoll
*	����˵��: У�飬����
*	��    ��: ��
*	�� �� ֵ: ��
*********************************************************************************************************
*/
static void bsp_AnglePoll(void)
{	
	if (angle.timeout == false)	/* ����3.5���ַ�ʱ���ִ��MODH_RxTimeOut()������ */
	{
		/* û�г�ʱ���������ա���Ҫ����  */
		return ;
	}

	/* ��ʱ����  */
	angle.timeout = false;

	if (angle.rxCount != SIZE_WITH_ROLL_PITCH)
	{
		WARNING("angle len err:%d\r\n",angle.rxCount);
		goto err_ret;
	}
	
	/* ����SUMУ��� */
	if(bsp_AngleCheck() == false)
	{
		WARNING("angle chk err:\r\n");
		goto err_ret;
	}
	
	/* ����Ӧ�ò�Э�� */
	bsp_AngleAnalyzeApp();

err_ret:
	angle.rxCount = 0;	/* ��������������������´�֡ͬ�� */
}

/*
*********************************************************************************************************
*	�� �� ��: bsp_AngleCheck
*	����˵��: У��������֡
*	��    ��: ��
*	�� �� ֵ: ��
*********************************************************************************************************
*/
static bool bsp_AngleCheck(void)
{
	/*HEAD*/
	if( angle.buf[0] != 0xA5 || 
		angle.buf[1] != 0xA5 )
	{
		return false;
	}
	
    angle.temp_angular_velocity  =  angle.buf[9] << 8  | angle.buf[8] ;
    angle.temp_yaw               =  angle.buf[11] << 8 | angle.buf[10] ;
    angle.temp_pitch             =  angle.buf[13] << 8 | angle.buf[12] ;
    angle.temp_roll              =  angle.buf[15] << 8 | angle.buf[14] ;
    angle.temp_chk               =  angle.buf[17] << 8 | angle.buf[16] ;

	if((uint16_t)(angle.temp_angular_velocity+angle.temp_yaw+angle.temp_pitch+angle.temp_roll) != angle.temp_chk)
	{
		return false;
	}
	
	return true;
}	

/*
*********************************************************************************************************
*	�� �� ��: bsp_AngleAnalyzeApp
*	����˵��: �õ���ʵ�ĽǶ�
*	��    ��: ��
*	�� �� ֵ: ��
*********************************************************************************************************
*/
static void bsp_AngleAnalyzeApp(void)
{	
	int16_t val = 0;
	
	/*���ٶ�X Y Z*/
	val = angle.buf[3] << 24 | angle.buf[2];
	angle.accX = val / 1024.0F;
	
	val = angle.buf[5] << 24 | angle.buf[4];
	angle.accY = val / 1024.0F;
	
	val = angle.buf[7] << 24 | angle.buf[6];
	angle.accZ = val / 1024.0F;
	
	
	/*�Ƕ�X Y Z*/
	angle.yaw   =  (int16_t)(angle.buf[11] << 8 | angle.buf[10])  / 100.0F;
    angle.pitch =  (int16_t)(angle.buf[13] << 8 | angle.buf[12])  / 100.0F;
    angle.roll  =  (int16_t)(angle.buf[15] << 8 | angle.buf[14])  / 100.0F;
	
}



/*
*********************************************************************************************************
*	�� �� ��: bsp_AngleAdd
*	����˵��: ����ת����ĽǶ�
*	��    ��: angle1��-180~180����angle2��-180~180��
*	�� �� ֵ: ת����ĽǶ�
*********************************************************************************************************
*/
float bsp_AngleAdd(float angle1 , float angle2)
{
	float ret = 0.0F;
	
	ret = angle1 + angle2;
	
	if(angle1>=0.0F)
	{
		if(ret >= 0.0F && ret <= 180.0F)
		{
			return ret ;
		}
		else if(ret > 180.0F)
		{
			return -(180 - (ret-180.0F));
		}
		else
		{
			return ret ;
		}
	}
	else
	{
		if(ret < 0.0F && ret > -180.0F)
		{
			return ret ;
		}
		else if(ret < -180.0F)
		{
			return (180-(-ret-180));
		}
		else
		{
			return ret ;
		}
	}
}

