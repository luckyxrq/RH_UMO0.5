#include "bsp.h"

#define CALIBRATION_SIZE      8

static Angle angle;
static uint8_t CalibrationBuf[CALIBRATION_SIZE] = {0x41,0x78,0xFF,0x06,0x02,0x00,0xC2,0x6D};

static void bsp_AngleTimeout(void);
static void bsp_AnglePoll(void);
static void bsp_AngleAnalyzeApp(void);
static bool bsp_AngleCheck(void);
static uint8_t bsp_AngleChkXorCalc(uint8_t buf[] , uint8_t len);


/* �����ڶ�Ӧ��RCCʱ�� */
#define RCC_ALL_ANGLE 	(RCC_APB2Periph_GPIOB)

#define GPIO_PORT_RST    GPIOB
#define GPIO_PIN_RST	 GPIO_Pin_13

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
	uint8_t i = 0 ;
	uint8_t sum = 0 ;
	
	/*HEAD1*/
	if( angle.buf[POS_HEAD1_CHK + 0] != 0x41 || 
		angle.buf[POS_HEAD1_CHK + 1] != 0x78 || 
		angle.buf[POS_HEAD1_CHK + 2] != 0xFF || 
		angle.buf[POS_HEAD1_CHK + 3] != 0x06 || 
		angle.buf[POS_HEAD1_CHK + 4] != 0x81 || 
		angle.buf[POS_HEAD1_CHK + 5] != 0x47)
	{
		return false;
	}
	
	/*HEAD2*/
	if( angle.buf[POS_HEAD2_CHK + 0] != 0x00 || 
		angle.buf[POS_HEAD2_CHK + 1] != 0x8C || 
		angle.buf[POS_HEAD2_CHK + 2] != 0x0C)
	{
		return false;
	}
	
	/*HEAD3*/
	if( angle.buf[POS_HEAD3_CHK + 0] != 0x02 || 
		angle.buf[POS_HEAD3_CHK + 1] != 0x8C || 
		angle.buf[POS_HEAD3_CHK + 2] != 0x0C)
	{
		return false;
	}
	
	/*HEAD4*/
	if( angle.buf[POS_HEAD4_CHK + 0] != 0x01 || 
		angle.buf[POS_HEAD4_CHK + 1] != 0xB0 || 
		angle.buf[POS_HEAD4_CHK + 2] != 0x10)
	{
		return false;
	}
	
	/*TAIL*/
	if( angle.buf[POS_TAIL_CHK + 0] != 0x6D )
	{
		return false;
	}
	
	for(i=0;i<SIZE_WITH_ROLL_PITCH - 2 ;i++)
	{
		sum += angle.buf[i];
	}
	
	if(bsp_AngleChkXorCalc((uint8_t*)angle.buf,SIZE_WITH_ROLL_PITCH - 2) != angle.buf[POS_ANGLE_CHK])
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
	uint32_t val = 0;
	
	/*���ٶ�X Y Z*/
	val = angle.buf[12] << 24 | angle.buf[11] << 16 | angle.buf[10] << 8 | angle.buf[9];
	memcpy(&angle.accX , &val , 4);
	
	val = angle.buf[16] << 24 | angle.buf[15] << 16 | angle.buf[14] << 8 | angle.buf[13];
	memcpy(&angle.accY , &val , 4);
	
	val = angle.buf[20] << 24 | angle.buf[19] << 16 | angle.buf[18] << 8 | angle.buf[17];
	memcpy(&angle.accZ , &val , 4);
	
	
	/*�Ƕ�X Y Z*/
	val = angle.buf[57] << 24 | angle.buf[56] << 16 | angle.buf[55] << 8 | angle.buf[54];
	memcpy(&angle.roll , &val , 4);
	
	val = angle.buf[61] << 24 | angle.buf[60] << 16 | angle.buf[59] << 8 | angle.buf[58];
	memcpy(&angle.pitch , &val , 4);
	
	val = angle.buf[65] << 24 | angle.buf[64] << 16 | angle.buf[63] << 8 | angle.buf[62];
	memcpy(&angle.yaw , &val , 4);
	
}


static uint8_t bsp_AngleChkXorCalc(uint8_t buf[] , uint8_t len)
{
	uint8_t i = 0 ;
	uint8_t chk = 0 ;
	
	for(i=0;i<len;i++)
	{
		chk ^= buf[i];
	}
	
	return chk;
}


void bsp_IMU_Calibration(void)
{
	comSendBuf(COM2,CalibrationBuf,CALIBRATION_SIZE);
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
