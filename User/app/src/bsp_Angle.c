#include "bsp.h"





static Angle angle;
static AngleFrame angleFrame;

static void bsp_AngleTimeout(void);
static void bsp_AnglePoll(void);
static void bsp_AngleAnalyzeApp(void);
static bool bsp_AngleCheck(void);
static void bsp_AngleRst(void);



/* �����ڶ�Ӧ��RCCʱ�� */
#define RCC_ALL_ANGLE 	(RCC_APB2Periph_GPIOB)

#define GPIO_PORT_RST    GPIOB
#define GPIO_PIN_RST	 GPIO_Pin_13


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
	
	bsp_AngleRst();
}

/*
*********************************************************************************************************
*	�� �� ��: bsp_AngleRst
*	����˵��: ��λ������ģ��
*	��    ��: ��
*	�� �� ֵ: ��
*********************************************************************************************************
*/
static void bsp_AngleRst(void)
{
	GPIO_SetBits(GPIO_PORT_RST,GPIO_PIN_RST);
	bsp_DelayMS(2000);
	GPIO_ResetBits(GPIO_PORT_RST,GPIO_PIN_RST);
	bsp_DelayUS(10);
	GPIO_SetBits(GPIO_PORT_RST,GPIO_PIN_RST);
}


/*
*********************************************************************************************************
*	�� �� ��: bsp_AngleRead
*	����˵��: ���ض���
*	��    ��: ��
*	�� �� ֵ: ��
*********************************************************************************************************
*/
float bsp_AngleRead(void)
{
	return angle.angle;
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

	if (angle.rxCount != 13)
	{
		WARNING("���������ݳ��ȴ���:%d\r\n",angle.rxCount);
		goto err_ret;
	}
	
	/* ����SUMУ��� */
	if(bsp_AngleCheck() == false)
	{
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
	angleFrame.HEAD     = angle.buf[1] << 8  | angle.buf[0];
	angleFrame.RATE     = angle.buf[3] << 8  | angle.buf[2];
	angleFrame.ANGLE    = angle.buf[5] << 8  | angle.buf[4];
	angleFrame.SUM1     = angle.buf[7] << 8  | angle.buf[6];
	angleFrame.RESERVE  = angle.buf[9] << 8  | angle.buf[8];
	angleFrame.SUM2     = angle.buf[10];
	angleFrame.END      = angle.buf[12] << 8 | angle.buf[11];
	
	
	/*У��SUM1*/
	if((angleFrame.HEAD+angleFrame.RATE+angleFrame.ANGLE) != angleFrame.SUM1)
	{
		return false;
	}

	#if 0
	DEBUG("%X   %X\r\n",angleFrame.HEAD+angleFrame.RATE+angleFrame.ANGLE,angleFrame.SUM1);
	#endif
	
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
	int16_t ret = angle.buf[5] << 8 | angle.buf[4];
	angle.angle = ret / 100.0F;
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
