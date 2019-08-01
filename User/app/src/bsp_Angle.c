#include "bsp.h"



typedef struct
{
	volatile bool timeout ;
	volatile uint16_t rxCount;
	volatile uint8_t  buf[RX_BUF_SIZE] ;
	volatile float angle;
}Angle;


typedef struct
{
	volatile int16_t HEAD;
	volatile int16_t RATE;
	volatile int16_t ANGLE;
	volatile int16_t SUM1;
	volatile int16_t RESERVE;
	volatile int8_t SUM2;
	volatile int16_t END;
}AngleFrame;

static Angle angle;
static AngleFrame angleFrame;

static void bsp_AngleTimeout(void);
static void bsp_AnglePoll(void);
static void bsp_AngleAnalyzeApp(void);
static bool bsp_AngleCheck(void);

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



float bsp_AngleAdd(float angle1 , float angle2)
{
	if(angle1>=0 && angle1<180)
	{
		
	}
}
