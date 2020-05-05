#include "bsp.h"

#define CALIBRATION_SIZE      8

static Angle angle;
static uint8_t CalibrationBuf[CALIBRATION_SIZE] = {0x41,0x78,0xFF,0x06,0x02,0x00,0xC2,0x6D};

static void bsp_AngleTimeout(void);
static void bsp_AnglePoll(void);
static void bsp_AngleAnalyzeApp(void);
static bool bsp_AngleCheck(void);
static uint8_t bsp_AngleChkXorCalc(uint8_t buf[] , uint8_t len);


/* 按键口对应的RCC时钟 */
#define RCC_ALL_ANGLE 	(RCC_APB2Periph_GPIOB)

#define GPIO_PORT_RST    GPIOB
#define GPIO_PIN_RST	 GPIO_Pin_13

/*
*********************************************************************************************************
*	函 数 名: bsp_AngleRead,(为了兼容旧版驱动接口，故提供此函数)
*	功能说明: 返回度数
*	形    参: 无
*	返 回 值: 无
*********************************************************************************************************
*/
float bsp_AngleRead(void)
{
	return bsp_IMU_GetData(YAW);
}

/*
*********************************************************************************************************
*	函 数 名: bsp_AngleReadRaw,(为了兼容旧版驱动接口，故提供此函数)
*	功能说明: 返回度数，原始度数是实际度数的100倍
*	形    参: 无
*	返 回 值: 无
*********************************************************************************************************
*/
int16_t bsp_AngleReadRaw(void)
{
	return bsp_IMU_GetData(YAW)*100;
}



/*
*********************************************************************************************************
*	函 数 名: bsp_IMU_GetData
*	功能说明: 返回需要的数据
*	形    参: 无
*	返 回 值: 无
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
*	函 数 名: bsp_InitAngle
*	功能说明: 初始化陀螺仪模块
*	形    参: 无
*	返 回 值: 无
*********************************************************************************************************
*/
void bsp_InitAngle(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;

	/* 打开GPIO时钟 */
	RCC_APB2PeriphClockCmd(RCC_ALL_ANGLE, ENABLE);

	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_OD;	
	
	GPIO_InitStructure.GPIO_Pin = GPIO_PIN_RST;
	GPIO_Init(GPIO_PORT_RST, &GPIO_InitStructure);

}

/*
*********************************************************************************************************
*	函 数 名: bsp_AngleRst
*	功能说明: 复位陀螺仪模块
*	形    参: 无
*	返 回 值: 无
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
*	函 数 名: bsp_AngleRevByte
*	功能说明: 接收到陀螺仪发过来的数据了，一个字节
*	形    参: 无
*	返 回 值: 无
*********************************************************************************************************
*/
void bsp_AngleRevByte(uint8_t byte)
{
	/*
		3.5个字符的时间间隔，只是用在RTU模式下面，因为RTU模式没有开始符和结束符，
		两个数据包之间只能靠时间间隔来区分，Modbus定义在不同的波特率下，间隔时间是不一样的，
		所以就是3.5个字符的时间，波特率高，这个时间间隔就小，波特率低，这个时间间隔相应就大

		4800  = 7.297ms
		9600  = 3.646ms
		19200  = 1.771ms
		38400  = 0.885ms
	*/
	uint32_t timeout;

	angle.timeout = false;
	
	timeout = 35000000 / RX_BAUD;		/* 计算超时时间，单位us 35000000*/
	
	/* 硬件定时中断，定时精度us*/
	bsp_StartHardTimer(3, timeout, (void *)bsp_AngleTimeout);

	if (angle.rxCount < RX_BUF_SIZE)
	{
		angle.buf[angle.rxCount++] = byte;
	}
}

/*
*********************************************************************************************************
*	函 数 名: bsp_AngleTimeout
*	功能说明: 超过3.5个字节认为是1帧数据
*	形    参: 无
*	返 回 值: 无
*********************************************************************************************************
*/
static void bsp_AngleTimeout(void)
{
	angle.timeout = true;
	
	bsp_AnglePoll();
}


/*
*********************************************************************************************************
*	函 数 名: bsp_AnglePoll
*	功能说明: 校验，解析
*	形    参: 无
*	返 回 值: 无
*********************************************************************************************************
*/
static void bsp_AnglePoll(void)
{	
	if (angle.timeout == false)	/* 超过3.5个字符时间后执行MODH_RxTimeOut()函数。 */
	{
		/* 没有超时，继续接收。不要清零  */
		return ;
	}

	/* 超时清零  */
	angle.timeout = false;

	if (angle.rxCount != SIZE_WITH_ROLL_PITCH)
	{
		WARNING("angle len err:%d\r\n",angle.rxCount);
		goto err_ret;
	}
	
	/* 计算SUM校验和 */
	if(bsp_AngleCheck() == false)
	{
		WARNING("angle chk err:\r\n");
		goto err_ret;
	}
	
	/* 分析应用层协议 */
	bsp_AngleAnalyzeApp();

err_ret:
	angle.rxCount = 0;	/* 必须清零计数器，方便下次帧同步 */
}

/*
*********************************************************************************************************
*	函 数 名: bsp_AngleCheck
*	功能说明: 校验陀螺仪帧
*	形    参: 无
*	返 回 值: 无
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
*	函 数 名: bsp_AngleAnalyzeApp
*	功能说明: 得到真实的角度
*	形    参: 无
*	返 回 值: 无
*********************************************************************************************************
*/
static void bsp_AngleAnalyzeApp(void)
{	
	uint32_t val = 0;
	
	/*加速度X Y Z*/
	val = angle.buf[12] << 24 | angle.buf[11] << 16 | angle.buf[10] << 8 | angle.buf[9];
	memcpy(&angle.accX , &val , 4);
	
	val = angle.buf[16] << 24 | angle.buf[15] << 16 | angle.buf[14] << 8 | angle.buf[13];
	memcpy(&angle.accY , &val , 4);
	
	val = angle.buf[20] << 24 | angle.buf[19] << 16 | angle.buf[18] << 8 | angle.buf[17];
	memcpy(&angle.accZ , &val , 4);
	
	
	/*角度X Y Z*/
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
*	函 数 名: bsp_AngleAdd
*	功能说明: 返回转动后的角度
*	形    参: angle1（-180~180），angle2（-180~180）
*	返 回 值: 转动后的角度
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
