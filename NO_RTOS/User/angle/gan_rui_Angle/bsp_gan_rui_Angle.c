#include "bsp.h"


#define CALIBRATION_SIZE      8

static Angle angle;

static void bsp_AngleTimeout(void);
static void bsp_AnglePoll(void);
static void bsp_AngleAnalyzeApp(void);
static bool bsp_AngleCheck(void);


/* 按键口对应的RCC时钟 */
#define RCC_ALL_ANGLE 	(RCC_APB2Periph_GPIOA)

#define GPIO_PORT_RST    GPIOA
#define GPIO_PIN_RST	 GPIO_Pin_1


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
*	函 数 名: bsp_AngleAnalyzeApp
*	功能说明: 得到真实的角度
*	形    参: 无
*	返 回 值: 无
*********************************************************************************************************
*/
static void bsp_AngleAnalyzeApp(void)
{	
	int16_t val = 0;
	
	/*加速度X Y Z*/
	val = angle.buf[3] << 24 | angle.buf[2];
	angle.accX = val / 1024.0F;
	
	val = angle.buf[5] << 24 | angle.buf[4];
	angle.accY = val / 1024.0F;
	
	val = angle.buf[7] << 24 | angle.buf[6];
	angle.accZ = val / 1024.0F;
	
	
	/*角度X Y Z*/
	angle.yaw   =  (int16_t)(angle.buf[11] << 8 | angle.buf[10])  / 100.0F;
    angle.pitch =  (int16_t)(angle.buf[13] << 8 | angle.buf[12])  / 100.0F;
    angle.roll  =  (int16_t)(angle.buf[15] << 8 | angle.buf[14])  / 100.0F;
	
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

