#ifndef __BSP_COMMUNICATION_H
#define __BSP_COMMUNICATION_H



#define M_PI 					3.141592654
#define WHEEL_LENGTH  			0.23f

#define  BB16_SOF_1				0		// ֡ͷ1
#define  BB16_SOF_2				1		// ֡ͷ2
#define  BB16_SOF_3				2		// ֡ͷ3
#define  BB16_MESSAGE_ID 		3		// ��ϢID
#define  BB16_LENGHT			4		// ���ݳ��� 
#define  BB16_PAYLOAD			5		// ���ݸ���
#define  BB16_CKB				6		// У��B
#define  BB16_CKA				7		// У��A
#define  BB16_END				8		// ֡β

#define  CMD_ID_SPEED 	        0x25	// 
#define  CMD_ID_DISTANCE  		0x35	// 
#define  CMD_ID_ANGLE  			0x45	// 

#define ROBOT_ERROR_NUM_CLIF          0xE0
#define	ROBOT_ERROR_NUM_LEFT_WHEEL    0xE1
#define	ROBOT_ERROR_NUM_RIGHT_WHEEL   0xE2
#define	ROBOT_ERROR_NUM_ROLLER_MOTOR  0xE3
#define	ROBOT_ERROR_NUM_BRUSH_MOTOR   0xE4
#define ROBOT_ERROR_NUM_VACUUM_MOTOR  0xE5
#define ROBOT_ERROR_NUM_OFFLANDR      0xE6
#define ROBOT_ERROR_NUM_OFFLANDL      0xE7
#define ROBOT_ERROR_NUM_BATTERY       0xE8
#define ROBOT_ERROR_NUM_BUMPER        0xE9
#define ROBOT_ERROR_NUM_DUST_HALL     0xEA
#define ROBOT_ERROR_NUM_DEFAULT       0xEF


//robot status 
#define ROBTO_STATE_INIT       0xA0
#define	ROBOT_STATE_STANDBY    0xA1
#define	ROBOT_STATE_WORKING    0xA2
#define	ROBOT_STATE_SUSPEND    0xA3
#define	ROBOT_STATE_CHARGING   0xA4
#define ROBOT_STATE_DEFAULT    0xAF

//robot work way
#define ROBOT_WORKWAY_HOME     0xB1
#define ROBOT_WORKWAY_CLEAN    0xB2
#define ROBOT_WORKWAY_CHARGE   0xB3
#define ROBOT_WORKWAY_DEFAULT  0xBF

//robot control way
#define	ROBOT_CONTROL_KEY        0xC1
#define	ROBOT_CONTROL_APP        0xC2
#define	ROBOT_CONTROL_BOT3       0xC3
#define	ROBOT_CONTROL_DEFAULT    0xCF



void bsp_SendReportFrame(void);
uint8_t bsp_ReveiceCmdFrame(int16_t* left_velocity,int16_t* right_velocity);

extern double DegToRad(double deg);
extern double RadToDeg(double rad);



#endif



//void decision_task(void *pdata) //Decision//             According the user operating or default program 	
//{	
//	u8 i=0;
//	u16 cnt,clif_right =0,clif_left =0,clif_middle =0;
//	u16 clifadc16[10],clifadc17[10],clifadc37[10];
//	u16 refadc16,refadc17,refadc37;
//	u16 refadcdt;
//	
//#ifdef COMMENT
//{
//	// ��ѭ��
//		//a ��ȡ������Ϣ��־λ
//	
//		//b ���»���״̬����������
//		    //0��ʼ��
//				//a ���������´��������ݲ��궨
//				//b ��������ײ�����������Ƿ�����
//			//1��ͣ
//				//a �ر����е����Դ
//				//b ������ͣʱ�䣬��ʱ��������״̬
//			//2����
//				//a ��ȡ����״̬
//			    //b ��ȡ��ؿ���״̬
//				//c ��ȡ�������
//				//d ��ȡ���´���������
//				//e ��ȡ����Թ���ײ�ر�����
//				//f	��ȡ��ص�ѹ����
//				//g �����������������ݲ�����ɨ����ײ ���ߵ��滮 ��BOT3��
//				//h �����������쳣������������������ͣ״̬
//			//3����
//				//a �͹���ģʽ
//			//4�س�
//			    //a �ر���ɨ�������
//				//b ��ȡ��ؿ���״̬
//				//c ��ȡ�˶��������
//				//d ��ȡ���´���������
//				//e ��ȡ����Թ���ײ�ر�����
//				//f ��ȡ���Ժ����������
//				//g ��ȡ���״̬
//				//h �����������������ݲ��Իس䣨������BOT3��
//				//i �����������쳣������������������ͣ״̬
//}			
//#endif

//	
//	while(1)
//	{	
//		last_robot_state = cur_robot_state;
////*****************************************************************************************************//		
//		if(home_key_down_flag == 1)
//		{
//			if(last_robot_state == ROBOT_STATE_DEFAULT)       cur_robot_state = ROBTO_STATE_INIT;
//			else if(last_robot_state == ROBOT_STATE_WORKING)  cur_robot_state = ROBOT_STATE_SUSPEND;
//			else if(last_robot_state == ROBOT_STATE_CHARGING) cur_robot_state = ROBOT_STATE_SUSPEND;
//			else if(last_robot_state == ROBOT_STATE_SUSPEND)  cur_robot_state = ROBOT_STATE_WORKING;
//			else if(last_robot_state == ROBOT_STATE_STANDBY)  cur_robot_state = ROBOT_STATE_WORKING;
//			
//			robot_work_way  = ROBOT_WORKWAY_HOME;
//		}
//		else if(charge_key_down_flag == 1)
//		{
//			if(last_robot_state == ROBOT_STATE_DEFAULT)       cur_robot_state = ROBTO_STATE_INIT;
//			else if(last_robot_state == ROBOT_STATE_WORKING)  cur_robot_state = ROBOT_STATE_CHARGING;
//			else if(last_robot_state == ROBOT_STATE_CHARGING) cur_robot_state = ROBOT_STATE_SUSPEND;
//			else if(last_robot_state == ROBOT_STATE_SUSPEND)  cur_robot_state = ROBOT_STATE_CHARGING;
//			else if(last_robot_state == ROBOT_STATE_STANDBY)  cur_robot_state = ROBOT_STATE_CHARGING;
//			
//			robot_work_way  = ROBOT_WORKWAY_CHARGE;
//		}
//		else if(clean_key_down_flag == 1)
//		{
//			if(last_robot_state == ROBOT_STATE_DEFAULT)       cur_robot_state = ROBTO_STATE_INIT;
//			else if(last_robot_state == ROBOT_STATE_WORKING)  cur_robot_state = ROBOT_STATE_SUSPEND;
//			else if(last_robot_state == ROBOT_STATE_CHARGING) cur_robot_state = ROBOT_STATE_SUSPEND;
//			else if(last_robot_state == ROBOT_STATE_SUSPEND)  cur_robot_state = ROBOT_STATE_WORKING;
//			else if(last_robot_state == ROBOT_STATE_STANDBY)  cur_robot_state = ROBOT_STATE_WORKING;
//			
//			robot_work_way  = ROBOT_WORKWAY_CLEAN;
//		}
////*****************************************************************************************************//
//		
//		
////*****************************************************************************************************//		
//		if(cur_robot_state == ROBTO_STATE_INIT)
//		{
//			for(i=0;i<5;i++)
//			{
//				clifadc16[i] = Get_Adc(6);   //middle
//				clifadc17[i] = Get_Adc(7);   //left
//				clifadc37[i] = Get_Adc3(7);  //right
//				delay_ms(10);
//				refadc16+=clifadc16[i];
//				refadc17+=clifadc17[i];
//				refadc37+=clifadc37[i];
//			}
//			
//			refadc16/=5; 
//			refadc17/=5; 
//			refadc37/=5; 
//			refadcdt = (refadc16+refadc17+refadc37)/3;	
//		//	printf("%d,%d,%d\n",refadc16,refadc17,refadc37);		
//		//	printf("refadcdt:%d, %d,%d,%d\n",refadcdt,refadc16-refadcdt,refadc17-refadcdt,refadc37-refadcdt);
//			
//			if(((refadcdt>refadc16)?(refadcdt-refadc16):(refadc16-refadcdt)) > MAXCLIFFADCDT )
//			{
//				//	printf("��������´�����");
//				cur_robot_state = ROBOT_STATE_DEFAULT;
//				robot_error_num = ROBOT_ERROR_NUM_CLIF;
//				continue;
//			}
//			if(((refadcdt>refadc17)?(refadcdt-refadc17):(refadc17-refadcdt)) > MAXCLIFFADCDT )
//			{
//				//	printf("��������´�����");
//				cur_robot_state = ROBOT_STATE_DEFAULT;
//				robot_error_num = ROBOT_ERROR_NUM_CLIF;				
//				continue;
//			}
//			if(((refadcdt>refadc37)?(refadcdt-refadc37):(refadc37-refadcdt)) > MAXCLIFFADCDT )
//			{
//				//	printf("��������´�����");
//				cur_robot_state = ROBOT_STATE_DEFAULT;
//				robot_error_num = ROBOT_ERROR_NUM_CLIF;
//				continue;
//			}
//			
//			
//			cur_robot_state = (robot_work_way == ROBOT_WORKWAY_CHARGE)? ROBOT_STATE_CHARGING:ROBOT_STATE_WORKING;
//			
//			if(robot_work_way==ROBOT_WORKWAY_CLEAN || robot_work_way==ROBOT_WORKWAY_HOME)
//			{
//				TIM_Cmd(TIM4, ENABLE); 
//				RollerMotorSetRPM(ROLLMOTORRPM);
//				delay_ms(500);
//				BrushMotorSetRPM(BRUSHMOTORRPM);
//				delay_ms(500);
//				VacuumMotorSet(ENABLE);
//				delay_ms(1000);
//			}
//			TIM_Cmd(TIM1, ENABLE);
//			WheelBrake();
//			
//			
//		}
////*****************************************************************************************************//
//		else if(cur_robot_state == ROBOT_STATE_STANDBY)
//		{
//			//do nothing
//		}
////*****************************************************************************************************//
//		else if(cur_robot_state == ROBOT_STATE_WORKING)
//		{
//			//a ��ȡ����״̬
//			//b ��ȡ��ؿ���״̬
//			//c ��ȡ�������
//			//d ��ȡ��ص�ѹ����
//			//e ��ȡ����Թ���ײ�ر�����
//			//f	��ȡ���´���������
//			//g �����������쳣������������������ͣ״̬
//			//h �����������������ݲ�����ɨ����ײ ���ߵ��滮 ��BOT3��
//			
//			if(HALL_IN) 
//			{
//				cur_robot_state = ROBOT_STATE_SUSPEND;
//				robot_error_num = ROBOT_ERROR_NUM_DUST_HALL;
//				continue;
//			}
//			if(OFF_LANDR_KEY)
//			{
//				cur_robot_state = ROBOT_STATE_SUSPEND;
//				robot_error_num = ROBOT_ERROR_NUM_OFFLANDR;
//				continue;
//			}
//			if(OFF_LANDL_KEY)
//			{
//				cur_robot_state = ROBOT_STATE_SUSPEND;
//				robot_error_num = ROBOT_ERROR_NUM_OFFLANDL;
//				continue;
//			}
//			//get roller motor adc value
//			roller_motor_adc_value = GetRollerMotorAdcValue();
//			if(roller_motor_adc_value >ROLLER_MOTOR_MAX_ADC_VALUE)
//			{
//				cur_robot_state = ROBOT_STATE_SUSPEND;
//				robot_error_num = ROBOT_ERROR_NUM_ROLLER_MOTOR;
//				continue;
//			}
//			
//			//get brush motor adc vaule
//			brush_motor_adc_value = GetBrushMotorAdcValue();
//			if(brush_motor_adc_value >BRUSH_MOTOR_MAX_ADC_VALUE)
//			{
//				cur_robot_state = ROBOT_STATE_SUSPEND;
//				robot_error_num = ROBOT_ERROR_NUM_BRUSH_MOTOR;
//				continue;
//			}
//				
//			//get vacuum motor adc value
//			vacuum_motor_adc_value = GetVacuumMotorAdcValue();
//			if(vacuum_motor_adc_value >VACUUM_MOTOR_MAX_ADC_VALUE)
//			{
//				cur_robot_state = ROBOT_STATE_SUSPEND;
//				robot_error_num = ROBOT_ERROR_NUM_VACUUM_MOTOR;
//				continue;
//			}
//			//get robot battery adc value
//			robot_battery_adc_value = 2000; //? wait for debug
//			if(robot_battery_adc_value<ROBOT_BATTERY_MIN_ADC_VALUE)
//			{
//				cur_robot_state = ROBOT_STATE_CHARGING;
//				robot_error_num = ROBOT_ERROR_NUM_BATTERY;
//				continue;
//			}
//			//get left wheel adc value
//			wheel_left_motor_adc_value = Get_Adc(ADC_Channel_9);
//			if(wheel_left_motor_adc_value>WHEEL_MOTOR_MAX_ADC_VALUE)
//			{
//				cur_robot_state = ROBOT_STATE_SUSPEND;
//				robot_error_num = ROBOT_ERROR_NUM_LEFT_WHEEL;
//				continue;
//			}
//			//get right wheel adc value
//			wheel_right_motor_adc_value = Get_Adc3(ADC_Channel_8);
//			if(wheel_right_motor_adc_value>WHEEL_MOTOR_MAX_ADC_VALUE)
//			{
//				cur_robot_state = ROBOT_STATE_SUSPEND;
//				robot_error_num = ROBOT_ERROR_NUM_RIGHT_WHEEL;
//				continue;
//			}
//	
//			
//			
//			
//			//get cliff infrared data
//			if((refadc16 - Get_Adc(6))>1500) clif_middle++;
//			if((refadc17 - Get_Adc(7))>1500) clif_left++;
//			if((refadc37 - Get_Adc3(7))>1500) clif_right++;	
//			//get  collision infrared data
//			
//			//get bumper data
//			if(robot_work_way  == ROBOT_WORKWAY_HOME)
//			{
//				if(IR_CLIFFS_RX3)
//				{
//					cmd_velocity_right = -VELOCITY;
//					cmd_velocity_left = -VELOCITY;
//					auto_charge_station_flag = 1;
//					delay_ms(500);
//					cmd_velocity_right = VELOCITY;
//					cmd_velocity_left = -VELOCITY;
//					auto_charge_station_flag = 1;
//					//RobotPointTurnAround(motor_cmd.speed,CCLOCK);
//					delay_ms(800);
//				}
//				else if(IR_CLIFFS_RX6)
//				{
//					cmd_velocity_right = -VELOCITY;
//					cmd_velocity_left = -VELOCITY;
//					auto_charge_station_flag = 1;
//					delay_ms(500);
//					cmd_velocity_right = -VELOCITY;
//					cmd_velocity_left = VELOCITY;
//					auto_charge_station_flag = 1;
//					//RobotPointTurnAround(motor_cmd.speed,CLOCK);
//					delay_ms(800);	
//				}
//				else if(clif_right>2 )
//				{
//					clif_right = 0;
//					cmd_velocity_right = -VELOCITY;
//					cmd_velocity_left = -VELOCITY;
//					auto_charge_station_flag = 1;
//					delay_ms(500);
//					cmd_velocity_right = VELOCITY;
//					cmd_velocity_left = -VELOCITY;
//					auto_charge_station_flag = 1;
//					//RobotPointTurnAround(motor_cmd.speed,CCLOCK);
//					delay_ms(800);
//					
//				}
//				else if(clif_left>2)
//				{
//					clif_left = 0;
//					cmd_velocity_right = -VELOCITY;
//					cmd_velocity_left = -VELOCITY;
//					auto_charge_station_flag = 1;
//					//RobotGoStraight(-motor_cmd.speed);
//					delay_ms(500);
//					cmd_velocity_right = -VELOCITY;
//					cmd_velocity_left = VELOCITY;
//					auto_charge_station_flag = 1;
//					//RobotPointTurnAround(motor_cmd.speed,CLOCK);
//					delay_ms(800);	
//				}
//				else if(clif_middle>2)
//				{
//					clif_middle = 0;
//					cmd_velocity_right = -VELOCITY;
//					cmd_velocity_left = -VELOCITY;
//					auto_charge_station_flag = 1;
//					//RobotGoStraight(-motor_cmd.speed);
//					delay_ms(500);
//					cmd_velocity_right = -VELOCITY;
//					cmd_velocity_left = VELOCITY;
//					auto_charge_station_flag = 1;
//					//RobotPointTurnAround(motor_cmd.speed,CLOCK);
//					delay_ms(800);	
//				}
//				else
//				{
//					cmd_velocity_right = VELOCITY;
//					cmd_velocity_left = VELOCITY;
//					auto_charge_station_flag = 1;
//				}
//				
//			}
//			
//			if(robot_work_way  == ROBOT_WORKWAY_CLEAN)
//			{
//				
//			}
//			
//				
//		}
////*****************************************************************************************************//
//		else if(cur_robot_state == ROBOT_STATE_SUSPEND)
//		{
//			TIM_Cmd(TIM1, ENABLE);
//			TIM_Cmd(TIM4, ENABLE);			
//			WheelBrake();
//			RobotGoStraight(0);
//			RollerMotorSetRPM(0);
//			BrushMotorSetRPM(0);
//			VacuumMotorSet(DISABLE);
//			
//			switch(robot_error_num)
//			{
//				case ROBOT_ERROR_NUM_CLIF : 
//					break;
//				case ROBOT_ERROR_NUM_LEFT_WHEEL : 
//					break;	
//				case ROBOT_ERROR_NUM_RIGHT_WHEEL : 
//					break;	
//				case ROBOT_ERROR_NUM_ROLLER_MOTOR : 
//					break;	
//				case ROBOT_ERROR_NUM_BRUSH_MOTOR :
//					break;    
//				case ROBOT_ERROR_NUM_VACUUM_MOTOR : 
//					break;	
//				case ROBOT_ERROR_NUM_OFFLANDR : 
//					break;     
//				case ROBOT_ERROR_NUM_OFFLANDL :
//					break;	
//				case ROBOT_ERROR_NUM_BATTERY :
//					break;
//				case ROBOT_ERROR_NUM_BUMPER :
//					break;	
//				case ROBOT_ERROR_NUM_DUST_HALL :
//					break;	
//				case ROBOT_ERROR_NUM_DEFAULT :
//					break;	
//				default:
//					break;
//				
//			}
//			robot_error_num = ROBOT_ERROR_NUM_DEFAULT;
//			cur_robot_state = ROBOT_STATE_STANDBY;
//		}
////*****************************************************************************************************//
//		else if(cur_robot_state == ROBOT_STATE_CHARGING)
//		{
//			WheelBrake();
//			RollerMotorSetRPM(0);
//			BrushMotorSetRPM(0);
//			VacuumMotorSet(DISABLE);
//			
//			//printf("AutogoChargeStation!!!! \n");
//			NewChargeWay(1);
//			
//		}
////*****************************************************************************************************//
//				
//	}
//}
















