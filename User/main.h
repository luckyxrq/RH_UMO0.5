#define BRUSHMOTORRPM  					2000//660
#define ROLLMOTORRPM   					2000
#define MOTORSPEED                      200

#define ROLLER_MOTOR_MAX_ADC_VALUE 		4000   //
#define	BRUSH_MOTOR_MAX_ADC_VALUE 		4000   //the true value wait for debug
#define	VACUUM_MOTOR_MAX_ADC_VALUE 		4000   //the true value wait for debug
#define ROBOT_BATTERY_MIN_ADC_VALUE     2000   //wait to debug??????????????
#define WHEEL_MOTOR_MAX_ADC_VALUE 		4000   //

#define POWERON 						0
#define POWEROFF 						-1

#define MAXCLIFFADCDT                   3000//1000 //700

#define ROBOT_STATE_INIT       0x00
#define	ROBOT_STATE_STANDBY    0x01
#define	ROBOT_STATE_WORKING    0x02
#define	ROBOT_STATE_SUSPEND    0x03
#define	ROBOT_STATE_CHARGING   0x04
#define ROBOT_STATE_DEFAULT    0xFF

#define ROBOT_WORKWAY_HOME     0x01
#define ROBOT_WORKWAY_CLEAN    0x02
#define ROBOT_WORKWAY_CHARGE   0x03
#define ROBOT_WORKWAY_DEFAULT  0xFF

#define	ROBOT_CONTROL_KEY        0x10
#define	ROBOT_CONTROL_APP        0x20
#define	ROBOT_CONTROL_BOT3       0x30

#define ROBOT_ERROR_NUM_CLIF          0x00
#define	ROBOT_ERROR_NUM_LEFT_WHEEL    0x01
#define	ROBOT_ERROR_NUM_RIGHT_WHEEL   0x02
#define	ROBOT_ERROR_NUM_ROLLER_MOTOR  0x03
#define	ROBOT_ERROR_NUM_BRUSH_MOTOR   0x04
#define ROBOT_ERROR_NUM_VACUUM_MOTOR  0x05
#define ROBOT_ERROR_NUM_OFFLANDR      0x06
#define ROBOT_ERROR_NUM_OFFLANDL      0x07
#define ROBOT_ERROR_NUM_BATTERY       0x08
#define ROBOT_ERROR_NUM_BUMPER        0x09
#define ROBOT_ERROR_NUM_DUST_HALL     0x0A
#define ROBOT_ERROR_NUM_DEFAULT       0xFF


