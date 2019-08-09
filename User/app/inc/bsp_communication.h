#ifndef __BSP_COMMUNICATION_H
#define __BSP_COMMUNICATION_H

#pragma pack(1)
typedef struct
{
	uint8_t sof1;                  //0xAA
	uint8_t sof2;                  //0xAA
	uint8_t sof3;                  //0xAA
	uint8_t identifier;	           //0x25
	uint8_t size_of_payload_field; //0x04
	int32_t left_wheel_pulse_count;
	int32_t right_wheel_pulse_count;
	uint8_t button_control_cmd;
	uint16_t distance_of_left_infrared;
	uint16_t distance_of_right_infrared;
	uint16_t distance_of_front_infrared;
	int16_t  angle_deg;
	uint16_t adc_1;
	uint16_t adc_2;
	uint16_t adc_3;
	uint16_t adc_4;
	int16_t acc_x;
	int16_t acc_y;
	int16_t acc_z;
	uint16_t obstacle_signal;
	uint8_t battery_level_soc;
	uint8_t charge_status;
	uint8_t checksum_msb;
	uint8_t checksum_lsb;
	uint8_t end_of_falg;           //0x55
}ReportFrame;
#pragma pack()

void bsp_SendReportFrame(void);

#endif
