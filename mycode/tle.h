#ifndef __TLE_H__
#define __TLE_H__

#include "main.h"

#define ENCODER_CPR   32767
#define VEL_VEC_SIZE	40

/// TLE5012 ///
#define READ_STATUS 0x8001
#define READ_ANGLE_VALUE 0x8021
#define READ_SPEED_VALUE 0x8031

typedef struct sEncoder {

	float position;
	float position_last;
	int turns;
	int cnt;
	int cnt_offset;
	int cnt_last;
	float angle;
	float elec_angle;
	float velocity;
	float velocity_elec;
	float vel_tle;
	float velocity_last;
	float vel_tle_last;
	
	float vel_vec[VEL_VEC_SIZE];

} tEncoder;

extern tEncoder Encoder;

uint16_t SPI2_ReadWriteByte(uint16_t byte);
uint16_t tle_read_reg(uint16_t READ_RES);
float tle_read_angle(void);
float tle_read_speed(float dt);
void Encoder_Sample(float dt);

#endif
