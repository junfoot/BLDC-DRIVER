#ifndef __PARA_H__
#define __PARA_H__

#include "main.h"
#include <stdbool.h>

#define SHUNT_RESISTENCE	0.001f								// Resistance of phase current sampling resistor
#define V_SCALE (19.0f * 3.3f / 4096.0f)     					// Bus volts per A/D Count (0.015311 V)
#define I_SCALE (3.3f / 4096.0f) / SHUNT_RESISTENCE / 40.0f		// Amps per A/D Count (0.02014 A)

#define PWM_ARR		3400           	// 25KHz
#define DT			0.00004f		// 40us
#define CURRENT_MEAS_HZ		(1.0f / DT)

#define OFFSET_LUT_NUM		128
#define COGGING_MAP_NUM		2048

typedef struct sUsrConfig
{
	int order;
	
	int input_mode;
	int control_mode;
	
	// Motor
	int pole_pairs;				// (Auto)
	float phase_resistance;		// (Auto)
	float phase_inductance;		// (Auto)
	float inertia;				// [Nm/(turn/s^2)]
	float torque_constant;		// [Nm/A]

	float current_ctrl_p_gain;	// (Auto)
	float current_ctrl_i_gain;	// (Auto)
	
	float vel_gain;
	float pos_gain;
	float vel_integrator_gain;

	float torque_ramp_rate;
	float vel_ramp_rate;
	
	int gain_scheduling_enable;
	float gain_scheduling_width;
	float input_pos_filter_bandwidth;
	
	float traj_vel;			// [turn/s]
	float traj_accel;		// [(turn/s)/s]
	float traj_decel;		// [(turn/s)/s]
	
	// Protect
	float protect_under_voltage;
	float protect_over_voltage;
	float protect_over_speed;
	
	// Encoder
	int encoder_dir_rev;					// (Auto)
	int encoder_offset;						// (Auto)
	int16_t offset_lut[OFFSET_LUT_NUM];		// (Auto)
	
	// Calib
	int calib_valid;
	float calib_current;
	float calib_max_voltage;
	
	// Anti cogging
	int anticogging_enable;
	float anticogging_pos_threshold;
	float anticogging_vel_threshold;
	
	float vel_limit;
	float current_limit;
	
	int current_ctrl_bandwidth; // Current loop bandwidth 100~2000
	
	uint32_t crc;
	
	float vel_P;
	float vel_I;
	float pos_P;
	float pos_I;
	float pos_D;

	float vel_integral_prev;
	float vel_error_prev;
	float pos_integral_prev;
	float pos_error_prev;

}UsrConfig;

extern UsrConfig Usr;
void set_config(void);

typedef struct sCoggingMap{
	float map[COGGING_MAP_NUM];
	
	// Pad to 64 bit
	uint32_t _pad_0;
	
	uint32_t crc;
} tCoggingMap;

extern tCoggingMap *pCoggingMap;

void USR_CONFIG_set_default_config(void);
void USR_CONFIG_set_default_cogging_map(void);

int USR_CONFIG_read_config(void);
int USR_CONFIG_save_config(void);
int USR_CONFIG_read_cogging_map(void);
int USR_CONFIG_save_cogging_map(void);

#endif
