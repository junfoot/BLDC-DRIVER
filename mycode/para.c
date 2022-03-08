#include "para.h"
#include <string.h>
#include "util.h"
#include "control.h"
#include "heap.h"
#include "flash.h"
#include "util.h"
#include <stdio.h>

UsrConfig Usr;
tCoggingMap *pCoggingMap = NULL;

#define USR_CONFIG_COGGING_MAP_ROM_ADDR	((uint32_t)(0x8000000 + 55 * FLASH_PAGE_SIZE))	// Page 55
#define USR_CONFIG_ROM_ADDR				((uint32_t)(0x8000000 + 60 * FLASH_PAGE_SIZE))	// Page 60

void set_config(void)
{
	Usr.order = 1;
	printf("\n\rDRV8323\n\r");
	
	Usr.input_mode = INPUT_MODE_PASSTHROUGH;
	Usr.control_mode = CONTROL_MODE_POSITION_CONTROL;

// 	typedef enum {
// 	INPUT_MODE_PASSTHROUGH        = 0,
// 	INPUT_MODE_TORQUE_RAMP        = 1,
// 	INPUT_MODE_VEL_RAMP           = 2,
// 	INPUT_MODE_POS_FILTER         = 3,
// 	INPUT_MODE_TRAP_TRAJ          = 4,
// } tInputMode;

// typedef enum {
// 	CONTROL_MODE_TORQUE_CONTROL   = 0,
// 	CONTROL_MODE_VELOCITY_CONTROL = 1,
// 	CONTROL_MODE_POSITION_CONTROL = 2,
// } tControlMode;

	// Motor
	Usr.pole_pairs = 4;
	Usr.phase_resistance = 0.043198f;
	Usr.phase_inductance = 0.000007f;
	Usr.inertia = 0.001f;				// [Nm/(turn/s^2)]
	Usr.torque_constant = 0.06f;		// [Nm/A]

	// Control
	Usr.torque_ramp_rate = 0.01f;
	Usr.vel_ramp_rate = 50.0f;
	Usr.input_pos_filter_bandwidth = 10;
	Usr.gain_scheduling_enable = 0;
	Usr.gain_scheduling_width = 0.0001f;
	Usr.pos_gain = 260.0f;
	Usr.vel_gain = 0.15f;
	Usr.vel_integrator_gain = 0.02f;
	Usr.vel_limit = 90;
	Usr.current_limit = 20;
	
	Usr.protect_under_voltage = 12;
	Usr.protect_over_voltage  = 30;
	Usr.protect_over_speed = 100;
	
	Usr.traj_vel = 50;
	Usr.traj_accel = 100;
	Usr.traj_decel = 100;
	
	// Anticogging
	Usr.anticogging_enable = 0;
	Usr.anticogging_pos_threshold = 0.002f;
	Usr.anticogging_vel_threshold = 0.01f;
	
	// Encoder
	Usr.encoder_dir_rev = 0;
	
	// Calib
	Usr.calib_valid = 0;
	Usr.calib_current = 10.0f;
	Usr.calib_max_voltage = 5.0f;
	
	Usr.current_ctrl_bandwidth = 1000;
	
	Usr.current_ctrl_p_gain = 3;
	Usr.current_ctrl_i_gain = 100;
}



void USR_CONFIG_set_default_cogging_map(void)
{
	if(pCoggingMap == NULL){
		pCoggingMap = HEAP_malloc(sizeof(tCoggingMap));
	}
	
	for(int i=0; i<COGGING_MAP_NUM; i++){
		pCoggingMap->map[i] = 0;
	}
}

int USR_CONFIG_read_config(void)
{
	int state = 0;

	memcpy(&Usr, (uint8_t*)USR_CONFIG_ROM_ADDR, sizeof(UsrConfig));

	uint32_t crc;
	crc = crc32((uint8_t*)&Usr, sizeof(UsrConfig)-4, 0);
	if(crc != Usr.crc){
		state = -1;
	}
	
	return state;
}

int USR_CONFIG_save_config(void)
{
	int status = 0;
	
	uint32_t primask = cpu_enter_critical();
	
    FLASH_unlock();
	
	// Erase
	status = FLASH_erase_page(60, 1);
	
	if(status == 0){
		Usr.crc = crc32((uint8_t*)&Usr, sizeof(UsrConfig)-4, 0);
		uint64_t* pData = (uint64_t*)&Usr;
		for(int i=0; i<sizeof(UsrConfig)/8; i++){
			status = FLASH_program(USR_CONFIG_ROM_ADDR+i*8, *(pData+i));
			if(status != 0){
				break;
			}
		}
	}
	
	FLASH_lock();
	
	cpu_exit_critical(primask);
	
	return status;
}

int USR_CONFIG_read_cogging_map(void)
{
	int state = 0;
	
	if(pCoggingMap == NULL){
		pCoggingMap = HEAP_malloc(sizeof(tCoggingMap));
	}
	
	memcpy(pCoggingMap, (uint8_t*)USR_CONFIG_COGGING_MAP_ROM_ADDR, sizeof(tCoggingMap));
	
	uint32_t crc;
	crc = crc32((uint8_t*)pCoggingMap, sizeof(tCoggingMap)-4, 0);
	if(crc != pCoggingMap->crc){
		state = -1;
	}
	
	return state;
}

int USR_CONFIG_save_cogging_map(void)
{
	int status = 0;
	
	if(pCoggingMap == NULL){
		return -1;
	}
	
	uint32_t primask = cpu_enter_critical();
	
    FLASH_unlock();
	
	// Erase
	status = FLASH_erase_page(55, 5);
	
	if(status == 0){
		pCoggingMap->crc = crc32((uint8_t*)pCoggingMap, sizeof(tCoggingMap)-4, 0);
		uint64_t* pData = (uint64_t*)pCoggingMap;
		for(int i=0; i<sizeof(tCoggingMap)/8; i++){
			status = FLASH_program(USR_CONFIG_COGGING_MAP_ROM_ADDR+i*8, *(pData+i));
			if(status != 0){
				break;
			}
		}
	}
	
	FLASH_lock();
	
	cpu_exit_critical(primask);
	
	return status;
}
