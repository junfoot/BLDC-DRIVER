#include "fsm.h"
#include <math.h>
#include "foc.h"
#include "drv.h"
#include "util.h"
#include "para.h"
#include <string.h>
#include "tle.h"
#include "control.h"
#include "calibration.h"
#include "anticog.h"
#include "simplefoc.h"

// initialization fsm
static tFSM Fsm = {
	.state = FS_MENU_MODE,
	.next_state = FS_MENU_MODE,
	.ready = 1,
	.bytecount = 0,
};

static volatile int mErrorCode = 0;
static volatile int mErrorCodeLast = 0;

static void enter_state(void);
static void exit_state(void);
static void reset_error(void);

float vq;

char tmp1[10], tmp2[10];
float temp;

void FSM_input(char *order)
{
	// choose mode
	if(strcmp(order, "1") == 0){
		Fsm.next_state = FS_MENU_MODE;
		Fsm.ready = 0;
	}
	else if(strcmp(order, "2") == 0){
		if(mErrorCode == 0 && Usr.calib_valid){
			Fsm.next_state = FS_MOTOR_MODE;
			Fsm.ready = 0;
		}
	}
	else if(strcmp(order, "3") == 0){
		Fsm.next_state = FS_CALIBRATION_MODE;
		Fsm.ready = 0;
	}
	else if(strcmp(order, "4") == 0){
	  if(mErrorCode == 0 && Usr.calib_valid && Usr.input_mode == INPUT_MODE_PASSTHROUGH && Usr.control_mode == CONTROL_MODE_POSITION_CONTROL){
			Fsm.next_state = FS_ANTICOGGING_MODE;
			Fsm.ready = 0;
		}
	}
	else if(strcmp(order, "5") == 0){
		if(mErrorCode == 0){
				Fsm.next_state = FS_SIMPLE_MODE;
				Fsm.ready = 0;
			}
	}
	else if(strcmp(order, "p") == 0){
			Usr.control_mode = CONTROL_MODE_POSITION_CONTROL;
			printf("set control mode to POSITION\r\n");
	}
	else if(strcmp(order, "v") == 0){
			Usr.control_mode = CONTROL_MODE_VELOCITY_CONTROL;
			printf("set control mode to VELOCITY\r\n");
	}
	else if(strcmp(order, "t") == 0){
			Usr.control_mode = CONTROL_MODE_TORQUE_CONTROL;
			printf("set control mode to TORQUE\r\n");
	}
	else if(strcmp(order, "a") == 0){
		Usr.input_mode = INPUT_MODE_PASSTHROUGH;
		printf("set input mode to PASSTHROUGH\r\n");
	}
	else if(strcmp(order, "s") == 0){
		Usr.input_mode = INPUT_MODE_TORQUE_RAMP;
		printf("set input mode to TORQUE_RAMP\r\n");
	}
	else if(strcmp(order, "d") == 0){
		Usr.input_mode = INPUT_MODE_VEL_RAMP;
		printf("set input mode to VEL_RAMP\r\n");
	}
	else if(strcmp(order, "f") == 0){
		Usr.input_mode = INPUT_MODE_POS_FILTER;
		printf("set input mode to POS_FILTER\r\n");
	}
	else if(strcmp(order, "g") == 0){
		Usr.input_mode = INPUT_MODE_TRAP_TRAJ;
		printf("set input mode to TRAP_TRAJ\r\n");
	}
	else if(strcmp(order, "z") == 0){
		Usr.anticogging_enable = 1 - Usr.anticogging_enable;
		printf("anticogging %d\r\n",Usr.anticogging_enable);
	}
	
	// input data
	else{
		sscanf(data, "%s %s",tmp1,tmp2);
		temp = strtof(tmp2,NULL);
		
		if(strcmp(tmp1, "tin") == 0){
			Controller.input_torque = temp;
		}
		else if(strcmp(tmp1, "vin") == 0){
			Controller.input_vel = temp;
		}
		else if(strcmp(tmp1, "pin") == 0){
			Controller.input_pos = temp;
			CONTROLLER_move_to_pos(temp);
		}
		else if(strcmp(tmp1, "posp") == 0){
			Usr.pos_P = temp;
		}
		else if(strcmp(tmp1, "posi") == 0){
			Usr.pos_I = temp;
		}
		else if(strcmp(tmp1, "posd") == 0){
			Usr.pos_D = temp;
		}
		else if(strcmp(tmp1, "velp") == 0){
			Usr.vel_P = temp;
		}
		else if(strcmp(tmp1, "veli") == 0){
			Usr.vel_I = temp;
		}
		else if(strcmp(tmp1, "veld") == 0){
			Usr.vel_D = temp;
		}
	}

}

int cntt = 0;

void FSM_loop(void)
{

	/* state transition management */
	if(Fsm.next_state != Fsm.state){
		exit_state();
		if(Fsm.ready){
			Fsm.state = Fsm.next_state;
			enter_state();
		}
	}

	switch(Fsm.state){
		case FS_MENU_MODE:
			return;
		
		// working mode
		case FS_MOTOR_MODE:
		case FS_ANTICOGGING_MODE:
			{
				float current_setpoint = 0;
				current_setpoint = current_calculate(&Controller);

				vq = FOC_current(&Foc, 0, current_setpoint, Encoder.elec_angle, Encoder.elec_angle+ 1.5f*DT*Encoder.velocity_elec);
				
				// Over speed check
				if(fabs(Encoder.velocity) > Usr.protect_over_speed){
					mErrorCode |= ERR_OVER_SPEED;
				}
			}
			break;
			
		case FS_CALIBRATION_MODE:
			CALIBRATION_loop(&Foc);
			break;
		
		case FS_SIMPLE_MODE:
			
			set_Uq(&simple_para);
//			simple_algorithm(&Foc, &simple_para, Encoder.elec_angle);
			apply_voltage_timings(Foc.v_bus, 0, simple_para.simple_Uq, Encoder.elec_angle);
		
			break;

		default:
			break;
	}

	// Over voltage check
	if(Foc.v_bus >= Usr.protect_over_voltage){
		mErrorCode |= ERR_OVER_VOLTAGE;
	}
	
	// Under voltage check
	if(Foc.v_bus <= Usr.protect_under_voltage){
		mErrorCode |= ERR_UNDER_VOLTAGE;
	}
	
	// DRV8323 error
	if(DRV8323_isFault()){
		mErrorCode |= DRV8323_getFault();
	}
	
	if(mErrorCode){
		FOC_disarm();
		Fsm.next_state = FS_MENU_MODE;
		Fsm.ready = 0;
	}
	
	if(mErrorCode != mErrorCodeLast){
		printf("Error Code: %X\r\n", mErrorCode);
		mErrorCodeLast = mErrorCode;
	}
}

int FSM_get_error(void)
{
	return mErrorCode;
}

tFsmStat FSM_get_stat(void)
{
	return Fsm.state;
}

static void enter_state(void)
{
	switch(Fsm.state){
		case FS_MENU_MODE:
			printf("Enter MENU Mode\r\n");
			break;
		
		case FS_MOTOR_MODE:
			printf("Enter Motor Mode\r\n");
			CONTROLLER_reset(&Controller);
			FOC_reset(&Foc);
			FOC_arm();
			break;
		
		case FS_CALIBRATION_MODE:
			printf("Calibration Start\r\n");
			CALIBRATION_start();
			break;

		case FS_ANTICOGGING_MODE:
			printf("Anticogging Start\r\n");
			CONTROLLER_reset(&Controller);
			FOC_reset(&Foc);
			FOC_arm();
			ANTICOGGING_start();
			break;	
		
		case FS_SIMPLE_MODE:
			set_simple_para();
			FOC_reset(&Foc);
			FOC_arm();
			printf("Enter SimpleFOC Mode\r\n");
			break;

		default:
			break;
	}
}

static void exit_state(void)
{
	switch(Fsm.state){
		case FS_MENU_MODE:
			printf("Exit MENU\n\r");
			Fsm.ready = 1;
			break;

		case FS_MOTOR_MODE:
			FOC_disarm();
			printf("Exit motor mode\r\n");
			Fsm.ready = 1;
			break;

		case FS_CALIBRATION_MODE:
			FOC_disarm();
			CALIBRATION_end();
			printf("Calibration End\r\n");
			Fsm.ready = 1;
			break;

		case FS_ANTICOGGING_MODE:
			USR_CONFIG_save_cogging_map();
			FOC_disarm();
			ANTICOGGING_abort();
			printf("Anticogging End\r\n");
			Fsm.ready = 1;
			break;	
		
		case FS_SIMPLE_MODE:
			FOC_disarm();
			printf("Exit SimpleFOC mode\r\n");
			Fsm.ready = 1;
			break;

		default:
			break;
	}
}

static void reset_error(void)
{
	DRV8323_reset();
	
	uint32_t primask = cpu_enter_critical();
	mErrorCode = 0;
	mErrorCodeLast = 0;
	cpu_exit_critical(primask);
}

