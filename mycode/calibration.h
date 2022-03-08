#ifndef __CALIBRATION_H__
#define __CALIBRATION_H__

#include "main.h"
#include "foc.h"

// 校准步骤
typedef enum eCalibrateStep{
	CS_NULL = 0,
	
	CS_MOTOR_R_START,
	CS_MOTOR_R_LOOP,
	CS_MOTOR_R_END,
	
	CS_MOTOR_L_START,
	CS_MOTOR_L_LOOP,
	CS_MOTOR_L_END,
	
	CS_ENCODER_DIR_FIND_START,
	CS_ENCODER_DIR_FIND_LOCK,
	CS_ENCODER_DIR_FIND_LOOP,
	CS_ENCODER_DIR_FIND_END,
	
	CS_ENCODER_OFFSET_START,
	CS_ENCODER_OFFSET_LOCK,
	CS_ENCODER_OFFSET_CW_LOOP,
	CS_ENCODER_OFFSET_CCW_LOOP,
	CS_ENCODER_OFFSET_END,
	
	CS_ERROR,
}tCalibrationStep;

typedef enum eCalibrationError{
	CE_NULL = 0,
	CE_PHASE_RESISTANCE_OUT_OF_RANGE,
	CE_MOTOR_POLE_PAIRS_OUT_OF_RANGE,
}tCalibrationError;

void CALIBRATION_start(void);
void CALIBRATION_end(void);
void CALIBRATION_loop(FOCStruct *foc);
void apply_voltage_timings(float vbus, float v_d, float v_q, float pwm_phase);

#endif
