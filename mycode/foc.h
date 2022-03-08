#ifndef __FOC_H__
#define __FOC_H__

#include "main.h"
#include <stdbool.h>

typedef struct {
  uint16_t adc_phase_a, adc_phase_b, adc_vbus;            // Raw ADC Values
	float v_bus;                                            // DC link voltage
  	float i_a, i_b, i_c;                                    // Phase currents
	float i_d_filt, i_q_filt, i_bus_filt;                   // D/Q currents
	float current_ctrl_integral_d, current_ctrl_integral_q;	// Current error integrals
	int adc_phase_a_offset, adc_phase_b_offset;            	// ADC offsets
} FOCStruct;

extern FOCStruct Foc;

void FOC_zero_current(FOCStruct *foc);
void FOC_arm(void);
void FOC_disarm(void);
bool FOC_is_armed(void);
void FOC_reset(FOCStruct *foc);
float FOC_current(FOCStruct *foc, float Id_des, float Iq_des, float I_phase, float pwm_phase);

void clarke_transform(float Ia, float Ib, float Ic, float *Ialpha, float *Ibeta);
void park_transform(float Ialpha, float Ibeta, float Theta, float *Id, float *Iq);
void inverse_park(float mod_d, float mod_q, float Theta, float *mod_alpha, float *mod_beta);
int svm(float alpha, float beta, float* tA, float* tB, float* tC);

#endif
