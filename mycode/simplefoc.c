#include "simplefoc.h"
#include "tle.h"
#include "foc.h"
#include "util.h"
#include <stdio.h>
#include <math.h>
#include "para.h"

tSimple_para simple_para;

void set_simple_para()
{
	simple_para.simple_P = 3;
	simple_para.simple_I = 300;
	simple_para.simple_D = 0;
	
	simple_para.velocity = 20;
	simple_para.vel_limit = 5;
	
	simple_para.integral_prev = 0;
  	simple_para.error_prev = 0;

}

void set_Uq(tSimple_para *simple_para)
{
	float error = simple_para->velocity - Encoder.velocity;
	// u_p  = P *e(k)
	float proportional = simple_para->simple_P * error;
	// u_ik = u_ik_1  + I*Ts/2*(ek + ek_1)
  float integral = simple_para->integral_prev + simple_para->simple_I * DT * 0.5f * (error + simple_para->error_prev);
	// antiwindup - limit the output
  integral = CLAMP(integral, -simple_para->vel_limit, simple_para->vel_limit);
	// u_dk = D(ek - ek_1)/Ts
  float derivative = simple_para->simple_D*(error - simple_para->error_prev)/DT;
	// sum all the components
  float output = proportional + integral + derivative;
	// antiwindup - limit the output variable
  output = CLAMP(output, -simple_para->vel_limit, simple_para->vel_limit);
	
	// saving for the next pass
  simple_para->integral_prev = integral;
  simple_para->error_prev = error;
	
	simple_para->simple_Uq = 2;
}


void simple_algorithm(FOCStruct *foc, tSimple_para *simple_para, float theta)
{
	// Modulation
	float mod_to_V = (2.0f / 3.0f) * foc->v_bus;
	float V_to_mod = 1.0f / mod_to_V;
	float mod_q = V_to_mod * simple_para->simple_Uq;
	
	// Inverse park transform
	float mod_alpha;
  float mod_beta;
	inverse_park(0, mod_q, theta, &mod_alpha, &mod_beta);
	
	// SVM
	float dtc_a, dtc_b, dtc_c;
	svm(mod_alpha, mod_beta, &dtc_a, &dtc_b, &dtc_c);

//  printf("%f  %f  %f\r\n",dtc_a, dtc_b, dtc_c);

	// Apply duty
	TIM1->CCR3 = (uint16_t)(dtc_a * (float)PWM_ARR);
	TIM1->CCR2 = (uint16_t)(dtc_b * (float)PWM_ARR);
	TIM1->CCR1 = (uint16_t)(dtc_c * (float)PWM_ARR);
}
