#include "foc.h"
#include "drv.h"
#include "systick.h"
#include "util.h"
#include "tim.h"
#include "para.h"
#include <math.h>
#include "arm_math.h"
#include "arm_common_tables.h"

FOCStruct Foc;
static bool mIsArmed = false;

void FOC_zero_current(FOCStruct *foc)
{
	
    int adc_sum_a = 0;
    int adc_sum_b = 0;
    int n = 1000;
    for (int i = 0; i<n; i++){	// Average n samples of the ADC
        SYSTICK_delay_us(100);
        adc_sum_a += foc->adc_phase_a;
        adc_sum_b += foc->adc_phase_b;
    }
    foc->adc_phase_a_offset = adc_sum_a/n;
    foc->adc_phase_b_offset = adc_sum_b/n;
}

void FOC_arm(void)
{
	uint32_t prim = cpu_enter_critical();
	
	PWMC_switch_on_pwm();
	DRV8323_enable_gd();
	mIsArmed = true;
	
	cpu_exit_critical(prim);
}

void FOC_disarm(void)
{
	uint32_t prim = cpu_enter_critical();
	
	DRV8323_disable_gd();
	PWMC_switch_off_pwm();
	mIsArmed = false;
	
	cpu_exit_critical(prim);
}

bool FOC_is_armed(void)
{
	return mIsArmed;
}

void FOC_reset(FOCStruct *foc)
{
    /* Set all duty to 50% */
		TIM1->CCR3 = (PWM_ARR>>1);
		TIM1->CCR2 = (PWM_ARR>>1);
		TIM1->CCR1 = (PWM_ARR>>1);
	
		Foc.i_a_prev = 0;
		Foc.i_b_prev = 0;
		Foc.i_c_prev = 0;
		
    foc->i_d_filt = 0;
		foc->i_q_filt = 0;
    foc->current_ctrl_integral_d = 0;
    foc->current_ctrl_integral_q = 0;
}

float FOC_current(FOCStruct *foc, float Id_des, float Iq_des, float I_phase, float pwm_phase)
{
	// Clarke transform
	float i_alpha, i_beta;
	clarke_transform(foc->i_a, foc->i_b, foc->i_c, &i_alpha, &i_beta);
	
	// Park transform
	float i_d, i_q;
	park_transform(i_alpha, i_beta, I_phase, &i_d, &i_q);
	
	// Current Filter used for report
	foc->i_d_filt = 0.95f*foc->i_d_filt + 0.05f*i_d;
	foc->i_q_filt = 0.95f*foc->i_q_filt + 0.05f*i_q;
	
	// Apply PI control
	float Ierr_d = Id_des - i_d;
	float Ierr_q = Iq_des - i_q;
	float v_d = Usr.current_ctrl_p_gain * Ierr_d + foc->current_ctrl_integral_d;
	float v_q = Usr.current_ctrl_p_gain * Ierr_q + foc->current_ctrl_integral_q;
	
	
    // Modulation
    float mod_to_V = (2.0f / 3.0f) * foc->v_bus;
    float V_to_mod = 1.0f / mod_to_V;
    float mod_d = V_to_mod * v_d;
    float mod_q = V_to_mod * v_q;
	
	// Vector modulation saturation, lock integrator if saturated
    float mod_scalefactor = 0.8f * SQRT3_BY_2 * 1.0f / sqrtf(mod_d * mod_d + mod_q * mod_q);
    if (mod_scalefactor < 1.0f) {
        mod_d *= mod_scalefactor;
        mod_q *= mod_scalefactor;
        foc->current_ctrl_integral_d *= 0.99f;
        foc->current_ctrl_integral_q *= 0.99f;
    } else {
        foc->current_ctrl_integral_d += Ierr_d * (Usr.current_ctrl_i_gain * DT);
        foc->current_ctrl_integral_q += Ierr_q * (Usr.current_ctrl_i_gain * DT);
    }
	
	// Compute estimated bus current
    foc->i_bus_filt = mod_d * foc->i_d_filt + mod_q * foc->i_q_filt;
	
	// Inverse park transform
	float mod_alpha;
    float mod_beta;
	inverse_park(mod_d, mod_q, pwm_phase, &mod_alpha, &mod_beta);

	// SVM
	float dtc_a, dtc_b, dtc_c;
	svm(mod_alpha, mod_beta, &dtc_a, &dtc_b, &dtc_c);

    // SEGGER_RTT_printf(0,"%f  %f  %f\r\n",dtc_a, dtc_b, dtc_c);

	// Apply duty
	TIM1->CCR3 = (uint16_t)(dtc_a * (float)PWM_ARR);
	TIM1->CCR2 = (uint16_t)(dtc_b * (float)PWM_ARR);
	TIM1->CCR1 = (uint16_t)(dtc_c * (float)PWM_ARR);
		
	return v_q;
}

void clarke_transform(float Ia, float Ib, float Ic, float *Ialpha, float *Ibeta)
{
	// Ialpha = Ia
	// Ibeta = -(2*Ib+Ia)/sqrt(3)
	// plus 2/3
	*Ialpha = Ia;
	*Ibeta  = (Ib - Ic) * ONE_BY_SQRT3;		// Ic = -Ia - Ib 
}

void park_transform(float Ialpha, float Ibeta, float Theta, float *Id, float *Iq)
{
	// Id =  Ialpha * cos(Theta) + Ibeta * sin(Theta)
	// Iq = -Ialpha * sin(Theta) + Ibeta * cos(Theta)
	
	float c = arm_cos_f32(Theta);
  float s = arm_sin_f32(Theta);
	*Id =   Ialpha * c + Ibeta * s;
  *Iq = - Ialpha * s + Ibeta * c;
}

void inverse_park(float mod_d, float mod_q, float Theta, float *mod_alpha, float *mod_beta)
{
	// mod_alpha = mod_d * Cos(Theta) - mod_q * Sin(Theta)
	// mod_beta  = mod_d * Sin(Theta) + mod_q * Cos(Theta)
	
	float c = arm_cos_f32(Theta);
  float s = arm_sin_f32(Theta);
  *mod_alpha = mod_d * c - mod_q * s;
  *mod_beta  = mod_d * s + mod_q * c;
}

int svm(float alpha, float beta, float* tA, float* tB, float* tC)
{
    int Sextant;

	  // check which sextant
    if (beta >= 0.0f) {
        if (alpha >= 0.0f) {
            //quadrant I
            if (ONE_BY_SQRT3 * beta > alpha)
                Sextant = 2; //sextant v2-v3
            else
                Sextant = 1; //sextant v1-v2

        } else {
            //quadrant II
            if (-ONE_BY_SQRT3 * beta > alpha)
                Sextant = 3; //sextant v3-v4
            else
                Sextant = 2; //sextant v2-v3
        }
    } else {
        if (alpha >= 0.0f) {
            //quadrant IV
            if (-ONE_BY_SQRT3 * beta > alpha)
                Sextant = 5; //sextant v5-v6
            else
                Sextant = 6; //sextant v6-v1
        } else {
            //quadrant III
            if (ONE_BY_SQRT3 * beta > alpha)
                Sextant = 4; //sextant v4-v5
            else
                Sextant = 5; //sextant v5-v6
        }
    }

    switch (Sextant) {
        // sextant v1-v2
        case 1: {
            // Vector on-times
            float t1 = alpha - ONE_BY_SQRT3 * beta;
            float t2 = TWO_BY_SQRT3 * beta;

            // PWM timings
            *tA = (1.0f - t1 - t2) * 0.5f;
            *tB = *tA + t1;
            *tC = *tB + t2;
        } break;

        // sextant v2-v3
        case 2: {
            // Vector on-times
            float t2 = alpha + ONE_BY_SQRT3 * beta;
            float t3 = -alpha + ONE_BY_SQRT3 * beta;

            // PWM timings
            *tB = (1.0f - t2 - t3) * 0.5f;
            *tA = *tB + t3;
            *tC = *tA + t2;
        } break;

        // sextant v3-v4
        case 3: {
            // Vector on-times
            float t3 = TWO_BY_SQRT3 * beta;
            float t4 = -alpha - ONE_BY_SQRT3 * beta;

            // PWM timings
            *tB = (1.0f - t3 - t4) * 0.5f;
            *tC = *tB + t3;
            *tA = *tC + t4;
        } break;

        // sextant v4-v5
        case 4: {
            // Vector on-times
            float t4 = -alpha + ONE_BY_SQRT3 * beta;
            float t5 = -TWO_BY_SQRT3 * beta;

            // PWM timings
            *tC = (1.0f - t4 - t5) * 0.5f;
            *tB = *tC + t5;
            *tA = *tB + t4;
        } break;

        // sextant v5-v6
        case 5: {
            // Vector on-times
            float t5 = -alpha - ONE_BY_SQRT3 * beta;
            float t6 = alpha - ONE_BY_SQRT3 * beta;

            // PWM timings
            *tC = (1.0f - t5 - t6) * 0.5f;
            *tA = *tC + t5;
            *tB = *tA + t6;
        } break;

        // sextant v6-v1
        case 6: {
            // Vector on-times
            float t6 = -TWO_BY_SQRT3 * beta;
            float t1 = alpha + ONE_BY_SQRT3 * beta;

            // PWM timings
            *tA = (1.0f - t6 - t1) * 0.5f;
            *tC = *tA + t1;
            *tB = *tC + t6;
        } break;
    }

    // if any of the results becomes NaN, result_valid will evaluate to false
    int result_valid =
            *tA >= 0.0f && *tA <= 1.0f
         && *tB >= 0.0f && *tB <= 1.0f
         && *tC >= 0.0f && *tC <= 1.0f;
	
    return result_valid ? 0 : -1;
}
