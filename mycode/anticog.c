#include "anticog.h"
#include "systick.h"
#include "para.h"
#include "tle.h"
#include "arm_math.h"
#include "arm_common_tables.h"
#include "heap.h"
#include "fsm.h"
#include "util.h"
#include "control.h"

// 步骤
typedef enum eAntiCoggingStep{
	AS_NULL = 0,
	AS_START,
	AS_FORWARD_LOOP,
	AS_BACKWARD_LOOP,
	AS_END,
}tAntiCoggingStep;

bool AnticoggingValid = false;
extern int sample_cnt;

static float current_sum;
static int cnt;
int sample_flag;

static tAntiCoggingStep AntiCoggingStep = AS_NULL;

void ANTICOGGING_start(void)
{
	AnticoggingValid = false;
	USR_CONFIG_set_default_cogging_map();
	AntiCoggingStep = AS_START;
}

void ANTICOGGING_abort(void)
{
	USR_CONFIG_set_default_cogging_map();
	AntiCoggingStep = AS_NULL;
}

bool ANTICOGGING_is_running(void)
{
	return (AntiCoggingStep != AS_NULL);
}

void ANTICOGGING_loop(ControllerStruct *controller)
{
	static int index = 0;
	static uint32_t tick = 0;
	
	if(AntiCoggingStep == AS_NULL){
		return;
	}
	
	// 200 Hz
	if(SYSTICK_get_ms_since(tick) < 5){
		return;
	}
	tick = SYSTICK_get_tick();
	
	switch(AntiCoggingStep){
		case AS_START:
			{
				sample_flag = 2;
				current_sum = 0;
				cnt = 0;
				float position = Encoder.position;
				float err = position - (int)position + 0.1;
				if(err > 0.001f){
					controller->input_pos -= 0.001f;
				}else if(err < -0.001f){
					controller->input_pos += 0.001f;
				}else{
					controller->input_pos -= err;
					index = 0;
					AntiCoggingStep = AS_FORWARD_LOOP;
				}
			}
			break;
		
		case AS_FORWARD_LOOP:
			{
				float pos_err = controller->input_pos - Encoder.position;
				if(fabs(pos_err) < Usr.anticogging_pos_threshold && fabs(Encoder.velocity) < Usr.anticogging_vel_threshold){
					if(sample_flag == 2)
						sample_flag = 1;
					
//					current_sum += CONTROLLER_get_integrator_current();
//					cnt++;
					if(sample_flag == 0){
						cnt = 0;
						if(controller->input_pos >= 0 && index < COGGING_MAP_NUM){
							pCoggingMap->map[index] = CONTROLLER_get_integrator_current();
							printf("F %d %f\n\r", index, CONTROLLER_get_integrator_current());
							index ++;
							
						}
						erase_current_sample();
						current_sum = 0;
						controller->input_pos += (1.0f / COGGING_MAP_NUM);
						sample_flag = 2;
						if(controller->input_pos >= 1.1){
							index --;
							controller->input_pos -= (1.0f / COGGING_MAP_NUM);
							AntiCoggingStep = AS_BACKWARD_LOOP;
						}
					}


				}
			}
			break;
			
		case AS_BACKWARD_LOOP:
			{
				float pos_err = controller->input_pos - Encoder.position;
				if(fabs(pos_err) < Usr.anticogging_pos_threshold && fabs(Encoder.velocity) < Usr.anticogging_vel_threshold){
					if(sample_flag == 2)
						sample_flag = 1;
					
//					current_sum += CONTROLLER_get_integrator_current();
//					cnt++;
					if(sample_flag == 0){
						cnt = 0;
						if(controller->input_pos <= 1 && index >= 0){
							pCoggingMap->map[index] += CONTROLLER_get_integrator_current();
							printf("B %d %f\n\r", index, CONTROLLER_get_integrator_current());
							index --;
							
						}											
						erase_current_sample();
						current_sum = 0;
						controller->input_pos -= (1.0f / COGGING_MAP_NUM);
						
						sample_flag = 2;
						if(controller->input_pos <= -0.1){
							AntiCoggingStep = AS_END;
						}
					}


				}
			}
			break;
		
		case AS_END:
			for(index=0; index<COGGING_MAP_NUM; index++){
				pCoggingMap->map[index] /= 2.0f;
			}
			AnticoggingValid = true;
//			sample_cnt = 0;
			FSM_input("1");
			AntiCoggingStep = AS_NULL;
			break;
		
		default:
			break;
	}
}
