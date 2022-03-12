#include "control.h"
#include "para.h"
#include "tle.h"
#include "util.h"
#include <math.h>
#include "traj.h"
#include "anticog.h"

ControllerStruct Controller;

static volatile bool input_pos_updated = false;

static float mPosSetPoint;
static float mVelSetPoint;
static float mTorqueSetPoint;

static float input_filter_kp;
static float input_filter_ki;

static float vel_integrator_torque;

float CONTROLLER_get_integrator_current(void)
{
	return vel_integrator_torque;
}

static void move_to_pos(float goal_point)
{
    TRAJ_plan(goal_point, mPosSetPoint, mVelSetPoint,
                                 Usr.traj_vel,		    // Velocity
                                 Usr.traj_accel,		// Acceleration
                                 Usr.traj_decel);		// Deceleration
	
    Traj.t = 0.0f;
    Traj.trajectory_done = false;
}

void CONTROLLER_reset(ControllerStruct *controller)
{
	controller->input_pos = Encoder.position;
	controller->input_vel = 0;
	controller->input_torque = 0;
	
	mPosSetPoint = controller->input_pos;
	mVelSetPoint = 0;
	mTorqueSetPoint = 0;
	
	vel_integrator_torque = 0;
	
	// update_filter_gains
	float bandwidth = MIN(Usr.input_pos_filter_bandwidth, 0.25f * CURRENT_MEAS_HZ);
  	input_filter_ki = 2.0f * bandwidth;
  	input_filter_kp = 0.25f * (input_filter_ki * input_filter_ki);
}

float current_calculate(ControllerStruct *controller)
{
		// Update inputs
    switch (Usr.input_mode) {
        case INPUT_MODE_PASSTHROUGH: {
            mPosSetPoint = Controller.input_pos;
            mVelSetPoint = Controller.input_vel;
            mTorqueSetPoint = Controller.input_torque; 
        } break;
				case INPUT_MODE_TORQUE_RAMP: {
            float max_step_size = fabs(DT * Usr.torque_ramp_rate);
            float full_step = Controller.input_torque - mTorqueSetPoint;
            float step = CLAMP(full_step, -max_step_size, max_step_size);
            mTorqueSetPoint += step;
        } break;
        case INPUT_MODE_VEL_RAMP: {
            float max_step_size = fabs(DT * Usr.vel_ramp_rate);
            float full_step = Controller.input_vel - mVelSetPoint;
            float step = CLAMP(full_step, -max_step_size, max_step_size);
            mVelSetPoint += step;
            mTorqueSetPoint = (step / DT) * Usr.inertia;
        } break;
        case INPUT_MODE_POS_FILTER: {
            // 2nd order pos tracking filter
            float delta_pos = Controller.input_pos - mPosSetPoint; // Pos error
            float delta_vel = Controller.input_vel - mVelSetPoint; // Vel error
            float accel = input_filter_kp*delta_pos + input_filter_ki*delta_vel; // Feedback
            mTorqueSetPoint = accel * Usr.inertia; // Accel
            mVelSetPoint += DT * accel; 		// delta vel
            mPosSetPoint += DT * mVelSetPoint; 	// Delta pos
        } break;
        case INPUT_MODE_TRAP_TRAJ: {
            if(input_pos_updated){
                move_to_pos(Controller.input_pos);
                input_pos_updated = false;
            }
			
            // Avoid updating uninitialized trajectory
            if (Traj.trajectory_done){
                break;
			}
            
            if (Traj.t > Traj.Tf_) {
                // Drop into position control mode when done to avoid problems on loop counter delta overflow
                Usr.control_mode = CONTROL_MODE_POSITION_CONTROL;
                mPosSetPoint = Controller.input_pos;
                mVelSetPoint = 0.0f;
                mTorqueSetPoint = 0.0f;
                Traj.trajectory_done = true;
            } else {
				TRAJ_eval(Traj.t);
				mPosSetPoint = Traj.Y;
				mVelSetPoint = Traj.Yd;
				mTorqueSetPoint = Traj.Ydd * Usr.inertia;
				Traj.t += DT;
            }
        } break;
        default: {
        } break;
    }

    float current,error_pos,error;
		float proportional,integral,derivative;
    switch(Usr.control_mode){
        case CONTROL_MODE_POSITION_CONTROL:

            error = mPosSetPoint - Encoder.position;

            // u_p  = P *e(k)
            proportional = Usr.pos_P * error;
            // u_ik = u_ik_1  + I*Ts/2*(ek + ek_1)
            integral = Usr.pos_integral_prev + Usr.pos_I * DT * 0.5f * (error + Usr.pos_error_prev);
            // u_dk = D(ek - ek_1)/Ts
						derivative = Usr.pos_D * (error - Usr.pos_error_prev)/DT;
            // sum all the components
            current = proportional + integral + derivative;
	
            // saving for the next pass
            Usr.pos_integral_prev = integral;
            Usr.pos_error_prev = error;

            break;
        case CONTROL_MODE_VELOCITY_CONTROL:

            error = mVelSetPoint - Encoder.velocity;
            // u_p  = P *e(k)
            proportional = Usr.vel_P * error;
            // u_ik = u_ik_1  + I*Ts/2*(ek + ek_1)
            integral = Usr.vel_integral_prev + Usr.vel_I * DT * 0.5f * (error + Usr.vel_error_prev);
            // u_dk = D(ek - ek_1)/Ts
            // sum all the components
            current = proportional + integral;
	
            // saving for the next pass
            Usr.vel_integral_prev = integral;
            Usr.vel_error_prev = error;
            

            break;
        case CONTROL_MODE_TORQUE_CONTROL:
            current = mTorqueSetPoint / Usr.torque_constant;
            break;
        default:
            break;
    }

    // Current limit
    bool limited = false;
    if (current > Usr.current_limit) {
        limited = true;
        current = Usr.current_limit;
    }
    if (current < -Usr.current_limit) {
        limited = true;
        current = -Usr.current_limit;
    }

    return current;
}


