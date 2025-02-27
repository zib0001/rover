#ifndef INC_PID_CONTROL_H_
#define INC_PID_CONTROL_H_

#include "main.h"

typedef struct
{
	float p_gain; /* p gain */
	float i_gain; /* i gain */
	float d_gain; /* d gain */
	float last_error; /* last error. It is necesary to compute the derivative */
	float error_integral; /* integral of an error */
	float output; /* output of the PID */
	uint16_t sam_rate; /* sampling rate */
	float integral_max; /* Maximum of the error integral */
	float pid_max; /* Maximum of the PID */
}pid_instance;
typedef enum
{
	pid_ok = 0,
	pid_numerical
}pid_typedef;

// Define Velocity-PWM Mapping Table
typedef struct {
    float velocity;    // RPM or velocity
    float pwm_duty;    // PWM duty cycle (%)
} Velocity_PWM_Map;
pid_typedef apply_pid(pid_instance *pid, float input_error);
void reset_pid(pid_instance *pid);
void set_pid(pid_instance *pid, float p, float i, float d);

float get_pwm_from_velocity(float desired_velocity);

#endif /* INC_PID_CONTROL_H_ */
