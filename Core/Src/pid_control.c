#include "pid_control.h"

/*	@brief setting pid gains
 * 	@param pid: pid instance
 * 	@param p: proportional gain
 * 	@param i: inegral gain
 * 	@param d: derivative gain
 * 	@retval: none
 * */

void set_pid(pid_instance *pid, float p, float i, float d)
{
	pid ->error_integral = 0;
	pid ->p_gain = p;
	pid ->i_gain = i;
	pid ->d_gain = d;
}

/*	@brief resetting the pid
 * 	@param pid: pid instance
 * 	@retval: none
 * */
void reset_pid(pid_instance *pid)
{
	pid -> error_integral = 0;
	pid -> last_error = 0;

}

/*	@brief apply pid
 * 	This function computes the PID output considering the PID gains and limits
 * 	@param pid: pid instance
 * 	@param input_error: input error
 * 	@retval: none
 * */
pid_typedef apply_pid(pid_instance *pid, float input_error)
{
    pid->error_integral += input_error;

    // Anti-windup: Clamp integral term
    if (pid->error_integral > pid->integral_max) {
        pid->error_integral = pid->integral_max;
    }
    if (pid->error_integral < -pid->integral_max) {
        pid->error_integral = -pid->integral_max;
    }

    // Prevent division by zero
    if (pid->sam_rate == 0) {
        pid->sam_rate = 1;
    }

    // Compute PID output with corrected D-term scaling
    pid->output = pid->p_gain * input_error +
                  pid->i_gain * (pid->error_integral) / pid->sam_rate +
                  pid->d_gain * (input_error - pid->last_error) / pid->sam_rate;

    // Output saturation (clamping)
    if (pid->output > pid->pid_max) {
        pid->output = pid->pid_max;
        pid->error_integral -= input_error; // Stop integrating when max is reached
    }
    if (pid->output < -pid->pid_max) {
        pid->output = -pid->pid_max;
        pid->error_integral -= input_error; // Stop integrating when min is reached
    }

    pid->last_error = input_error;

    return pid_ok;
}





#define TABLE_SIZE (sizeof(velocity_pwm_table) / sizeof(Velocity_PWM_Map))
// Predefined Velocity-to-PWM Mapping Table
const Velocity_PWM_Map velocity_pwm_table[] = {
    { -68.0, -100.0 }, { -60.0, -90.0 }, { -52.0, -80.0 },
    { -45.0, -70.0 }, { -37.0, -60.0 }, { -23.0, -40.0 },
    { -16.3, -30.0 }, { -9.5, -20.0 }, { -3.772727, -11.81818 },
    { -3.136364, -10.90909 }, { -2.5, -10.0 },
    {  0.0,  0.0 },
    {  2.5,  10.0 }, {  3.136364,  10.90909 }, {  3.772727,  11.81818 },
    {  9.5,  20.0 }, {  16.3,  30.0 }, {  23.0,  40.0 },
    {  37.0,  60.0 }, {  45.0,  70.0 }, {  52.0,  80.0 },
    {  60.0,  90.0 }, {  68.0,  100.0 }
};


/**
 * @brief Get PWM duty cycle for a given velocity using linear interpolation.
 * @param desired_velocity: Target velocity (RPM).
 * @return PWM duty cycle (0-100%).
 */
float get_pwm_from_velocity(float desired_velocity) {
    for (int i = 0; i < TABLE_SIZE - 1; i++) {
        if (desired_velocity >= velocity_pwm_table[i].velocity &&
            desired_velocity <= velocity_pwm_table[i + 1].velocity) {

            float v1 = velocity_pwm_table[i].velocity;
            float v2 = velocity_pwm_table[i + 1].velocity;
            float pwm1 = velocity_pwm_table[i].pwm_duty;
            float pwm2 = velocity_pwm_table[i + 1].pwm_duty;

            // Linear Interpolation
            return pwm1 + ((desired_velocity - v1) / (v2 - v1)) * (pwm2 - pwm1);
        }
    }

    // Clamp PWM if velocity is outside the range
    if (desired_velocity < velocity_pwm_table[0].velocity)
        return velocity_pwm_table[0].pwm_duty;
    if (desired_velocity > velocity_pwm_table[TABLE_SIZE - 1].velocity)
        return velocity_pwm_table[TABLE_SIZE - 1].pwm_duty;

    return 0.0; // Default case (should not occur)
}
