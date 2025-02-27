

#include "motor_encoder.h"
#include "stdio.h"

/* @brief compute velocity of the motor using the timer encoder
 * @param encoder: Encoder instance
 * @retval: none:
 */
void get_encoder_speed(encoder_inst *encoder)
{

	    int32_t temp_counter = __HAL_TIM_GET_COUNTER(encoder->htim_encoder);
	    int32_t temp_timer = HAL_GetTick();

	    // Calculate the time period in seconds (HAL_GetTick() returns milliseconds, so divide by 1000)
	    encoder->timer_period = (temp_timer - encoder->last_timer) / 1000.0f;

	    // Avoid division by zero by checking the time period and adjusting accordingly
	    if (encoder->timer_period == 0)
	    {
	        encoder->velocity = 0;
	        return;
	    }

	    // Scaling factor: 60 is used to convert from seconds to minutes (for RPM)
	    float scaling = 60.0f / (PPR * encoder->timer_period);

	    // If running this code for the first time after reset, set velocity to zero
	    if (encoder->first_time)
	    {
	        encoder->velocity = 0;
	        encoder->first_time = 0;
	        encoder->timer_period = 0;
	    }
	    else
	    {
	        // If the counter value is equal to the old value, the velocity is zero
	        if (temp_counter == encoder->last_counter_value)
	        {
	            encoder->velocity = 0;
	        }
	        // If the counter value is higher (normal case)
	        else if (temp_counter > encoder->last_counter_value)
	        {
	            // Check for overflow
	            if (temp_counter - encoder->last_counter_value > __HAL_TIM_GET_AUTORELOAD(encoder->htim_encoder) / 2)
	            {
	                encoder->velocity = scaling * (-(encoder->last_counter_value) -
	                        (__HAL_TIM_GET_AUTORELOAD(encoder->htim_encoder) - temp_counter));
	            }
	            else
	            {
	                encoder->velocity = scaling * (temp_counter - encoder->last_counter_value);
	            }
	        }
	        // If the counter value is lower (counter wrapping around)
	        else
	        {
	            // Check for overflow
	            if ((encoder->last_counter_value - temp_counter) < __HAL_TIM_GET_AUTORELOAD(encoder->htim_encoder) / 2)
	            {
	                encoder->velocity = scaling * (temp_counter - encoder->last_counter_value);
	            }
	            else
	            {
	                encoder->velocity = scaling * (temp_counter +
	                        (__HAL_TIM_GET_AUTORELOAD(encoder->htim_encoder) - encoder->last_counter_value));
	            }
	        }
	    }

	    // Update position based on velocity and time period
	    encoder->position += encoder->velocity * encoder->timer_period;

	    // Save the current counter and time for the next cycle
	    encoder->last_timer = temp_timer;
	    encoder->last_counter_value = temp_counter;
}


/* @brief resetting the encoder state
 * @param encoder: encoder instance
 * @retval: none
 */
void reset_encoder(encoder_inst *encoder)
{
	encoder -> position = 0;
	encoder -> first_time = 1;
	encoder -> last_counter_value = 0;
	encoder -> velocity = 0;
}


