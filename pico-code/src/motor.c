#include <math.h>
#include <stdio.h>
#include "include/motor.h"
#include "include/pinout.h"
#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "hardware/pwm.h"
#include "hardware/clocks.h"

uint32_t top; 

MotorPWMChannelSlice pwmChannelSliceMap[WHEEL_COUNT];

const MotorStates motorStateMap[MOTOR_COMMAND_COUNT] =
{       
        // PWM1 | PWM2
        {   1   ,  0   }, // FORWARD
        {   0   ,  1   }, // REVERSE
        {   0   ,  0   }, // SHORT_BRAKE_LOW
        {   1   ,  1   }  // SHORT_BRAKE_HIGH
};

void motor_pwm_init()
{
        // set all GPIO pins to function as PWM channels
        uint32_t PWM1_MASK = (1 << motorPinMap[FL].pwm1) | (1 << motorPinMap[FR].pwm1) | (1 << motorPinMap[RL].pwm1) | (1 << motorPinMap[RR].pwm1); 
        uint32_t PWM2_MASK = (1 << motorPinMap[FL].pwm2) | (1 << motorPinMap[FR].pwm2) | (1 << motorPinMap[RL].pwm2) | (1 << motorPinMap[RR].pwm2);   
        uint32_t PWM_FULL_MASK = PWM1_MASK | PWM2_MASK; 
        gpio_set_function_masked(PWM_FULL_MASK, GPIO_FUNC_PWM);       
        
        // set top value to set for PWM counter
        top = (clock_get_hz(clk_sys)/MAXIMUM_PWM_FREQUENCY); 

        // assign each motor pwm channel to a pico pwm channel 
        for(int i = 0; i < WHEEL_COUNT; i++)
        {
                pwmChannelSliceMap[i].pwm1_slice = pwm_gpio_to_slice_num(motorPinMap[i].pwm1); 
                pwmChannelSliceMap[i].pwm2_slice = pwm_gpio_to_slice_num(motorPinMap[i].pwm2); 
                pwmChannelSliceMap[i].pwm1_channel = pwm_gpio_to_channel(motorPinMap[i].pwm1); 
                pwmChannelSliceMap[i].pwm2_channel = pwm_gpio_to_channel(motorPinMap[i].pwm2); 

                uint slice1 = pwmChannelSliceMap[i].pwm1_slice; 
                uint slice2 = pwmChannelSliceMap[i].pwm2_slice; 
                uint channel1 = pwmChannelSliceMap[i].pwm1_channel; 
                uint channel2 = pwmChannelSliceMap[i].pwm2_channel;

                // set each channel to have the same carrier frequency 
                pwm_set_wrap(slice1, top); 
                pwm_set_wrap(slice2, top); 
                
                // set clkdiv for highest resolution
                pwm_set_clkdiv(slice1, 1.0f);
                pwm_set_clkdiv(slice2, 1.0f);

                // set to 0% duty cycle for short brake low at start
                pwm_set_chan_level(slice1, channel1, 0);
                pwm_set_chan_level(slice2, channel2, 0);

                pwm_set_enabled(slice1, true);
                pwm_set_enabled(slice2, true);
        }
}

void set_motor_pwm_channels(float *duty_cycles)
{
        // assume duty cycle a value between -1 and 1 representing the direction of the motor
        // and the percentage that the signal is expected to be high
        for (int i = 0; i < WHEEL_COUNT; i++)
        {                
                uint slice1 = pwmChannelSliceMap[i].pwm1_slice; 
                uint slice2 = pwmChannelSliceMap[i].pwm2_slice; 
                uint channel1 = pwmChannelSliceMap[i].pwm1_channel; 
                uint channel2 = pwmChannelSliceMap[i].pwm2_channel;

                uint16_t carrier_level_1 = (uint16_t) top * ((float)fabs(duty_cycles[i]));
                uint16_t carrier_level_2 = (uint16_t) top * ((float)fabs(duty_cycles[i]));

                // if wheel is inverted on the robot, the sign needs to be inverted so that the wheel turns the correct direction 
                duty_cycles[i] = (float)inverter[i] * duty_cycles[i];

                // determine sign of motor direction and if pwm channel is disabled
                if (duty_cycles[i] < 0)
                {
                        // FORWARD
                        carrier_level_1 *= motorStateMap[FORWARD].pwm1_state;  
                        carrier_level_2 *= motorStateMap[FORWARD].pwm2_state;
                }
                else if (duty_cycles[i] > 0)
                {
                        // REVERSE
                        carrier_level_1 *= motorStateMap[REVERSE].pwm1_state; 
                        carrier_level_2 *= motorStateMap[REVERSE].pwm2_state; 
                }
                //printf("top: %lu\n", top); 
                //printf("carrier levels (1): %u (2): %u\n", carrier_level_1, carrier_level_2);
                pwm_set_chan_level(slice1, channel1, carrier_level_1);
                pwm_set_chan_level(slice2, channel2, carrier_level_2);
        }  
}