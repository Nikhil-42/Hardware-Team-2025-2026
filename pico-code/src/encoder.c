#include <stdio.h>
#include <inttypes.h>
#include "include/encoder.h"
#include "include/pinout.h"
#include "pico/stdlib.h"
#include "hardware/gpio.h"
  
volatile int64_t cumulative_encoder_counts[WHEEL_COUNT] = {0, 0, 0, 0};
volatile int64_t previous_encoder_counts[WHEEL_COUNT] = {0, 0, 0, 0};
volatile int32_t encoder_counts_difference[WHEEL_COUNT] = {0, 0, 0, 0};
volatile absolute_time_t previous_tick_time[WHEEL_COUNT] = {0, 0, 0, 0};
volatile int64_t tick_dt[WHEEL_COUNT] = {0, 0, 0, 0};
volatile int8_t direction[WHEEL_COUNT] = {0, 0, 0, 0};
//int8_t ch2_sign = 0;

int8_t gpio_to_encoder_map[MAX_GPIO] = {0};
int8_t gpio_to_motor_ch[MAX_GPIO] = {0};

/*
        For the ISR, determine which wheel triggered the callback and 
        increment or decrement the counter according to channel levels
*/
void encoder_irq(uint gpio, uint32_t events)
{
        uint8_t channel = gpio_to_motor_ch[gpio]; 
        uint8_t enc_ch = gpio_to_encoder_map[gpio];
        uint8_t encA_level = gpio_get(motorPinMap[channel].encoderA);
        uint8_t encB_level = gpio_get(motorPinMap[channel].encoderB); 
        
        // if no channel found, return
        if(enc_ch < 0) {return;}
        if (enc_ch == 0)
        {
                // if A and B channel levels are different, then motor is moving clockwise
                if (encA_level != encB_level)
                {
                        cumulative_encoder_counts[channel]++; 
                        direction[channel] = 1; 
                        //printf("INCREMENT A | Encoder counts now: %lld\n", (signed long long)cumulative_encoder_counts[channel]); 
                }
                // otherwise, the motor is moving counterclockwise
                else 
                {
                        cumulative_encoder_counts[channel]--;
                        direction[channel] = -1;
                        //printf("DECREMENT A | Encoder counts now: %lld\n", (signed long long)cumulative_encoder_counts[channel]);
                }
        }
        else if (enc_ch == 1)
        {

                        absolute_time_t time_now = get_absolute_time();
                        tick_dt[channel] = absolute_time_diff_us(previous_tick_time[channel], time_now);
                        previous_tick_time[channel] = time_now;
                
                // if A and B channel levels are the same, motor is moving clockwise 
                if(encB_level == encA_level)
                {
                        cumulative_encoder_counts[channel]++;
                        direction[channel] = 1;
                        //printf("INCREMENT B | Encoder counts now: %lld\n", (signed long long)cumulative_encoder_counts[channel]); 
                }
                //otherwise, the motor is moving ccw
                else
                {       
                        cumulative_encoder_counts[channel]--;
                        direction[channel] = -1;
                        //printf("DECREMENT B | Encoder counts now: %lld\n", (signed long long)cumulative_encoder_counts[channel]);
                }
        }
        
}

void gpio_encoder_map_init()
{
        // for each gpio pin assign a -1 for no encoder, 0 for encoder A, 1 for encoder B
        for(int i = 0; i < MAX_GPIO; i++)
        {
                gpio_to_encoder_map[i] = -1;
                gpio_to_motor_ch[i] = -1;
        }
        for(int i = 0; i < WHEEL_COUNT; i++)
        {
                gpio_to_encoder_map[motorPinMap[i].encoderA] = 0;
                gpio_to_motor_ch[motorPinMap[i].encoderA] = i;
                gpio_to_encoder_map[motorPinMap[i].encoderB] = 1;
                gpio_to_motor_ch[motorPinMap[i].encoderB] = i;
        }
        for (int i = 0; i < MAX_GPIO; i++)
        {
                printf("encoder map at gpio %d: %d\n", i, gpio_to_encoder_map[i]);
                printf("gpio %d to motor ch: %d\n", i, gpio_to_motor_ch[i]);
        }
}

void encoder_init()
{
        //configure encoder masks as inputs 
        uint32_t ENCA_MASK = (1 << motorPinMap[FL].encoderA) | (1 << motorPinMap[FR].encoderA) | (1 << motorPinMap[RL].encoderA) | (1 << motorPinMap[RR].encoderA);
        uint32_t ENCB_MASK = (1 << motorPinMap[FL].encoderB) | (1 << motorPinMap[FR].encoderB) | (1 << motorPinMap[RL].encoderB) | (1 << motorPinMap[RR].encoderB);
        uint32_t ENC_FULL_MASK = (ENCA_MASK | ENCB_MASK);

        gpio_init_mask(ENC_FULL_MASK); 
        gpio_set_dir_in_masked(ENC_FULL_MASK);

        // configure edge-triggered interrupts for the encoder channels 
        for (int i = 0; i < WHEEL_COUNT; i ++)
        {
                MotorChannel channel = motorPinMap[i]; 
                uint8_t encA_pin = channel.encoderA; 
                gpio_set_irq_enabled_with_callback(encA_pin, (GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL), true, encoder_irq);
                
                uint8_t encB_pin = channel.encoderB; 
                gpio_set_irq_enabled_with_callback(encB_pin, (GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL), true, encoder_irq);
        }
}

void calculate_rpms(float* rpms)
{
        // determine number of encoder counts since last sample on each channel 
        // note that FL and RL are inverted because of the wheel geoemetry (i.e., ccw rotation results in positive displacement)
        encoder_counts_difference[FL] = - (cumulative_encoder_counts[FL] - previous_encoder_counts[FL]);
        encoder_counts_difference[FR] = cumulative_encoder_counts[FR] - previous_encoder_counts[FR];
        encoder_counts_difference[RL] = - (cumulative_encoder_counts[RL] - previous_encoder_counts[RL]);
        encoder_counts_difference[RR] = cumulative_encoder_counts[RR] - previous_encoder_counts[RR];

        for(int i = 0; i < WHEEL_COUNT; i++)
        {
                // if enough time has passed without an encoder tick, assume wheel is not moving
                absolute_time_t time_now = get_absolute_time();
                if (absolute_time_diff_us(previous_tick_time[i], time_now) > WATCHDOG_TIMEOUT_US)
                {
                        tick_dt[i] = 0; 
                }
                rpms[i] = 0;
                printf("enc difference: %" PRId32 "\n", encoder_counts_difference[i]);
                
                // calculate rpms
                //rpms[i] =  (float)encoder_counts_difference[i] * 60.0f / (COUNTS_PER_REVOULTION * 2000e-6f);
                if (tick_dt[i] != 0)
                {
                        rpms[i] = (float)direction[i] * (float)inverter[i] * 60.0f * 1000000.0f * 2 / ((float)tick_dt[i] * COUNTS_PER_REVOULTION);
                }
                printf("ticks per interval: %f", (float)tick_dt[i]);
                printf("RPMs: %f\n", rpms[i]);
                previous_encoder_counts[i] = cumulative_encoder_counts[i];
        }
}