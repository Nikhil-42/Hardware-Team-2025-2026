#include <stdio.h>
#include "include/encoder.h"
#include "include/pinout.h"
#include "pico/stdlib.h"
#include "hardware/gpio.h"
  
volatile int64_t cumulative_encoder_counts[WHEEL_COUNT] = {0, 0, 0, 0};
volatile int64_t previous_encoder_counts[WHEEL_COUNT] = {0, 0, 0, 0};
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
        
        // if no channel found, return
        if(enc_ch < 0) {return;}

        uint8_t encA_level = gpio_get(motorPinMap[channel].encoderA);
        uint8_t encB_level = gpio_get(motorPinMap[channel].encoderB); 
        
        if (enc_ch == 0)
        {
                // if A and B channel levels are different, then motor is moving clockwise
                if (encA_level != encB_level)
                {
                        cumulative_encoder_counts[channel]++; 
                        //printf("INCREMENT A | Encoder counts now: %lld\n", (signed long long)cumulative_encoder_counts[channel]); 
                }
                // otherwise, the motor is moving counterclockwise
                else 
                {
                        cumulative_encoder_counts[channel]--;
                        //printf("DECREMENT A | Encoder counts now: %lld\n", (signed long long)cumulative_encoder_counts[channel]);
                }
        }
        else if (enc_ch == 1)
        {
                // if A and B channel levels are the same, motor is moving clockwise 
                if(encB_level == encA_level)
                {
                        cumulative_encoder_counts[channel]++;
                        //printf("INCREMENT B | Encoder counts now: %lld\n", (signed long long)cumulative_encoder_counts[channel]); 
                }
                //otherwise, the motor is moving ccw
                else
                {       
                        cumulative_encoder_counts[channel]--;
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
        for(int i = 0; i < WHEEL_COUNT; i++)
        {
                // determine number of encoder counts since last sample
                int64_t encoder_counts_difference = cumulative_encoder_counts[i] - previous_encoder_counts[i];
                
                // calculate rpms
                rpms[i] =  (float)encoder_counts_difference * 60.0f / (COUNTS_PER_REVOULTION * 2000e-6f);
                //printf("RPMs for wheel %d at %lld encoder counts:, %f\n", i, encoder_counts_difference, rpms[i]);
                previous_encoder_counts[i] = cumulative_encoder_counts[i];
        }
}