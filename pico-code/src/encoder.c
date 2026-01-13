#include <stdio.h>
#include "include/encoder.h"
#include "include/pinout.h"
#include "pico/stdlib.h"
#include "hardware/gpio.h"
  
volatile int64_t cumulative_encoderA_counts[WHEEL_COUNT] = {0, 0, 0, 0};
volatile int64_t previous_encoderA_counts[WHEEL_COUNT] = {0, 0, 0, 0};

/*
        For the ISR, determine which wheel triggered the callback and 
        increment or decrement the counter according to channel levels
*/
void encoderA_irq(uint gpio, uint32_t events)
{
        uint8_t channel = -1; 
        // determine which wheel's encoder channel triggered the callback 
        for(int i = 0; i < WHEEL_COUNT; i++)
        {
                if(gpio == motorPinMap[i].encoderA)
                {
                        channel = i;
                        //printf("channel %d triggered\n", channel);
                        break;
                } 
        }
        
        uint8_t encA_level = gpio_get(motorPinMap[channel].encoderA);
        uint8_t encB_level = gpio_get(motorPinMap[channel].encoderB); 

        // if A and B channel levels are different, then motor is moving clockwise 
        if (encA_level != encB_level)
        {
                cumulative_encoderA_counts[channel]++; 
                //printf("INCREMENT | Encoder counts now: %lld\n", (signed long long)cumulative_encoderA_counts[channel]); 
                
        }
        // otherwise, the motor is moving counterclockwise
        else 
        {
                cumulative_encoderA_counts[channel]--;
                //printf("DECREMENT | Encoder counts now: %lld\n", (signed long long)cumulative_encoderA_counts[channel]);
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

        // configure edge-triggered interrupts for the encoder pins 
        for (int i = 0; i < WHEEL_COUNT; i ++)
        {
                MotorChannel channel = motorPinMap[i]; 
                uint8_t encA_pin = channel.encoderA; 
                gpio_set_irq_enabled_with_callback(encA_pin, (GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL), true, encoderA_irq); 
        }
}

void calculate_rpms(float* rpms)
{
        for(int i = 0; i < WHEEL_COUNT; i++)
        {
                // determine number of encoder counts since last sample
                int64_t encoder_counts_difference = cumulative_encoderA_counts[i] - previous_encoderA_counts[i]; 
                
                // calculate rpms
                rpms[i] =  (float)encoder_counts_difference * 60.0f / (COUNTS_PER_REVOULTION * 2000e-6f);
                printf("RPMs for wheel %d at %lld encoder counts:, %f\n", i, encoder_counts_difference, rpms[i]);
                previous_encoderA_counts[i] = cumulative_encoderA_counts[i];
        }
}