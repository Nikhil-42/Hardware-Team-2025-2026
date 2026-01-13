#pragma once
#include <stdint.h>
#include "pico/stdlib.h"
#include "pinout.h"

// ------------------------------------- DEFINES ------------------------------------------------------
#define MAXIMUM_PWM_FREQUENCY 20000 // 20kHz
#define MINIMUM_PWM_WIDTH 10*10^-6  // 10 us 
#define MOTOR_COMMAND_COUNT 4

// -----------------------------------CUSTOM DATA TYPES -----------------------------------------------
typedef struct MotorPWMChannelSlice
{
        uint pwm1_channel;
        uint pwm2_channel;
        uint pwm1_slice; 
        uint pwm2_slice;
} MotorPWMChannelSlice;

// stores the channel and slice information for each motor 
extern MotorPWMChannelSlice pwmChannelSliceMap[WHEEL_COUNT];

// for indexing an array of MotorStates
enum MotorCommand {FORWARD, REVERSE, SHORT_BRAKE_HIGH, SHORT_BRAKE_LOW}; 

// stores the states required for the PWM channels to turn in certain directions 
typedef struct MotorStates
{
        uint16_t pwm1_state;
        uint16_t pwm2_state;
} MotorStates;

extern const MotorStates motorStateMap[MOTOR_COMMAND_COUNT];

// stores the duty cycles to write to each motor 
extern uint32_t motor_duty_cycles[WHEEL_COUNT]; 
extern int16_t expected_motor_speeds[WHEEL_COUNT]; 

// ---------------------------------- FUNCTION PROTOTYPES ---------------------------------------------
/*
        Allocates PWM slices for each motor channel based on their GPIO values 
*/
void motor_pwm_init();

/*
        Disable pwm channels associated with a motor 
*/
void disable_motor_pwm_carriers(uint motor); 

/*
        set the duty cycle and pwm channel states of the pwm channels based on the expected duty cycle and speeds
*/
void set_motor_pwm_channels(float *duty_cycles);

