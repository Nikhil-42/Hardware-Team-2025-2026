#pragma once
#include "pico/stdlib.h"

#define PID_Kp 2.0f
#define PID_Ki 1.5f
#define PID_Kd 0.0f
#define PID_SAMPLE_TIME 0.002f // 2000 us (0.002 s)
#define PID_ACCUM_MAX 0.5
#define PID_OUT_MAX 1.0f
#define PID_OUT_MIN -1.0f

typedef struct PIDConstants
{
        float kp;
        float ki;
        float kd;
} PIDConstants;

typedef struct PIDController {
        // controller variables 
        float Kp;
        float Ki;
        float Kd; 

        // sample time 
        float sample_time;

        // error and accumulator terms 
        float previous_error; 
        float integration_accumulator;
        
        // limits on integation_accumulator and controller output 
        float integration_accumulator_max;
        float output_max;
        float output_min;

        // output from controller
        float output;
} PIDController;

/*
        initializes the PID controller for all wheels
*/
void pid_init_all(PIDController *all_pids); 

/*
        initializes the PID controller with controller variables 
*/
void pid_controller_init(PIDController *pid, float Kp, float Ki, float Kd, float dt, float accum_max, float out_max, float out_min); 

/*
        updates pid controller according to the pid variables and error from setpoint 
*/
float pid_controller_update(PIDController *pid, float setpoint, float process_variable);

/*
        updates all provides pid controllers
*/
void pid_controller_update_all(PIDController *all_pids, int16_t *setpoints, float *process_variables, float *pid_outputs);