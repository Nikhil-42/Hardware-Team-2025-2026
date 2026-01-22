#include "include/pid.h"
#include "include/pinout.h"
#include <stdio.h>
#include <math.h>

const PIDConstants wheel_pid_gains[WHEEL_COUNT] = 
{
        {0.04f, 0.01f, 0.0f},
        {0.04f, 0.01f, 0.0f},
        {0.04f, 0.01f, 0.0f},
        {0.04f, 0.01f, 0.0f}
};

void pid_init_all(PIDController *all_pids)
{
        for(int i = 0; i < WHEEL_COUNT; i++)
        {
                pid_controller_init
                (       
                        &all_pids[i], 
                        wheel_pid_gains[i].kp, wheel_pid_gains[i].ki, wheel_pid_gains[i].kd, 
                        PID_SAMPLE_TIME, PID_ACCUM_MAX, PID_OUT_MAX, PID_OUT_MIN
                ); 
        }
        
}

void pid_controller_init(PIDController *pid, float Kp, float Ki, float Kd, float dt, float accum_max, float out_max, float out_min)
{
        pid->Kp = Kp;
        pid->Ki = Ki;
        pid->Kd = Kd;

        pid->sample_time = dt;

        pid->integration_accumulator = 0.0f;
        pid->previous_error = 0.0f;
        pid->integration_accumulator_max = accum_max; 
        pid->output_max = out_max;
        pid->output_min = out_min;
} 

float pid_controller_update(PIDController *pid, float setpoint, float process_variable)
{
        float error = setpoint - process_variable; 
        
        pid->integration_accumulator += error * pid->sample_time; 
        
        // clamp accumulator on either side of sum 
        if(pid->integration_accumulator > pid->integration_accumulator_max)
        {
                pid->integration_accumulator = pid->integration_accumulator_max;
        }
        else if (pid->integration_accumulator < -pid->integration_accumulator_max)
        {
                pid->integration_accumulator = -pid->integration_accumulator_max;        
        }

        float p = pid->Kp * error; 
        float i = pid->Ki * pid->integration_accumulator; 
        float d = pid->Kd * (error - pid->previous_error) / pid->sample_time; 

        pid->output = p + i + d;

        // if both the set point and process variable are close to zero, we should reset the accumulator so that it doesn't 
        // contribute to the output (i.e.)
        // creates a dropout region so that small encoder displacements do not cause the motor to shake 
        if (fabs(setpoint) < 1e-3 && fabs(process_variable))
        {
                pid->integration_accumulator = 0.0f;
                pid->output = 0.0f;
        }

        // clamp pid output to properly drive motor
        if (pid->output > pid->output_max)
        {
                pid->output = pid->output_max;
        }
        else if (pid->output < pid->output_min)
        {
                pid->output = pid->output_min;
        }
        printf("p: %f\n", p);
        printf("i: %f\n", i);
        printf("d: %f\n", d);
        printf("PID output %f\n", pid->output);
        pid->previous_error = error;
        return pid->output;
}

void pid_controller_update_all(PIDController *all_pids, float *setpoints, float *process_variables, float *pid_outputs)
{
        for(int i = 0; i < WHEEL_COUNT; i++)
        {
                pid_outputs[i] = pid_controller_update(&all_pids[i], setpoints[i], process_variables[i]); 
        }
}