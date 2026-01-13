#include <stdio.h>
#include <stdint.h>
#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "include/encoder.h"
#include "include/motor.h"
#include "include/pid.h"
#include "include/ma.h"
#include "include/uart.h"

#define SAMPLING_INTERVAL_US 2000 

volatile bool sampling_flag = false; 

float motor_rpm[WHEEL_COUNT] = {0}; 

/*
        Determines the motor's rpm using encoder counts at every time interval
*/
static int64_t sampling_callback (alarm_id_t id, void *user_data)
{
        sampling_flag = true; 
        return SAMPLING_INTERVAL_US; 
}

int main()
{
        stdio_init_all();
        // wait for USB to enumerate
        sleep_ms(2000);

        // initialize all motor pwm channels 
        motor_pwm_init();
        float motor_rpm_per_sample[WHEEL_COUNT];

        // initialize all encoder channels 
        encoder_init();
        
        // initialize PID controllers according to wheel_pid_gains
        PIDController wheel_pid[WHEEL_COUNT]; 
        pid_init_all(wheel_pid); 

        float pid_outputs[WHEEL_COUNT] = {0};
        int16_t rpm_setpoints[WHEEL_COUNT] = {0};

        // initialize moving average filters
        runningAverageFilter ma_filt[WHEEL_COUNT]; 
        running_average_init_all(ma_filt, WHEEL_COUNT);

        float ma_filt_out[WHEEL_COUNT] = {0};
        
        add_alarm_in_us(SAMPLING_INTERVAL_US, sampling_callback, NULL, false);
        printf("properly initialized with alarm"); 
        while(true)
        {
                // continually read from UART and update expected expected motor speed setpoints if full packet is recieved
                if(rx_flag)
                {
                        rx_flag = false;
                        if(check_full_uart_packet())
                        {
                                extract_speed_packet(rpm_setpoints, WHEEL_COUNT);
                        }
                }

                if(sampling_flag)
                {
                        
                        // at every sampling interval, compute the current rpm  
                        sampling_flag = false;
                        calculate_rpms(motor_rpm_per_sample);

                        // update running average filter outputs
                        running_average_update_all(ma_filt, motor_rpm_per_sample, ma_filt_out, WHEEL_COUNT);
        
                        // compute pid outputs for each motor
                        pid_controller_update_all(wheel_pid, rpm_setpoints, motor_rpm_per_sample, pid_outputs); 
                        // write the duty cycle and pwm state to each motor
                        set_motor_pwm_channels(pid_outputs);

                        // send calculated speed information back over UART
                        send_speed_packet(motor_rpm_per_sample, WHEEL_COUNT);
                }
        }
}
