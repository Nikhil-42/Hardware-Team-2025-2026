#include <stdio.h>
#include <stdint.h>
#include <inttypes.h>
#include <math.h>
#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "include/encoder.h"
#include "include/motor.h"
#include "include/pid.h"
#include "include/ma.h"
#include "include/kinematics.h"
#include "include/keyboard.h"
#include "include/uart.h"
#include "include/odom.h"

#define SAMPLING_INTERVAL_US 2000 

volatile bool sampling_flag = false; 
bool first_packet_arrived = false;

float motor_rpm[WHEEL_COUNT] = {0}; 

/*
        set sampling flag to be used as main interval timer
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

        // initialize UART 
        uart0_init();

        // initialize all motor pwm channels 
        motor_pwm_init();
        float motor_rpm_per_sample[WHEEL_COUNT];

        // initialize all encoder channels 
        encoder_init();
        gpio_encoder_map_init();

        // Enable onboard LED for packet receipt indication
        gpio_init(PICO_DEFAULT_LED_PIN);
        gpio_set_dir(PICO_DEFAULT_LED_PIN, GPIO_OUT);
        bool led_state = false;
        gpio_put(PICO_DEFAULT_LED_PIN, led_state);
        
        // initialize PID controllers according to wheel_pid_gains
        PIDController wheel_pid[WHEEL_COUNT]; 
        pid_init_all(wheel_pid); 

        float pid_outputs[WHEEL_COUNT] = {0};

        // set a robot velocity 
        robot_velocities_t robo_v = {0.0f, 0.0f, 0.0f};
        robot_velocities_t cmd_vel_send = {0.0f, 0.0f, 0.0f};

        float rpm_setpoints[WHEEL_COUNT] = {0};

        // initialize moving average filters
        //runningAverageFilter ma_filt[WHEEL_COUNT]; 
        //running_average_init_all(ma_filt, WHEEL_COUNT);
        //float ma_filt_out[WHEEL_COUNT] = {0};

        // initialize pose
        pose_t pose = {0, 0, 0, 0};
        
        add_alarm_in_us(SAMPLING_INTERVAL_US, sampling_callback, NULL, false);
        while(true)
        {
                // continually read from UART and update expected expected motor speed setpoints if full packet is recieved
                if(cmd_ready)
                {
                        robo_v = cmd_vel;
                        cmd_ready = false;
                        first_packet_arrived = true; 

                        // indicate packet receipt by toggling onboard LED
                        led_state = !led_state;
                        gpio_put(PICO_DEFAULT_LED_PIN, led_state);
                }
                //check_keyboard_stroke(&robo_v);

                if(sampling_flag)
                {
                        absolute_time_t start_time = get_absolute_time();
                        // at every sampling interval, compute the current rpm  
                        sampling_flag = false;
                        calculate_rpms(motor_rpm_per_sample);

                        // update odometry
                        odom_update(&pose);

                        // compute inverse kinematics for wheels
                        solve_mecanum_ik(&robo_v, rpm_setpoints);
                        
                        // update running average filter outputs
                        //running_average_update_all(ma_filt, motor_rpm_per_sample, ma_filt_out, WHEEL_COUNT);

                        // compute pid outputs for each motor
                        pid_controller_update_all(wheel_pid, rpm_setpoints, motor_rpm_per_sample, pid_outputs); 
                        
                        // write the duty cycle and pwm state to each motor
                        set_motor_pwm_channels(pid_outputs); 
                        
                        // send calculated speed information back over UART
                        // update cmd_vel with calculated motor rpms to report back to Pi                 
                        solve_mecanum_fk(&cmd_vel_send, motor_rpm_per_sample);

                        // only send speed packets if we have recieved cmd_vel from Pi
                        if (first_packet_arrived)
                        {
                                send_speed_packet(&cmd_vel_send, &pose);
                        }


                }
        }
}
