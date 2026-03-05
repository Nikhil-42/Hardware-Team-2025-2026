#include <stdio.h>
#include <stdint.h>
#include <inttypes.h>
#include <math.h>
#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "encoder.h"
#include "motor.h"
#include "pid.h"
#include "ma.h"
#include "nec_transmit.h"
#include "kinematics.h"
#include "keyboard.h"
#include "uart.h"
#include "odom.h"
#include "pinout.h"

#define SAMPLING_INTERVAL_US 2000 
#define IR_INTERVAL_US 200000
#define WATCHDOG_TIMEOUT_US 100000

absolute_time_t last_cmd_time;
absolute_time_t last_ant_time;
size_t ant_i = 0;

volatile bool sampling_flag = false;
bool okay_to_send = false;
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

        PIO pio = pio0;
        int tx_sm = nec_tx_init(pio, IR_TRANSMIT_PIN);

        if (tx_sm < 0) {
                printf("Failed to initialize NEC transmitter\n");
                return 1;
        }

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

        //pwm38k_init();

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
                        okay_to_send = true; 

                        // indicate packet receipt by toggling onboard LED
                        led_state = !led_state;
                        gpio_put(PICO_DEFAULT_LED_PIN, led_state);

                        // reset the watchdog if new packet arrives 
                        last_cmd_time = get_absolute_time();
                }

                if(absolute_time_diff_us(last_ant_time, get_absolute_time()) > IR_INTERVAL_US)
                {
                        if (ant_nec_data[ant_i] != 255)
                        {

                                uint32_t frame = nec_encode_frame(EARTH_ADDR, ant_nec_data[ant_i]);
                                printf("SENDING NEC FRAME: 0x%02x\n", frame);
                                nec_send_frame(pio, tx_sm, frame);
                        }
                        last_ant_time = get_absolute_time();
                        ant_i = (ant_i + 1) % 4;
                }

                //if there hasnt been a new packet during a WATCHDOG_TIMEOUT period prevent pico from sending more and turn off motors 
                if(absolute_time_diff_us(last_cmd_time, get_absolute_time()) > WATCHDOG_TIMEOUT_US)
                {
                        okay_to_send = false; 
                        robo_v = (robot_velocities_t){0.0, 0.0, 0.0};
                        for(int i = 0; i < WHEEL_COUNT; i++)
                        {
                                pid_outputs[i] = 0.0;
                        }
                        set_motor_pwm_channels(pid_outputs);
                }

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
                        if (okay_to_send)
                        {
                                send_speed_packet(&cmd_vel_send, &pose);
                        }
                }
        }
}
