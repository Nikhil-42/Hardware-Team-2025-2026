#include <stdio.h>
#include <stdint.h>
#include "pico/time.h"
#include "pico/error.h"
#include "pico/stdio.h"
#include "pico/stdlib.h"
#include "include/kinematics.h"
#include "include/keyboard.h"

// for direct conversion to mecanum wheel angular velocities
/*
const int8_t stroke_to_motor_direction[7][4] = {
        {1, -1, 1, -1}, // [0] 'w': forward
        {}, // [1] 'a': strafe left
        {}, // [2] 's': backwards
        {}, // [3] 'd': strafe right 
        {}, // [4] 'q': rotate ccw
        {}, // [5] 'e': rotate cw
        {}  // [6] ' ': stop
};
*/

void check_keyboard_stroke(robot_velocities_t* robo_v)
{
        char c = getchar_timeout_us(0);
        if(c == PICO_ERROR_TIMEOUT)
        {
                return;
        }
        switch (c)
        {
                case 'w':
                        robo_v->vx = VX_SETPOINT;
                        robo_v->vy = 0.0f;
                        robo_v->wz = 0.0f;
                        break;

                case 'a':
                        robo_v->vx = 0.0f;
                        robo_v->vy = -VY_SETPOINT;
                        robo_v->wz = 0.0f;
                        break;
                case 's':
                        robo_v->vx = -VX_SETPOINT;
                        robo_v->vy = 0.0f;
                        robo_v->wz = 0.0f;
                        break;
                case 'd':
                        robo_v->vx = 0.0f;
                        robo_v->vy = VY_SETPOINT;
                        robo_v->wz = 0.0f; 
                        break;
                case 'q':
                        robo_v->vx = 0.0f;
                        robo_v->vy = 0.0f;
                        robo_v->wz = WZ_SETPOINT;
                        break;
                case 'e':
                        robo_v->vx = 0.0f;
                        robo_v->vy = 0.0f;
                        robo_v->wz = -WZ_SETPOINT;
                        break;
                case ' ':
                        robo_v->vx = 0.0f;
                        robo_v->vy = 0.0f;
                        robo_v->wz = 0.0f;
                        break;
                
        }
}