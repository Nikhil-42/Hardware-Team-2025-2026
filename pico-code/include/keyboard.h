#include <stdint.h>
#include "kinematics.h"

#define VX_SETPOINT 0.2f
#define VY_SETPOINT 0.2f
#define WZ_SETPOINT 0.2f

//extern const int8_t stroke_to_motor_direction[7][4];

/*
        waits for keyboard stroke from serial monitor and updates robot velocity data type
        according to strokes captured
*/
void check_keyboard_stroke(robot_velocities_t* robo_v); 
