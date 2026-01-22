#pragma once
#include <stdint.h>

#define PI 3.14159265359f
#define RADPS_TO_RPM_CONV (30 / PI)
// M1 fl, M2 fr, M3 rl, M4 rr

// measurements in meters
#define R 0.03f // [m] radius of wheel
#define Lx 0.2f // [m] half distance between front wheels
#define Ly 0.2f // [m] half distance between front and rear wheel

//extern const float ik_matrix[4][3];
//extern const float fk_matrix[3][4];

typedef struct robot_velocities_t {
        float vx; // [m/s]
        float vy; // [m/s]
        float wz; // [rad/s]
} robot_velocities_t;

/*
        given robot velocities (v_x, v_y, w_z), calculate the angular velocity w_i for each mecanum wheel  
*/
void solve_mecanum_ik(robot_velocities_t* robo_v, float* wheel_angular_velocities); 

/*
        given wheel angular velocities, compute the robot velocities [v_x, v_y, w_z]
        where,
        v_x is the longitudinal velocity [m/s]   (forward/backward)
        v_y is the tranversal velocity   [m/s]   (left/right)
        w_z is the angular velocity      [rad/s] (rotation around z-axis)

*/
void solve_mecanum_fk(robot_velocities_t* robo_v, float* wav);
