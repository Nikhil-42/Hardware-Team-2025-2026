#include "include/kinematics.h"
#include "include/motor.h"
#include <stdint.h>
#include <stdio.h>

// Resources:
// see pg 3 of https://research.ijcaonline.org/volume113/number3/pxc3901586.pdf
// for ik and fk matrices and math 

/*
const float ik_matrix[4][3] = {
        {1/R, -1/R, -(Lx + Ly)/R},
        {1/R,  1/R,  (Lx + Ly)/R},
        {1/R,  1/R, -(Lx + Ly)/R},
        {1/R, -1/R,  (Lx + Ly)/R}
};

const float fk_matrix[3][4] = {
        {R/4, R/4, R/4, R/4},
        {-R/4, R/4, R/4, -R/4},
        {-R/(4 * (Lx + Ly)), R/(4 * (Lx + Ly)), -R/(4 * (Lx + Ly)), R/(4 * (Lx + Ly))}
};
*/

void solve_mecanum_ik(robot_velocities_t* robo_v, float* wheel_angular_velocities)
{
        wheel_angular_velocities[FL] = (RADPS_TO_RPM_CONV) * ((1.0f/R) * (robo_v->vx - robo_v->vy - (Lx + Ly) * robo_v->wz)); 
        wheel_angular_velocities[FR] = (RADPS_TO_RPM_CONV) * ((1.0f/R) * (robo_v->vx + robo_v->vy + (Lx + Ly) * robo_v->wz));
        wheel_angular_velocities[RL] = (RADPS_TO_RPM_CONV) * ((1.0f/R) * (robo_v->vx + robo_v->vy - (Lx + Ly) * robo_v->wz));
        wheel_angular_velocities[RR] = (RADPS_TO_RPM_CONV) * ((1.0f/R) * (robo_v->vx - robo_v->vy + (Lx + Ly) * robo_v->wz));
        
        for(int i = 0; i < WHEEL_COUNT; i++)
        {
                printf("[ik] solved wheel %d angular velocity: %f\n", i, wheel_angular_velocities[i]); 
        }
}

// wav is wheel_angular_velocities
void solve_mecanum_fk(robot_velocities_t* robo_v, float* wav)
{
        robo_v->vx = (R/4.0f)*( wav[FL] + wav[FR] + wav[RL] + wav[RR]);
        robo_v->vy = (R/4.0f)*(-wav[FL] + wav[FR] + wav[RL] - wav[RR]);
        robo_v->wz = (1.0f / RADPS_TO_RPM_CONV) *(R/(4.0f * (Lx + Ly)))*(-wav[FL] + wav[FR] - wav[RL] + wav[RR]);
        
        //printf("solved vx: %f\n", robo_v->vx);
        //printf("solved vy: %f\n", robo_v->vy);
        //printf("solved wz: %f\n", robo_v->wz);
}