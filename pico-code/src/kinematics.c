#include "include/kinematics.h"
#include <stdint.h>
#include <stdio.h>

// Resources:
// see pg 3 of https://research.ijcaonline.org/volume113/number3/pxc3901586.pdf
// for ik and fk matrices and math 

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

void solve_mecanum_ik(robot_velocities_t* robo_v, float* wheel_angular_velocities)
{
        // note that wheels i = 1 and i = 3 have an extra negative factor to account for the fact that these wheels are rotated 180 degrees and thus should 
        // rotate in the opposite direction than expected
        wheel_angular_velocities[0] = -(RADPS_TO_RPM_CONV) * ((1.0f/R) * (robo_v->vx - robo_v->vy - (Lx + Ly) * robo_v->wz)); 
        wheel_angular_velocities[1] = (RADPS_TO_RPM_CONV) * ((1.0f/R) * (robo_v->vx + robo_v->vy + (Lx + Ly) * robo_v->wz));
        wheel_angular_velocities[2] = -(RADPS_TO_RPM_CONV) * ((1.0f/R) * (robo_v->vx + robo_v->vy - (Lx + Ly) * robo_v->wz));
        wheel_angular_velocities[3] = (RADPS_TO_RPM_CONV) * ((1.0f/R) * (robo_v->vx - robo_v->vy + (Lx + Ly) * robo_v->wz));
        

        for(int i = 0; i < 4; i++)
        {
                printf("[ik] solved wheel %d angular velocity: %f\n", i, wheel_angular_velocities[i]); 
        }
}

// wav is wheel_angular_velocities
void solve_mecanum_fk(robot_velocities_t* robo_v, float* wav)
{
        robo_v->vx = (R/4.0f)*( wav[0] + wav[1] + wav[2] + wav[3]);
        robo_v->vy = (R/4.0f)*(-wav[0] + wav[1] + wav[2] - wav[3]);
        robo_v->wz = (1.0f / RADPS_TO_RPM_CONV) *(R/(4.0f * (Lx + Ly)))*(-wav[0] + wav[1] - wav[2] + wav[3]);
        
        //printf("solved vx: %f\n", robo_v->vx);
        //printf("solved vy: %f\n", robo_v->vy);
        //printf("solved wz: %f\n", robo_v->wz);
}