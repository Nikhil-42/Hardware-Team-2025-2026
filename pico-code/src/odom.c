#include <stdio.h>
#include <math.h>
#include "include/odom.h"
#include "include/encoder.h"
#include "include/motor.h"
#include "include/kinematics.h"

#define WHEEL_CIRCUMFERENCE (2 * PI * R)
#define DS_CONV_FACTOR (WHEEL_CIRCUMFERENCE/COUNTS_PER_REVOULTION)
float ds[WHEEL_COUNT]; 

void odom_update(pose_t* pose)
{
        absolute_time_t current_time = get_absolute_time();
        int64_t dt = absolute_time_diff_us(pose->prev_time, current_time);

        for(int i = 0; i < WHEEL_COUNT; i++)
        {
                ds[i] = encoder_counts_difference[i] * DS_CONV_FACTOR;
        }

        // calculate displacements 
        float dx = ( ds[FL] + ds[FR] + ds[RL] + ds[RR]) / 4.0f;
        float dy = (-ds[FL] + ds[FR] + ds[RL] - ds[RR]) / 4.0f;
        float dtheta = (-ds[FL] + ds[FR] - ds[RL] + ds[RR]) / (4.0f * (Lx + Ly));

        // rotate   
        pose->x += dx * (float)cos((double)pose->theta) - dy * (float)sin((double)pose->theta);
        pose->y += dy * (float)cos((double)pose->theta) + dx * (float)sin((double)pose->theta);
        pose->theta = (float)((int64_t)(pose->theta + dtheta) % (int64_t)(2 * PI));
        printf("Pose [x, y, theta]: [%f, %f, %f]", pose->x, pose->y, pose->theta);

}