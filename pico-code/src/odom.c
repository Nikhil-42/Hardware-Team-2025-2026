#include <stdio.h>
#include <math.h>
#include "include/odom.h"
#include "include/encoder.h"
#include "include/motor.h"
#include "include/kinematics.h"

#define WHEEL_CIRCUMFERENCE (2.0f * PI * R)
#define DS_CONV_FACTOR (WHEEL_CIRCUMFERENCE/COUNTS_PER_REVOULTION)
float ds[WHEEL_COUNT]; 

int count = 0;

void odom_update(pose_t* pose)
{
        absolute_time_t current_time = get_absolute_time();
        int64_t dt = absolute_time_diff_us(pose->prev_time, current_time);

        for(int i = 0; i < WHEEL_COUNT; i++)
        {
                ds[i] = encoder_counts_difference[i] * DS_CONV_FACTOR;
                //printf("ds: %f\t", ds[i]);
        }
        //printf("\n");
        // calculate displacements 
        float dx = ( ds[FL] + ds[FR] + ds[RL] + ds[RR]) / 4.0f;
        float dy = (-ds[FL] + ds[FR] + ds[RL] - ds[RR]) / 4.0f;
        float dtheta = (-ds[FL] + ds[FR] - ds[RL] + ds[RR]) / (4.0f * (Lx + Ly));

        // rotate   
        pose->x += dx * (float)cos((double)pose->theta) - dy * (float)sin((double)pose->theta);
        pose->y += dy * (float)cos((double)pose->theta) + dx * (float)sin((double)pose->theta);
        pose->theta = fmodf(pose->theta + dtheta, 2.0f * PI);

        /*
        count++;
        if(count > 1000)
        {
                count = 0;
                printf("Pose [x, y, theta]: [%f, %f, %f]\n", pose->x, pose->y, pose->theta);
        }
        */        

        // calculate pose by integrating rates 
                 

}

// update poses with rates rather than encoder displacements 
void update_pose_rate(pose_t* pose)
{

}