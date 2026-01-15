#pragma once
#include <stdint.h>
#include "pico/time.h"
#include "pico/stdlib.h"
#include "pico/types.h"

typedef struct pose_t {
        float x;
        float y;
        float theta;
        absolute_time_t prev_time; 
} pose_t;

/*
        update odometry using encoder counts and precise timing 
*/
void odom_update(pose_t* pose);