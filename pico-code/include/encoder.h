#pragma once
#include <stdint.h>
#include "pinout.h"
#include "pico/types.h"

// The quadrature encoder on the 18.75:1 DC motors provides 64 counts per revolution of the motor shaft when counting 
// both edges of both channels. Currently the code counts both edges on 1 channel
// So, for the encoder A counts seen by the gearbox output, we have (64 / 2) * 18.75 = 600
// -------------------------------------- DEFINES -----------------------------------------------------

extern volatile int64_t cumulative_encoder_counts[WHEEL_COUNT]; 
extern volatile int64_t previous_encoder_counts[WHEEL_COUNT];
extern volatile int32_t encoder_counts_difference[WHEEL_COUNT]; 
extern int8_t gpio_to_encoder_map[MAX_GPIO];
extern int8_t gpio_to_motor_ch[MAX_GPIO];
extern volatile absolute_time_t previous_tick_time[WHEEL_COUNT]; 
extern volatile int64_t tick_dt[WHEEL_COUNT];
extern volatile int8_t direction[WHEEL_COUNT];

#define GEAR_RATIO 18.75f
#define COUNTS_PER_REVOULTION 1200.0f // counting level changes on both channels 
#define WATCHDOG_TIMEOUT_US 100000 // us (100 ms)

// ---------------------------------- FUNCTION PROTOTYPES ---------------------------------------------
/*
        Initializes encoder pinouts and starts ISRs and timer callbacks for
        encoder counting and rpm's, respectively  
*/
void encoder_init();

/*
        Calculates the rpm of a single motor
*/
void calculate_rpms(float* rpms);

/*
        // maps gpio pins (as array index) to encoder channels assigned in pinout 
*/
void gpio_encoder_map_init();