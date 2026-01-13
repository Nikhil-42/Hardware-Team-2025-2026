#pragma once
#include <stdint.h>
#include "pinout.h"

// The quadrature encoder on the 18.75:1 DC motors provides 64 counts per revolution of the motor shaft when counting 
// both edges of both channels. Currently the code counts both edges on 1 channel
// So, for the encoder A counts seen by the gearbox output, we have (64 / 2) * 18.75 = 600
// -------------------------------------- DEFINES -----------------------------------------------------

extern volatile int64_t cumulative_encoderA_counts[WHEEL_COUNT]; 
extern volatile int64_t encoderA_counts_since_last_sample[WHEEL_COUNT]; 

#define GEAR_RATIO 18.75f
#define COUNTS_PER_REVOULTION 600 // counting only level changes on 1 channel 

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