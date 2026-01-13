#pragma once
#include <stdint.h>

// ----------------------------------- MOTOR PINMAPPING -----------------------------------------------
#define WHEEL_COUNT 4
enum WheelPosition {FL, FR, RL, RR};

typedef struct MotorChannel
{
        uint8_t encoderA;
        uint8_t encoderB;
        uint8_t pwm1;
        uint8_t pwm2; 
} MotorChannel;  

// Stores the GPIO pins corresponding to each motor channel 
extern const MotorChannel motorPinMap [WHEEL_COUNT];

// transciever pins whenever that code is migrated here
#define IR_TRANSCIEVER_PIN