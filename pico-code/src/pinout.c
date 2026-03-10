#include "pinout.h"

// GPIO pins for each motor channel
const MotorChannel motorPinMap [WHEEL_COUNT] = 
{
        //| encA | encB | pwm1 | pwm2 | 
        {   28,    27,     8,     9    }, //FL [MDR1]
        {   21,    20,     4,     5    }, //FR [MDR2]
        {   26,    22,     6,     7    }, //RL [MDR3]
        {   19,    18,     2,     3   }, //RR [MDR4]
};