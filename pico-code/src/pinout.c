#include "include/pinout.h"

// GPIO pins for each motor channel
const MotorChannel motorPinMap [WHEEL_COUNT] = 
{
        //| encA | encB | pwm1 | pwm2 | 
        {   28,    27,     3,     2    }, //FL [MDR1]
        {   26,    22,     5,     4    }, //FR [MDR2]
        {   21,    20,     7,     6    }, //RL [MDR3]
        {   19,    18,    16,     17   }, //RR [MDR4]
};