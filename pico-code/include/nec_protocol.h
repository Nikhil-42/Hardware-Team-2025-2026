#ifndef NEC_PROT_H_
#define NEC_PROT_H_

#include <stdint.h>
#include "hardware/timer.h"
#include "hardware/clocks.h"
#include "hardware/pwm.h"

#define IR_TRANS_PIN 0

// ----------------- EARTH TRANSMISSION CODE DEFINES ------------------- //
#define EARTH_ADDR 0xBB
// a single transmission command takes the form 
// ( ( ANTENNA upper byte << 4) | COLOR lower byte ) 
// antenna codes 
#define ANTENNA_1_gc 0x00
#define ANTENNA_2_gc 0x30
#define ANTENNA_3_gc 0x50
#define ANTENNA_4_gc 0x60
// color codes
#define RED_gc 0x09
#define GREEN_gc 0x0A
#define BLUE_gc 0x0C
#define PURPLE_gc 0x0F

#define ANTENNA_CODE_BP 4

// ----------------------- GENERAL NEC DEFINES ---------------------------- //
// see https://sibotic.wordpress.com/wp-content/uploads/2013/12/adoh-necinfraredtransmissionprotocol-281113-1713-47344.pdf
// for NEC communication protocol
// basics: NEC communication requires a 38kHz carrier waveform. 
// Information is encoded into the carrier using pulse-distance modulation by changing the space between pulses. 

#define NEC_PULSE_US 562
#define NEC_LEAD_PULSE_US ((NEC_PULSE_US) * 16) //9000 us 
#define NEC_LEAD_SPACE_US ((NEC_PULSE_US) * 8)
#define NEC_0_SPACE_US ((NEC_PULSE_US) * 1)
#define NEC_1_SPACE_US ((NEC_PULSE_US) * 3)

void pwm38k_init(); 
// initializes PWM, counter, and clkdiv values for proper PWM operation

// enable PWM channel 
void enable_carrier();

// disable PWM channel
void disable_carrier();

uint32_t nec_encode(uint8_t address, uint8_t command); 
/*
FUNCTION: 
        > creates and returns transmission frame for address and command 
INPUTS:
        > device address 
        > command 
        > pointer to waveform that stores encoded information

*/

void nec_send(uint32_t data); 
/*
FUNCTION:
        > interrupts PWM cycle through series of enables and disables based on data information 
INPUTS:
        > encoded transmission frame 
*/

#endif