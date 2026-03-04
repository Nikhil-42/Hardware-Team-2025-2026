/**
 * Copyright (c) 2021 mjcross
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "pico/stdlib.h"
#include "hardware/pio.h"

// ----------------- EARTH TRANSMISSION CODE DEFINES ------------------- //

#define EARTH_ADDR 0xBB
// a single transmission command takes the form 
// ( ( ANTENNA upper byte << 4) | COLOR lower byte ) 
// antenna codes 
#define ANTENNA_1_gc 0x00 // button
#define ANTENNA_2_gc 0x30 // crank
#define ANTENNA_3_gc 0x50 // pressure plate
#define ANTENNA_4_gc 0x60 // keypad
// color codes
#define RED_gc 0x09 
#define GREEN_gc 0x0A
#define BLUE_gc 0x0C
#define PURPLE_gc 0x0F

#define ANTENNA_CODE_BP 4

extern uint8_t ant_nec_data[4];

// public API

int nec_tx_init(PIO pio, uint pin);
uint32_t nec_encode_frame(uint8_t address, uint8_t data);
void nec_send_frame(PIO pio, int sm, uint32_t frame);
