#pragma once
#include <stdint.h>
#include <stdbool.h>

#define UART0_BAUD 115200
#define UART0_DATA 8
#define UART0_STOP 1
#define BYTES_IN_SPEED_TRANSMISSION 12
#define SPEED_DATA_BYTES 8
#define RX_BUFFER_LENGTH 50

extern const uint8_t SPEED_START_BYTE_H;
extern const uint8_t SPEED_START_BYTE_L;
extern const uint8_t SPEED_END_BYTE_H;
extern const uint8_t SPEED_END_BYTE_L;

// stores characters read from UART rx channel
extern uint8_t rx_fifo[RX_BUFFER_LENGTH];
extern uint8_t rx_fifo_idx;

// stores flag to indicate UART receptions
extern bool rx_flag;

// stores most recent index of terminating byte in a valid UART packet
extern uint8_t valid_packet_end_idx;

// stores 

/*
        initialize the uart module with the appropriate frame settings and baud rate
*/
void uart0_init();

/*
        extracts a length N speed packet from a larger uint8_t fifo ending with a global terminating index
*/
void extract_speed_packet(int16_t *buf, int N);

/*
        checks if a UART speed packet exists in the rx_buffer starting at rx_fifo_idx 
        returns true if such a packet exists, false otherwise. 
        updates a global variable to indicate the index of the terminating byte
*/
bool check_full_uart_packet(); 


/*
        given a buffer of rpms, create and transmit the UART packet
*/
void send_speed_packet(float* buf, int N);