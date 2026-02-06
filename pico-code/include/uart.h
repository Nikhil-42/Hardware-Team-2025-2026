#pragma once
#include <stdint.h>
#include <stdbool.h>
#include "include/kinematics.h"
#include "include/odom.h"

#define UART0_BAUD 115200
#define UART0_DATA 8
#define UART0_STOP 1
#define UART1_BAUD 115200
#define UART1_DATA 8
#define UART1_STOP 1
#define SPEED_DATA_VALUES_COUNT 3
#define POSITION_DATA_VALUES_COUNT 3
#define BYTES_PER_DATA_VALUE 4
#define TOTAL_SPEED_DATA_BYTES (SPEED_DATA_VALUES_COUNT * BYTES_PER_DATA_VALUE)
#define TOTAL_POSITION_DATA_BYTES (POSITION_DATA_VALUES_COUNT * BYTES_PER_DATA_VALUE)
#define BYTES_IN_SPEED_TRANSMISSION (TOTAL_SPEED_DATA_BYTES + 4) // for two start and two end byte 
#define BYTES_IN_SENT_PACKET (TOTAL_SPEED_DATA_BYTES + TOTAL_POSITION_DATA_BYTES + 4) 
#define RX_BUFFER_LENGTH 100

#define SPEED_START_BYTE_H 0xFF
#define SPEED_START_BYTE_L 0xFF
#define SPEED_END_BYTE_H 0xEE
#define SPEED_END_BYTE_L 0xEE

// stores flag to indicate UART receptions
extern bool cmd_ready;
extern robot_velocities_t cmd_vel;

/*
        initialize the uart module with the appropriate frame settings and baud rate
*/
void uart0_init();

/*
        initialize the uart module with the appropriate frame settings and baud rate
*/
void uart1_init();

/*
        extracts robot velocities from a larger uint8_t fifo ending with a global terminating index
*/
void extract_speed_packet(robot_velocities_t *robo_v);

/*
        checks if a UART speed packet exists in the rx_buffer starting at rx_fifo_idx 
        returns true if such a packet exists, false otherwise. 
        updates a global variable to indicate the index of the terminating byte
*/
bool check_full_uart_packet(); 

/*
        given robot velocities and pose, 
*/
void send_speed_packet(robot_velocities_t* robo_v, pose_t* pose);