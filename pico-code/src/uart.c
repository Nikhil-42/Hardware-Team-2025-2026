#include <stdio.h>
#include <inttypes.h>
#include <memory.h>
#include "include/kinematics.h"
#include "include/odom.h"
#include "include/uart.h"
#include "hardware/uart.h"
#include "hardware/gpio.h"
#include "hardware/irq.h"

typedef enum {
    READING_0 = 0,
    READING_1 = 1,
    READING_2 = 2,
    READING_3 = 3,
    READING_4 = 4,
    READING_5 = 5,
    READING_6 = 6,
    READING_7 = 7,
    READING_8 = 8,
    READING_9 = 9,
    READING_10 = 10,
    READING_11 = 11,
    COMMIT = 12,
    COMMIT_VERIFY = 13,
    SCANNING = 255,
    SCANNING_VERIFY = 254,
} rx_state_t;

rx_state_t rx_state = SCANNING;
uint8_t rx_buffer[12];

bool cmd_ready = false;
robot_velocities_t cmd_vel = {0.0f, 0.0f, 0.0f};

void handle_received_byte(uint8_t rx_c) {
        switch (rx_state) {
                case SCANNING:
                        if (rx_c == SPEED_START_BYTE_L) {
                                rx_state = SCANNING_VERIFY;
                        }
                        break;
                case SCANNING_VERIFY:
                        if (rx_c == SPEED_START_BYTE_H) {
                                rx_state = READING_0;
                        } else {
                                rx_state = SCANNING;
                        }
                        break;
                case READING_0: case READING_1: case READING_2: case READING_3:
                case READING_4: case READING_5: case READING_6: case READING_7:
                case READING_8: case READING_9: case READING_10: case READING_11:
                        rx_buffer[rx_state] = rx_c;
                        rx_state++;
                        break;
                case COMMIT:
                        if (rx_c == SPEED_END_BYTE_L) {         
                                rx_state = COMMIT_VERIFY;
                        } else {
                                rx_state = SCANNING;
                        }
                        break;
                case COMMIT_VERIFY:
                        if (rx_c == SPEED_END_BYTE_H && cmd_ready == false) {
                                // full packet received
                                memcpy(&cmd_vel, rx_buffer, sizeof(robot_velocities_t));
                                rx_state = SCANNING;
                                cmd_ready = true;
                        } else {
                                rx_state = SCANNING;
                        }
                        break;
                default:
                        rx_state = SCANNING;
                        break;
        }
}

void uart0_rx_isr()
{
        handle_received_byte(uart_getc(uart0));
}

void uart1_rx_isr()
{
        handle_received_byte(uart_getc(uart1));
}

void uart0_init()
{
        // configure GPIO 0 and 1 to be UART 0 TX and RX
        gpio_set_function(0, UART_FUNCSEL_NUM(uart0, 0));
        gpio_set_function(1, UART_FUNCSEL_NUM(uart0, 1));

        // set the baud rate of the uart channel
        uart_init(uart0, UART0_BAUD); 

        // disable use of CTS/RTS lines 
        uart_set_hw_flow(uart0, false, false);
        
        // format the tranmission frames 
        uart_set_format(uart0, UART0_DATA, UART0_STOP, UART_PARITY_NONE);

        // disable UART's FIFO to use our own (maybe unecessary)
        uart_set_fifo_enabled(uart0, false);

        // set properties of the ISR
        irq_set_exclusive_handler(UART0_IRQ, uart0_rx_isr);
        irq_set_enabled(UART0_IRQ, true);

        // enable RX interrupts on the uart channel
        uart_set_irqs_enabled(uart0, true, false);

}

void uart1_init()
{
        // configure GPIO 0 and 1 to be UART 0 TX and RX
        gpio_set_function(4, UART_FUNCSEL_NUM(uart1, 4));
        gpio_set_function(5, UART_FUNCSEL_NUM(uart1, 5));

        // set the baud rate of the uart channel
        uart_init(uart1, UART1_BAUD); 

        // disable use of CTS/RTS lines 
        uart_set_hw_flow(uart1, false, false);
        
        // format the tranmission frames 
        uart_set_format(uart1, UART1_DATA, UART1_STOP, UART_PARITY_NONE);

        // disable UART's FIFO to use our own (maybe unecessary)
        uart_set_fifo_enabled(uart1, false);

        // set properties of the ISR
        irq_set_exclusive_handler(UART1_IRQ, uart1_rx_isr);
        irq_set_enabled(UART1_IRQ, true);

        // enable RX interrupts on the uart channel
        uart_set_irqs_enabled(uart1, true, false);       
}

void send_speed_packet(robot_velocities_t* robo_v, pose_t* pose)
{
        uint8_t packet[BYTES_IN_SENT_PACKET]; 

        // assign each data point to buffer location so that it can be easily iterated over
        float buf[6]; 
        buf[0] = robo_v->vx;
        buf[1] = robo_v->vy;
        buf[2] = robo_v->wz;
        buf[3] = pose->x;
        buf[4] = pose->y;
        buf[5] = pose->theta;  

        // add start and stop sync bytes to packet
        packet[0] = SPEED_START_BYTE_H;
        packet[1] = SPEED_START_BYTE_L;
        packet[BYTES_IN_SENT_PACKET - 2] = SPEED_END_BYTE_H;
        packet[BYTES_IN_SENT_PACKET - 1] = SPEED_END_BYTE_L;

        for(int i = 0; i < POSITION_DATA_VALUES_COUNT; i++)
        {
                int32_t buf_i = (int32_t)buf[i];
                uint8_t packet_msb = (uint8_t)((buf_i >> 24) & 0xFF);
                uint8_t packet_byte2 = (uint8_t)((buf_i >> 16) & 0xFF);
                uint8_t packet_byte1 = (uint8_t)((buf_i >> 8) & 0xFF);
                uint8_t packet_lsb = (uint8_t)(buf_i & 0xFF);
                packet[4*i + 2] = packet_msb;
                packet[4*i + 3] = packet_byte2;
                packet[4*i + 4] = packet_byte1; 
                packet[4*i + 5] = packet_lsb; 
        }

        uart_write_blocking(uart0, packet, BYTES_IN_SENT_PACKET);
}