#include <stdio.h>
#include <inttypes.h>
#include "include/kinematics.h"
#include "include/odom.h"
#include "include/uart.h"
#include "hardware/uart.h"
#include "hardware/gpio.h"
#include "hardware/irq.h"

const uint8_t SPEED_START_BYTE_H = 0xFF;
const uint8_t SPEED_START_BYTE_L = 0xFF;
const uint8_t SPEED_END_BYTE_H = 0xEE;
const uint8_t SPEED_END_BYTE_L = 0xEE;

uint8_t rx_fifo_idx = 0;
bool rx_flag = false;
uint8_t valid_packet_end_idx = -1;

uint8_t rx_fifo[RX_BUFFER_LENGTH];

void uart0_rx_isr()
{

}

void uart1_rx_isr()
{
        uint8_t rx_c = uart_getc(uart1);
        rx_fifo[rx_fifo_idx] = rx_c;
        rx_fifo_idx = (rx_fifo_idx + 1) % RX_BUFFER_LENGTH;
        rx_flag = true; 
}

void uart0_init()
{
        // configure GPIO 0 and 1 to be UART 0 TX and RX
        //gpio_set_function(0, UART_FUNCSEL_NUM(uart0, 0));
        //gpio_set_function(1, UART_FUNCSEL_NUM(uart0, 1));

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
        //gpio_set_function(0, UART_FUNCSEL_NUM(uart0, 0));
        //gpio_set_function(1, UART_FUNCSEL_NUM(uart0, 1));

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

void extract_speed_packet(robot_velocities_t *robo_v)
{
        float buf[3]; 
        // starting index in circular fifo
        uint8_t starting_idx = (valid_packet_end_idx - BYTES_IN_SPEED_TRANSMISSION + 3) % RX_BUFFER_LENGTH;
        // assume format 
        for(int i = 0; i < 3; i++)
        {
                uint8_t msb_idx = (starting_idx + 4*i) % RX_BUFFER_LENGTH; 
                uint8_t byte2_idx = (starting_idx + 4*i + 1) % RX_BUFFER_LENGTH;
                uint8_t byte1_idx = (starting_idx + 4*i + 2) % RX_BUFFER_LENGTH;
                uint8_t lsb_idx =  (starting_idx + 4*i + 3) % RX_BUFFER_LENGTH; 
                int16_t buf_i = (((int32_t) rx_fifo[msb_idx]) << 24) | (((int32_t) rx_fifo[byte2_idx]) << 16) | (((int32_t) rx_fifo[byte1_idx]) << 8) | (((int32_t) rx_fifo[lsb_idx]));
                buf[i] = (float)buf_i; 
                printf("Extracted speed: ");
                printf("%f\n", buf_i);
        }
        // copy buffer values read from UART to robot velocities 
        robo_v->vx = buf[0]; 
        robo_v->vy = buf[1];
        robo_v->wz = buf[2];
}

bool check_full_uart_packet()
{       
        // assign at beginning to reduce chance of jumping to ISR and losing information
        valid_packet_end_idx = rx_fifo_idx; 

        // check if end byte is formed (i.e., both high and low bytes are present)
        if((rx_fifo[rx_fifo_idx] != SPEED_END_BYTE_L) || (rx_fifo[(rx_fifo_idx - 1) % RX_BUFFER_LENGTH] != SPEED_END_BYTE_H))
        {
                valid_packet_end_idx = -1;
                return false; 
        }
        // check if start byte is formed (i.e., both high and low bytes are present)
        if((rx_fifo[(rx_fifo_idx - TOTAL_SPEED_DATA_BYTES - 2) % RX_BUFFER_LENGTH] != SPEED_START_BYTE_L) || (rx_fifo[(rx_fifo_idx - TOTAL_SPEED_DATA_BYTES - 3) % RX_BUFFER_LENGTH] != SPEED_START_BYTE_H))
        {
                valid_packet_end_idx = -1;
                return false; 
        }
        // if both checks are passed, then both start and end bytes are present 
        return true; 
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

        uart_write_blocking(uart1, packet, BYTES_IN_SENT_PACKET);
}