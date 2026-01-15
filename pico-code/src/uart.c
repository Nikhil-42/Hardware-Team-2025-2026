#include <stdio.h>
#include <inttypes.h>
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
        uint8_t rx_c = uart_getc(uart0);
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

void extract_speed_packet(int16_t *buf, int N)
{
        // starting index in circular fifo
        uint8_t starting_idx = (valid_packet_end_idx - BYTES_IN_SPEED_TRANSMISSION + 3) % RX_BUFFER_LENGTH;
        for(int i = 0; i < N; i++)
        {
                uint8_t high_byte_idx = (starting_idx + 2*i) % RX_BUFFER_LENGTH; 
                uint8_t low_byte_idx = (starting_idx + 2*i + 1) % RX_BUFFER_LENGTH;
                int16_t buf_i = (((int16_t) rx_fifo[high_byte_idx]) << 8) | ((int16_t) rx_fifo[low_byte_idx]);
                buf[i] = buf_i; 
                printf("Extracted speed: ");
                printf("%" PRId16 "\n", buf_i);
        }
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
        if((rx_fifo[(rx_fifo_idx - SPEED_DATA_BYTES - 1) % RX_BUFFER_LENGTH] != SPEED_START_BYTE_L) || (rx_fifo[(rx_fifo_idx - SPEED_DATA_BYTES - 2) % RX_BUFFER_LENGTH] != SPEED_START_BYTE_H))
        {
                valid_packet_end_idx = -1;
                return false; 
        }
        // if both checks are passed, then both start and end bytes are present 
        return true; 
}

void send_speed_packet(float* buf, int N)
{
        uint8_t packet[BYTES_IN_SPEED_TRANSMISSION]; 

        // add start and stop sync bytes to packet
        packet[0] = SPEED_START_BYTE_H;
        packet[1] = SPEED_START_BYTE_L;
        packet[BYTES_IN_SPEED_TRANSMISSION - 2] = SPEED_END_BYTE_H;
        packet[BYTES_IN_SPEED_TRANSMISSION - 1] = SPEED_END_BYTE_L;

        for(int i = 0; i < N; i++)
        {
                int16_t buf_i = (int16_t)buf[i];
                uint8_t packet_hi = (uint8_t)((buf_i >> 8) & 0xFF);
                uint8_t packet_lo = (uint8_t)(buf_i & 0xFF);
                packet[2*i + 2] = packet_hi;
                packet[2*i + 3] = packet_lo; 
        }

        uart_write_blocking(uart0, packet, BYTES_IN_SPEED_TRANSMISSION);
}