#ifndef _hal_cv_h
#define _hal_cv_h

#include "stdio.h"
#include "string.h"
#include "port.h"

void h_uart_init(int number, int baudrate);
void h_uart_transmitt(int number, unsigned char * buffer, int size);
unsigned char h_uart_receive(int number);

template<typename ...T>
void h_uart_printf(int number, const char * format, T ... ts)
{
    char buffer[256];
    snprintf(buffer, sizeof(buffer), format, ts ...);
    auto size = strlen(buffer);
    snprintf(&buffer[size], sizeof(buffer) - size, "%s", "\r\n");

    h_uart_transmitt(number, (unsigned char *)buffer, strlen(buffer));
}

void h_dma_init();
void h_dma_m2m(void * destination, void * source, int size);
void h_dma_z2m(void * destination, int size);

void h_fpga_bridge_init(int bridge);
void h_fpga_bridge_write(int bridge, unsigned int offset, unsigned int value);
unsigned int h_fpga_bridge_read(int bridge, unsigned int offset);
void h_fpga_configure(unsigned char * bitstream, int size);

void h_sd_init();
void h_sd_write(unsigned int address, void * source, int size);
void h_sd_read(unsigned int address, void * destination, int size);

#endif