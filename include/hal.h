#ifndef _hal_cv_h
#define _hal_cv_h

#include "stdio.h"
#include "string.h"
#include "alt_generalpurpose_io.h"

/* ---------------------------------------------| UART |--------------------------------------------- */

bool h_uart_init(int number, int baudrate);
bool h_uart_transmitt(int number, unsigned char * buffer, int size);
int h_uart_receive(int number, unsigned char * buffer);
char h_uart_getch(int number);
bool h_uart_clear(int number);

template<typename ...T>
bool h_uart_transmitt(int number, const char * format, T ... ts)
{
    char buffer[128];

    snprintf(buffer, 128, format, ts...);

    return h_uart_transmitt(number, (unsigned char *) buffer, strlen(buffer));
}

/* ---------------------------------------------| DMA |--------------------------------------------- */

void h_dma_init();
void h_dma_m2m(void * destination, void * source, int size);
void h_dma_z2m(void * destination, int size);

/* ---------------------------------------------| FPGA |--------------------------------------------- */

bool h_fpga_init();
bool h_fpga_configure(unsigned char * bitstream, int size);

/* ---------------------------------------------| SD |--------------------------------------------- */

bool h_sd_init();
bool h_sd_write(unsigned int address, void * source, int size);
bool h_sd_read(unsigned int address, void * destination, int size);

/* ---------------------------------------------| GPIO |--------------------------------------------- */

// 0 A, 1 B, 2 C

ALT_GPIO_PORT_t _h_gpio_pin2port(int pin);

bool h_gpio_init();
bool h_gpio_configure(int pin, bool input = false, bool interrupt = false);

void h_gpio_set(int pin, bool value);
bool h_gpio_get(int pin);
void h_gpio_toogle(int pin);

bool h_gpio_is_irq_pending(int pin);
void h_gpio_clear_irq_pending(int pin);

/* ---------------------------------------------| SPI |--------------------------------------------- */

bool h_spi_init(int speed);
bool h_spi_wait();
bool h_spi_read(unsigned char * buffer, int size, int slave);
bool h_spi_write(unsigned char * buffer, int size, int slave);
bool h_spi_write_read(unsigned char * from, int size_from, unsigned char * to, int size_to, int slave);

/* ---------------------------------------------| QSPI |--------------------------------------------- */

bool h_qspi_init();
bool h_qspi_deinit();
bool h_qspi_erase(unsigned int address);
bool h_qspi_read(int address, int size, unsigned char * buffer);
bool h_qspi_write(int address, int size, unsigned char * buffer);

/* ---------------------------------------------| CLK |--------------------------------------------- */

unsigned int h_clk_mpu();
unsigned int h_clk_periph();

#endif