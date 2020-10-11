#ifndef __SHIKI_UART_TOOLS__
#define __SHIKI_UART_TOOLS__

#include <stdint.h>

int suart_write_data(int s_fd, unsigned char* data, int size_of_data);
int suart_write(int s_fd, unsigned char* string);
unsigned long suart_check_input_bytes(int s_fd);
int suart_read(int s_fd, unsigned char* buffer);
unsigned char *suart_read_chuncked_data(
 int s_fd,
 int16_t bytes_per_recv,
 unsigned char* stop_bits,
 int8_t stop_bits_size,
 int16_t *_total_bytes
);
int suart_get_some_data(int s_fd, unsigned char* buff, int buffer_size);
unsigned char suart_getchar(int s_fd);
int suart_open(const char* port, int baud, int blocking, int timeout);
int8_t suart_close(int s_fd);
void suart_flush(int s_fd);

#endif