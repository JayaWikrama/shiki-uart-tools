#ifndef __SHIKI_UART_TOOLS__
#define __SHIKI_UART_TOOLS__

#include <stdint.h>

int suart_write(int s_fd, unsigned char* string);
int suart_read(int s_fd, unsigned char* buffer);
unsigned char suart_getchar(int s_fd);
int suart_open(const char* port, int baud, int blocking, int timeout);
int8_t suart_close(int s_fd);

#endif