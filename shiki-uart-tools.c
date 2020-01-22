#include <errno.h>
#include <fcntl.h> 
#include <string.h>
#include <termios.h>
#include <unistd.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <time.h>
#include <stdarg.h>
#include "shiki-uart-tools.h"

static void suart_debug(const char *function_name, char *debug_type, char *debug_msg, ...){
	time_t debug_time;
	struct tm *d_tm;
	va_list aptr;
	
	time(&debug_time);
	d_tm = localtime(&debug_time);

	char tmp_debug_msg[500];
	va_start(aptr, debug_msg);
	vsprintf(tmp_debug_msg, debug_msg, aptr);
	va_end(aptr);

	if (strcmp(debug_type, "INFO")==0) printf("\033[1;32m%02d-%02d-%04d %02d:%02d:%02d\033[1;34m S_UART\033[1;32m %s: %s: %s\033[0m", d_tm->tm_mday, d_tm->tm_mon+1, d_tm->tm_year+1900, d_tm->tm_hour, d_tm->tm_min, d_tm->tm_sec, debug_type, function_name, tmp_debug_msg);
	else if (strcmp(debug_type, "WARNING")==0) printf("\033[1;33m%02d-%02d-%04d %02d:%02d:%02d\033[1;34m S_UART\033[1;33m %s: %s: %s\033[0m", d_tm->tm_mday, d_tm->tm_mon+1, d_tm->tm_year+1900, d_tm->tm_hour, d_tm->tm_min, d_tm->tm_sec, debug_type, function_name, tmp_debug_msg);
	else if (strcmp(debug_type, "ERROR")==0) printf("\033[1;31m%02d-%02d-%04d %02d:%02d:%02d\033[1;34m S_UART\033[1;31m %s: %s: %s\033[0m", d_tm->tm_mday, d_tm->tm_mon+1, d_tm->tm_year+1900, d_tm->tm_hour, d_tm->tm_min, d_tm->tm_sec, debug_type, function_name, tmp_debug_msg);
}

static int8_t suart_set_attrib(int fd, int speed, int blocking_mode, int timeout, int parity){
    struct termios stty_attrib;
    memset (&stty_attrib, 0, sizeof stty_attrib);
    if (tcgetattr(fd, &stty_attrib) != 0){
        suart_debug(__func__, "ERROR", "error %d from tcgetattr\n", errno);
        return -1;
    }

    cfsetospeed (&stty_attrib, speed);
    cfsetispeed (&stty_attrib, speed);

    stty_attrib.c_cflag = (stty_attrib.c_cflag & ~CSIZE) | CS8; // 8-bit chars

    stty_attrib.c_iflag &= ~IGNBRK; // disable break processing
    stty_attrib.c_lflag = 0; // no signaling chars, no echo, no canonical processing
    stty_attrib.c_oflag = 0; // no remapping, no delays
    stty_attrib.c_cc[VMIN]  = blocking_mode ? 1 : 0; // blocking mode
    stty_attrib.c_cc[VTIME] = timeout; // 0.5 seconds read timeout

    stty_attrib.c_iflag &= ~(IXON | IXOFF | IXANY); // shut off xon/xoff ctrl

    stty_attrib.c_cflag |= (CLOCAL | CREAD); // ignore modem controls, enable reading
    stty_attrib.c_cflag &= ~(PARENB | PARODD); // shut off parity
    stty_attrib.c_cflag |= parity;
    stty_attrib.c_cflag &= ~CSTOPB;
    stty_attrib.c_cflag &= ~CRTSCTS;
    stty_attrib.c_iflag &= ~(INLCR | ICRNL);

    if (tcsetattr (fd, TCSANOW, &stty_attrib) != 0){
        suart_debug(__func__, "ERROR", "error %d from tcsetattr\n", errno);
        return -1;
    }
    return 0;
}

int suart_write(int s_fd, unsigned char* string){
	return write(s_fd, string, strlen((char *) string));
}

int suart_read(int s_fd, unsigned char* buff){
    int8_t bytes_recv = 0;
    int total_bytes = 0, total_tmp = 0;
    unsigned char *buff_tmp, *part_receive;
    buff_tmp = (unsigned char *) malloc(9 * sizeof(unsigned char));
    if (buff_tmp == NULL){
        suart_debug(__func__, "ERROR", "failed to allocate memory\n");
        return -1;
    }
    part_receive = (unsigned char *) malloc(9 * sizeof(unsigned char));
    if (part_receive == NULL){
        suart_debug(__func__, "ERROR", "failed to allocate memory\n");
        return -1;
    }
    memset (buff_tmp, 0x00, 9 * sizeof(char));
    do{
        memset (part_receive, 0x00, 9 * sizeof(char));
    	bytes_recv = read(s_fd, (void *) part_receive, 8);
        if (bytes_recv > 0){
            total_tmp = total_bytes;
            total_bytes = total_bytes + bytes_recv;
            buff_tmp = (unsigned char *) realloc(buff_tmp, (total_bytes + 1) * sizeof(unsigned char));
            memcpy(buff_tmp + total_tmp, part_receive, bytes_recv);
            buff_tmp[total_bytes] = 0x00;
        }
    } while (bytes_recv == 8);
    free(part_receive);
    tcflush(s_fd,TCIOFLUSH);
    if (total_bytes == 0){
        free(buff_tmp);
        return 0;
    }
    else if (total_bytes == 0 && bytes_recv == -1){
        free(buff_tmp);
        suart_debug(__func__, "ERROR", "failed to access file descriptor\n");
        return -1;
    }
    memcpy(buff, buff_tmp, (total_bytes + 1)*sizeof(char));
    free(buff_tmp);
    return total_bytes;
}


unsigned char suart_getchar(int s_fd){
    unsigned char buff[2];
    int retval = 0;
    retval = read(s_fd, (void *) buff, 1);
    if (retval == 0){
        tcflush(s_fd,TCIOFLUSH);
        return 0;
    }
    else if(retval != 1){
        suart_debug(__func__, "WARNING", "invalid data\n");
        return -1;
    }
    retval = buff[0];
    return retval;
}

int suart_open(const char* port, int baud, int blocking, int timeout){
    int s_fd = 0;
	s_fd = open (port, O_RDWR | O_NOCTTY | O_SYNC);
	if (s_fd < 0)
	{
		suart_debug(__func__, "ERROR", "error %d opening %s: %s\n", errno, port, strerror (errno));
		return -1;
	}
	if (suart_set_attrib(s_fd, baud, blocking, timeout, 0) !=0){
        close(s_fd);
        return -2;
    }
    suart_debug(__func__, "INFO", "port %s : %d succes to open\n", port, s_fd);
    tcflush(s_fd,TCIOFLUSH);
	return s_fd;
}

int8_t suart_close(int s_fd){
    int retval = close(s_fd);
    if (retval != 0){
        suart_debug(__func__, "ERROR", "file descriptor : %d failed to close\n", s_fd);
        return -1;
    }
    return 0;
}
