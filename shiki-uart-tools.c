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
#include <sys/ioctl.h>
#include <sys/time.h>
#include "shiki-uart-tools.h"

int suart_debug_mode_status = 0;
int suart_write_debug_status = 0;

static void suart_debug(const char *function_name, char *debug_type, char *debug_msg, ...){
    if (suart_debug_mode_status == 1 || strcmp(debug_type, "INFO") != 0){
        struct tm *d_tm;
        struct timeval tm_debug;
        uint16_t msec = 0;
	    va_list aptr;
		
	    gettimeofday(&tm_debug, NULL);
	    d_tm = localtime(&tm_debug.tv_sec);
        msec = tm_debug.tv_usec/1000;
	
	    char* tmp_debug_msg;
        tmp_debug_msg = (char *) malloc(1024*sizeof(char));
        if (tmp_debug_msg == NULL){
            printf("%02d-%02d-%04d %02d:%02d:%02d.%03i ERROR: %s: failed to allocate debug variable memory",
             d_tm->tm_mday, d_tm->tm_mon+1, d_tm->tm_year+1900, d_tm->tm_hour, d_tm->tm_min, d_tm->tm_sec, msec, __func__
            );
            return;
        }
	    va_start(aptr, debug_msg);
	    vsprintf(tmp_debug_msg, debug_msg, aptr);
	    va_end(aptr);
        #ifdef __USE_COLOR__
            if (strcmp(debug_type, "INFO")==0)
                printf("\033[1;32m%02d-%02d-%04d %02d:%02d:%02d.%03d\033[1;34m SUART\033[1;32m %s: %s: %s\033[0m",
                 d_tm->tm_mday, d_tm->tm_mon+1, d_tm->tm_year+1900, d_tm->tm_hour, d_tm->tm_min, d_tm->tm_sec,
                 msec, debug_type, function_name, tmp_debug_msg
                );
            if (strcmp(debug_type, "WEBSERVER INFO")==0)
                printf("\033[1;32m%02d-%02d-%04d %02d:%02d:%02d.%03d\033[1;34m SUART\033[1;32m %s: %s: %s\033[0m",
                 d_tm->tm_mday, d_tm->tm_mon+1, d_tm->tm_year+1900, d_tm->tm_hour, d_tm->tm_min, d_tm->tm_sec,
                 msec, debug_type, function_name, tmp_debug_msg
                );
    	    else if (strcmp(debug_type, "WARNING")==0)
                printf("\033[1;33m%02d-%02d-%04d %02d:%02d:%02d.%03d\033[1;34m SUART\033[1;33m %s: %s: %s\033[0m",
                 d_tm->tm_mday, d_tm->tm_mon+1, d_tm->tm_year+1900, d_tm->tm_hour, d_tm->tm_min, d_tm->tm_sec,
                 msec, debug_type, function_name, tmp_debug_msg
                );
    	    else if (strcmp(debug_type, "ERROR")==0)
                printf("\033[1;31m%02d-%02d-%04d %02d:%02d:%02d.%03d\033[1;34m SUART\033[1;31m %s: %s: %s\033[0m",
                 d_tm->tm_mday, d_tm->tm_mon+1, d_tm->tm_year+1900, d_tm->tm_hour, d_tm->tm_min, d_tm->tm_sec,
                 msec, debug_type, function_name, tmp_debug_msg
                );
            else if (strcmp(debug_type, "CRITICAL")==0)
                printf("\033[1;31m%02d-%02d-%04d %02d:%02d:%02d.%03d\033[1;34m SUART\033[1;31m %s: %s: %s\033[0m",
                 d_tm->tm_mday, d_tm->tm_mon+1, d_tm->tm_year+1900, d_tm->tm_hour, d_tm->tm_min, d_tm->tm_sec,
                 msec, debug_type, function_name, tmp_debug_msg
                );
	    #else
            printf("%02d-%02d-%04d %02d:%02d:%02d.%03d %s: %s: %s",
             d_tm->tm_mday, d_tm->tm_mon+1, d_tm->tm_year+1900, d_tm->tm_hour, d_tm->tm_min, d_tm->tm_sec,
             msec, debug_type, function_name, tmp_debug_msg
            );
        #endif
        if (suart_write_debug_status == 1){
            char write_cmd[128 + strlen(function_name) + strlen(tmp_debug_msg)];
            memset(write_cmd, 0x00, sizeof(write_cmd));
            sprintf(write_cmd, "echo '%02d-%02d-%04d %02d:%02d:%02d.%03d %s: %s: %s' >> debug.dsrc",
             d_tm->tm_mday, d_tm->tm_mon+1, d_tm->tm_year+1900, d_tm->tm_hour, d_tm->tm_min, d_tm->tm_sec,
             msec, debug_type, function_name, tmp_debug_msg
            );
            system(write_cmd);
        }
        free(tmp_debug_msg);
        tmp_debug_msg = NULL;
    }
}

static int8_t suart_set_attrib(int fd, int speed, int blocking_mode, int timeout, int parity){
    struct termios stty_attrib;
    memset (&stty_attrib, 0, sizeof stty_attrib);
    if (tcgetattr(fd, &stty_attrib) != 0){
        suart_debug(__func__, "ERROR", "error %d from tcgetattr\n", errno);
        return -1;
    }

    if (speed == 0){
        cfsetospeed (&stty_attrib, B0);
        cfsetispeed (&stty_attrib, B0);
    }
    else if (speed == 50){
        cfsetospeed (&stty_attrib, B50);
        cfsetispeed (&stty_attrib, B50);
    }
    else if (speed == 75){
        cfsetospeed (&stty_attrib, B75);
        cfsetispeed (&stty_attrib, B75);
    }
    else if (speed == 110){
        cfsetospeed (&stty_attrib, B110);
        cfsetispeed (&stty_attrib, B110);
    }
    else if (speed == 134){
        cfsetospeed (&stty_attrib, B134);
        cfsetispeed (&stty_attrib, B134);
    }
    else if (speed == 150){
        cfsetospeed (&stty_attrib, B150);
        cfsetispeed (&stty_attrib, B150);
    }
    else if (speed == 200){
        cfsetospeed (&stty_attrib, B200);
        cfsetispeed (&stty_attrib, B200);
    }
    else if (speed == 300){
        cfsetospeed (&stty_attrib, B300);
        cfsetispeed (&stty_attrib, B300);
    }
    else if (speed == 600){
        cfsetospeed (&stty_attrib, B600);
        cfsetispeed (&stty_attrib, B600);
    }
    else if (speed == 1200){
        cfsetospeed (&stty_attrib, B1200);
        cfsetispeed (&stty_attrib, B1200);
    }
    else if (speed == 1800){
        cfsetospeed (&stty_attrib, B1800);
        cfsetispeed (&stty_attrib, B1800);
    }
    else if (speed == 2400){
        cfsetospeed (&stty_attrib, B2400);
        cfsetispeed (&stty_attrib, B2400);
    }
    else if (speed == 4800){
        cfsetospeed (&stty_attrib, B4800);
        cfsetispeed (&stty_attrib, B4800);
    }
    else if (speed == 9600){
        cfsetospeed (&stty_attrib, B9600);
        cfsetispeed (&stty_attrib, B9600);
    }
    else if (speed == 19200){
        cfsetospeed (&stty_attrib, B19200);
        cfsetispeed (&stty_attrib, B19200);
    }
    else if (speed == 38400){
        cfsetospeed (&stty_attrib, B38400);
        cfsetispeed (&stty_attrib, B38400);
    }
    else if (speed == 57600){
        cfsetospeed (&stty_attrib, B57600);
        cfsetispeed (&stty_attrib, B57600);
    }
    else if (speed == 115200){
        cfsetospeed (&stty_attrib, B115200);
        cfsetispeed (&stty_attrib, B115200);
    }
    else if (speed == 230400){
        cfsetospeed (&stty_attrib, B230400);
        cfsetispeed (&stty_attrib, B230400);
    }
    else if (speed == 921600){
        cfsetospeed (&stty_attrib, B921600);
        cfsetispeed (&stty_attrib, B921600);
    }
    else {
        suart_debug(__func__, "WARNING", "custom baudrate?\n");
        cfsetospeed (&stty_attrib, ((speed*B9600)/9600));
        cfsetispeed (&stty_attrib, ((speed*B9600)/9600));
    }

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

int suart_write_data(int s_fd, unsigned char* data, int size_of_data){
	return write(s_fd, data, size_of_data);
}

unsigned long suart_check_input_bytes(int s_fd){
    unsigned long input_bytes = 0;
    if (ioctl(s_fd, FIONREAD, &input_bytes) != 0){
        return -1;
    }
    return input_bytes;
}

void suart_flush(int s_fd){
    tcflush(s_fd,TCIOFLUSH);
}

int suart_read(int s_fd, unsigned char* buff){
    int8_t bytes_recv = 0;
    int total_bytes = 0, total_tmp = 0;
    unsigned char *buff_tmp = NULL, *part_receive = NULL;
    buff_tmp = (unsigned char *) malloc(9 * sizeof(unsigned char));
    if (buff_tmp == NULL){
        suart_debug(__func__, "ERROR", "failed to allocate memory\n");
        return -1;
    }
    part_receive = (unsigned char *) malloc(9 * sizeof(unsigned char));
    if (part_receive == NULL){
        suart_debug(__func__, "ERROR", "failed to allocate memory\n");
        free(buff_tmp);
        buff_tmp = NULL;
        return -1;
    }
    memset (buff_tmp, 0x00, 9 * sizeof(char));
    do{
        memset (part_receive, 0x00, 9 * sizeof(char));
    	bytes_recv = read(s_fd, (void *) part_receive, 1);
        if (bytes_recv > 0){
            total_tmp = total_bytes;
            total_bytes = total_bytes + bytes_recv;
            buff_tmp = (unsigned char *) realloc(buff_tmp, (total_bytes + 1) * sizeof(unsigned char));
            memcpy(buff_tmp + total_tmp, part_receive, bytes_recv);
            buff_tmp[total_bytes] = 0x00;
        }
    } while (bytes_recv == 1);
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

unsigned char *suart_read_chuncked_data(
 int s_fd,
 int16_t bytes_per_recv,
 unsigned char* stop_bits,
 int8_t stop_bits_size,
 int16_t *_total_bytes
){
    int16_t bytes_recv = 0;
    int16_t total_bytes = 0;
    unsigned char *buff_tmp = NULL;
    unsigned char *part_receive = NULL;
    buff_tmp = (unsigned char *) malloc((bytes_per_recv + 2) * sizeof(unsigned char));
    if (buff_tmp == NULL){
        suart_debug(__func__, "ERROR", "failed to allocate memory (1)\n");
        *_total_bytes = 0;
        return NULL;
    }
    part_receive = (unsigned char *) malloc((bytes_per_recv + 2) * sizeof(unsigned char));
    if (part_receive == NULL){
        suart_debug(__func__, "ERROR", "failed to allocate memory (2)\n");
        free(buff_tmp);
        buff_tmp = NULL;
        *_total_bytes = 0;
        return NULL;
    }
    memset (buff_tmp, 0x00, bytes_per_recv * sizeof(char));
    do{
        memset (part_receive, 0x00, (bytes_per_recv + 1) * sizeof(char));
    	bytes_recv = read(s_fd, (void *) part_receive, bytes_per_recv);
        if (bytes_recv > 0){
            total_bytes += bytes_recv;
            buff_tmp = (unsigned char *) realloc(buff_tmp, (total_bytes + 2) * sizeof(unsigned char));
            memcpy(buff_tmp + (total_bytes - bytes_recv), part_receive, bytes_recv);
            buff_tmp[total_bytes] = 0x00;
            if (total_bytes > (int16_t) stop_bits_size){
                if (memcmp(buff_tmp + (total_bytes - stop_bits_size), stop_bits, stop_bits_size) == 0){
                    break;
                }
            }
        }
    } while (bytes_recv > 0);
    free(part_receive);
    *_total_bytes = total_bytes;
    part_receive = NULL;
    if (total_bytes == 0){
        free(buff_tmp);
        return NULL;
    }
    else if (total_bytes == 0 && bytes_recv == -1){
        free(buff_tmp);
        suart_debug(__func__, "ERROR", "failed to access file descriptor\n");
        *_total_bytes = -1;
        return NULL;
    }
    return buff_tmp;
}

int suart_get_some_data(int s_fd, unsigned char* buff, int buffer_size){
    int bytes_recv = 0;
    int bytes_request = buffer_size;
    int total_bytes = 0, total_tmp = 0;
    unsigned char *buff_tmp = NULL, *part_receive = NULL;
    buff_tmp = (unsigned char *) malloc((buffer_size + 1) * sizeof(unsigned char));
    if (buff_tmp == NULL){
        suart_debug(__func__, "ERROR", "failed to allocate memory\n");
        return -1;
    }
    part_receive = (unsigned char *) malloc((buffer_size + 1) * sizeof(unsigned char));
    if (part_receive == NULL){
        suart_debug(__func__, "ERROR", "failed to allocate memory\n");
        free(buff_tmp);
        buff_tmp = NULL;
        return -1;
    }
    memset (buff_tmp, 0x00, (buffer_size + 1) * sizeof(char));
    do{
        memset (part_receive, 0x00, (buffer_size + 1) * sizeof(char));
    	bytes_recv = read(s_fd, (void *) part_receive, bytes_request);
        if (bytes_recv > 0){
            total_tmp = total_bytes;
            bytes_request = bytes_request - bytes_recv;
            total_bytes = total_bytes + bytes_recv;
            memcpy(buff_tmp + total_tmp, part_receive, bytes_recv);
            buff_tmp[total_bytes] = 0x00;
            suart_debug(__func__, "INFO", "receive %d (0x%02x) (-%d) bytes from %d bytes\n", total_bytes, buff_tmp[total_bytes - 1], bytes_request, buffer_size);
        }
    } while (bytes_recv > 0 && total_bytes < buffer_size && bytes_recv > 0);
    free(part_receive);
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
        return 0;
    }
    else if(retval != 1){
        suart_debug(__func__, "WARNING", "invalid data\n");
        return 0;
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
