#include <termios.h>
#include <stdio.h>
#include <fcntl.h>
#include <unistd.h>
#include <string.h>
#include <stdlib.h>
#include <errno.h>
#include <linux/serial.h>
#include <sys/ioctl.h>

#include "api_uart.h"


int32_t debug_log(const char *name, uint8_t *data, int32_t len)
{
#ifdef __DEBUG_LOG
	uint16_t i = 0;

	printf("len=%d, %s:", len, name);
	if (NULL != strstr(name, "pub") || NULL != strstr(name, "pri")){
		for(i=0; i<len; i++){
			printf("%c",data[i]);
		}
	} else {
		for(i=0; i<len; i++){
			printf("%02X ",data[i]);
		//printf("%c",data[i]);
		}
	}
	printf("\n");
	return 0;
#endif
}

#if 0
/*
 * tk1物理串口：/dev/ttyTHS0
 * 虚拟串口：/dev/ttyGS0
 * usb串口：/dev/ttyUSB0
 */
int32_t uart_open(char* devname, struct uart_para *up)
{
	int32_t tmp_fd;
	int32_t err;
	struct termios curr_term;

	tmp_fd = open(devname, O_RDWR | O_NOCTTY | O_NDELAY); //O_NDELAY(O_NONBLOCK)
	if (tmp_fd < 0){
		printf("open  err %d  %s \n",  errno, strerror(errno));
		return -1;
	}

	speed_t speed = up->speed;

	if ((err = tcgetattr(tmp_fd, &curr_term)) != 0) {
		printf("open: tcgetattr(%d) = %d,  error: %s\r\n",
				tmp_fd, err, strerror(errno));
		close(tmp_fd);
		return -2;
	}

	// init serial
	bzero(&curr_term, sizeof(curr_term));
	curr_term.c_cflag = (curr_term.c_cflag & ~CSIZE) | CS8;
	curr_term.c_cflag |= B115200;

	if (tcsetattr (tmp_fd, TCSANOW, &curr_term) != 0)
	{
		fprintf (stderr, "error %d from tcsetattr", errno);
		return -3;
	}

	return tmp_fd;
}
#endif

#if 1
/*
 * tk1物理串口：/dev/ttyTHS0
 * 虚拟串口：/dev/ttyGS0
 * usb串口：/dev/ttyUSB0
 */
int32_t uart_open(char* devname, struct uart_para *up)
{
	int32_t tmp_fd;
	struct termios curr_term;
	int32_t err;

	tmp_fd = open(devname, O_RDWR | O_NOCTTY | O_NDELAY); //O_NDELAY(O_NONBLOCK)
	if(tmp_fd < 0){
		printf("open  err %d  %s \n",  errno, strerror(errno));
		return -1;
	}

	speed_t speed = up->speed;

	if ((err = tcgetattr(tmp_fd, &curr_term)) != 0) {
		printf("open: tcgetattr(%d) = %d,  error: %s\r\n",
				tmp_fd, err, strerror(errno));
		close(tmp_fd);
		return -2;
	}

	// init serial
	bzero(&curr_term, sizeof(curr_term));
	curr_term.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR
			| ICRNL | IXON);
	curr_term.c_oflag &= ~OPOST;
	curr_term.c_lflag &= ~(ECHO | ECHONL | ICANON | ISIG | IEXTEN);
	curr_term.c_cflag &= ~(CSIZE | PARENB);
	curr_term.c_cflag |= CS8;
	curr_term.c_cflag &= ~CRTSCTS;

	tcsetattr(tmp_fd, TCSANOW, &curr_term);
	tcflush(tmp_fd, TCIOFLUSH);
	tcsetattr(tmp_fd, TCSANOW, &curr_term);
	tcflush(tmp_fd, TCIOFLUSH);

	curr_term.c_cflag &= ~PARENB;
	curr_term.c_iflag &= ~INPCK;
	curr_term.c_cflag &= ~CSTOPB;
	curr_term.c_cflag &= ~CSIZE;
	curr_term.c_cflag |= CS8;
	curr_term.c_cflag &= ~CRTSCTS;
	curr_term.c_iflag &= ~(IXON | IXOFF | IXANY);
	curr_term.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);

	//disable cr lf swape
	curr_term.c_iflag &= ~(INLCR | ICRNL | IGNCR);
	curr_term.c_oflag &= ~(ONLCR | OCRNL);

	if (cfsetispeed(&curr_term, speed)) {
		printf("Set in speed error\n");
		close(tmp_fd);
		return -3;
	}

	if (cfsetospeed(&curr_term, speed)) {
		printf("Set out speed error\n");
		close(tmp_fd);
		return -4;
	}
	tcsetattr(tmp_fd, TCSANOW, &curr_term);

	return tmp_fd;
}
#endif

int32_t uart_send(int32_t fd, uint8_t *send, int32_t length)
{
	int32_t dwPos = 0;
	int32_t ret = 0;
	int32_t rty_cnts = 3;

	tcflush(fd, TCIOFLUSH);

	if (send != NULL) {
		do {
			ret = write(fd, (send + dwPos), (length - dwPos));
			if (ret > 0) {
				rty_cnts = 3;
				//printf("WriteComm ret = %d\n", ret);
				dwPos += ret;
			} else {
				if(errno == EAGAIN){
					if(rty_cnts--){
						printf("Sleep & Try again\n");
						usleep(1000);
						continue;
					}
				}else{
					printf("write error, fd = %d, err = %s, ret = %d\n", fd, strerror(errno), ret);
					return -1;
				}
			}
		} while (dwPos < length);
	} else {
		printf("ndl_write lpBuf is NULL!!!\n");
		return -1;
	}

	return 0;

}

int32_t uart_recv(int32_t fd, uint8_t *recv, int32_t length, int32_t timeout)
{
	int32_t ret = 0;
	struct timeval tv;

	int32_t btotal = 0;
	int32_t bread = 0;
	uint8_t *buf = recv;
	fd_set rfds;
	
	while (btotal < length) {
		FD_ZERO(&rfds);
		FD_SET(fd, &rfds);
		tv.tv_sec = timeout / 1000; //set the rcv wait time
		tv.tv_usec = (timeout % 1000) * 1000; //100*1000us = 0.1s

		ret = select(fd + 1, &rfds, NULL, NULL, &tv);
		if((ret < 0) && (errno != EINTR)){
			printf("Select failed with error %d\n", errno);
			return -1;
		}
		if(ret == 0){
			if(btotal > 0)
				break;
			//printf("select ret == 0\n");
			return -2;
		}
		if(ret){
			if(FD_ISSET(fd, &rfds)){
				if((bread = read(fd, buf, length - btotal)) > 0){
					btotal += bread;
					buf += bread;
				}
				if(bread < 0){
					if(errno != EWOULDBLOCK){
						printf("select ret == 0, bread < 0!!!\n");
						return -3;
					}
				}
			}
		}
	}
	if(btotal <= 0){
		printf("select btotal <= 0!!!\n");
		return -4;
	} else{
		return btotal;
	}
}


/*
 * queue_selector:
   TCIFLUSH: flushes data received but not read.
   TCOFLUSH: flushes data written but not transmitted.
   TCIOFLUSH: flushes both data received but not read, and data written but not transmitted.
 */
void uart_flush(int32_t fd, int32_t queue_selector)
{
	tcflush(fd, queue_selector);   
}

int32_t uart_close(int32_t fd)
{
	if(fd > 0){
		return close(fd);
	}
	return -1;
}


int32_t std_uart_485(int32_t fd)
{
	struct serial_rs485 rs485conf;

	memset(&rs485conf, 0, sizeof(rs485conf));
	
	/* Enable RS485 mode: */
	rs485conf.flags |= SER_RS485_ENABLED;

	/* Set logical level for RTS pin equal to 0 when sending: */
	rs485conf.flags |= SER_RS485_RTS_ON_SEND;
	/* or, set logical level for RTS pin equal to 1 when sending: */
	//rs485conf.flags &= ~(SER_RS485_RTS_ON_SEND);

	/* Set logical level for RTS pin equal to 0 after sending: */
	//rs485conf.flags |= SER_RS485_RTS_AFTER_SEND;
	/* or, set logical level for RTS pin equal to 1 after sending: */
	rs485conf.flags &= ~(SER_RS485_RTS_AFTER_SEND);

	/* Set rts delay before send, if needed:(in microseconds) */
	rs485conf.delay_rts_before_send = 100;

	/* Set rts delay after send, if needed: (in microseconds) */
	rs485conf.delay_rts_after_send = 100;

	/* Set this flag if you want to receive data even whilst sending data */
	rs485conf.flags |= SER_RS485_RX_DURING_TX;

	if (ioctl(fd, TIOCSRS485, &rs485conf) < 0) {
		printf("Error: TIOCSRS485 ioctl not supported.\n");
		return -1;
	}
	else
		return 0;
}


