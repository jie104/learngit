#include <stdint.h>
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <getopt.h>
#include <fcntl.h>
#include <time.h>
#include <sys/stat.h> 
#include <sys/types.h>
#include <sys/ipc.h>
#include <sys/sem.h>
#include <sys/ioctl.h>
#include <linux/ioctl.h>
#include <sys/stat.h>
#include <linux/types.h>
#include <linux/spi/spidev.h>

#define ARRAY_SIZE(a) (sizeof(a) / sizeof((a)[0]))

static const char *device = "/dev/spidev0.0";
static uint32_t mode = 0;
static uint8_t bits = 8;
static uint32_t speed =500000;
static uint16_t delay = 0;
static int fd = 0;

uint8_t default_tx[2] = {0xFF, 0xFF};
uint8_t default_rx[2] = {0x00, 0x00};
static char buf[32] = {0};
static int n = 0;
static int ndelay = 0;

static void rx_fifo(int fd, uint8_t* rx, size_t len)
{
	int i = 0;
	for(i = 0; i < len; i++)
	{
		if(rx[i] != 0) {
			buf[n++] = rx[i];
			if(n >= 32) {
				write(fd, buf, 32);
				n = 0;
				memset(buf, 0, sizeof(buf));
			}
			ndelay = 0;
		} else {
			usleep(ndelay * ndelay * ndelay * 100);
			ndelay++;
			if(ndelay >= 10) {
				ndelay = 10;
			}
		}
	}
}


static void transfer(int fd, uint8_t const *tx, uint8_t const *rx, size_t len)
{
	struct spi_ioc_transfer tr = {
		.tx_buf = (unsigned long)tx,
		.rx_buf = (unsigned long)rx,
		.len = len,
		.delay_usecs = delay,
		.speed_hz = speed,
		.bits_per_word = bits,
	};

	/* 配置SPI传输速率，QUAL:双倍速率，QUAD:4倍速率 */
  if (mode & SPI_TX_QUAD)
    tr.tx_nbits = 4;
  else if (mode & SPI_TX_DUAL)
    tr.tx_nbits = 2;
	if (mode & SPI_RX_QUAD)
		tr.rx_nbits = 4;
	else if (mode & SPI_RX_DUAL)
		tr.rx_nbits = 2;
  if (!(mode & SPI_LOOP)) {
		if (mode & (SPI_TX_QUAD | SPI_TX_DUAL))
			tr.rx_buf = 0;
		else if (mode & (SPI_RX_QUAD | SPI_RX_DUAL))
			tr.tx_buf = 0;
	}
	
  int ret = ioctl(fd, SPI_IOC_MESSAGE(1), &tr);
	if (ret < 1)
		printf("can't send spi message\n");
}

void open_spi() 
{
	int ret = 0;
	fd = open(device, O_RDWR);
	if (fd < 0)
		printf("can't open device\n");

	/* spi mode */
	ioctl(fd, SPI_IOC_WR_MODE32, &mode);
	if (ret == -1)
		printf("can't set spi mode\n");
	ioctl(fd, SPI_IOC_RD_MODE32, &mode);
  	if (ret == -1)
		printf("can't get spi mode\n");
	/* bits per word*/
	ioctl(fd, SPI_IOC_WR_BITS_PER_WORD, &bits);
	if (ret == -1)
		printf("can't set bits per word\n");
	ioctl(fd, SPI_IOC_RD_BITS_PER_WORD, &bits);
	if (ret == -1)
		printf("can't get bits per word\n");
	/*max speed hz*/
	ioctl(fd, SPI_IOC_WR_MAX_SPEED_HZ, &speed);
  	if (ret == -1)
		printf("can't set max speed hz\n");
	ioctl(fd, SPI_IOC_RD_MAX_SPEED_HZ, &speed);
  	if (ret == -1)
		printf("can't get max speed hz\n");
}

int main(int argc, char *argv[])
{
	int32_t ret = 0;

	open_spi();
	printf("spi mode: 0x%x\n", mode);
	printf("bits per word: %d\n", bits);
	printf("max speed: %d Hz (%d KHz)\n", speed, speed/1000);

	ret = mkfifo("/sros/log/srtos_fifo", 0777);
	if(ret < 0){
		perror("error, srtos mkfifo()");
	}
	int fifofd = open("/sros/log/srtos_fifo", O_WRONLY);
/*
	key_t key = ftok("/sros/log", 10);
	int semid = semget(key, 1, IPC_CREAT | 0666);
	
	struct sembuf sp = {
		.sem_num = 0,
		.sem_op = -1,
		.sem_flgndelay = 0,
	};
	struct sembuf sv = {
		.sem_num = 0,
		.sem_op = +1,
		.sem_flg = 0,
	};
*/
	while(1) {
		//int val = semctl(semid, 0, GETVAL);
		//if(val <= 0) {
		//	printf("SROS disabled the SRTOS log show.\n");
		//	semop(semid, &sp, 1);
		//	semop(semid, &sv, 1);
		//	close(fifofd);
		//	fifofd = open("/sros/log/srtos_fifo", O_WRONLY);	//O_WRONLY
		//	printf("SROS enable the SRTOS log show.\n");
		//}
	/*	if(!data_flag) {
			close(fd);
			usleep(100000);
			open_spi();
			usleep(100000);
			ndelay = 0;
			data_flag = 1;
		}
	*/
		memset(default_rx, 0, ARRAY_SIZE(default_rx));
		transfer(fd, default_tx, default_rx, 1);
		rx_fifo(fifofd, default_rx, 1);
		usleep(600);
	}

  close(fd);
  close(fifofd);
	return 0;
}
