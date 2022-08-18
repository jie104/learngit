#include <math.h>
#include <pthread.h>
#include <signal.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/time.h>
#include <unistd.h>

#include "api_uart.h"
#include "sr_ls_imu.h"

#define DATA_LENGTH (25) /* 23 */
#define UART_BUF_MAX_SIZE (1024 * 20)
#define DATA_MIN_LEN (20)

#define HEAD_SYNC1 (0x7F)
#define HEAD_SYNC2 (0x93) /* 0x80 */

struct sr_ring {
    uint32_t front;
    uint32_t rear;

    uint8_t buffer[UART_BUF_MAX_SIZE];
};

static int32_t fd = -1;
static pthread_t tid_imu_data = 0;
static pthread_t tid_imu_parse = 0;

static char dev_name[64] = {'\0'};
static struct sr_ring ring = {0};

static lsimu_data_t imu_data = {0};
static imu_handler imu_handle;
static volatile bool run_flag = true;

static struct timeval tv;
static pthread_mutex_t thread_mutex = PTHREAD_MUTEX_INITIALIZER;
static pthread_rwlock_t rwlock_imu;

static long long timestamp2;

/**
 * 解析B4数据包
 *
 * @param[in]	data  : 用于存储IMU数据的变量
 * @param[in]	buffer: IMU具体数据缓冲区
 * @param[in]	length:解析数据的长度
 *
 * @return		0成功，其他失败
 */
int32_t parse_packet_b4(lsimu_data_t *data, uint8_t *buffer, int32_t length) {
    static volatile long long tmp_timestamp = 0;
    static volatile long long tmp_raw_timestamp = 0;
    static uint16_t raw_timestamp_last = 0;
    static uint32_t timeout_cnt = 0;

    static const int32_t Sensor_Scale = 65536;
    static const int32_t Angle_Scale = 360;
    static const int32_t Accel_Scale = 20;
    static const int32_t Rate_Scale = 1260;
    static const int32_t Temp_Scale = 200;
    double sensors[11] = {0};
    static uint32_t filter = 0;

    if (data == NULL || buffer == NULL || length < DATA_MIN_LEN) {
        return -1;
    }

    memset(sensors, 0, sizeof(sensors));
    sensors[0] = (short)(buffer[0] + (buffer[1] << 8));
    sensors[1] = (short)(buffer[2] + (buffer[3] << 8));
    sensors[2] = (short)(buffer[4] + (buffer[5] << 8));
    sensors[3] = (short)(buffer[6] + (buffer[7] << 8));
    sensors[4] = (short)(buffer[8] + (buffer[9] << 8));
    sensors[5] = (short)(buffer[10] + (buffer[11] << 8));
    sensors[6] = (short)(buffer[12] + (buffer[13] << 8));
    sensors[7] = (short)(buffer[14] + (buffer[15] << 8));
    sensors[8] = (short)(buffer[16] + (buffer[17] << 8));
    sensors[9] = (short)(buffer[18] + (buffer[19] << 8));
    sensors[10] = (uint16_t)(buffer[20] + (buffer[21] << 8));

    gettimeofday(&tv, NULL);
    if (filter < 5) {  // 上电矫正数据准确性
        tmp_timestamp = ((long long)tv.tv_sec) * 1000 + tv.tv_usec / 1000;
        raw_timestamp_last = sensors[10];
        ++filter;
        return -2;
    }

    pthread_mutex_lock(&thread_mutex);
    // Accel x, y, z
    data->Ax = sensors[0] * Accel_Scale / Sensor_Scale;
    data->Ay = sensors[1] * Accel_Scale / Sensor_Scale;
    data->Az = sensors[2] * Accel_Scale / Sensor_Scale;

    // Gyro x, y, z
    data->Gx = sensors[3] * Rate_Scale / Sensor_Scale;
    data->Gy = sensors[4] * Rate_Scale / Sensor_Scale;
    data->Gz = sensors[5] * Rate_Scale / Sensor_Scale;

    // Roll, Pitch, Yaw
    data->Roll = sensors[6] * Angle_Scale / Sensor_Scale;
    data->Pitch = sensors[7] * Angle_Scale / Sensor_Scale;
    data->Yaw = sensors[8] * Angle_Scale / Sensor_Scale;

    // Temp
    data->Temp = sensors[9] * Temp_Scale / Sensor_Scale;

    // raw timestamp
    if ((sensors[10] - raw_timestamp_last) < 0) {
        tmp_raw_timestamp++;
    }
    raw_timestamp_last = sensors[10];
    data->raw_timestamp = tmp_raw_timestamp * 65535 + sensors[10];

    data->timestamp = ((long long)tv.tv_sec) * 1000 + tv.tv_usec / 1000;
    // data->timestamp = timestamp2;
    pthread_mutex_unlock(&thread_mutex);

    if (data->timestamp - tmp_timestamp > 50) {
        timeout_cnt++;
        printf("warning: %s timeout = %lld ms, cnt = %d\n", __FUNCTION__, data->timestamp - tmp_timestamp, timeout_cnt);
    }
    tmp_timestamp = data->timestamp;

    if (imu_handle) {
        imu_handle(data);
    }

    return 0;
}

int32_t data_check_sum(uint8_t *buffer, int32_t length) {
    int32_t ret = -1;
    uint8_t check = 0;

    if (buffer == NULL || length < DATA_LENGTH) {
        return -1;
    }

    for (int32_t i = 2; i < DATA_LENGTH - 1; i++) {
        check += buffer[i];
    }
    check = ~check;
    if (check == buffer[DATA_LENGTH - 1]) {
        return 0;
    } else {
        debug_log("data_check_sum", buffer, DATA_LENGTH);
        printf("error, %s, check = 0x%x, ~check = 0x%x, ret = %d\n", __FUNCTION__, ~check, check, ret);
        return -2;
    }

    return ret;
}

/**
 * 解析IMU数据
 *
 * @param[in]	data  : 存储IMU数据的变量
 * @param[in]	buffer: 串口接收IMU具体数据缓冲区
 * @param[in]	length:缓冲区有效数据的长度
 *
 * @return		0成功，其他失败
 */

int32_t parse_imu_data(lsimu_data_t *data, uint8_t *buffer, int32_t length) {
    int32_t ret = -1;
    uint8_t check = 0;

    if (data == NULL || buffer == NULL || length < DATA_LENGTH) {
        return -1;
    }

    for (int32_t i = 2; i < DATA_LENGTH - 1; i++) {
        check += buffer[i];
    }
    check = ~check;
    if (check == buffer[DATA_LENGTH - 1]) {
        ret = parse_packet_b4(data, buffer + 2, DATA_LENGTH - 2);
    } else {
        debug_log("parse", buffer, DATA_LENGTH);
        printf("error, check sum %s, check = 0x%x, ~check = 0x%x, ret = %d\n", __FUNCTION__, ~check, check, ret);
        return -2;
    }

    return ret;
}

#if 0
/**
 * 解析IMU数据
 *
 * @param[in]	data  : 存储IMU数据的变量
 * @param[in]	buffer: 串口接收IMU具体数据缓冲区
 * @param[in]	length:缓冲区有效数据的长度
 *
 * @return		0成功，其他失败
 */

int32_t ParseImuData(lsimu_data_t* data, uint8_t* buffer, int32_t length)
{
	int32_t index = 0;
	
	if (data == NULL || buffer == NULL || length < DATA_LENGTH) {
		return -1;
	}
	
	while (length >= DATA_LENGTH) {
		
		if (buffer[index++] == 0x7F && buffer[index++] == 0x80) {
			uint8_t sum = buffer[DATA_LENGTH - 1];
			uint8_t check = 0;
			for (int32_t i = 2; i < DATA_LENGTH-1; i++) {
				check += buffer[i];
			}
			check = ~check;
			if (check == sum) {
				ParsePacketB4(data, buffer + 2, DATA_LENGTH - 2);
			} 
			index = 0;
			length = length - DATA_LENGTH;
			memcpy(buffer, buffer + DATA_LENGTH, length);
		} else {
			index = 0;
			length = length - 1;
			memcpy(buffer, buffer + 1, length);
		}
	}
	
	return 0;
}
#endif

#if 0
static void handler(int32_t signum)
{
	printf("signal imu_close\r\n");
	imu_close();
}
#endif

static int32_t uart_event_register(imu_handler handler, uint32_t *cookie) {
    if (NULL == handler) return -1;
    imu_handle = handler;
    return 0;
}

void set_timer(void) {
    struct itimerval itv, oldtv;
    itv.it_interval.tv_sec = 0;
    itv.it_interval.tv_usec = 1000;
    itv.it_value.tv_sec = 0;
    itv.it_value.tv_usec = 1000;

    setitimer(ITIMER_REAL, &itv, &oldtv);
}

void sigalrm_handler(int sig) { timestamp2++; }

static void *pthread_imu_data(imu_handler cb) {
    struct uart_para up;
    int32_t ret = -1;
    int32_t nlen = 0;
	
    memset(&ring, 0, sizeof(ring));
    memset(&up, 0, sizeof(up));
    up.speed = B460800;
    fd = uart_open(dev_name, &up);
    if (fd < 0) {
        printf("error, uart_open, fd=%d\n", fd);
        return NULL;
    }

    if ((ret = uart_event_register(cb, NULL)) < 0) {
        printf("error, uart_event_register, ret=%d\r\n", ret);
        return NULL;
    }

    printf("enter %s\r\n", __FUNCTION__);

    while (run_flag) {
        nlen = uart_recv(fd, &ring.buffer[ring.rear], DATA_LENGTH*50, 4);

//        if (nlen < DATA_MIN_LEN) {
//            debug_log("recv1", &ring.buffer[ring.rear], nlen);
 //           continue;
 //       } else {
            if ((ring.rear + nlen) % UART_BUF_MAX_SIZE != ring.front) {
                //debug_log("recv2", &ring.buffer[ring.rear], nlen);
				#if 0
                ret = data_check_sum(&ring.buffer[ring.rear], nlen);
                if (ret < 0) {
                    continue;
                }
				#endif

                pthread_rwlock_wrlock(&rwlock_imu);
                ring.rear = (ring.rear + nlen) % UART_BUF_MAX_SIZE;
                pthread_rwlock_unlock(&rwlock_imu);
                // printf("front = %d, rear = %d\n", ring.front, ring.rear);
            }
//        }
    }

    printf("exit %s success\r\n", __FUNCTION__);
    return NULL;
}

static void *pthread_imu_parse(void *arg) {
    int32_t ret = -1;
	int32_t diff = 0;

    // signal(SIGINT, handler);
    signal(SIGALRM, sigalrm_handler);
    set_timer();

    printf("enter %s\r\n", __FUNCTION__);

    while (run_flag) {

        if (abs(ring.rear - ring.front) >= DATA_LENGTH) {
            if (ring.buffer[ring.front] == HEAD_SYNC1 &&
                ring.buffer[(ring.front + 1) % UART_BUF_MAX_SIZE] == HEAD_SYNC2) {
                debug_log("recv3", &ring.buffer[ring.front], DATA_LENGTH);

                memset(&imu_data, 0, sizeof(imu_data));
                ret = parse_imu_data(&imu_data, &ring.buffer[ring.front], DATA_LENGTH);
                if (ret < 0) {
                    ring.buffer[ring.front] = 0;
                    ring.front = (ring.front + 1) % UART_BUF_MAX_SIZE;
                    continue;
                }

                memset(&ring.buffer[ring.front], 0, DATA_LENGTH);
                ring.front = (ring.front + DATA_LENGTH) % UART_BUF_MAX_SIZE;
            } else {
                ring.buffer[ring.front] = 0;
                ring.front = (ring.front + 1) % UART_BUF_MAX_SIZE;
            }
        }
        usleep(100);
    }

    printf("exit %s success\r\n", __FUNCTION__);
    return NULL;
}

/*
 * pthread 线程有两种状态，joinable（非分离）状态和detachable（分离）状态，默认为joinable。
 * joinable：当线程函数自己返回退出或pthread_exit时都不会释放线程所用资源，包括栈，线程描述符等
 * 因此，joinable 线程执行完后不使用pthread_join的话就会造成内存泄漏。
 * 解决办法：
 * 1.创建线程前设置 PTHREAD_CREATE_DETACHED 属性
 */
int32_t imu_open(const char *dev, imu_handler cb) {
    if (!dev) return -1;

    pthread_attr_t attr;
    pthread_attr_init(&attr);
    pthread_attr_setdetachstate(&attr, PTHREAD_CREATE_DETACHED);

    strcpy(dev_name, dev);
    pthread_rwlock_init(&rwlock_imu, NULL);

    if (pthread_create(&tid_imu_data, &attr, (void *)pthread_imu_data, cb) < 0) {
        printf("error, pthread create tid_imu failed\r\n");
        return -1;
    }
    usleep(2000);
    if (pthread_create(&tid_imu_parse, &attr, pthread_imu_parse, NULL) < 0) {
        printf("error, pthread create tid_imu failed\r\n");
        return -1;
    }
    pthread_attr_destroy(&attr);
    return 0;
}

int32_t imu_close(void) {
    run_flag = false;
    pthread_mutex_destroy(&thread_mutex);
    pthread_rwlock_destroy(&rwlock_imu);
    return uart_close(fd);
}
