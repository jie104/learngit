//#include 	"Linux_Com.h"

#include <errno.h>
#include <fcntl.h>
#include <pthread.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <termios.h>
#include <unistd.h>
#include "atom_macro.h"
#include "atomprotocol.h"

// int print_flag = 1;
#define iBufferSize 501
#define UARTBufferLength 98304
#define u8 unsigned char
//#define RING_BUFFER_SIZE 1024
/*Baudrate constants*/
#define B9600 0000015     // 9600 b/s
#define B38400 0000017    // 38400 b/s
#define B57600 0010001    // 57600 b/s
#define B115200 0010002   // 115200 b/s
#define B230400 0010003   // 230400 b/s
#define B460800 0010004   // 460800 b/s
#define B500000 0010005   // 500000 b/s
#define B576000 0010006   // 576000 b/s
#define B921600 0010007   // 921600 b/s
#define B1500000 0010012  // 1500000 b/s
#define rxBufferSize 12
int dataReady;
// const static size_t rxBufferSize = 12;
unsigned char strRxBuf[rxBufferSize];  //����Ҫ��unsigned
int nullRecvCounter = 0;
int nullFirstRecvCounter = 0;
pthread_t receiveDataThread;
static char chrUARTBuffers[UARTBufferLength] = {0};
static unsigned long ulUARTBufferStart = 0, ulUARTBufferEnd = 0;

unsigned char *pBufferStart, *pConsumer;
unsigned char *pProducer, *pCurrent, *pBufferEnd;

u8 ringBuf[RING_BUFFER_SIZE];
// u8 tempbuf[RING_BUFFER_SIZE] = { '\0' };
// u32 unProcessBytes, previous_bytes, processedBytes;
unsigned long uLen;
unsigned long ulLen1;
unsigned long ulLen2;
char chrBuffer[iBufferSize] = {0};

void Process();
int nFd;
void *ReceiveCOMData(void *arg);
signed char OpenCOMDevice(const char *Port, const int iBaudrate);

unsigned short CollectUARTData(char chrUARTBufferOutput[]) {
    unsigned long ulLength = 0;
    unsigned long ulEnd;
    unsigned long ulStart;

    ulEnd = ulUARTBufferEnd;
    ulStart = ulUARTBufferStart;
    if (ulEnd == ulStart) return (0);
    if (ulEnd > ulStart) {
        memcpy((void *)chrUARTBufferOutput, (void *)(chrUARTBuffers + ulStart), ulEnd - ulStart);
        ulLength = ulEnd - ulStart;
    } else {
        memcpy((void *)chrUARTBufferOutput, (void *)(chrUARTBuffers + ulStart), UARTBufferLength - ulStart);
        if (ulEnd != 0) {
            memcpy((void *)(chrUARTBufferOutput + (UARTBufferLength - ulStart)), (void *)chrUARTBuffers, ulEnd);
        }
        ulLength = UARTBufferLength + ulEnd - ulStart;
    }
    ulUARTBufferStart = ulEnd;
    return (unsigned short)ulLength;
}

unsigned char ATR_Linux_OpenCom(char *Port, const int iBaudrate) {
    unsigned char r;
    signed char ret = OpenCOMDevice(Port, iBaudrate);
    if (ret >= 0) {
        r = 0x00;
    } else
        r = 0x1C;
    return r;
}

void CloseCOMDevice(void) { close(nFd); }

void ATR_Linux_CloseCom() { CloseCOMDevice(); }

signed char OpenCOMDevice(const char *Port, const int iBaudrate) {
    printf("Connecting %s\n", Port);
    nFd = open(Port, O_RDWR | O_NOCTTY | O_NDELAY);
    printf("nFd %d\n", nFd);
    if (nFd < 0) {
        printf("Connect %s failed!\n", Port);
        return -1;
    }

    struct termios options = {0};  //����һ���O�� comport ����Ҫ�ĽY���w �K����ՃȲ�

    /* c_cflag ����ģʽ��
     * CLOCAL:�����κ�modem status lines
     * CREAD:������Ԫ����
     * CS8:���ͻ���Օr��ʹ�ð˂�λԪ
     */
    int setSpeed;

    switch (iBaudrate) {
        case 9600:
            setSpeed = B9600;
            break;
        case 38400:
            setSpeed = B38400;
            break;
        case 57600:
            setSpeed = B57600;
            break;
        case 115200:
            setSpeed = B115200;
            break;
        case 230400:
            setSpeed = B230400;
            break;
        case 460800:
            setSpeed = B460800;
            break;
        case 921600:
            setSpeed = B921600;
            break;
        case 1500000:
            setSpeed = B1500000;
            break;
        default:
            setSpeed = B115200;
            break;
    }

    options.c_cflag =
        (setSpeed | CLOCAL | CREAD | CS8);  //����,�O�� baud rate,����׃ comport ������, ��������, 8 data bits

    cfsetispeed(&options, setSpeed);
    cfsetospeed(&options, setSpeed);

    options.c_cc[VTIME] = 1;  // 10 = 1��,���x�ȴ��ĕr�g����λ�ǰٺ���
    options.c_cc[VMIN] = 0;   //���x��Ҫ��ȴ�����С�ֹ���,�@�������Ͻo 0
    tcflush(nFd, TCIOFLUSH);  // ˢ�º����̌��Mȥfd

    fcntl(nFd, F_SETFL, 0);  // zuse

    if ((tcsetattr(nFd, TCSAFLUSH, &options)) == -1) {  //���ر��C�O��,TCSANOW >> ���̸�׃��ֵ
        nFd = -1;
    }

    // printf("%s connected!\n",pDev[PortNo]);
    printf("%s connected!\n", Port);

    if (pthread_create(&receiveDataThread, NULL, &ReceiveCOMData, NULL) != 0)
        printf("Create receive com data thread failed!\n");
    else
        printf("Create receive com data thread successfully!\n");
    // ReceiveCOMData();
    // while(1)
    //	{
    //				printf("thread 0 = chrBuffer ");
    //				for (int i = 0; i < UARTBufferLength; i++)
    //				{
    //					printf("%02x ", (unsigned char)chrUARTBuffers[i]);
    //				}
    //				printf("\n");
    //				printf("Recv: uLen = %d\n", UARTBufferLength);

    //	}

    return nFd;
}

void *ReceiveCOMData(void *arg)  // Thread run function
{
    unsigned char *pBufferStart, *pConsumer;
    unsigned char *pProducer, *pCurrent, *pBufferEnd;
    unsigned long ulUARTBufferEndTemp = ulUARTBufferEnd;

    while (1) {
        memset(chrBuffer, 0, iBufferSize);
        uLen = read(nFd, chrBuffer, iBufferSize - 1);
        // printf("uLen=%d\n",uLen);

        if (uLen <= 0) {
            // sleep (1);
            continue;
        }
        if (uLen > 0) {
            // printf("dataReady=1");
            dataReady = 1;
            /*if (!print_flag)
            {
                    printf("thread 1 = chrBuffer ");
                    for (int i = 0; i < uLen; i++)
                    {
                            printf(" %02x ", (unsigned char)chrBuffer[i]);
                    }
                    printf("\n");
                    printf("Recv: uLen = %d\n", uLen);
            }
*/
        }

        if ((ulUARTBufferEndTemp + uLen) > UARTBufferLength) {
            ulLen1 = UARTBufferLength - ulUARTBufferEndTemp;
            ulLen2 = uLen - ulLen1;
            // printf("ulLen1=%d ulLen2=%d\n",ulLen1,ulLen2);

            if (ulLen1 > 0) {
                memcpy((void *)&chrUARTBuffers[ulUARTBufferEnd], (void *)chrBuffer, ulLen1);
                pProducer += ulLen1;
            }

            if (ulLen2 > 0) {
                memcpy((void *)&chrUARTBuffers[0], (void *)(chrBuffer + ulLen1), ulLen2);
                // pProducer = ringBuf + ulLen2;
            }

            ulUARTBufferEndTemp = ulLen2;
            // printf("ulUARTBufferEndTemp=%d\n",ulUARTBufferEndTemp);
        } else {
            // printf("Copying memory.\n");

            memcpy((void *)&chrUARTBuffers[ulUARTBufferEnd], (void *)chrBuffer, uLen);
            ulUARTBufferEndTemp += uLen;
            // printf("ulUARTBufferEndTemp=%d\n",ulUARTBufferEndTemp);
        }

        if (ulUARTBufferEndTemp == ulUARTBufferStart) {
            printf("Error!");
        } else {
            ulUARTBufferEnd = ulUARTBufferEndTemp;
        }

        // Process();
    }
}

signed char SendUARTMessageLength(const char chrSendBuffer[], const unsigned short usLen) {
    write(nFd, chrSendBuffer, usLen);
    return 0;
}

signed char SetBaudrate(const int iBaudrate) {
    struct termios options = {0};  //����һ���O�� comport ����Ҫ�ĽY���w �K����ՃȲ�
    options.c_cflag =
        (iBaudrate | CLOCAL | CREAD | CS8);  //����,�O�� baud rate,����׃ comport ������, ��������, 8 data bits

    cfsetispeed(&options, iBaudrate);
    cfsetospeed(&options, iBaudrate);

    if ((tcsetattr(nFd, TCSANOW, &options)) == -1) return -1;

    return 0;
}
/*
int main()
{
        int i = 10000;
        unsigned char SendBuffer[] = { 0x41 , 0x78 , 0xFF , 0x01 ,0x02 , 0x00,0xc5, 0x6d };
        OpenCOMDevice("/dev/ttyUSB1",115200);
        while(i-- );
        SendUARTMessageLength((const char *)SendBuffer,8);
        while(1);
        return 0;
}
*/
