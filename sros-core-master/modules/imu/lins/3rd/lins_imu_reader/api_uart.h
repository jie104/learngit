#ifndef __API_UART_H__
#define __API_UART_H__

#include <stdint.h>
#include <sys/time.h>
#include <termios.h>

#ifdef __cplusplus
extern "C" {
#endif

#define __DEBUG_LOG

enum uart_parity {
    UART_PARITY_NONE = 0,
    UART_PARITY_ODD,
    UART_PARITY_EVEN,
    UART_PARITY_MARK,
    UART_PARITY_SPACE,
};

struct uart_para {
    speed_t speed;
    uint32_t baudrate;
    uint8_t data_bits;
    uint8_t stop_bits;
    uint8_t parity_bits;
};

int32_t uart_open(char *devname, struct uart_para *up);
int32_t uart_send(int32_t fd, uint8_t *send, int32_t length);
int32_t uart_recv(int32_t fd, uint8_t *recv, int32_t length, int32_t timeout);
int32_t uart_close(int32_t fd);
/*
 * queue_selector:
   TCIFLUSH: flushes data received but not read.
   TCOFLUSH: flushes data written but not transmitted.
   TCIOFLUSH: flushes both data received but not read, and data written but not transmitted.
 */
void uart_flush(int32_t fd, int32_t queue_selector);
int32_t debug_log(const char *name, uint8_t *data, int32_t len);

#ifdef __cplusplus
}
#endif
#endif
