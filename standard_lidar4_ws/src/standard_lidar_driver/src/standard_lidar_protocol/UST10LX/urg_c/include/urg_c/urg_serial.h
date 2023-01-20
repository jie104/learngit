#ifndef URG_SERIAL_H
#define URG_SERIAL_H

/*!
  \file
  \brief �V���A���ʐM

  \author Satofumi KAMIMURA

  $Id: urg_serial.h,v 1d233c7a2240 2011/02/19 03:08:45 Satofumi $
*/

#ifdef __cplusplus
extern "C" {
#endif

#include "urg_detect_os.h"

#if defined(URG_WINDOWS_OS)
#include <windows.h>
#elif defined(ANDROID)
#include <termios.h>
#define tcdrain(fd) ioctl(fd, TCSBRK, 1) 
#else
#include <termios.h>
#include <sys/select.h>
#endif
#include "urg_ring_buffer.h"


enum {
    RING_BUFFER_SIZE_SHIFT = 7,
    RING_BUFFER_SIZE = 1 << RING_BUFFER_SIZE_SHIFT,

    ERROR_MESSAGE_SIZE = 256,
};
enum {
    False = 0,
    True,
};

//! �V���A���ʐM�p
typedef struct
{
#if defined(URG_WINDOWS_OS)
    HANDLE hCom;                /*!< �ڑ����\�[�X */
    int current_timeout;        /*!< �^�C���A�E�g�̐ݒ莞�� [msec] */
#else
    int fd;                     /*!< �t�@�C���f�B�X�N���v�^*/
    struct termios sio;         /*!< �ʐM�ݒ� */
#endif

    ring_buffer_t ring;         /*!< �����O�o�b�t�@ */
    char buffer[RING_BUFFER_SIZE]; /*!< �o�b�t�@�̈� */
    char has_last_ch;          /*!< �����߂������������邩�̃t���O */
    char last_ch;              /*!< �����߂����P���� */
} urg_serial_t;


//! �ڑ����J��
extern int serial_open(urg_serial_t *serial, const char *device, long baudrate);


//! �ڑ������
extern void serial_close(urg_serial_t *serial);


//! �{�[���[�g��ݒ肷��
extern int serial_set_baudrate(urg_serial_t *serial, long baudrate);


//! �f�[�^�𑗐M����
extern int serial_write(urg_serial_t *serial, const char *data, int size);


//! �f�[�^����M����
extern int serial_read(urg_serial_t *serial,
                       char *data, int max_size, int timeout);


//! ���s�܂ł̃f�[�^����M����
extern int serial_readline(urg_serial_t *serial,
                           char *data, int max_size, int timeout);


//! �G���[��������i�[���ĕԂ�
extern int serial_error(urg_serial_t *serial,
                        char *error_message, int max_size);

#ifdef __cplusplus
}
#endif

#endif /* !URG_SERIAL_H */
