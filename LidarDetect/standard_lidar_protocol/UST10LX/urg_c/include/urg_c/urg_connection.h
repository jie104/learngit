#ifndef URG_CONNECTION_H
#define URG_CONNECTION_H

/*!
  \file
  \brief �ʐM�̏���

  \author Satofumi KAMIMURA

  $Id: urg_connection.h,v 1d233c7a2240 2011/02/19 03:08:45 Satofumi $
*/

#ifdef __cplusplus
extern "C" {
#endif

#include "urg_serial.h"
#include "urg_tcpclient.h"


/*!
  \brief �萔��`
*/
enum {
    URG_CONNECTION_TIMEOUT = -1, //!< �^�C���A�E�g�����������Ƃ��̖߂�l
};


/*!
  \brief �ʐM�^�C�v
*/
typedef enum {
    URG_SERIAL,                 //!< �V���A��, USB �ڑ�
    URG_ETHERNET,               //!< �C�[�T�[�l�b�g�ڑ�
} urg_connection_type_t;


/*!
  \brief �ʐM���\�[�X
*/
typedef struct
{
    urg_connection_type_t type; //!< �ڑ��^�C�v
    urg_serial_t serial;        //!< �V���A���ڑ�
    urg_tcpclient_t tcpclient;  //!< �C�[�T�[�l�b�g�ڑ�
} urg_connection_t;


/*!
  \brief �ڑ�

  �w�肳�ꂽ�f�o�C�X�ɐڑ�����B

  \param[in,out] connection �ʐM���\�[�X
  \param[in] connection_type �ڑ��^�C�v
  \param[in] device �ڑ���
  \param[in] baudrate_or_port �{�[���[�g / �|�[�g�ԍ�

  \retval 0 ����
  \retval <0 �G���[

  connection_type �ɂ�

  - URG_SERIAL ... �V���A���ʐM
  - URG_ETHERNET .. �C�[�T�[�l�b�g�ʐM

  ���w�肷��B

  device, baudrate_or_port �̎w��� connection_type �ɂ��w��ł���l���قȂ�B
  �Ⴆ�΁A�V���A���ʐM�̏ꍇ�͈ȉ��̂悤�ɂȂ�B

  Example
  \code
  connection_t connection;
  if (! connection_open(&connection, URG_SERIAL, "COM1", 115200)) {
      return 1;
  } \endcode

  �܂��A�C�[�T�[�l�b�g�ʐM�̏ꍇ�͈ȉ��̂悤�ɂȂ�B

  Example
  \code
  connection_t connection;
  if (! connection_open(&connection, URG_ETHERNET, "192.168.0.10", 10940)) {
      return 1;
  } \endcode

  \see connection_close()
*/
extern int connection_open(urg_connection_t *connection,
                           urg_connection_type_t connection_type,
                           const char *device, long baudrate_or_port);


/*!
  \brief �ؒf

  �f�o�C�X�Ƃ̐ڑ���ؒf����B

  \param[in,out] connection �ʐM���\�[�X

  \code
  connection_close(&connection); \endcode

  \see connection_open()
*/
extern void connection_close(urg_connection_t *connection);


/*! �{�[���[�g��ݒ肷�� */
extern int connection_set_baudrate(urg_connection_t *connection, long baudrate);


/*!
  \brief ���M

  �f�[�^�𑗐M����B

  \param[in,out] connection �ʐM���\�[�X
  \param[in] data ���M�f�[�^
  \param[in] size ���M�o�C�g��

  \retval >=0 ���M�f�[�^��
  \retval <0 �G���[

  Example
  \code
  n = connection_write(&connection, "QT\n", 3); \endcode

  \see connection_read(), connection_readline()
*/
extern int connection_write(urg_connection_t *connection,
                            const char *data, int size);


/*!
  \brief ��M

  �f�[�^����M����B

  \param[in,out] connection �ʐM���\�[�X
  \param[in] data ��M�f�[�^���i�[����o�b�t�@
  \param[in] max_size ��M�f�[�^���i�[�ł���o�C�g��
  \param[in] timeout �^�C���A�E�g���� [msec]

  \retval >=0 ��M�f�[�^��
  \retval <0 �G���[

  timeout �ɕ��̒l���w�肵���ꍇ�A�^�C���A�E�g�͔������Ȃ��B

  1 ��������M���Ȃ������Ƃ��� #URG_CONNECTION_TIMEOUT ��Ԃ��B

  Example
  \code
enum {
    BUFFER_SIZE = 256,
    TIMEOUT_MSEC = 1000,
};
char buffer[BUFFER_SIZE];
n = connection_read(&connection, buffer, BUFFER_SIZE, TIMEOUT_MSEC); \endcode

  \see connection_write(), connection_readline()
*/
extern int connection_read(urg_connection_t *connection,
                           char *data, int max_size, int timeout);


/*!
  \brief ���s�����܂ł̎�M

  ���s�����܂ł̃f�[�^����M����B

  \param[in,out] connection �ʐM���\�[�X
  \param[in] data ��M�f�[�^���i�[����o�b�t�@
  \param[in] max_size ��M�f�[�^���i�[�ł���o�C�g��
  \param[in] timeout �^�C���A�E�g���� [msec]

  \retval >=0 ��M�f�[�^��
  \retval <0 �G���[

  data �ɂ́A'\\0' �I�[���ꂽ������ max_size ���z���Ȃ��o�C�g�������i�[�����B �܂�A��M�ł��镶���̃o�C�g���́A�ő�� max_size - 1 �ƂȂ�B

  ���s������ '\\r' �܂��� '\\n' �Ƃ���B

  ��M�����ŏ��̕��������s�̏ꍇ�́A0 ��Ԃ��A1 ��������M���Ȃ������Ƃ��� #URG_CONNECTION_TIMEOUT ��Ԃ��B

  \see connection_write(), connection_read()
*/
extern int connection_readline(urg_connection_t *connection,
                               char *data, int max_size, int timeout);

#ifdef __cplusplus
}
#endif

#endif /* !URG_CONNECTION_H */
