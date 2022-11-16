#ifndef URG_SENSOR_H
#define URG_SENSOR_H

/*!
  \file
  \~japanese
  \brief URG �Z���T����

  URG �p�̊�{�I�Ȋ֐���񋟂��܂��B


  \~english
  \brief URG sensor

  URG �p�̊�{�I�Ȋ֐���񋟂��܂��B

  \~
  \author Satofumi KAMIMURA

  $Id: urg_sensor.h,v 540bc11f70c8 2011/05/08 23:04:49 satofumi $
*/

#ifdef __cplusplus
extern "C" {
#endif

#include "urg_connection.h"
#include "urg_time.h"


    /*!
      \~japanese
      \brief �v���^�C�v
    */
    typedef enum {
        URG_DISTANCE,           /*!< \~japanese ���� */
        URG_DISTANCE_INTENSITY, /*!< \~japanese ���� + ���x */
        URG_MULTIECHO,          /*!< \~japanese �}���`�G�R�[�̋��� */
        URG_MULTIECHO_INTENSITY, /*!< \~japanese �}���`�G�R�[��(���� + ���x) */
        URG_STOP,                /*!< \~japanese �v���̒�~ */
        URG_UNKNOWN,             /*!< \~japanese �s�� */
    } urg_measurement_type_t;

    /*!
      \~japanese
      \brief �������� byte �ŕ\�����邩�̎w��
    */
    typedef enum {
        URG_COMMUNICATION_3_BYTE, /*!< \~japanese ������ 3 byte �ŕ\������ */
        URG_COMMUNICATION_2_BYTE, /*!< \~japanese ������ 2 byte �ŕ\������ */
    } urg_range_data_byte_t;


    enum {
        URG_SCAN_INFINITY = 0,  /*!< \~japanese ������̃f�[�^�擾 */
        URG_MAX_ECHO = 3, /*!< \~japanese �}���`�G�R�[�̍ő�G�R�[�� */
    };


    /*! \~japanese �G���[�n���h�� \~english error handler */
    typedef urg_measurement_type_t
    (*urg_error_handler)(const char *status, void *urg);


    /*!
      \~japanese
      \brief URG �Z���T�Ǘ�

      \~english
      \brief URG sensor
    */
    typedef struct
    {
        int is_active;
        int last_errno;
        urg_connection_t connection;

        int first_data_index;
        int last_data_index;
        int front_data_index;
        int area_resolution;
        long scan_usec;
        int min_distance;
        int max_distance;
        int scanning_first_step;
        int scanning_last_step;
        int scanning_skip_step;
        int scanning_skip_scan;
        urg_range_data_byte_t range_data_byte;

        int timeout;
        int specified_scan_times;
        int scanning_remain_times;
        int is_laser_on;

        int received_first_index;
        int received_last_index;
        int received_skip_step;
        urg_range_data_byte_t received_range_data_byte;
        int is_sending;

        urg_error_handler error_handler;

        char return_buffer[80];
    } urg_t;


    /*!
      \~japanese
      \brief �ڑ�

      �w�肵���f�o�C�X�ɐڑ����A�������v���ł���悤�ɂ���B

      \param[in,out] urg URG �Z���T�Ǘ�
      \param[in] connection_type �ʐM�^�C�v
      \param[in] device_or_address �ڑ��f�o�C�X��
      \param[in] baudrate_or_port �ڑ��{�[���[�g [bps] / TCP/IP �|�[�g

      \retval 0 ����
      \retval <0 �G���[

      connection_type �ɂ́A�ȉ��̍��ڂ��w��ł��܂��B

      - #URG_SERIAL
      - �V���A���AUSB �ڑ�

      - #URG_ETHERNET
      - �C�[�T�[�l�b�g�ڑ�

      Example
      \code
      urg_t urg;

      if (urg_open(&urg, URG_SERIAL, "/dev/ttyACM0", 115200) < 0) {
      return 1;
      }

      ...

      urg_close(&urg); \endcode

      \attention URG C ���C�u�����̑��̊֐����Ăяo���O�ɁA���̊֐����Ăяo���K�v������܂��B

      \~
      \see urg_close()
    */
    extern int urg_open(urg_t *urg, urg_connection_type_t connection_type,
                        const char *device_or_address,
                        long baudrate_or_port);


    /*!
      \~japanese
      \brief �ؒf

      ���[�U���������AURG �Ƃ̐ڑ���ؒf���܂��B

      \param[in,out] urg URG �Z���T�Ǘ�

      \~
      \see urg_open()
    */
    extern void urg_close(urg_t *urg);


    /*!
      \brief �^�C���A�E�g���Ԃ̐ݒ�

      \param[in,out] urg URG �Z���T�Ǘ�
      \param[in] msec �^�C���A�E�g���鎞�� [msec]

      \attention urg_open() ���Ăяo���� timeout �̐ݒ�l�̓f�t�H���g�l�ɏ���������邽�߁A���̊֐��� urg_open() ��ɌĂяo�����ƁB
    */
    extern void urg_set_timeout_msec(urg_t *urg, int msec);


    /*! \~japanese �^�C���X�^���v���[�h�̊J�n */
    extern int urg_start_time_stamp_mode(urg_t *urg);


    /*!
      \~japanese
      \brief �^�C���X�^���v�̎擾

      \param[in,out] urg URG �Z���T�Ǘ�

      \retval >=0 �^�C���X�^���v [msec]
      \retval <0 �G���[

      Example
      \code
      urg_start_time_stamp_mode(&urg);

      before_ticks = get_pc_msec_function();
      time_stamp = urg_time_stamp(&urg);
      after_ticks = get_pc_msec_function();

      // �^�C���X�^���v�ɂ��Ă̌v�Z
      ...

      urg_stop_time_stamp_mode(&urg); \endcode

      �ڂ����� \ref sync_time_stamp.c ���Q�Ƃ��ĉ������B
    */
    extern long urg_time_stamp(urg_t *urg);


    /*! \~japanese �^�C���X�^���v���[�h�̏I�� */
    extern int urg_stop_time_stamp_mode(urg_t *urg);


    /*!
      \~japanese
      \brief �����f�[�^�̎擾���J�n

      �����f�[�^�̎擾���J�n���܂��B���ۂ̃f�[�^�� urg_get_distance(), urg_get_distance_intensity(), urg_get_multiecho(), urg_get_multiecho_intensity() �Ŏ擾�ł��܂��B

      \param[in,out] urg URG �Z���T�Ǘ�
      \param[in] type �f�[�^�E�^�C�v
      \param[in] scan_times �f�[�^�̎擾��
      \param[in] skip_scan �f�[�^�̎擾�Ԋu

      \retval 0 ����
      \retval <0 �G���[

      type �ɂ͎擾����f�[�^�̎�ނ��w�肵�܂��B

      - #URG_DISTANCE ... �����f�[�^
      - #URG_DISTANCE_INTENSITY ... �����f�[�^�Ƌ��x�f�[�^
      - #URG_MULTIECHO ... �}���`�G�R�[�ł̋����f�[�^
      - #URG_MULTIECHO_INTENSITY ... �}���`�G�R�[�ł�(�����f�[�^�Ƌ��x�f�[�^)

      scan_times �͉���̃f�[�^���擾���邩�� 0 �ȏ�̐��Ŏw�肵�܂��B�������A0 �܂��� #URG_SCAN_INFINITY ���w�肵���ꍇ�́A������̃f�[�^���擾���܂��B\n
      �J�n�����v���𒆒f����ɂ� urg_stop_measurement() ���g���܂��B

      skip_scan �̓~���[�̉�]���̂����A�P��̃X�L������ɉ���X�L�������Ȃ������w�肵�܂��Bskip_scan �Ɏw��ł���͈͂� [0, 9] �ł��B

      \image html skip_scan_image.png ����ɂP�񂾂��v�����邩

      ���Ƃ��΁A�~���[�̂P��]�� 100 [msec] �̃Z���T�� skip_scan �� 1 ���w�肵���ꍇ�A�f�[�^�̎擾�Ԋu�� 200 [msec] �ɂȂ�܂��B

      Example
      \code
      enum { CAPTURE_TIMES = 10 };
      urg_start_measurement(&urg, URG_DISTANCE, CAPTURE_TIMES, 0);

      for (i = 0; i < CAPTURE_TIMES; ++i) {
      int n = urg_get_distance(&urg, data, &time_stamp);

      // ��M�����f�[�^�̗��p
      ...
      } \endcode

      \~
      \see urg_get_distance(), urg_get_distance_intensity(), urg_get_multiecho(), urg_get_multiecho_intensity(), urg_stop_measurement()
    */
    extern int urg_start_measurement(urg_t *urg, urg_measurement_type_t type,
                                     int scan_times, int skip_scan);


    /*!
      \~japanese
      \brief �����f�[�^�̎擾

      �Z���T���狗���f�[�^���擾���܂��B���O�� urg_start_measurement() �� #URG_DISTANCE �w��ŌĂяo���Ă����K�v������܂��B

      \param[in,out] urg URG �Z���T�Ǘ�
      \param[out] data �����f�[�^ [mm]
      \param[out] time_stamp �^�C���X�^���v [msec]

      \retval >=0 ��M�����f�[�^��
      \retval <0 �G���[

      data �ɂ́A�Z���T����擾���������f�[�^���i�[����܂��Bdata �̓f�[�^���i�[����̃T�C�Y���m�ۂ��Ă����K�v������܂��Bdata �Ɋi�[�����f�[�^���� urg_max_data_size() �Ŏ擾�ł��܂��B

      time_stamp �ɂ́A�Z���T�����̃^�C���X�^���v���i�[����܂��Btime_stamp ���擾�������Ȃ��ꍇ NULL ���w�肵�ĉ������B

      Example
      \code
      long *data = (long*)malloc(urg_max_data_size(&urg) * sizeof(data[0]));

      ...

      // �f�[�^�̂ݎ擾����
      urg_start_measurement(&urg, URG_DISTANCE, 1, 0);
      int n = urg_get_distance(&urg, data, NULL);

      ...

      // �f�[�^�ƃ^�C���X�^���v���擾����
      long time_stamp;
      urg_start_measurement(&urg, URG_DISTANCE, 1, 0);
      n = urg_get_distance(&urg, data, &time_stamp); \endcode

      \~
      \see urg_start_measurement(), urg_max_data_size()
    */
    extern int urg_get_distance(urg_t *urg, long data[], long *time_stamp, unsigned long long *system_time_stamp);


    /*!
      \~japanese
      \brief �����Ƌ��x�f�[�^�̎擾

      urg_get_distance() �ɉ����A���x�f�[�^�̎擾���ł���֐��ł��B���O�� urg_start_measurement() �� #URG_DISTANCE_INTENSITY �w��ŌĂяo���Ă����K�v������܂��B

      \param[in,out] urg URG �Z���T�Ǘ�
      \param[out] data �����f�[�^ [mm]
      \param[out] intensity ���x�f�[�^
      \param[out] time_stamp �^�C���X�^���v [msec]

      \retval >=0 ��M�����f�[�^��
      \retval <0 �G���[

      ���x�f�[�^�Ƃ́A�����v�Z�Ɏg�����g�`�̔��ˋ��x�ł���A�Z���T�̃V���[�Y���ɓ������قȂ�܂��B ���x�f�[�^���g�����ƂŁA���̂̔��˗�����̑�܂��ȔZ�W�𐄑��ł��܂��B

      data, time_stamp �ɂ��Ă� urg_get_distance() �Ɠ����ł��B

      intensity �ɂ́A�Z���T����擾�������x�f�[�^���i�[����܂��Bintensity �̓f�[�^���i�[����̃T�C�Y���m�ۂ��Ă����K�v������܂��Bintensity �Ɋi�[�����f�[�^���� urg_max_data_size() �Ŏ擾�ł��܂��B

      \~
      Example
      \code
      int data_size = urg_max_data_size(&urg);
      long *data = malloc(data_size * sizeof(long));
      long *intensity = malloc(data_size * sizeof(unsigned short));

      ...

      urg_start_measurement(&urg, URG_DISTANCE_INTENSITY, 1, 0);
      int n = urg_get_distance_intensity(&urg, data, intesnity, NULLL); \endcode

      \~
      \see urg_start_measurement(), urg_max_data_size()
    */
    extern int urg_get_distance_intensity(urg_t *urg, long data[],
                                          unsigned short intensity[],
                                          long *time_stamp, unsigned long long *system_time_stamp);


    /*!
      \~japanese
      \brief �����f�[�^�̎擾 (�}���`�G�R�[��)

      �}���`�G�R�[�ł̋����f�[�^�擾�֐��ł��B���O�� urg_start_measurement() �� #URG_MULTIECHO �w��ŌĂяo���Ă����K�v������܂��B

      \param[in,out] urg URG �Z���T�Ǘ�
      \param[out] data_multi �����f�[�^ [mm]
      \param[out] time_stamp �^�C���X�^���v [msec]

      \retval >=0 ��M�����f�[�^��
      \retval <0 �G���[

      �}���`�G�R�[�Ƃ͕����̋����f�[�^�ł��B �}���`�G�R�[�́A�P�̃��[�U�����ɂ����ĕ����̋����f�[�^������ꂽ�Ƃ��ɓ����܂��B

      \image html multiecho_image.png �}���`�G�R�[�̃C���[�W�}

      time_stamp �ɂ��Ă� urg_get_distance() �Ɠ����ł��B

      data_multi �ɂ́A�Z���T����擾���������f�[�^���P�� step ������ő�� #URG_MAX_ECHO (3 ��)�i�[����܂��B�}���`�G�R�[�����݂��Ȃ����ڂ̃f�[�^�l�� -1 ���i�[����Ă��܂��B

      \verbatim
      data_multi[0] ... step n �̋����f�[�^ (1 ��)
      data_multi[1] ... step n �̋����f�[�^ (2 ��)
      data_multi[2] ... step n �̋����f�[�^ (3 ��)
      data_multi[3] ... step (n + 1) �� �����f�[�^ (1 ��)
      data_multi[4] ... step (n + 1) �� �����f�[�^ (2 ��)
      data_multi[5] ... step (n + 1) �� �����f�[�^ (3 ��)
      ... \endverbatim

      �i�[���́A�e step �ɂ����� urg_get_distance() �̂Ƃ��Ɠ��������̃f�[�^�� (3n + 0) �̈ʒu�Ɋi�[����A����ȊO�̃f�[�^�� (3n + 1), (3n + 2) �̈ʒu�ɍ~���Ɋi�[����܂��B\n
      �܂� data_multi[3n + 1] >= data_multi[3n + 2] �ɂȂ邱�Ƃ͕ۏ؂���܂��� data_multi[3n + 0] �� data_multi[3n + 1] �̊֌W�͖���`�ł��B(data_multi[3n + 1] == data_multi[3n + 2] �����藧�̂̓f�[�^�l�� -1 �̂Ƃ��B)

      \~
      Example
      \code
      long *data_multi = malloc(3 * urg_max_data_size(&urg) * sizeof(long));

      ...

      urg_start_measurement(&urg, URG_MULTIECHO, 1, 0);
      int n = urg_get_distance_intensity(&urg, data_multi, NULLL); \endcode

      \~
      \see urg_start_measurement(), urg_max_data_size()
    */
    extern int urg_get_multiecho(urg_t *urg, long data_multi[], long *time_stamp, unsigned long long *system_time_stamp);


    /*!
      \~japanese
      \brief �����Ƌ��x�f�[�^�̎擾 (�}���`�G�R�[��)

      urg_get_multiecho() �ɉ����A���x�f�[�^�̎擾�ł���֐��ł��B���O�� urg_start_measurement() �� #URG_MULTIECHO_INTENSITY �w��ŌĂяo���Ă����K�v������܂��B

      \param[in,out] urg URG �Z���T�Ǘ�
      \param[out] data_multi �����f�[�^ [mm]
      \param[out] intensity_multi ���x�f�[�^
      \param[out] time_stamp �^�C���X�^���v [msec]

      \retval >=0 ��M�����f�[�^��
      \retval <0 �G���[

      data_multi, time_stamp �ɂ��Ă� urg_get_multiecho() �Ɠ����ł��B

      intensity_multi �̃f�[�^�̕��т� data_multi �ƑΉ��������̂ɂȂ�܂��Bintensity_multi �Ɋi�[�����f�[�^���� urg_max_data_size() �Ŏ擾�ł��܂��B

      \~
      Example
      \code
      int data_size = urg_max_data_size(&urg);
      long *data_multi = malloc(3 * data_size * sizeof(long));
      long *intensity_multi = malloc(3 * data_size * sizeof(unsigned short));

      ...

      urg_start_measurement(&urg, URG_DISTANCE_INTENSITY, 1, 0);
      int n = urg_get_multiecho_intensity(&urg, data_multi,
      intesnity_multi, NULLL); \endcode

      \~
      \see urg_start_measurement(), urg_max_data_size()
    */
    extern int urg_get_multiecho_intensity(urg_t *urg, long data_multi[],
                                           unsigned short intensity_multi[],
                                           long *time_stamp, unsigned long long *system_time_stamp);


    /*!
      \~japanese
      \brief �v���𒆒f���A���[�U�����������܂�

      \ref urg_start_measurement() �̌v���𒆒f���܂��B

      \param[in,out] urg URG �Z���T�Ǘ�

      \retval 0 ����
      \retval <0 �G���[

      \~
      Example
      \code
      urg_start_measurement(&urg, URG_DISTANCE, URG_SCAN_INFINITY, 0);
      for (int i = 0; i < 10; ++i) {
      urg_get_distance(&urg, data, NULL);
      }
      urg_stop_measurement(&urg); \endcode

      \~
      \see urg_start_measurement()
    */
    extern int urg_stop_measurement(urg_t *urg);


    /*!
      \~japanese
      \brief �v���͈͂�ݒ肵�܂�

      �Z���T���v������͈͂� step �l�Ŏw�肵�܂��Burg_get_distance() �Ȃǂ̋����f�[�^�擾�̊֐��ŕԂ����f�[�^���́A�����Ŏw�肵���͈͂Ő�������܂��B

      \param[in,out] urg URG �Z���T�Ǘ�
      \param[in] first_step �v���̊J�n step
      \param[in] last_step �v���̏I�� step
      \param[in] skip_step �v���f�[�^���O���[�s���O�����

      \retval 0 ����
      \retval <0 �G���[

      �Z���T�� step �́A�Z���T���ʂ� 0 �Ƃ��A�Z���T�㕔���猩�Ĕ����v�܂��̌��������̒l�ƂȂ鏇�Ɋ���U���܂��B

      \image html sensor_angle_image.png �Z���T�� step �̊֌W

      step �̊Ԋu�ƁA�ő�l�A�ŏ��l�̓Z���T�ˑ��ł��Bstep �l�̍ő�l�A�ŏ��l�� urg_step_min_max() �Ŏ擾�ł��܂��B\n

      first_step, last_step �Ńf�[�^�̌v���͈͂��w�肵�܂��B�v���͈͂� [first_step, last_step] �ƂȂ�܂��B

      skip_step �́A�v���f�[�^���O���[�s���O��������w�肵�܂��B�w��ł���l�� [0, 99] �ł��B\n
      skip_step �́A�w�肳�ꂽ���̃f�[�^�� 1 �ɂ܂Ƃ߂邱�ƂŁA�Z���T�����M����f�[�^�ʂ����炵�A�����擾���s���֐��̉����������߂�Ƃ��Ɏg���܂��B�������A�f�[�^���܂Ƃ߂邽�߁A������f�[�^�̕���\�͌���܂��B

      �Ⴆ�Έȉ��̂悤�ȋ����f�[�^��������ꍇ��
      \verbatim
      100, 101, 102, 103, 104, 105, 106, 107, 108, 109
      \endverbatim

      skip_step �� 2 ���w�肷��ƁA������f�[�^��
      \verbatim
      \endverbatim

      �f�[�^�́A�܂Ƃ߂�f�[�^�̂����A��ԏ����Ȓl�̃f�[�^���p�����܂��B

      \~
      Example
      \code
      urg_set_scanning_parameter(&urg, urg_deg2step(&urg, -45),
      urg_deg2step(&urg, +45), 1);
      urg_start_measurement(&urg, URG_DISTANCE, 0);
      int n = urg_get_distance(&urg, data, NULL);
      for (int i = 0; i < n; ++i) {
      printf("%d [mm], %d [deg]\n", data[i], urg_index2deg(&urg, i));
      } \endcode

      \~
      \see urg_step_min_max(), urg_rad2step(), urg_deg2step()
    */
    extern int urg_set_scanning_parameter(urg_t *urg, int first_step,
                                          int last_step, int skip_step);


    /*!
      \~japanese
      \brief �ʐM�f�[�^�̃T�C�Y�ύX

      �����f�[�^���Z���T�����M�̍ۂ̃f�[�^�T�C�Y��ύX���܂��B

      \param[in,out] urg URG �Z���T�Ǘ�
      \param[in] data_byte �����l��\������f�[�^�̃o�C�g��

      \retval 0 ����
      \retval <0 �G���[

      data_byte �ɂ�

      - URG_COMMUNICATION_3_BYTE ... ������ 3 byte �ŕ\������
      - URG_COMMUNICATION_2_BYTE ... ������ 2 byte �ŕ\������

      ���w��ł��܂��B\n
      ������Ԃł͋����� 3 byte �ŕ\������悤�ɂȂ��Ă��܂��B���̐ݒ�� 2 byte �ɐݒ肷�邱�ƂŁA�Z���T�����M����f�[�^���� 2/3 �ɂȂ�܂��B�������A�擾�ł��鋗���̍ő�l�� 4095 �ɂȂ邽�߁A�ϑ��������Ώۂ� 4 [m] �ȓ��͈̔͂ɑ��݂���ꍇ�̂ݗ��p���ĉ������B
    */
    extern int urg_set_communication_data_size(urg_t *urg,
                                               urg_range_data_byte_t data_byte);


    /*! \~japanese ���[�U�𔭌������� */
    extern int urg_laser_on(urg_t *urg);


    /*! \~japanese ���[�U���������� */
    extern int urg_laser_off(urg_t *urg);


    /*! \~japanese �Z���T���ċN������ */
    extern int urg_reboot(urg_t *urg);


    /*!
      \~japanese
      \brief �Z���T������d�͂̏�ԂɑJ�ڂ�����

      �����d�͂̃��[�h�ł́A�X�L���i�̉�]����~���v�������f����܂��B

      - �����d�͂̃��[�h
        - ���[�U���������Čv�������f�����B
        - �X�L���i�̉�]����~����B

      �����d�͂̃��[�h���甲���邽�߂ɂ� \ref urg_wakeup() �֐����Ăяo���ĉ������B

      \see urg_wakeup()
    */
    extern void urg_sleep(urg_t *urg);


    /*!
      \~japanese
      \brief �Z���T������d�͂̃��[�h����ʏ�̏�ԂɑJ�ڂ�����

      \see urg_sleep()
    */
    extern void urg_wakeup(urg_t *urg);

    /*!
      \~japanese
      \brief �Z���T���v���ł����Ԃ���Ԃ�

      \retval 1 �Z���T���v���ł����Ԃɂ���
      \retval 0 �Z���T���v���ł����ԂɂȂ�

      �N������ŃX�L���i�̉�]�����肵�Ă��Ȃ��ꍇ��A���炩�̃G���[�Ōv���ł��Ȃ��ꍇ�A���̊֐��� 0 ��Ԃ��܂��B
    */
    extern int urg_is_stable(urg_t *urg);


    /*!
      \~japanese
      \brief �Z���T�^���𕶎���ŕԂ�

      �Z���T�̌^���𕶎���ŕԂ��B�Ԃ���镶����̓Z���T�ˑ��ƂȂ�B

      \param[in] urg URG �Z���T�Ǘ�

      \return �Z���T�^���̕�����
    */
    extern const char *urg_sensor_product_type(urg_t *urg);


    /*!
      \~japanese
      \brief �Z���T�̃V���A�� ID �������Ԃ�

      �Z���T�̃V���A�� ID �������Ԃ��B�Ԃ���镶����̓Z���T�ˑ��ƂȂ�B

      \param[in] urg URG �Z���T�Ǘ�

      \return �V���A�� ID ������
    */
    extern const char *urg_sensor_serial_id(urg_t *urg);

    /*!
      \brief returns the vendor name

      \param[in] URG

      \return The vendor name
    */
    extern const char *urg_sensor_vendor(urg_t *urg);


    /*!
      \~japanese
      \brief �Z���T�̃o�[�W�����������Ԃ�

      �Z���T�̃\�t�g�E�F�A�E�o�[�W�����������Ԃ��B�Ԃ���镶����̓Z���T�ˑ��ƂȂ�B

      \param[in] urg URG �Z���T�Ǘ�

      \return �o�[�W����������
    */
    extern const char *urg_sensor_firmware_version(urg_t *urg);

    extern const char *urg_sensor_firmware_date(urg_t *urg);

    /*!
      \brief returns the protocol version

      \param[in] URG

      \return The current protocol version
    */
    extern const char *urg_sensor_protocol_version(urg_t *urg);

    /*!
      \~japanese
      \brief �Z���T�̃X�e�[�^�X�������Ԃ�

      �Z���T�̃X�e�[�^�X�������Ԃ��B�Ԃ���镶����̓Z���T�ˑ��ƂȂ�B

      \param[in] urg URG �Z���T�Ǘ�
      \return �X�e�[�^�X������
    */
    extern const char *urg_sensor_status(urg_t *urg);


    /*!
      \~japanese
      \brief �Z���T�̏�Ԃ�Ԃ�

      �Z���T�̃X�e�[�^�X�������Ԃ��B�Ԃ���镶����̓Z���T�ˑ��ƂȂ�B

      \param[in] urg URG �Z���T�Ǘ�
      \return ��Ԃ�����������

      \attention ��Ԃɂ��Ă� SCIP �̒ʐM�d�l�����Q�Ƃ̂��ƁB
    */
    extern const char *urg_sensor_state(urg_t *urg);


    /*!
      \~japanese
      \brief �v���p�̃G���[�n���h����o�^����

      �G���[�n���h���� Gx, Mx �n�̃R�}���h�̉����� "00" �� "99" �ȊO�̂Ƃ��ɌĂяo�����B
    */
    extern void urg_set_error_handler(urg_t *urg, urg_error_handler handler);


    /*!
      \~japanese
      \brief SCIP ������̃f�R�[�h���s��

      \param[in] data SCIP ������
      \param[in] data �� byte �T�C�Y

      \retval �f�R�[�h��̐��l
    */
    extern long urg_scip_decode(const char data[], int size);


#ifdef __cplusplus
}
#endif

#endif /* !URG_SENSOR_H */
