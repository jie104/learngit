#ifndef URG_UTILS_H
#define URG_UTILS_H

/*!
  \file
  \~japanese
  \brief URG �Z���T�p�̕⏕�֐�

  \~english
  \brief URG sensor utility

  \~
  \author Satofumi KAMIMURA

  $Id: urg_utils.h,v 630ee326c5ce 2011/02/19 08:06:25 Satofumi $
*/

#ifdef __cplusplus
extern "C" {
#endif

#include "urg_sensor.h"


    /*!
      \~japanese
      \brief URG �̃G���[�������������Ԃ�

      \param[in] urg URG �Z���T�Ǘ�

      \retval URG �̃G���[������������

      \~
      Example
      \code
      if (!urg_open(&urg, "/dev/ttyACM0", 115200, URG_SERIAL)) {
      printf("urg_open: %s\n", urg_error(&urg));
      return -1;
      } \endcode
    */
    extern const char *urg_error(const urg_t *urg);


    /*!
      \~japanese
      \brief �Z���T���Ԃ������̍ő�l�A�ŏ��l��Ԃ�

      �Z���T���Ԃ������� [�ŏ��l, �ő�l] �ŕԂ��܂��B

      \param[in] urg URG �Z���T�Ǘ�
      \param[out] min_distance �ŏ��l [mm]
      \param[out] max_distance �ő�l [mm]

      \~
      Example
      \code
      long min_distance, max_distance;
      urg_distance_min_max(&urg, &min_distance, &max_distance);

      for (int i = 0; i < n; ++i) {
      long distance = data[i];
      if ((distance < min_distance) || (distance > max_distance)) {
      continue;
      }
      ...
      } \endcode
    */
    extern void urg_distance_min_max(const urg_t *urg,
                                     long *min_distance, long *max_distance);


    /*!
      \~japanese
      \brief �v�� step �̍ő�l�A�ŏ��l��Ԃ�

      urg_set_scanning_parameter() �Ŏw��ł���͈͂� [�ŏ��l, �ő�l] �ŕԂ��B

      \param[in] urg URG �Z���T�Ǘ�
      \param[out] min_step �ŏ��l
      \param[out] max_step �ő�l

      step �̓Z���T���ʂ� 0 �ł���A�Z���T�㕔���猩���ꍇ�̔����v�܂��̕��������A���v�܂��̕��������� step �l�ƂȂ�B

      \image html sensor_step_image.png �Z���T�� step �̊֌W

      min_step, max_step �̒l�̓Z���T�ɂ���ĈقȂ�B

      \~
      Example
      \code
      urg_step_min_max(&urg, &min_step, &max_step);

      printf("range first: %d [deg]\n", urg_step2deg(&urg, min_step));
      printf("range last : %d [deg]\n", urg_step2deg(&urg, max_step)); \endcode

      \see urg_set_scanning_parameter(), urg_step2rad(), urg_step2deg()
    */
    extern void urg_step_min_max(const urg_t *urg, int *min_step, int *max_step);


    /*! \~japanese �P�X�L�����ɂ����鎞�� [usec] ��Ԃ� */
    extern long urg_scan_usec(const urg_t *urg);


    /*! \~japanese �擾�f�[�^���̍ő�l��Ԃ� */
    extern int urg_max_data_size(const urg_t *urg);


    /*!
      \~japanese
      \brief �C���f�b�N�X�Ɗp�x(radian)�̕ϊ����s��

      �C���f�b�N�Ƃ� urg_get_distance() �Ȃǂ̋����f�[�^�擾�֐����Ԃ����f�[�^�z��ɂ��Ă̒l�ł���B���̊֐��́A�Ō�ɍs���������f�[�^�擾�֐��̃f�[�^�z��ɂ��ėL���ƂȂ�B

      \param[in] urg URG �Z���T�Ǘ�
      \param[in] index �C���f�b�N�X

      \return �p�x [radian]

      index �́A�擾�����v���f�[�^�ɂ��Ă̒l�ł��� step ��p�x�Ƃ̊֌W�͎擾�ݒ�ɂ��قȂ�B

      \image html sensor_index_image.png �Z���T�̌v���͈͂ƃC���f�b�N�X�̊֌W

      \~
      Example
      \code
      int n = urg_get_distance(&urg, data, NULL);
      for (int i = 0; i < n; ++i) {
      long distance = data[i];
      double radian = urg_index2rad(i);
      double x = distance * cos(radian);
      double y = distance * sin(radian);
      printf("%.1f, %.1f\n", x, y);
      } \endcode

      \see urg_index2deg(), urg_rad2index(), urg_deg2index()
    */
    extern double urg_index2rad(const urg_t *urg, int index);


    /*! \~japanese �C���f�b�N�X�Ɗp�x(degree)�̕ϊ����s�� */
    extern double urg_index2deg(const urg_t *urg, int index);


    /*! \~japanese �p�x(radian)�ƃC���f�b�N�X�̕ϊ����s�� */
    extern int urg_rad2index(const urg_t *urg, double radian);


    /*! \~japanese �p�x(degree)�ƃC���f�b�N�X�̕ϊ����s�� */
    extern int urg_deg2index(const urg_t *urg, double degree);


    /*!
      \~japanese
      \brief �p�x(radian)�� step �̕ϊ����s��

      urg_step_min_max() �Œ�`����Ă��� step �ɂ��āA�p�x(radian)�� step �̕ϊ����s���B

      \param[in] urg URG �Z���T�Ǘ�
      \param[in] radian �p�x [radian]

      \return step

      \image html sensor_angle_image.png �Z���T�� step �Ɗp�x�Ƃ̊֌W

      �p�x���� step �֕ϊ��������ʂ������łȂ��ꍇ�A���ʂ� 0 �̕����ɐ؂�̂Ă�ꂽ�l�ƂȂ�B

      \~
      \see urg_step_min_max(), urg_deg2step(), urg_step2rad(), urg_step2deg()
    */
    extern int urg_rad2step(const urg_t *urg, double radian);


    /*! \~japanese �p�x(degree)�� step �̕ϊ����s�� */
    extern int urg_deg2step(const urg_t *urg, double degree);


    /*! \~japanese step �� �p�x(radian)�̕ϊ����s�� */
    extern double urg_step2rad(const urg_t *urg, int step);


    /*! \~japanese step �� �p�x(degree)�̕ϊ����s�� */
    extern double urg_step2deg(const urg_t *urg, int step);

    /*! \~japanese step �ƃC���f�b�N�X�̕ϊ����s�� */
    extern int urg_step2index(const urg_t *urg, int step);

#ifdef __cplusplus
}
#endif

#endif /* !URG_UTILS_H */
