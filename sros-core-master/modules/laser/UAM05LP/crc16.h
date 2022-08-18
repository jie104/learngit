#ifndef CRC16_H
#define CRC16_H

#include <stdio.h>

unsigned int char_to_int(const char *buf, int buf_len);
unsigned int int_to_char(char *destbuf, unsigned int input_num);

bool checkSum_crc16(const char* buf, int buf_len);

#endif