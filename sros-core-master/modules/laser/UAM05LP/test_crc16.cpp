#include <stdio.h>
#include <string.h>
#include <iostream>
#include <string>

#include "crc16.h"

#define  LOG(ERROR) std::cout
#define  LOG(INFO) std::cout



int main(void)
{
    bool result = false;
    char buf[] = "0000EVR0034920";
    result = checkSum_crc16(buf, strlen(buf) );
    std::cout << std::endl;
    std::cout << "main : buf_len=" << strlen(buf) << " result = " << result << std::endl;
    char destBuf[] = "3492";
    unsigned int num = 0x0F4F;
    int_to_char(destBuf, num);
    std::cout << "main : int_to_char=" << destBuf[0] << destBuf[1] << destBuf[2] << destBuf[3] << std::endl;
    return 0;
}