#include <stdio.h>
#include <string.h>
#include <iostream>
#include <string>

/****************************Info********************************************** 
 * Name:    CRC-16/CCITT        x16+x12+x5+1 
 * CRC Standard: Kermit
 * Polynomial: 0x1021
 * Shift Direction: Right
 * Initial Value: 0x0000
 * Byte Swap: Yes
 * Reverse CRC Result: Yes
 * Alias:   CRC-CCITT,CRC-16/CCITT-TRUE,CRC-16/KERMIT 
 *****************************************************************************/
static void InvertUint16(unsigned short *dBuf,unsigned short *srcBuf)
{
	int i;
	unsigned short tmp[4]={0};
 
	for(i=0;i< 16;i++)
	{
		if(srcBuf[0]& (1 << i))
		tmp[0]|=1<<(15 - i);
	}
	dBuf[0] = tmp[0];
}
static unsigned short CRC16_CCITT(const char *data, unsigned int datalen)
{
	unsigned short wCRCin = 0x0000;
	unsigned short wCPoly = 0x1021;
	unsigned char wChar = 0;
	
	InvertUint16(&wCPoly,&wCPoly);
	while (datalen--) 	
	{
		wCRCin ^= *(data++);
		for(int i = 0;i < 8;i++)
		{
			if(wCRCin & 0x01)
				wCRCin = (wCRCin >> 1) ^ wCPoly;
			else
				wCRCin = wCRCin >> 1;
		}
	}
	return (wCRCin);
}

// ASCII to int number  MSB first  buf[0] ~ highest 4bits
unsigned int char_to_int(const char *buf, int buf_len)
{
    char ch = '0';
    unsigned int num = 0;
    if( (buf_len < 1) || (nullptr == buf)) {
        std::cout << "input error len = " << buf_len << " buf:" << buf << std::endl;
        return 0;
    }

    for (int i = 0; i < 4; i ++)
    {
        ch = buf[i];
        if((ch >= '0')  && (ch <= '9'))
        {
            ch = ch - '0';
        }
        else if((ch >= 'A') && (ch <= 'F'))
        {
            ch = ch - 'A' + 10;
        }
        else
        {
            std::cout << "invalid ch:" << ch << std::endl;
        }
        num |= (unsigned int)ch << ((3 - i) * 4);
    }

    return num;
}

// ASCII to int number  MSB first  buf[0] ~ highest 4bits of num
unsigned int int_to_char(char *destbuf, unsigned int input_num)
{
    char ch = '0';
    if((nullptr == destbuf) || (strlen(destbuf) < sizeof(int))) {
        std::cout << "input error " << " buf:" << destbuf << std::endl;
        return 0;
    }

    for (int i = 0; i < 4; i ++)
    {
        ch = (input_num >> (4*(3 - i))) & 0x000F;
        if((ch >= 0)  && (ch <= 9))
        {
            ch += '0';
        }
        else if((ch >= 10) && (ch <= 16))
        {
            ch = ch - 10 + 'A';
        }
        else
        {
            std::cout << "invalid ch:" << ch << std::endl;
        }
        destbuf[i] = ch;
    }

    return 1;
}

/** UAM-05LP
 * 
buf[0] = STX

buf[buf_len - 5] = CRC  lowest 4bits
buf[buf_len - 2] = CRC  highest 4bits

buf[buf_len - 1] = ETX

**/
bool checkSum_crc16(const char* buf, int buf_len)
{
    #define CRC16_POLYNOMIAL 0x1021  //CRC16 polynomial: X16+X12+X5+1
    unsigned short result = 0;
    unsigned int buf_crc = 0;
    if( buf_len < 6 ) {
        std::cout << "input error len = " << buf_len << " buf:" << buf << std::endl;
        return false;
    }
    result = CRC16_CCITT(&buf[1], buf_len-2-4);
    buf_crc = char_to_int(&buf[buf_len - 5], 4);

    std::cout << std::hex << "result=" << result << " buf_crc=" << buf_crc << std::endl;

  if (result == buf_crc)
    return true;
  else
    return false;
}
