//
// Created by cwt on 21-11-12.
//

#ifndef  SRC_SPI_LOG_HPP
#define SRC_SPI_LOG_HPP

#include <core/src.h>
#include <core/state.h>
#include <glog/logging.h>
#include <boost/thread.hpp>
#include <stdint.h>
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <getopt.h>
#include <fcntl.h>
#include <time.h>
#include <sys/ioctl.h>
#include <linux/ioctl.h>
#include <sys/stat.h>
#include <linux/types.h>


namespace sdkSrtos {
class SrcSdkSpiLog  {
public:
	SrcSdkSpiLog() {
		open_log();
	}

	~SrcSdkSpiLog()  {
		close_log();
	}

	bool open_log() {
		fd_ = open("/sros/log/srtos_fifo", O_RDONLY);	//O_RDONLY
		if(fd_ < 0) {
			LOG(INFO) << "!!!!!SrcSdk: fifo open failed.";
			return false;
		}
		return true;
	}

	void close_log() {
		close(fd_);
	}

	void srtos_log_loop()
	{
		int ret = 0;
		if(fd_ <= 0) {
			ret = open_log();
			if(!ret) {
				return;
			}
			LOG(INFO) << "!!!!!SrcSdk: fifo open success.";
		}
		ret = read(fd_, tran_buf, sizeof(tran_buf));
		if(ret > 0) {
			show_log(tran_buf, ret);
		} else {
			usleep(100000); 
		}
	}

	bool  show_log(char* rx_data, int len) 
	{
		int ret = 0;
		int i = 0;
		char data[128] = {0};
		static int  offset = 0;
		static int  flag = 1;
		static char  buf[512] = {0};
		
		for(i = 0; i < len; i++) {
			data[i] = rx_data[i];
		}

		ret = mstrstr(data, "SRTOS", 0);
		if(ret >= 0) {
			flag = !flag;
		}
		if(offset + len > sizeof(buf)) {
			memset(buf, 0, sizeof(buf));
			offset = 0;
			flag = 1;
			return false;
		}

		for(i = 0; i < len; i++) {
			buf[offset + i] = data[i];
		}
		offset += len;

		if(flag == 1) {
			do {
				ret = mstrstr(buf, "SRTOS", 6);
				if(ret > 0) {
					char show[512] = {0};
					char tmp[512] = {0};

					for(i = 0; i < ret; i++) {
						show[i] = buf[i];
					}
					for(i = ret; i < offset; i++) {
						tmp[i-ret] = buf[i];
					}
					memset(buf, 0, sizeof(buf));
					for(i = 0; i < offset - ret; i++) {
						buf[i] = tmp[i];
					}
					offset = i;
					flag = 0;
					LOG(INFO) << show;
				}
			} while(ret > 0);
		}
		return true;
	}

	int  mstrstr(char* ptr,  const char* str,  int  offset)
	{
		int  i = offset;

		while(ptr[i] != '\0') {
			if(ptr[i] == str[0]) {
				if(ptr[i+1] == str[1]) {
					if(ptr[i+2] == str[2]) {
						if(ptr[i+3] == str[3]) {
							if(ptr[i+4] == str[4]) {
								return i;
							}
						}
					}
				}
			}
			i++;
		}
		return -1;
	}

private:	
	char tran_buf[128];
	int fd_;
};
}
#endif
