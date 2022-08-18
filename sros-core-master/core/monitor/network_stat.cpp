#include "network_stat.h"
#include <stdio.h>
#include <string.h>
#include <iostream>
#include <sstream>
#include <fstream>
#include <vector>
#include <chrono>

#include <linux/kernel.h>
#include <sys/sysinfo.h>

uint64_t newtork_statics(const std::string &net_card, int type) {
    //type = 9 packets; =10 bytes; =11 err 
    uint64_t ret = 0;
    char result[20];
    char command[1024];
    memset(command, '\0', 1024);
    snprintf(command, sizeof(command) - 1, "cat /proc/net/dev | grep %s |awk -F' ' '{print $%d}'", net_card.c_str(), type);
    
    FILE *fp = popen(command, "r");
    if (!fp) {
        char szerror[2048];
        memset(&(szerror[0]), 0, sizeof(szerror));
        if (errno != 0) {
            snprintf(szerror, sizeof(szerror) - 1, "popen failed with error %i '%s'", errno, strerror(errno));
        } else {
            snprintf(szerror, sizeof(szerror) - 1, "popen failed but errno was not set");
        }
        std::cout<<"Error: File "<<__FILE__<<"line "<<__LINE__<<" : "<<szerror<<std::endl;
        return 0;
    }

    size_t bytes = fread(result, 1, 20, fp);
    if (bytes > 0) {
        ret = atoll(result);
    } else {
        std::cout <<"read pipe error!"<<std::endl;
    }
    pclose(fp);
    return ret;
}
