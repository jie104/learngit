//
// Created by lhx on 18-4-24.
//

#include "soc_id.h"

#include <sys/utsname.h>

#include <fstream>

#include "core/util/utils.h"

std::string get_soc_id() {
    std::string kernel_release;
    //kernel_release = execShell("uname -r");
    struct utsname buffer;
    if(uname(&buffer) != 0) {
        kernel_release = "";
    } else {
        kernel_release = buffer.release;
    }
    //目前只有4.14版本的soc_id存放在/proc/cpuinfo中
    if(kernel_release.compare(0, 4, "4.14") == 0) {
        std::string soc_id = execShell("cat /proc/cpuinfo | grep Serial | cut -d ':' -f2");
        soc_id.erase(0, soc_id.find_first_not_of(" "));
        soc_id.erase(soc_id.find_last_not_of(" ") + 1);
        return soc_id;

    } else {
        const char * path = "/proc/device-tree/serial-number";

        std::ifstream file_stat(path);
        std::string line;

        std::getline(file_stat, line);

        return line;
    }
}
