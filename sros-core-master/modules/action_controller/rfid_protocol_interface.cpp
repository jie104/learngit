/**
 * describe: 
 * Created by caoyan on 2021-3-4.
**/

#include "rfid_protocol_interface.h"
#include <sstream>
#include <iomanip>

namespace ac {
    
std::string RfidProtInterface::vecToString(std::vector<uint8_t>& raw_data) {
    std::string data;
    if(raw_data.empty()) {
        return data;
    }

    std::stringstream stream;
    for(int i = 0; i < raw_data.size(); ++i) {
        stream << std::setfill ('0') << std::setw(2) << std::hex << (int)raw_data[i];
    }

    data = stream.str();

    return data;

}

}