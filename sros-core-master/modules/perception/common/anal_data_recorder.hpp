//
// Created by lfc on 18-9-4.
//

#ifndef SROS_ANAL_DATA_RECORDER_HPP
#define SROS_ANAL_DATA_RECORDER_HPP

#include <fstream>
#include <iomanip>
#include <iostream>
#include <memory>
#include <glog/logging.h>


class AnalDataRecorder {
public:
    AnalDataRecorder(std::string map_name,std::vector<std::string> &headers) {
        file_operator.open(map_name, std::ios::out);
        if (!file_operator.is_open()) {
            LOG(INFO) << "err to open the file:" << map_name;
        }
        file_operator.setf(file_operator.left);
        for (auto &header:headers) {
            write(file_operator,header);
        }
        file_operator << std::endl;
    }

    virtual ~AnalDataRecorder(){

    }

    template <typename Type>
    void writeData(const std::vector<Type> &values) {
        for (auto &value:values) {
            write(file_operator, value);
        }
        file_operator << std::endl;
    }
private:
    template <typename Type> void write(std::fstream &stream,Type &value) {
        stream << std::setw(11) << value << ",";
    }

    std::fstream file_operator;
};
typedef std::shared_ptr<AnalDataRecorder> AnalDataRecorder_Ptr;


#endif //SROS_ANAL_DATA_RECORDER_HPP
