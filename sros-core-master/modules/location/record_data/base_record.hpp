//
// Created by lfc on 17-6-21.
//

#ifndef PROJECT_BASE_RECORD_HPP
#define PROJECT_BASE_RECORD_HPP

#include <fstream>
#include <Eigen/Dense>
#include <iostream>
#include <glog/logging.h>

namespace record {

class BaseRecord {
public:
    enum RecordType {
        LMK_POSE_RECORD = 1,
    };

    BaseRecord(RecordType type) : record_type(type) {

    }

    virtual ~BaseRecord() {
        is_open = false;
        record_stream.close();
    }

    bool openFile(const char *path) {
        record_stream.open(path, std::ios::out);
        if (record_stream.is_open()) {
            file_name = path;
            is_open = true;
            record_stream.width(20);
//            record_stream.precision(10);
            record_stream.setf(std::ios::fixed, std::ios::floatfield);
            record_stream.setf(record_stream.left);
            return true;
        }
        LOG(INFO) << "cannot open the file!";
        is_open = false;
        return false;
    }

    template<typename T_SAVE>
    void writeData(T_SAVE &data, std::string name) {
        if (is_open) {
            record_stream << name;
            record_stream << ",";
            record_stream << data;
            record_stream << ",";
        } else {
            LOG(INFO) << "the file is close!!!!!!!!! cannot write the data!";
        }
    }

    template<typename T_SAVE>
    void writeData(T_SAVE &data) {
        if (is_open) {
            record_stream.width(20);
            record_stream << data;
            record_stream << " ";
        } else {
            LOG(INFO) << "the file is close!!!!!!!!! cannot write the data!";
        }
    }

    void writePose(Eigen::Vector3f &pose) {
        float tmp_coord = pose.x();
        writeData(tmp_coord);
        tmp_coord = pose.y();
        writeData(tmp_coord);
        tmp_coord = pose.z();
        writeData(tmp_coord);
    }

    void writeName(std::string name) {
        writeData(name);
    }

    void writePoseName(std::string name) {
        std::string tmp_name = name + "_x";
        writeData(tmp_name);
        tmp_name = name + "_y";
        writeData(tmp_name);
        tmp_name = name + "_b";
        writeData(tmp_name);
    }

    void writeCovName(std::string name) {
        std::string tmp_name = name + "_00";
        writeData(tmp_name);
        tmp_name = name + "_01";
        writeData(tmp_name);
        tmp_name = name + "_02";
        writeData(tmp_name);
        tmp_name = name + "_10";
        writeData(tmp_name);
        tmp_name = name + "_11";
        writeData(tmp_name);
        tmp_name = name + "_12";
        writeData(tmp_name);
        tmp_name = name + "_20";
        writeData(tmp_name);
        tmp_name = name + "_21";
        writeData(tmp_name);
        tmp_name = name + "_22";
        writeData(tmp_name);
    }

    void writeCov(Eigen::Matrix3f &cov) {
        float tmp_cov = cov(0, 0);
        writeData(tmp_cov);
        tmp_cov = cov(0, 1);
        writeData(tmp_cov);
        tmp_cov = cov(0, 2);
        writeData(tmp_cov);
        tmp_cov = cov(1, 0);
        writeData(tmp_cov);
        tmp_cov = cov(1, 1);
        writeData(tmp_cov);
        tmp_cov = cov(1, 2);
        writeData(tmp_cov);
        tmp_cov = cov(2, 0);
        writeData(tmp_cov);
        tmp_cov = cov(2, 1);
        writeData(tmp_cov);
        tmp_cov = cov(2, 2);
        writeData(tmp_cov);
    }

    std::string selectStrByArrow(std::string input) {
        std::string output = input.substr(input.find("->", 0) + 2);
        return output;
    }

    std::string selectStrBydot(std::string input) {
        std::string output = input.substr(input.find(".", 0) + 1);
        return output;
    }

    void writeEndl() {
        if (is_open) {
            record_stream << std::endl;
        }
    }

    void close() {
        is_open = false;
        record_stream.close();
    }

    bool isOpen() {
        return is_open;
    }

    RecordType getType() {
        return record_type;
    }

private:
    BaseRecord() {

    }

    RecordType record_type;
    std::fstream record_stream;
    bool is_open;
    std::string file_name;

};

}


#endif //PROJECT_BASE_RECORD_HPP
