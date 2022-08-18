//
// Created by lfc on 19-10-26.
//

#ifndef SROS_BASE_CAMERA_DEVICE_HPP
#define SROS_BASE_CAMERA_DEVICE_HPP

#include <boost/filesystem/operations.hpp>
#include "core/device/device.h"
#include "depth_image_backup.hpp"
#include "modules/camera/img_with_stamp_info.hpp"

namespace camera {
typedef std::function<void(ImgWithStampInfoPtr)> ImgMsgCallback;
class BaseCameraDevice : public sros::device::Device {
 public:
    BaseCameraDevice(ImgMsgCallback callback, const std::string &name, sros::device::DeviceID device_id,
                     sros::device::DeviceCommInterfaceType device_comm_interface_type,
                     sros::device::DeviceMountType device_type)
        : sendMsg(callback), Device(name, device_id, device_comm_interface_type, device_type) {
        manageRecordFile();
    }

    static void manageRecordFile(){
        static bool have_managed = false;
        if (!have_managed) {//确保只执行一次
            have_managed = true;
            if(!boost::filesystem::exists("/sros/record/")){
                boost::filesystem::create_directory("/sros/record");
                return;
            }

            boost::filesystem::directory_iterator begin("/sros/record/");
            boost::filesystem::directory_iterator end;
            std::vector<std::string> remove_files;
            for (;begin != end; begin++) {
                if (!boost::filesystem::is_directory(*begin)) {
                    std::string file_name = begin->path().filename().string();
                    auto curr_position = file_name.find(".bag");//确定是bag
                    if (curr_position >= 0 && curr_position < file_name.size()) {
                        std::stringstream file_stream;
                        file_stream << file_name;
                        std::string curr_time,curr_date;
                        std::getline(file_stream, curr_time, '.');
                        std::getline(file_stream, curr_date, '.');
                        auto curr_real_time = getCurrTime();
                        auto delta_date = deltaDate(curr_real_time->tm_yday, curr_date);
                        if (delta_date > 2) {//如果大于2天，数据自动清除
                            remove_files.push_back(begin->path().string());
                        }
                        LOG(INFO) << "file name:" << file_name << "," << delta_date;
                    }
                }
            }
            for (auto &file : remove_files) {
                boost::filesystem::remove(file);
            }
        }
    }

    static std::string creatFileName(std::string device_name){
        auto curr_time = getCurrTime();
        std::stringstream file_name;
        file_name << "/sros/record/";
        file_name << (curr_time->tm_year + 1900) << "-" << (curr_time->tm_mon + 1) << "-" << curr_time->tm_mday << "-"
                  << curr_time->tm_hour << "-" << curr_time->tm_min << "-" << curr_time->tm_sec<<".";
        file_name << curr_time->tm_yday << "." << device_name << ".bag";
        LOG(INFO) << "file name is:" << file_name.str();
        return file_name.str();
    }

    static struct tm* getCurrTime(){
        auto curr_time_stamp = std::time(nullptr);
        auto curr_time = std::localtime(&curr_time_stamp);
        return curr_time;
    }

    static int deltaDate(int &curr_date,std::string &last_date){
        try {
            int last_date_int = std::stoi(last_date);
            auto delta_time = curr_date - last_date_int;
            return abs(delta_time);
        } catch (...) {
            LOG(INFO) << "cannot convert date:" << last_date;
            return 100;
        }
    }


    void setEachRecordSize(int record_max_size){ record_max_size_ = record_max_size; }//设置每次记录的最大帧数，当记录数据达到最大时，则停止保存。

    void startRecord(){
        record_incre_ = 0;
        recorded_state_ = true;
    }

    bool isRecording(){ return recorded_state_; }

    template <class DataType>
    void recordData(DataType& data){
        if (recorded_state_) {
            if (!file_op_.isOpen()) {
                auto curr_file_name = creatFileName(getName());
                file_op_.openToWrite(curr_file_name);
            }
            data.computeForBackup();
            file_op_.backupData(data);
            if (record_incre_++ > record_max_size_) {
                record_incre_ = 0;
                recorded_state_ = false;
                file_op_.close();
            }
        }
    }

    virtual bool open() { return false; }

    virtual bool isOpened() { return false; }

    virtual bool isEnabled() { return false; }

    virtual void close() {}

    virtual bool enableCamera() { return false; }

    virtual bool disableCamera() { return false; }

    virtual bool reset() { return false;}

    virtual void setFPS(double fps) { fps_ = fps; }

    virtual void setOBSZISE(int obszie) { obszie_ = obszie; }

    virtual void setParameter(const std::string &name, const double value) {}

    virtual void getParameter(const std::string &name, double &value) {}

    virtual void setParameter(const std::string &name, const int value) {}

    virtual void getParameter(const std::string &name, int &value) {}

    virtual void setParameter(const std::string &name, const std::string value) {}

    virtual void getParameter(const std::string &name, std::string &value) {}

    void sleepFor10ms() { usleep(1e4); }

 protected:
    ImgMsgCallback sendMsg;
    double fps_ = 40.0;
    int obszie_ = 3;

 private:
    bool recorded_state_ = false;
    int record_incre_ = 0;
    int record_max_size_ = 100;//单次记录，只记录100张数据
    serialization::SerializeFileOperator file_op_;
};
typedef std::shared_ptr<BaseCameraDevice> BaseCameraDevicePtr;

}  // namespace camera

#endif  // SROS_BASE_CAMERA_DEVICE_HPP
