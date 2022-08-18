//
// Created by lfc on 18-12-13.
//

#ifndef SROS_BASE_SENSOR_MANAGER_HPP
#define SROS_BASE_SENSOR_MANAGER_HPP

#include <boost/function.hpp>
#include <boost/filesystem.hpp>
#include <modules/camera/camera_device/depth_image_backup.hpp>
#include "../obstacle_module_para.hpp"
#include "core/msg/ObstacleMsg.hpp"
#include "core/tf/TFOperator.h"
#include "core/pose.h"

namespace sensor{
typedef boost::function<void(sros::core::base_msg_ptr)> ObaMsgCallback;
enum SensorType{
    TYPE_LASER_SENSOR = 1,
    TYPE_R2100_SENSOR = 2,
    TYPE_SH100_SENSOR = 3,
    TYPE_EU100_TIM320_SENSOR = 4,
    TYPE_STEREO_CAMERA = 5,
    TYPE_UST05LA_SENSOR = 6,
    TYPE_SH200_SENSOR = 7,
    TYPE_KELI_SENSOR = 8,
    TYPE_IFM_SENSOR = 9
};

class BaseSensorManager {
public:
    BaseSensorManager(oba::ObstacleModulePara_Ptr oba_para, ObaMsgCallback obaMsgCallback,SensorType sensor_type)
            : para(oba_para), sendObaMsg(obaMsgCallback),type(sensor_type),enable_publish_obstacle_(true){
       slam::tf::FrameToFrame base_to_world_frame;
       base_to_world_frame.parent_frame = "world";
       base_to_world_frame.child_frame = "base_link";
       base_to_world_tf.reset(new slam::tf::TFOperator(base_to_world_frame));
       manageRecordFile();
    }

    virtual ~BaseSensorManager(){}

    /**
     * @brief Enable public obstacle, used to turn on the shield.
     */
    virtual void enable(){
        enable_publish_obstacle_ = true;
    }

    virtual bool enableState(){ return enable_publish_obstacle_; }

    /**
     * @brief Disable public obstacle, used to shield obstacles.
     */
    virtual void disable(){
        enable_publish_obstacle_ = false;
    }

    virtual void processMsg(sros::core::base_msg_ptr& m){}

    virtual void setObaName(std::string map_name){};

    virtual void set_target_position(sros::core::Pose position){};

    SensorType getType() {
        return type;
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

    bool isRecording() const { return recorded_state_; }

    template <class DataType>
    void recordData(DataType& data){
        if (recorded_state_) {
            if (!file_op_.isOpen()) {
                auto curr_file_name = creatFileName("oba");
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

protected:
    /** @brief default allowed to public obstacle.*/
    bool enable_publish_obstacle_;

    ObaMsgCallback sendObaMsg;
    oba::ObstacleModulePara_Ptr para;
    std::shared_ptr<slam::tf::TFOperator> base_to_world_tf;
private:
    SensorType type;

    bool recorded_state_ = false;
    int record_incre_ = 0;
    int record_max_size_ = 100;//单次记录，只记录100张数据
    serialization::SerializeFileOperator file_op_;
};

typedef std::shared_ptr<BaseSensorManager> BaseSensorManager_Ptr;

}



#endif //SROS_BASE_SENSOR_MANAGER_HPP
