//
// Created by lfc on 2020/12/23.
//

#ifndef SROS_FEATURE_INFO_MSG_HPP
#define SROS_FEATURE_INFO_MSG_HPP
#include "base_msg.h"
#include "core/pose.h"

namespace sros{
namespace core{
class FeatureInfoMsg: public BaseMsg {
 public:
    struct FeatureInfo{
        Pose pose_in_world_;//识别到的特征在世界坐标系坐标
        std::vector<Location> points_;//该特征相关联的特征点
        std::string feature_name_;//特征名称，用来标记特征
        std::string sensor_name_;
        int code_type_;//特征ID，该
    };

    FeatureInfoMsg(std::string topic_name = "FEATURE_INFO") : BaseMsg(topic_name, TYPE_DMCODE_DATA, true) {

    }

    virtual ~FeatureInfoMsg(){}


    uint64_t getTimestamp() const { return (uint64_t)time_; }

    void setTimestamp(const uint64_t timestamp) { time_ = timestamp; }

    virtual void getTime(){

    }

    std::vector<FeatureInfo> feature_infos;
 private:


};

typedef std::shared_ptr<FeatureInfoMsg> FeatureInfoMsg_ptr;
}
}


#endif  // SROS_FEATURE_INFO_MSG_HPP
