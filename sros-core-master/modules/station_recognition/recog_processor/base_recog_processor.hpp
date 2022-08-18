//
// Created by lfc on 17-10-16.
//

#ifndef SROS_BASE_RECOG_PROCESSOR_HPP
#define SROS_BASE_RECOG_PROCESSOR_HPP

#include <memory>
#include <Eigen/Dense>
#include <core/msg/laser_scan_msg.hpp>
#include "feat_recog/base_recognition.hpp"
namespace recog{
//para内部包含了所有特征提取器的参数
struct RecogPara{
    //针对于角特征
    virtual ~RecogPara(){};
    float angle_feat_theta;
    float angle_feat_dist_1;
    float angle_feat_dist_2;

    float intensity_thresh = 800;
    float split_dist = 0.05;
    int num_count = 5;
};

struct RecogRackPara: public RecogPara{
    virtual ~RecogRackPara(){};
    float rack_length = 1.02;
    float rack_width = 0.569;
};

struct RecogChargingPilePara:public RecogPara{
    virtual ~RecogChargingPilePara(){};
    float charging_pile_width = 0.38;
    float reflective_stickers_length = 0.05;

};

typedef std::shared_ptr<RecogPara> RecogPara_Ptr;

struct RecogInfo{
    Eigen::Vector3f pose;
    Eigen::Matrix3f cov;
};

struct RecogInfos{
    std::vector<RecogInfo> recog_infos;
};

typedef std::shared_ptr<RecogInfos> RecogInfos_Ptr;

enum RecogProType{
    TYPE_RECOGPRO_ANGLE = 1,
    TYPE_RECOGPRO_LMKRACK = 2,
    TYPE_RECOGPRO_CHARGINGPILE=3,
};
class BaseRecogProcessor {
public:
    BaseRecogProcessor(RecogProType type_):type(type_){

    }

    virtual ~BaseRecogProcessor(){

    }


    void setRecogPara(RecogPara_Ptr recog_para_) {
        recog_para = recog_para_;
        setProcessorPara();
    };

    virtual RecogInfos_Ptr getRecogInfo(sros::core::LaserScan_ptr scan,Eigen::Vector3f laser_pose) {
        return 0;
    }

protected://仅仅在测试的时候为ｐｕｂｌｉｃ　默认为protected
    BaseRecognition_Ptr feature_recog_;
    virtual void setProcessorPara(){

    }
    RecogPara_Ptr recog_para;
private:
    BaseRecogProcessor(){

    }
private:
    RecogProType type;



};

typedef std::shared_ptr<BaseRecogProcessor> BaseRecogProcessor_Ptr;
typedef std::shared_ptr<RecogPara> BaseRecogPara_Ptr;

}


#endif //SROS_BASE_RECOG_PROCESSOR_HPP
