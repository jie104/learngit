//
// Created by lfc on 17-1-5.
//

#ifndef LMKSLAM_RECORD_DEBUG_INFO_H
#define LMKSLAM_RECORD_DEBUG_INFO_H

#include <fstream>
#include <Eigen/Dense>
#include <iostream>
#include <memory>
#include "base_record.hpp"
#define name_to_str(value_name) #value_name
namespace record {
struct RecordPoseInfo {
    int64_t index;
    Eigen::Vector3f matpose;
    Eigen::Vector3f optpose;
    Eigen::Matrix3f cov;
    int numlmks;
    int nummatlmk;
};

typedef std::shared_ptr<RecordPoseInfo> RecordPoseInfo_Ptr;

class PoseRecord: public BaseRecord{
public:

    PoseRecord(std::string file_path):BaseRecord(BaseRecord::LMK_POSE_RECORD) {
        if(!openFile(file_path.c_str())) {
            LOG(INFO) << "err to open the file! failed to initialize!";
        }
        writeHeadName();

    }

    virtual ~PoseRecord(){

    }

    bool writePoseData(RecordPoseInfo_Ptr pose) {
        if (!pose) {
            LOG(INFO) << "the pose does not exist! will return!";
            return false;
        }
        writeData(pose->index);
        writePose(pose->matpose);
        writePose(pose->optpose);
        writeCov(pose->cov);
        writeData(pose->numlmks);
        writeData(pose->nummatlmk);
        writeEndl();
        return true;
    }




private:
    PoseRecord():BaseRecord(BaseRecord::LMK_POSE_RECORD){

    }

    void writeHeadName(){
        RecordPoseInfo record;
        std::string tmp_name = selectStrBydot(name_to_str(record.index));
        writeName(tmp_name);
        tmp_name = selectStrBydot(name_to_str(record.matpose));
        writePoseName(tmp_name);
        tmp_name = selectStrBydot(name_to_str(record.optpose));
        writePoseName(tmp_name);
        tmp_name = selectStrBydot(name_to_str(record.cov));
        writeCovName(tmp_name);
        tmp_name = selectStrBydot(name_to_str(record.numlmks));
        writeName(tmp_name);
        tmp_name = selectStrBydot(name_to_str(record.nummatlmk));
        writeName(tmp_name);
        writeEndl();
    }

};

typedef std::shared_ptr<PoseRecord> PoseRecord_ptr;
}


#endif //SROS_RECORD_DEBUG_INFO_H
