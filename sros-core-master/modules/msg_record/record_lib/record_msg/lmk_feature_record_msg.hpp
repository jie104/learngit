//
// Created by lfc on 17-11-20.
//

#ifndef SROS_LMK_FEATURE_RECORD_MSG_HPP
#define SROS_LMK_FEATURE_RECORD_MSG_HPP

#include "base_record_msg.hpp"
#include <Eigen/Dense>

namespace record {
class LmkFeatureRecordMsg : public BaseRecordMsg {

public:
    LmkFeatureRecordMsg():BaseRecordMsg(TYPE_RECORD_LMK) {
        pose.setZero();
        cov.setIdentity();
    }

    virtual ~LmkFeatureRecordMsg(){

    }

    virtual void encodeBody(std::ostream& record_stream) {//TODO:第三步,子类复写encodeBody虚函数
        writeName(convertToStr(lmk_id), record_stream);//将变量名编码
        writeData(lmk_id,record_stream);
        writeName(convertToStr(lmk_type), record_stream);
        writeData(lmk_type, record_stream);
        writeName(convertToStr(dim), record_stream);
        writeData(dim, record_stream);
        writeName(convertToStr(pose), record_stream);
        writeData(pose[0], record_stream);
        writeData(pose[1], record_stream);
        writeData(pose[2], record_stream);
        writeName(convertToStr(cov), record_stream);
        writeData(cov(0,0), record_stream);
        writeData(cov(0,1), record_stream);
        writeData(cov(0,2), record_stream);
        writeData(cov(1,0), record_stream);
        writeData(cov(1,1), record_stream);
        writeData(cov(1,2), record_stream);
        writeData(cov(2,0), record_stream);
        writeData(cov(2,1), record_stream);
        writeData(cov(2,2), record_stream);
        writeEndl(record_stream);//结束换行符
    }

    virtual void decodeBody(MsgMap& msg_map) {//TODO:第四步,子类复写decodeBody虚函数
        for (auto &iter:msg_map) {
            auto &value_name = iter.first;
            auto &values = iter.second;
            if (value_name == convertToStr(lmk_id)) {
                lmk_id = values[0];
            }else if (iter.first == convertToStr(lmk_type)) {
                lmk_type = values[0];
            }else if (value_name == convertToStr(dim)) {
                dim = values[0];
            }else if (value_name == convertToStr(pose)) {
                pose[0] = values[0];
                pose[1] = values[1];
                pose[2] = values[2];
            }else if(value_name == convertToStr(cov)) {
                cov(0, 0) = values[0];
                cov(0, 1) = values[1];
                cov(0, 2) = values[2];
                cov(1, 0) = values[3];
                cov(1, 1) = values[4];
                cov(1, 2) = values[5];
                cov(2, 0) = values[6];
                cov(2, 1) = values[7];
                cov(2, 2) = values[8];
            }
        }
    }

    int lmk_id = 0;
    int lmk_type = 0;
    int dim = 0;
    Eigen::Vector3f pose;//pose,如果是二维的,则只使用前两位
    Eigen::Matrix3f cov;//cov,如果是二维的,使用前四个数值

};

}


#endif //SROS_LMK_FEATURE_RECORD_MSG_HPP
