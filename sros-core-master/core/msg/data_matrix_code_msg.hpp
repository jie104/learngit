//
// Created by lfc on 2019/11/22.
//

#ifndef SROS_DATA_MATRIX_CODE_MSG_HPP
#define SROS_DATA_MATRIX_CODE_MSG_HPP
#include <memory>
#include <vector>
#include "base_msg.h"
#define DM_CODE_NOT_DETECTED 0
#define DM_CODE_DETECTED 1
#ifndef TYPE_DM_CODE
#define TYPE_DM_CODE 1
#endif
#ifndef TYPE_FM_CODE
#define TYPE_FM_CODE 2
#endif
#ifndef TYPE_SCAN_ANGLE_CODE
#define TYPE_SCAN_ANGLE_CODE 3
#endif
#ifndef TYPE_SCAN_LMK_CODE
#define TYPE_SCAN_LMK_CODE 4
#endif

namespace sros {
namespace core {
class DataMatrixCodeMsg : public BaseMsg {
 public:
    DataMatrixCodeMsg(std::string topic_name = "DM_CODE_INFO") : BaseMsg(topic_name, TYPE_DMCODE_DATA, true) {}

    virtual ~DataMatrixCodeMsg() {}

    uint64_t getTimestamp() const { return (uint64_t)time_; }

    void setTimestamp(const uint64_t timestamp) { time_ = timestamp; }

    virtual void getTime(){

    }

    void setCameraName(const std::string& name) { camera_name_ = name; }

    const std::string& getCameraName() const { return camera_name_; }

    void setCodeInfo(const int state, const int x, const int y, const int angle, const int code_int) {
        state_ = state;
        const double tithe_mm_to_m = 0.0001;
        const double milli_rad_to_rad = 0.001;
        x_ = (double)x * tithe_mm_to_m;
        y_ = (double)y * tithe_mm_to_m;
        angle_ = (double)angle * milli_rad_to_rad;
        code_int_ = code_int;
    }

    void setCodeInfo(const int state, const int x, const int y, const int z, const int angle, const int roll, const int pitch, const int code_int) {
        state_ = state;
        const double tithe_mm_to_m = 0.0001;
        const double milli_rad_to_rad = 0.001;
        x_ = (double)x * tithe_mm_to_m;
        y_ = (double)y * tithe_mm_to_m;
        z_ = (double)z * tithe_mm_to_m;

        angle_ = (double)angle * milli_rad_to_rad;
        roll_ = (double)roll * milli_rad_to_rad;
        pitch_ = (double)pitch * milli_rad_to_rad;
        code_int_ = code_int;
    }

    void getCodeInfo(int& state, double& x, double& y, double& angle, int& code_int) const {
        state = state_;
        x = x_;
        y = y_;
        angle = angle_;
        code_int = code_int_;
    }

    const std::string& getCodeStr() const { return code_str_; }

    void setCodeStr(const std::string& code_str) { code_str_ = code_str; }

    const int getCodeType(){ return code_type_; }

    void setCodeType(const int& code_type){code_type_ = code_type;}

    int state_ = DM_CODE_NOT_DETECTED;     //识别到为1,否则为0
    double x_ = 0;                         //单位为m,有效值0.0001m
    double y_ = 0;                         //单位为m,有效值0.0001m
    double angle_ = 0;                     //单位为rad,,有效值0.0001rad
    double z_ = 0;                         //单位为m,有效值0.0001m
    double roll_ = 0;                      //单位为rad,,有效值0.0001rad
    double pitch_ = 0;                     //单位为rad,,有效值0.0001rad
    int code_int_ = 0;                     //从二维码信息中提取出来的int值
    int code_type_ = TYPE_DM_CODE;         //1为dm_code;2为fm_code;3为scan_angle_code;4为scan_lmk_code;
    std::string code_str_;                 //完整的二维码编码信息
    std::string camera_name_;              //摄像头信息
    bool right_handed_system = true;

//    static const int TYPE_DM_CODE = 1;
//    static const int TYPE_FM_CODE = 2;
//    static const int TYPE_SCAN_ANGLE_CODE = 3;
//    static const int TYPE_SCAN_LMK_CODE = 4;
    
    double loc_x_err_ = 0;
    double loc_y_err_ = 0;
    double loc_yaw_err_ = 0;
};

typedef std::shared_ptr<DataMatrixCodeMsg> DataMatrixCodeMsg_ptr;
}  // namespace core
}  // namespace sros

#endif  // SROS_DATA_MATRIX_CODE_MSG_HPP
