//
// Created by lhx on 18-7-20.
//

#ifndef SROS_DM_DETECTOR_H
#define SROS_DM_DETECTOR_H

#include "core/pose.h"

//#include <opencv2/opencv.hpp>
//#include <memory>
//#include "dm_code_reader.hpp"

class DmCodeReader;

enum DmDetectState {
    TYPE_DM_NONE = 1, //未识别到
    TYPE_DM_RECOGNIZED = 2, //含义是,检测到,识别到
    TYPE_DM_DETECTED = 3,//含义是,只检测到,未识别到
};

class DMResult {
public:
    DmDetectState detect_state;

    std::string code_str;

    double x_offset; // 单位mm
    double y_offset; // 单位mm
    double angle_offset; // 单位rad
    double confidence = 0.0f;
};

struct Camera6DPose{
    DmDetectState detect_state;
    std::string code_str;
    double confidence = 0.0f;

    double x;           // 单位mm
    double y;           // 单位mm
    double z;           // 单位mm
    double roll;        //绕x轴旋转角,单位°
    double pitch;       //绕Y轴旋转角,单位°
    double yaw;         //绕Z轴旋转角,单位°
};

class DataMatrixDetector {
public:
    DataMatrixDetector();

    ~DataMatrixDetector();

    void setBackgroundColor(std::string background_color); //DARK or BRIGHT 设置亮暗背景

    void setCameraParam(std::vector<double>& camera_param,std::vector<double>& disort_param);

    DMResult detect(cv::Mat& image, bool debug_output = false);

    std::string decodeDataMatrix(const cv::Mat& img, std::vector<cv::Point>& corner_points);

    bool savePngImg(std::string m_filename,const cv::Mat &img);

    static int getVersion();

    static std::string getVersionStr();

    void setRoughLocationMethod(std::string method);

    void setDMCodeRunflag(bool runstate);

    void setRunCodeState(bool detect_circle_flag);

    void setDMCodeCirflag(bool runstate);

    void setCodeType(std::string type);

    void setVanishPoint(std::vector<double>& vanish_param);

    void setCameraInstallType(std::string type);

    bool getCamera_6d_pose(Camera6DPose &pose);

    void setcirringvale(int mincirring,int maxcirring);

 private:
    Camera6DPose camera_6d_pose;
    std::shared_ptr<DmCodeReader> dm_reader;
    std::string background_color_ = "BRIGHT";
    std::vector<double> camera_param_;
    std::vector<double> disort_param_;
    std::vector<double> vanish_param_;
    std::string recog_method_;
    std::string code_type_;
    std::string camera_install_type_;   //相机在机器人上安装位置
    DMResult lastsult;
};


#endif //SROS_DM_DETECTOR_H
