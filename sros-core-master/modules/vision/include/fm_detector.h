//
// Created by orange on 20-11-12.
//


#include <opencv2/opencv.hpp>
#include <memory>

class fmCodeReader;

enum FmDetectState {
    TYPE_FM_NONE = 1, //未识别到
    TYPE_FM_RECOGNIZED = 2, //含义是,检测到,识别到
    TYPE_FM_DETECTED = 3,//含义是,只检测到,未识别到
};

//返回二维码坐标下相机的姿态
class FMResult {
public:
    FmDetectState detect_state;

    std::string code_str;

    double x_offset; // 单位mm
    double y_offset; // 单位mm
    double z_offset; // 单位mm
    double angle_roll_offset; // 单位rad
    double angle_yaw_offset; // 单位rad
    double angle_pitch_offset; // 单位rad
    double confidence = 0.0f;
};

class FractalMarkerDetector {
public:
    FractalMarkerDetector();

    ~FractalMarkerDetector();

    /** save Image
     * savePngImg
     * @param m_filename
     * @param img
     * @return
     */
    bool savePngImg(std::string m_filename,const cv::Mat &img);
    
    /**
     * @brief 设置背景是亮还是黑,及反色
     * @param background_color
     */
    void setBackgroundColor(std::string background_color);

    /**
     * @brief 设置二维码的真实尺寸
     * @param marker_size 单位m
     */
    void setCodeParam(double marker_size);

    /** 设置相机内参
     * setCameraParam
     * @param camera_param_yaml
     */
    void setCameraParam(std::string camera_param_yaml);

    /** 设置相机内参
     * setCameraParam
     * @param camera_param
     * @param disort_param
     */
    void setCameraParam(std::vector<double>& camera_param,std::vector<double>& disort_param);

    /** 设置鱼眼相机内参
     * setFishEyeCameraParam
     * @param camera_param 内参
     * @param disort_param 畸变
     * @param img_size 图像尺寸
     * @param xi
     * @param scale
     */
    void setFishEyeCameraParam(std::vector<double>& camera_param,std::vector<double>& disort_param, cv::Size img_size,double xi, double scale);

    /** 二维码检测并定位，只返回距离最近的码结果
     * detect
     * @param image
     * @param debug_output
     * @return
     */
    FMResult detect(cv::Mat& image, bool debug_output = false);

    /** 获取所有码结果
     * getAllFmResult
     * @return
     */
    std::vector<FMResult> getAllFmResult();

    static int getVersion();

    static std::string getVersionStr();

    /** 设置码型
     * "ARUCO_MIP_36h12"
     * "TAG36h11"
     * "TAG36h10"
     * "ALL_DICTS"
     * "FRACTAL_MARKER"
     * setCodeType
     * @param type
     */
    void setCodeType(std::string type);


private:
    std::shared_ptr<fmCodeReader> fm_reader;

    std::string type;
    std::string camera_param_yaml_;
    std::string background_color_ = "BRIGHT";  //默认的二维码背景
    float marker_size;                         //单位m
    
    std::vector<FMResult> results;
    std::vector<double> camera_param_;
    std::vector<double> disort_param_;
    std::vector<double> vanish_param_;
    std::string recog_method_;
    std::string code_type_;
    std::string camera_install_type_;   //相机在机器人上安装位置


};
