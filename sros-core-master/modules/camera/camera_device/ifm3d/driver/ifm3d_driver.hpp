#ifndef IFM3D_DRIVER_HPP_
#define IFM3D_DRIVER_HPP_

//#include <ifm3d/Config.h>
//#include <ifm3d/Dump.h>
//#include <ifm3d/Extrinsics.h>
//#include <ifm3d/Trigger.h>
#include <glog/logging.h>

#include <unistd.h>
#include <chrono>
#include <memory>
#include <mutex>
#include <utility>
#include "opencv/highgui.h"
namespace ifm3d {
class Camera;
class FrameGrabber;
class ImageBuffer;
}  // namespace ifm3d

namespace ifm {
    // 注意: 特别约定index==1的"Imager"参数项用于正常功能, index==2的用于标定
    const int DEFAULT_NORMAL_INDEX = 1;
    const int DEFAULT_CALIB_INDEX = 2;

    const std::vector<std::string> TypeName = {
        "upto02m_low",               //2米低曝光
        "upto02m_moderate",          //2米中曝光
        "upto02m_high",              //2米高曝光
        "upto03m_low",               //3米低曝光
        "upto03m_moderate",          //3米中曝光
        "upto03m_high",              //3米高曝光
        "under5m_low",               //5米低曝光
        "upto07m_low",               //7米低曝光
        "upto07m_moderate",          //7米中曝光
        "upto07m_high",              //7米高曝光
        "upto15m_low",               //15米低曝光
        "upto15m_moderate",          //15米中曝光
        "upto15m_high",              //15米高曝光
        "upto30m_low",               //30米低曝光
        "upto30m_moderate",          //30米中曝光
        "upto30m_high"               //30米高曝光
    };

class Ifm3dDriver {
 public:
    Ifm3dDriver();

    void setIPAddress(std::string ip) { camera_ip_ = std::move(ip); }

    bool init(uint16_t mask = 15);

    bool getFrame(uint64_t &stamp, cv::Mat &confidence_img, cv::Mat &xyz_img);

    /**
     * @brief get 3d camera data form driver.
     * @param[out] stamp The time point at which the device gets the frame.
     * @param[out] confidence_img Credibility of pixels.
     * @param[out] amplitude_img The amplitude of a pixel.
     * @param[out] xyz_img Point cloud data.
     * @return True is returned if the frame is retrieved successfully.
     *         otherwise false is returned.
     */
    bool getFrame(uint64_t& stamp, cv::Mat& confidence_img,
                  cv::Mat &amplitude_img, cv::Mat& xyz_img, float& fork_height);

    bool tryInit(int max_try_times = 10) {
        int try_time = 0;
        while (try_time++ < max_try_times) {
            if (init()) {
                return true;
            }
            sleep(1);
        }
        return false;
    }

    /**
     * @brief 初始化o3d相机参数驱动接口.
     * @return 成功返回true，失败返回false.
     */
    bool initIfm3dParam();

    /**
     * @brief 设置o3d相机单个参数驱动接口.
     * @param[out] name 要设置的参数名.
     * @param[out] value 要设置的参数的值.
     * @return 成功返回true，失败返回false.
     */
    bool setIfm3dParam(std::string name, std::string value);

    /**
     * @brief 获取o3d相机单个参数驱动接口.
     * @param[out] name 要获取的参数名.
     * @return 成功返回true，失败返回false.
     */
    bool getIfm3dParam(std::string name);

    /**
     * @brief 设置o3d相机开始标定和结束标定参数驱动接口.
     * @param[out] name_str 要设置的参数名数组.
     * @param[out] value_str 对应要设置的参数值数组.
     * @param[out] size 要设置的参数个数.
     * @return 成功返回true，失败返回false.
     */
    bool setIfm3dCalibrParam(std::string name_str[], std::string value_str[], int size);

    void close() { opened_ = false; }

    bool opened() const { return opened_; }

    bool getInitParamFlag() {
        return init_param_flag_;
    }
    
    void setInitParamFlag(bool flag) {
        init_param_flag_ = flag;
    }

    /**
     * @brief 获取o3d相机 当前处于激活状态的apps配置
     * @param[in] void
     * @return[int] index  对应JSON格式 "Imager" 参数项的 index
     */
    int getAvtiveAppsIndex();

    /**
     * @brief 切换o3d相机的apps配置. 初始化中预定义了两组"Imager"参数.通过index选择其中一组生效
     * @param[in] index 对应JSON格式 "Imager" 参数项的 Index
     * @return 成功返回true，失败返回false.
     */
    bool setAvtiveAppImagerIndex(int index);

    /**
     * @brief 临时设置o3d相机曝光, 参数能快速生效,断电不保存,调用了Session / Json相关接口会丢失
     * @param[in] exposurreTimeUs 要设置的曝光参数,单位: microsecond, 如果是多重曝光,代表长帧
     * @param[in] exposureTimeRatio 要设置的参曝光retio, 仅在多重曝光的时候, 用于计算短帧曝光
     * @return 成功返回true，失败返回false.
     */
    bool setIfm3dAppsImagerExposureTemporary(int exposurreTimeUs, int exposureTimeRatio);

    /**
     * @brief 设置o3d相机曝光, 此接口运行需要比较长的时间,参数可断电保存
     * @param[in] exposurreTimeUs 要设置的曝光参数,单位: microsecond, 如果是多重曝光,代表长帧
     * @param[in] exposureTimeRatio 要设置的参曝光retio, 仅在多重曝光的时候, 用于计算短帧曝光
     * @return 成功返回true，失败返回false.
     */
    bool setIfm3dAppsImagerExposureParams(int exposurreTimeUs, int exposureTimeRatio);

    /**
     * @brief 设置o3d相机曝光Type, 此接口运行需要比较长的时间,参数可断电保存
     * @param[in] str_type_param 要设置的曝光Type, 普通字符串,取值范围
     * @return 成功返回true，失败返回false.
     */
    bool setIfm3dAppsImagerTypeParam(const std::string str_type_param);

    /**
     * @brief 设置o3d相机 任意参数, 由输入指定 此接口运行需要比较长的时间,参数可断电保存
     * @param[in] str_json_config 要设置的参数(String), 字符串必须符合Json格式
     * @return 成功返回true，失败返回false.
     */
    bool setIfm3dJsonStrParam(const std::string str_json_config);

    /**
     * @brief 设置o3d相机 软件触发模式
     * @param[in] enable；true/false
     * @return 成功返回true，失败返回false.
     */
    bool setSoftwareTriggerMode(const bool enable);

    /**
     * @brief 读取软件触发方式配置
     * @param[in]
     * @return 成功返回true，失败返回false.
     */
    bool getSoftwareTriggerModeEnabled();

    /**
     * @brief 重启o3d相机
     * @return 成功返回true，失败返回false.
     */
    bool rebootIfm3dCamera();

    const std::string getSerialNumberStr();

    const std::string getDeviceNameStr();

 private:
    std::shared_ptr<ifm3d::Camera> cam_;
    std::shared_ptr<ifm3d::FrameGrabber> fg_;
    std::shared_ptr<ifm3d::ImageBuffer> im_;
    std::mutex mutex_;

    std::string camera_ip_;
    int xmlrpc_port_;
    std::string password_;
    std::string serial_number_str_;

    int timeout_millis_ = 500;
    bool opened_ = false;
    const int64_t timeout_1s = 1000;
    bool init_param_flag_ = false;
    bool software_trigger_enabled_;
};

}  // namespace ifm

#endif  // IFM3D_DRIVER_HPP
