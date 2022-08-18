//
// Created by cwt on 2021/11/22.
//
#ifndef SDK_SVC200_H__
#define SDK_SVC200_H__

#include <termio.h>
#include <signal.h>
#include <vector>
#include <iostream>
#include <core/src.h>
#include <core/state.h>
#include <glog/logging.h>
#include <string>
#include "core/settings.h"
#include "../CqUsbCam/CqUsbCam.h"
#include "../CqUsbCam/SensorCapbablity.h"
#include "base_camera_device.hpp"


namespace camera {

/* SVC200 firmware version */
#define FIRMWARE_BASEVERSION            1L              /**< major version number */
#define FIRMWARE_SUBVERSION              0L              /**< minor version number */
#define FIRMWARE_REVISION                      0L              /**< revise version number */

#define FIRMWARE_VERSION            ((FIRMWARE_BASEVERSION * 1000 * 1000) +  (FIRMWARE_SUBVERSION * 1000) + FIRMWARE_REVISION)

#define E2PROM_SIZE                                 (0x1FFF - 0x0100)
#define E2PROM_ADDR_START               0x0
#define E2PROM_PAGE_OFFSET             0x10

// eeprom区域分割：P1+P2+P3
/******************************* 相机属性参数 ****************************************/
// P1实际地址分页处理：16*1024 KB 
// P1偏移索引最大值  = 16x1024/16 = 1024
// eeprom底层厂商没有做分页处理，应用层需要注意16字节对齐，否则会出现写入失败的情况!!
#define FIRMWARE_VERSION_ADDR                   (E2PROM_PAGE_OFFSET*0)
#define FIRMWARE_VERSION_LEN                    4
#define DEV_MODEL_ADDR                          (E2PROM_PAGE_OFFSET*1)
#define DEV_MODEL_LEN                           1
#define DEV_SN_ADDR                             (E2PROM_PAGE_OFFSET*2)
#define DEV_SN_LEN                              16
#define RESOLU_ADDR                             (E2PROM_PAGE_OFFSET*3)
#define RESOLU_LEN                              4
#define TRIG_MODEL_ADDR                         (E2PROM_PAGE_OFFSET*4)
#define TRIG_MODEL_LEN                          4
#define EXPO_MODEL_ADDR                         (E2PROM_PAGE_OFFSET*5)
#define EXPO_MODEL_LEN                          4
#define EXPO_VALUE_ADDR                        (E2PROM_PAGE_OFFSET*6)
#define EXPO_VALUE_LEN                          4
#define GAIN_MODEL_ADDR                        (E2PROM_PAGE_OFFSET*7)
#define GAIN_MODEL_LEN                          4
#define GAIN_VALUE_ADDR                         (E2PROM_PAGE_OFFSET*8)
#define GAIN_VALUE_LEN                          4
#define LIGHT_PWM_FREQ_ADDR                     (E2PROM_PAGE_OFFSET*9)
#define LIGHT_PWM_FREQ_LEN                      4
#define LIGHT_PWM_WIDTH_LEN                     4
#define LIGHT_PWM_WIDTH_ADDR                    (E2PROM_PAGE_OFFSET*10)

#define VALGORITHM_INTRINSIC1_ADDR               (E2PROM_PAGE_OFFSET*11)
#define VALGORITHM_INTRINSIC1_LEN                8
#define VALGORITHM_INTRINSIC2_ADDR              (E2PROM_PAGE_OFFSET*12)
#define VALGORITHM_INTRINSIC2_LEN                8
#define VALGORITHM_INTRINSIC3_ADDR               (E2PROM_PAGE_OFFSET*13)
#define VALGORITHM_INTRINSIC3_LEN                8
#define VALGORITHM_INTRINSIC4_ADDR               (E2PROM_PAGE_OFFSET*14)
#define VALGORITHM_INTRINSIC4_LEN                8
#define VALGORITHM_INTRINSIC5_ADDR               (E2PROM_PAGE_OFFSET*15)
#define VALGORITHM_INTRINSIC5_LEN                8
#define VALGORITHM_INTRINSIC6_ADDR              (E2PROM_PAGE_OFFSET*16)
#define VALGORITHM_INTRINSIC6_LEN                8
#define VALGORITHM_INTRINSIC7_ADDR               (E2PROM_PAGE_OFFSET*17)
#define VALGORITHM_INTRINSIC7_LEN                8
#define VALGORITHM_INTRINSIC8_ADDR               (E2PROM_PAGE_OFFSET*18)
#define VALGORITHM_INTRINSIC8_LEN                8
#define VALGORITHM_INTRINSIC9_ADDR               (E2PROM_PAGE_OFFSET*19)
#define VALGORITHM_INTRINSIC9_LEN                8

#define VALGORITHM_DISTORT1_ADDR                (E2PROM_PAGE_OFFSET*20)
#define VALGORITHM_DISTORT1_LEN                 8
#define VALGORITHM_DISTORT2_ADDR                (E2PROM_PAGE_OFFSET*21)
#define VALGORITHM_DISTORT2_LEN                 8
#define VALGORITHM_DISTORT3_ADDR                (E2PROM_PAGE_OFFSET*22)
#define VALGORITHM_DISTORT3_LEN                 8
#define VALGORITHM_DISTORT4_ADDR                (E2PROM_PAGE_OFFSET*23)
#define VALGORITHM_DISTORT4_LEN                 8
#define VALGORITHM_DISTORT5_ADDR                (E2PROM_PAGE_OFFSET*24)
#define VALGORITHM_DISTORT5_LEN                 8

#define VALGORITHM_VANISHING_POINT1_ADDR        (E2PROM_PAGE_OFFSET*25)
#define VALGORITHM_VANISHING_POINT1_LEN         8
#define VALGORITHM_VANISHING_POINT2_ADDR        (E2PROM_PAGE_OFFSET*26)
#define VALGORITHM_VANISHING_POINT2_LEN         8


#define  SVC_CURRENT                            "svc200_current"
#define  SVC_VOLTAGE                              "svc200_voltage"
#define  SVC_PWM_FREQ                        "svc200_pwm_freq"
#define  SVC_PWM_WIDTH                     "svc200_pwm_width"
#define  SVC_AUTO_GAIN_EXPO          "svc200_auto_gain_expo"
#define  SVC_VALUE_GAIN                      "svc200_value_gain"
#define  SVC_VALUE_EXPOSURE         "svc200_value_expo"
#define  SVC_TRIG_MODE                      "svc200_trig_mode"
#define  SVC_TRIG_FPGAFREQ            "svc200_trig_fpga_freq"
#define  SVC_TRIG_SOFTONCE           "svc200_trig_soft_once"
#define  SVC_MIRROR_TYPE                 "svc200_mirror_type"
#define  SVC_RESOLUTION                   "svc200_resolution"
#define  SVC_FIRMWARE_VERSION   "svc200_firmware_version"
/********************************** 备份参数区******************************/
// P2实际地址分页处理后：4*1024 KB 
// P2偏移索引最大值  = 4x1024/16 = 256, 还需要加上P1
// eeprom底层厂商没有做分页处理，应用层需要注意16字节对齐，否则会出现写入失败的情况!!
// 存储顺序与P1一致
/******************************* 相机属性参数 ********************************/
/******************************* 相机用户参数 ********************************/
// P3实际地址分页处理后：8*1024 KB 
// P3偏移索引最大值  = 8x1024/16 = 512, 还需要加上P1+P2
// eeprom底层厂商没有做分页处理，应用层需要注意16字节对齐，否则会出现写入失败的情况!!
union ConvertType2d {
    double x;
    uint8_t buf[8];
};

union ConvertType2f {
    float x;
    uint8_t buf[4];
};

union ConvertType2i {
    int32_t x;
    uint8_t buf[4];
};

#pragma pack(4)
struct VisionAlgorithm {
    double intrinsic[9];    // 相机内参
    double distort[5];       // 畸变系数
    double vanishingPoint[2]; // 中心点
};

struct CameraConfig {
    uint32_t Resolu;    // 图像分辨率
    uint32_t trigMode;   // 触发模式
    uint32_t expoModel;  // 曝光模式
    uint32_t expoValue;  // 设置软件触发时，曝光时间值
    uint32_t gainModel;  // 增益模式    
    uint32_t gainValue;  // 设置软件触发时，增益值 
    uint32_t lightPWMFreq;  // 补光灯pwm频率
    uint32_t lightPWMWidth;  // 补光灯pwm占空比 
};
#pragma pack()

enum Svc200Model {
    SVC200_NULL = 0,
    SVC200_D_LO_01,    /* 上视、下视，二维码定位 */
    SVC200_UD_LI_01,     /* 下视，纹理识别、二维码定位 */
    SVC200_S_LI_01,     /* 后视，二维码定位 */
    SVC200_NONE,        /* 无法识别型号 */
};

typedef struct CameraParam {
    struct CameraConfig config;
    struct VisionAlgorithm vAlgorithm;
} CameraParam_t;

typedef struct DevBind {
    int32_t deviceNo;   // 当前设备编号或句柄 
    string sn;
    enum Svc200Model model;
} DevBind_t;

typedef struct DeviceInfo {
	DevBind_t devBind;
	CameraParam_t cameraParam;
} DeviceInfo_t;


class SVC200ChipSelectSingleton {
public:
	static std::shared_ptr<SVC200ChipSelectSingleton> getInstance();
    void initializeAllDevices();
	int32_t getTotalNum(void);
	CCqUsbCam* getUsbCam();
	void  resetAllDevice(void);
	bool getCameraFDBySN(const std::string& camera_sn,int32_t& fd);
	bool getCameraDevBindBySN(const std::string& camera_sn,DevBind_t& devBind);
	bool getCameraParamBySN(const std::string& camera_sn,CameraParam_t& CamParam);
    int32_t findCameraDevice(CCqUsbCam &cam);
    int32_t checkSvcSN(CCqUsbCam &cam, uint8_t &model, string &e2promSN);
    void getCameraConfigIntrinsicParam(CCqUsbCam& cam, CameraParam_t& cameraParam);

private:
	SVC200ChipSelectSingleton() {
		initializeAllDevices();
	}
	

public:
	CCqUsbCam cam_;
	int32_t device_num_ = 0;
	const std::string sensor = "MT9V034";
	std::map<std::string,DeviceInfo_t>  dev_map_;		// std::string：cam_sn  
	static std::shared_ptr<SVC200ChipSelectSingleton >  svc200_singleton_; 
};


class SVC200CameraDevice : public BaseCameraDevice {
 public:
    SVC200CameraDevice(std::string device_address, ImgMsgCallback callback, const std::string &name, sros::device::DeviceID device_id);
    ~SVC200CameraDevice();

    virtual bool open();
    virtual bool isOpened();
    virtual void close();
    virtual bool enableCamera();
    virtual bool disableCamera();
    virtual bool isEnabled();
    virtual bool reset();
    virtual void getParameter(const std::string &name, double &value);
    virtual void setParameter(const std::string &name, const int value);
    virtual void getParameter(const std::string &name, int &value);
    bool initCamera();
    void handleSVC200Camera(void *frameData);
	void scanSvc200State();

 private:
    int32_t svc200SetResolu(uint8_t resolu);
    int32_t svc200SetMirrorType(const uint32_t mirrorType);
    int32_t svc200SetTrigMode(const uint32_t trigType);
    int32_t svc200SoftTrigOnce();
    int32_t svc200SetFpgaTrigFreq(const uint32_t freq);
    int32_t svc200SetAutoGainExpo(const bool autoGain, const bool autoExpo);
    int32_t svc200SetExpoValue(const uint32_t expoVal);
    int32_t svc200SetGainValue(const uint32_t gainVal);
    int32_t svc200SetLightPWMFreq(uint32_t freq);
    int32_t svc200SetLightPWMWidth(uint32_t width);
	uint16_t readAdc(CCqUsbCam* usbCam, uint8_t ch);
    void checkSpeed();


    bool open_flag_ = false;      //是否open camera的标志位 【 调用initCamera() 初始化相机 】
    bool read_flag_ = false;      //是否启动camera capture的标志位【调用cam.StartCap()启动capture 】
    bool enable_state_ = false;             //是否enable camera的标志位 【 调用enableCamera()使能相机 】
	bool is_running_ = false;
	int camera_fd_ = 0;
    std::string camera_sn_;
    const std::string sensor = "MT9V034";
    unsigned int  g_width = 640;
    unsigned int  g_height = 480;
	CCqUsbCam* cam_usb_ = NULL;
	std::shared_ptr<camera::SVC200ChipSelectSingleton> svc200_singleton_;
};

}
#endif //SDK_SVC200_H


