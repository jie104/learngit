//
// Created by neethan on 2021/7/28.
//
#ifndef SDK_SVC200_H
#define SDK_SVC200_H


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

enum SvcPosition {
    SVC_POS_NONE = 0,
    SVC_POS_UP,    /* 上视 */
    SVC_POS_DOWN,  /* 下视 */
    SVC_POS_FRONT, /* 前视 */
    SVC_POS_BACK,  /* 后视 */
};

struct CameraParam {
    struct CameraConfig config;
    struct VisionAlgorithm vAlgorithm;
};

typedef struct DevBind {
    int32_t deviceNo;   // 当前设备编号或句柄 
    string sn;
    enum Svc200Model model;
} DevBind_t;


static uint16_t ReadAdc(CCqUsbCam *usbCam, uint8_t ch)
{
    uint8_t rxval[2];
    uint8_t  chData[64];

    switch (ch)
    {
        case 0:
            ch = 0x88;
            break;
        case 1:
            ch = 0xc8;
            break;
    }
    arbFuncStruct arb;

    memset(chData,0,64);
    arb.order.pData = chData;
    arb.order.ReqCode = 0xFF;
    arb.order.DataBytes = 2;
    arb.order.Direction = USB_ORDER_IN;
    arb.order.Index = ch & 0xff;
    usbCam->ArbFunc(&arb);

    memcpy(rxval, chData, 2);
    uint16_t irxval = rxval[0] << 8;
    irxval += rxval[1];
    irxval=irxval/16;
    printf("read ADC:%d\n",irxval);
    return irxval;
}

static int64_t getTimeStamp64(uint8_t* p)
{
    int64_t timestamp = 0;
    int j = 2;
    for (int i = 7; i >= 4; i--)
    {
        int64_t temp = p[j];
        j = j + 1;
        timestamp += temp<< i * 8;
    }
    j = 10;
    for (int i = 3; i >= 0; i--)
    {
        int64_t temp = p[j];
        timestamp += temp<< i * 8;
        j = j + 1;
    }
    return timestamp;
}

static const char*  intrinsic_name[] = {"intrinsicParam1", "intrinsicParam2", "intrinsicParam3", "intrinsicParam4",
                                        "intrinsicParam5", "intrinsicParam6", "intrinsicParam7","intrinsicParam8", "intrinsicParam9"};
static const char* distor_name[] = {"distortParam1", "distortParam2",   "distortParam3",  "distortParam4",  "distortParam5"};
static const char* vanishingPoint_name[] = {"vanishingPointParam1", "vanishingPointParam2"};
static list<DevBind_t>  devBindList;

class SVC200CameraDevice : public BaseCameraDevice {
 public:
    SVC200CameraDevice(std::string device_address, ImgMsgCallback callback, const std::string &name, sros::device::DeviceID device_id)
        : camera_name_(device_address),
          BaseCameraDevice(callback,
                           name,
                           device_id,
                           sros::device::DEVICE_COMM_INTERFACE_TYPE_USB_0,
                           sros::device::DEVICE_MOUNT_SROS) {

        int32_t ret;
        DevBind_t  info;
        m_pCamInUse = NULL;

        ret = pthread_mutex_init(&m_mutexDisp, NULL);
        if(ret){
            printf("pthread_mutex_init m_mutexDisp failed\n");
        }
        ret = pthread_mutex_init(&m_mutexCam, NULL);
        if(ret){
            printf("pthread_mutex_init m_mutexCam failed\n");
        }

        cam0.SelectSensor(sensor);
        FindCameraDevice(cam0, TotalDeviceNum,devBindList);

        auto &s = sros::core::Settings::getInstance();
        //std::string  svc100_camera = s.getValue<std::string>("camera.svc200_model", " ");
        SVC200_GetDevInfo(name,  info);
        if(SVC200_default_Id == -1) {
            LOG(INFO) << "!!!!!svc200 device found failed, end the driver";
            return;
        }
        open();

        int  trig_mode = 0;
        std::string  svc200_trigger_mode = s.getValue<std::string>("camera.trigger_mode", " ");
        if(svc200_trigger_mode.compare("自动触发") == 0) {
            trig_mode = TRIGMODE_AUTO;
        } else if(svc200_trigger_mode.compare("FPGA触发") == 0) {
            trig_mode = TRIGMODE_FPGA;
        } else if(svc200_trigger_mode.compare("软件单次触发") == 0) {
            trig_mode = TRIGMODE_SOFT;
        } else {
            LOG(INFO) << "!!!!!svc200 trigger set error, no the trigger type.";
        }
        usleep(20000);
        setParameter(SVC_TRIG_MODE,  trig_mode);
        if(trig_mode == TRIGMODE_SOFT) {
            setParameter(SVC_TRIG_SOFTONCE, 0);
        }
    }

    ~SVC200CameraDevice(){
        close();
    }

    virtual bool open() {
        SVC200_open_flag = false;
        //    disableCamera();
        setStateInitialization();
        const int try_times_thresh = 1;
        int curr_time = 0;

        while (curr_time++ < try_times_thresh) {
            SVC200_open_flag = initCamera(SVC200_default_Id);
            if (SVC200_open_flag) {
                LOG(INFO) << "open SVC200 Successful";
                setStateOK();
                break;
            } else {
                LOG(INFO) << "cannot open ! try again!";
            }
        }
        if (!isOpened()) {
            LOG(ERROR)<<"open SVC200 Failure: cannot open SVC200 "<<SVC200_default_Id;
            setStateOpenFailed();
            return false;
        }
        m_pCamInUse = &cam0;

        int read_result = cam0.StartCap(g_height, g_width,  std::bind(&SVC200CameraDevice::handleSVC200Camera, this, std::placeholders::_1));
        LOG(INFO)<<"open read_result : "<<read_result;
        if(read_result != 0) {
            SVC200_read_flag = false;
            LOG(ERROR) << "open SVC200 Failure: SVC200 readCap failure";
            return false;
        }
        SVC200_read_flag = true;
        initCq = true;
        LOG(INFO) << "!!!!!open SVC200 Successful!";
        return true;
    }

    virtual bool isOpened() {
        return SVC200_open_flag;
    }

    virtual void close() {
        SVC200_open_flag = false;
        if(this->SVC200_read_flag){
            cam0.StopCap();
            SVC200_read_flag = false;
        }

        //    disableCamera();
        cam0.ReleaseInterface();
        cam0.CloseUSB();
        LOG(INFO) << "!!!!!close SVC200!";
    }

    virtual bool enableCamera() {
        if (!SVC200_open_flag || initCq) {
            initCq = false;
            setStateInitialization();
            SVC200_open_flag = initCamera(SVC200_default_Id);
            if (!SVC200_open_flag) {
                LOG(ERROR)<<"!!!!!enable SVC200 Failure: cannot open SVC200 "<<SVC200_default_Id;
                return false;
            }
            setStateOK();
        }

        if(SVC200_read_flag) {
            cam0.StopCap();
        }

        enable_state_ = true;
        int read_result = cam0.StartCap(g_height, g_width,  std::bind(&SVC200CameraDevice::handleSVC200Camera, this, std::placeholders::_1));
        LOG(INFO)<<"!!!!!cam0 StartCap read_result : "<< read_result;
        if(read_result != 0){
            SVC200_read_flag = false;
            LOG(ERROR) << "!!!!!cam0 StartCap Failure: SVC200 readCap failure";
            return false;
        }
        SVC200_read_flag = true;
        LOG(INFO) << "!!!!!enable SVC200 Successful!";

        return true;
    }

    virtual bool disableCamera() {
        if(this->SVC200_read_flag){
            cam0.StopCap();
            SVC200_read_flag = false;
        }
        enable_state_ = false;
        LOG(INFO) << "!!!!!disable SVC200!";
        return true;
    }

    virtual bool isEnabled(){
        return enable_state_;
    };

    virtual bool reset() {
        return  SrReset(SVC200_default_Id);
    }

    /*
      *     函数功能：获取相机的电流/电压 ADC值; 获取相机的内参、畸变参数、标定中心等参数。
      *     形参列表：
      *         name：指定获取哪个参数的数据
      *         value：获取到的数值
      *     返回值：无
      *     参数列表：
      *         SVC200_CURRENT  -- SVC200_Current 
      *         SVC200_VOLTAGE -- SVC200_Voltage
      *     
      *     uint8_t*  intrinsic_name[9] = {"intrinsicParam1", "intrinsicParam2",...};
      *     uint8_t* distor_name[5] = {"distortParam1", "distortParam2", ... };
      *     uint8_t* vanishingPoint_name[2] = {"vanishingPointParam1", "vanishingPointParam2"};
    */
    virtual void getParameter(const std::string &name, double &value) {
        uint16_t ad = 0;
        uint16_t i = 0;
        uint32_t addr = 0;
        uint32_t len = 8;
        int  ret = 0;
        union ConvertType2d ct2d;

        if(NULL == m_pCamInUse)
            return;

        /*   从相机中读取相机内参 1-9 */
        for(i = 0; i < 9; i++) {
            addr  = E2PROM_PAGE_OFFSET*(11+i);
            if(name.compare(intrinsic_name[i]) == 0) {
                memset(&ct2d, 0, sizeof(ct2d));
                ret = cam0.RdEeprom(addr, ct2d.buf, len);
                if (ret == len){
                    value = ct2d.x;
                    LOG(INFO)  << "!!!!!intrinsic_addr:" << addr << ", value:" << ct2d.x;
                } else {
                    memset(&ct2d, 0, sizeof(ct2d));
                    ret = cam0.RdEeprom(addr, ct2d.buf, len);
                    if(ret != len){
                        LOG(INFO)  << "!!!!!intrinsic param read failed.";
                    }
                }
                return;
            }
        }
        /*  从相机中读取畸变参数 1-5  */
        for(i = 0; i < 5; i++) {
            addr  = E2PROM_PAGE_OFFSET*(20+i);
            if(name.compare(distor_name[i]) == 0) {
                memset(&ct2d, 0, sizeof(ct2d));
                ret = cam0.RdEeprom(addr, ct2d.buf, len);
                if (ret == len){
                    value = ct2d.x;
                    LOG(INFO)  << "!!!!!distor_addr:" << addr << ", value:" << ct2d.x;
                } else {
                    memset(&ct2d, 0, sizeof(ct2d));
                    ret = cam0.RdEeprom(addr, ct2d.buf, len);
                    if(ret != len){
                        LOG(INFO)  << "!!!!!distor param read failed.";
                    }
                }
                return;
            }
        }
        /*   从相机中读取标定中心点参数 1-2 */
        for(i = 0; i < 2; i++) {
            addr  = E2PROM_PAGE_OFFSET*(25+i);
            if(name.compare(vanishingPoint_name[i]) == 0) {
                memset(&ct2d, 0, sizeof(ct2d));
                ret = cam0.RdEeprom(addr, ct2d.buf, len);
                if (ret == len){
                    value = ct2d.x;
                    LOG(INFO)  << "!!!!!vanishingPoint_addr:" << addr << ", value:" << ct2d.x;
                } else {
                    memset(&ct2d, 0, sizeof(ct2d));
                    ret = cam0.RdEeprom(addr, ct2d.buf, len);
                    if(ret != len){
                        LOG(INFO)  << "!!!!!vanishingPoint param read failed.";
                    }
                }
                return;
            }
        }
        /*  从相机中读取电压/电流值  */
        if(name.compare(SVC_CURRENT) == 0) {
            ad = ReadAdc(m_pCamInUse, 0);
            value = ad/10;        // mA
        } else if(name.compare(SVC_VOLTAGE) == 0) {
            ad = ReadAdc(m_pCamInUse, 1);
            value = ad*21/1000+0.55;    // V
        } else {
            LOG(WARNING) << "!!!!!SVC200, double parameter error!";
        }
    }

    /*
      *     函数功能：设置相机各种参数【  PWM频率、PWM占空比、增益模式、曝光模式、
      *          增益值、曝光值、FPGA触发频率、软件一次触发、相机触发模式、图像镜像类型、相机分辨率 】
      *     形参列表：
      *         name：指定要设置的参数
      *         value：要设置的参数值
      *     返回值：无
      * 
      *     参数列表：
      *         SVC_PWM_FREQ
      *         SVC_PWM_WIDTH
      *         SVC_AUTO_GAIN_EXPO
      *         SVC_VALUE_GAIN
      *         SVC_VALUE_EXPOSURE
      *         SVC_TRIG_MODE
      *         SVC_TRIG_FPGAFREQ
      *         SVC_TRIG_SOFTONCE
      *         SVC_MIRROR_TYPE
      *         SVC_RESOLUTION
    */
    virtual void setParameter(const std::string &name, const int value) {
        int ret = 0;
        if(NULL == m_pCamInUse)
            return;

        if(name.compare(SVC_PWM_FREQ) == 0) {
            ret = SrSetLightPWMFreq(value);
            if(ret)
                LOG(WARNING) << "!!!!!SVC200, set pwm freq failed!";
        } else if(name.compare(SVC_PWM_WIDTH) == 0) {
            ret = SrSetLightPWMWidth(value);
            if(ret)
                LOG(WARNING) << "!!!!!SVC200, set pwm width failed!";
        } else if(name.compare(SVC_AUTO_GAIN_EXPO)  == 0) {
            SrSetAutoGainExpo(1, 1);
        } else if(name.compare(SVC_VALUE_GAIN) == 0) {
            SrSetAutoGainExpo(0, 1);
            SrSetGainValue(value);
        } else if(name.compare(SVC_VALUE_EXPOSURE) == 0) {
            SrSetAutoGainExpo(1, 0);
            SrSetExpoValue(value);
        } else if(name.compare(SVC_TRIG_MODE) == 0) {
            SrSetTrigMode(value);
        } else if(name.compare(SVC_TRIG_FPGAFREQ) == 0) {
            SrSetFpgaTrigFreq(value);
        } else if(name.compare(SVC_TRIG_SOFTONCE) == 0) {
            SrSoftTrigOnce();
        } else if(name.compare(SVC_MIRROR_TYPE) == 0) {
            SrSetMirrorType(value);
        } else if(name.compare(SVC_RESOLUTION) == 0) {
            SrSetResolu(value);
        } else {
            LOG(WARNING) << "!!!!!SVC200, int parameter error!";
        }
    }

    /*
     *     函数功能：获取相机各种算法参数
     *         【  PWM频率、PWM占空比、相机触发模式、相机分辨率  】
     *     形参列表：
     *         name：指定要获取的参数
     *         value：获取到的参数值
     *     返回值：无
     *
     *     #define  SVC_PWM_FREQ                        "svc200_pwm_freq"
     *     #define  SVC_PWM_WIDTH                     "svc200_pwm_width"
     *     #define  SVC_AUTO_GAIN_EXPO          "svc200_auto_gain_expo"
     *     #define  SVC_VALUE_GAIN                      "svc200_value_gain"
     *     #define  SVC_VALUE_EXPOSURE         "svc200_value_expo"
     *     #define  SVC_TRIG_MODE                      "svc200_trig_mode"
     *     #define  SVC_MIRROR_TYPE                 "svc200_mirror_type"
     *     #define  SVC_RESOLUTION                   "svc200_resolution"
     *     #define  SVC_FIRMWARE_VERSION   "svc200_firmware_version"
   */
    virtual void getParameter(const std::string &name, int &value) {
        if(NULL == m_pCamInUse)
            return;

        union ConvertType2i ct2i;
        uint32_t addr = 0;
        uint32_t len = 4;

        if(name.compare(SVC_FIRMWARE_VERSION) == 0) {
            memset(&ct2i, 0, sizeof(ct2i));
            cam0.RdEeprom(FIRMWARE_VERSION_ADDR, ct2i.buf, len);
            value = ct2i.x;
            LOG(INFO) << "!!!!!FIRMWARE_VERSION: " <<  ct2i.x;
        } else if(name.compare(SVC_PWM_FREQ) == 0) {
            memset(&ct2i, 0, sizeof(ct2i));
            cam0.RdEeprom(LIGHT_PWM_FREQ_ADDR, ct2i.buf, len);
            if(100 <= ct2i.x && ct2i.x <= 50000) {
                value = ct2i.x;
                LOG(INFO) << "PWM_FREQ: " <<  ct2i.x;
            }
        } else if(name.compare(SVC_PWM_WIDTH) == 0) {
            memset(&ct2i, 0, sizeof(ct2i));
            cam0.RdEeprom(LIGHT_PWM_WIDTH_ADDR, ct2i.buf, len);
            if(0 <= ct2i.x && ct2i.x <= 100) {
                value = ct2i.x;
                LOG(INFO) << "PWM_WIDTH: " <<  ct2i.x;
            }
        } else if(name.compare(SVC_VALUE_GAIN) == 0) {
            memset(&ct2i, 0, sizeof(ct2i));
            cam0.RdEeprom(GAIN_VALUE_ADDR, ct2i.buf, len);
            if(0 <= ct2i.x && ct2i.x <= 64) {
                value = ct2i.x;
                LOG(INFO) << "GAIN_VALUE: " <<  ct2i.x;
            }
        } else if(name.compare(SVC_VALUE_EXPOSURE) == 0) {
            memset(&ct2i, 0, sizeof(ct2i));
            cam0.RdEeprom(EXPO_VALUE_ADDR, ct2i.buf, len);
            if(0 <= ct2i.x && ct2i.x <= 65535) {
                value = ct2i.x;
                LOG(INFO) << "EXPOSURE_VALUE: " <<  ct2i.x;
            }
        } else if(name.compare(SVC_TRIG_MODE) == 0) {
            memset(&ct2i, 0, sizeof(ct2i));
            cam0.RdEeprom(TRIG_MODEL_ADDR, ct2i.buf, len);
            if(TRIGMODE_AUTO == ct2i.x || TRIGMODE_FPGA == ct2i.x || TRIGMODE_SIGNAL == ct2i.x || TRIGMODE_SOFT == ct2i.x) {
                value = ct2i.x;
                LOG(INFO) << "TRIG_MODEL: " <<  ct2i.x;
            }
        } else if(name.compare(SVC_RESOLUTION) == 0) {
            memset(&ct2i, 0, sizeof(ct2i));
            cam0.RdEeprom(RESOLU_ADDR, ct2i.buf, len);
            if(RESOLU_752_480 == ct2i.x || RESOLU_640_480 == ct2i.x) {
                value = ct2i.x;
                LOG(INFO) << "RESOLU: " <<  ct2i.x;
            }
        } else {
            LOG(WARNING) << "!!!!!SVC200, int parameter error!";
        }
    }

    /*
      *     函数功能：设置camera的sn序列号
      *     形参列表：
      *         name：指定要设置的参数
      *         value：要设置的参数值
      *     返回值：无
    */
    virtual void setParameter(const std::string &name, const std::string value) {
        auto &s = sros::core::Settings::getInstance();
        s.setValue<std::string>(name, value);
        LOG(WARNING) << "!!!!!SVC200CameraDevice:" << name << "; SN: " << value;
    }

    /*
     *     函数功能：获取camera的sn序列号
     *     形参列表：
     *         name：指定要获取的参数
     *         value：获取到的参数值
     *     返回值：无
   */
    virtual void getParameter(const std::string &name, std::string &value) {
        list<DevBind_t>::iterator i;

        if(0 == devBindList.size())
            return;

        for (i = devBindList.begin(); i != devBindList.end(); ++i){
            if (SVC200_default_Id == (*i).deviceNo){
                value  = (*i).sn;
                LOG(WARNING) <<"!!!!!SrGetDevInfo svc sn: "<< value;
                return;
            }
        }
    }

    bool initCamera(int SVC200_Id){
        cam0.SelectSensor(sensor);
        int usbCnt = cam0.OpenUSB();
        LOG(INFO) << "CqUsbCam Count: " << usbCnt;

        if (usbCnt <= 0) {
            LOG(ERROR) << "CqUsbCam Exiting ...";
            return false;
        }
        cam0.ClaimInterface(SVC200_Id);
        return true;
    }

    int32_t SrReset(int32_t deviceNo) {
        list<DevBind_t>::iterator i;

        if(0 == devBindList.size())
            return -1;
        for (i = devBindList.begin(); i != devBindList.end(); ++i){
            if (deviceNo == (*i).deviceNo){
                return cam0.ResetDevice(deviceNo);
            }
        }
        return -1;
    }

    int32_t CheckSvcSN(CCqUsbCam &cam, uint8_t &model, string &e2promSN)
    {
        uint8_t sn[DEV_SN_LEN] = {0};
        uint32_t len = 0;
        int32_t ret = 0;

        // 读取设备型号
        len = DEV_MODEL_LEN;
        ret = cam.RdEeprom(DEV_MODEL_ADDR, &model, len);
        if(ret != len){
            ret = cam.RdEeprom(DEV_MODEL_ADDR, &model, len);
            if(ret != len){
                printf("error, RdEeprom DEV_MODEL_ADDR, ret=%d\n", ret);
            }
        }

        // 读取设备SN
        memset(sn, 0, DEV_SN_LEN);
        len = DEV_SN_LEN;
        ret  = cam.RdEeprom(DEV_SN_ADDR, sn, len);
        if(ret != len){
            ret  = cam.RdEeprom(DEV_SN_ADDR, sn, len);
            if(ret != len){
                printf("error, RdEeprom DEV_SN_ADDR, ret=%d\n", ret);
                return -2;
            }
        }

        string tmp(reinterpret_cast<char const*>(sn), DEV_SN_LEN);
        e2promSN = tmp;
        return 0;
    }

    int32_t FindCameraDevice(CCqUsbCam &cam, uint32_t &devNum, list<DevBind_t> &m_devBindList)
    {
        int32_t ret = 0;
        uint8_t model = 0;
        string sn;
        DevBind_t dev;

        devNum = CCqUsbCam::OpenUSB();
        if(devNum <= 0) {
            printf("svc device(s) no found!\n");
        }
        LOG(INFO)<<"!!!!!SVC200 Find USB device count:"<< devNum;

        for(uint8_t i =0; i<devNum; i++) {
            ret = cam.ClaimInterface(i);
            if(ret) {
                printf("error, cam.ClaimInterface=%d\n", ret);
                return -2;
            }

            ret = CheckSvcSN(cam, model, sn);
            if(ret) {
                printf("error, CheckSvcSN=%d\n", ret);
                return -3;
            }

            dev.deviceNo = i;
            dev.model = (enum Svc200Model)model;
            dev.sn = sn;
            m_devBindList.push_back(dev);
            cam.ReleaseInterface();
            LOG(INFO)<<"!!!!!SVC200 DevInfo svc No:"<< dev.deviceNo <<  ", devModel: " << dev.model <<", sn:"<< dev.sn<<endl;
        }

        return 0;
    }

    int32_t  SVC200_GetDevInfo(std::string  devModel, DevBind_t & m_devBind)
    {
        list<DevBind_t>::iterator i;
        if(0==devBindList.size()) {
            LOG(INFO) << "!!!!!svc200 select model failed";
            return -1;
        }
        LOG(INFO) << "!!!!!svc200 select BindList size" <<devBindList.size();

        std::string  sn;
        std::string  sn_name;
        enum Svc200Model  model;
        if(devModel.compare("svc200_back") == 0) {
            model = SVC200_S_LI_01;
            sn_name.assign("camera.back_serial_number");
            LOG(INFO) << "!!!!!svc200 select back model";
        } else if(devModel.compare("svc200_up") == 0) {
            model = SVC200_UD_LI_01;
            sn_name.assign("camera.up_serial_number");
            LOG(INFO) << "!!!!!svc200 select up model";
        } else if(devModel.compare("svc200_down") == 0) {
            model = SVC200_D_LO_01;
            sn_name.assign("camera.down_serial_number");
            LOG(INFO) << "!!!!!svc200 select down model";
        } else {
            LOG(INFO) << "!!!!!svc200 select failed, no the model type.";
            SVC200_default_Id = -1;
            return -2;
        }

        for (i = devBindList.begin(); i != devBindList.end(); ++i){
            if (model == (*i).model){
                m_devBind.deviceNo = (*i).deviceNo;
                m_devBind.model  = (*i).model;
                m_devBind.sn     = (*i).sn;
                LOG(INFO) <<"!!!!!SrGetDevInfo svc No:"<<m_devBind.deviceNo <<", sn:"<< m_devBind.sn;
                SVC200_default_Id = m_devBind.deviceNo;

                getParameter(sn_name, sn);
                setParameter(sn_name, sn);
                return 0;
            }
        }
        LOG(INFO) <<"!!!!!SrGetDevInfo  device no found";
        SVC200_default_Id = -1;
        return -1;
    }

    void handleSVC200Camera(void *frameData) {
        CImgFrame *m_frame = (CImgFrame *) frameData;
        cv::Mat frame(m_frame->m_width, m_frame->m_height, CV_8UC1, (unsigned char *) m_frame->m_imgBuf);
        cv::Mat image_data = frame.clone();

//        char   pic_name[24] = {0};
//        sprintf(pic_name,  "/sros/svc200_%d.jpg",  SVC200_default_Id);
//        cv::imwrite(pic_name,  image_data.clone());
//        LOG(INFO)<<"!!!!!SVC200 jpeg had  write!";
        //添加发送msg,但使用假的stamp
        ImgWithStampInfoPtr img_with_stamp(new ImgWithStampInfo);
        img_with_stamp->camera_name = getName();
        img_with_stamp->topic_name = "TOPIC_COLOR";
        img_with_stamp->stamp = sros::core::util::get_time_in_us();
        img_with_stamp->img = image_data;

        if(SVC200_read_flag){
            if(image_data.empty()){
                SrReset(SVC200_default_Id);
                LOG(INFO)<<"!!!!!SVC200 reset!";
                return;
            }
        }

        if(!image_data.empty()){
            keepAlive();
            if(SVC200_read_flag){
                if (sendMsg) {
                    sendMsg(img_with_stamp);
                }
            }
        } else{
            LOG(INFO)<<"test SVC200 image data is empty!";
            setStateOpenFailed();
            SVC200_open_flag = false;
        }
    }

 private:
    int32_t SrGetDevInfo(int32_t deviceNo,  DevBind_t  &devBind) {
        list<DevBind_t>::iterator  i;

        if(0 == devBindList.size())
            return -1;
        for (i = devBindList.begin(); i != devBindList.end(); ++i) {
            if (deviceNo == (*i).deviceNo){
                devBind.deviceNo = (*i).deviceNo;
                devBind.model  = (*i).model;
                devBind.sn     = (*i).sn;
                return 0;
            }
        }
        return -1;
    }

    uint8_t SrGetSvcTotalNum(void) {
        return TotalDeviceNum;
    }

    /**
     * 设置相机分辨率
     *
     * @param[in]  resolu：RESOLU_752_480，RESOLU_640_480
     *
     * @return   0成功，其他失败
     */
    int32_t SrSetResolu(uint8_t resolu) {
        switch(resolu){
            case RESOLU_752_480:
                m_pCamInUse->SetResolution(RESOLU_752_480);
                if(m_pCamInUse==&cam0){
                    g_width=752;
                    g_height=480;
                }
                break;
            case RESOLU_640_480:
                m_pCamInUse->SetResolution(RESOLU_640_480);
                if(m_pCamInUse==&cam0){
                    g_width=640;
                    g_height=480;
                }
                break;

            default:
                printf("error, svc200 have no resolu %d\n", resolu);
                break;
        }
        return 0;
    }

    /**
     * 设置图像镜像类型
     *
     * @param[in]  mirrorType：标准，MIRROR_NORMAL
     *                         X镜像，MIRROR_X
     *                         Y镜像，MIRROR_Y
     *                         XY镜像,MIRROR_XY
     *
     * @return   0成功，其他失败
     */
    int32_t SrSetMirrorType(const uint32_t mirrorType) {
        switch(mirrorType){
            case MIRROR_NORMAL:
            case MIRROR_X:
            case MIRROR_Y:
            case MIRROR_XY:
                m_pCamInUse->SetMirrorType(mirrorType);
                return 0;
            default:
                break;
        }
        return -2;
    }

    /**
     * 设置相机触发模式
     *
     * @param[in]  trigType:自动触发，TRIGMODE_AUTO；
     *                      FPGA触发，TRIGMODE_FPGA；
     *                      信号触发，TRIGMODE_SIGNAL；
     *                      软件触发，TRIGMODE_SOFT；
     *
     * @return   0成功，其他失败
     */
    int32_t SrSetTrigMode(const uint32_t trigType) {
        switch(trigType){
            case TRIGMODE_AUTO:
            case TRIGMODE_FPGA:
            case TRIGMODE_SIGNAL:
            case TRIGMODE_SOFT:
                LOG(INFO)<<"!!!!!SVC200 set trigger mode!";
                return m_pCamInUse->SetTrigMode(trigType);
            default:
                break;
        }
        return -2;
    }

    /**
     * 设置软件触发一次
     *
     * @param[in]  None
     *
     * @return   0成功，其他失败
     */
    int32_t SrSoftTrigOnce() {
        LOG(INFO)<<"!!!!!SVC200 set soft once trigger!";
        return m_pCamInUse->SoftTrigOnce();
    }

    /**
     * 设置FPGA触发频率
     *
     * @param[in]  freq:0~45
     *
     * @return   0成功，其他失败
     */
    int32_t SrSetFpgaTrigFreq(const uint32_t freq) {
        return m_pCamInUse->SetFpgaTrigFreq(freq);
    }

    /**
     * 设置增益、曝光模式
     *
     * @param[in]  autoGain:1,自动增益， 0,手动增益
     * @param[in]  autoExpo:1,自动曝光， 0,手动曝光
     *
     * @return   0成功，其他失败
     */
    int32_t SrSetAutoGainExpo(const bool autoGain, const bool autoExpo) {
        return m_pCamInUse->SetAutoGainExpo(autoGain, autoExpo);
    }

    /**
     * 手动设置曝光值
     *
     * @param[in]  expoVal:取值范围[0~65536]
     *
     * @return   0成功，其他失败
     */
    int32_t SrSetExpoValue(const uint32_t expoVal) {
        return m_pCamInUse->SetExpoValue(expoVal);
    }

    /**
     * 手动设置增益值
     *
     * @param[in]  gainVal:取值范围[0~64]
     *
     * @return   0成功，其他失败
     */
    int32_t SrSetGainValue(const uint32_t gainVal) {
        return m_pCamInUse->SetGainValue(gainVal);
    }

    /**
     * 设置补光灯pwm频率
     *
     * @param[in]  freq:100-50000Hz
     * @param[out]  None
     * 
     * @return   0成功，其他失败
     */
    int32_t SrSetLightPWMFreq(uint32_t freq) {
        uint32_t value=freq;
        uint32_t high=value>>8;
        uint32_t low=value&0xff;

        if(freq >= 100 && freq <= 50000){
            m_pCamInUse->WrFpgaReg(0x0c, high);
            m_pCamInUse->WrFpgaReg(0x0d, low);
            return 0;
        }
        return -2;
    }

    /**
     * 设置补光灯pwm占空比
     *
     * @param[in]  width:占空比0-100,通过0,100开关灯
     * @param[out]  None
     * 
     * @return   0成功，其他失败
     */
    int32_t SrSetLightPWMWidth(uint32_t width) {
        if(width>=0 && width<= 100){
            m_pCamInUse->WrFpgaReg(0x0e, width);
            return 0;
        }
        return -2;
    }

    int32_t SrRdE2prom(const uint32_t addr, uint8_t *buffer, uint32_t &length) {
        return cam0.RdEeprom(addr, buffer, length);
    }

    int32_t SrWrE2prom(const uint32_t addr, uint8_t *buffer, uint32_t length) {
        return cam0.WrEeprom(addr, buffer,length);
    }

    int32_t SrRdSensorReg(const uint32_t addr, uint32_t &value) {
        if(NULL == m_pCamInUse)
            return -1;
        return m_pCamInUse->RdSensorReg(addr, value);
    }

    int32_t SrRdFpgaReg(const uint32_t addr, uint32_t &value) {
        if(NULL == m_pCamInUse)
            return -1;
        return m_pCamInUse->RdFpgaReg(addr, value);
    }

    int32_t mutexDisp_lock(void) {
        return pthread_mutex_lock(&m_mutexDisp);
    }

    int32_t mutexDisp_unlock (void) {
        return pthread_mutex_unlock(&m_mutexDisp);
    }

    int32_t mutexCam_lock(void) {
        return pthread_mutex_lock(&m_mutexCam);
    }

    int32_t mutexCam_unlock (void) {
        return pthread_mutex_unlock(&m_mutexCam);
    }

    void checkspeed() {
        unsigned int speed = 0;
        cam0.GetUsbSpeed(speed);
        if (speed == LIBUSB_SPEED_SUPER) {
            LOG(INFO) << "USB 3.0 device found on cam0!";
            cam0.SendUsbSpeed2Fpga(LIBUSB_SPEED_SUPER);
        } else if (speed == LIBUSB_SPEED_HIGH) {
            LOG(INFO) << "USB 2.0 device found on cam0!";
            cam0.SendUsbSpeed2Fpga(LIBUSB_SPEED_HIGH);
        } else {
            LOG(INFO) << "Device speed unknown on cam0!";
        }
    }

    int SVC200_default_Id = 0;
    bool SVC200_open_flag = false;      //是否open camera的标志位 【 调用initCamera() 初始化相机 】
    bool SVC200_read_flag = false;      //是否启动camera capture的标志位【调用cam0.StartCap()启动capture 】
    bool enable_state_ = false;             //是否enable camera的标志位 【 调用enableCamera()使能相机 】
    bool initCq = false;                            //camera open之后、初次enable camera，再次initcamera()

    CCqUsbCam cam0;
    CCqUsbCam *m_pCamInUse;
    pthread_mutex_t m_mutexDisp;
    pthread_mutex_t m_mutexCam;

    std::string camera_name_;
    const std::string sensor = "MT9V034";
    unsigned int  g_width = 752;
    unsigned int  g_height = 480;
    unsigned int TotalDeviceNum = 0;
    //list<DevBind_t>  devBindList;
};
}
#endif //SDK_SVC200_H
