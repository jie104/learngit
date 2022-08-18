//
// Created by ehl on 2022/07/07.
//

#ifndef ASTRA_DRIVER_H__
#define ASTRA_DRIVER_H__

#include <stdio.h>
#include <math.h>
#include <iostream>
#include <string>
#include <vector>
#include <map>
#include <boost/function.hpp>
#include "include/OpenNI.h"
#include "../stereo_point.h"
#include "core/util/time.h"
#include <glog/logging.h>

#define MIN_DISTANCE  20
#define MAX_DISTANCE  3000
#define RESOULTION_X  640
#define RESOULTION_Y  480
#define SAMPLE_READ_WAIT_TIMEOUT 2000       //数据等待时间：2000ms

using namespace openni;

namespace astra_dev {

//定义相机内参数据类型
typedef struct xnIntrinsic_Params {
    xnIntrinsic_Params() :
        c_x(320.0), c_y(240.0), f_x(480.0), f_y(480.0)
    {}

    xnIntrinsic_Params(float c_x_, float c_y_, float f_x_, float f_y_) :
        c_x(c_x_), c_y(c_y_), f_x(f_x_),f_y(f_y_)
    {}

    float c_x;
    float c_y;
    float f_x;
    float f_y;
} xIntrinsic_Params;

typedef boost::function<void (size_t, size_t, const std::shared_ptr<StereoPoints> filter_points)>  callback_t;

using OBDeviceFrameCallback = std::function<void(int,  int, StereoPoints )>;


class AstraDriver {
 public:
    // use Uri to init device
    AstraDriver(std::string driver_address);
    ~AstraDriver();

    bool findSerialNumber(const std::string &number);

    void waitForInitialized();

    // init & open device and init camera params
    bool init();

    // open & start stream
    bool openStream();

    // stop stream, close device
    void close();

    // check if the stream is opened or started
    bool isOpened();
    // bool isStart();

    // get camera frame data, return StereoPoints with width and height
    // 对于 Orbbec 的相机, 此接口不安全!
    bool getStereoData(int& frameWidth, int& frameHeight, StereoPoints& stereoPoints);

    // get device intrinsic value
    std::map<std::string,std::vector<float>> getDevice_sn_intrinsic();

    // convert depth data to StereoPoints
    bool processFrameData(const VideoFrameRef& frame, int& frameWidth, int& frameHeight, StereoPoints& stereoPoints);

    std::string getOBDeviceUri(void);

    void setOBDeviceUri(const std::string &device_uri);

    void getLDPStatus();

    void getProperty();

    // init & start FrameCallback, keep listening new data coming
    bool initFrameCallback();

    // register data callback func
    void setFrameDataCallback(OBDeviceFrameCallback func) { data_callback_func_ = func; }

    // within camera callback, analyze camera frame data, return StereoPoints with width and height
    void analyzeFrame(const openni::VideoFrameRef& frame);

    // init & open OBDevice, optional API
    bool openDevice(const char* uri);

    // start stream, optional API
	bool startStream();

    // stop stream, optional API
	bool stopStream();

    // close device, optional API
	void closeDevice();
 public:
    int m_count_depth;

 private:
    std::shared_ptr<openni::Device> openni_device_;
    std::shared_ptr<openni::VideoStream> depth_;

	// openni::VideoStream depthStream_;
	openni::VideoStream::NewFrameListener* frameListener_;
    OBDeviceFrameCallback data_callback_func_;

    std::string device_uri_; // OBDevice Uri

    bool generate_stereopoints_ = false;
    bool initialized_state_ = false;
    bool opened_ = false;  // camera stream opened & started

    uint32_t devNum_;
    openni::Array<openni::DeviceInfo> deviceList_;
    std::map<std::string, xIntrinsic_Params> sn_intrinsic_;
    std::map<std::string, std::vector<float>> output_sn_intrinsic_;
    std::map<std::string, std::string> sn_uri_;
    std::string current_sn_ ;  // OBDevice serial number
    std::string current_uri_;
    float current_fx_;
    float current_fy_;
    float current_cx_;
    float current_cy_;
};

class FrameCallback : public VideoStream::NewFrameListener
{
	public:
		FrameCallback(AstraDriver* obDevice) :obDevice_(obDevice)
        {
        }

		void onNewFrame(VideoStream& stream)
		{
			if (nullptr != obDevice_ && obDevice_->isOpened())
			{
                stream.readFrame(&m_frame);
                obDevice_->analyzeFrame(m_frame);
			}
		}
	private:
		VideoFrameRef m_frame;
		AstraDriver* obDevice_;
};

}	//namespace astra_dev
#endif  //ASTRA_DRIVER_H__
