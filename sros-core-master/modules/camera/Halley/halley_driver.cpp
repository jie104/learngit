//
// Created by ehl on 2022/07/07.
//
#include "halley_driver.h"
#include <boost/bind.hpp>
#include <boost/make_shared.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/cstdint.hpp>
#include <boost/thread/thread.hpp>
#include <boost/thread/shared_mutex.hpp>
#include <opencv2/opencv.hpp>
#include <glog/logging.h>
#include "include/PS1080.h"
#include "include/OniEnums.h"

using namespace openni;

namespace astra_dev {

AstraDriver::AstraDriver(std::string driver_address)
    : device_uri_(driver_address),opened_(false),frameListener_(NULL) {
        LOG(INFO)<<"halley_driver new OBDevice AstraDriver Uri: " << driver_address;
        current_uri_ = driver_address;
        m_count_depth = 0;
        data_callback_func_ = NULL;
}

AstraDriver::~AstraDriver() {
    close();
}

bool AstraDriver::findSerialNumber(const std::string& serial_number) {
    if(serial_number != device_uri_) {
        LOG(ERROR) << "cannot found camera Uri: " << serial_number;
        return false;
    }
    return true;
}

void AstraDriver::waitForInitialized(){
    uint sleep_in_50_ms = 5e4;
    while (!initialized_state_) {
        usleep(sleep_in_50_ms);
        break;
    }
}

bool AstraDriver::init() {
    openni::Status rc;
    initialized_state_ = false;
    if (device_uri_.empty()) {
        LOG(ERROR) << "device_uri_ null, init error!!!";
    }
    LOG(INFO)<<"init device Uri: "<< device_uri_;
    openni_device_ = std::make_shared<openni::Device>();
    rc = openni_device_->open(device_uri_.c_str());
    if (rc != openni::STATUS_OK) {
        LOG(INFO)<<"!!!!!ERROR: [open camera failed] Scan " << device_uri_ << ". " << openni::OpenNI::getExtendedError();
        return false;
    }

    //设置Videmode
    // const Array<VideoMode>& videoModes = depth_->getSensorInfo().getSupportedVideoModes();
    // for (int i = 0; i < videoModes.getSize(); i++)
    // {
    //     VideoMode mode = videoModes[i];
    //     if(mode.getResolutionX() == 640 && mode.getResolutionY() == 480 && mode.getFps() == 30 && mode.getPixelFormat() == PIXEL_FORMAT_DEPTH_1_MM)
    //     {
    //         LOG(INFO) << "set: PIXEL_FORMAT_DEPTH_1_MM  video mode: 640 x 480 @30fps OK";
    //         rc = depth_->setVideoMode(mode);
    //         break;
    //     }
    // }

    OBCameraParamsData cameraParamData;
    int dataSize = sizeof(OBCameraParamsData);
    memset(&cameraParamData, 0, dataSize);
    //设置正确匹配的参数分辨率参数
    cameraParamData.depthRes = XN_CAMERA_PARAMS_DEPTH_RES_640_480;
    cameraParamData.colorRes = XN_CAMERA_PARAMS_COLOR_RES_DEFAULT;

    rc = openni_device_->getProperty(openni::OBEXTENSION_ID_CAM_PARAMS, (uint8_t *)&cameraParamData, &dataSize);
    if (rc != openni::STATUS_OK) {
        LOG(INFO)<<"!!!!!ERROR: [get intrinsic_params failed]  " << device_uri_ << ". " << openni::OpenNI::getExtendedError();
        return false;
    }

    OBCameraParams cameraParam;
    dataSize = sizeof(cameraParam);
    memset(&cameraParam, 0, sizeof(cameraParam));

    cameraParam = cameraParamData.params;

    xIntrinsic_Params IrParam;
    std::vector<float> irparam;
    IrParam.f_x = cameraParam.l_intr_p[0];
    irparam.emplace_back(cameraParam.l_intr_p[0]);
    IrParam.f_y = cameraParam.l_intr_p[1];
    irparam.emplace_back(cameraParam.l_intr_p[1]);
    IrParam.c_x = cameraParam.l_intr_p[2];
    irparam.emplace_back(cameraParam.l_intr_p[2]);
    IrParam.c_y = cameraParam.l_intr_p[3];
    irparam.emplace_back(cameraParam.l_intr_p[3]);
    sn_intrinsic_[device_uri_] = IrParam; 	//相机端口号和相机参数进行绑定
    output_sn_intrinsic_[device_uri_] = irparam;

    current_uri_ = device_uri_;
    current_fx_ = sn_intrinsic_[device_uri_].f_x;
    current_fy_ = sn_intrinsic_[device_uri_].f_y;
    current_cx_ = sn_intrinsic_[device_uri_].c_x;
    current_cy_ = sn_intrinsic_[device_uri_].c_y;
    LOG(INFO)<<"!!!!!param show:  current_uri_["  << current_uri_  << "] current_fx_["  << current_fx_  << "] current_fy_[" << current_fy_
                << "] current_cx_[" << current_cx_  << "] current_cy_[" << current_cy_;
    initialized_state_ = true;

    return true;
}

bool AstraDriver::openStream() {
    LOG(INFO)<<"halley_driver start stream";
    if(!initialized_state_ || !openni_device_) {
        LOG(INFO)<<"!!!!!ERROR: [halley init failed]. ";
        return false;
    }

    openni::Status rc;
    int n = current_uri_.length();
    char uri_array[n+1] = {0};
    strcpy(uri_array, current_uri_.c_str());

    depth_ = std::make_shared<openni::VideoStream>();
    if (openni_device_->getSensorInfo(SENSOR_DEPTH) != NULL) {
        rc = depth_->create(*openni_device_, openni::SENSOR_DEPTH);
        if(rc != openni::STATUS_OK) {
            LOG(INFO)<<"!!!!!ERROR: [Couldn't create depth stream] device serial_number: " << uri_array;
            //  openni::OpenNI::shutdown();
            return false;
        }
    }

    rc = depth_->start();
    if(rc != openni::STATUS_OK) {
        LOG(INFO)<<"!!!!!ERROR: [Couldn't start depth stream] deviceURI: " << uri_array;
        // openni::OpenNI::shutdown();
        return false;
    }

    if(!depth_->isValid()) {
        LOG(INFO)<<"!!!!!ERROR: [Depth stream is unvalid] deviceURI: " << uri_array;
        // openni::OpenNI::shutdown();
        return false;
    }

    opened_ = true;
    LOG(INFO)<<"OK: [create & start stream] deviceURI: " << uri_array;

    return true;
}

void AstraDriver::close() {
    if(depth_)
    {
        depth_->stop();
        depth_->destroy();
        LOG(INFO) << "depth_ stop and destroy done";
    }
    
	if (frameListener_ != NULL)
	{
		depth_->removeNewFrameListener(frameListener_);
		delete frameListener_;
		frameListener_ = NULL;
        LOG(INFO) << "frameListener_ removed";
	}

    if(openni_device_)
    {
        LOG(INFO) << "openni_device_ close " << current_uri_;
        openni_device_->close();
    }

    opened_ = false;
    initialized_state_ = false;

    LOG(INFO) << "closed end ################ " << current_uri_;
}

bool AstraDriver::isOpened() {
    return opened_;
}

bool AstraDriver::processFrameData(const VideoFrameRef& frame, int& frameWidth, int& frameHeight, StereoPoints& stereoPoints) {
    DepthPixel* pDepth = (DepthPixel*)frame.getData();
    if (NULL == pDepth) {
        LOG(ERROR) << "ERROR: [ depth frame is NULL] device uri: " << current_uri_.c_str();
        return false;
    }

    int width = frame.getWidth();
    int height = frame.getHeight();
    float world_x, world_y, world_z;

    float fdx = current_fx_;
    float fdy = current_fy_ ;
    float u0 = current_cx_ ;
    float v0 = current_cy_ ;
    
    stereoPoints.reserve(width*height);
    for (int v = 0; v < height; v++){
        for (int u = 0; u < width; u++){
            uint16_t disdepth = pDepth[v * width + u];
            if(disdepth < MIN_DISTANCE || disdepth > MAX_DISTANCE) {
                stereoPoints.push_back({-100.f,-100.f,-100.f,0});
            } else {
                world_x = disdepth * (u - u0) / fdx;
                world_y = disdepth * (v - v0) / fdy;
                world_z = disdepth;
                stereoPoints.push_back({-world_x/1000.f,world_y/1000.f,world_z/1000.f,0});
            }
        }
    }
    frameWidth = width;
    frameHeight = height;
    return true;
}

bool AstraDriver::getStereoData(int& frameWidth, int& frameHeight, StereoPoints& stereoPoints) {
    LOG(INFO) << "halley_driver test getStereoData";
    openni::Status rc;
    openni::VideoFrameRef frame;
    if(!opened_){
        LOG(INFO)<<"!!!!!ERROR: [Depth camera close status] device Uri: " << current_uri_.c_str();
        return false;
    }

    int changedStreamDummy;
    VideoStream* pStream = depth_.get();
    rc = OpenNI::waitForAnyStream(&pStream, 1, &changedStreamDummy, 500);
    if (rc != STATUS_OK) {
        LOG(INFO)<<"!!!!!ERROR: [waitForAnyStream] device Uri: " << current_uri_.c_str();
        return false;
    }

    //get depth frame
    rc = depth_->readFrame(&frame);
    if (rc != STATUS_OK) {
        LOG(INFO)<<"!!!!!ERROR: [readFrame] device Uri: " << current_uri_.c_str();
        return false;
    }

    //check if the frame format is depth frame format
    if (frame.getVideoMode().getPixelFormat() != PIXEL_FORMAT_DEPTH_1_MM && frame.getVideoMode().getPixelFormat() != PIXEL_FORMAT_DEPTH_100_UM) {
        LOG(ERROR) << "!!!!!ERROR: [unexpected frame format] device serial_number: " << current_uri_.c_str();
        return false;
    }

    //640*400  16UC1
    return processFrameData(frame, frameWidth, frameHeight, stereoPoints);
}

std::map<std::string,std::vector<float>> AstraDriver::getDevice_sn_intrinsic() {
    return output_sn_intrinsic_;
}

std::string AstraDriver::getOBDeviceUri(void) {
    return current_uri_;
}

void AstraDriver::setOBDeviceUri(const std::string &device_uri) {
    device_uri_ = device_uri;
    current_uri_ = device_uri;
}

void AstraDriver::getLDPStatus()
{
	openni::Status rc = openni::STATUS_OK;
	int dataSize = 4;
	int ldp_enable = 0;
	rc = openni_device_->getProperty(XN_MODULE_PROPERTY_LDP_STATUS, (uint8_t*)&ldp_enable, &dataSize);
	LOG(INFO) << " ldp status : " << ldp_enable;

}

void AstraDriver::getProperty()
{
	openni::Status rc = openni::STATUS_OK;
	int dataSize = 4;
	int laser_en = 0;
	rc = openni_device_->getProperty(XN_MODULE_PROPERTY_EMITTER_STATE_V1, (uint8_t*)&laser_en, &dataSize);
	LOG(INFO) << " laser status : " << laser_en;

}

bool AstraDriver::initFrameCallback()
{
    if(!opened_) {
        return false;
    }
    openni::Status rc = openni::STATUS_OK;
    if(NULL == frameListener_) {
	    frameListener_ = new FrameCallback(this);
        rc = depth_->addNewFrameListener(frameListener_);
    } else {
        LOG(WARNING) << "frameListener_  already inited";
    }
    if (openni::STATUS_OK == rc) {
        return true;
    } else {
        return false;
    }
}

void AstraDriver::analyzeFrame(const VideoFrameRef& frame)
{
    int width = 0;
    int height = 0;
    StereoPoints stereoPoints;
	switch (frame.getVideoMode().getPixelFormat())
	{
		case PIXEL_FORMAT_DEPTH_1_MM:
		case PIXEL_FORMAT_DEPTH_100_UM:
			// LOG(INFO) << (long long)frame.getTimestamp() << " deviceURI: " << openni_device_->getDeviceInfo().getUri();
            processFrameData(frame, width, height, stereoPoints);
            if(NULL != data_callback_func_) {
                data_callback_func_(width, height, stereoPoints);
            }
			break;
		default:
			LOG(INFO) << "Unknown format";
	}
}

bool AstraDriver::openDevice(const char* uri)
{
	openni::Status nRetVal = openni::OpenNI::initialize();
	if (nRetVal != openni::STATUS_OK)
	{
		return false;
	}

	/*Open the requested device.*/
	nRetVal = openni_device_->open(uri);
	if (nRetVal != openni::STATUS_OK)
	{
		LOG(INFO) << "Device: " << uri << ", open failed, try again open***********************************";
		nRetVal = openni_device_->open(uri);
		if (nRetVal != openni::STATUS_OK)
		{
			LOG(INFO) << "Device: " << uri << ", open failed2";
			return false;
		}
		//return false;
	}
	return startStream();
}


bool AstraDriver::startStream(){
	if (!openni_device_->isValid())
	{
		return false;
	}

	Status rc;
	if (openni_device_->getSensorInfo(SENSOR_DEPTH) != NULL)
	{
		rc = depth_->create(*openni_device_, SENSOR_DEPTH);
		if (rc != STATUS_OK)
		{
			LOG(INFO) << "Couldn't create depth stream" <<  OpenNI::getExtendedError();
			return false;
		}
	}

    // 此处设置 video mode 会崩 ?
	// const Array<VideoMode>& videmodes = openni_device_->getSensorInfo(SENSOR_DEPTH)->getSupportedVideoModes();
	// for (int i = 0; i < videmodes.getSize(); i++)
	// {
	// 	VideoMode videoMode = videmodes[i];
	// 	if (videoMode.getResolutionX() == 640 && videoMode.getResolutionY() == 480 && videoMode.getPixelFormat() == PIXEL_FORMAT_DEPTH_1_MM 
	// 			&& videoMode.getFps() == 30)
	// 	{
	// 		depthStream_.setVideoMode(videoMode);
	// 		break;
	// 	}
	// }

	rc = depth_->start();
	if (rc != STATUS_OK)
	{
		LOG(INFO) << "Couldn't start the depth stream" << OpenNI::getExtendedError();
		return false;
	}
	opened_ = true;

	frameListener_ = new FrameCallback(this);
	depth_->addNewFrameListener(frameListener_);

	return true;
}


bool AstraDriver::stopStream(){
	LOG(INFO) << "Stop stream...............start";
	opened_ = false;	
	if (frameListener_ != NULL)
	{
		depth_->removeNewFrameListener(frameListener_);
		delete frameListener_;
		frameListener_ = NULL;
	}

	depth_->stop();
	LOG(INFO) << "Stop stream...............end";

	return true;
}


void AstraDriver::closeDevice()
{
	LOG(INFO) << "Destroy stream...............start";
	depth_->destroy();
	LOG(INFO) << "Destroy stream...............end";
	openni_device_->close();
	LOG(INFO) << "Device close...............end";
}

}	//namespace astra_dev
