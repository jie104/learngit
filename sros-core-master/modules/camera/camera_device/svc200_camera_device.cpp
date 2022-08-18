//
// Created by cwt on 2021/11/22.
//

#include "svc200_camera_device.h"

namespace camera {

std::shared_ptr<SVC200ChipSelectSingleton >  SVC200ChipSelectSingleton::svc200_singleton_; 

std::shared_ptr<SVC200ChipSelectSingleton> SVC200ChipSelectSingleton::getInstance(){
	if (svc200_singleton_) {
		LOG(INFO) << "singleton ptr get.";
		return svc200_singleton_;
	}
	LOG(INFO) << "singleton reset.";
	svc200_singleton_.reset(new SVC200ChipSelectSingleton);
	return svc200_singleton_;
}

void SVC200ChipSelectSingleton::initializeAllDevices(){
	findCameraDevice(cam_);
}

bool SVC200ChipSelectSingleton::getCameraFDBySN(const std::string& camera_sn,int32_t& fd){
	int32_t num = getTotalNum();
	if(num <= 0) {
		LOG(INFO) << "ERROR: no camera exist." << num;
		fd = -1;
		return false;
	}
	LOG(INFO) << "svc200 number: " << num;

	auto curr_device_info_iter = dev_map_.find(camera_sn);
	if(curr_device_info_iter != dev_map_.end()){
		fd = curr_device_info_iter->second.devBind.deviceNo;
		return true;
	}
	LOG(INFO)<<"cannot find fd by camera sn!"<<camera_sn;
	return false;
}

bool SVC200ChipSelectSingleton::getCameraDevBindBySN(const std::string& camera_sn,DevBind_t& devBind){
	int32_t num = getTotalNum();
	if(num <= 0) {
		LOG(INFO) << "ERROR: no camera exist." << num;
		return false;

	}
	LOG(INFO) << "svc200 number: " << num;

	auto curr_device_info_iter = dev_map_.find(camera_sn);
	if(curr_device_info_iter != dev_map_.end()){
		devBind = curr_device_info_iter->second.devBind;
		return true;
	}
	LOG(INFO)<<"cannot find devBind by camera sn!"<<camera_sn;
	return false;
}

bool SVC200ChipSelectSingleton::getCameraParamBySN(const std::string& camera_sn,CameraParam_t& CamParam){
	int32_t num = getTotalNum();
	if(num <= 0) {
		LOG(INFO) << "ERROR: no camera exist." << num;
		return false;

	}
	LOG(INFO) << "svc200 number: " << num;

	auto curr_device_info_iter = dev_map_.find(camera_sn);
	if(curr_device_info_iter != dev_map_.end()){
		CamParam = curr_device_info_iter->second.cameraParam;
		return true;
	}
	LOG(INFO)<<"cannot find CamParam by camera sn!"<<camera_sn;
	return false;
}

int32_t SVC200ChipSelectSingleton::findCameraDevice(CCqUsbCam &cam)
{
	int32_t ret = 0;
	uint8_t model = 0;
	string sn;
	DevBind_t dev;
	CameraParam_t cameraParam;
	DeviceInfo_t  device_info;

	cam.SelectSensor(sensor);
	device_num_ = cam.OpenUSB();
	if(device_num_ <= 0) {
		LOG(INFO) << "ERROR: SVC device no found." << device_num_;
		return -1;
	}
	LOG(INFO)<<"!!!!!SVC200 Find device count:"<< device_num_;

	for(int8_t i =0; i<device_num_; i++) {
		ret = cam.ClaimInterface(i);
		if(ret) {
			LOG(INFO)<< "ERROR: cam.ClaimInterface-" << ret;
			return -2;
		}

		ret = checkSvcSN(cam, model, sn);
		if(ret) {
			cam.ReleaseInterface();
			LOG(INFO)<< "ERROR: CheckSvcSN-" << ret;
			return -3;
		}

		dev.deviceNo = i;
		dev.model = (enum Svc200Model)model;
		dev.sn = sn;

		getCameraConfigIntrinsicParam(cam, cameraParam);
		device_info.devBind = dev;
		device_info.cameraParam = cameraParam;
		dev_map_[sn] = device_info;

		cam.ReleaseInterface();
		LOG(INFO)<<"!!!!!SVC200 DevInfo No:"<< dev.deviceNo <<  ", devModel: " << dev.model <<", sn:"<< dev.sn;
	}

	return 0;
}

int32_t SVC200ChipSelectSingleton::checkSvcSN(CCqUsbCam &cam, uint8_t &model, string &e2promSN)
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

int32_t SVC200ChipSelectSingleton::getTotalNum(void) {
	return device_num_;
}

void  SVC200ChipSelectSingleton::resetAllDevice(void) 
{
	for(auto curr_iter = dev_map_.begin();  curr_iter != dev_map_.end();  curr_iter++) {
		cam_.ResetDevice(curr_iter->second.devBind.deviceNo);
	}
}

CCqUsbCam* SVC200ChipSelectSingleton::getUsbCam()
{
	//int32_t num = getTotalNum();
	//if(num <= 0) {
	//	LOG(INFO) << "ERROR: no camera exist." << num;
	//	return NULL;
	//}
	//LOG(INFO) << "svc200 number: " << num;

	return  &cam_;
}

/*
	*     函数功能：获取相机版本号、分辨率、触发模式、曝光模式、曝光值、增益模式、增益值、PWM频率、PWM占空比；
	*							 获取相机的内参、畸变参数、标定中心参数
	*     形参列表：
	*         cam：相机设备指针（input）
	*         cameraParam: 相机配置参数、内参存储变量（output）
	*     返回值：无
*/
void SVC200ChipSelectSingleton::getCameraConfigIntrinsicParam(CCqUsbCam& cam, CameraParam_t& cameraParam) {
	int  ret = 0;
	uint16_t i = 0;
	uint32_t len = 8;
	uint32_t addr = 0;
	union ConvertType2i ct2i;
	union ConvertType2d ct2d;
	uint8_t sn[DEV_SN_LEN] = {0};

	// 获取相机软件版本号
	len = FIRMWARE_VERSION_LEN;
	memset(&ct2i, 0, sizeof(ct2i));
	ret = cam.RdEeprom(FIRMWARE_VERSION_ADDR, ct2i.buf, len);
	LOG(INFO) << "FIRMWARE_VERSION_ADDR: " << FIRMWARE_VERSION_ADDR << "; value: " << ct2i.x;

    memset(sn, 0, DEV_SN_LEN);
    len = DEV_SN_LEN;
    ret  = cam.RdEeprom(DEV_SN_ADDR, sn, len);
	LOG(INFO) << "Camera device SN: " << sn;

	/****************相机配置参数******************/
	// 获取相机分辨率
	len = RESOLU_LEN;
	memset(&ct2i, 0, sizeof(ct2i));
	ret = cam.RdEeprom(RESOLU_ADDR, ct2i.buf, len);
	if(RESOLU_752_480 == ct2i.x || RESOLU_640_480 == ct2i.x){
		cameraParam.config.Resolu = ct2i.x;
		LOG(INFO) << "RESOLU_ADDR: " << RESOLU_ADDR << "; value: " << ct2i.x;
	} else {
		cameraParam.config.Resolu = RESOLU_752_480;
		LOG(INFO) << "ERROR: Can't match the existing resolu.";
	}

	// 获取相机触发模式
	len = TRIG_MODEL_LEN;
	memset(&ct2i, 0, sizeof(ct2i));
	ret = cam.RdEeprom(TRIG_MODEL_ADDR, ct2i.buf, len);
	if(TRIGMODE_AUTO == ct2i.x || TRIGMODE_FPGA == ct2i.x || TRIGMODE_SIGNAL == ct2i.x || TRIGMODE_SOFT == ct2i.x){
		cameraParam.config.trigMode = ct2i.x;
		LOG(INFO) << "TRIG_MODEL_ADDR: " << TRIG_MODEL_ADDR << "; value: " << ct2i.x;
	} else {
		cameraParam.config.trigMode = TRIGMODE_SOFT;
		LOG(INFO) << "ERROR: Can't match the existing trigger mode.";
	}

	// 获取相机曝光模式
	len = EXPO_MODEL_LEN;
	memset(&ct2i, 0, sizeof(ct2i));
	ret = cam.RdEeprom(EXPO_MODEL_ADDR, ct2i.buf, len);
	if(0 == ct2i.x || 1 == ct2i.x ){
		cameraParam.config.expoModel = ct2i.x;
		LOG(INFO) << "EXPO_MODEL_ADDR: " << EXPO_MODEL_ADDR << "; value: " << ct2i.x;
	} else {
		cameraParam.config.expoModel = TRIGMODE_AUTO;
		LOG(INFO) << "ERROR: Can't match the existing expo mode.";
	}

	// 获取相机曝光值
	len = EXPO_VALUE_LEN;
	memset(&ct2i, 0, sizeof(ct2i));
	ret = cam.RdEeprom(EXPO_VALUE_ADDR, ct2i.buf, len);
	if(ct2i.x >= 0 && ct2i.x <= 65535){
		cameraParam.config.expoValue = ct2i.x;
		LOG(INFO) << "EXPO_VALUE_ADDR: " << EXPO_VALUE_ADDR << "; value: " << ct2i.x;
	} else {
		cameraParam.config.expoValue = 0;
		LOG(INFO) << "ERROR: Value is out of range[0 - 65535].";
	}

	// 获取相机增益模式
	len = GAIN_MODEL_LEN;
	memset(&ct2i, 0, sizeof(ct2i));
	ret = cam.RdEeprom(GAIN_MODEL_ADDR, ct2i.buf, len);
	if(0 == ct2i.x || 1 == ct2i.x ){
		cameraParam.config.gainModel = ct2i.x;
		LOG(INFO) << "GAIN_MODEL_ADDR: " << GAIN_MODEL_ADDR << "; value: " << ct2i.x;
	} else {
		cameraParam.config.gainModel = 1; // 自动增益
		LOG(INFO) << "ERROR: Can't match the existing gain mode.";
	}

	// 获取相机增益值
	len = GAIN_VALUE_LEN;
	memset(&ct2i, 0, sizeof(ct2i));
	ret = cam.RdEeprom(GAIN_VALUE_ADDR, ct2i.buf, len);
	if(ct2i.x >= 0 && ct2i.x <= 64){
		cameraParam.config.gainValue = ct2i.x;
		LOG(INFO) << "GAIN_VALUE_ADDR: " << GAIN_VALUE_ADDR << "; value: " << ct2i.x;
	} else {
		cameraParam.config.gainValue = 0;
		LOG(INFO) << "ERROR: Value is out of range[0 - 64].";
	}

	// 获取补光灯pwm频率Hz
	len = LIGHT_PWM_FREQ_LEN;
	memset(&ct2i, 0, sizeof(ct2i));
	ret = cam.RdEeprom(LIGHT_PWM_FREQ_ADDR, ct2i.buf, len);
	if(ct2i.x >= 100 && ct2i.x <= 50000){
		cameraParam.config.lightPWMFreq = ct2i.x;
		LOG(INFO) << "LIGHT_PWM_FREQ_ADDR: " << LIGHT_PWM_FREQ_ADDR << "; value: " << ct2i.x;
	} else {
		cameraParam.config.lightPWMFreq = 50000;
		LOG(INFO) << "ERROR: Value is out of range[100 - 50000].";
	}

	// 获取补光灯pwm占空比
	len = LIGHT_PWM_WIDTH_LEN;
	memset(&ct2i, 0, sizeof(ct2i));
	ret = cam.RdEeprom(LIGHT_PWM_WIDTH_ADDR, ct2i.buf, len);
	if(ct2i.x >= 0 && ct2i.x <= 100){
		cameraParam.config.lightPWMWidth = ct2i.x;
		LOG(INFO) << "LIGHT_PWM_WIDTH_ADDR: " << LIGHT_PWM_WIDTH_ADDR << "; value: " << ct2i.x;
	} else {
		cameraParam.config.lightPWMWidth = 100;
		LOG(INFO) << "ERROR: Value is out of range[0 - 100].";
	}

	/**************** 视觉算法相机标定参数 ******************/  
	// 获取相机内参1-9
	len = VALGORITHM_INTRINSIC1_LEN;
	addr = VALGORITHM_INTRINSIC1_ADDR;
	for(i=0; i<9; i++){
		memset(&ct2d, 0, sizeof(ct2d));
		ret = cam.RdEeprom(addr, ct2d.buf, len);
		if (ret == len){
			cameraParam.vAlgorithm.intrinsic[i] = ct2d.x;
			LOG(INFO)  << "!!!!!intrinsic_addr:" << addr << ", value:" << ct2d.x;
		} else {
			ret = cam.RdEeprom(addr, ct2d.buf, len);
			if(ret != len){
				LOG(INFO)  << "!!!!!ERROR: intrinsic_addr:" << addr << ", value:" << ct2d.x;
			}
		}
		addr  = E2PROM_PAGE_OFFSET*(12+i);
	}

	// 获取相机畸变参数1-5
	len = VALGORITHM_DISTORT1_LEN;
	addr = VALGORITHM_DISTORT1_ADDR;
	for(i=0; i<5; i++){
		memset(&ct2d, 0, sizeof(ct2d));
		ret = cam.RdEeprom(addr, ct2d.buf, len);
		if (ret == len){
			cameraParam.vAlgorithm.distort[i] = ct2d.x;
			LOG(INFO)  << "!!!!!VALGORITHM_addr:" << addr << ", value:" << ct2d.x;
		} else {
			ret = cam.RdEeprom(addr, ct2d.buf, len);
			if(ret != len){
				LOG(INFO)  << "!!!!!ERROR: VALGORITHM_addr:" << addr << ", value:" << ct2d.x;
			}
		}
		addr  = E2PROM_PAGE_OFFSET*(21+i);
	}

	// 获取相机标定中心点参数1-2
	len = VALGORITHM_VANISHING_POINT1_LEN;
	addr = VALGORITHM_VANISHING_POINT1_ADDR;
	for(i=0; i<2; i++){
		memset(&ct2d, 0, sizeof(ct2d));
		ret = cam.RdEeprom(addr, ct2d.buf, len);
		if (ret == len){
			cameraParam.vAlgorithm.vanishingPoint[i] = ct2d.x;
			LOG(INFO)  << "!!!!!VALGORITHM_addr:" << addr << ", value:" << ct2d.x;
		} else {
			ret = cam.RdEeprom(addr, ct2d.buf, len);
			if(ret != len){
				LOG(INFO)  << "!!!!!ERROR: VALGORITHM_addr:" << addr << ", value:" << ct2d.x;
			}
		}
		addr  = E2PROM_PAGE_OFFSET*(21+i);
	}
}

/*USB相机重启之后，对应的文件描述符会不会改变，需要实际验证一下*/

/*  <--SVC200ChipSelectSingleton ****************************** SVC200CameraDevice--> */


SVC200CameraDevice::SVC200CameraDevice(std::string camera_sn, ImgMsgCallback callback, const std::string &name, sros::device::DeviceID device_id)
	: camera_sn_(camera_sn),
		BaseCameraDevice(callback, name, device_id, sros::device::DEVICE_COMM_INTERFACE_TYPE_USB_0, sros::device::DEVICE_MOUNT_SROS) {
	
	
	svc200_singleton_ = SVC200ChipSelectSingleton::getInstance();
	cam_usb_ = svc200_singleton_->getUsbCam();
	if(svc200_singleton_->getCameraFDBySN(camera_sn_, camera_fd_)){
		if(camera_fd_ >= 0) {
			//open();		//测试使用，正式版需要删除
			LOG(INFO) << "ready use svc200 camera, the fd is " << camera_fd_;
		} else {
			LOG(WARNING) << "error: ready use svc200 camera, the fd is " << camera_fd_;
		}
	}else{
		LOG(INFO) << "error: get use svc200_singleton fd failed.  " << camera_sn_;
	}

	boost::thread(boost::bind(&SVC200CameraDevice::scanSvc200State, this));		//热插拔
}

SVC200CameraDevice::~SVC200CameraDevice(){
	close();
}

 bool SVC200CameraDevice::open() {
	if(!cam_usb_){
		return false;
	}
	if(open_flag_) {
		return true;
	}
	open_flag_ = false;
	setStateInitialization();
	const int try_times_thresh = 1;
	int curr_time = 0;

	while (curr_time++ < try_times_thresh) {
		open_flag_ = initCamera();
		if (open_flag_) {
			LOG(INFO) << "open SVC200 Successful";
			setStateOK();
			break;
		} else {
			LOG(INFO) << "cannot open ! try again!";
		}
	}
	if (!isOpened()) {
		LOG(ERROR)<<"open SVC200 Failure: cannot open SVC200 "<<camera_fd_;
		setStateOpenFailed();
		return false;
	}

	int read_result = cam_usb_->StartCap(g_height, g_width,  std::bind(&SVC200CameraDevice::handleSVC200Camera, this, std::placeholders::_1));
	LOG(INFO)<<"open read_result : "<<read_result;
	if(read_result != 0) {
		read_flag_ = false;
		LOG(ERROR) << "open SVC200 Failure: SVC200 readCap failure";
		setStateOpenFailed();
		return false;
	}
	read_flag_ = true;
	LOG(INFO) << "!!!!!open SVC200 Successful!";
	return true;
}

bool SVC200CameraDevice::isOpened() {
	return open_flag_;
}

void SVC200CameraDevice::close() {
	if(!cam_usb_){
		return;
	}

	open_flag_ = false;
	if(this->read_flag_){
		cam_usb_->StopCap();
		read_flag_ = false;
	}

	cam_usb_->ReleaseInterface();
	cam_usb_->CloseUSB();
	LOG(INFO) << "!!!!!close SVC200!";
}

bool SVC200CameraDevice::enableCamera() {
	if(!cam_usb_){
		return false;
	}
	if(open_flag_) {
		return true;
	}
	
	if (!open_flag_) {
		setStateInitialization();
		open_flag_ = initCamera();
		if (!open_flag_) {
			LOG(ERROR)<<"!!!!!enable SVC200 Failure: cannot open SVC200 "<< camera_fd_;
			return false;
		}
		setStateOK();
	}

	if(read_flag_) {
		cam_usb_->StopCap();
	}

	enable_state_ = true;
	int read_result = cam_usb_->StartCap(g_height, g_width,  std::bind(&SVC200CameraDevice::handleSVC200Camera, this, std::placeholders::_1));
	LOG(INFO)<<"!!!!!cam_usb_ StartCap read_result : "<< read_result;
	if(read_result != 0){
		read_flag_ = false;
		LOG(ERROR) << "!!!!!cam_usb_ StartCap Failure: SVC200 readCap failure";
		return false;
	}
	read_flag_ = true;
	LOG(INFO) << "!!!!!enable SVC200 Successful!";

	return true;
}

bool SVC200CameraDevice::disableCamera() {
	if(!cam_usb_){
		return false;
	}
	
	if(this->read_flag_){
		cam_usb_->StopCap();
		read_flag_ = false;
	}
	enable_state_ = false;
	LOG(INFO) << "!!!!!disable SVC200!";
	setStateOff();
	return true;
}

bool SVC200CameraDevice::isEnabled(){
	return enable_state_;
};

bool SVC200CameraDevice::reset() {
	if(!cam_usb_){
		return false;
	}
	return cam_usb_->ResetDevice(camera_fd_);
}

/*
	*     函数功能：获取相机的电流/电压 ADC值; 
	*     形参列表：
	*         name：指定获取哪个参数的数据
	*         value：获取到的数值
	*     返回值：无
	*     参数列表：
	*         SVC200_CURRENT  -- SVC200_Current 
	*         SVC200_VOLTAGE -- SVC200_Voltage
*/
void SVC200CameraDevice::getParameter(const std::string &name, double &value) {
	uint16_t ad = 0;

	/*  从相机中读取电压/电流值  */
	if(name.compare(SVC_CURRENT) == 0) {
		ad = readAdc(cam_usb_, 0);
		value = ad/10;        // mA
	} else if(name.compare(SVC_VOLTAGE) == 0) {
		ad = readAdc(cam_usb_, 1);
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
void SVC200CameraDevice::setParameter(const std::string &name, const int value) {
	int ret = 0;
	if(name.compare(SVC_PWM_FREQ) == 0) {
		ret = svc200SetLightPWMFreq(value);
		if(ret)
			LOG(WARNING) << "!!!!!SVC200, set pwm freq failed!";
	} else if(name.compare(SVC_PWM_WIDTH) == 0) {
		ret = svc200SetLightPWMWidth(value);
		if(ret)
			LOG(WARNING) << "!!!!!SVC200, set pwm width failed!";
	} else if(name.compare(SVC_AUTO_GAIN_EXPO)  == 0) {
		svc200SetAutoGainExpo(1, 1);
	} else if(name.compare(SVC_VALUE_GAIN) == 0) {
		svc200SetAutoGainExpo(0, 1);
		svc200SetGainValue(value);
	} else if(name.compare(SVC_VALUE_EXPOSURE) == 0) {
		svc200SetAutoGainExpo(1, 0);
		svc200SetExpoValue(value);
	} else if(name.compare(SVC_TRIG_MODE) == 0) {
		svc200SetTrigMode(value);
	} else if(name.compare(SVC_TRIG_FPGAFREQ) == 0) {
		svc200SetFpgaTrigFreq(value);
	} else if(name.compare(SVC_TRIG_SOFTONCE) == 0) {
		svc200SoftTrigOnce();
	} else if(name.compare(SVC_MIRROR_TYPE) == 0) {
		svc200SetMirrorType(value);
	} else if(name.compare(SVC_RESOLUTION) == 0) {
		svc200SetResolu(value);
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
void SVC200CameraDevice::getParameter(const std::string &name, int &value) {
	union ConvertType2i ct2i;
	uint32_t addr = 0;
	uint32_t len = 4;

	if(name.compare(SVC_FIRMWARE_VERSION) == 0) {
		memset(&ct2i, 0, sizeof(ct2i));
		cam_usb_->RdEeprom(FIRMWARE_VERSION_ADDR, ct2i.buf, len);
		value = ct2i.x;
		LOG(INFO) << "!!!!!FIRMWARE_VERSION: " <<  ct2i.x;
	} else if(name.compare(SVC_PWM_FREQ) == 0) {
		memset(&ct2i, 0, sizeof(ct2i));
		cam_usb_->RdEeprom(LIGHT_PWM_FREQ_ADDR, ct2i.buf, len);
		if(100 <= ct2i.x && ct2i.x <= 50000) {
			value = ct2i.x;
			LOG(INFO) << "PWM_FREQ: " <<  ct2i.x;
		}
	} else if(name.compare(SVC_PWM_WIDTH) == 0) {
		memset(&ct2i, 0, sizeof(ct2i));
		cam_usb_->RdEeprom(LIGHT_PWM_WIDTH_ADDR, ct2i.buf, len);
		if(0 <= ct2i.x && ct2i.x <= 100) {
			value = ct2i.x;
			LOG(INFO) << "PWM_WIDTH: " <<  ct2i.x;
		}
	} else if(name.compare(SVC_VALUE_GAIN) == 0) {
		memset(&ct2i, 0, sizeof(ct2i));
		cam_usb_->RdEeprom(GAIN_VALUE_ADDR, ct2i.buf, len);
		if(0 <= ct2i.x && ct2i.x <= 64) {
			value = ct2i.x;
			LOG(INFO) << "GAIN_VALUE: " <<  ct2i.x;
		}
	} else if(name.compare(SVC_VALUE_EXPOSURE) == 0) {
		memset(&ct2i, 0, sizeof(ct2i));
		cam_usb_->RdEeprom(EXPO_VALUE_ADDR, ct2i.buf, len);
		if(0 <= ct2i.x && ct2i.x <= 65535) {
			value = ct2i.x;
			LOG(INFO) << "EXPOSURE_VALUE: " <<  ct2i.x;
		}
	} else if(name.compare(SVC_TRIG_MODE) == 0) {
		memset(&ct2i, 0, sizeof(ct2i));
		cam_usb_->RdEeprom(TRIG_MODEL_ADDR, ct2i.buf, len);
		if(TRIGMODE_AUTO == ct2i.x || TRIGMODE_FPGA == ct2i.x || TRIGMODE_SIGNAL == ct2i.x || TRIGMODE_SOFT == ct2i.x) {
			value = ct2i.x;
			LOG(INFO) << "TRIG_MODEL: " <<  ct2i.x;
		}
	} else if(name.compare(SVC_RESOLUTION) == 0) {
		memset(&ct2i, 0, sizeof(ct2i));
		cam_usb_->RdEeprom(RESOLU_ADDR, ct2i.buf, len);
		if(RESOLU_752_480 == ct2i.x || RESOLU_640_480 == ct2i.x) {
			value = ct2i.x;
			LOG(INFO) << "RESOLU: " <<  ct2i.x;
		}
	} else {
		LOG(WARNING) << "!!!!!SVC200, int parameter error!";
	}
}

bool SVC200CameraDevice::initCamera(){
	LOG(INFO) << "svc200 ClaimInterface  camera_fd_:" << camera_fd_;
	if(camera_fd_ < 0) {
		return false;
	}
	try {
		int ret = cam_usb_->ClaimInterface(camera_fd_);
		if(ret < 0) {
			LOG(INFO) << "ERROR: SVC200 claim failed! the camera errorCode is " << ret;
			return false;
		}
	} catch (std::system_error &e) {
		LOG(ERROR) << e.what();
		return false;
	} catch (...) {
		LOG(ERROR) << "unkown exception!";
		return false;
	}
	return true;
}

void SVC200CameraDevice::handleSVC200Camera(void *frameData) {
	CImgFrame *m_frame = (CImgFrame *) frameData;
	cv::Mat frame(m_frame->m_width, m_frame->m_height, CV_8UC1, (unsigned char *) m_frame->m_imgBuf);
	cv::Mat image_data = frame.clone();

    //char   pic_name[24] = {0};
    //sprintf(pic_name,  "/sros/svc200_%d.jpg",  camera_fd_);
    // cv::imwrite(pic_name,  image_data.clone());
//    LOG(INFO)<<"!!!!!SVC200 jpeg had  write!";
	//添加发送msg,但使用假的stamp
	ImgWithStampInfoPtr img_with_stamp(new ImgWithStampInfo);
	img_with_stamp->camera_name = getName();
	img_with_stamp->topic_name = "TOPIC_COLOR";
	img_with_stamp->stamp = sros::core::util::get_time_in_us();
	img_with_stamp->img = image_data;
	is_running_ = true;

	if(read_flag_){
		if(image_data.empty()){
			reset();
			LOG(INFO)<<"!!!!!SVC200 reset!";
			return;
		}
	}

	if(!image_data.empty()){
		keepAlive();
		if(read_flag_){
			if (sendMsg) {
				sendMsg(img_with_stamp);
			}
		}
	} else{
		LOG(INFO)<<"test SVC200 image data is empty!";
		setStateOpenFailed();
		open_flag_ = false;
	}
}

void SVC200CameraDevice::scanSvc200State()
{
	sleep(5);
	while(true) {
		if(is_running_) {
			usleep(3000000);
			is_running_ = false;
		} else {
			LOG(INFO)<<"!!!!!SVC200 detech...reset...open...";
			usleep(500000);
			int device_num_ = cam_usb_->OpenUSB();
			if(device_num_ <= 0) {		//相机数量可作为Matrix配置参数，掉线时、减少findCameraDevice调用频率
				LOG(INFO) << "WARNING: scan svc200 device, device number: " << device_num_;
				setStateOpenFailed();
				continue;
			}
			LOG(INFO) << "scan svc200 device, device number: " << device_num_;

			if(isOpened()) {
				close();
				cam_usb_->ResetFlag();
				usleep(200000);
			}
			
			svc200_singleton_->findCameraDevice(*cam_usb_);
			if(svc200_singleton_->getCameraFDBySN(camera_sn_, camera_fd_)){
				if(camera_fd_ >= 0) {
					cam_usb_ = svc200_singleton_->getUsbCam();
					LOG(INFO) << "ready use svc200 camera, the fd is " << camera_fd_;
				} else {
					LOG(WARNING) << "camera fd error, the fd is " << camera_fd_;
					setStateOpenFailed();
					continue;
				}
			}else{
				LOG(INFO) << "get svc200 fd failed.  " << camera_sn_;
				setStateOpenFailed();
				continue;
			}

			open();
			is_running_ = true;
		}
		usleep(500000);
	}
}

/**
 * 设置相机分辨率
 *
 * @param[in]  resolu：RESOLU_752_480，RESOLU_640_480
 *
 * @return   0成功，其他失败
 */
int32_t SVC200CameraDevice::svc200SetResolu(uint8_t resolu) {
	switch(resolu){
		case RESOLU_752_480:
			cam_usb_->SetResolution(RESOLU_752_480);
			g_width=752;
			g_height=480;
			break;
		case RESOLU_640_480:
			cam_usb_->SetResolution(RESOLU_640_480);
			g_width=640;
			g_height=480;
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
int32_t SVC200CameraDevice::svc200SetMirrorType(const uint32_t mirrorType) {
	switch(mirrorType){
		case MIRROR_NORMAL:
		case MIRROR_X:
		case MIRROR_Y:
		case MIRROR_XY:
			cam_usb_->SetMirrorType(mirrorType);
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
int32_t SVC200CameraDevice::svc200SetTrigMode(const uint32_t trigType) {
	switch(trigType){
		case TRIGMODE_AUTO:
		case TRIGMODE_FPGA:
		case TRIGMODE_SIGNAL:
		case TRIGMODE_SOFT:
			LOG(INFO)<<"!!!!!SVC200 set trigger mode!";
			return cam_usb_->SetTrigMode(trigType);
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
int32_t SVC200CameraDevice::svc200SoftTrigOnce() {
	LOG(INFO)<<"!!!!!SVC200 set soft once trigger!";
	return cam_usb_->SoftTrigOnce();
}

/**
 * 设置FPGA触发频率
 *
 * @param[in]  freq:0~45
 *
 * @return   0成功，其他失败
 */
int32_t SVC200CameraDevice::svc200SetFpgaTrigFreq(const uint32_t freq) {
	return cam_usb_->SetFpgaTrigFreq(freq);
}

/**
 * 设置增益、曝光模式
 *
 * @param[in]  autoGain:1,自动增益， 0,手动增益
 * @param[in]  autoExpo:1,自动曝光， 0,手动曝光
 *
 * @return   0成功，其他失败
 */
int32_t SVC200CameraDevice::svc200SetAutoGainExpo(const bool autoGain, const bool autoExpo) {
	return cam_usb_->SetAutoGainExpo(autoGain, autoExpo);
}

/**
 * 手动设置曝光值
 *
 * @param[in]  expoVal:取值范围[0~65536]
 *
 * @return   0成功，其他失败
 */
int32_t SVC200CameraDevice::svc200SetExpoValue(const uint32_t expoVal) {
	return cam_usb_->SetExpoValue(expoVal);
}

/**
 * 手动设置增益值
 *
 * @param[in]  gainVal:取值范围[0~64]
 *
 * @return   0成功，其他失败
 */
int32_t SVC200CameraDevice::svc200SetGainValue(const uint32_t gainVal) {
	return cam_usb_->SetGainValue(gainVal);
}

/**
 * 设置补光灯pwm频率
 *
 * @param[in]  freq:100-50000Hz
 * @param[out]  None
 * 
 * @return   0成功，其他失败
 */
int32_t SVC200CameraDevice::svc200SetLightPWMFreq(uint32_t freq) {
	uint32_t value=freq;
	uint32_t high=value>>8;
	uint32_t low=value&0xff;

	if(freq >= 100 && freq <= 50000){
		cam_usb_->WrFpgaReg(0x0c, high);
		cam_usb_->WrFpgaReg(0x0d, low);
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
int32_t SVC200CameraDevice::svc200SetLightPWMWidth(uint32_t width) {
	if(width>=0 && width<= 100){
		cam_usb_->WrFpgaReg(0x0e, width);
		return 0;
	}
	return -2;
}

uint16_t SVC200CameraDevice::readAdc(CCqUsbCam* usbCam, uint8_t ch)
{
	uint8_t rxval[2];
	uint8_t  chData[64];

	switch (ch) {
		case 0:
			ch = 0x88;  break;
		case 1:
			ch = 0xc8;  break;
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
	return irxval;
}

void SVC200CameraDevice::checkSpeed() {
	unsigned int speed = 0;
	cam_usb_->GetUsbSpeed(speed);
	if (speed == LIBUSB_SPEED_SUPER) {
		LOG(INFO) << "USB 3.0 device found on cam!";
		cam_usb_->SendUsbSpeed2Fpga(LIBUSB_SPEED_SUPER);
	} else if (speed == LIBUSB_SPEED_HIGH) {
		LOG(INFO) << "USB 2.0 device found on cam!";
		cam_usb_->SendUsbSpeed2Fpga(LIBUSB_SPEED_HIGH);
	} else {
		LOG(INFO) << "Device speed unknown on cam!";
	}
}

}