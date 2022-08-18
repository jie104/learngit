//
// Created by duan on 20-4-9.
//
#ifndef CAMERA_DEVICE_MANAGE_H_
#define CAMERA_DEVICE_MANAGE_H_
#include <vector>
#include <string>
#include <map>
#include <fstream>
#include <memory>
#include <mutex>

#include <dirent.h>     //for DIR
#include <glog/logging.h>
#include <sys/stat.h>
#include "./SonixCameraLib/sr_sonix_camera_interface.hpp"


namespace Device{

    enum VidInfo{
        SVC100 = 0x0c45,
        MYNTEYE = 0x04b4
    };

    enum CameraState{
        OPEND = 0,          //打开状态
        CLOSED = 1,         //关闭状态
        DROPPED             //掉线状态
    };

    struct CameraDevice{
        int vid = 0;
        int pid = 0;
        std::string video_name;     //like, video0, video1
        std::string serial_number;
        std::string name;           //like, MYNTEYE-S-1030,
        CameraState state = CLOSED;          //相机状态

        std::vector<double> intrinsic;
        std::vector<double> distort;
        std::vector<double> vanish;

        bool isValid(){
            return (vid >0 && pid > 0 && !serial_number.empty() && !video_name.empty());
        }
    };

    class CameraManaged{
    public:
        CameraManaged() = default;

        ~CameraManaged() = default;

        /**
         *
         * @param camera_serial_number
         * @param is_opened, true is open, false is closed or line dropped
         */
        void setCameraState(const std::string camera_serial_number, CameraState state){

            mutex_.lock();

            auto it = camera_devices_.find(camera_serial_number);

            if(it != camera_devices_.end()){
                if(state == CameraState::DROPPED){
                    camera_devices_.erase(camera_serial_number);

//                    for(auto camera : camera_devices_){
//                        LOG(INFO) << "dropped later, camera device is:" << camera.second.video_name;
//                    }
                }else{
                    camera_devices_[camera_serial_number].state = state;
                }
            }else{
                LOG(INFO) << "camera serial number is not valid, during set camera dropped!!!";
            }

            mutex_.unlock();
        }

        std::string getCameraAddress(const std::string camera_serial_number){

            mutex_.lock();

            //如果没有初始化就进行调用，需要先索引全部相机map
            if(is_first_getall_){
                std::map<std::string, CameraDevice>  temp;
                int count = 0;
                getAllCameraDevice(temp,count);
                is_first_getall_ = false;
            }

            std::string video_name;
            auto it = camera_devices_.find(camera_serial_number);
            if(it != camera_devices_.end()) {
                //如果相机不处于掉线状态，随时可以取走相机序列码
                video_name = getCameraAddressInner(camera_serial_number);

            }else{
                //相机掉线了，需要重新扫描端口，重新分配相机Video地址（例如：video0）
                getDroppedDevice();

                video_name = getCameraAddressInner(camera_serial_number);
            }

            mutex_.unlock();

            return video_name;
        }

        void removeCameraAddress(std::string camera_serial_number){
            mutex_.lock();
            auto it = camera_devices_.find(camera_serial_number);
            if (it != camera_devices_.end()) {
                camera_devices_.erase(it);
            }
            mutex_.unlock();
        }

        void getAllCameraDevice(std::map<std::string, CameraDevice> &devices,int &totally_count){
            int temp_totally_count_=0;
            //第一次初始化时，遍历全部设备
            totally_count_ = 0;
            if(is_first_getall_){
                camera_devices_.clear();
                is_first_getall_ = false;

                std::vector<std::string> device_names = queryDevies();
                for(auto &name : device_names){
                    CameraDevice device;
                    getPidVid(name, device);

                    //找到SVC100相机，且相机没有进行图像采集，可以打开相机读取flash程序
                    if(device.vid == VidInfo::SVC100){
                        uint16_t flag=0;
                        temp_totally_count_=totally_count_;
                        usb::SonixCameraInterfacePtr interface_ptr = std::make_shared<usb::SonixCameraInterface>();
                        if (!interface_ptr->v4l2_camera_ptr_->open_device(device.video_name))
                        {
                            flag=1;
                        }
                        if (!interface_ptr->v4l2_camera_ptr_->initCamera())
                        {
                            flag=1;
                        }
                        if (!interface_ptr->v4l2_camera_ptr_->close_device())
                        {
                            flag=1;
                        }
                        if (!interface_ptr->readUsbFlash(device.video_name, device.serial_number, device.intrinsic, device.distort, device.vanish))
                        {
                            flag=1;
                        }
                        if (flag==0)
                        {
                            totally_count_++;
                        }
                        LOG(INFO) << "device.vid:" << device.vid <<  "  device.pid:"<< device.pid<<"  device.name:"<<device.name<<"  device.video_name:"<<device.video_name<<"  device.serial_number:"<<device.serial_number;
                    }else{
                        LOG(ERROR) << "Cannot support vid of camera is :" << device.vid;
                    }

                    if(device.isValid()&&(totally_count_!=temp_totally_count_)){
                        camera_devices_.emplace(device.serial_number, device);
                    }
                }
            }else{
                //初始化成功后，维护map，重新连接是否有掉线的相机
                getDroppedDevice();
            }
            totally_count = totally_count_;
            devices = camera_devices_;
            LOG(INFO) << "totally_count:" << totally_count << "devices.size" << devices.size();   
        }

    private:
        std::map<std::string, CameraDevice> camera_devices_;
        int totally_count_ = 0;
        std::mutex mutex_;
        bool is_first_getall_ = true;

    private:
        void getDroppedDevice(){
            std::vector<std::string> device_names = queryDevies();
            for(int i = 0; i < device_names.size(); i++){
                bool is_find = false;

                //判断找到的设备是否处于打开状态
                std::string name_temp = "/dev/" + device_names[i];
                for(auto camera : camera_devices_){
                    if(camera.second.video_name == name_temp)
                        is_find = true;
                }

                if(is_find){
                    //在map中找到当前设备,读取已经关闭、但没掉线相机的参数
                    for(auto camera : camera_devices_){
                        if(camera.second.video_name == name_temp){

                            //相机在不占用的状态下，才能读取flash参数
                            if(camera.second.vid == VidInfo::SVC100 && camera.second.state == CameraState::CLOSED){

                                usb::SonixCameraInterfacePtr interface_ptr = std::make_shared< usb::SonixCameraInterface>();

                                interface_ptr->readUsbFlash(camera.second.video_name,
                                                            camera.second.serial_number,
                                                            camera.second.intrinsic,
                                                            camera.second.distort,
                                                            camera.second.vanish);
                            }
                        }
                    }
                }else{
                    //当前map没有当前usb设备，则为掉线设备， 新建一个设备
                    CameraDevice device;
                    getPidVid(device_names[i], device);
                    int temp_totally_count_=0;
                    //找到SVC100相机，且相机没有进行图像采集，可以打开相机读取flash程序
                    if(device.vid == VidInfo::SVC100){
                        uint16_t flag=0;
                        temp_totally_count_=totally_count_;
                        usb::SonixCameraInterfacePtr interface_ptr = std::make_shared< usb::SonixCameraInterface>();
                        if (!interface_ptr->v4l2_camera_ptr_->open_device(device.video_name))
                        {
                            flag=1;
                        }
                        if (!interface_ptr->v4l2_camera_ptr_->initCamera())
                        {
                            flag=1;
                        }
                        if (!interface_ptr->v4l2_camera_ptr_->close_device())
                        {
                            flag=1;
                        }
                        if (!interface_ptr->readUsbFlash(device.video_name, device.serial_number, device.intrinsic, device.distort, device.vanish))
                        {
                            flag=1;
                        }
                        if (flag==0)
                        {
                            totally_count_++;
                        }
                        LOG(INFO) << "device.vid:" << device.vid <<  "  device.pid:"<< device.pid<<"  device.name:"<<device.name<<"  device.video_name:"<<device.video_name<<"  device.serial_number:"<<device.serial_number;
                    }else{
                        LOG(ERROR) << "Cannot support vid of camera is :" << device.vid;
                    }

                    if(device.isValid()&&(totally_count_!=temp_totally_count_)){
                        camera_devices_.emplace(device.serial_number, device);
                    }
                }
            }
        }

        std::string getCameraAddressInner(const std::string camera_serial_number) {

            auto it = camera_devices_.find(camera_serial_number);

            if (it != camera_devices_.end()) {

                if (camera_devices_[camera_serial_number].state != CameraState::DROPPED) {
                    return camera_devices_[camera_serial_number].video_name;
                } else {
                    LOG(INFO) << "camera serial number is not valid, during getting camera address!!!";
                }
            }

            return std::string();
        }

        void getPidVid(const std::string &name, CameraDevice &device){

            std::string dev_name = "/dev/" + name;
            device.video_name = dev_name;
//            device.name = name;

            struct stat st;
            if (stat(dev_name.c_str(), &st) < 0) {  // file status
                LOG(ERROR) << "Cannot identify '" << dev_name << "': " << errno << ", "
                           << strerror(errno);
            }

            if (!S_ISCHR(st.st_mode)) {  // character device?
                LOG(ERROR) << dev_name << " is no device";
            }

            if (!(std::ifstream("/sys/class/video4linux/" + name + "/name") >> device.name))
                LOG(ERROR) << "Failed to read camera name!!!";

            std::string modalias;
            if (!(std::ifstream("/sys/class/video4linux/" + name + "/device/modalias") >> modalias))
                LOG(ERROR) << "Failed to read modalias!!!";
            if (modalias.size() < 14 || modalias.substr(0, 5) != "usb:v" ||
                modalias[9] != 'p')
                LOG(ERROR)<< "Not a usb format modalias";
            if (!(std::istringstream(modalias.substr(5, 4)) >> std::hex >> device.vid))
                LOG(ERROR) << "Failed to read vendor ID";
            if (!(std::istringstream(modalias.substr(10, 4)) >> std::hex >> device.pid))
                LOG(ERROR) << "Failed to read product ID";

        }

        std::vector<std::string> queryDevies(){

            std::vector<std::string> device_names;

            DIR *dir = opendir("/sys/class/video4linux");
            if (!dir)
                LOG(INFO) << "can not access /sys/class/video4linux";
            while (dirent *entry = readdir(dir)) {
                std::string name = entry->d_name;
                if (name == "." || name == "..")
                    continue;

                // Resolve a pathname to ignore virtual video devices
                std::string path = "/sys/class/video4linux/" + name;
                char buff[PATH_MAX];
                ssize_t len = ::readlink(path.c_str(), buff, sizeof(buff) - 1);
                if (len != -1) {
                    buff[len] = '\0';
                    std::string real_path = std::string(buff);
                    if (real_path.find("virtual") != std::string::npos)
                        continue;
                }
                try {
                    device_names.push_back(name);
                } catch (const std::exception &e) {
                    VLOG(2) << "Not a USB video device: " << e.what();
                }
            }
            closedir(dir);

//            for(int i = 0; i < device_names.size(); i++){
//                LOG(INFO) << "device name is :" << device_names[i];
//            }

            return device_names;
        }

    };

    typedef std::shared_ptr<CameraManaged> CameraManagedPtr;

}
#endif