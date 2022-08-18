//
// Created by duan on 19-11-23.
//

#ifndef SONIX_CAMERA_SR_SONIX_CAMERA_INTERFACE_H
#define SONIX_CAMERA_SR_SONIX_CAMERA_INTERFACE_H

#include <memory>
#include <fstream>
#include <iostream>
#include "sr_sonix_camera_para.hpp"
#include "V4L2Cam/camera.h"
namespace usb{

class SonixCameraInterface{
public:
    SonixCameraInterface(){
        usb_camera_ptr_ = std::make_shared<SonixCamera>();
        v4l2_camera_ptr_ = std::make_shared<dml_camera::V4L2Camera>();
    }

    ~SonixCameraInterface(){

    }

    bool readUsbFlash(const std::string &v4l2_address, std::string &serial_number, std::vector<double> &intrinsic,
                      std::vector<double> &distort, std::vector<double> &vanish_point){

        bool read_ok = false;
        if(v4l2_camera_ptr_->open_device(v4l2_address)){
            std::string usb_bus;
            if(v4l2_camera_ptr_->getUsbBus(usb_bus)){
                v4l2_camera_ptr_->close_device();
                LOG(INFO) << "usb_bus is :" << usb_bus;
                read_ok = getCameraPara(usb_bus, serial_number, intrinsic, distort, vanish_point);
            }else{
                v4l2_camera_ptr_->close_device();
            }
        }else{
            LOG(INFO) << "readFlash : can not open V4L2 camera";
        }

        return read_ok;
    }


private:
    /**
     *
     * @param camera_type
     * @param serial_number, camera serial number which is defined by standard-robots company, 3060011910300001
     * @param intrinsic
     * @param distort
     * @param vanish_point
     * @return
     */
    bool getCameraPara(const std::string &v4l2_usb_address, std::string &serial_number, std::vector<double> &intrinsic,
                         std::vector<double> &distort, std::vector<double> &vanish_point){
        serial_number = std::string();
        intrinsic.clear();
        distort.clear();
        vanish_point.clear();

        if(getCameraParaInteral(v4l2_usb_address)){
            serial_number = serial_number_;
            intrinsic = intrinsic_;
            distort = distort_;
            vanish_point = vanish_point_;

            LOG(INFO) << "read flash is succeed!!!";
            return true;
        }else{
            return false;
        }
    }


    bool getCameraParaInteral(const std::string &type){

        int v4l2_dev_number = getV4l2DevNumber(type);
        if(v4l2_dev_number < 0){
            LOG(WARNING) << "canot resolve the dev number! will return false!" << v4l2_dev_number;
            return false;
        }

        int usb_dev_number = -1;
        first_search_id_ = v4l2_dev_number;
        if(usb_camera_ptr_->open(first_search_id_, find_usb_id_, usb_dev_number)){
            int read_state = usb_camera_ptr_->readParaFromFlash(camera_type_, serial_number_, intrinsic_, distort_, vanish_point_);
            usb_camera_ptr_->close();
            LOG(INFO) << "find usb id is :" << find_usb_id_ << "serial number:" << serial_number_ << "," << read_state;
            if (serial_number_.size() != 16) {
                LOG(INFO) << "serial number is wrong!!";
                return false;
            }
        }else{      //没有找到usb
            usb_camera_ptr_->close();
            LOG(INFO) << "can not find  " + type + " camera";
            return false;
        }

        return true;
    }

    int getV4l2DevNumber(const std::string &usb_str){
        // pci-0000:00:14.0-usb-0:11:1.0-video-index0
        // pci-0000:00:14.0-usb-0:6:1.0-video-index0

//        std::string loc_str = "usb-0:";
//        int loc_pos = usb_str.find(loc_str);
//
//        std::string sub_str = usb_str.substr(loc_pos, usb_str.length() - loc_pos);
//        int pos1 = sub_str.find_first_of(":");
//        int pos2 = sub_str.find_last_of(":");
//
//        if(pos2 == pos1)
//            return -1;

        int pos = usb_str.find_last_of("-");
        std::string num = usb_str.substr(pos + 1, usb_str.length() - 1);
        std::string fold_index = "/sys/bus/usb/drivers/usb/4-" + num + "/devnum";

        int dev_number = -1;
        std::ifstream intext(fold_index);
        if(!intext.good()){
            LOG(INFO) << "can not open " + fold_index;
            std::string fold_index = "/sys/bus/usb/drivers/usb/1-" + num + "/devnum";
            std::ifstream intext(fold_index);
            if (!intext.good()) {

            }else{
                intext >> dev_number;
            }
        }else{
            intext >> dev_number;
        }
        intext.close();

        LOG(INFO) << "usb dev number is :" << dev_number;

        return dev_number;
    }
public:
    dml_camera::V4L2CameraPtr v4l2_camera_ptr_;
private:
    int first_search_id_ = 0;
    int find_usb_id_ = -1;
    SonixCameraPtr usb_camera_ptr_;

    //para
    std::string camera_type_;
    std::string serial_number_;
    std::vector<double> intrinsic_;
    std::vector<double> distort_;
    std::vector<double> vanish_point_;

};


typedef std::shared_ptr<SonixCameraInterface> SonixCameraInterfacePtr;

}



#endif //SONIX_CAMERA_SR_SONIX_CAMERA_INTERFACE_H
