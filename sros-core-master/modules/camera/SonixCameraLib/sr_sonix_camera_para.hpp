//
// Created by duan on 19-11-11.
//

#ifndef SONIX_CAMERA_SONIX_CAMERA_H
#define SONIX_CAMERA_SONIX_CAMERA_H

#include <iostream>
#include <vector>
#include <cstring>
#include <memory>
#include <glog/logging.h>

#include "sr_sonix_camrea.h"

#define FLASH_ADDRESS_START 0xF000
#define FLASH_ADDRESS_END 0xF800
#define FLASH_ADDRESS_OFFSET 16

namespace usb{

class SonixCamera {
public:
    enum ModeType{
        READ_FLASH = 0,
        WRITE_FLASH = 1,
    };


public:
    SonixCamera() = default;

    ~SonixCamera() = default;

    bool open(int first_search_id, int &founded_usb_id, int &dev_number){
        int ret = -1;
        ret = sr_sonix_cam_init(first_search_id, &founded_usb_id, &dev_number);
        if(ret < 0){
            LOG(INFO) << "can not open sonix camera by usb method!!!";
        }else{
            LOG(INFO) << "sonix camera open succeed!!!";
            is_opened = true;
        }

        return (ret < 0 ? false : true);
    }

    bool close(){
        int ret = sr_sonix_deinit();

        if(ret < 0){
            LOG(INFO) << "sonix camera close failed!!!";
        }else{
            LOG(INFO) << "sonix camera close succeed!!!";
        }
        
        if(!is_opened){
            return false;
        }

        return (ret < 0 ? false : true);
    }


    bool writeParaToFlash(const std::string &serial_number, const std::vector<double> &intrinsic,
                          const std::vector<double> &distort, const std::vector<double> &vanish_point, bool is_camera_down = false){

        if(serial_number.size() != sn_size_){
            LOG(INFO) << "sonix camera : serial_number size is not euqal to 16!!!";
            return false;
        }

        if(intrinsic.size() != intrinsic_number_){
            LOG(INFO) << "sonix camera : number of intrinsic is not euqal to 9!!!";
            return false;
        }

        if(distort.size() != distort_number_){
            LOG(INFO) << "sonix camera : number of distort para is not euqal to 5!!!";
            return false;
        }

        if(vanish_point.size() != vanish_number_){
            LOG(INFO) << "sonix camera : vanish point size is not euqal to 2!!!";
            return false;
        }


        std::string camera_type = is_camera_down ? "dw;" : "up;";
        std::string str_all_data = camera_type + serial_number + ";";
        for(auto &data : intrinsic){
            str_all_data += std::to_string(data) + ";";
        }

        for(auto &data : distort){
            str_all_data += std::to_string(data) + ";";
        }

        for(auto &data : vanish_point){
            str_all_data += std::to_string(data) + ";";
        }

        return (writeStringToFlash(str_all_data));
    }

//    bool writeCameraTypeToFlash(bool is_camera_down){
//        if(is_camera_down){
//            std::string str ="dw;";
//            return (writeStringToFlash(str));
//        }
//
//        return true;
//    }


    bool readParaFromFlash(std::string &camera_type, std::string &serial_number, std::vector<double> &intrinsic,
                           std::vector<double> &distort, std::vector<double> &vanish_point){

        camera_type = std::string();
        serial_number = std::string();
        intrinsic = std::vector<double>();
        distort = std::vector<double>();
        vanish_point = std::vector<double>();

        std::string data_str;           //sn, intrinsic, distort, vanish
        bool is_read_ok = readStringFromFlash(data_str);

//        LOG(INFO) << "read para from flash is :" << is_read_ok;
//        LOG(INFO) << "data_str flash is :" << data_str;

        if(is_read_ok){
            const std::string split = ";";
            char *s_input = (char*)(data_str.c_str());
            char *p = std::strtok(s_input, split.c_str());

            int index = 1;
            if(p != NULL){
                camera_type = std::string(p);
                p = std::strtok(NULL, split.c_str());

                int first_index = sn_number_;
                int second_index = first_index+ intrinsic_number_;
                int third_index = second_index + distort_number_;
                int forth_index = third_index + vanish_number_;

                while (p != NULL){
                    double val = std::atof(p);
                    if(index <= first_index){
                        serial_number = std::string(p);
                    }else if(index <= second_index && index > first_index )
                        intrinsic.push_back(val);
                    else if(index <=  third_index && index > second_index)
                        distort.push_back(val);
                    else if(index <= forth_index && index > third_index)
                        vanish_point.push_back(val);
                    index ++;

                    p = std::strtok(NULL, split.c_str());
                }
            }
        }

        return is_read_ok;
    }


private:
    bool readStringFromFlash(std::string &str) {
        LOG(INFO) << "read string from flash!";
        const int length = 512;
        uint8_t buf[length] = {0};
//        memset(buf, 0, sizeof(buf));
        int ret = sr_serial_flash_rw(READ_FLASH, FLASH_ADDRESS_START, buf, length);

        bool is_ok = ret < 0 ? false : true;
        if(is_ok){
            str = std::string((char*)buf);
        }

        return is_ok;
    }

    bool writeStringToFlash(const std::string &str){

        int ret = sr_serial_flash_rw(WRITE_FLASH, FLASH_ADDRESS_START, (uint8_t*)(str.c_str()), str.size());

        return (ret < 0 ? false : true);
    }


private:
    int sn_size_ = 16;
    int sn_number_ = 1;
    int intrinsic_number_ = 9;
    int distort_number_ = 5;
    int vanish_number_ = 2;

    bool is_opened = false;

//    uint16_t cur_address_ = FLASH_ADDRESS_START;

};


typedef std::shared_ptr<SonixCamera> SonixCameraPtr;

}


#endif //SONIX_CAMERA_SONIX_CAMERA_H
