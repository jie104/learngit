/**
 * describe: 
 * Created by caoyan on 2021-3-4.
**/

#include "rfid_protocol_pepperl_fuchs.h"

#include <chrono>
#include <future>
#include <vector>
#include <ctype.h>
#include "rfid_manager.h"
#include "core/settings.h"

namespace ac {

/*
0x57, 0x50, 0x55, 0x51, 0x56, 0x00, 0x01, 0x53, 0x23, 0x0D
30 23 0D 

0x57, 0x50, 0x55, 0x50, 0x54, 0x00, 0x02, 0x00, 0x00, 0x23, 0x0D
30 23 0D

0x53, 0x53, 0x30, 0x23, 0x0D
35 23 0D 没有标签
41 23 0D 同时读取到多个标签
30 34 00 30 08 33 b2 dd d9 01 40 20 19 06 04 23 0d 单个标签值

34 23 0d 错误
*/
   
    

//返回状态定义
#define STATE_SUCCESS 0x30                  //成功
#define STATE_FAIL_NO_LABEL 0x35            //没有标签
#define STATE_FAIL_MULTIPLE_LABEL 0x41      //读取到多个标签


//报文结尾定义
#define CHAR_END   0x0D                     //最后一个
#define CHAR_END_LAST  0x23                 //倒数第二个

RfidProtPepperlFuchs::RfidProtPepperlFuchs():RfidProtInterface() {}    

RfidProtPepperlFuchs::~RfidProtPepperlFuchs() {}

bool RfidProtPepperlFuchs::init() {
    //设置标签模式
    if(!setSLabelMode()) {
        return false;
    }

    auto &s = sros::core::Settings::getInstance();
    uint16_t rfid_ppf_power = s.getValue<unsigned int>("device.rfid_ppf_power", 500);
    LOG(INFO) << "rfid_ppf_power:" << rfid_ppf_power;

    //设置功率
    if(!setPowerValue(rfid_ppf_power)) {
        return false;
    }

    return true;
}

bool RfidProtPepperlFuchs::setData(std::vector<uint8_t>& send_data) {
    LOG(INFO) << "send data: " << vecToString(send_data);

    is_async_get_rfid_ = true;
    aysnc_rfid_string_ = "";
    set_async_rfid_string_.clear();

    RfidManager::getInstance()->sendData(send_data);

    return true;
}

bool RfidProtPepperlFuchs::setDataAndRecv(std::vector<uint8_t>& send_data, std::vector<uint8_t>& recv_data) {
    //清除接收的数据
    recv_raw_data_.clear();

    //Matrix配置增加“rfid倍加福请求应答超时设置“项
    auto &s = sros::core::Settings::getInstance();
    uint16_t rfid_reqepc_timeout = s.getValue<unsigned int>("device.rfid_reqepc_timeout", 2000);
    //LOG(INFO) << "rfid_reqepc_timeout:" << rfid_reqepc_timeout;
    sync_wait_time_ = rfid_reqepc_timeout;

    LOG(INFO) << "send data: " << vecToString(send_data);

    //等待数据
    promise_ptr_ = std::make_shared<std::promise<std::vector<uint8_t>>>();
    auto future = promise_ptr_->get_future();

    RfidManager::getInstance()->sendData(send_data);

    auto status = future.wait_for(std::chrono::milliseconds(sync_wait_time_)); 
    if(status == std::future_status::timeout) {
        promise_ptr_ = nullptr;
        LOG(ERROR) << "recv timeout";
        return false;
    }

    recv_data = future.get();
    LOG(INFO) << "recv data: " << vecToString(recv_data);
    promise_ptr_ = nullptr;
    return true;
}

void RfidProtPepperlFuchs::parseMsg(std::vector<uint8_t> &data) {
    recv_raw_data_.insert(recv_raw_data_.end(), data.begin(), data.end());
    //获取返回数据
    int len = recv_raw_data_.size();
    
    if(len < 3) {
        return;
    }

    //以0x23, 0x0D 为一笔报文的结尾
    if(recv_raw_data_[len - 1] == CHAR_END && recv_raw_data_[len - 2] == CHAR_END_LAST) {

        LOGGER(INFO, ACTION_TASK) << "parseMsg: " << vecToString(recv_raw_data_);

        std::unique_lock<std::mutex> lk(mutex_);

        if (is_async_get_rfid_) {

            std::string rfid_string;

            bool ret = parseRfid(recv_raw_data_, rfid_string);

            if (ret) {

                auto &s = sros::core::Settings::getInstance();
                auto label_mode = s.getValue<std::string>("device.rfid_ppf_label_mode", SINGLE_LABEL);
                if (label_mode == SINGLE_LABEL) {
                    //串读，后面的线盘反而读取的效果更好，就用第一次读取到的值
                    if (aysnc_rfid_string_ == "") {
                        aysnc_rfid_string_ = rfid_string;
                    } else {
                        LOG(INFO) << "ignore ...";
                    }
                    
                } else {
                    set_async_rfid_string_.insert(rfid_string);
                }
            }
            
            recv_raw_data_.clear();

            return;
        }

        if(promise_ptr_) {
            promise_ptr_->set_value(recv_raw_data_);
            recv_raw_data_.clear();
        }
    }

}

bool RfidProtPepperlFuchs::setSLabelMode() {

    auto &s = sros::core::Settings::getInstance();

    auto label_mode = s.getValue<std::string>("device.rfid_ppf_label_mode", SINGLE_LABEL);

    bool ret = false;
    std::vector<uint8_t> recv_data;

    if (label_mode == SINGLE_LABEL) {

        ret = setDataAndRecv(set_slabel_data_, recv_data);

    } else {
        ret = setDataAndRecv(set_multi_slabel_data_, recv_data);
    }
    
    if(!ret) {
        LOG(ERROR) << "setDataAndRecv fail";
        return false;
    }

    //报文解析
    if(recv_data.size() != 3 || recv_data[0] != STATE_SUCCESS) {
        LOG(ERROR) << "setSLabelMode fail";
        return false;
    } else {
        LOG(INFO) << "setSLabelMode success";
    }

    return true;
}

bool RfidProtPepperlFuchs::setPowerValue(uint16_t value) {

    set_power_data_[7] = value >> 8;
    set_power_data_[8] = value & 0xff;

    std::vector<uint8_t> recv_data;
    bool ret = setDataAndRecv(set_power_data_, recv_data);

    if(!ret) {
        LOG(ERROR) << "setDataAndRecv fail";
        return false;
    }

    //报文解析
    if(recv_data.size() != 3 || recv_data[0] != STATE_SUCCESS) {
        LOG(ERROR) << "setPowerValue fail";
        return false;
    } else {
        LOG(INFO) << "setPowerValue success";
    }

    return true;

}

bool RfidProtPepperlFuchs::setQuitMOde() {
    std::vector<uint8_t> recv_data;
    bool ret = setDataAndRecv(quit_enhanced_data_, recv_data);
    
    if(!ret) {
        LOGGER(ERROR, ACTION_TASK) << "setQuitMOde timeout";
        return false;
    }

    //报文解析
    if(recv_data.size() != 3 || recv_data[0] != STATE_SUCCESS) {
        LOGGER(ERROR, ACTION_TASK) << "setQuitMOde fail";
        return false;
    } else {
        LOG(INFO) << "setQuitMOde success";
    }

    return true;
}

void RfidProtPepperlFuchs::asyncGetRfidCmd() {
     setData(get_epc_data_es_);
}

std::string RfidProtPepperlFuchs::asyncGetRfidData() {
    std::string rfid_string;

    do {
        std::unique_lock<std::mutex> lk(mutex_);

        auto &s = sros::core::Settings::getInstance();
        auto label_mode = s.getValue<std::string>("device.rfid_ppf_label_mode", SINGLE_LABEL);
        if (label_mode == SINGLE_LABEL) {
            rfid_string = aysnc_rfid_string_;
        } else {

            auto size = set_async_rfid_string_.size();
            if (size == 0) {
                rfid_string = "";

            } else if (size == 1) {
                LOGGER(INFO, ACTION_TASK) << "--------read_single_label--------";
                for (auto it : set_async_rfid_string_) {
                    rfid_string = it;
                }

                LOGGER(INFO, ACTION_TASK) << rfid_string;
                LOGGER(INFO, ACTION_TASK) << "---------------------------------";
            } else {
                LOGGER(INFO, ACTION_TASK) << "--------read_multi_label---------";
                for (auto it : set_async_rfid_string_) {
                    LOGGER(INFO, ACTION_TASK) << it;
                }
                LOGGER(INFO, ACTION_TASK) << "---------------------------------";
            }
            
        }

        is_async_get_rfid_ = false;
        promise_ptr_ = nullptr;

    } while(0);
    
    while(!setQuitMOde()) {
        LOG(INFO) << "setQuitMOde again";
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }

    return rfid_string;
}

bool RfidProtPepperlFuchs::parseRfid(const std::vector<uint8_t>& recv_data, 
                                     std::string& rfid_string) {
    
    //LOG(INFO) << "parse_rfid_raw_data: " << vecToString(recv_data);                                     

    //报文解析
    int len = recv_data.size();
    if(len == 3) {
        //失败时报文长度为3
        if(recv_data[0] == STATE_FAIL_NO_LABEL) {
            LOGGER(ERROR, ACTION_TASK) << "get rfid fail, no label";
            return false;
        } else if(recv_data[0] == STATE_FAIL_MULTIPLE_LABEL) {
            LOGGER(ERROR, ACTION_TASK) << "get rfid fail, multiple label";
            return false;
        }
    } else if(len == 17) {
        //成功时报文长度为17，返回 从[4,10] 这7个字节
        std::vector<uint8_t> rfid_data(recv_data.begin() + 3, recv_data.end() - 7);
        std::string hexString = vecToString(rfid_data);
        if(isAscii(rfid_data)){
            rfid_string.assign(std::string(rfid_data.begin(), rfid_data.end()));
            LOGGER(INFO, ACTION_TASK) << "get rfid, hexStr: " << hexString << ", asciiStr: " << rfid_string;
            return true;
        }else{
            LOGGER(ERROR, ACTION_TASK) << "not ascci，cannot convert string";
            return false;
        }
    } else if(len == 19) { //多标签 ES模式下返回的标签
        //成功时报文长度为19，返回 从[6,12] 这7个字节
        std::vector<uint8_t> rfid_data(recv_data.begin() + 5, recv_data.end() - 7);
        std::string hexString = vecToString(rfid_data);
        if(isAscii(rfid_data)){
            rfid_string.assign(std::string(rfid_data.begin(), rfid_data.end()));
            LOGGER(INFO, ACTION_TASK) << "get rfid, hexStr: " << hexString << ", asciiStr: " << rfid_string;
            return true;
        }else{
            LOGGER(ERROR, ACTION_TASK) << "not ascci，cannot convert string";
            return false;
        }
    } else {
        LOGGER(ERROR, ACTION_TASK) << "get rfid fail, err msg";
    }

    return false;
}

std::string RfidProtPepperlFuchs::getRfid() {
    std::vector<uint8_t> recv_data;
    std::string rfid_string;

    bool ret = setDataAndRecv(get_epc_data_, recv_data);

    if(!ret) {
        LOG(ERROR) << "setDataAndRecv fail";
        return rfid_string;
    }

    //报文解析
    int len = recv_data.size();
    if(len == 3) {
        //失败时报文长度为3
        if(recv_data[0] == STATE_FAIL_NO_LABEL) {
            LOGGER(ERROR, ACTION_TASK) << "get rfid fail, no label";
            return rfid_string;
        } else if(recv_data[0] == STATE_FAIL_MULTIPLE_LABEL) {
            LOGGER(ERROR, ACTION_TASK) << "get rfid fail, multiple label";
            return rfid_string;
        }
    } else if(len == 17) {
        //成功时报文长度为17，返回 从[4,10] 这7个字节
        std::vector<uint8_t> rfid_data(recv_data.begin() + 3, recv_data.end() - 7);
        std::string hexString = vecToString(rfid_data);
        if(isAscii(rfid_data)){
            rfid_string.assign(std::string(rfid_data.begin(), rfid_data.end()));
            LOGGER(INFO, ACTION_TASK) << "get rfid, hexStr: " << hexString << ", asciiStr: " << rfid_string;
            return rfid_string;
        }else{
            LOGGER(ERROR, ACTION_TASK) << "not ascci，cannot convert string";
            return rfid_string;
        }
       
    } else {
        LOGGER(ERROR, ACTION_TASK) << "get rfid fail, err msg";
    }

    return rfid_string;

}

//数据是否为ascci码
bool RfidProtPepperlFuchs::isAscii(std::vector<uint8_t>& data) {
    for (int i = 0; i < data.size();i++)
    {
        if(!isascii(static_cast<int>(data[i]))){
           return false;
        }
    }
    
    return true;
}

}
