//
// Created by lhx on 17-2-18.
//

#ifndef SROS_CONNECTION_H
#define SROS_CONNECTION_H

#include "async_serial.h"
#include "core/state.h"
#include "core/logger.h"
#include "core/device/device.h"
#include "core/dump.h"

namespace usart {

using namespace std;

/**
 * 将串口通信抽象为一个连接，管理串口设备/Frame
 */
template<class Frame>
class Connection : private boost::noncopyable {
public:
    Connection();

    Connection(const string& dev_name, unsigned int baud_rate, bool rs485_mode = false);

    ~Connection();

    bool connect(const string& dev_name, unsigned int baud_rate, bool rs485_mode = false);

    bool disconnect();

    bool isConnected() const { return serial_ptr_ && serial_ptr_->isOpen(); }

    bool sendData(const vector<uint8_t>& data);

    typedef boost::function<void(const vector<uint8_t>&)> RecvDataCallback_t;
    void setRecvDataCallback(RecvDataCallback_t callback);
    void setHwDevName(const string hwDevName);
    string get_dev_name();
private:

    /// 接收底层串口设备的数据
    void onRecvSerialData(const char* data, size_t size);

    void onRecvSerialDataBySrc(const char* data, size_t size);

    string hw_dev_name_;

    string serial_dev_name_;
    unsigned int serial_baud_rate_;
    bool rs485_mode_;

    RecvDataCallback_t recv_data_callback_;

    Frame frame_;

    typedef std::shared_ptr<CallbackAsyncSerial> SerialDevice_ptr;
    SerialDevice_ptr serial_ptr_; // 串口设备
};

template <class  Frame>
string Connection<Frame>::get_dev_name() {
    return serial_dev_name_;
}
template<class Frame>
Connection<Frame>::Connection() {

}

template<class Frame>
Connection<Frame>::Connection(const string &dev_name, unsigned int baud_rate, bool rs485_mode)
        : serial_dev_name_(dev_name),
          serial_baud_rate_(baud_rate),
          rs485_mode_(rs485_mode) {

}

template<class Frame>
Connection<Frame>::~Connection() {
    disconnect();
}

template<class Frame>
bool Connection<Frame>::connect(const string &device_name, unsigned int baud_rate, bool rs485_mode) {
    serial_dev_name_ = device_name;
    serial_baud_rate_ = baud_rate;
    rs485_mode_ = rs485_mode;

    serial_ptr_ = make_shared<CallbackAsyncSerial>();
    serial_ptr_->open(serial_dev_name_, serial_baud_rate_, rs485_mode_);

    serial_ptr_->setCallback(boost::bind(&Connection::onRecvSerialData, this, _1, _2));

    return serial_ptr_->isOpen();
}


template<class Frame>
bool Connection<Frame>::disconnect() {
    if (serial_ptr_ && serial_ptr_->isOpen()) {
        serial_ptr_->close();
        return true;
    }

    return false;
}

template<class Frame>
bool Connection<Frame>::sendData(const vector<uint8_t> &data) {
    if (!serial_ptr_ || !serial_ptr_->isOpen()) {
        return false;
    }

    auto frame_data = frame_.generateFrameData(data);

//    LOG(INFO) << numberListToStr(frame_data.cbegin(), frame_data.cend());

    serial_ptr_->write((char *)(&frame_data[0]), frame_data.size());

    return true;
}

template<class Frame>
void Connection<Frame>::onRecvSerialData(const char *data, size_t size) {

    if(sros::core::Dump::getInstance()->runningDumpSrc()
        && (hw_dev_name_ == sros::device::DEVICE_SRC)) {
        vector<uint8_t > recv_data;
        for (int i = 0; i < size; i++) {
            recv_data.push_back((uint8_t) data[i]);
        }

        sros::core::Dump::getInstance()->appendSrcData(numberListToStr(recv_data.cbegin(), recv_data.cend()));
    }

    if(sros::core::Dump::getInstance()->runningDumpVsc()
       && (hw_dev_name_ == sros::device::DEVICE_VSC)) {
        vector<uint8_t > recv_data;
        for (int i = 0; i < size; i++) {
            recv_data.push_back((uint8_t) data[i]);
        }

        sros::core::Dump::getInstance()->appendVscData(numberListToStr(recv_data.cbegin(), recv_data.cend()));
    }


    for (int i = 0; i < size; i++) {
        frame_.feed((uint8_t) data[i]);

        if (frame_.isFrameReady()) { // 一帧数据接收完毕
            auto payload = frame_.getPayload();

            if (recv_data_callback_) {
                recv_data_callback_(payload);
            }
        }
    }
}

template<class Frame>
void Connection<Frame>::setRecvDataCallback(Connection::RecvDataCallback_t callback) {
    recv_data_callback_ = callback;
}

template<class Frame>
void Connection<Frame>::setHwDevName(const string hwDevName) {
    hw_dev_name_ = hwDevName;
}

}

#endif //SROS_CONNECTION_H
