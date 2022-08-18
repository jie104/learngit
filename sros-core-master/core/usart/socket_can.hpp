/*
 * Author: Hongxiang Li
 *
 * Created on Sep 13, 2018, 09:35
 *
 * Copyright 2018
 */

#ifndef _SOCKET_CAN_H
#define _SOCKET_CAN_H

#include <stdio.h>
#include <unistd.h>
#include <stdlib.h>
#include <string.h>
#include <libgen.h>
#include <net/if.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <linux/can/bcm.h>

#include <glog/logging.h>

#include <boost/noncopyable.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/enable_shared_from_this.hpp>
#include <boost/asio.hpp>
#include <boost/array.hpp>
#include <boost/function.hpp>

#include <deque>
#include <iomanip>
#include <memory>
#include <string>
#include <vector>
#include <iostream>

class SocketCan;

typedef std::shared_ptr<SocketCan> SocketCan_ptr;

class SocketCan : public std::enable_shared_from_this<SocketCan>,
                  private boost::noncopyable {
public:
    /**
     * Constructor.
     * \param can device name, example "can0" or "vcan0"
     */
    SocketCan() : socket_(0), addr_() { }

    ~SocketCan() {
        if (socket_) {
            close();
        }
    }

    bool open(const std::string &can_device, int recv_timeout = 0) {
        /* open socket */
        if ((socket_ = socket(PF_CAN, SOCK_RAW, CAN_RAW)) < 0) {
            LOG(ERROR) << "device " << can_device << " socket create failed.";
            return false;
        }

        struct ifreq ifr;
        strncpy(ifr.ifr_name, can_device.c_str(), IFNAMSIZ - 1);
        ifr.ifr_name[IFNAMSIZ - 1] = '\0';
        ifr.ifr_ifindex = if_nametoindex(ifr.ifr_name);

        ioctl(socket_, SIOCGIFINDEX, &ifr);

        if (!ifr.ifr_ifindex) {
            LOG(ERROR) << "device " << can_device << " error in ifr_ifindex.";
            return false;
        }

        struct timeval tv;
        tv.tv_sec = recv_timeout; // 设置socket接收操作超时
        tv.tv_usec = 0;
        setsockopt(socket_, SOL_SOCKET, SO_RCVTIMEO, (const char*)&tv, sizeof tv);

        addr_.can_family = AF_CAN;
        addr_.can_ifindex = ifr.ifr_ifindex;

        if (bind(socket_, (struct sockaddr *) &addr_, sizeof(addr_)) < 0) {
            LOG(ERROR) << "device " << can_device << " error in bind().";
            return false;
        }

        return true;
    }

    /**
     * Write a string to the CAN device.
     * \param s string to write
     */
    bool send(canid_t can_id, const std::vector<uint8_t> &content) {
        if (socket_ == 0) {
            LOG(WARNING) << "socket_can: socket is not ok";
            return false;
        }

        if (content.size() > CAN_MAX_DLEN) {
            LOG(ERROR) << "socket_can: write content is too long. " << content.size();
            return false;
        }

        auto frame = buildFrame(can_id, content);

        auto bytes = ::write(socket_, &frame, sizeof(struct can_frame));

        if (bytes < 0) {
            LOG(ERROR) << "socket_can: write error " << bytes;
            return false;
        }

        return true;
    }

    can_frame buildFrame(canid_t can_id, const std::vector<uint8_t> &data) {
        struct can_frame frame;
        frame.can_id = can_id;
        frame.can_dlc = data.size();
        int i = 0;
        while (i < data.size()) {
            frame.data[i] = data[i];
            i++;
        }
        return frame;
    }

    void startRecv() {
        while (socket_) {
            struct can_frame frame;

            auto nbytes = read(socket_, &frame, sizeof(struct can_frame));

//            LOG(INFO) << "socket_can: got " << nbytes << " byte frame";
//            print_frame(frame);

            if (nbytes < 0) {
                LOG(ERROR) << "socket_can: read error " << nbytes;
                break;
            }

            std::vector<uint8_t> data;

            if (frame.can_id & CAN_ERR_FLAG) {
                LOG(ERROR) << " error can frame!!!!!!!!!!!!!";
            } else if (frame.can_id & CAN_RTR_FLAG) {
                LOG(WARNING) << " request can frame!!!!!!!!!!!!!";
            } else {
                for (int i = 0; i < frame.can_dlc; i++) {
                    data.push_back(frame.data[i]);
                }

                if (msg_callback_f_) {
                    msg_callback_f_(frame.can_id, data);
                }
            }
        }

        LOG(WARNING) << "socket_can: recv loop exit";
    }


    void close() {
        ::close(socket_);
        socket_ = 0;
    }

    static void print_frame(can_frame frame) {
        int i;
        std::cout << "CAN FRAME: can_id " << std::setfill('0') << std::setw(2) << std::hex << static_cast<int>(frame.can_id) << ": ";
        if (frame.can_id & CAN_RTR_FLAG) {
            std::cout << "remote request";
        } else {
            std::cout << std::hex << "[" << static_cast<int>(frame.can_dlc) << "]";
            for (i = 0; i < frame.can_dlc; i++)
                std::cout << " " << std::setfill('0') << std::setw(2) << std::hex << static_cast<int>(frame.data[i]);
        }
        std::cout << std::endl;
    }

    typedef std::function<void(canid_t, const std::vector<uint8_t> &)> MessageCallbackFunc_t;

    void setMessageCallbackFunc(MessageCallbackFunc_t callback) {
        msg_callback_f_ = callback;
    }

private:
    int socket_;

    struct sockaddr_can addr_;

    MessageCallbackFunc_t msg_callback_f_;
};

#endif  /* _SOCKET_CAN_H */
