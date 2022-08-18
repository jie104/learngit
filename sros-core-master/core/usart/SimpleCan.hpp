/*
 * Author: Peng Lei
 *
 * Created on Mar 9, 2018, 15:12 PM
 */

#ifndef _SIMPLECAN_H
#define _SIMPLECAN_H

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

#include <deque>
#include <iomanip>

#include <boost/noncopyable.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/enable_shared_from_this.hpp>
#include <boost/asio.hpp>
#include <boost/array.hpp>
#include <boost/function.hpp>


class SimpleCan;

typedef std::shared_ptr<SimpleCan> simple_can_ptr;
typedef std::shared_ptr<struct can_frame> can_frame_ptr;
typedef std::deque<can_frame_ptr> FrameQueue;

class SimpleCan : public std::enable_shared_from_this<SimpleCan>,
                  private boost::noncopyable {
public:
    /**
     * Constructor.
     * \param can device name, example "can0" or "vcan0"
     */
    SimpleCan(std::string &can_device, boost::asio::io_service &ios) :
            ios_(ios),
            stream_(ios) {
        init_socket(can_device);
        stream_.assign(can_raw_socket_);
    }

    int init_socket(const std::string &can_device) {
        /* open socket */
        if ((can_raw_socket_ = socket(PF_CAN, SOCK_RAW, CAN_RAW)) < 0) {
            perror("socket");
            return 1;
        }

        strncpy(ifr_.ifr_name, can_device.c_str(), IFNAMSIZ - 1);
        ifr_.ifr_name[IFNAMSIZ - 1] = '\0';
        ifr_.ifr_ifindex = if_nametoindex(ifr_.ifr_name);

        ioctl(can_raw_socket_, SIOCGIFINDEX, &ifr_);

        if (!ifr_.ifr_ifindex) {
            perror("if_nametoindex");
            return 1;
        }

        addr_.can_family = AF_CAN;
        addr_.can_ifindex = ifr_.ifr_ifindex;

        if (bind(can_raw_socket_, (struct sockaddr *) &addr_, sizeof(addr_)) < 0) {
            perror("bind");
            return 1;
        }
    }

    /**
     * Write a string to the CAN device.
     * \param s string to write
     */
    void writeString(uint32_t can_id, std::vector<uint8_t> &content) {
        if (content.size() > CAN_MAX_DLEN) {
            LOG(ERROR) << "writeString: content is too long. " << content.size();
            return;
        }

        auto frame = buildFrame(can_id, content);
        ios_.post(boost::bind(&SimpleCan::do_write, this, frame));
    }

    void do_write(can_frame_ptr frame) {
        bool is_writing = !frame_queue_.empty();
        frame_queue_.push_back(frame);
        if (!is_writing) {
//            print_frame(frame_queue_.front());
            /* share_ptr to void * */
            stream_.async_write_some(boost::asio::buffer(frame_queue_.front().get(),
                                                         sizeof(struct can_frame)),
                                     boost::bind(&SimpleCan::handle_write, shared_from_this()));
        }
    }

    void handle_write() {
        frame_queue_.pop_front();
        if (!frame_queue_.empty()) {
//            print_frame(frame_queue_.front());
            stream_.async_write_some(boost::asio::buffer(frame_queue_.front().get(),
                                                         sizeof(struct can_frame)),
                                     boost::bind(&SimpleCan::handle_write, shared_from_this()));
        }
    }

    can_frame_ptr buildFrame(uint32_t can_id, std::vector<uint8_t> &data) {
        auto frame = std::make_shared<struct can_frame>();
        frame->can_id = can_id;
        frame->can_dlc = data.size();
        int i = 0;
        while (i < data.size()) {
            frame->data[i] = data[i];
            i++;
        }
        return frame;
    }

    void startRecv() {
        std::cout << "start read!" << std::endl;
        stream_.async_read_some(boost::asio::buffer(&frame_recv_, sizeof(frame_recv_)),
                                boost::bind(&SimpleCan::handle_recv, shared_from_this(), boost::ref(stream_)));
    }

    void handle_recv(boost::asio::posix::basic_stream_descriptor<> &stream) {
        std::vector<uint8_t> data;
        if (frame_recv_.can_id & CAN_ERR_FLAG) {
            LOG(ERROR) << " error can frame!!!!!!!!!!!!!";
        } else if (frame_recv_.can_id & CAN_RTR_FLAG) {
            LOG(WARNING) << " request can frame!!!!!!!!!!!!!";
        } else {
            for (int i = 0; i < frame_recv_.can_dlc; i++) {
                data.push_back(frame_recv_.data[i]);
            }

            if (msg_callback_f_) {
                msg_callback_f_(frame_recv_.can_id, data);
            }
        }
        stream_.async_read_some(boost::asio::buffer(&frame_recv_, sizeof(frame_recv_)),
                                boost::bind(&SimpleCan::handle_recv, shared_from_this(), boost::ref(stream_)));
    }

    void Close() {
        close(can_raw_socket_);
    }

    static void print_frame(can_frame_ptr frame) {
        int i;
        std::cout << "CAN FRAME SENT: can_id " << std::setfill('0') << std::setw(2) << std::hex << (int) frame->can_id
                  << ": ";
        if (frame->can_id & CAN_RTR_FLAG) {
            std::cout << "remote request";
        } else {
            std::cout << std::hex << "[" << (int) frame->can_dlc << "]";
            for (i = 0; i < frame->can_dlc; i++)
                std::cout << " " << std::setfill('0') << std::setw(2) << std::hex << (int) frame->data[i];
        }
        std::cout << std::endl;
    }

    typedef boost::function<void(int, std::vector<uint8_t> &)> MessageCallbackFunc_t;

    void setMessageCallbackFunc(MessageCallbackFunc_t callback) {
        msg_callback_f_ = callback;
    }

private:
    int can_raw_socket_;

    struct sockaddr_can addr_;
    struct can_frame frame_recv_;
    struct ifreq ifr_;

    FrameQueue frame_queue_;

    boost::asio::io_service &ios_;
    boost::asio::posix::basic_stream_descriptor<> stream_;

    MessageCallbackFunc_t msg_callback_f_;
};

#endif    /* _SIMPLECAN_H */
