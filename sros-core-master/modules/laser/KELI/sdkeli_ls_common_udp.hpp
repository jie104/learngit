#ifndef SDKELI_LS_COMMON_UDP__
#define SDKELI_LS_COMMON_UDP__

#include <netinet/in.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "sdkeli_ls_common.hpp"

#include <algorithm>
#include <boost/asio.hpp>
#include <boost/lambda/lambda.hpp>
#include <boost/lexical_cast.hpp>
#include <iterator>

namespace sdkeli_lspdm_udp {
class CSDKeliLsCommonUdp : public CSDKeliLsCommon {
 public:
    CSDKeliLsCommonUdp(const std::string &host_name, const std::string &port, int &timelimit,
                       CSDKeliLs1207DEParser *parser)
        : CSDKeliLsCommon(parser),  //初始化缓存区域大小，关于参数动态调整、scan发布等publisher成员
          mHostName(host_name),
          mPort(port),
          mTimeLimit(timelimit) {}
    virtual ~CSDKeliLsCommonUdp() {
        if (mSocket != -1) {  //避免重复close
            StopScanner();    //向设备发消息
            CloseDevice();    //关闭socket
        }
    }
    virtual int CloseDevice() {
        if (mSocket != -1) {
            close(mSocket);
            mSocket = -1;
            LOG(INFO) << "close socket and CloseDevice";
            //            ROS_ERROR("close socket and CloseDevice");
        }
    }

 protected:
    /*Override functions*/
    virtual int InitDevice() {
        int opt = 1;
        struct sockaddr_in cliAddr;
        struct hostent *h;
        mSocket = -1;
        mSocket = socket(AF_INET, SOCK_DGRAM, 0);
        if (mSocket < 0) {
            //            ROS_ERROR("create udp socket failed");
            return ExitError;
        }

        h = gethostbyname(mHostName.c_str());
        if (h == NULL) {
            //            ROS_ERROR("unknown host '%s' /n", mHostName.c_str());
            return ExitError;
        }
        //        ROS_INFO("sending data to '%s' (IP : %s) (PORT : %s)", h->h_name, inet_ntoa(*(struct in_addr
        //        *)h->h_addr_list[0]), mPort.c_str());
        remoteServAddr.sin_family = h->h_addrtype;
        memcpy((char *)&remoteServAddr.sin_addr.s_addr, h->h_addr_list[0], h->h_length);
        remoteServAddr.sin_port = htons(atoi(mPort.c_str()));
        /*
        bzero(&addr, sizeof(addr));
        addr.sin_family = AF_INET;
        addr.sin_addr.s_addr = inet_addr(mHostName.c_str());
        addr.sin_port = htons(std::stoi(mPort));
        */

        bzero(&cliAddr, sizeof(cliAddr));
        cliAddr.sin_family = AF_INET;
        cliAddr.sin_addr.s_addr = INADDR_ANY;
        // cliAddr.sin_port = htons(atoi(mPort.c_str()));
        cliAddr.sin_port = 0;
        setsockopt(mSocket, SOL_SOCKET, SO_REUSEADDR, (char *)&opt, sizeof(opt));
        if (bind(mSocket, (struct sockaddr *)&cliAddr, sizeof(cliAddr)) < 0) {
            //            ROS_ERROR("bind socket failed");
            close(mSocket);
            mSocket = -1;
            return ExitError;
        }

        //        ClearConnectFlag();

        return ExitSuccess;
    }  //重写了的

    virtual int SendDeviceReq(const unsigned char *req, std::vector<unsigned char> *resp) {
        if (mSocket == -1) {
            //            ROS_ERROR("SendDeviceReq: Socket NOT open");
            //        mDiagUpdater.broadcast(diagnostic_msgs::DiagnosticStatus::ERROR,
            //                               "SDKELI_LS - SendDeviceReq: socket NOT open!");

            return ExitError;
        }

        if (SendUdpData2Device((char *)req, SIZE_OF_CMD) != SIZE_OF_CMD) {
            //            std::cout << "req: " << char(*req) << std::endl;
            //            ROS_ERROR("Write error for req command");
            //        mDiagUpdater.broadcast(diagnostic_msgs::DiagnosticStatus::ERROR,
            //                               "SDKELI_LS - SendDeviceReq: Write command failed!");

            return ExitError;
        }
        return ExitSuccess;
    }

    virtual int GetDataGram(unsigned char *receiveBuffer, int bufferSize, int *length) {
        int len;
        struct timeval tv;
        socklen_t addrlen;
        fd_set rfds;
        struct sockaddr_in recvAddr;
        if (mSocket == -1) {
            //            ROS_ERROR("GetDataGram: Socket NOT open");
            //        mDiagUpdater.broadcast(diagnostic_msgs::DiagnosticStatus::ERROR,
            //                               "SDKELI_LS - GetDataGram: socket NOT open!");
            LOG(ERROR) << "socket not open!";
            return ExitError;
        }
        addrlen = sizeof(recvAddr);

        if (1)  //(!stream_stopped_)
        {
            tv.tv_sec = mTimeLimit;
            tv.tv_usec = 0;
            FD_ZERO(&rfds);
            FD_SET(mSocket, &rfds);
            if (select(mSocket + 1, &rfds, NULL, NULL, &tv) > 0) {
                *length = recvfrom(mSocket, receiveBuffer, bufferSize, 0, (struct sockaddr *)&recvAddr, &addrlen);
                if (len > 0) {
                    //    printf("echo from %s:UDP%u/n", inet_ntoa(recvAddr.sin_addr),ntohs(recvAddr.sin_port));
                }
            } else {
                std::string errorStr = "SDKELI_LS - GetDataGram: No full response for read after " + mTimeLimit;
                errorStr = errorStr + "S";
                //            mDiagUpdater.broadcast(diagnostic_msgs::DiagnosticStatus::ERROR, errorStr);
                //                ROS_WARN("GetDataGram timeout for %ds", mTimeLimit);
                LOG(ERROR) << errorStr << ",time out for" << mTimeLimit << "s";
                return ExitError;
            }
        }

        return ExitSuccess;
    }

    int SendUdpData2Device(char *buf, int length) {
        int n = -1;
        if (mSocket > 0) {
            n = sendto(mSocket, buf, length, 0, (struct sockaddr *)&remoteServAddr, sizeof(remoteServAddr));
        }
        return n;
    }

 private:
    int mSocket;
    size_t mBytesReceived;
    std::string mHostName;
    std::string mPort;
    int mTimeLimit;
    struct sockaddr_in remoteServAddr;
};
typedef boost::shared_ptr<CSDKeliLsCommon> CSDKeliLsCommonUdp_ptr;
// using CSDKeliLsCommonUdp_ptr = boost::shared_ptr<CSDKeliLsCommon>;
} /*namespace sdkeli_lspdm_udp*/

#endif /*SDKELI_LS_COMMON_TCP__*/
