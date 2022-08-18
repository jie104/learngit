/*
 * LMS1xx.cpp
 *
 *  Created on: 09-08-2010
 *  Author: Konrad Banachowicz
 ***************************************************************************
 *   This library is free software; you can redistribute it and/or         *
 *   modify it under the terms of the GNU Lesser General Public            *
 *   License as published by the Free Software Foundation; either          *
 *   version 2.1 of the License, or (at your option) any later version.    *
 *                                                                         *
 *   This library is distributed in the hope that it will be useful,       *
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of        *
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU     *
 *   Lesser General Public License for more details.                       *
 *                                                                         *
 *   You should have received a copy of the GNU Lesser General Public      *
 *   License along with this library; if not, write to the Free Software   *
 *   Foundation, Inc., 59 Temple Place,                                    *
 *   Suite 330, Boston, MA  02111-1307  USA                                *
 *                                                                         *
 ***************************************************************************/

#include <sys/time.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <errno.h>
#include <fcntl.h>
#include <unistd.h>
#include <glog/logging.h>

#include "LMS5xx.h"

namespace lms5xx {


LMS5xx::LMS5xx() : connected_(false) {
}

LMS5xx::~LMS5xx() {
}

void LMS5xx::connect(std::string host, int port) {
    if (!connected_) {
        socket_fd_ = socket(PF_INET, SOCK_STREAM, IPPROTO_TCP);
        if (socket_fd_) {
            struct sockaddr_in stSockAddr;
            stSockAddr.sin_family = PF_INET;
            stSockAddr.sin_port = htons(port);
            inet_pton(AF_INET, host.c_str(), &stSockAddr.sin_addr);

            int ret = ::connect(socket_fd_, (struct sockaddr *) &stSockAddr, sizeof(stSockAddr));

            if (ret == 0) {
                connected_ = true;
            }
        }
    }
}

void LMS5xx::disconnect() {
    if (connected_) {
        close(socket_fd_);
        connected_ = false;
    }
}

bool LMS5xx::isConnected() {
    return connected_;
}

void LMS5xx::startMeas() {
    char buf[100];
    sprintf(buf, "%c%s%c", 0x02, "sMN LMCstartmeas", 0x03);

    write(socket_fd_, buf, strlen(buf));

    int len = read(socket_fd_, buf, 100);
    if (buf[0] != 0x02)
        LOG(INFO) << "invalid packet recieved";
    buf[len] = 0;
}

void LMS5xx::stopMeas() {
    char buf[100];
    sprintf(buf, "%c%s%c", 0x02, "sMN LMCstopmeas", 0x03);

    write(socket_fd_, buf, strlen(buf));

    int len = read(socket_fd_, buf, 100);
    if (buf[0] != 0x02)
        LOG(INFO) << "invalid packet recieved";
    buf[len] = 0;
}

status_t LMS5xx::queryStatus() {
    char buf[100];
    sprintf(buf, "%c%s%c", 0x02, "sRN STlms", 0x03);

    write(socket_fd_, buf, strlen(buf));

    int len = read(socket_fd_, buf, 100);
    if (buf[0] != 0x02)
        LOG(INFO) << "invalid packet recieved";
    buf[len] = 0;

    int ret;
    sscanf((buf + 10), "%d", &ret);
    for (int i = 0; i < 12; ++i) {
        printf("%c", buf[i]);

    }
    printf("\n");

    return (status_t) ret;
}

void LMS5xx::login() {
    char buf[100];
    int result;
    sprintf(buf, "%c%s%c", 0x02, "sMN SetAccessMode 03 F4724744", 0x03);

    fd_set readset;
    struct timeval timeout;


    do   //loop until data is available to read
    {
        timeout.tv_sec = 1;
        timeout.tv_usec = 0;

        write(socket_fd_, buf, strlen(buf));

        FD_ZERO(&readset);
        FD_SET(socket_fd_, &readset);
        result = select(socket_fd_ + 1, &readset, NULL, NULL, &timeout);

    }
    while (result <= 0);

    int len = read(socket_fd_, buf, 100);
    if (buf[0] != 0x02)
        LOG(INFO) << "invalid packet recieved";
    buf[len] = 0;
}

scanCfg LMS5xx::getScanCfg() const {
    scanCfg cfg;
    char buf[100];
    sprintf(buf, "%c%s%c", 0x02, "sRN LMPscancfg", 0x03);

    write(socket_fd_, buf, strlen(buf));

    int len = read(socket_fd_, buf, 100);
    if (buf[0] != 0x02)
        LOG(INFO) << "invalid packet recieved";
    buf[len] = 0;

    sscanf(buf + 1, "%*s %*s %X %*d %X %X %X", &cfg.scaningFrequency,
           &cfg.angleResolution, &cfg.startAngle, &cfg.stopAngle);
    return cfg;
}

void LMS5xx::setScanCfg(const scanCfg &cfg) {
    char buf[100];
    sprintf(buf, "%c%s %X +1 %X %X %X%c", 0x02, "sMN mLMPsetscancfg",
            cfg.scaningFrequency, cfg.angleResolution, cfg.startAngle,
            cfg.stopAngle, 0x03);

    write(socket_fd_, buf, strlen(buf));

    int len = read(socket_fd_, buf, 100);

    buf[len - 1] = 0;
}

void LMS5xx::setScanDataCfg(const scanDataCfg &cfg) {
    char buf[100];
    sprintf(buf, "%c%s 0 0 1 0 0 0 0 0 0 0 0 +1%c", 0x02,  "sWN LMDscandatacfg",0x03);

//    sprintf(buf, "%c%s 0 0 %d %d 0 0 0 0 0 0 0 +1%c", 0x02,
//            "sWN LMDscandatacfg", cfg.remission ? 1 : 0,
//            cfg.resolution, 0x03);


    write(socket_fd_, buf, strlen(buf));
    int len = read(socket_fd_, buf, 100);
//    buf[len - 1] = 0;
    for (int i = 0; i < len; ++i) {
        printf("%c", buf[i]);

    }
    printf("\n");
}

scanOutputRange LMS5xx::getScanOutputRange() const {
    scanOutputRange outputRange;
    char buf[100];
    sprintf(buf, "%c%s%c", 0x02, "sRN LMPoutputRange", 0x03);

    write(socket_fd_, buf, strlen(buf));

    read(socket_fd_, buf, 100);

    sscanf(buf + 1, "%*s %*s %*d %X %X %X", &outputRange.angleResolution,
           &outputRange.startAngle, &outputRange.stopAngle);
    return outputRange;
}

void LMS5xx::scanContinous(int start) {
    char buf[100];
    sprintf(buf, "%c%s %d%c", 0x02, "sEN LMDscandata", start, 0x03);

    write(socket_fd_, buf, strlen(buf));

    int len = read(socket_fd_, buf, 100);

    if (buf[0] != 0x02)
        LOG(INFO) << "invalid packet recieved";

    buf[len] = 0;


    for (int i = 0; i < 12; ++i) {
        printf("%c", buf[i]);

    }
    printf("\n");
}

bool LMS5xx::getScanData(scanData *scan_data) {
    fd_set rfds;
    FD_ZERO(&rfds);
    FD_SET(socket_fd_, &rfds);

    // Block a total of up to 100ms waiting for more data from the laser.
    int parse_err_count = 0;
    int parse_err_count_thresh = 2;

    while (1) {
        parse_err_count = 0;
        // Would be great to depend on linux's behaviour of updating the timeval, but unfortunately
        // that's non-POSIX (doesn't work on OS X, for example).
        struct timeval tv;
        tv.tv_sec = 1;
        tv.tv_usec = 500000;

        int retval = select(socket_fd_ + 1, &rfds, NULL, NULL, &tv);


        if (retval != -1) {

            if (!buffer_.readFrom(socket_fd_)) {
                LOG(INFO) << "err to read!";
                return false;
            }
            // Will return pointer if a complete message exists in the buffer,
            // otherwise will return null.
            char *buffer_data = buffer_.getNextBuffer();

            if (buffer_data) {
                if (buffer_data[0] != 2) {
                    LOG(INFO) << "err to get the start!";
                    return false;
                }
                if (!parseScanData(buffer_data, scan_data)) {
                    parse_err_count++;
                    if (parse_err_count >= parse_err_count_thresh) {//连续两次parse出错,则返回错误,重新连接.
                        LOG(INFO) << "err to parse the scan data! will return false!";
                        return false;
                    }else {
                        LOG(INFO) << "will clear the buffer and reparse the scan!";
                        buffer_.popLastBuffer();
                        continue;
                    }
                }
                buffer_.popLastBuffer();
                return true;
            }
//            LOG(INFO) << "the buffer is not full!";
        }
        else {
            // Select timed out or there was an fd error.
            LOG(INFO) << "will continue to read***";
//      sleep(1);
            return false;
//      readTillEnd();
//      continue;
        }
    }
}


bool LMS5xx::parseScanData(char *buffer, scanData *data) {
    char *tok = strtok(buffer, " "); //Type of command
    tok = strtok(NULL, " "); //Command
    tok = strtok(NULL, " "); //VersionNumber
    tok = strtok(NULL, " "); //DeviceNumber
    tok = strtok(NULL, " "); //Serial number
    tok = strtok(NULL, " "); //DeviceStatus
    tok = strtok(NULL, " "); //MessageCounter
    tok = strtok(NULL, " "); //ScanCounter
    tok = strtok(NULL, " "); //PowerUpDuration
    tok = strtok(NULL, " "); //TransmissionDuration
    tok = strtok(NULL, " "); //InputStatus
    tok = strtok(NULL, " "); //OutputStatus
    tok = strtok(NULL, " "); //ReservedByteA
    tok = strtok(NULL, " "); //ScanningFrequency
    tok = strtok(NULL, " "); //MeasurementFrequency
    tok = strtok(NULL, " ");
    tok = strtok(NULL, " ");
    tok = strtok(NULL, " ");
    tok = strtok(NULL, " "); //NumberEncoders

    int NumberEncoders;
    if (!tok) {
        LOG(INFO) << "err to toke the 511 data!";
        return false;
    }
    sscanf(tok, "%d", &NumberEncoders);
    for (int i = 0; i < NumberEncoders; i++) {
        tok = strtok(NULL, " "); //EncoderPosition
        tok = strtok(NULL, " "); //EncoderSpeed
    }

    tok = strtok(NULL, " "); //NumberChannels16Bit
    int NumberChannels16Bit;
    sscanf(tok, "%d", &NumberChannels16Bit);

    for (int i = 0; i < NumberChannels16Bit; i++) {
        int type = -1; // 0 DIST1 1 DIST2 2 RSSI1 3 RSSI2
        char content[6];
        tok = strtok(NULL, " "); //MeasuredDataContent
        sscanf(tok, "%s", content);
        if (!strcmp(content, "DIST1")) {
            type = 0;
        }
        else if (!strcmp(content, "DIST2")) {
            type = 1;
        }
        else if (!strcmp(content, "DIST3")) {
            type = 2;
        }
        else if (!strcmp(content, "DIST4")) {
            type = 3;
        }
        else if (!strcmp(content, "DIST5")) {
            type = 4;
        }else if(!strcmp(content, "RSSI1")){
            type = 5;
        }else if(!strcmp(content, "RSSI2")){
            type = 6;
        }else if(!strcmp(content, "RSSI3")) {
            type = 7;
        }else if(!strcmp(content, "RSSI4")) {
            type = 8;
        }else if(!strcmp(content, "RSSI5")) {
            type = 9;
        }

        tok = strtok(NULL, " "); //ScalingFactor
        tok = strtok(NULL, " "); //ScalingOffset
        tok = strtok(NULL, " "); //Starting angle
        tok = strtok(NULL, " "); //Angular step width
        tok = strtok(NULL, " "); //NumberData
        int NumberData;

        sscanf(tok, "%X", &NumberData);
        if (NumberData != scan_number_check) {
            if (type == 0 || type == 5) {
                LOG(INFO) << "the check is wrong!";
                LOG(INFO) << "the check is:" << scan_number_check;
                LOG(INFO) << "the check is:" << NumberData;
                LOG(INFO) << "the content is:" << std::string(content);
                return false;
            }else {
                if (NumberData > scan_number_check) {
                    return false;
                }
                if (NumberData < 0) {
                    return false;
                }
                LOG(INFO) << "the check is:" << NumberData;
                LOG(INFO) << "the content is:" << std::string(content);
            }
        }
//    printf("NumberData : %d,type is: %d\n", NumberData, type);
        if (type == 0) {
            data->dist_len1 = NumberData;
        }
        else if (type == 1) {
            data->dist_len2 = NumberData;
        }
        else if (type == 2) {
            data->dist_len3 = NumberData;
        }
        else if (type == 3) {
            data->dist_len4 = NumberData;
        }

        else if (type == 4) {
            data->dist_len5 = NumberData;
        }

        else if (type == 5) {
            data->rssi_len1 = NumberData;
        }

        else if (type == 6) {
            data->rssi_len2 = NumberData;
        }

        else if (type == 7) {
            data->rssi_len3 = NumberData;
        }

        else if (type == 8) {
            data->rssi_len4 = NumberData;
        }

        else if (type == 9) {
            data->rssi_len5 = NumberData;
        }

        for (int i = 0; i < NumberData; i++) {
            int dat;
            tok = strtok(NULL, " "); //data
            sscanf(tok, "%X", &dat);

            if (type == 0) {
                data->dist1[i] = dat;
            }
            else if (type == 1) {
                data->dist2[i] = dat;
            }
            else if (type == 2) {
                data->dist3[i] = dat;
            }
            else if (type == 3) {
                data->dist4[i] = dat;
            }
            else if (type == 4) {
                data->dist5[i] = dat;
            }
            else if (type == 5) {
                data->rssi1[i] = dat;
            }
            else if (type == 6) {
                data->rssi2[i] = dat;
            }
            else if (type == 7) {
                data->rssi3[i] = dat;
            }
            else if (type == 8) {
                data->rssi4[i] = dat;
            }
            else if (type == 9) {
                data->rssi5[i] = dat;
            }
        }
    }

    tok = strtok(NULL, " "); //NumberChannels8Bit
    int NumberChannels8Bit;
    sscanf(tok, "%d", &NumberChannels8Bit);

    for (int i = 0; i < NumberChannels8Bit; i++) {
        int type = -1;
        char content[6];
        tok = strtok(NULL, " "); //MeasuredDataContent
        sscanf(tok, "%s", content);
        if (!strcmp(content, "DIST1")) {
            type = 0;
        }
        else if (!strcmp(content, "DIST2")) {
            type = 1;
        }
        else if (!strcmp(content, "DIST3")) {
            type = 2;
        }
        else if (!strcmp(content, "DIST4")) {
            type = 3;
        }
        else if (!strcmp(content, "DIST5")) {
            type = 4;
        }else if(!strcmp(content, "RSSI1")){
            type = 5;
        }else if(!strcmp(content, "RSSI2")){
            type = 6;
        }else if(!strcmp(content, "RSSI3")) {
            type = 7;
        }else if(!strcmp(content, "RSSI4")) {
            type = 8;
        }else if(!strcmp(content, "RSSI5")) {
            type = 9;
        }
        tok = strtok(NULL, " "); //ScalingFactor
        tok = strtok(NULL, " "); //ScalingOffset
        tok = strtok(NULL, " "); //Starting angle
        tok = strtok(NULL, " "); //Angular step width
        tok = strtok(NULL, " "); //NumberData
        int NumberData;

        sscanf(tok, "%X", &NumberData);
        if (NumberData != scan_number_check) {
            if (type == 0 || type == 5) {
                LOG(INFO) << "the check is wrong!";
                LOG(INFO) << "the check is:" << scan_number_check;
                LOG(INFO) << "the check is:" << NumberData;
                LOG(INFO) << "the content is:" << std::string(content);
                return false;
            }else{
                if (NumberData > scan_number_check) {
                    return false;
                }
                if (NumberData < 0) {
                    return false;
                }

                LOG(INFO) << "the check is:" << NumberData;
                LOG(INFO) << "the content is:" << std::string(content);
            }
        }
        if (type == 0) {
            data->dist_len1 = NumberData;
        }
        else if (type == 1) {
            data->dist_len2 = NumberData;
        }
        else if (type == 2) {
            data->dist_len3 = NumberData;
        }
        else if (type == 3) {
            data->dist_len4 = NumberData;
        }

        else if (type == 4) {
            data->dist_len5 = NumberData;
        }

        else if (type == 5) {
            data->rssi_len1 = NumberData;
        }

        else if (type == 6) {
            data->rssi_len2 = NumberData;
        }

        else if (type == 7) {
            data->rssi_len3 = NumberData;
        }

        else if (type == 8) {
            data->rssi_len4 = NumberData;
        }

        else if (type == 9) {
            data->rssi_len5 = NumberData;
        }
        for (int i = 0; i < NumberData; i++) {
            int dat;
            tok = strtok(NULL, " "); //data
            sscanf(tok, "%X", &dat);


            if (type == 0) {
                data->dist1[i] = dat;
            }
            else if (type == 1) {
                data->dist2[i] = dat;
            }
            else if (type == 2) {
                data->dist3[i] = dat;
            }
            else if (type == 3) {
                data->dist4[i] = dat;
            }
            else if (type == 4) {
                data->dist5[i] = dat;
            }
            else if (type == 5) {
                data->rssi1[i] = dat;
            }
            else if (type == 6) {
                data->rssi2[i] = dat;
            }
            else if (type == 7) {
                data->rssi3[i] = dat;
            }
            else if (type == 8) {
                data->rssi4[i] = dat;
            }
            else if (type == 9) {
                data->rssi5[i] = dat;
            }
        }
    }
    return true;
}

void LMS5xx::saveConfig() {
    char buf[100];
    sprintf(buf, "%c%s%c", 0x02, "sMN mEEwriteall", 0x03);

    write(socket_fd_, buf, strlen(buf));

    int len = read(socket_fd_, buf, 100);

    if (buf[0] != 0x02)
        LOG(INFO) << "invalid packet recieved";
    buf[len] = 0;
}

void LMS5xx::startDevice() {
    char buf[100];
    sprintf(buf, "%c%s%c", 0x02, "sMN Run", 0x03);

    write(socket_fd_, buf, strlen(buf));

    int len = read(socket_fd_, buf, 100);

    if (buf[0] != 0x02)
        LOG(INFO) << "invalid packet recieved";
    buf[len] = 0;
}

void LMS5xx::setScanNumberCheck(int number_check) {
    scan_number_check = number_check;
}

bool LMS5xx::readTillEnd() {
    char buf[1];
    bool is_end = false;
    int read_count = 0;
    while (!is_end) {
        read(socket_fd_, buf, 1);
        read_count++;
        if (read_count > 10000) {
            printf("error to get the end will return false");
            return false;
        }

        if (buf[0] == 48) {
            read(socket_fd_, buf, 1);
            if (buf[0] == LMS_ETX) {
                is_end = true;
                return true;
            }
        }
    }
}

void LMS5xx::setEchoCfg(const ScanEchoCfg &cfg) {
    char buf[100];
    sprintf(buf, "%c%s %d%c", 0x02,
            "sWN FREchoFilter", cfg.echo_cfg, 0x03);
    write(socket_fd_, buf, strlen(buf));
    int len = read(socket_fd_, buf, 100);
//    buf[len - 1] = 0;
    for (int i = 0; i < len; ++i) {
        printf("%c", buf[i]);

    }
    printf("\n");
}
}
