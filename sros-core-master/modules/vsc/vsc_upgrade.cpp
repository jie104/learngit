/**
 * 
 * @copyright   : Copyright (C) 2019 Standard-robots, Inc
 * @file        : vsc_upgrade.cpp
 * @description : 
 * @author      : EHL (linenhui@standard-robots.com / enhuilyn@qq.com)
 * @date        : 2022/05/12
 * @brief       : V1.0.0 
 */


#include "vsc_upgrade.h"

#include <cmath>
#include <thread>
#include <glog/logging.h>
#include <boost/thread.hpp>
#include <fstream>
#include <memory>

#include "core/exec_error.hpp"
#include "core/fault_center.h"
#include "core/msg/common_command_msg.hpp"
#include "core/msg/common_state_msg.hpp"
#include "core/msg/notification_msg.hpp"
#include "core/msg/sonar_data_msg.hpp"
#include "core/msg/usart_data_msg.hpp"
#include "core/settings.h"
#include "core/src.h"
#include "core/state.h"
#include "core/util/md5.h"


namespace fs = boost::filesystem;

using namespace std;
using namespace sros;
using namespace sros::core;
using namespace sros::device;

namespace vsc {

VSCUpgrade::VSCUpgrade()
    : command_msg_seq_no_(1) {
}

VSCUpgrade::~VSCUpgrade(){}

#define SRC_APP_ADDR 0x08008000
#define IRQ_TABLE_ADDR 0x08008000

uint32_t cal_crc(const uint32_t *ptr, uint32_t len) {
    uint32_t dwPolynomial = 0x04c11db7;
    uint32_t xbit;
    uint32_t data;
    uint32_t crc_register = 0xFFFFFFFF;  // init
    while (len--) {
        xbit = (uint32_t)(1 << 31);

        data = *ptr++;
        for (int bits = 0; bits < 32; bits++) {
            if (crc_register & 0x80000000) {
                crc_register <<= 1;
                crc_register ^= dwPolynomial;
            } else {
                crc_register <<= 1;
            }
            if (data & xbit) crc_register ^= dwPolynomial;

            xbit >>= 1;
        }
    }
    return crc_register;
}

static ssize_t readline(int fd, char *line) {
    ssize_t ret = 0;
    size_t count = 0;
    while ((ret = read(fd, line + count, 1)) > 0) {
        if (line[count] == '\n') {
            line[count] = '\0';
            break;
        }
        count++;
    }
    if (ret <= 0) return ret;
    return count;
}

bool VSCUpgrade::setUpgradeFilePath(const std::string &upgrade_bin_file_path) {
    is_in_upgrade_ = true;
    auto path = fs::path(upgrade_bin_file_path);
    if (fs::is_symlink(path)) {  // 传进来的可能是一个符号链接
        upgrade_bin_file_ = fs::read_symlink(path).string();
        LOG(INFO) << "upgrade_bin_file_path is symlink! " << upgrade_bin_file_path << " -> " << upgrade_bin_file_;
    } else {
        upgrade_bin_file_ = upgrade_bin_file_path;
    }
    upgrade_md5sum_file_ = upgrade_bin_file_ + "_md5sum.txt";

    if (isIapUpgradeFileReady()) {
        return true;
    }

    return false;
}


// 从db中读取所有src相关配置项，写入srtos的mc中
bool VSCUpgrade::syncSrcParamToMc(void) {
    auto &setting = Settings::getInstance();
    ItemInfoLists configs = setting.getItemInfoListOfClass("vsc");

    for (auto config : configs) {
        auto strSrcItem = config.key;
        auto strMcItem = strSrcItem.replace(0,4,"mc.");
        setting.setValue(strMcItem,config.value);
        // LOG(INFO) << "strSrcItem:" << strSrcItem << ", strMcItem:" << strMcItem << ", setValue:" << config.value;
    }
    LOGGER(INFO, SROS) <<"sync VSC parameter to mc";
    return  true;
}

void VSCUpgrade::setUpgradeResult(int result) {
    if (state_ != IAP_UTEST) {
        // handle this  message only in USER TEST mode
        return;
    }
    // handle upgrade result
    if (result == 1) {
        LOG(INFO) << " VSC upgrade success!";
        recordUpdate(true);
        backupUpgradeFiles();
        //syncSrcParamToMc();
    } else {
        LOG(INFO) << " VSC upgrade failed!";
        recordUpdate(false);
    }
}

void VSCUpgrade::handleRequestData(uint32_t index) {
    LOG(INFO) <<"handle request data!";
    std::vector<uint8_t> data;
    int block_size = 512;
    int last_read_bytes = 0;
    int i = 0;
    uint8_t buffer[512];
    if (index > 0 && state_ != IAP_DATA) {
        LOG(WARNING) << "invalid request! state_ = " << state_ << " index = " << index
                     << " sros is not in active-iap mode";
        return;
    }

    FILE *fp = fopen(upgrade_bin_file_.c_str(), "rb");

    if (!fp) {
        LOG(WARNING) << "stm32 update file not exists!";
        return;
    }
    fseek(fp, 0, SEEK_END);

    auto filesize = ftell(fp);
    if (index * block_size > filesize) {
        fclose(fp);
        LOG(WARNING) << "------------------- > invalid request !";
        return;
    }

    fseek(fp, index * block_size, SEEK_SET);

    if (!feof(fp)) {
        last_read_bytes = fread(buffer, sizeof(uint8_t), block_size, fp);
        LOG(INFO) << "[index = " << index << "] read " << last_read_bytes << " bytes from " << upgrade_bin_file_
                << " filesize: " << filesize << " progress: " << index << " / " << filesize / block_size;
    }

    if (last_read_bytes < block_size) {
        i = 0;
        while (i < 4 - (last_read_bytes % 4)) {
            buffer[last_read_bytes + i] = 0xff;
            ++i;
        }
        //这里可以不加'\0'，极端情况下如果读出511个字节，会越界崩溃的
        //buffer[last_read_bytes + i] = '\0';
        //LOG(INFO) << "[last_read_bytes + i = " << (last_read_bytes + i) << "]";
    }
    fclose(fp);

    last_read_bytes += (last_read_bytes % 4) ? (4 - (last_read_bytes % 4)) : 0;

    auto crc32 = cal_crc(reinterpret_cast<uint32_t *>(buffer), last_read_bytes / 4);
    // LOG(WARNING) <<"calculate crc for bytes = "<<last_read_bytes <<" crc = "<<std::hex<<(int)crc32;

    data.push_back(0xab);
    data.push_back(0xf2);
    data.push_back((uint8_t)(last_read_bytes + 8));
    data.push_back((uint8_t)((last_read_bytes + 8) >> 8));

    push_uint32t(data, index);

    push_uint32t(data, crc32);

    for (i = 0; i < last_read_bytes; i++) {
        data.push_back(buffer[i]);
    }
    usleep(200 * 1000);
    // printVector(data);
    sendIAPdata(data);
}

void VSCUpgrade::handleRequestTest() {
    LOG(INFO) << "iap request test (vsc upgrade send file finished!)";
    std::vector<uint8_t> ack;
    ack.push_back(0xab);
    ack.push_back(0xf4);
    ack.push_back(0x01);
    ack.push_back(0x00);
    ack.push_back(IAP_UTEST);
    bool ret = sendIAPdata(ack);
    LOG(INFO) << "new thread start " << is_in_upgrade_;
    if(is_in_upgrade_) {
        LOG(INFO) << "new thread sendUpgradeTest ...";
       boost::thread(boost::bind(&VSCUpgrade::sendUpgradeTest, this)); 
    }
}

void VSCUpgrade::handleRequestResult(uint32_t result) {
    LOG(INFO) << "handle request result = " << result;
    std::vector<uint8_t> ack;
    ack.push_back(0xab);
    if (!result) {
        LOG(WARNING) << "stm32_upgrade.bin format wrong!!";
        ack.push_back(0xf5);
    } else {
        ack.push_back(0xf4);
    }
    ack.push_back(0x01);
    ack.push_back(0x00);
    ack.push_back(IAP_RESULT);
    bool ret = sendIAPdata(ack);
    if (!ret) {
        LOG(ERROR) << "send request result failed!";
    }
}

void VSCUpgrade::checkUpgradeResult(uint32_t version) {
    LOG(INFO) << "result version = " << version;
    if(is_in_upgrade_) {
        if(version > 0) {
            setUpgradeResult(1);
            LOG(INFO) << "isInUpgrade " << isInUpgrade();
            if (!isInUpgrade()) {
                vsc_upgrade_callback_f_(1);
            }
        } else {
            setUpgradeResult(0);
            LOG(INFO) << "isInUpgrade "  << isInUpgrade();
            if (!isInUpgrade()) {
                vsc_upgrade_callback_f_(0);
            }
        }
    }
}

void VSCUpgrade::handleRequestConnection() {
    LOG(INFO) << "handle request connection!";
    std::vector<uint8_t> ack;
    ack.push_back(0xab);
    if (isIapUpgradeFileReady()) {
        ack.push_back(0xf4);
    } else {
        ack.push_back(0xf5);
    }
    ack.push_back(0x01);
    ack.push_back(0x00);
    ack.push_back(IAP_CONN);

    // printVector(ack);
    sendIAPdata(ack);
}

void VSCUpgrade::handleRequestInfo() {
    LOG(INFO) << "handle request info!";
    std::string version = "4.1.2";

    getFileVersion(upgrade_bin_file_, version, firmware_file_version_);
    LOG(INFO) << "bin_version int = " << firmware_file_version_ << " str = " << version;

    auto md5sum = readMD5String(upgrade_md5sum_file_);
    auto filesize = getFileSize(upgrade_bin_file_);
    filesize += (filesize % 4) ? (4 - (filesize % 4)) : 0;
    auto data = makeUpgradeInfo(version, md5sum, filesize, 0);

    //src v1不在发送md5校验值，v1/v2 统一发送32位CRC
    uint32_t crcvalue = makeUpgradeInfoCrc(data);

    LOG(INFO) << "handleRequestInfo filesize = " << filesize << " crcvalue = " << std::hex << crcvalue;
    // printVector(data);
    sendIAPdata(data);
}

uint32_t VSCUpgrade::makeUpgradeInfoCrc(std::vector<uint8_t> &data) {
    std::ifstream in(upgrade_bin_file_, std::ifstream::in | std::ifstream::binary);
    if (!in) {
        LOG(ERROR) << "open file error! filepath:" << upgrade_bin_file_;
        return false;
    }

    unsigned char file_buffer[1024];
    int read_block_size = 1024;
    std::streamsize length;

    uint32_t dwPolynomial = 0x04c11db7;
    uint32_t xbit;
    uint32_t xdata;
    uint32_t crc_register = 0xFFFFFFFF;  // init

    while (!in.eof()) {
        in.read(reinterpret_cast<char *>(file_buffer), read_block_size);
        length = in.gcount();
        if (length < read_block_size) {
            int i = 0;
            while (i < 4 - (length % 4)) {
                file_buffer[length + i] = 0xff;
                ++i;
            }
        }

        length += (length % 4) ? (4 - (length % 4)) : 0;
        //cal crc
        const uint32_t *ptr = reinterpret_cast<uint32_t *>(file_buffer);
        length = length / 4;

        while(length--) {
            xbit = (uint32_t)(1 << 31);

            xdata = *ptr++;
            for (int bits = 0; bits < 32; bits++) {
                if (crc_register & 0x80000000) {
                    crc_register <<= 1;
                    crc_register ^= dwPolynomial;
                } else {
                    crc_register <<= 1;
                }
                if (xdata & xbit) crc_register ^= dwPolynomial;

                xbit >>= 1;
            }
        }
    }//in.eof

    in.close();

    //添加到报文
    push_uint32t(data, crc_register);
    push_uint32t(data, 0);
    push_uint32t(data, 0);
    push_uint32t(data, 0);

    return crc_register;
}

void VSCUpgrade::sendUpgradeTest() {
    LOG(INFO) << "sleep 2s and send Upgrade test to VSC application!";
    boost::this_thread::sleep_for(boost::chrono::milliseconds(2000));
    LOG(INFO) << "Send Upgrade test to VSC application!";
    /* NOTE:
     * 注意 VSC 升级后重启的延时
     * */
    std::thread([&]() {

        for (auto i = 0; i < 10; ++i) {
            if (!is_in_upgrade_) {
                break;
            }
            // 主动请求VSC300版本信息
            vector<uint8_t> request_vsc_info = {0x00, 0x01};
            if(usart_ptr_) {
                LOG(INFO) << "send 00 01";
                usart_ptr_->sendData(request_vsc_info);
            }
            boost::this_thread::sleep_for(boost::chrono::milliseconds(1000));
        }
        LOG(INFO) << "check result time out error!!!";
        checkUpgradeResult(0);
    }).detach();
}

void VSCUpgrade::recordUpdate(bool is_update) {
    is_in_upgrade_ = false;
    char update_log[256];
    std::ofstream out;
    time_t timep;
    time(&timep);
    std::string time = asctime(gmtime(&timep));
    time[time.size() - 1] = ' ';
    if (is_update) {
        snprintf(update_log, update_info_.size() + time.size() + 2 + 10, "%s %s success\n", time.c_str(),
                 update_info_.c_str());
    } else {
        snprintf(update_log, update_info_.size() + time.size() + 2 + 7, "%s %s fail\n", time.c_str(),
                 update_info_.c_str());
    }

    out.open(IAP_UPDATE_LOG, ios::out | ios::app | ios::binary);
    if (!out.is_open()) {
        LOG(ERROR) << "can not open " << IAP_UPDATE_LOG;
        return;
    }
    out.write(reinterpret_cast<char *>(&update_log), strlen(update_log));
    out.close();
    LOG(INFO) << "recode log update to " << IAP_UPDATE_LOG << ": " << update_log;
}

void VSCUpgrade::backupUpgradeFiles() {
    LOG(INFO) << "start backup upgrade files.";
    if (fs::exists(fs::path(upgrade_bin_file_))) {
        if (!fs::exists(fs::path(VSC_BACKUP_PATH))) {
            fs::create_directory(fs::path(VSC_BACKUP_PATH));
        }

        auto src_bin_file = fs::path(upgrade_bin_file_);
        auto src_md5sum_file = fs::path(upgrade_md5sum_file_);
        auto dst_bin_file = fs::path(VSC_BACKUP_PATH) / src_bin_file.filename();
        auto dst_md5sum_file = fs::path(VSC_BACKUP_PATH) / src_md5sum_file.filename();

#if BOOST_VERSION >= 105800
        if (src_bin_file != dst_bin_file) {
            fs::copy_file(src_bin_file, dst_bin_file, fs::copy_option::overwrite_if_exists);
        }
        if (src_md5sum_file != dst_md5sum_file) {
            fs::copy_file(src_md5sum_file, dst_md5sum_file, fs::copy_option::overwrite_if_exists);
        }
        LOG(INFO) << "Copy " << src_bin_file << " -> " << dst_bin_file;
        LOG(INFO) << "Copy " << src_md5sum_file << " -> " << dst_md5sum_file;
#else
        std::ifstream bin_src(src_bin_file.string(), std::ios::binary);
        std::ofstream bin_dst(dst_bin_file.string(), std::ios::binary);
        bin_dst << bin_src.rdbuf();

        std::ifstream md5sum_src(src_md5sum_file.string(), std::ios::binary);
        std::ofstream md5sum_dst(dst_md5sum_file.string(), std::ios::binary);
        md5sum_dst << md5sum_src.rdbuf();
#endif

        fs::remove(default_vsc_upgrade_file_symlink);
        fs::create_symlink(dst_bin_file, default_vsc_upgrade_file_symlink);
        LOG(INFO) << "symlink " << default_vsc_upgrade_file_symlink << " -> " << dst_bin_file;
    }
}

void VSCUpgrade::push_uint32t(std::vector<uint8_t> &data, uint32_t value) {
    data.push_back((uint8_t)value);
    data.push_back((uint8_t)(value >> 8));
    data.push_back((uint8_t)(value >> 16));
    data.push_back((uint8_t)(value >> 24));
}

bool VSCUpgrade::isIapUpgradeFileReady() {
    namespace fs = boost::filesystem;

    if (upgrade_bin_file_ == "" || upgrade_md5sum_file_ == "") {
        LOG(WARNING) << "No vsc upgrade file provide, use default upgrade file instead. "
                     << default_vsc_upgrade_file_symlink.string();

        boost::system::error_code ec;
        if (fs::exists(default_vsc_upgrade_file_symlink, ec)) {
            auto default_upgrade_file = fs::read_symlink(default_vsc_upgrade_file_symlink);
            upgrade_bin_file_ = default_upgrade_file.string();
            upgrade_md5sum_file_ = upgrade_bin_file_ + "_md5sum.txt";

            LOG(INFO) << "Default vsc upgrade file: " << upgrade_bin_file_;
        } else {
            LOG(ERROR) << "No default vsc upgrade file can be found! " << ec.message();
            return false;
        }
    }

    if (fs::exists(fs::path(upgrade_bin_file_))) {
        auto md5sum = readMD5String(upgrade_md5sum_file_);
        if (md5sum.size() && checkMd5sum(upgrade_bin_file_, md5sum)) {
            LOG(INFO) << "vsc bin file is ready";
            return true;
        } else {
            LOG(ERROR) << "vsc md5sum check failed";
        }
    } else {
        LOG(ERROR) << "vsc bin file not exist: " << upgrade_bin_file_;
    }

    return false;
}

std::string VSCUpgrade::readMD5String(const string &path) {
    std::string value;
    char line[1024];

    LOG(INFO) << "upgrade_md5sum_file_: " << upgrade_md5sum_file_;
    int fd = open(upgrade_md5sum_file_.c_str(), O_RDONLY);
    if (fd == -1) {
        LOG(INFO) << "failed to open file " << upgrade_md5sum_file_;
        return value;
    }

    if (readline(fd, line) > 0) {
        value = strtok(line, " ");
        update_info_ = value + "   " + upgrade_bin_file_;
    } else {
        LOG(WARNING) << "read line error";
    }
    return value;
}

int VSCUpgrade::getFileSize(const string &path) {
    FILE *fp = fopen(path.c_str(), "rb");

    if (!fp) {
        return 0;
    }
    fseek(fp, 0, SEEK_END);
    auto filesize = ftell(fp);
    fclose(fp);

    return filesize;
}

int VSCUpgrade::getFileVersion(const string &path,  std::string &versionStr, uint32_t &versionNum) {
    std::string filename = path.substr(path.find_last_of('/'));
    std::string str = filename.substr(filename.find_first_of('_') + 2);
    int endPos = str.find_last_of('.');
    versionStr = str.substr(0, endPos);
    LOG(INFO) << "endPps = " << endPos << " str = " << str << "versionStr = " << versionStr;
    int version[3] = {0};
    int index = 0;
    for(int i = 0; i < endPos; i++)
    {
        // LOG(INFO) << "versionStr[i] = " << versionStr[i];
        if(versionStr[i] == '.') {
            index ++;
            if(index > 2) {
                break;
            }
        } else if(versionStr[i] >= '0' && versionStr[i] <= '9') {
            version[index] = version[index] * 10 + versionStr[i] - '0';
        } else {
            break;
        }
    }
    LOG(INFO) << "version[] = " << version[0] << "." << version[1] << "." << version[2];
    versionNum = version[0] * 1000 * 1000 + version[1] * 1000 + version[2];
    return versionNum;
}

std::vector<uint8_t> VSCUpgrade::makeUpgradeInfo(const std::string &version, const std::string &md5, int size,
                                                 uint32_t stamp) {
    std::vector<uint8_t> upgradeinfo;
    upgradeinfo.push_back(0xab);
    upgradeinfo.push_back(0xf1);
    upgradeinfo.push_back(0x24);
    upgradeinfo.push_back(0x00);

    auto s1 = splitWithStl(version, ".");
    int i = s1.size() - 1;
    while (i >= 0) {
        upgradeinfo.push_back(atoi(s1[i].c_str()));
        --i;
    }
    // reserved
    upgradeinfo.push_back(0x00);

    // #define SRC_APP_ADDR 0x08008000
    push_uint32t(upgradeinfo, (uint32_t)SRC_APP_ADDR);
    push_uint32t(upgradeinfo, (uint32_t)IRQ_TABLE_ADDR);

    push_uint32t(upgradeinfo, stamp);
    push_uint32t(upgradeinfo, size);

    //src v1不在发送md5校验值，v1/v2 统一发送32位CRC
    //hashstring2vector(md5, upgradeinfo);

    return upgradeinfo;
}

bool VSCUpgrade::checkMd5sum(const std::string &filePath, const std::string &md5sum) {
    std::ifstream in(filePath, std::ifstream::in | std::ifstream::binary);
    if (!in) {
        LOG(ERROR) << "open file error! filepath:" << filePath;
        return false;
    }

    MD5 md5;
    unsigned char file_buffer[1024];
    std::streamsize length;

    while (!in.eof()) {
        in.read(reinterpret_cast<char *>(file_buffer), 1024);
        length = in.gcount();
        if (length > 0) {
            md5.update(file_buffer, length);
        }
    }

    in.close();
    md5.finalize();
    bool ret = (md5.hexdigest() == md5sum);
    if (!ret) {
        LOG(ERROR) << "check md5 sum error!\n  Actual: " << md5.hexdigest() << "\nExpected: " << md5sum;
    }
    return ret;
}

std::vector<std::string> VSCUpgrade::splitWithStl(const std::string &str, const std::string &pattern) {
    std::vector<std::string> resVec;

    if ("" == str) {
        return resVec;
    }

    std::string strs = str + pattern;

    size_t pos = strs.find(pattern);
    size_t size = strs.size();

    while (pos != std::string::npos) {
        std::string x = strs.substr(0, pos);
        resVec.push_back(x);
        strs = strs.substr(pos + 1, size);
        pos = strs.find(pattern);
    }
    return resVec;
}

void VSCUpgrade::hashstring2vector(const std::string &str, std::vector<uint8_t> &data) {
    if (str.size() != 32) {
        LOG(WARNING) << "invalid hash string! size = " << str.size();
        return;
    }
    int i = str.size() - 1;
    while (i >= 0) {
        uint8_t s = charToInt(str[i]) + (charToInt(str[i - 1]) << 4);
        i -= 2;
        data.push_back(s);
    }
}

uint8_t VSCUpgrade::charToInt(char c) {
    if (c >= '0' && c <= '9') {
        return c - '0';
    } else if (c >= 'a' && c <= 'f') {
        return c - 'a' + 10;
    }
    return 0;
}

void VSCUpgrade::sendMsg(src::BaseMsg_ptr msg) {
    if (usart_ptr_ && msg) {
        msg->encode();
        std::vector<uint8_t> payload = msg->rawData();
        LOG(INFO) << "sendMsg msg->rawData() : " << std::hex << numberListToStr(payload.cbegin(), payload.cend());
        usart_ptr_->sendData(msg->rawData());
    } else {
        LOG(WARNING) << "sendMsg null";
    }
}

void VSCUpgrade::sendMsg(const std::vector<uint8_t> &data) {
    LOG(INFO) << "sendMsg data : " << std::hex << numberListToStr(data.cbegin(), data.cend());
    if (usart_ptr_) {
        usart_ptr_->sendData(data);
    }
}

void VSCUpgrade::sendCommandMsg(COMMAND_t command, int32_t param0, int32_t param1) {
    // if (src_sdk_proto_do_) {
    //     src_sdk_proto_do_->sendCommandMsgDo(command, param0, param1, 0, 0);
    // }
    LOG(INFO) << "@@@@@@@ COMMAND: command_id: " << command << ", p0 = " << param0 << ", p1 = " << param1
            << ", seq = " << command_msg_seq_no_;

    src::CommandMsg_ptr m = std::make_shared<src::CommandMsg>();
    m->setCommand(command);
    m->setParam0(param0);
    m->setParam1(param1);
    m->setParam2(0);
    m->setParam3(0);

    m->setSeqNO(command_msg_seq_no_);

    sendMsg(m);

    command_msg_seq_no_ += 1;  // 自增序列号
}

void VSCUpgrade::sendCommandMsg(COMMAND_t command) {
    
    if(COMMAND_UPGRADE_REQUEST == command) {
        std::vector<uint8_t> payload;
        payload.push_back(0x00);  // type
        payload.push_back(0x03);  // cmd
        if(usart_ptr_) {
           usart_ptr_->sendData(payload); 
        }
        LOG(INFO) << "sendData : " << std::hex << numberListToStr(payload.cbegin(), payload.cend());
    } else {
       sendCommandMsg(command, 0, 0);
    }
}

bool VSCUpgrade::sendIAPdata(const std::vector<uint8_t> &data) {
    if(data.size() < 32) {
       LOG(INFO) << "sendIAPdata : " << std::hex << numberListToStr(data.cbegin(), data.cend()); 
    }
    if (data.size()) {
        if(usart_ptr_) {
            return usart_ptr_->sendData(data);
        }
    }
    return false;
}

bool VSCUpgrade::upgradeRequest(const std::string &upgrade_bin_file_path) {
    if (!setUpgradeFilePath(upgrade_bin_file_path)) {
        return false;
    }
    sendCommandMsg(COMMAND_UPGRADE_REQUEST);
    return true;
}


void VSCUpgrade::onIAPRequest(const std::vector<uint8_t> &data) {
    if (data.size() < 4) {
        LOG(WARNING) << "invalid IAP data: size is " << data.size();
        return;
    }
    state_ = (Vsc300State)data[1];
    auto command = data[1];
    LOG(INFO) << "VSCUpgrade recv command: 0x" << std::hex << (int)command;

    LOG(INFO) << "sleep 10ms ... send";
    boost::this_thread::sleep_for(boost::chrono::milliseconds(10));

    if (command == IAP_CONN) {
        handleRequestConnection();
    } else if (command == IAP_REQUEST) {
        is_in_upgrade_ = true;

        try {
            LOG(ERROR) << "IAP_REQUEST";

            boost::thread(boost::bind(&VSCUpgrade::handleRequestInfo, this));
            LOG(ERROR) << "thread";

        } catch (std::system_error &e) {
            LOG(ERROR) << e.what();
        } catch (...) {
            LOG(ERROR) << "unkown exception!";
        }
    } else if (command == IAP_DATA) {
        uint32_t index = data[4] + (data[5] << 8) + (data[6] << 16) + (data[7] << 24);
        handleRequestData(index);
    } else if (command == IAP_RESULT) {
        handleRequestResult(data[4]);
    } else if (command == IAP_UTEST) {
        // test only
        handleRequestTest();
    } else if (command == IAP_DISCON) {
        LOG(INFO) << "iap request disconnect";
    }
}

void VSCUpgrade::handleIAPRequest(const std::vector<uint8_t> &data) {
    LOG(INFO) << "Recv Usart Data: " << std::hex << numberListToStr(data.cbegin(), data.cend());
    onIAPRequest(data);
}

void VSCUpgrade::upgradeTest() {
    std::vector<uint8_t> ack;
    ack.push_back(0xab);
    ack.push_back(0xf4);
    ack.push_back(0x01);
    ack.push_back(0x00);
    ack.push_back(IAP_UTEST);
    sendIAPdata(ack);
}

void VSCUpgrade::upgradSrcTest() {
    src::CommandMsg_ptr m = std::make_shared<src::CommandMsg>();
    m->setCommand(COMMAND_UPGRADE_TEST);
    m->setParam0(0);
    m->setParam1(0);
    m->setParam2(0);
    m->setParam3(0);

    m->setSeqNO(command_msg_seq_no_);

    // src_sdk->sendMsg(m);
    sendMsg(m);

    command_msg_seq_no_ += 1;  // 自增序列号
}

} // namespace vsc
