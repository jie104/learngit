/**
 * describe:
 * Created by pengjiali on 18-12-13.
 **/

#include "src_upgrade.h"
#include <glog/logging.h>
#include <boost/thread.hpp>
#include <fstream>
#include <memory>
#include <thread>
#include "core/src.h"
#include "core/util/md5.h"
#include "core/settings.h"


namespace fs = boost::filesystem;
using namespace sros::core;

namespace sdk {

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

bool SrcUpgrade::setUpgradeFilePath(const std::string &upgrade_bin_file_path) {
    is_in_upgrade_ = true;
    auto path = fs::path(upgrade_bin_file_path);
    if (fs::is_symlink(path)) {  // 传进来的可能是一个符号链接
        upgrade_bin_file_ = fs::read_symlink(path).string();
        LOG(INFO) << "upgrade_bin_file_path is symlink! " << upgrade_bin_file_path << " -> " << upgrade_bin_file_;
    } else {
        upgrade_bin_file_ = upgrade_bin_file_path;
    }
    upgrade_md5sum_file_ = upgrade_bin_file_ + "_md5sum.txt";

    if (isStm32UpgradeFileReady()) {
        return true;
    }

    return false;
}


// 从db中读取所有src相关配置项，写入srtos的mc中
bool SrcUpgrade::syncSrcParamToMc(void) {
    auto &setting = Settings::getInstance();
    ItemInfoLists configs = setting.getItemInfoListOfClass("src");

    for (auto config : configs) {
        auto strSrcItem = config.key;
        auto strMcItem = strSrcItem.replace(0,4,"mc.");
        setting.setValue(strMcItem,config.value);
        // LOG(INFO) << "strSrcItem:" << strSrcItem << ", strMcItem:" << strMcItem << ", setValue:" << config.value;
    }
    LOGGER(INFO, SROS) <<"sync SRC parameter to mc";
    return  true;
}

void SrcUpgrade::setUpgradeResult(int result) {
    if (state_ != IAP_UTEST) {
        // handle this  message only in USER TEST mode
        return;
    }
    // handle upgrade result
    if (result == 1) {
        LOG(INFO) << " SRC upgrade success!";
        recordUpdate(true);
        backupUpgradeFiles();
        //syncSrcParamToMc();
    } else {
        LOG(INFO) << " SRC upgrade failed!";
        recordUpdate(false);
    }
}

void SrcUpgrade::onIAPRequest(const std::vector<uint8_t> &data) {
    if (data.size() < 4) {
        LOG(WARNING) << "invalid IAP data: size is " << data.size();
        return;
    }

    state_ = (Stm32State)data[1];

    auto command = data[1];

    LOG(INFO) << "SrcUpgrade command: " << (int)command;

    if (command == IAP_CONN) {
        handleRequestConnection();
    } else if (command == IAP_REQUEST) {
        is_in_upgrade_ = true;

        try {
            LOG(ERROR) << "IAP_REQUEST";

            boost::thread(boost::bind(&SrcUpgrade::handleRequestInfo, this));
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

void SrcUpgrade::handleRequestData(uint32_t index) {
    // LOG(INFO) <<"handle request data!";
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
                  << " at " << index * block_size;
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
    // printVector(data);
    src_sdk->sendIAPdata(data);
}

void SrcUpgrade::handleRequestTest() {
    LOG(INFO) << "iap request test (src upgrade finished!)";
    std::vector<uint8_t> ack;
    ack.push_back(0xab);
    ack.push_back(0xf4);
    ack.push_back(0x01);
    ack.push_back(0x00);
    ack.push_back(IAP_UTEST);
    src_sdk->sendIAPdata(ack);

    boost::thread(boost::bind(&SrcUpgrade::sendSRCUpgradeTest, this));
}

void SrcUpgrade::handleRequestResult(uint32_t result) {
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
    bool ret = src_sdk->sendIAPdata(ack);
    if (!ret) {
        LOG(ERROR) << "send request result failed!";
    }
}

void SrcUpgrade::handleRequestConnection() {
    LOG(INFO) << "handle request connection!";
    std::vector<uint8_t> ack;
    ack.push_back(0xab);
    if (isStm32UpgradeFileReady()) {
        ack.push_back(0xf4);
    } else {
        ack.push_back(0xf5);
    }
    ack.push_back(0x01);
    ack.push_back(0x00);
    ack.push_back(IAP_CONN);

    // printVector(ack);
    src_sdk->sendIAPdata(ack);
}

void SrcUpgrade::handleRequestInfo() {
    LOG(INFO) << "handle request info!";
    std::string version = "4.1.2";
    auto md5sum = readMD5String(upgrade_md5sum_file_);
    auto filesize = getFileSize(upgrade_bin_file_);
    auto data = makeUpgradeInfo(version, md5sum, filesize, 0);

    //src v1不在发送md5校验值，v1/v2 统一发送32位CRC
    uint32_t crcvalue = makeUpgradeInfoCrc(data);

    LOG(INFO) << "filesize = " << filesize << " crcvalue = " << std::hex << crcvalue;
    // printVector(data);
    src_sdk->sendIAPdata(data);
}

uint32_t SrcUpgrade::makeUpgradeInfoCrc(std::vector<uint8_t> &data) {
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

void SrcUpgrade::sendSRCUpgradeTest() {
    LOG(INFO) << "sleep 10s and send Upgrade test to SRC application!";
    boost::this_thread::sleep_for(boost::chrono::milliseconds(10000));
    LOG(INFO) << "Send Upgrade test to SRC application!";
    /* NOTE:
     * 最近2个月内出现版本回退的现象，部分src需要10秒才多能起来，src收不到第一次发的请求测试,然后src重启3次后会，api会主动升级。
     * */

    std::thread([&]() {

        //为了兼容srtos -> src 版本回退升级
        bool isSrtosRollBackSrc = src_sdk->isSrtosRollBackSrc();

        for (auto i = 0; i < 50; ++i) {
            if (!is_in_upgrade_) {
                break;
            }

            if (isSrtosRollBackSrc) {
                src_sdk->upgradSrcTest();
            } else {
                src_sdk->upgradeTest();
            }
            
            boost::this_thread::sleep_for(boost::chrono::milliseconds(1000));
        }
    }).detach();
}

void SrcUpgrade::recordUpdate(bool is_update) {
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

    out.open(STM32_UPDATE_LOG, ios::out | ios::app | ios::binary);
    if (!out.is_open()) {
        LOG(ERROR) << "can not open " << STM32_UPDATE_LOG;
        return;
    }
    out.write(reinterpret_cast<char *>(&update_log), strlen(update_log));
    out.close();
    LOG(INFO) << "recode log update to " << STM32_UPDATE_LOG << ": " << update_log;
}

void SrcUpgrade::backupUpgradeFiles() {
    LOG(INFO) << "start backup upgrade files.";
    if (fs::exists(fs::path(upgrade_bin_file_))) {
        if (!fs::exists(fs::path(STM32_BACKUP_PATH))) {
            fs::create_directory(fs::path(STM32_BACKUP_PATH));
        }

        auto src_bin_file = fs::path(upgrade_bin_file_);
        auto src_md5sum_file = fs::path(upgrade_md5sum_file_);
        auto dst_bin_file = fs::path(STM32_BACKUP_PATH) / src_bin_file.filename();
        auto dst_md5sum_file = fs::path(STM32_BACKUP_PATH) / src_md5sum_file.filename();

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

        fs::remove(default_upgrade_file_symlink);
        fs::create_symlink(dst_bin_file, default_upgrade_file_symlink);
        LOG(INFO) << "symlink " << default_upgrade_file_symlink << " -> " << dst_bin_file;
    }
}

void SrcUpgrade::push_uint32t(std::vector<uint8_t> &src, uint32_t value) {
    src.push_back((uint8_t)value);
    src.push_back((uint8_t)(value >> 8));
    src.push_back((uint8_t)(value >> 16));
    src.push_back((uint8_t)(value >> 24));
}

bool SrcUpgrade::isStm32UpgradeFileReady() {
    namespace fs = boost::filesystem;

    if (upgrade_bin_file_ == "" || upgrade_md5sum_file_ == "") {
        LOG(WARNING) << "No src upgrade file provide, use default upgrade file instead. "
                     << default_upgrade_file_symlink.string();

        boost::system::error_code ec;
        if (fs::exists(default_upgrade_file_symlink, ec)) {
            auto default_upgrade_file = fs::read_symlink(default_upgrade_file_symlink);
            upgrade_bin_file_ = default_upgrade_file.string();
            upgrade_md5sum_file_ = upgrade_bin_file_ + "_md5sum.txt";

            LOG(INFO) << "Default src upgrade file: " << upgrade_bin_file_;
        } else {
            LOG(ERROR) << "No default src upgrade file can be found! " << ec.message();
            return false;
        }
    }

    if (fs::exists(fs::path(upgrade_bin_file_))) {
        auto md5sum = readMD5String(upgrade_md5sum_file_);
        if (md5sum.size() && checkMd5sum(upgrade_bin_file_, md5sum)) {
            LOG(INFO) << "stm32 bin file is ready";
            return true;
        } else {
            LOG(ERROR) << "stm32 md5sum check failed";
        }
    } else {
        LOG(ERROR) << "stm32 bin file not exist: " << upgrade_bin_file_;
    }

    return false;
}

std::string SrcUpgrade::readMD5String(const string &path) {
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

int SrcUpgrade::getFileSize(const string &path) {
    FILE *fp = fopen(path.c_str(), "rb");

    if (!fp) {
        return 0;
    }
    fseek(fp, 0, SEEK_END);
    auto filesize = ftell(fp);
    fclose(fp);

    return filesize;
}

std::vector<uint8_t> SrcUpgrade::makeUpgradeInfo(const std::string &version, const std::string &md5, int size,
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

bool SrcUpgrade::checkMd5sum(const std::string &filePath, const std::string &md5sum) {
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

std::vector<std::string> SrcUpgrade::splitWithStl(const std::string &str, const std::string &pattern) {
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

void SrcUpgrade::hashstring2vector(const std::string &src, std::vector<uint8_t> &data) {
    if (src.size() != 32) {
        LOG(WARNING) << "invalid hash string! size = " << src.size();
        return;
    }
    int i = src.size() - 1;
    while (i >= 0) {
        uint8_t s = charToInt(src[i]) + (charToInt(src[i - 1]) << 4);
        i -= 2;
        data.push_back(s);
    }
}

uint8_t SrcUpgrade::charToInt(char c) {
    if (c >= '0' && c <= '9') {
        return c - '0';
    } else if (c >= 'a' && c <= 'f') {
        return c - 'a' + 10;
    }
    return 0;
}
}  // namespace sdk