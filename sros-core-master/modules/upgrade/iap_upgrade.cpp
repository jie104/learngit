//
// Created by ehl on 22-04-26.
//

#include "iap_upgrade.h"
#include "core/msg/common_command_msg.hpp"
#include "core/msg/common_state_msg.hpp"
#include "core/device/can_interface.h"
#include "core/state.h"
#include "core/util/md5.h"
#include "core/task/task_manager.h"
#include "core/settings.h"

namespace fs = boost::filesystem;

namespace sros {

using namespace sros::device;

IapUpgrade::IapUpgrade(const uint32_t dev_type)
{
    if((dev_type == device::IAP_DEV_SPU100) || (dev_type == device::IAP_DEV_SH100) || (dev_type == device::IAP_DEV_BU100)) {
        initCanDevice(dev_type);
    } else if(dev_type == device::IAP_DEV_VSC300) {
        initUsartDevice(dev_type);
    }
    LOG(INFO) << "init device success.";
}

void IapUpgrade::initUsartDevice(const uint32_t dev_type)
{
    LOG(INFO) << "init usart device ...";
}

void IapUpgrade::initCanDevice(const uint32_t dev_type)
{
    iap_dev_type_ = dev_type;
    iap_dev_offset_ = IAP_DEV_ID_OFFSET_SPU100;
    for (uint32_t j = 0; j < 6; ++j)
    {
        if(iap_dev_type_ == IAP_DEV[j][0]) {
            iap_dev_offset_ = IAP_DEV[j][1];
            dev_default_id_ = IAP_DEV[j][2];
        }
    }
    
    uint32_t can_id[ID_TABLE_SIZE] = {0};
    uint32_t response_id[ID_TABLE_SIZE] = {0}; 

    LOG(INFO) << "iap init ... iap_dev_type_="<<iap_dev_type_<<" iap_dev_offset_="<<iap_dev_offset_;

    iap_can_ptr_.resize(ID_TABLE_SIZE);
    for (uint32_t j = 0; j < ID_TABLE_SIZE; ++j)
    {
        int dev_id = j + 1000;
        if(j == 0) {
            can_id[j] = CAN_ID_TABLE[j][1] + iap_dev_type_;
            response_id[j] = CAN_ID_TABLE[j][2] + iap_dev_offset_;
        } else {
            can_id[j] = CAN_ID_TABLE[j][1] + iap_dev_offset_;
            response_id[j] = CAN_ID_TABLE[j][2] + iap_dev_offset_;
        }
        can_id[j] |= 0x80000000U; // 扩展帧
        response_id[j] |= 0x80000000U;

        std::string device_name = "SPU_IAP_" + IAP_NAME[j];
        sros::device::DeviceID device_id = (sros::device::DeviceID)(dev_id + j);
        LOG(INFO) << "device_name:"<<device_name<<" device_id:"<<device_id<<" CAN ID:"<<std::hex<<can_id[j]<<", "<<response_id[j];

        iap_can_ptr_[j] = sros::device::createDevice<sros::device::IapCan>(
            device_name, device_id, sros::device::DEVICE_COMM_INTERFACE_TYPE_CAN_1,
            std::make_shared<sros::device::CanInterface>(can_id[j], response_id[j]));   

        iap_can_ptr_[j]->init(iap_dev_type_, iap_dev_offset_, CAN_ID_TABLE[j][0]);
        
        auto func = std::bind(&IapUpgrade::onDataRecieve, this, CAN_ID_TABLE[j][0], std::placeholders::_1);
        iap_can_ptr_[j]->setIapDataCallback(func);
        iap_can_ptr_[j]->setModelNo(getDeviceTypeName(dev_type));
        iap_can_ptr_[j]->setStateNone();
    }
}

void IapUpgrade::onDataRecieve(const uint32_t cmd_id, int response)
{
    switch (cmd_id)
    {
    case IAP_HOST_SCAN:
        LOG(INFO) << "IAP_HOST_SCAN: " << "response = " << std::hex << response;
        break;
    case IAP_HOST_REQUEST:
        if(response >= 0) {
            LOG(INFO) << "request device enter iap mode, response(1:OK; 0:error) = " << response;
            if(1 == response) {// in bootloader mode
                LOG(INFO) << "request device in bootloader mode success.";
                iap_host_state_ = IAP_HOST_SEND_FILE_INFO;
                iap_dev_state_ = IAP_DEV_BOOTLOADER;
            } else {
                LOG(INFO) << "request device in bootloader mode failed.";
                iap_host_state_ = IAP_HOST_SCAN;
                iap_dev_state_ = IAP_DEV_APP_MODE; // 1: app mode
            }
        } else {
            LOG(ERROR) << "IAP_HOST_REQUEST: got confirm from device timeout error!!!";
            iap_host_state_ = IAP_HOST_SCAN;
        }
        break;
    case IAP_HOST_SEND_FILE_INFO:
        if(response >= 0) {
            LOG(INFO) << "device response data pkg number = " << response;
            if((response == total_pkg_num_) && (IAP_HOST_SEND_FILE_INFO == iap_host_state_)) {
                LOG(INFO) << "got confirm from device success.";
            } else if(IAP_HOST_SEND_DATA_INFO == iap_host_state_) {
                LOG(INFO) << "device request the next packet data...";
                pkg_index_ = response; // 重新发送固件数据
            } else {
                LOG(INFO) << "got confirm from device error!!!";
            }
        } else {
            LOG(ERROR) << "got confirm from device error!!! response=" << response;
        }
        break;
    case IAP_HOST_SEND_DATA_INFO:
        if(response >= 0) {
            LOG(INFO) << "response = " << response;
            iap_dev_state_ = IAP_DEV_ACK;
        } else {
            LOG(ERROR) << "got confirm from device error!!! response=" << response;
        }
        break;
    case IAP_HOST_SEND_DATA:
        if(response >= 0) {
            LOG(INFO) << "device response pkg data index = " << response  << " success.";
            iap_dev_state_ = IAP_DEV_ACK;
            if(response + 1 >= total_pkg_num_) {
                iap_host_state_ = IAP_HOST_SEND_TEST;
                if((IAP_DEV_SPU100 == iap_dev_type_) || (IAP_DEV_BU100 == iap_dev_type_)) {
                   setUpgradeResult(1); 
                }
            } else {
                pkg_index_ = response + 1;
            }
        } else {
            LOG(ERROR) << "got confirm from device error!!! response = " << response;
        }
        break;
    case IAP_HOST_SEND_TEST:
        LOG(INFO) << "IAP_HOST_SEND_TEST: " << "response = " << std::hex << response;
        break;
    case IAP_HOST_RESPONSE_DEV_DATA:
        LOG(INFO) << "IAP_HOST_RESPONSE_DEV_DATA: " << "response = " << std::hex << response;
        if(response >= 0) {
            if(IAP_HOST_SEND_DATA_INFO == iap_host_state_) {
                LOG(INFO) << "Host in sending IAP_HOST_SEND_DATA_INFO already. pkg_index_";
                iap_dev_state_ = IAP_DEV_REQ_DATA;
                pkg_index_ = response;
            }
        } else {
            LOG(ERROR) << "got confirm from device error!!!";
        }
        break;
    case IAP_HOST_RESPONSE_DEV_INFO:
        LOG(INFO) << "IAP_HOST_RESPONSE_DEV_INFO: " << "response = " << std::hex << response;
        if(response >= 0) {
            if(IAP_HOST_SEND_DATA_INFO == iap_host_state_) {
                LOG(INFO) << "Host in sending IAP_HOST_SEND_DATA_INFO already. pkg_index_";
                iap_dev_state_ = IAP_DEV_REQ_DATA;
                pkg_index_ = response;
            }
        } else {
            LOG(ERROR) << "got confirm from device error!!!";
        }
        break;
    case IAP_HOST_GET_DEV_RESULT:
        LOG(INFO) << "IAP_HOST_GET_DEV_RESULT: " << "response = " << std::hex << response;
        iap_can_ptr_[IAP_HOST_GET_DEV_RESULT]->setStateOK();
        if(response == 0x01) {
            LOG(ERROR) << "device status : upgrade error!!!";
            iap_dev_state_ = IAP_DEV_NACK;
            if(IAP_HOST_SEND_DATA == iap_host_state_) {
                LOG(ERROR) << "device status : upgrade error!!!";
                setUpgradeResult(UPGRADED_IAP_STEP1);
            }
        }
        break;
    default:
        break;
    }
}

bool IapUpgrade::sendCommand(const sros::device::IAP_CAN_COMMAND_t command) {
    std::vector<uint8_t> data;
    switch (command)
    {
    case IAP_HOST_SCAN:
        data = {0x1, (uint8_t)iap_dev_type_, (uint8_t)(iap_dev_type_>>8), 0x00, 0x00, 0x00, 0x00, 0x00};
        break;
    case IAP_HOST_REQUEST:
        data = {0x1, (uint8_t)dev_default_id_, (uint8_t)(dev_default_id_>>8), 0x00, 0x00, 0x00, 0x00, 0x00};
        break;
    case IAP_HOST_SEND_FILE_INFO:
        //data = {0x1, iap_dev_offset_&0xFF, (iap_dev_offset_>>8)&0xFF, 0x00, 0x00, 0x00, 0x00, 0x00};
        break;
    case IAP_HOST_SEND_DATA_INFO:
        ///data = {0x1, iap_dev_offset_&0xFF, (iap_dev_offset_>>8)&0xFF, 0x00, 0x00, 0x00, 0x00, 0x00};
        break;
    case IAP_HOST_SEND_DATA:
        //data = {0x1, iap_dev_offset_&0xFF, (iap_dev_offset_>>8)&0xFF, 0x00, 0x00, 0x00, 0x00, 0x00};
        break;
    case IAP_HOST_SEND_TEST:
        data = {(uint8_t)dev_default_id_, (uint8_t)(dev_default_id_>>8),0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
        break;
    default:
        LOG(WARNING) << "Invalid command error!!!";
        return false;
    }
    LOG(INFO) << "cmd id: " << command << " name:" << IAP_NAME[command]  << " send size: "<<data.size() << std::hex <<" data: "<<numberListToStr(data.cbegin(), data.cend());
    return iap_can_ptr_[command]->sendData(data);
}

bool IapUpgrade::isIapUpgradeFileReady() {
    // namespace fs = boost::filesystem;

    if (upgrade_bin_file_ == "" || upgrade_md5sum_file_ == "") {
        LOG(ERROR) << "No iap upgrade file can be found! ";
        return false;
    }

    if (fs::exists(fs::path(upgrade_bin_file_))) {
        auto md5sum = readMD5String(upgrade_md5sum_file_);
        if (md5sum.size() && checkMd5sum(upgrade_bin_file_, md5sum)) {
            LOG(INFO) << "iap bin file is ready";
            return true;
        } else {
            LOG(ERROR) << "iap md5sum check failed";
        }
    } else {
        LOG(ERROR) << "iap bin file not exist: " << upgrade_bin_file_;
    }

    return false;
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

std::string IapUpgrade::readMD5String(const string &path) {
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

int IapUpgrade::getFileSize(const string &path) {
    FILE *fp = fopen(path.c_str(), "rb");

    if (!fp) {
        return 0;
    }
    fseek(fp, 0, SEEK_END);
    auto filesize = ftell(fp);
    fclose(fp);

    return filesize;
}

bool IapUpgrade::checkMd5sum(const std::string &filePath, const std::string &md5sum) {
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

bool IapUpgrade::setUpgradeFilePath(const std::string &upgrade_bin_file_path) {
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
    LOG(INFO) << "upgrade_bin_file_path error!!!";
    return false;
}


bool IapUpgrade::upgradeRequest(const std::string &upgrade_bin_file_path) {
    if (!setUpgradeFilePath(upgrade_bin_file_path)) {
        return false;
    }
    // scan
    int response = iap_can_ptr_[0]->scanDevice(iap_dev_type_);
    if(response >= 0) {
        iap_dev_state_ = response;
    } else {
        LOG(ERROR) << "scan device type " << iap_dev_type_ << " error!!!";
        return false;
    }
    setDeviceVersion(iap_dev_state_);

    // request
    response = iap_can_ptr_[1]->requestDeviceEnterIAP();
    if(1 == response) {
        iap_dev_state_ = IAP_DEV_BOOTLOADER;
        LOG(INFO) <<"request result(1): OK, device in bootloader mode";
    } else if(0 == response) {
        LOG(WARNING) <<"request result(0): not ready, device in APP mode.";
        if(iap_dev_state_ == IAP_DEV_APP_MODE) {
            if(iap_dev_type_ == IAP_DEV_SPU100 || iap_dev_type_ == IAP_DEV_BU100) {
                LOG(INFO) <<"request result: OK for SPU100 app mode";
            } else {
                LOG(INFO) <<"request result: retry once for other devices but SPU100.";
                return iap_can_ptr_[1]->requestDeviceEnterIAP();
            }
        } else {
            LOG(ERROR) <<"request result(0): unknown error!!!";
            setUpgradeResult(0);
            return false;
        }
    } else {
        LOG(ERROR) <<"request result: timeout error!!!";
        if(iap_dev_type_ == IAP_DEV_SPU100 || iap_dev_type_ == IAP_DEV_BU100) {
            LOG(INFO) <<"request result: OK for SPU100 (test)";
        } else {
            setUpgradeResult(0);
            return false;
        }
    }
    // send file info and data info, data pkg ...
    if(iap_dev_state_ == IAP_DEV_BOOTLOADER) {
        if(!upgradeSend(upgrade_bin_file_path)) {
            LOG(ERROR) <<"send firmware data error!!!";
            setUpgradeResult(0);
            return false;
        }
    }
    // send TEST cmd to check if upgrade success
    if(IAP_DEV_SPU100 != iap_dev_type_ && IAP_DEV_BU100 != iap_dev_type_) {
        if(!upgradeTest()) {
           LOG(ERROR) <<"IAP send test cmd done.";  
        }
        LOG(INFO) <<"IAP send test cmd done.";        
    }
    LOG(INFO) <<"IAP done."; 
    return true;
}

bool IapUpgrade::upgradeSend(const std::string &upgrade_bin_file_path) {
    if (!setUpgradeFilePath(upgrade_bin_file_path)) {
        return false;
    }
    LOG(INFO) <<"IAP upgradeSend start ...";
    // send bin file info (package number) , data index + crc and  package data
    
    if(!handleRequestFileInfo()) {
        return false;
    }

    pkg_index_ = 0;
    if(!handleRequestData(0)) {
        return false;
    }
    iap_can_ptr_[4]->setStateOK();

    LOG(INFO) << "iap_host_state_ = " << iap_host_state_;
    iap_host_state_ = IAP_HOST_SEND_DATA;
    int time_out = 150*20;
    int ret = 0;
    int retry_count = 0;
    while(1) {
        ret = handleRequestData(pkg_index_);
        if(!ret) {
            retry_count ++;
        } else {
            retry_count = 0;
            if((IAP_HOST_SEND_TEST == iap_host_state_) || (pkg_index_ >= total_pkg_num_)) {
                break;
            }
            continue;
        }
        usleep(50*1000);
        if(!time_out--) {
            LOG(ERROR) << "sending data timeout error!!!";
            return false;
        }
        if(IAP_DEV_NACK == iap_dev_state_ || retry_count > 5) {
            LOG(ERROR) << "davice states error!!!";
            return false;
        }
    }
    LOG(INFO) <<"IAP send data done.";
    return true;
}

void IapUpgrade::disableDevice() {
    for(int i = 0; i < ID_TABLE_SIZE; i ++) {
        iap_can_ptr_[i]->setStateNone();
    }
}

bool IapUpgrade::upgradeTest() {
    // scan device
    int response = iap_can_ptr_[0]->scanDevice(iap_dev_type_);
    if(response >= 0) {
        iap_dev_state_ = response;
    } else {
        LOG(ERROR) << "scan device type " << iap_dev_type_ << " error!!!";
        setUpgradeResult(0);
        return false;
    }
    setDeviceVersion(iap_dev_state_);
    // send TEST cmd
    if(sendCommand(IAP_HOST_SEND_TEST)) {
        LOG(INFO) <<"IAP check upgrade result success.";
    } else {
        LOG(ERROR) <<"IAP check upgrade result error!!!";
        setUpgradeResult(0);
        return false;
    }
    sleep(2);
    disableDevice();
    setUpgradeResult(1);
    return true;
}

uint32_t IapUpgrade::checkIapDeviceType(const std::string &upgrade_bin_file_path)
{
    return iap_dev_type_;
}


void IapUpgrade::recordUpdate(bool is_update) {
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


void IapUpgrade::backupUpgradeFiles() {
    LOG(INFO) << "start backup upgrade files.";
    if (fs::exists(fs::path(upgrade_bin_file_))) {
        if (!fs::exists(fs::path(IAP_BACKUP_PATH))) {
            fs::create_directory(fs::path(IAP_BACKUP_PATH));
        }

        auto src_bin_file = fs::path(upgrade_bin_file_);
        auto src_md5sum_file = fs::path(upgrade_md5sum_file_);
        auto dst_bin_file = fs::path(IAP_BACKUP_PATH) / src_bin_file.filename();
        auto dst_md5sum_file = fs::path(IAP_BACKUP_PATH) / src_md5sum_file.filename();

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

        fs::remove(default_iap_upgrade_file_symlink);
        fs::create_symlink(dst_bin_file, default_iap_upgrade_file_symlink);
        LOG(INFO) << "symlink " << default_iap_upgrade_file_symlink << " -> " << dst_bin_file;
    }
}


void IapUpgrade::setUpgradeResult(int result) {
    // handle upgrade result
    if (result == 1) {
        LOG(INFO) << "setUpgradeResult success!";
        recordUpdate(true);
        //backupUpgradeFiles();
    } else {
        LOG(INFO) << "setUpgradeResult failed!!!";
        recordUpdate(false);
    }
    if(iap_upgrade_callback_f_)
        iap_upgrade_callback_f_(result);
}

bool IapUpgrade::handleRequestFileInfo()
{
    iap_host_state_ = IAP_HOST_SEND_FILE_INFO;
    
    std::vector<uint8_t> data;
    FILE *fp = fopen(upgrade_bin_file_.c_str(), "rb");

    if (!fp) {
        LOG(WARNING) << "IAP update file not exists!";
        return false;
    }
    fseek(fp, 0, SEEK_END);

    auto filesize = ftell(fp);
    fclose(fp);

    total_pkg_num_ = (filesize + IAP_CAN_DATA_PKG_SIZE_MAX - 1) / IAP_CAN_DATA_PKG_SIZE_MAX ;

    if(iap_can_ptr_[2]->sendFileInfo(total_pkg_num_) < 0) {
        LOG(WARNING) << "IAP send file info error!!!";
        iap_host_state_ = IAP_HOST_SCAN;
        return false;
    }
    LOG(WARNING) << "IAP send file info success.";
    return true;
}


int IapUpgrade::handleRequestDataInfo(const uint32_t index, const uint32_t pkgDataSize, const uint32_t crc)
{
    iap_host_state_ = IAP_HOST_SEND_DATA_INFO;
    LOG(INFO) << "sendDataPacketInfo index = " << index << " pkg size = " << pkgDataSize << std::hex << " crc = " << crc;

    return iap_can_ptr_[3]->sendDataPacketInfo(index, pkgDataSize, crc);
}

bool IapUpgrade::handleRequestData(uint32_t index) {
    // LOG(INFO) <<"handle request data!";
    std::vector<uint8_t> data;
    const int block_size = 512;
    int last_read_bytes = 0;
    int i = 0;
    uint8_t buffer[512];
    LOG(INFO) << "handleRequestData: iap_host_state_ = " << iap_host_state_;
    if (index > 0 && (iap_host_state_ == IAP_HOST_SCAN)) {
        LOG(WARNING) << "invalid request! iap_host_state_ = " << iap_host_state_ << " index = " << index
                     << " sros is not in active-iap mode";
        return false;
    }

    FILE *fp = fopen(upgrade_bin_file_.c_str(), "rb");

    if (!fp) {
        LOG(WARNING) << "IAP update file not exists!";
        return false;
    }
    fseek(fp, 0, SEEK_END);

    auto filesize = ftell(fp);
    if (index * block_size > filesize) {
        fclose(fp);
        LOG(WARNING) << "------------------- > invalid request !" << " index = " << index << " filesize = " << filesize;
        return true;
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

    uint16_t crc16 = 0;
    for (i = 0; i < last_read_bytes; i++) {
        crc16 += buffer[i];
    }
    crc16 = ~crc16 + 1;
    LOG(WARNING) <<"calculate crc for bytes = "<<last_read_bytes <<" crc = "<<std::hex<<(int)crc16;

    if(handleRequestDataInfo(index, last_read_bytes, crc16) < 0) {
        LOG(ERROR) << "error!!!";
        return false;
    }

    for (i = 0; i < last_read_bytes; i++) {
        data.push_back(buffer[i]);
    }

    // printVector(data);
    LOG(INFO) << "start send data pkg: " << index + 1 << "/" <<  total_pkg_num_ << " size: " << data.size();
    return iap_can_ptr_[4]->sendIAPdata(data);
}

int IapUpgrade::getDeviceType()
{
    return iap_dev_type_;
}

void IapUpgrade::setDeviceVersion(int mode)
{
    version_str_ = iap_can_ptr_[0]->getDevVersionStr();
    for(int i = 0; i < ID_TABLE_SIZE; i ++) {
        std::string device_name = getDeviceTypeName(iap_dev_type_);
        iap_can_ptr_[i]->setVersionNo(version_str_);
        if(IAP_DEV_BOOTLOADER == mode) {
            device_name += "_bootloader";
        } else {
            device_name += "_app_mode";
        }
        iap_can_ptr_[i]->setModelNo(device_name);
    }
}

std::string IapUpgrade::getDeviceVersion()
{
    return version_str_;
}

std::string IapUpgrade::getDeviceTypeName(const uint32_t dev_type)
{
    std::string device_name;
    switch(dev_type)
    {
        case IAP_DEV_EU200:
            device_name = "EU200";
        break;
        case IAP_DEV_SH100:
            device_name = "SH100";
        break;
        case IAP_DEV_SPU100:
            device_name = "SPU100";
        break;
        case IAP_DEV_LC200:
            device_name = "LC200";
        break;
        case IAP_DEV_VSC300:
            device_name = "VSC300";
        break;
        case IAP_DEV_BU100:
            device_name = "BU100";
        break;
        default:
            device_name = "IAP";
        break;
    }
    return device_name;
}

} // namespace sros