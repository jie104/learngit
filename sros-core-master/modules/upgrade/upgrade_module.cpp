/**
 * describe:
 * Created by pengjiali on 18-12-12.
 **/

#include "upgrade_module.h"
#include <glog/logging.h>
#include <memory>
#include "core/msg/command_msg.hpp"
#include "core/msg/common_msg.hpp"
#include "core/msg/notification_msg.hpp"
#include "core/src.h"
#include "core/util/utils.h"
#include "core/util/md5.h"
#include "core/src.h"
#include "core/version.h"
#include "core/settings.h"
#include "core/exec_error.hpp"

#include "core/device/device_manager.h"

namespace sros {

using namespace std;
using namespace sros::core;

// 用于加密导入/导出文件的秘钥
std::string UPDATE_KEY = "SROS2016-update";
std::string SROS_BIN_DIR = "/sros/bin/";
std::string IMPORT_NETWORK_FILE_PATH = "/sros/update/network-monitor.sh";
std::string IMPORT_STARTUP_FILE_PATH = "/sros/update/startup.sh";
std::string BACKUP_SROS_UPGRADE_PATH = "/sros/backup/upgrade/sros";
std::string BACKUP_SRC_UPGRADE_PATH = "/sros/backup/upgrade/src";
std::string BACKUP_IAP_UPGRADE_PATH = "/sros/backup/upgrade/iap";
std::string BACKUP_VSC_UPGRADE_PATH = "/sros/backup/upgrade/vsc";

const char SROS_UPDATE_LOG[] = "/sros/update.log";
const char SROS_IAP_UPDATE_LOG[] = "/sros/log/iap_update.log";
const char SROS_UPDATE_RECORD[] = "/sros/update_record";
const char SROS_IAP_UPDATE_RECORD[] = "/sros/.update_record_iap";
const string TABLE_UPGRADE_RECORD = "upgrade_record";

const uint16_t SROS_UPGRADED = 0xFF;
const uint16_t SROS_DEFAULT = 0x00;
const uint16_t SROS_UPGRADED_IAP_STEP1 = 0x11; // 针对自研硬件SPU100: 升级请求发送成功,但未发送固件
// const uint16_t SROS_UPGRADED_IAP_STEP1_ACK = 0xF1; // 断电重启后,由启动脚本设置该值
const uint16_t SROS_UPGRADED_IAP_STEP2 = 0x22;
const uint16_t SROS_UPGRADED_IAP_BU100_STEP2 = 0x33;
// const uint16_t SROS_UPGRADED_IAP_STEP2_ACK = 0xF2;

const int16_t UPGRADE_FLAG_SROS = 0x1;
const int16_t UPGRADE_FLAG_SRC = 0x2;
const int16_t UPGRADE_FLAG_IAP = 0x10;
const int16_t UPGRADE_FLAG_SPU = 0x13;
const int16_t UPGRADE_FLAG_IAP_CONTINUE = 0x20;
const int16_t UPGRADE_FLAG_IAP_DONE = 0x21;

const int16_t UPGRADE_SUCCESS = 0x1;
const int16_t UPGRADE_FAILED = 0x0;
const int16_t MAX_UPGRADE_FILE_BACKUP = 0x05;


bool sortByDatetime(const fs::path &p1, const fs::path &p2) {
    return fs::last_write_time(p1) < fs::last_write_time(p2);
}

UpgradeModule::UpgradeModule(): Module("UpgradeModule"),
db_(&g_db), update_result_(0), is_in_upgrade_(false) {
    createBackupPath();
}

void UpgradeModule::run() {
    LOG(INFO) << "UpgradeModule module start running";

    subscribeTopic("TOPIC_UPDATA_SROS", CALLBACK(&UpgradeModule::onUpdateSrosMsg));  // 升级sros
    subscribeTopic("TOPIC_UPDATA_SRC", CALLBACK(&UpgradeModule::onUpdateSrcMsg));    // 升级src
    subscribeTopic("TOPIC_UPDATA_VSC_RESULT", CALLBACK(&UpgradeModule::onVSCUpgradeResultMsg));    // 升级vsc结果
    subscribeTopic("TIMER_5S", CALLBACK(&UpgradeModule::onTimer_5s));

    src_sdk->setUpgradeCallback(boost::bind(&UpgradeModule::onSRCUpgradeResult, this, _1));

    // 检查上次重启的升级结果是否成功
    update_result_ = checkUpdateResult();
   // handleIapUpdateResult();
    boost::thread(boost::bind(&UpgradeModule::handleIapUpdateResult, this));

    dispatch();
}

void UpgradeModule::onTimer_5s(sros::core::base_msg_ptr m) {
    if (update_result_ != 0) {
        int16_t upgrade_result = UPGRADE_FAILED;
        if (update_result_ == 1) {
            upgrade_result = UPGRADE_SUCCESS;
        }
        updateUpgradeRecord(UPGRADE_FLAG_SROS, upgrade_result, SROS_VERSION, SROS_VERSION_STR);
        if (!g_state.network_session_manager.empty()) {
            notifyUpdateResult();
        }
    }
}

void UpgradeModule::onUpdateSrosMsg(sros::core::base_msg_ptr m) {
    auto msg = std::dynamic_pointer_cast<sros::core::CommonMsg>(m);
    const std::string &sros_upgrade_file_path = msg->str_0_;

    std::string file_md5 = calculateFileMd5(sros_upgrade_file_path);
    std::string backup_file_path = "";
    if (file_md5 != "") {
        backup_file_path = BACKUP_SROS_UPGRADE_PATH + "/" + file_md5 + getFilePostfix(sros_upgrade_file_path);
    }

    if (backup_file_path != "") {
        backupUpgradeFile(sros_upgrade_file_path, backup_file_path);
        removeOldUpgradeFile();
    }

    createUpgradeRecord(UPGRADE_FLAG_SROS, "", SROS_VERSION, SROS_VERSION_STR, backup_file_path);

    bool ret = updateSros(sros_upgrade_file_path);
    if (ret) {
        // sros升级成功后直接重启不需要发送成功消息
        // 有自研硬件在升级,暂时不重启SROS
        if(is_in_upgrade_ && upgrade_reset_sros_) {
            LOG(INFO) << "in iap upgrade ... not restart sros this time ... return";
            return;
        }
        // 重启SROS，在startup.sh中完成SROS剩余的升级
        auto reset_msg = std::make_shared<sros::core::CommandMsg>(getName());
        reset_msg->command = sros::core::CMD_RESET_SROS;
        sendMsg(reset_msg);
    } else {
        // 升级失败后，将/sros/update/目录下的内容全删除掉,防止判断升级失败了后重启还将继续升级sros
        // FIXME(pengjiali): 由于历史原因，我们是先置位再升级，若升级失败再将置位删掉，这种其实是存在风险的，该改为先升级再置位
        std::string cmd = "rm " + IMPORT_UPDATE_DIR + "* -r";
        systemWrapper(cmd);

        auto msg_reply = std::make_shared<sros::core::CommonMsg>("TOPIC_UPDATA_SROS_REPLY");
        msg_reply->int_0_ = -1;  // 代表升级失败
        sendMsg(msg_reply);
    }
}

void UpgradeModule::onUpdateSrcMsg(sros::core::base_msg_ptr m) {
    auto msg = std::dynamic_pointer_cast<sros::core::CommonMsg>(m);

    const std::string &src_upgrade_file_path = msg->str_0_;
    auto backup_file_path = fs::path(STM32_BACKUP_PATH) / fs::path(src_upgrade_file_path).filename();

    src_update_to_srtos = false;
    srtos_update_to_src = false;
    LOG(INFO) << "update src with file: " << src_upgrade_file_path;
    if(src_upgrade_file_path.find("srtos") != -1 && src_sdk->getVersion() == sdk::SRC_PROTO_VERSION_V1 
                && src_sdk->isSetSrcVersionManual() == false) {
        LOG(INFO) << "src update srtos";
        src_update_to_srtos = true;
    } else if(src_upgrade_file_path.find("src") != -1 && src_sdk->getVersion() == sdk::SRC_PROTO_VERSION_V2) {
        LOG(INFO) << "srtos update src";
        srtos_update_to_src = true;
    }

    createUpgradeRecord(UPGRADE_FLAG_SRC, "", src_sdk->getSRCVersion(), src_sdk->getSRCVersionStr(), backup_file_path.string());

    if (!src_sdk->upgradeRequest(src_upgrade_file_path)) {  // 请求升级失败
        auto msg_reply = std::make_shared<sros::core::CommonMsg>("TOPIC_UPDATA_SRC_REPLY");
        msg_reply->int_0_ = -1;  // 代表升级失败
        sendMsg(msg_reply);
        return;
    }

    request_upgrade_src_ = true;
}

void UpgradeModule::updateDeviceIapRequest(const std::string &iap_upgrade_file_path) {

    recordUpdateIap(SROS_UPGRADED_IAP_STEP1); // 标记正在执行升级,若传输文件过程中,断电了,重启系统不会删除固件文件,且SROS不更新一次
    is_in_upgrade_ = true;
    if (!iap_can_upgrade_->upgradeRequest(iap_upgrade_file_path)) {  // 请求升级
        LOG(WARNING) <<"iap scan and request error!!!";
        recordUpdateIap(0);
        auto msg_reply = std::make_shared<sros::core::CommonMsg>("TOPIC_UPDATA_IAP_REPLY");
        msg_reply->int_0_ = -1;  // 代表升级失败
        sendMsg(msg_reply);
        return;
    }
}

void UpgradeModule::updateDeviceIapTest(const uint32_t dev_type) {
    startIapUpgrade(dev_type);
    LOG(INFO) << "updateDeviceIapTest run IAP result test ...";
    if (!iap_can_upgrade_->upgradeTest()) {
        auto msg_reply = std::make_shared<sros::core::CommonMsg>("TOPIC_UPDATA_IAP_REPLY");
        msg_reply->int_0_ = -1;  // 代表升级失败
        sendMsg(msg_reply);
        return;
    }
    update_result_ = 1;
    // notify to user
    LOG(INFO) << "updateDeviceIapTest : notify to user";
    if (!g_state.network_session_manager.empty()) {
        notifyUpdateResult();
    }
    update_result_ = 0;
}


void UpgradeModule::backupIapUpgradeFile(const std::string &dev_name, const std::string &upgrade_file_path) {

    if (!fs::exists(fs::path(IAP_BACKUP_PATH))) {
        fs::create_directory(fs::path(IAP_BACKUP_PATH));
    }
    if (!fs::exists(fs::path(VSC_BACKUP_PATH))) {
        fs::create_directory(fs::path(VSC_BACKUP_PATH));
    }
    
    std::string upgrade_md5sum_file;
    std::string backup_file_path;
    std::string file_md5 = calculateFileMd5(upgrade_file_path);
    if (file_md5 != "") {
        upgrade_md5sum_file = upgrade_file_path + "_md5sum.txt";
        LOG(INFO) << "save md5 info to file " << upgrade_md5sum_file;
        std::string cmd_str = "echo " + file_md5 + " > " + upgrade_md5sum_file;
        systemWrapper(cmd_str);
        if(!dev_name.compare("VSC")) {
            backup_file_path = BACKUP_VSC_UPGRADE_PATH + upgrade_file_path.substr(upgrade_file_path.find_last_of('/'));
        } else {
            backup_file_path = BACKUP_IAP_UPGRADE_PATH + upgrade_file_path.substr(upgrade_file_path.find_last_of('/')) + "_" + file_md5;
        }
    }

    // VSC bin文件保存到 独立的 vsc 目录, 其它保存到 iap 目录
    if (backup_file_path != "") {
        backupUpgradeFile(upgrade_file_path, backup_file_path);
    }

    // VSC md5 以 txt 文件形式保存到独立的 vsc 目录, 其它自研硬件的 md5 数值保存在固件文件名上
    if (!dev_name.compare("VSC")) {
        if(upgrade_md5sum_file != "") {
            backup_file_path = BACKUP_VSC_UPGRADE_PATH + upgrade_md5sum_file.substr(upgrade_md5sum_file.find_last_of('/'));
            backupUpgradeFile(upgrade_md5sum_file, backup_file_path);            
        }
    }

    //get device version info
    uint32_t device_verion_number = 0; 
    std::string device_version_str = "0.0.0";
    std::string device_name_str = dev_name;
    auto dm = sros::device::DeviceManager::getInstance();
    auto device_map = dm->getDeviceList();
    for (const auto &it : *device_map) {
        const auto &device = it.second;
        if (device->getState() != sros::device::DEVICE_NONE) {  // 存在device为DEVICE_NONE的情况
            if(!dev_name.compare(device->getName())) {
                device_version_str = device->getVersionNo();
                break;
            }
        }
    }
    LOG(INFO) << "DeviceManager found device " << dev_name << " version: " << device_version_str;

    //record to SQL database
    createUpgradeRecord(UPGRADE_FLAG_IAP, device_name_str, device_verion_number, device_version_str, backup_file_path);
}

// TODO : 启动独立线程处理升级, 自研硬件可并发升级, 需要加睡眠等待,之后再重启SROS
bool UpgradeModule::updateDeviceIAP() {

    // check if iap firmware directory exist.
    std::string iap_firmware_dir = IMPORT_UPDATE_DIR + "iap_bin/";
    if (fs::exists(fs::path(iap_firmware_dir))) {
        LOG(INFO) << "IAP firmware files exist. " << iap_firmware_dir;
        // find iap bin files
        fs::directory_iterator dir_end;
        for (fs::directory_iterator pos(iap_firmware_dir); pos != dir_end; ++pos) {
            fs::path p = pos->path();
            std::string file_name_str = p.string();
            std::string file_name_postfix = file_name_str.substr(file_name_str.find_last_of('.'));
            LOG(INFO) << "file_name_str = " << file_name_str;
            iap_type_ = TypeNone;
            if((file_name_str.find("vsc") != std::string::npos) && (0 == file_name_postfix.compare(".bin"))) {
                iap_type_ = TypeVsc;
                LOG(INFO) << "iap firmware bin file name: " << file_name_str;
                backupIapUpgradeFile("VSC",file_name_str);
                upgrade_reset_sros_ = true; // test test
                is_in_upgrade_ = true;
                // 通知 vsc_module 对 VSC300 进行升级
                auto upgrade_msg = std::make_shared<sros::core::CommonMsg>("TOPIC_UPDATA_VSC");
                upgrade_msg->str_0_ = file_name_str;
                sendMsg(upgrade_msg);
            } else if ((file_name_str.find("spu100_") != std::string::npos) && (0 == file_name_postfix.compare(".bin"))) {
                iap_type_ = TypeSpu;
                LOG(INFO) << "iap firmware bin file: " << file_name_str;
                backupIapUpgradeFile("SPU",file_name_str);
                startIapUpgrade(sros::device::IAP_DEV_SPU100);
                if(iap_can_upgrade_ != nullptr) {
                    LOG(INFO) << "iap upgrade request";
                    updateDeviceIapRequest(file_name_str);
                }
                break;
            } else if ((file_name_str.find("bu100_") != std::string::npos) && (0 == file_name_postfix.compare(".bin"))) {
                iap_type_ = TypeBu100;
                LOG(INFO) << "iap firmware bin file: " << file_name_str;
                backupIapUpgradeFile("BU100",file_name_str);
                startIapUpgrade(sros::device::IAP_DEV_BU100);
                if(iap_can_upgrade_ != nullptr) {
                    LOG(INFO) << "iap upgrade request";
                    updateDeviceIapRequest(file_name_str);
                }
                break;
            } else if ((file_name_str.find("sh100_") != std::string::npos) && (0 == file_name_postfix.compare(".bin"))) {
                LOG(INFO) << "iap firmware bin file name: " << file_name_str;
                upgrade_reset_sros_ = false; // test test

                // 发送请求,直到发送固件完成, 读到状态后再返回.
                break;
            }
        }
        std::thread t(&UpgradeModule::loopCheckIapResult, this);
        t.detach();
    } else {
        LOG(ERROR) << "IAP firmware dir " <<  iap_firmware_dir << " not found.";
        return false;
    }
    LOG(INFO) << "VSC upgrade task exist, return";

    return true;
}

bool UpgradeModule::startIapUpgrade(uint32_t dev_type)
{
    if(nullptr == iap_can_upgrade_) {
        LOG(INFO) << "iap_can_upgrade_ init dev_type:" <<dev_type;
        switch (dev_type)
        {
        case sros::device::IAP_DEV_SPU100:
            iap_can_upgrade_ = std::make_shared<IapUpgrade>(sros::device::IAP_DEV_SPU100);
            if(iap_can_upgrade_) {
               iap_can_upgrade_->setUpgradeCallback(boost::bind(&UpgradeModule::onIAPUpgradeResult, this, _1)); 
            }
            LOG(INFO) << "sleep ...";
            sleep(1);
            break;
        case sros::device::IAP_DEV_BU100:
            iap_can_upgrade_ = std::make_shared<IapUpgrade>(sros::device::IAP_DEV_BU100);
            if(iap_can_upgrade_) {
               iap_can_upgrade_->setUpgradeCallback(boost::bind(&UpgradeModule::onIAPUpgradeResult, this, _1)); 
            }
            LOG(INFO) << "sleep ...";
            sleep(1);
            break;
        case sros::device::IAP_DEV_VSC300:

            break;
        default:
            break;
        }
        LOG(INFO) << "iap_can_upgrade_ init done";
        return true;
    } else {
        LOG(ERROR) << "iap_can_upgrade_ already init";
        return false;
    }
}

bool UpgradeModule::updateSros(const std::string &file_path) {
    namespace fs = boost::filesystem;

    std::string cmd_str;
    cmd_str = "cat " + file_path + " | openssl des3 -md md5 -d -k " + UPDATE_KEY + " | tar xzvf - -C / ";

    auto &s = sros::core::Settings::getInstance();
    auto enable_sros_native_debug = (s.getValue<std::string>("debug.enable_sros_native_debug", "False") == "True");
    if (!systemWrapper(cmd_str)) {
        if (!enable_sros_native_debug) {
            return false;  // 若是本地调试，由于部分库不能放到指定位置导致解压失败，但是我们允许继续
        }
    }

    syncDisk();

    // execute pre-update shell,留一条后门
    std::string pre_update_sh = "source " + IMPORT_UPDATE_DIR + "prepare_update.sh";
    if (fs::exists(fs::path(pre_update_sh))) {
        if (!systemWrapper(pre_update_sh)) {
            LOG(INFO) << "prepare update failed!";
        }
    }

    // Update startup.sh
    if (fs::exists(fs::path(IMPORT_NETWORK_FILE_PATH)) && fs::exists(fs::path(IMPORT_STARTUP_FILE_PATH))) {
        cmd_str = "";
        cmd_str = "mv " + IMPORT_STARTUP_FILE_PATH + " " + SROS_BIN_DIR;
        systemWrapper(cmd_str);
        LOG(INFO) << "/sros/bin/startup.sh updated!!!";

        cmd_str = "";
        cmd_str = "mv " + IMPORT_NETWORK_FILE_PATH + " " + SROS_BIN_DIR;
        systemWrapper(cmd_str);
        LOG(INFO) << "/sros/bin/network-monitor.sh updated!!!";
    } else {
        LOG(WARNING) << IMPORT_STARTUP_FILE_PATH << " and " << IMPORT_NETWORK_FILE_PATH << " DO NOT EXSITS!";
    }

    syncDisk();

    // check if iap firmware directory exist.
    std::string iap_firmware_dir = IMPORT_UPDATE_DIR + "iap_bin/";
    if (fs::exists(fs::path(iap_firmware_dir))) {
        upgrade_reset_sros_ = false; // Matrix 请求升级自研硬件,不需要主动reset sros
        if(!updateDeviceIAP()) {
            LOG(ERROR) << "in updateSros : update device IAP error!!!";
            return false;
        }
    } else {
        LOG(INFO) << "no IAP upgrade.";
        recordUpdateIap(0);
    }
    return true;
}

void UpgradeModule::onSRCUpgradeResult(int result) {
    LOG(INFO) << "onSRCUpgradeResult: " << result;
    int32_t upgrade_result = UPGRADE_FAILED;
    if (result == 1) {
        upgrade_result = UPGRADE_SUCCESS;
        if (request_upgrade_src_) {  // 是外部请求的升级才需要重启sros
            if(src_update_to_srtos) {
                 syncSrcParamToMc();   
            } else if(srtos_update_to_src) {
                 syncMcParamToSrc();
            }
            auto reset_msg = std::make_shared<sros::core::CommandMsg>(getName());
            reset_msg->command = sros::core::CMD_RESET_SROS;
            sendMsg(reset_msg);
        }
    } else {
        auto msg_reply = std::make_shared<sros::core::CommonMsg>("TOPIC_UPDATA_SRC_REPLY");
        msg_reply->int_0_ = -1;  // 代表升级失败
        sendMsg(msg_reply);
    }

    updateUpgradeRecord(UPGRADE_FLAG_SRC, upgrade_result, src_sdk->getSRCVersion(), src_sdk->getSRCVersionStr());
}

void UpgradeModule::onIAPUpgradeResult(int result) {
    LOG(INFO) << "onIAPUpgradeResult: " << result;
    int32_t upgrade_result = UPGRADE_FAILED;
    if (result == UPGRADED_IAP_STEP1) {
        LOG(WARNING) <<"device upgrade send data failed, restart sros and restart sending data ...";
        upgrade_reset_sros_ = true;
        recordUpdateIap(SROS_UPGRADED_IAP_STEP1);
    } else if(result == 1) {
        LOG(INFO) << "device upgrade got firmware data done.";
        if(iap_type_ == TypeSpu) {
            recordUpdateIap(SROS_UPGRADED_IAP_STEP2);
        } else if(iap_type_ == TypeBu100) {
            recordUpdateIap(SROS_UPGRADED_IAP_BU100_STEP2);
        }
        upgrade_result = UPGRADE_SUCCESS;
    } else {
        LOG(INFO) << "device upgrade failed, clear /sros/.update_record_iap";
        recordUpdateIap(0);
        auto msg_reply = std::make_shared<sros::core::CommonMsg>("TOPIC_UPDATA_IAP_REPLY");
        msg_reply->int_0_ = -1;  // 代表升级失败
        sendMsg(msg_reply);
    }
    is_in_upgrade_ = false;
    LOG(INFO) << "onIAPUpgradeResult done";
    // updateUpgradeRecord(UPGRADE_FLAG_IAP, upgrade_result, 0, "IAP result version");
    if (upgrade_reset_sros_) {  // spu100 不需要重启sros, 其他自研硬件的升级,成功后要重启SROS完成最后的升级
        LOG(INFO) << "upgrade done, need reset sros ...";
        auto reset_msg = std::make_shared<sros::core::CommandMsg>(getName());
        reset_msg->command = sros::core::CMD_RESET_SROS;
        sendMsg(reset_msg);
    }
}

void UpgradeModule::onVSCUpgradeResultMsg(sros::core::base_msg_ptr m) {
    auto msg = std::dynamic_pointer_cast<sros::core::CommonMsg>(m);

    const int &result = msg->int_0_;

    LOG(INFO) << "onVSCUpgradeResultMsg: " << result;
    int32_t upgrade_result = UPGRADE_FAILED;
    if (result == 1) {
        upgrade_result = UPGRADE_SUCCESS;
    } else {
        auto msg_reply = std::make_shared<sros::core::CommonMsg>("TOPIC_UPDATA_IAP_REPLY");
        msg_reply->int_0_ = -1;  // 代表升级失败
        sendMsg(msg_reply);
    }
    is_in_upgrade_ = false;
    LOG(INFO) << "onVSCUpgradeResultMsg done";
    // updateUpgradeRecord(UPGRADE_FLAG_SRC, upgrade_result, src_sdk->getSRCVersion(), src_sdk->getSRCVersionStr());
    if (upgrade_reset_sros_) {  // 是外部请求的升级才需要重启sros
        LOG(INFO) << "upgrade done, need reset sros ...";
        auto reset_msg = std::make_shared<sros::core::CommandMsg>(getName());
        reset_msg->command = sros::core::CMD_RESET_SROS;
        sendMsg(reset_msg);
    }
}

int UpgradeModule::checkUpdateResult() {
    uint32_t flag = 0;
    std::ifstream in(SROS_UPDATE_RECORD, std::ifstream::in | std::ifstream::binary);
    if (!in) {
        return 0;
    }
    in.read(reinterpret_cast<char *>(&flag), sizeof(uint32_t));
    in.close();

    if (flag == SROS_UPGRADED) {
        // check SROS update
        std::string cmd_str = string("cat ") + SROS_UPDATE_LOG + "|awk 'END {print $4}'";
        char results[10];
        FILE *fp = popen(cmd_str.c_str(), "r");
        if (!fp) {
            return 3;
        }
        fread(results, 1, 10, fp);
        pclose(fp);

        if (0 == strncmp(results, "fail", 4)) {
            recordUpdate(false);
            return -1;
        } else if (0 == strncmp(results, "success", 7)) {
            recordUpdate(false);
            return 1;
        } else {
            recordUpdate(false);
            return 2;
        }
    }

    return 0;
}


int UpgradeModule::handleIapUpdateResult() {
    // check IAP update
    uint32_t flag = 0;
    std::ifstream in(SROS_IAP_UPDATE_RECORD, std::ifstream::in | std::ifstream::binary);
    if (!in) {
        return 0;
    }
    in.read(reinterpret_cast<char *>(&flag), sizeof(uint32_t));
    in.close();
    LOG(INFO) << "handleIapUpdateResult flag: 0x" << std::hex << flag;
    if(flag == SROS_UPGRADED_IAP_STEP1) {
        LOG(INFO) << "[update] continue to run SPU100 IAP upgrade ...";
        upgrade_reset_sros_ = true; // test test 
        if(!updateDeviceIAP()) {
            LOG(ERROR) << "on run() : update Device IAP error!!!";
        }
    } else if(flag == SROS_UPGRADED_IAP_STEP2) {
        LOG(INFO) << "[update] run SPU100 IAP result success";
        recordUpdateIap(0);
        upgrade_reset_sros_ = false; // test test 
        updateDeviceIapTest(sros::device::IAP_DEV_SPU100);
    } else if (flag == SROS_UPGRADED_IAP_BU100_STEP2) {
        LOG(INFO) << "[update] run Bu100 IAP result success";
        recordUpdateIap(0);
        upgrade_reset_sros_ = false; // test test 
        updateDeviceIapTest(sros::device::IAP_DEV_BU100);
    }
    return 0;
}

void UpgradeModule::recordUpdate(bool is_update) {
    uint32_t flag = is_update ? SROS_UPGRADED : SROS_DEFAULT;
    std::ofstream out(SROS_UPDATE_RECORD, std::ofstream::out | std::ofstream::binary);
    if (!out) {
        return;
    }
    out.write(reinterpret_cast<char *>(&flag), sizeof(uint32_t));
    out.close();
}

void UpgradeModule::recordUpdateIap(const uint32_t flag_param) {
    uint32_t flag = flag_param;
    LOG(INFO) << "recordUpdateIap flag: " << std::hex << flag;
    std::ofstream out(SROS_IAP_UPDATE_RECORD, std::ofstream::out | std::ofstream::binary);
    if (!out) {
        return;
    }
    out.write(reinterpret_cast<char *>(&flag), sizeof(uint32_t));
    out.close();
}

void UpgradeModule::loopCheckIapResult() {
    uint32_t time_out = 0;
    while (true) {

        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
        LOG(INFO) << "loopCheckIapResult " << time_out;
        if(!is_in_upgrade_) {
            LOG(INFO) << "loopCheckIapResult upgrade done time:" << time_out;
           break;
        }
        if(time_out ++ > 60) {
            LOG(WARNING) << "loopCheckIapResult time out error!!!";
            upgrade_reset_sros_ = true;
            onIAPUpgradeResult(0);
            break;
        }
    }
}

void UpgradeModule::notifyUpdateResult() {
    auto msg = make_shared<sros::core::NotificationMsg>("TOPIC_NOTIFY");
    msg->notify_type = sros::core::NotificationMsg::NOTIFY_UPDATE_FINISHED;
    msg->param_int = update_result_;
    update_result_ = 0;
    sendMsg(msg);
}

bool UpgradeModule::createUpgradeRecord(int32_t type_code, const std::string &username, uint32_t last_version_int,
                                        const std::string &last_version_str, const std::string &upgrade_file_path) {
    try {
        std::lock_guard<std::recursive_mutex>  db_mutex(g_db_mutex);

        std::string sql = "INSERT INTO " + TABLE_UPGRADE_RECORD +
                          " (type_code, username, last_version_int, last_version_str, file_path, start_time) VALUES (" +
                          std::to_string(type_code) + ", '" + username  + "', " + std::to_string(last_version_int) + ", '" +
                          last_version_str + "', '" + upgrade_file_path + "', " + std::to_string(core::util::get_timestamp_in_ms()) + ");";
        LOG(INFO) <<  sql;
        return db_->exec(sql) > 0;
    } catch (std::exception &e) {
        LOG(ERROR) << "Catch a sql exception: " << e.what();
        return false;
    }
}

bool UpgradeModule::updateUpgradeRecord(int32_t type_code, int32_t result,
                                        uint32_t cur_version, const std::string &cur_version_str) {
    try {
        std::lock_guard<std::recursive_mutex>  db_mutex(g_db_mutex);

        std::string sql = "UPDATE " + TABLE_UPGRADE_RECORD + " SET result=" + std::to_string(result) + ", cur_version_int="
                + std::to_string(cur_version) + ", cur_version_str='" + cur_version_str + "', finish_time=" +
                std::to_string(core::util::get_timestamp_in_ms()) +
                " WHERE cur_version_int=" + std::to_string(0) + " AND type_code=" + std::to_string(type_code) + ";";
        LOG(INFO) <<  sql;
        return db_->exec(sql) > 0;
    } catch (std::exception &e) {
        LOG(ERROR) << "Catch a sql exception: " << e.what();
        return false;
    }
}

bool UpgradeModule::updateUpgradeRecordFilePath(const std::string &old_path, const std::string &new_path) {
    try {
        std::lock_guard<std::recursive_mutex>  db_mutex(g_db_mutex);

        std::string sql = "UPDATE " + TABLE_UPGRADE_RECORD + " SET file_path='" + new_path +
                          "' WHERE file_path='" + old_path + "';";
        LOG(INFO) <<  sql;
        return db_->exec(sql) > 0;
    } catch (std::exception &e) {
        LOG(ERROR) << "Catch a sql exception: " << e.what();
        return false;
    }
}

void UpgradeModule::createBackupPath() {
    std::string cmd_str = "mkdir -p " + BACKUP_SROS_UPGRADE_PATH;
    systemWrapper(cmd_str);
//    cmd_str = "mkdir -p " + BACKUP_SRC_UPGRADE_PATH;
//    systemWrapper(cmd_str);
}

void UpgradeModule::backupUpgradeFile(const std::string &source_file, const std::string &target_file) {
    std::string cmd_str = "cp " + source_file + " " + target_file;
    systemWrapper(cmd_str);
}

std::string UpgradeModule::calculateFileMd5(const std::string &filepath) {
    std::ifstream in(filepath, std::ifstream::in | std::ifstream::binary);
    if (!in) {
        LOG(ERROR) << "open file error! filepath:" << filepath;
        return "";
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
    return md5.hexdigest();
}

std::string UpgradeModule::getFilePostfix(const std::string &file_path) {
    return file_path.substr(file_path.find_first_of('.'));
}

void UpgradeModule::getOrderFilesInDir(const std::string &dir, const std::string &suffix, std::vector<fs::path> &file_list) {
    fs::recursive_directory_iterator end;
    for (fs::recursive_directory_iterator iterator(dir); iterator != end; ++iterator) {
        fs::path file_path = *iterator;
        if (fs::is_directory(file_path)) {
            continue;
        }

        if (!fs::is_regular_file(file_path)) {
            continue;
        }
        if (suffix != "" && fs::extension(file_path) != suffix) {
            continue;
        }
        file_list.push_back(file_path);
    }
    sort(file_list.begin(), file_list.end(), sortByDatetime);
}

void UpgradeModule::removeOldUpgradeFile() {
    std::vector<fs::path> file_list;
    getOrderFilesInDir(BACKUP_SROS_UPGRADE_PATH, "", file_list);
    if (file_list.size() > MAX_UPGRADE_FILE_BACKUP) {
        for (int16_t i = 0; i < file_list.size() - MAX_UPGRADE_FILE_BACKUP; i++) {
            std::string file_path = file_list[i].string();
            try {
                fs::remove(fs::path(file_path));
                updateUpgradeRecordFilePath(file_path, "");
            } catch (const fs::filesystem_error& ex) {
                LOG(ERROR) << "Remove file error: " << ex.what();
            }
        }
    }
}

bool UpgradeModule::syncSrcParamToMc() {
    auto &setting = Settings::getInstance();
    ItemInfoLists configs = setting.getItemInfoListOfClass("src");

    for (auto config : configs) {
        auto strSrcItem = config.key;
        if(strSrcItem == "src.motordrive_type" || strSrcItem == "src.base_type") {
            continue;
        }
        auto strMcItem = strSrcItem.replace(0,4,"mc.");
        setting.setValue(strMcItem,config.value);
        LOG(INFO) << "strMcItem:" << strMcItem <<", setValue:" << config.value;
    }
    return  true;
}

bool UpgradeModule::syncMcParamToSrc() {
    auto &setting = Settings::getInstance();
    ItemInfoLists configs = setting.getItemInfoListOfClass("mc");

    for (auto config : configs) {
        auto strMcItem = config.key;
        if(strMcItem == "mc.motordrive_type" || strMcItem == "mc.base_type") {
            continue;
        }
        auto strSrcItem = strMcItem.replace(0,3,"src.");
        setting.setValue(strSrcItem,config.value);
        LOG(INFO) << "strSrcItem:" << strSrcItem << ", setValue:" << config.value;
    }
    return  true;
}

}
