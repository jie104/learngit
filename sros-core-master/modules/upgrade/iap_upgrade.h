//
// Created by lhx on 16-1-29.
//

#ifndef SROS_IAP_UPGRADE_H
#define SROS_IAP_UPGRADE_H

#include <boost/function.hpp>
#include <boost/thread/condition.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/filesystem.hpp>
#include <fstream>
#include <future>
#include <list>
#include <mutex>
#include <stdint.h>
#include <string>
#include <vector>

#include "core/device/device.h"
#include "core/usart/connection.hpp"
#include "core/util/async_condition_variable.hpp"
#include "core/util/distribution_plot.h"
#include "modules/security/security_state_handle.h"
#include "core/msg/parameter_msg.hpp"
#include "core/hardware/IAP_CAN.h"

namespace sros {

const uint16_t UPGRADED_IAP_STEP1 = 0x11; // 针对自研硬件: 标记正在执行升级...
const char IAP_UPDATE_LOG[] = "/sros/log/iap_update.log";
const char IAP_BACKUP_PATH[] = "/sros/backup/upgrade/iap";
const char IAP_UPGRADE_DEFAULT_NAME[] = "iap_upgrade.bin";
const auto default_iap_upgrade_file_symlink =
    boost::filesystem::path(IAP_BACKUP_PATH) / boost::filesystem::path(IAP_UPGRADE_DEFAULT_NAME);


const char VSC_UPDATE_LOG[] = "/sros/log/vsc_update.log";
const char VSC_BACKUP_PATH[] = "/sros/backup/upgrade/vsc";
const char VSC_UPGRADE_DEFAULT_NAME[] = "vsc_upgrade.bin";
const auto default_vsc_upgrade_file_symlink =
    boost::filesystem::path(VSC_BACKUP_PATH) / boost::filesystem::path(VSC_UPGRADE_DEFAULT_NAME);


typedef boost::function<void(int)> IapUpgradeCallbackFunc_t;

using namespace std;

/**
 * Hardware devices firmware IAP upgrade
 */
class IapUpgrade {
 public:
    IapUpgrade(const uint32_t dev_type);
    virtual ~IapUpgrade() {}

    void initCanDevice(const uint32_t dev_type);
    void initUsartDevice(const uint32_t dev_type);
    
    void setUpgradeCallback(IapUpgradeCallbackFunc_t callback) { iap_upgrade_callback_f_ = callback; }
    void onDataRecieve(const uint32_t cmd_id, int response);
    bool sendCommand(const sros::device::IAP_CAN_COMMAND_t command);

    bool setUpgradeFilePath(const std::string &upgrade_bin_file_path);
    bool upgradeRequest(const std::string &upgrade_bin_file_path);
    bool upgradeSend(const std::string &upgrade_bin_file_path);
    bool upgradeTest();
    void disableDevice();

    void setUpgradeResult(int result);  // 设置升级结束的结果
    void onIAPRequest(const std::vector<uint8_t> &dataX);
    bool isInUpgrade() const { return is_in_upgrade_; };  // 查询现在是否正在升级
    int getDeviceType();

    std::string getDeviceTypeName(const uint32_t dev_type);
    std::string getDeviceVersion();
    void setDeviceVersion(int mode);
    uint32_t checkIapDeviceType(const std::string &upgrade_bin_file_path);
 private:
    void handleRequestConnection();
    bool handleRequestFileInfo();
    int handleRequestDataInfo(const uint32_t index, const uint32_t dataSize, const uint32_t crc);
    bool handleRequestData(uint32_t index);
    void handleRequestResult(uint32_t result);
    void handleRequestTest();
    void sendSRCUpgradeTest();

    void recordUpdate(bool is_update);
    void backupUpgradeFiles();

    bool isIapUpgradeFileReady();
    std::string readMD5String(const std::string &path);
    int getFileSize(const std::string &path);
    bool checkMd5sum(const std::string &filePath, const std::string &md5sum);

    std::string update_info_;
    std::string version_str_;
    uint32_t iap_host_state_;
    uint32_t iap_dev_state_;

    uint32_t dev_default_id_;
    uint32_t iap_dev_type_;
    uint32_t iap_dev_offset_;
    uint32_t total_pkg_num_;

    uint32_t pkg_index_;

    std::string upgrade_bin_file_;
    std::string upgrade_md5sum_file_;
    bool is_in_upgrade_ = false;  // 标记是否正在升级

    IapUpgradeCallbackFunc_t iap_upgrade_callback_f_;

    std::vector<sros::device::IapCan_ptr> iap_can_ptr_;
};
typedef std::shared_ptr<IapUpgrade> IapUpgrade_ptr;

}  // namespace sros

#endif  // SROS_IAP_UPGRADE_H
