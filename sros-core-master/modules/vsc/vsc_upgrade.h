/**
 * 
 * @copyright   : Copyright (C) 2019 Standard-robots, Inc
 * @file        : vsc_upgrade.h
 * @description : 
 * @author      : EHL (linenhui@standard-robots.com / enhuilyn@qq.com)
 * @date        : 2022/05/12
 * @brief       : V1.0.0 
 */

#ifndef SROS_VSC_UPGRADE_H_
#define SROS_VSC_UPGRADE_H_

#include <mutex>
#include <stdint.h>
#include <string>
#include <vector>
#include <boost/filesystem.hpp>

#include "core/core.h"
#include "core/msg/str_msg.hpp"
#include "core/msg/command_msg.hpp"
#include "core/task/task_manager.h"
#include "core/usart/connection.hpp"
#include "core/usart/frame_v1.h"
#include "modules/security/security_state_handle.h"
#include "src/sdk/protocol/all_msg.h"
#include "modules/upgrade/iap_upgrade.h"

namespace vsc {

enum Vsc300State {
    IAP_NONE = 0x00,
    IAP_CONN = 0xfa,
    IAP_REQUEST = 0xf1,
    IAP_DATA = 0xf2,  // UpgradeModule is in transmitting mode
    IAP_RESULT = 0xf3, // 结果校验
    IAP_ACK = 0xf4,
    IAP_NACK = 0xf5,
    IAP_UTEST = 0xf6,
    IAP_DISCON = 0xaf,
};

enum Vsc300Mode {
    ST_ACTIVE,
    ST_PASSIVE,
    ST_UPGRADE_RESULT,
};

typedef boost::function<void(int)> IapUpgradeCallbackFunc_t;

using namespace std;

class VSCUpgrade{
 public:
    VSCUpgrade();

    virtual ~VSCUpgrade();

    void init(shared_ptr<usart::Connection<usart::FrameV1<>>> usart_ptr) {usart_ptr_ = usart_ptr;}

    bool setUpgradeFilePath(const std::string &upgrade_bin_file_path);
    void setUpgradeResult(int result);  // 设置升级结束的结果
    void onIAPRequest(const std::vector<uint8_t> &dataX);
    bool isInUpgrade() const { return is_in_upgrade_; };  // 查询现在是否正在升级
    bool syncSrcParamToMc(void);             // 同步db src参数到srtos的mc
    bool upgradeRequest(const std::string &upgrade_bin_file_path);
    void handleIAPRequest(const std::vector<uint8_t> &data);
    void checkUpgradeResult(uint32_t result);
    void setUpgradeCallback(IapUpgradeCallbackFunc_t callback) { vsc_upgrade_callback_f_ = callback; }

 private:
    void handleRequestConnection();
    void handleRequestInfo();
    void handleRequestData(uint32_t index);
    void handleRequestResult(uint32_t result);
    void handleRequestTest();
    void sendUpgradeTest();

    void recordUpdate(bool is_update);
    void backupUpgradeFiles();

    void push_uint32t(std::vector<uint8_t> &src, uint32_t value);

    bool isIapUpgradeFileReady();
    std::string readMD5String(const std::string &path);
    int getFileSize(const std::string &path);
    int getFileVersion(const std::string &path, std::string &versionStr, uint32_t &versionNum);
    std::vector<uint8_t> makeUpgradeInfo(const std::string &version, const std::string &md5, int size, uint32_t stamp);
    bool checkMd5sum(const std::string &filePath, const std::string &md5sum);
    std::vector<std::string> splitWithStl(const std::string &str, const std::string &pattern);
    void hashstring2vector(const std::string &src, std::vector<uint8_t> &data);
    uint32_t makeUpgradeInfoCrc(std::vector<uint8_t> &data);
    uint8_t charToInt(char c);

    void sendCommandMsg(COMMAND_t command, int32_t param0, int32_t param1);

    void sendCommandMsg(COMMAND_t command);

    void sendMsg(src::BaseMsg_ptr msg);
    void sendMsg(const std::vector<uint8_t> &data);

    bool sendIAPdata(const std::vector<uint8_t> &data);

    void upgradeTest();
    
    void upgradSrcTest();
    uint32_t command_msg_seq_no_;  // CommandMsg的序列号

    uint32_t firmware_file_version_;
    Vsc300State state_;

    std::string update_info_;

    std::string upgrade_bin_file_;
    std::string upgrade_md5sum_file_;
    bool is_in_upgrade_ = false;  // 标记是否正在升级

    uint32_t total_pkg_num_;

    IapUpgradeCallbackFunc_t vsc_upgrade_callback_f_;

    shared_ptr<usart::Connection<usart::FrameV1<>>> usart_ptr_;  // 串口通信抽象
    
};

} // namespace vsc

#endif  // SROS_VSC_UPGRADE_H_
