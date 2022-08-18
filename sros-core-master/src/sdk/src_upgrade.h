/**
 * describe: 封装了src升级流程
 * Created by pengjiali on 18-12-13.
 **/

#ifndef SROS_SRC_UPGRADE_H
#define SROS_SRC_UPGRADE_H

#include <stdint.h>
#include <string>
#include <vector>
#include <boost/filesystem.hpp>

namespace sdk {

enum Stm32State {
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

enum Stm32Mode {
    ST_ACTIVE,
    ST_PASSIVE,
    ST_UPGRADE_RESULT,
};

class SrcUpgrade {
 public:
    bool setUpgradeFilePath(const std::string &upgrade_bin_file_path);
    void setUpgradeResult(int result);  // 设置升级结束的结果
    void onIAPRequest(const std::vector<uint8_t> &dataX);
    bool isInUpgrade() const { return is_in_upgrade_; };  // 查询现在是否正在升级
    bool syncSrcParamToMc(void);             // 同步db src参数到srtos的mc
    
 private:
    void handleRequestConnection();
    void handleRequestInfo();
    void handleRequestData(uint32_t index);
    void handleRequestResult(uint32_t result);
    void handleRequestTest();
    void sendSRCUpgradeTest();

    void recordUpdate(bool is_update);
    void backupUpgradeFiles();

    void push_uint32t(std::vector<uint8_t> &src, uint32_t value);

    bool isStm32UpgradeFileReady();
    std::string readMD5String(const std::string &path);
    int getFileSize(const std::string &path);
    std::vector<uint8_t> makeUpgradeInfo(const std::string &version, const std::string &md5, int size, uint32_t stamp);
    bool checkMd5sum(const std::string &filePath, const std::string &md5sum);
    std::vector<std::string> splitWithStl(const std::string &str, const std::string &pattern);
    void hashstring2vector(const std::string &src, std::vector<uint8_t> &data);
    uint32_t makeUpgradeInfoCrc(std::vector<uint8_t> &data);
    uint8_t charToInt(char c);

    Stm32State state_;

    std::string update_info_;

    std::string upgrade_bin_file_;
    std::string upgrade_md5sum_file_;
    bool is_in_upgrade_ = false;  // 标记是否正在升级
};

}  // namespace sdk

#endif  // SROS_SRC_UPGRADE_H
