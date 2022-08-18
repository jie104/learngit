/**
 * describe: 升级模块，用于升级sros、src
 * Created by pengjiali on 18-12-12.
**/

#ifndef SROS_UPGRADE_MODULE_H
#define SROS_UPGRADE_MODULE_H

#include <vector>
#include <boost/filesystem.hpp>

#include "core/module.h"
#include "core/db/db.h"
#include "iap_upgrade.h"

namespace sros {
namespace fs = boost::filesystem;

const std::string IMPORT_UPDATE_DIR = "/sros/update/";
const std::string IMPORT_UPDATE_FILE_PATH = "/sros/update/sros-import-update.tar.bz";

bool sortByDatetime (const fs::path &p1, const fs::path &p2);

class UpgradeModule : public sros::core::Module {
public:
    UpgradeModule();

    virtual ~UpgradeModule() {}

    virtual void run() override;

private:
    void onUpdateSrosMsg(sros::core::base_msg_ptr m);

    void onUpdateSrcMsg(sros::core::base_msg_ptr m);

    void updateDeviceIapRequest(const std::string &iap_upgrade_file_path);
    void updateDeviceIapTest(const uint32_t dev_type);
    bool updateDeviceIAP();
    bool startIapUpgrade(uint32_t dev_type);

    void onTimer_5s(sros::core::base_msg_ptr m);

    bool updateSros(const std::string &file_path);

    void onSRCUpgradeResult(int result);
    void onIAPUpgradeResult(int result);
    void onVSCUpgradeResultMsg(sros::core::base_msg_ptr m);

    int checkUpdateResult();
    int handleIapUpdateResult();
    void recordUpdate(bool is_update);
    void recordUpdateIap(const uint32_t flag);
    void loopCheckIapResult();
    void notifyUpdateResult();

    std::string calculateFileMd5(const std::string &filepath);

    std::string getFilePostfix(const std::string &file_path);
    void getOrderFilesInDir(const std::string &dir, const std::string &suffix, std::vector<fs::path> &file_list);
    void removeOldUpgradeFile();

    void createBackupPath();
    void backupUpgradeFile(const std::string &source_file, const std::string &target_file);
    void backupIapUpgradeFile(const std::string &dev_name, const std::string &upgrade_file_path);

    bool createUpgradeRecord(int32_t type_code, const std::string &username,
            uint32_t last_version_int, const std::string &last_version_str,
            const std::string &upgrade_file_path);

    bool updateUpgradeRecord(int32_t type_code, int32_t result, uint32_t cur_version, const std::string &cur_version_str);
    bool updateUpgradeRecordFilePath(const std::string &old_path, const std::string &new_path);

private:
    SQLite::Database *db_;

    bool syncSrcParamToMc();
    bool syncMcParamToSrc();
    // 标记是否是外界请求升级src，若是外界请求升级src的话，升级成功后需要重启sros，主动升级是不需要重启sros的
    bool request_upgrade_src_ = false;

    // 标记自研硬件IAP升级后, 是否需要重启SROS以完成最后的升级. 其中 spu100 不需要重启sros, 其他的都需要重启sros
    bool upgrade_reset_sros_ = false;

    bool src_update_to_srtos = false;
    bool srtos_update_to_src = false;
    int update_result_;

    bool is_in_upgrade_ = false;  // 标记是否正在升级

    IapUpgrade_ptr iap_can_upgrade_ = nullptr;

    enum IapType {
        TypeNone = 0,
        TypeSpu = 1,
        TypeBu100 = 2,
        TypeVsc = 3,
    };

    enum IapType iap_type_ = TypeNone;
};

}


#endif //SROS_UPGRADE_MODULE_H
