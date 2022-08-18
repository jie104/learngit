/**
 * @file src_test
 *
 * @author pengjiali
 * @date 19-12-17.
 *
 * @describe 本测试用例需要在tk1中运行
 *
 * @copyright Copyright (c) 2019 Standard Robots Co., Ltd. All rights reserved.
 */

#include <gtest/gtest.h>
#include <future>
#include <ostream>
#include "core/settings.h"
#include "core/src.h"
#include "core/util/utils.h"
#include "src/sdk/src_sdk.h"
#include "src/sdk/src_upgrade.h"
#include "src/sdk/src_sdk_v2.h"

using namespace sros::core;
using namespace std;
using namespace sdk;
namespace fs = boost::filesystem;

class SrcTest : public testing::Test {
 protected:
    void SetUp() override {
        // 确保sros未启动
        auto result_str = execShell("ps -ef | grep -E '/sros/bin/sros|\\./sros' | wc -l");
        auto current_sros_count = std::stoi(result_str);
        LOG(INFO) << "current sros count is " << current_sros_count;
        ASSERT_GT(3, current_sros_count);

        src_sdk->setStateCallback(boost::bind(&SrcTest::onSRCState, this, _1));
        src_sdk->setUpgradeCallback(boost::bind(&SrcTest::onSRCUpgradeResult, this, _1));

        sros::core::Settings &settings = Settings::getInstance();
        //        string port = settings.getValue<string>("main.src_uart_port", "/dev/ttyTHS2");
        string port = "/dev/ttyUSB0";
        auto baud_rate = settings.getValue<unsigned int>("main.src_uart_baud_rate", 460800);

        bool ret = src_sdk->connect(port, baud_rate);
        ASSERT_TRUE(ret); // 此处两个链接同时打开串口似乎可行

        src_sdk->initState();

        // 检测200ms内src将版本号上传
        std::future<int> future = promise_.get_future();
        std::future_status status = future.wait_for(std::chrono::milliseconds(2000));

        // 若src升级失败，完成被动升级
        if (status != std::future_status::ready) {
            LOG(INFO) << "src update failed!, waiting active upgrade!";
            ASSERT_TRUE(version_.empty());
            promise_ = std::promise<int>();
            // 60s内完成升级src
            std::future<int> future = promise_.get_future();
            std::future_status status = future.wait_for(std::chrono::seconds(60));
            ASSERT_EQ(status, std::future_status::ready);
        } else {
            ASSERT_FALSE(version_.empty());
        }
    }

    void TearDown() override { src_sdk->disconnect(); }

    void onSRCState(const SRCState &state) {
        if (!is_src_connected_) {
            LOG(INFO) << "src state is " << (int)state.src_state;
            version_ = src_sdk->getSRCVersionStr();
            LOGGER(INFO, SROS) << "SRC connected! version: " << version_;
            promise_.set_value(1);
            is_src_connected_ = true;
        }
        src_state_ = state;
    }

    void onSRCUpgradeResult(int result) {
        EXPECT_EQ(result, 1);
        promise_.set_value(1);
    }

    void upgrade_once(const std::string &upgrade_file_path) {
        LOG(INFO) << "upgreade once! upgrade_file_path is " << upgrade_file_path;
        promise_ = std::promise<int>();
        bool ret = src_sdk->upgradeRequest(upgrade_file_path);
        ASSERT_TRUE(ret);

        // 60s内完成升级src
        std::future<int> future = promise_.get_future();
        std::future_status status = future.wait_for(std::chrono::seconds(60));
        ASSERT_EQ(status, std::future_status::ready);

        // 检查默认的升级包软链接
        std::string dis_upgrade_file_path;
        if (fs::is_symlink(upgrade_file_path)) {
            dis_upgrade_file_path = fs::read_symlink(upgrade_file_path).string();
        } else {
            dis_upgrade_file_path = upgrade_file_path;
        }
        ASSERT_EQ(fs::path(dis_upgrade_file_path).filename(),
                  fs::read_symlink(default_upgrade_file_symlink).filename());

        // 检测升级日志
        auto result_str = execShell("tail -n 1 /sros/stm32_update.log | awk '{print $8}'");
        ASSERT_EQ(result_str, "success");
        auto dis_upgreade_file_str = execShell("tail -n 1 /sros/stm32_update.log | awk '{print $7}'");
        ASSERT_EQ(dis_upgreade_file_str, dis_upgrade_file_path);
    }

    bool is_src_connected_ = false;
    std::string version_;
    std::promise<int> promise_;
    SRCState src_state_;
};

TEST_F(SrcTest, StateTest) {
    ASSERT_FALSE(src_sdk->getSRCVersionStr().empty());
    EXPECT_EQ(src_state_.gpio_input, 0);

//    ASSERT_NE(g_src_state.total_power_cycle, 0);
//    ASSERT_NE(g_src_state.total_poweron_time, 0);
//    EXPECT_NE(g_src_state.total_mileage, 0);

    // 测试GPIO
    EXPECT_EQ(src_state_.gpio_input, 0);
    EXPECT_EQ(src_state_.gpio_output, 0);
    src_sdk->setGPIOOuput(0xFF);
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    ASSERT_EQ(src_state_.gpio_output, 0xFF);
}

TEST_F(SrcTest, CmdTest) {
    src_sdk->setSpeedLevel(1);
    src_sdk->executeAction(1, 1, 0, 0);
}

TEST_F(SrcTest, ConfigTest) {
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    auto testSrcConfigFunc = [&](std::string class_name, int config_start_id, int src_start_register_addr) {
      auto configs = Settings::getInstance().getItemInfoListOfClass(class_name);

      if (configs.empty()) {
          return;
      }

      int reg_count = configs.back().id - config_start_id + 1;
      auto reg_values = src_sdk->getParameters(src_start_register_addr, reg_count, 1000);
      for (auto i = 0; i < configs.size(); ++i) {
          int value = 0;
          try {
              value = std::stoi(configs[i].value);
          } catch (std::exception &e) {
              LOG(ERROR) << e.what();
          }
//          LOG(INFO) << configs[i].key << ": " << value << " vs " << reg_values[configs[i].id - config_start_id];
          ASSERT_EQ(reg_values[configs[i].id - config_start_id], value);
      }
    };

    testSrcConfigFunc("srtos", SRTOS_CONFIG_START_ID, SRTOS_ADDR_SYSTEM_CONFIG);
    testSrcConfigFunc("mc", SRTOS_CONFIG_MC_ID, SRTOS_ADDR_MC_CONFIG);
    testSrcConfigFunc("ac", SRTOS_CONFIG_AC_ID, SRTOS_ADDR_AC_CONFIG);
}

TEST_F(SrcTest, UpgradeTest_DefaultUpgradeFile) {
    ASSERT_TRUE(fs::exists(default_upgrade_file_symlink));
    ASSERT_TRUE(fs::is_symlink(default_upgrade_file_symlink));

    {
        SCOPED_TRACE("upgrade default_upgrade_file once");
        ASSERT_NO_FATAL_FAILURE(upgrade_once(default_upgrade_file_symlink.string()));
    }
    {
        SCOPED_TRACE("upgrade default_upgrade_file twice");
        ASSERT_NO_FATAL_FAILURE(upgrade_once(default_upgrade_file_symlink.string()));
    }
    {
        SCOPED_TRACE("upgrade default_upgrade_file_symlink once");
        ASSERT_NO_FATAL_FAILURE(upgrade_once(fs::read_symlink(default_upgrade_file_symlink).string()));
    }
    {
        SCOPED_TRACE("upgrade default_upgrade_file_symlink twice");
        ASSERT_NO_FATAL_FAILURE(upgrade_once(fs::read_symlink(default_upgrade_file_symlink).string()));
    }
}

/**
 * 若要循环测试：./src_test --gtest_repeat=3 --gtest_filter=SrcTest.UpgradeTest_AB --gtest_break_on_failure
 * --gtest_output="xml:/sros/test/util/SrcTest_Report.xml"
 */
TEST_F(SrcTest, UpgradeTest_AB) {
    const std::string resouces_path = "/sros/test/util/resources";
    const std::string a = "stm32_upgrade_test_a.bin";
    const std::string b = "stm32_upgrade_test_b.bin";
    const auto a_filepath = boost::filesystem::path(resouces_path) / boost::filesystem::path(a);
    const auto b_filepath = boost::filesystem::path(resouces_path) / boost::filesystem::path(b);
    const auto a_symlink_filepath = boost::filesystem::path(resouces_path) / fs::read_symlink(a_filepath);
    const auto b_symlink_filepath = boost::filesystem::path(resouces_path) / fs::read_symlink(b_filepath);
    ASSERT_TRUE(fs::exists(a_filepath));
    ASSERT_TRUE(fs::is_symlink(a_filepath));
    ASSERT_TRUE(fs::exists(a_symlink_filepath));
    ASSERT_TRUE(fs::exists(b_filepath));
    ASSERT_TRUE(fs::is_symlink(b_filepath));
    ASSERT_TRUE(fs::exists(b_symlink_filepath));

    {
        SCOPED_TRACE("upgrade a");
        ASSERT_NO_FATAL_FAILURE(upgrade_once(a_symlink_filepath.string()));
    }
    {
        SCOPED_TRACE("upgrade b");
        ASSERT_NO_FATAL_FAILURE(upgrade_once(b_symlink_filepath.string()));
    }
}
