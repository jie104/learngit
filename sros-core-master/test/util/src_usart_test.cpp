/**
 * @file src_test
 *
 * @author caoyan
 * @date 2020-10-20.
 *
 * @describe  src串口超时测试
 *
 * @copyright Copyright (c) 2019 Standard Robots Co., Ltd. All rights reserved.
 */

#include <gtest/gtest.h>
#include <future>
#include <ostream>
#include <vector>
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

class SrcUsartTest : public testing::Test {
 protected:
    void SetUp() override {
        //确保sros未启动
        auto result_str = execShell("ps -ef | grep -E '/sros/bin/sros|\\./sros' | wc -l");
        auto current_sros_count = std::stoi(result_str);
        LOG(INFO) << "current sros count is " << current_sros_count;
        ASSERT_GT(3, current_sros_count);

        //开始连接
        sros::core::Settings &settings = Settings::getInstance();
        //string port = settings.getValue<string>("main.src_uart_port", "/dev/ttyTHS2");
        //string port = "/dev/ttyUSB0";
        string port = "/dev/ttyTHS2";
        //auto baud_rate = settings.getValue<unsigned int>("main.src_uart_baud_rate", 460800);
        auto baud_rate = 460800;
        bool ret = src_sdk->connect(port, baud_rate);
        ASSERT_TRUE(ret); // 此处两个链接同时打开串口似乎可行
        //初始化
        //src_sdk->initState();
        //src_sdk->sendCommandMsg(COMMAND_CONNECT);

        //先去获取版本号，再去发起连接
        src_sdk->getSrcProtoVersion();

        // 开机后重启车辆，防止信息sros重启过，src还保留上一次
        if (src_sdk->getVersion() == sdk::SRC_PROTO_VERSION_V2) {
            src_sdk->sendCommandMsg(COMMAND_CPU_RESET);
            boost::this_thread::sleep_for(boost::chrono::milliseconds(1000));
        }

        src_sdk->sendCommandMsg(COMMAND_CONNECT);
    }

    void TearDown() override { src_sdk->disconnect(); }
};

TEST_F(SrcUsartTest, OverTimeTest) {
    int value = 0;
    std::vector<int> vec;

    if (src_sdk->getVersion() == sdk::SRC_PROTO_VERSION_V2) {
        //下发运动控制参数配置寄存器
        const uint16_t SET_RUN = 0x0001;
        vec.clear();
        vec.push_back(0x0d);
        vec.push_back(0xa1);
        vec.push_back(0xf0);
        vec.push_back(0x0a);
        vec.push_back(0x0b);
        vec.push_back(0x0c);
        vec.push_back(0x0d);
        ASSERT_TRUE(src_sdk->setParameters(SET_RUN, vec));
        //下发动作控制参数配置寄存器
        const uint16_t SET_ACT = 0x0201;
        vec.clear();
        vec.push_back(0x0d);
        vec.push_back(0xa1);
        vec.push_back(0xf0);
        vec.push_back(0x0a);
        vec.push_back(0x0b);
        vec.push_back(0x0c);
        vec.push_back(0x0d);
        ASSERT_TRUE(src_sdk->setParameters(SET_ACT, vec));

        //执行子任务
        const uint16_t EXE_TASK = 0x3000;
        vec.clear();
        vec.push_back(0x24);
        vec.push_back(0xa1);
        vec.push_back(0xf0);
        vec.push_back(0x0a);
        vec.push_back(0x0b);
        vec.push_back(0x0c);
        vec.push_back(0x0d);
        ASSERT_TRUE(src_sdk->setParameters(EXE_TASK, vec));

        //休眠
        sleep(10);

        int timeout_ms = 100;

        for(int i = 0; i < 40000; i++) {

            src_sdk->sendCommandMsg(COMMAND_SET_NAVALGORITHM, 3, 0);

            //GET
            const uint16_t GET_1 = 0x2011;
            value = 12;
            auto ret_get_1 = src_sdk->getParameters(GET_1, value, timeout_ms);
            //std::cout << "ret_get_1.size(): " << ret_get_1.size() << std::endl;
            if(ret_get_1.size() != 12) {
                std::cout << "ret_get_1.size(): " << ret_get_1.size() << std::endl;
            }
            ASSERT_TRUE(ret_get_1.size() == 12);

            //SET 0x2808
            const uint16_t SET_1 = 0x2808;
            vec.clear();
            vec.push_back(0x01);
            vec.push_back(0x02);
            vec.push_back(0x03);
            vec.push_back(0x04);
            vec.push_back(0x05);
            vec.push_back(0x06);
            vec.push_back(0x07);
            vec.push_back(0x08);
            ASSERT_TRUE(src_sdk->setParameters(SET_1, vec));

            //SET
            const uint16_t SET_2 = 0x2800;
            vec.clear();
            vec.push_back(0x00);
            vec.push_back(0x00);
            vec.push_back(0x00);
            vec.push_back(0x00);
            vec.push_back(0x00);
            vec.push_back(0x00);
            vec.push_back(0x00);
            vec.push_back(0x00);
            ASSERT_TRUE(src_sdk->setParameters(SET_2, vec));

            //GET
            const uint16_t GET_2 = 0x2400;
            value = 15;
            //ASSERT_TRUE(src_sdk->getParameters(GET_2, value, 32));
            auto ret_get_2 = src_sdk->getParameters(GET_2, value, timeout_ms);
            if(ret_get_2.size() != 15) {
                std::cout << "ret_get_2.size(): " << ret_get_2.size() << std::endl;
            }
            ASSERT_TRUE(ret_get_2.size() == 15);

            //SET
            const uint16_t SET_3 = 0x2800;
            vec.clear();
            vec.push_back(0x00);
            vec.push_back(0x00);
            vec.push_back(0x00);
            vec.push_back(0x00);
            vec.push_back(0x00);
            vec.push_back(0x00);
            vec.push_back(0x00);
            vec.push_back(0x00);
            ASSERT_TRUE(src_sdk->setParameters(SET_3, vec));

            //SET
            const uint16_t SET_4 = 0x2800;
            vec.clear();
            vec.push_back(0x00);
            vec.push_back(0x00);
            vec.push_back(0x00);
            vec.push_back(0x00);
            vec.push_back(0x00);
            vec.push_back(0x00);
            vec.push_back(0x00);
            vec.push_back(0x00);
            ASSERT_TRUE(src_sdk->setParameters(SET_4, vec));

            //GET
            const uint16_t GET_3 = 0x2890;
            value = 12;
            //ASSERT_TRUE(src_sdk->getParameters(GET_3, value, 32));
            auto ret_get_3 = src_sdk->getParameters(GET_3, value, timeout_ms);

            if(ret_get_3.size() != 12) {
                std::cout << "ret_get_3.size(): " << ret_get_3.size() << std::endl;
            }

            ASSERT_TRUE(ret_get_3.size() == 12);

            //GET
            const uint16_t GET_4 = 0x28d0;
            value = 20;
            //ASSERT_TRUE(src_sdk->getParameters(GET_4, value, 32));
            auto ret_get_4 = src_sdk->getParameters(GET_4, value, timeout_ms);

            if(ret_get_4.size() != 20) {
                std::cout << "ret_get_4.size(): " << ret_get_4.size() << std::endl;
            }
            ASSERT_TRUE(ret_get_4.size() == 20);

            //SET
            const uint16_t SET_5 = 0x2800;
            vec.clear();
            vec.push_back(0x00);
            vec.push_back(0x00);
            vec.push_back(0x00);
            vec.push_back(0x00);
            vec.push_back(0x00);
            vec.push_back(0x00);
            vec.push_back(0x00);
            vec.push_back(0x00);
            ASSERT_TRUE(src_sdk->setParameters(SET_5, vec));

            //GET
            const uint16_t GET_5 = 0x1018;
            value = 20;
            //ASSERT_TRUE(src_sdk->getParameters(GET_5, value, 32));
            auto ret_get_5 = src_sdk->getParameters(GET_5, value, timeout_ms);

            if(ret_get_5.size() != 20) {
                std::cout << "ret_get_5.size(): " << ret_get_5.size() << std::endl;
            }

            ASSERT_TRUE(ret_get_5.size() == 20);

            //std::this_thread::sleep_for(std::chrono::milliseconds(40));

        }
    } else {
        src_sdk->initProtoDo();

        //休眠
        sleep(10);

        int timeout_ms = 100;
        int count = 400000 * 3 * 10;

        for(int i = 0; i < count; i++) {

            const uint16_t ADDR_MONITOR_SRC_STATUS = 0x1540;
            const int register_count = 10;  // 寄存器个数

            auto results = src_sdk->getParameters(ADDR_MONITOR_SRC_STATUS, register_count, timeout_ms);
            if (results.size() != register_count) {
                std::cout << "results.size(): " << results.size() << std::endl;
            }
            ASSERT_TRUE(results.size() == 10);
            //std::this_thread::sleep_for(std::chrono::milliseconds(40));

        }
    }



}
