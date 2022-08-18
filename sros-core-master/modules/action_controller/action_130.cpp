//
// Created by caoyan on 1/15/21.
//

#include "action_130.h"
#include "core/logger.h"

using namespace std;
using namespace sros::core;

namespace ac {

void Action130::doStart() {
    LOG(INFO) << "Action task  wait for io value " << action_param1_ << " on bit " << action_param0_;

    auto input_bits = action_param0_;

    responseAction130(input_bits, true);
}

void Action130::doInCancel() {
    auto bits = action_param0_;
    responseAction130(bits, false);
}

void Action130::doTimerEvent(uint64_t cur_time) {

    auto bits = action_param0_;
    auto value = action_param1_;

    uint8_t input_value = (g_state.gpio_input >> bits) & (uint8_t)0x01;

    if (input_value == value) {
        responseIOInput(bits);

        responseAction130(bits, false);

        doActionFinishSucceed();
    }
}

void Action130::responseIOInput(int input_bits) {
    LOG(INFO) << "responseIOInput()";

    sros::core::Settings &settings = sros::core::Settings::getInstance();
    auto enable_input_response = settings.getValue<string>("io.enable_input_response", "False") == "True";

    if (!enable_input_response) {
        return;
    }

    auto input_response_type = settings.getValue<string>("io.input_response_type", "FIXED");
    auto input_response_fixed_bits = settings.getValue<int>("io.input_response_fixed_bits", 0);
    auto input_response_value = settings.getValue<int>("io.input_response_value", 1);
    auto input_response_timeout = settings.getValue<int>("io.input_response_timeout", 1000);

    auto input_response_bits = input_response_type == "FIXED" ? input_response_fixed_bits : input_bits;

    LOG(INFO) << "input_response_type = " << input_response_type;
    LOG(INFO) << "input_response_fixed_bits = " << input_response_fixed_bits;
    LOG(INFO) << "input_response_value = " << input_response_value;
    LOG(INFO) << "input_response_timeout = " << input_response_timeout;
    LOG(INFO) << "input_response_bits = " << input_response_bits;

    // uint8_t ori_output_value = g_state.gpio_output;  // 保存当前gpio output值

    // // 根据需要设置的IO输出，计算新output值
    // uint8_t bits_mask = (uint8_t) ~(0x01 << input_response_bits);
    // uint8_t new_output_value =
    //     (uint8_t)((ori_output_value & bits_mask) | (input_response_value << input_response_bits));

    // LOG(INFO) << "ori_output_value = " << (int)ori_output_value;
    // LOG(INFO) << "bits_mask = " << (int)bits_mask;
    // LOG(INFO) << "new_output_value = " << (int)new_output_value;

    // src_sdk->setGPIOOuput(new_output_value);

    uint8_t value = 1 << input_response_bits;
    if(input_response_value == 0) {
        src_sdk->setGPIOOuputBits(value, 0);
    } else {
        src_sdk->setGPIOOuputBits(0, value);
    }

    std::thread t([input_response_timeout, value, input_response_value] {
      this_thread::sleep_for(chrono::milliseconds(input_response_timeout));

        // 恢复为保存的output值
        // src_sdk->setGPIOOuput(ori_output_value);
        if(input_response_value == 0) {
            src_sdk->setGPIOOuputBits(0, value);
        } else {
            src_sdk->setGPIOOuputBits(value, 0);
        }

    });
    t.detach();
}


/**
 * 当ActionTask 130开始运行后，根据参数将指定的IO口设置为指定值，当动作任务结束时，设置该IO口为相反值
 * @param input_bits
 * @param is_action_start
 */
void Action130::responseAction130(int input_bits, bool is_action_start) {
    LOG(INFO) << "responseAction130()";

    auto &settings = sros::core::Settings::getInstance();
    auto enable_action_130_response = settings.getValue<string>("io.enable_action_130_response", "False") == "True";

    LOG(INFO) << "enable_action_130_response = " << enable_action_130_response;

    if (!enable_action_130_response) {
        return;
    }

    auto action_130_response_type = settings.getValue<string>("io.action_130_response_type", "FIXED");
    auto action_130_response_fixed_bits = settings.getValue<int>("io.action_130_response_fixed_bits", 6);
    auto action_130_response_value = settings.getValue<int>("io.action_130_response_value", 1);

    auto action_130_response_bits = action_130_response_type == "FIXED" ? action_130_response_fixed_bits : input_bits;

    LOG(INFO) << "action_130_response_type = " << action_130_response_type;
    LOG(INFO) << "action_130_response_fixed_bits = " << action_130_response_fixed_bits;
    LOG(INFO) << "action_130_response_value = " << action_130_response_value;
    LOG(INFO) << "action_130_response_bits = " << action_130_response_bits;

    // uint8_t ori_output_value = g_state.gpio_output;  // 保存当前gpio output值

    // // 计算需要设置的bit值，如果动作结束，需要对bit值取反
    // auto set_value = (uint8_t)(action_130_response_value << action_130_response_bits);

    // // 设置IO output值
    // auto new_output_value =
    //     is_action_start ? (uint8_t)(ori_output_value | set_value) : (uint8_t)(ori_output_value & ~set_value);

    // LOG(INFO) << "ori_output_value = " << (int)ori_output_value;
    // LOG(INFO) << "set_value = " << (int)set_value;
    // LOG(INFO) << "new_output_value = " << (int)new_output_value;

    // src_sdk->setGPIOOuput(new_output_value);

    uint8_t value = 1 << action_130_response_bits;
    if(action_130_response_value == 0 && is_action_start
        || action_130_response_value != 0 && !is_action_start) {
        src_sdk->setGPIOOuputBits(value, 0);
    } else {
        src_sdk->setGPIOOuputBits(0, value);
    }
}

}