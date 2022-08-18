/**
 * @file speaker.h
 *
 * @author pengjiali
 * @date 19-7-16.
 *
 * @describe
 *
 * @copyright Copyright (c) 2019 Standard Robots Co., Ltd. All rights reserved.
 */

#ifndef CORE_HARDWARE_SPEAKER_H_
#define CORE_HARDWARE_SPEAKER_H_

#include <map>
#include "core/device/IODevice.h"

namespace sros {
namespace device {

class Speaker : public IODevice {
 public:
    Speaker(const std::string &name, DeviceID device_id, DeviceCommInterfaceType device_comm_interface_type,
          std::shared_ptr<IOInterface> io_interface);

    bool syncPlay(uint32_t music_id, uint8_t volume);

 protected:
    void onDataRecive(const std::vector<uint8_t> &data) final;

 private:
    void loadMusic();

    std::vector<uint8_t> buildSpeakerCtrol(uint32_t music_id, uint8_t volume);

    std::map<int, int> music_map_; // <音乐id, 艾智威id>
};

typedef std::shared_ptr<Speaker> Speaker_ptr;
}  // namespace device
}  // namespace sros

#endif  // CORE_HARDWARE_SPEAKER_H_
