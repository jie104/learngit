/**
 * @file speaker.cpp
 *
 * @author pengjiali
 * @date 19-7-16.
 *
 * @describe
 *
 * @copyright Copyright (c) 2019 Standard Robots Co., Ltd. All rights reserved.
 */

#include "speaker.h"
#include "core/util/utils.h"
#include "core/db/db.h"
#include "core/settings.h"

namespace sros {
namespace device {

Speaker::Speaker(const std::string& name, DeviceID device_id, DeviceCommInterfaceType device_comm_interface_type,
                 std::shared_ptr<sros::device::IOInterface> io_interface)
    : IODevice(name, device_id, device_comm_interface_type, io_interface) {
    loadMusic();
}

bool Speaker::syncPlay(uint32_t music_id, uint8_t volume) {
    if (music_map_.find(music_id) == music_map_.cend()) {
        LOG(ERROR) << "music id " << music_id << " not exist!";
        return false;
    }

    asyncRequest(buildSpeakerCtrol(music_id, volume));

    return true;
}

std::vector<uint8_t> Speaker::buildSpeakerCtrol(uint32_t music_id, uint8_t volume) {
    auto request = std::vector<uint8_t>(8, 0);

    request[0] = 0x01;
    request[1] = 0x51;

    request[2] = music_map_.at(music_id);
    request[3] = volume;

    request[4] = 0x00;
    request[5] = 0x00;

    uint8_t xor_value = request[0] ^ request[1] ^ request[2] ^ request[3] ^ request[4] ^ request[5];
    request[6] = xor_value;
    request[7] = 0x02;

    return request;
}

void Speaker::onDataRecive(const std::vector<uint8_t>& data) {
    if (data.size() != 8) {
        LOG(INFO) << "Length check is incorrect!";
        last_error_code_ = ERROR_CODE_LENGTH_INCORRECT;
        is_error_ = true;
        //setResponse(false);
        return;
    }

    response_ = data;
    is_error_ = false;
    //setResponse(true);
}

void Speaker::loadMusic() {
    auto &s = sros::core::Settings::getInstance();
    auto azowie_voice_version = s.getValue<std::string>("hmi.azowie_voice_version", "v1");
    std::string sql;
    if (azowie_voice_version == "v1") {
        sql = "SELECT id, azowie_voice_file_id_v1 FROM music";
    } else {
        sql = "SELECT id, azowie_voice_file_id_v2 FROM music";
    }

    try {
        std::lock_guard<std::recursive_mutex> db_mutex(g_db_mutex);
        SQLite::Statement query(g_db, sql);
        while (query.executeStep()) {
            int id = query.getColumn(0).getInt();
            int azowie_voice_file_id = query.getColumn(1).getInt();
            music_map_.insert(std::make_pair(id, azowie_voice_file_id));
        }
    } catch (std::exception &e) {
        LOG(ERROR) << "Catch a exception: " << e.what() << "; sql is " << sql;
    }
}

}  // namespace device
}  // namespace sros
