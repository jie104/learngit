//
// Created by lhx on 16-4-24.
//

#ifndef SROS_POSESAVER_H
#define SROS_POSESAVER_H

#include <string>

#include "core/pose.h"

class PoseSaver {
public:
    PoseSaver();
    ~PoseSaver();

    static void save(const sros::core::Pose& pose, const std::string& map_name, uint16_t station_no);
    static bool load(sros::core::Pose& pose, std::string& map_name, uint16_t &station_no);

private:
    static void baseSave(const std::string &file_name, const sros::core::Pose& pose, const std::string& map_name, uint16_t station_no);
    static bool baseLoad(const std::string &file_name, sros::core::Pose& pose, std::string& map_name, uint16_t &station_no);

    // 被废弃，只做兼容
    static void oldSave(const sros::core::Pose& pose, const std::string& map_name, uint16_t station_no);
    static bool oldLoad(sros::core::Pose& pose, std::string& map_name, uint16_t &station_no);
};


#endif //SROS_POSESAVER_H
