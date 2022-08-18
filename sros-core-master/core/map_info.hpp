//
// Created by lhx on 16-1-6.
//

#ifndef PROJECT_MAP_INFO_H
#define PROJECT_MAP_INFO_H

#include <string>
#include <vector>
#include <memory>

namespace sros {
namespace core {

class MapInfo {
public:
    static const int MAX_MAP_NAME_LEN = 32;

    std::string name_; // len <= MAX_MAP_NAME_LEN
    std::string file_path_; // len <= 64
    std::string raw_pgm_path_; // len <= 64
    int32_t create_time_; // len = 4
    int32_t last_modified_time_; // len = 4

    bool operator == (const MapInfo& p) {
        return (name_ == p.name_
                && last_modified_time_ == p.last_modified_time_);
    }

};

typedef std::vector<MapInfo> MapVector_t;

}
}

#endif
