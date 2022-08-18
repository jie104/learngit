
/**
 * @file StationMark.hpp
 *
 * @author lhx
 * @date 2015年9月22日
 *
 * @describe
 *
 * @copyright Copyright (c) 2019 Standard Robots Co., Ltd. All rights reserved.
 */

#ifndef SROS_MAP_STATION_MARK
#define SROS_MAP_STATION_MARK

#include <cstring>
#include "Mark.hpp"

#define AREA_TYPE_NO_ENTER_VALUE 300

namespace sros {
namespace map {

class StationMark {
 public:
    StationMark()
        : id(0),
          type(1),
          pos(0, 0, 0),
          enter_pos(0, 0, 0),
          exit_pos(0, 0, 0),
          check_pos(0, 0, 0),
          no_rotate(false),
          pos_dynamic(false),
          station_offset(0.0),
          edge_id(0),
          param(0) {}

    StationMark(uint16_t id, uint16_t edge_id, double x, double y, double yaw)
            : id(id),
              type(1),
              pos(x, y, yaw),
              enter_pos(0, 0, 0),
              exit_pos(0, 0, 0),
              check_pos(0, 0, 0),
              no_rotate(false),
              pos_dynamic(false),
              station_offset(0.0),
              edge_id(edge_id),
              param(0) {}

    /**
     * @brief 检测Station是否有效
     * @return
     */
    operator bool() const noexcept { return id > 0; }

    bool operator==(const StationMark &other) const {
        if (this == &other) {
            return true;
        }

        if (this->id == other.id && this->type == other.type && (std::strcmp(this->name, other.name) == 0) &&
            this->pos == other.pos && this->enter_pos == other.enter_pos && this->exit_pos == other.exit_pos &&
            this->check_pos == other.check_pos && this->no_rotate == other.no_rotate &&
            this->enter_backward == other.enter_backward && this->exit_backward == other.exit_backward &&
            this->pos_dynamic == other.pos_dynamic && this->edge_id == other.edge_id && this->param == other.param) {
            return true;
        }
        return false;
    }

    bool operator!=(const StationMark &other) const { return !operator==(other); }

    uint16_t id;                   // 站点id
    uint16_t type;                 // 站点类别
    char name[MAX_MARK_NAME_LEN];  // 站点名称

    MarkPointf pos;  // 站点位置

    MarkPointf enter_pos;
    MarkPointf exit_pos;
    MarkPointf check_pos;  // 对站点进行检测的位置

    bool no_rotate;
    bool enter_backward;
    bool exit_backward;
    bool pos_dynamic;  // 站点位置是否需要动态检测

    float station_offset;  // 站点与特征偏移量

    // PGV 矫正用的
    std::string dmcode_id; // 下视二维码id
    MarkPointf dmcode_offset;  // 下视二维码偏差

    uint16_t edge_id;  // 所属edge的id

    uint32_t param;  // 留作其他用途
};

typedef MarkGroup<StationMark> StationMarkGroup;

}  // namespace map
}  // namespace sros

#endif  // SROS_MAP_STATION_MARK
