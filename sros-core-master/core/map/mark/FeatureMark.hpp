
/**
 * @file FeatureMark.hpp
 *
 * @author caoyan
 * @date 2021年2月25日
 *
 * @describe
 *
 * @copyright Copyright (c) 2019 Standard Robots Co., Ltd. All rights reserved.
 */

#ifndef SROS_MAP_FEATURE_MARK
#define SROS_MAP_FEATURE_MARK

#include <string>
#include "Mark.hpp"

#define AREA_TYPE_NO_ENTER_VALUE 300

namespace sros {
namespace map {

class FeatureMark {
 public:
    FeatureMark()
        : id(0),
          station_id(0),
          feature_type(0),
          feature_name(""),
          sensor_name(""),
          pos_x(0.0),
          pos_y(0.0),
          pos_z(0.0),
          pos_yaw(0.0),
          pos_pitch(0.0),
          pos_roll(0.0){}

    /**
     * @brief 检测Feature是否有效
     * @return
     */
    operator bool() const noexcept { return id > 0; }

    bool operator==(const FeatureMark &other) const {
        if (this == &other) {
            return true;
        }

        if (this->id == other.id
            && this->station_id == other.station_id
            && this->feature_type == other.feature_type
            && (std::strcmp(this->feature_name.c_str(), other.feature_name.c_str()) == 0)
            && (std::strcmp(this->sensor_name.c_str(), other.sensor_name.c_str()) == 0)  
            && this->pos_x == other.pos_x 
            && this->pos_y == other.pos_y 
            && this->pos_z == other.pos_z 
            && this->pos_yaw == other.pos_yaw 
            && this->pos_pitch == other.pos_pitch
            && this->pos_roll == other.pos_roll) {
            return true;
        }
        return false;
    }

    bool operator!=(const FeatureMark &other) const { return !operator==(other); }

    uint16_t id;                        // 特征信息编号
    uint16_t station_id;                // 特征绑定的站点id
    uint16_t feature_type;              // 特征类型，1：DM码、2：FM码、3：scan轮廓特征、4：sacn信标特征
    std::string feature_name;           // 特征名称
    std::string sensor_name;            // 特征传感器

    double pos_x;                      // 站点特征x坐标，单位0.001mm
    double pos_y;                      // 站点特征y坐标，单位0.001mm
    double pos_z;                      // 站点特征z坐标，单位0.001mm

    double pos_yaw;                    // 站点特征yaw角度，单位1/1000000弧度
    double pos_pitch;                  // 站点特征pitch角度，单位1/1000000弧度
    double pos_roll;                   // 站点特征roll角度，单位1/1000000弧度

};

typedef MarkGroup<FeatureMark> FeatureMarkGroup;

}  // namespace map
}  // namespace sros

#endif  // SROS_MAP_FEATURE_MARK
