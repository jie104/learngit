#ifndef SROS_MAP_AREA_MARK
#define SROS_MAP_AREA_MARK

#include <cmath>
#include "Mark.hpp"

namespace sros {
namespace map {

#define AREA_TYPE_NO_ENTER_VALUE 300

class AreaMark {
 public:
    AreaMark()
        : id(0),
          enter_station(0),
          type(AREA_TYPE_NO_ENTER),
          exit_station(0),
          is_play_music(true),
          is_broadcast(false),
          param_int(0) {}

    bool checkPointInside(MarkPointf p) {
        double AP_AB = (p.x - pa.x) * (pb.x - pa.x) + (p.y - pa.y) * (pb.y - pa.y);
        if (AP_AB <= 0) return false;

        double AB_len = std::pow(pb.x - pa.x, 2) + std::pow(pb.y - pa.y, 2);
        if (AP_AB >= AB_len) return false;

        double AP_AD = (p.x - pa.x) * (pd.x - pa.x) + (p.y - pa.y) * (pd.y - pa.y);
        if (AP_AD <= 0) return false;

        double AD_len = std::pow(pd.x - pa.x, 2) + std::pow(pd.y - pa.y, 2);
        if (AP_AD >= AD_len) return false;

        return true;
    }

    bool validate() {
        if (id == 0) {
            return false;
        }
        if (pa.x == 0 && pa.y == 0 && pc.x == 0 && pc.y == 0) {
            return false;
        }
        return true;
    }

    bool operator==(const AreaMark &other) const {
        if (this == &other) {
            return true;
        }

        if (this->id == other.id && this->type == other.type && (strcmp(this->name, other.name) == 0) &&
            this->pa == other.pa && this->pb == other.pb && this->pc == other.pc && this->pd == other.pd &&
            this->enter_station == other.enter_station && this->exit_station == other.exit_station &&
            this->is_broadcast == other.is_broadcast && this->is_play_music == other.is_play_music &&
            this->param_int == other.param_int) {
            return true;
        }

        return false;
    }

    bool operator!=(const AreaMark &other) const { return !operator==(other); }

    static bool isUserDefineType(uint16_t t) { return (t > 0 && t <= MAX_USER_DEFINE_TYPE_VALUE); }

    bool isNoEnterAreaType() {
        std::cout << "isNoEnterAreaType() type: " << type << " size: " << user_define_type_list.size() << std::endl;

        if (type == AREA_TYPE_NO_ENTER_VALUE) {
            return true;
        }

        if (type == AREA_TYPE_MULTIPLE_TYPE) {
            for (auto it : user_define_type_list) {
                std::cout << "user_define_type_list: " << it << std::endl;
                if (it == AREA_TYPE_NO_ENTER_VALUE) {
                    return true;
                }
            }
        }

        return false;
    }

    uint16_t id;                   // 区域id
    uint16_t type;                 // 区域类型
    char name[MAX_MARK_NAME_LEN];  // 区域名称

    std::vector<uint16_t> user_define_type_list;                         // 用户定义区域类型列表
    std::vector<std::pair<std::string, std::string>> user_define_param;  // 用户自定义的参数
    int z = 100;                                                         // 设置z值， z越大，值越优先

    MarkPointf pa;  // 顶点A坐标
    MarkPointf pb;  // 顶点B坐标
    MarkPointf pc;  // 顶点C坐标
    MarkPointf pd;  // 顶点D坐标

    uint16_t enter_station;  // 进入区域站点
    uint16_t exit_station;   // 退出区域站点

    bool is_play_music;  // 该区域是否播放音乐
    bool is_broadcast;   // 进入、退出区域时是否发送广播

    // int参数，考虑兼容性，区域类型为减速区域时，还是以原来方式处理；
    // 区域类型为限制车辆类型时，保存可以同时容纳的车辆数目
    int param_int;

    static const uint16_t MAX_USER_DEFINE_TYPE_VALUE = 256;

    static const uint16_t AREA_TYPE_NO_ENTER = 300;
    static const uint16_t AREA_TYPE_SPEED_LEVEL_20 = 312;
    static const uint16_t AREA_TYPE_SPEED_LEVEL_40 = 314;
    static const uint16_t AREA_TYPE_SPEED_LEVEL_60 = 316;
    static const uint16_t AREA_TYPE_SPEED_LEVEL_80 = 318;
    static const uint16_t AREA_TYPE_DISABLE_OBA = 350;       // 禁用避障区域
    static const uint16_t AREA_TYPE_DISABLE_ROTATE = 352;    // 禁止旋转区域, 允许旋转角度阈值存放在param_int中
    static const uint16_t AREA_TYPE_DISABLE_REROUTE = 354;   // 禁止二次规划路径区域
    static const uint16_t AREA_TYPE_LIMIT_CAR_NUMBER = 356;  // 限制车辆数目区域
    static const uint16_t AREA_TYPE_ONEWAY_TRAFFIC = 360;    // 单向行驶区域
    static const uint16_t AREA_TYPE_MARK_LOCATION = 361;  // 标记位置区域，只做位置标记使用，不影响任何导航功能

    const static uint16_t AREA_TYPE_LOCAL_PLANNER = 362; // 设置区域用于局部规划主动绕障。
    const static uint16_t AREA_TYPE_DISABLE_PHOTOELECTRIC_SWITCH_OBA = 366; // 设置区域用于禁用叉尖光电开关。
    const static uint16_t AREA_TYPE_MULTIPLE_TYPE = 500;  // 可以同时设置多个区域，可以设置配置
};

typedef MarkGroup<AreaMark> AreaMarkGroup;

}  // namespace map
}  // namespace sros

#endif  // SROS_MAP_AREA_MARK
