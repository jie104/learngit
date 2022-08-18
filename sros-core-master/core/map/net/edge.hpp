
/**
 * @file edge.hpp
 *
 * @author lhx
 * @date 2015年9月22日
 *
 * @describe
 *
 * @copyright Copyright (c) 2019 Standard Robots Co., Ltd. All rights reserved.
 */
#ifndef SROS_MAP_NETWORK_EDGE
#define SROS_MAP_NETWORK_EDGE

#include <cassert>
#include <cstdint>

#include "group.hpp"
#include "node.hpp"

namespace sros {
namespace map {
namespace net {

typedef ItemID_t edge_id_t;

enum EdgeType {
    EDGE_LINE = 1,
    EDGE_CIRCLE = 2,
    EDGE_BEZIER = 3,
};

/**
 * 设置车在路径上的朝向
 */
enum EdgeDirection {
    EDGE_DIRECTION_DEFAULT = 0,  ///<  默认方式，不考虑用户设置的路径起点和终点朝向
    EDGE_FOLLOW_S = 1,           ///< 按照起点的朝向作为小车在路径上的朝向
    EDGE_FOLLOW_E = 2,           ///< 按照终点的朝向作为小车在路径上的朝向
    EDGE_ROTATE = 3,  ///< 小车在路径上从起点朝向旋转到终点的朝向（仅全向底盘可用）
};

/**
 * 车辆运行过程中的车头朝向，通过地图中的facing算出来的
 * 和main.proto/Path/direction一致
 */
enum class VehicleDirection {
    NONE = 0,
    FORWARD = 1,
    BACKWARD = 2,
    LEFT = 3,
    RIGHT = 4,
};

/**
 * 车在路径上旋转时旋转的方向（顺时针/逆时针）
 */
enum EdgeRotateDirection {
    EDGE_ROTATE_ACW = 1,  ///< 逆时针
    EDGE_ROTATE_CW = 2,   ///< 顺时针
};

enum EdgeExecuteType { EXECUTE_FREE = 0x01, EXECUTE_LOAD = 0x02, EXECUTE_ALL = 0xFF };

template <typename T>
class Edge {
 public:
    Edge()
        : id(0),
          s_node(0),
          e_node(0),
          cost(0),
          radius(0),
          sx(0),
          sy(0),
          ex(0),
          ey(0),
          cx(0),
          cy(0),
          dx(0),
          dy(0),
          limit_v(0),
          limit_w(0),
          s_facing(0),
          e_facing(0),
          rotate_direction(EDGE_ROTATE_ACW),
          vehicle_direction(VehicleDirection::FORWARD),
          execute_type(EXECUTE_ALL),
          direction(EDGE_DIRECTION_DEFAULT) {}

    Edge(edge_id_t id, node_id_t s_node, node_id_t e_node, double cost, EdgeType type, T sx, T sy, T ex, T ey, T cx,
         T cy, T dx, T dy, double radius, double limit_v, double limit_w, double s_facing, double e_facing,
         EdgeRotateDirection rotate_direction, VehicleDirection vehicle_direction, EdgeDirection direction,
         EdgeExecuteType execute_type)
        : id(id),
          s_node(s_node),
          e_node(e_node),
          cost(cost),
          type(type),
          sx(sx),
          sy(sy),
          ex(ex),
          ey(ey),
          cx(cx),
          cy(cy),
          dx(dx),
          dy(dy),
          radius(radius),
          limit_v(limit_v),
          limit_w(limit_w),
          s_facing(s_facing),
          e_facing(e_facing),
          rotate_direction(rotate_direction),
          vehicle_direction(vehicle_direction),
          direction(direction),
          execute_type(execute_type) {}

    // 用于用户自己构造路径,注意所有的朝向都是算出来的，不适合万向底盘
    Edge(edge_id_t id, Node<T> s_node, Node<T> e_node, EdgeType type, T cx, T cy, T dx, T dy, double radius,
         double limit_v, double limit_w, EdgeRotateDirection rotate_direction, VehicleDirection vehicle_direction,
         EdgeExecuteType execute_type)
        : id(id),
          s_node(s_node.id),
          e_node(e_node.id),
          type(type),
          sx(s_node.x),
          sy(s_node.y),
          ex(e_node.x),
          ey(e_node.y),
          cx(cx),
          cy(cy),
          dx(dx),
          dy(dy),
          radius(radius),
          limit_v(limit_v),
          limit_w(limit_w),
          rotate_direction(rotate_direction),
          vehicle_direction(vehicle_direction),
          direction(EDGE_DIRECTION_DEFAULT),
          execute_type(execute_type) {
        assert((std::is_same<double, T>::value));  // T必须是double|cm,不然算不出朝向

        auto getLineFacingFunc = [](T s_x, T s_y, T e_x, T e_y, VehicleDirection vehicle_direction) {
            T d_x = e_x - s_x;
            T d_y = e_y - s_y;
            T d = sqrt(pow(d_x, 2) + pow(d_y, 2));
            double theta = acos(d_x / d);
            if (d_y < 0) {
                theta = 2 * M_PI - theta;
            }
            switch (vehicle_direction) {
                case VehicleDirection::NONE:
                case VehicleDirection::FORWARD: {
                    break;
                }
                case VehicleDirection::BACKWARD: {
                    theta += M_PI;
                    break;
                }
                case VehicleDirection::LEFT: {
                    theta += M_PI_2;
                    break;
                }
                case VehicleDirection::RIGHT: {
                    theta -= M_PI_2;
                    break;
                }
                default: {
                    break;
                }
            }
            theta = std::abs(fmod(fmod(theta, 2.0 * M_PI), 2.0 * M_PI));
            return theta;
        };

        switch (type) {
            case EDGE_LINE: {
                s_facing = getLineFacingFunc(sx, sy, ex, ey, vehicle_direction);
                e_facing = s_facing;

                T d_x = ex - sx;
                T d_y = ey - sy;
                T d = sqrt(pow(d_x, 2) + pow(d_y, 2));
                cost = d;
                break;
            }
            case EDGE_BEZIER: {
                s_facing = getLineFacingFunc(sx, sy, cx, cy, vehicle_direction);
                e_facing = getLineFacingFunc(dx, dy, ex, ey, vehicle_direction);

                // NOTE: 由于三次贝塞尔无法积分，所以此处用折线来近似模拟
                const double SPLIT = 1000.0;  // 分割成多少份
                auto x = [&](double t) {
                    return sx * std::pow((1 - t), 3) + 3 * cx * t * std::pow((1 - t), 2) +
                           3 * dx * std::pow(t, 2) * (1 - t) + ex * std::pow(t, 3);
                };
                auto y = [&](double t) {
                    return sy * std::pow((1 - t), 3) + 3 * cy * t * std::pow((1 - t), 2) +
                           3 * dy * std::pow(t, 2) * (1 - t) + ey * std::pow(t, 3);
                };
                auto last_x = sx;
                auto last_y = sy;
                double length = 0.0;
                for (auto i = 1; i <= SPLIT; ++i) {
                    double t = 1.0 / SPLIT * i;
                    double new_x = x(t);
                    double new_y = y(t);
                    length += sqrt(pow(last_x - new_x, 2) + pow(last_y - new_y, 2));
                    last_x = new_x;
                    last_y = new_y;
                }
                cost = length;
                break;
            }
            case EDGE_CIRCLE: {
                s_facing = getLineFacingFunc(sx, sy, cx, cy, vehicle_direction) + (radius > 0 ? -M_PI_2 : M_PI_2);
                e_facing = s_facing + M_PI;

                T d_x = ex - sx;
                T d_y = ey - sy;
                T d = sqrt(pow(d_x, 2) + pow(d_y, 2));
                cost = d * M_PI;
            }
            default: {
                break;
            }
        }
    }

    template <typename T1>
    bool operator==(const T1 &other) const {
        if (this == &other) {
            return true;
        }

        if (this->id == other.id && this->s_node == other.s_node && this->e_node == other.e_node &&
            this->cost == other.cost && this->type == other.type && this->sx == other.sx && this->sy == other.sy &&
            this->ex == other.ex && this->ey == other.ey && this->cx == other.cx && this->cy == other.cy &&
            this->dx == other.dx && this->dy == other.dy &&
            //            this->radius == other.radius &&  // 从地图文件中读取到nan导致比较失败
            this->limit_v == other.limit_v && this->limit_w == other.limit_w && this->s_facing == other.s_facing &&
            this->e_facing == other.e_facing && this->rotate_direction == other.rotate_direction &&
            this->is_back_edge == other.is_back_edge && this->direction == other.direction &&
            this->execute_type == other.param) {
            return true;
        }

        return false;
    }

    template <typename T1>
    bool operator!=(const T1 &other) const {
        return !operator==(other);
    }

    friend std::ostream &operator<<(std::ostream &out, const Edge<T> &edge) {
        std::string str = "Edge{id:" + std::to_string(edge.id) + ", type:";
        switch (edge.type) {
            case EDGE_LINE: {
                str += "EDGE_LINE";
                break;
            }
            case EDGE_CIRCLE:
                str += "EDGE_CIRCLE";
                break;
            case EDGE_BEZIER:
                str += "EDGE_BEZIER";
                break;
            default:
                break;
        }
        str += ", s_node:" + std::to_string(edge.s_node) + ", e_node:" + std::to_string(edge.e_node) +
               ", vehicle_direction:";
        switch (edge.vehicle_direction) {
            case VehicleDirection::FORWARD: {
                str += "FORWARD";
                break;
            }
            case VehicleDirection::BACKWARD: {
                str += "BACKWARD";
                break;
            }
            case VehicleDirection::LEFT: {
                str += "LEFT";
                break;
            }
            case VehicleDirection::RIGHT: {
                str += "RIGHT";
                break;
            }
            default: {
                break;
            }
        }
        out << str;
        return out;
    }

    edge_id_t id;

    node_id_t s_node;  // 起点node的id
    node_id_t e_node;  // 终点node的id

    double cost;  // 和sx的单位一致, 这个单位可能是m也能是cm

    EdgeType type;

    T sx;           // 起点x
    T sy;           // 起点y
    T ex;           // 终点x
    T ey;           // 终点y
    T cx;           // 圆弧中心x或控制点c的x
    T cy;           // 圆弧中心y或控制点c的y
    T dx;           // 控制点d的x
    T dy;           // 控制点d的y
    double radius;  // 如果radius > 0, 说明圆弧逆时针旋转, 否则顺时针旋转

    double limit_v;  // 路径的限制速度，单位m/s，0表示无限制
    double limit_w;  // 路径的限制角速度，单位rad/s，0表示无限制

    double s_facing;  // 路径起点朝向，与x轴正方向夹角，单位rad
    double e_facing;  // 路径终点朝向，与x轴正方向夹角，单位rad
    EdgeRotateDirection rotate_direction;

    /**
     * 以前Matrix_2.3.1之前的上会用到这个，之后的版本由于加了左右，Matrix为了节省计算量又定义了一个新字段存储，sros这一层还是用
     */
    //    VehicleDirection vehicle_direction;  //【废弃】
    //    是否是倒退路径，临时使用，不存入文件,并不是所有的输入都是正确的，最好根据s_facing来算
    VehicleDirection vehicle_direction = VehicleDirection::FORWARD;

    EdgeDirection direction;  // 车在路径上的朝向

    EdgeExecuteType execute_type;  // 路径那种货位状态可以走
};

typedef Edge<double> Edgef;

typedef Group<Edgef> EdgeGroup;

template <typename T>
Edge<T> makeLine(edge_id_t id, Node<T> s_node, Node<T> e_node,
                 VehicleDirection vehicle_direction = VehicleDirection::FORWARD, double limit_v = 0, double limit_w = 0,
                 EdgeExecuteType execute_type = EXECUTE_ALL) {
    return Edge<T>(id, s_node, e_node, EDGE_LINE, 0, 0, 0, 0, 0, limit_v, limit_w, EDGE_ROTATE_ACW, vehicle_direction,
                   execute_type);
}

template <typename T>
Edge<T> makeBezier(edge_id_t id, Node<T> s_node, Node<T> e_node, T cx, T cy, T dx, T dy,
                   VehicleDirection vehicle_direction = VehicleDirection::FORWARD, double limit_v = 0,
                   double limit_w = 0, EdgeExecuteType execute_type = EXECUTE_ALL) {
    return Edge<T>(id, s_node, e_node, EDGE_BEZIER, cx, cy, dx, dy, 0, limit_v, limit_w, EDGE_ROTATE_ACW,
                   vehicle_direction, execute_type);
}

}  // namespace net
}  // namespace map
}  // namespace sros

#endif  // SROS_MAP_NETWORK_EDGE
