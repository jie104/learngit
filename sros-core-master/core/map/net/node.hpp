/**
 * @file node.hpp
 *
 * @author lhx
 * @date 2015年9月22日
 *
 * @describe
 *
 * @copyright Copyright (c) 2019 Standard Robots Co., Ltd. All rights reserved.
 */
#ifndef SROS_MAP_NETWORK_NODE
#define SROS_MAP_NETWORK_NODE

#include <stdint.h>
#include "group.hpp"

namespace sros {
namespace map {
namespace net {

typedef ItemID_t node_id_t;

template <typename T>
class Node {
 public:
    Node() : id(0), x(0), y(0), yaw(0) {}
    Node(node_id_t id, T x, T y, double yaw = 0) : id(id), x(x), y(y), yaw(yaw) {}

    template <typename T1>
    bool operator==(const T1 &other) const {
        if (this == &other) {
            return true;
        }

        if (this->id == other.id && this->x == other.x && this->y == other.y /*&& this->yaw == other.yaw*/) {
            return true;
        }

        return false;
    }

    template <typename T1>
    bool operator!=(const T1 &other) const {
        return !operator==(other);
    }

    node_id_t id;

    T x;
    T y;
    double yaw;
};

typedef Node<double> Nodef;
typedef Node<int> Nodei;

typedef Group<Nodef> NodeGroup;

}  // namespace net
}  // namespace map
}  // namespace sros

#endif  // SROS_MAP_NETWORK_NODE
