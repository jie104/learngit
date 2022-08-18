
/**
 * @file Mark.hpp
 *
 * @author lhx
 * @date 2015年9月22日
 *
 * @describe
 *
 * @copyright Copyright (c) 2019 Standard Robots Co., Ltd. All rights reserved.
 */

#ifndef SROS_MAP_MARK
#define SROS_MAP_MARK

#include <stdint.h>
#include <map>
#include <type_traits>
#include <vector>

namespace sros {
namespace map {

template <typename T>
class MarkPoint {
 public:
    MarkPoint() : x(1), y(2), yaw(3) {}

    MarkPoint(T x_, T y_) : x(x_), y(y_), yaw(0) {}

    MarkPoint(T x_, T y_, T yaw_) : x(x_), y(y_), yaw(yaw_) {}

    template <typename T1>
    bool operator==(const T1 &other) const {
        if (this == &other) {
            return true;
        }

        if (this->x == other.x && this->y == other.y && this->yaw == other.yaw) {
            return true;
        }

        return false;
    }

    template <typename T1>
    bool operator!=(const T1 &other) const {
        return !operator==(other);
    }

    bool isNull() const { return (x == 0 && y == 0); }

    template <typename TOut>
    friend TOut &operator<<(TOut &out, const MarkPoint &point) {
        out << "Point(" << point.x << ", " << point.y << ", " << point.yaw << ")";
        return out;
    }

    T x;
    T y;
    T yaw;
};

#define MAX_MARK_NAME_LEN 32

typedef MarkPoint<double> MarkPointf;
typedef MarkPoint<int> MarkPointi;

typedef uint16_t MarkID_t;
typedef uint16_t MarkType_t;

template <typename T, typename T_ID = MarkID_t>
class MarkGroup {
 public:
    MarkGroup() { group_.clear(); }

    typedef std::map<T_ID, T> MarkMap_t;
    typedef typename std::map<T_ID, T>::iterator MarkMapIter_t;

    bool addItem(T mark) {
        return addItem(mark.id, mark);
    }
    
    bool addItem(T_ID id, T mark) {
        if (exist(id)) {
            return false;
        }

        group_.insert({id, mark});
        return true;
    }

    bool updateItem(T_ID id, T mark) {
        if (!exist(id)) {
            return false;
        }

        group_[id] = mark;
        return true;
    }

    bool updateItem(T mark) {
        if (!exist(mark.id)) {
            return false;
        }

        group_[mark.id] = mark;
        return true;
    }

    bool removeItem(T_ID id) { return group_.erase(id) > 0; }

    T getItem(T_ID id) {
        MarkMapIter_t it = group_.find(id);
        if (it == group_.end()) {
            return T();
        }
        return group_[id];
    }

    T_ID getAvailableID() {
        if (std::is_same<T_ID, T_ID>::value) {
            if (group_.empty()) {
                return 1;
            }

            // 根据std::map性质,返回group中最大键值+1
            MarkMapIter_t it = group_.end();
            it--;

            T_ID id = it->first + (T_ID)1;

            if (id > 0xffff) {
                return 0;  // 表示无可用id
            } else {
                return id;
            }
        } else {
            assert(false);  // 未实现
        }
    }

    std::vector<T> getItemList() {
        std::vector<T> list;
        for (MarkMapIter_t it = group_.begin(); it != group_.end(); it++) {
            list.push_back(it->second);
        }
        return list;
    }

    size_t getItemCnt() { return group_.size(); }

    void clearAllItem() { group_.clear(); }

    // 获得点p(x,y)所在mark的list
    std::vector<T> getInsideMarks(double x, double y, uint16_t area_type) {
        MarkPointf p(x, y);
        std::vector<T> inside_list;

        for (MarkMapIter_t it = group_.begin(); it != group_.end(); ++it) {
            if ((area_type == 0 || area_type == it->second.type) && it->second.checkPointInside(p)) {
                inside_list.push_back(it->second);
            }
        }
        return inside_list;
    }

    MarkMapIter_t begin() {
        return group_.begin();
    }
    MarkMapIter_t end() {
        return group_.end();
    }
 private:
    bool exist(T_ID id) {
        MarkMapIter_t it = group_.find(id);
        return (it != group_.end());
    }

    MarkMap_t group_;
};

}  // namespace map
}  // namespace sros

#endif  // SROS_MAP_MARK
