/**
 * @file group.hpp
 *
 * @author lhx
 * @date 2015年9月22日
 *
 * @describe
 *
 * @copyright Copyright (c) 2019 Standard Robots Co., Ltd. All rights reserved.
 */
#ifndef SROS_MAP_NETWORK_GROUP
#define SROS_MAP_NETWORK_GROUP

#include <stdint.h>
#include <map>
#include <vector>

namespace sros {
namespace map {
namespace net {

typedef uint16_t ItemID_t;

template <typename T>
class Group {
 public:
    Group() { group_.clear(); }

    typedef std::map<ItemID_t, T> ItemMap_t;
    typedef typename std::map<ItemID_t, T>::iterator ItemMapIter_t;

    bool addItem(T mark) {
        return addItem(mark.id, mark);
    }
    
    bool addItem(ItemID_t id, T mark) {
        if (exist(id)) {
            return false;
        }

        group_.insert({id, mark});
        return true;
    }

    bool updateItem(T mark) {
        return updateItem(mark.id, mark);
    }

    bool updateItem(ItemID_t id, T mark) {
        if (!exist(id)) {
            return false;
        }

        group_[id] = mark;
        return true;
    }

    bool removeItem(ItemID_t id) { return group_.erase(id) > 0; }

    T getItem(ItemID_t id) {
        ItemMapIter_t it = group_.find(id);
        if (it == group_.end()) {
            return T();
        }
        return group_[id];
    }

    ItemID_t getAvailableID() {
        if (group_.empty()) {
            return 1;
        }

        // 根据std::map性质,返回group中最大键值+1
        ItemMapIter_t it = group_.end();
        it--;

        ItemID_t id = it->first + (ItemID_t)1;

        if (id > 0xffff) {
            return 0;  // 表示无可用id
        } else {
            return id;
        }
    }

    std::vector<T> getItemList() const {
        std::vector<T> list;
        for (auto it = group_.cbegin(); it != group_.cend(); it++) {
            list.push_back(it->second);
        }
        return list;
    }

    size_t getItemCnt() { return group_.size(); }

    void clearAllItem() { group_.clear(); }

    //    // 获得点p(x,y)所在mark的list
    //    std::vector<T> getInsideMarks(double x, double y) {
    //        MarkPointf p(x, y);
    //        std::vector<T> inside_list;
    //
    //        for (ItemMapIter_t it = group_.begin(); it != group_.end(); ++it) {
    //            if (it->second.checkPointInside(p)) {
    //                inside_list.push_back(it->second);
    //            }
    //        }
    //        return inside_list;
    //    }

    ItemMapIter_t begin() {
        return group_.begin();
    }
    ItemMapIter_t end() {
        return group_.end();
    }

 private:
    bool exist(ItemID_t id) {
        ItemMapIter_t it = group_.find(id);
        return (it != group_.end());
    }

    ItemMap_t group_;
};

}  // namespace net
}  // namespace map
}  // namespace sros

#endif  // SROS_MAP_NETWORK_GROUP
