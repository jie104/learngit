//
// Created by lfc on 2020/10/21.
//

#ifndef SROS_CIRCLE_SET_HPP
#define SROS_CIRCLE_SET_HPP
#include <vector>
#include <set>
#include <glog/logging.h>

namespace sros{

template <class ArrayType>
class CircleSet {
 public:
    CircleSet(const int size = 10) : index_(0), real_size_(0), max_size_(size) { resize(max_size_); }

    void reset() {
        index_ = 0;
        real_size_ = 0;
        array_.clear();
        array_set_.clear();
    }

    void resize(int size) {
        reset();
        max_size_ = size;
        array_.resize(max_size_);
    }

    virtual ~CircleSet() {
        reset();
    }

    int size() {
        return array_set_.size();
    }

    bool empty() {
        return real_size_ == 0;
    }

    const ArrayType &index(int index) {
        int new_index = index_ + max_size_ - 1;
        int array_index = (new_index - index) % max_size_;
        return array_index >= 0 ? array_[array_index] : array_[0];
    }

    void push_back(const ArrayType &element) {
        array_[(index_) % max_size_] = element;
        real_size_++;
        index_++;
        index_ = index_ % max_size_;
        real_size_ = real_size_ < max_size_ ? real_size_ : max_size_;
        array_set_.insert(element);
        if(real_size_ == max_size_) {
            if(array_set_.erase(array_[index_])!=0) {
                if (array_set_.size() > max_size_) {
                    LOG(INFO) << "array set is wrong! will clear!"<<array_set_.size()<<","<<max_size_<<","<<array_[(index_) % max_size_];
                    array_set_.clear();
                    array_set_.insert(element);
                }
            }
        }
    }

    const ArrayType &getMinValue() {
        return *array_set_.begin();
    }


 private:
    unsigned int index_ = 0;
    int real_size_ = 0;
    int max_size_ = 10;
    std::vector<ArrayType> array_;
    std::set<ArrayType> array_set_;
};

}

#endif  // SROS_CIRCLE_SET_HPP
