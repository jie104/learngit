//
// Created by lfc on 19-5-15.
//

#ifndef SROS_CIRCLE_STAMP_ARRAY_HPP
#define SROS_CIRCLE_STAMP_ARRAY_HPP

#include <iostream>
#include <cstring>

template<class ArrayType>
class CircleStampArray {
public:
    CircleStampArray(int max_size) : index_(0),real_size_(0), max_size_(max_size), array_(nullptr) {
        array_ = new ArrayType[max_size_];
        memset(array_, 0, sizeof(ArrayType));
    }

    CircleStampArray():index_(0),real_size_(0), array_(nullptr) {
        resize(max_size_);
    }

    void reset() {
        real_size_ = 0;
        index_ = 0;
        delete[] array_;
        array_ = nullptr;
    }

    void resize(int size) {
        reset();
        max_size_ = size;
        array_ = new ArrayType[size];
        memset(array_, 0, sizeof(ArrayType));
    }

    virtual ~CircleStampArray() {
        reset();
    }

    int size() {
        return real_size_;
    }

    bool empty() {
        return real_size_ == 0;
    }

    const ArrayType &index(int index) {
        int new_index = index_ + max_size_ - 1;
        int array_index = (new_index - index) % max_size_; // index=0时，取得整个数列最新值
        return array_index >= 0 ? array_[array_index] : array_[0];
    }

    void push_back(const ArrayType &element) {
        array_[(index_) % max_size_] = element;
        real_size_++;
        index_++;
        index_ = index_ % max_size_;
        real_size_ = real_size_ < max_size_ ? real_size_ : max_size_;
    }

    ArrayType getMinValue() {
        if (real_size_ == 0) {
            std::cout << "real size is zero:" << real_size_ << std::endl;
            return (ArrayType) (0);
        }
        ArrayType min_ele = index(0);
        for (int i = 1; i < real_size_; ++i) {
            min_ele = min_ele > index(i) ? index(i) : min_ele;
        }
        return min_ele;
    }


private:
    unsigned int index_ = 0;
    int real_size_ = 0;
    int max_size_ = 10;
    ArrayType *array_;
};


#endif //SROS_CIRCLE_STAMP_ARRAY_HPP
