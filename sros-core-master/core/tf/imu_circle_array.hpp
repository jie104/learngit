//
// Created by shj on 21-10.
//

#ifndef VECTOR_IMU_CIRCLE_ARRAY_H
#define VECTOR_IMU_CIRCLE_ARRAY_H

#include <iostream>

template <class numtype>
class ImuCircleArray {
 public:
    ImuCircleArray(int size) : size_(size), index_(0), real_size_(0) { array_ = new numtype[size_]; }

 private:
    ImuCircleArray() : size_(0), array_(0), index_(0), real_size_(0) {}

 public:
    virtual ~ImuCircleArray() {
        if (array_) {
            delete[] array_;
            array_ = 0;
        }
    }

    void push_back(const numtype &element) {
        array_[(index_) % size_] = element;
        real_size_++;
        index_++;
        index_ = index_ % size_;
        real_size_ = real_size_ < size_ ? real_size_ : size_;
    }

    numtype &array(int index) {
        if (index >= size_) {
            //            printf("error to get the index! will return array[0]\n");
            if (array_) {
                return array_[0];
            } else {
                if (size_ == 0) {
                    size_ = 100;
                    array_ = new numtype[size_];
                    return array_[0];
                } else {
                    array_ = new numtype[size_];
                    return array_[0];
                }
            }
        }
        return real_size_>=size_?array_[(index + index_) % size_]:array_[(index) % size_];
    }

    numtype &operator[](int index) const {
        return real_size_>=size_?array_[(index + index_) % size_]:array_[(index) % size_];
    }

    void resize(int size) {
        size_ = size;
        real_size_ = 0;
        if (array_) {
            delete[] array_;
            array_ = 0;
        }
        array_ = new numtype[size_];
    }

    void reset() {
        real_size_ = 0;
        index_ = 0;
        delete[] array_;
        array_ = nullptr;
    }

    int size() const { return real_size_; }

    bool empty() const { return real_size_ == 0; }

    void clear() {
        if (array_) {
            delete[] array_;
            array_ = 0;
        }
        array_ = new numtype[size_];
        index_ = 0;
    }

 private:
    numtype *array_;
    int size_;
    int index_;
    int real_size_;
};

#endif  // VECTOR_IMU_CIRCLE_ARRAY_H
