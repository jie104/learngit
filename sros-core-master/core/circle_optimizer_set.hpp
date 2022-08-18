//
// Created by lfc on 2021/9/30.
//

#ifndef SROS_CIRCLE_OPTIMIZER_SET_HPP
#define SROS_CIRCLE_OPTIMIZER_SET_HPP

#include <vector>
#include <set>
#include <glog/logging.h>

namespace circle{

template<class ArrayType>
class CircleOptimizerArray {
 public:
    struct ValueInfo{
        int64_t index;
        ArrayType value;
    };

    CircleOptimizerArray(int max_size) : max_size_(max_size),value_index_(0) {
        resize(max_size);
    }

    CircleOptimizerArray(){
        resize(max_size_);
    }

    virtual ~CircleOptimizerArray() {
        reset();
    }

    void reset() {
        value_index_ = 0;
    }

    void resize(int size) {
        reset();
        max_size_ = size;
    }

    int size() {
        return max_size_;
    }

    bool empty() {
        return value_index_ == 0;
    }

    void push_back(const ArrayType &element) {
        if (value_index_ == 0) {//初始时刻，将当前值复制过来
            creatValue(min_value_, element, value_index_);
            bk_value_ = element;
        }else{
            if (element <= min_value_.value) {//发现比min_value更小的值，直接赋值，重新计数
                creatValue(min_value_, element,value_index_);
                bk_value_ = element;
            }else{
                if (bk_value_ > element) {//对bk_value进行初始化，如果发现当前值小于bkvalue值，则更小bk_value
                    bk_value_ = element;
                }
                if (abs(value_index_ - min_value_.index) >= max_size_) {//如果连续查找max_size依然找不到最小值，将当前找到的bk_value最小值复制给min_value,重新计数
                    creatValue(min_value_, bk_value_, value_index_);
                    bk_value_ = element;
                }
            }
        }
        value_index_++;
    }

    ArrayType getMinValue() { return min_value_.value; }

    void creatValue(ValueInfo& min_value,const ArrayType& value,int64_t index){
        min_value.index = index;
        min_value.value = value;
    }
 private:

    int max_size_ = 10;
    ValueInfo min_value_;
    ArrayType bk_value_;
    int64_t value_index_;
};

}


#endif  // SROS_CIRCLE_OPTIMIZER_SET_HPP
