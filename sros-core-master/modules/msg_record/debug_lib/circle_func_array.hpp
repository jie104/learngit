//
// Created by lfc on 16-1-27.
//
//
// Created by lfc on 16-1-27.
//

#ifndef CIRCLE_VECTOR_CIRCLEARRAY_H
#define CIRCLE_VECTOR_CIRCLEARRAY_H

#include <vector>
#include <glog/logging.h>

namespace debug{

template<class numtype>
class CircleFuncArray {
public:
    CircleFuncArray(int size_) {
        size = size_;
        array_.resize(size_);
        index = 0;
    }

    CircleFuncArray():size(0),index(0){
    }

    virtual ~CircleFuncArray() {
    }

    void pushBack(const numtype& element) {
        array_[(index) % size] = element;
        index++;
        index = index % size;
    }

    int findIndex(numtype& element) {
        int index_ = -1;
        for (int i = 0; i < size; ++i) {
            if (array_[(index + i) % size] == element) {
                index_ = i;
                return index_;
            }
        }
        LOG(INFO) << "cannot find the  element!" << index_;
        return index_;
    }

    numtype &array(int index_) {
        if (index_ >= size) {
            LOG(INFO)<<"error to get the index! will return array[0]\n";
            if (array_.size()) {
                return array_[0];
            }else {
                if (size == 0) {
                    size = 100;
                    array_.resize(size);
                    return array_[0];
                }else{
                    array_.resize(size);
                    return array_[0];
                }
            }
        }
        int return_index = (index - index_ + size - 1) % size;
        if (return_index >= 0) {
            return array_[return_index];//栈顶,实现了栈模式
        }else {
            LOG(INFO) << "the index is wrong!" << return_index << "," << index_;
            LOG(INFO) << "will return zero array!";
            return array_[0];
        }

    }

    void resize(int size_){
        size=size_;
        array_.resize(size_);
    }

    int getSize(){
        return size;
    }

    void clear() {
        index = 0;
        array_.clear();
        array_.resize(size);
    }
private:
    std::vector<numtype> array_;
    int size;
    int index;
};

}

#endif //CIRCLE_VECTOR_CIRCLEARRAY_H

