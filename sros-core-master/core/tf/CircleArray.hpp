//
// Created by lfc on 16-1-27.
//

#ifndef TEST_VECTOR_CIRCLEARRAY_H
#define TEST_VECTOR_CIRCLEARRAY_H

#include <iostream>

template <class numtype>
class CircleArray {
 public:
    CircleArray(int size_) {
        size = size_;
        array_ = new numtype[size];
        index = 0;
        exist_element = false;
        is_full = false;
    }
    CircleArray() : size(0), array_(0) { index = 0; exist_element = false; is_full = false;}
    virtual ~CircleArray() {
        if (array_) {
            delete[] array_;
            array_ = 0;
        }
    }

    void pushback(numtype element) {
        array_[(index) % size] = element;
        index++;
        if(index >= size) {
            is_full = true;
            index = 0;
        }
        // index = index % size;
        exist_element = true;
    }

    void findindex(numtype element, int &index_) {
        for (int i = 0; i < size; ++i) {
            if (array_[(index + i) % size] == element) {
                index_ = i;
                return;
            }
        }
        printf("cannot find the  element!\n");
        index_ = -1;
        return;
    }

    int currentindex() {
        return index;
    }

    bool existelement() {
        return exist_element;
    }

    bool isfull() {
        return is_full;
    }

    numtype &array(int index_) {
        if (index_ >= size) {
//            printf("error to get the index! will return array[0]\n");
            if (array_) {
                return array_[0];
            } else {
                if (size == 0) {
                    size = 100;
                    array_ = new numtype[size];
                    exist_element = false;
                    is_full = false;
                    return array_[0];
                } else {
                    array_ = new numtype[size];
                    exist_element = false;
                    is_full = false;
                    return array_[0];
                }
            }
        }
        return array_[(index + index_) % size];  // index=0时，取得整个数列最旧值
    }
    void resize(int size_) {
        size = size_;
        if (array_) {
            delete[] array_;
            array_ = 0;
        }
        array_ = new numtype[size];
        exist_element = false;
        is_full = false;
    }
    int getsize() { return size; }
    void clear() {
        if (array_) {
            delete[] array_;
            array_ = 0;
        }
        array_ = new numtype[size];
        index = 0;
        exist_element = false;
        is_full = false;
    }

 private:
    numtype *array_;
    int size;
    int index;
    bool exist_element;
    bool is_full;
};

#endif  // TEST_VECTOR_CIRCLEARRAY_H
