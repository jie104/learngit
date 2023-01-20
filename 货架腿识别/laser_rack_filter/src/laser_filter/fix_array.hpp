//
// Created by lfc on 18-6-19.
//

#ifndef RACK_SROS_FIX_ARRAY_HPP
#define RACK_SROS_FIX_ARRAY_HPP

#include <glog/logging.h>
#include <boost/thread/mutex.hpp>
#include <boost/thread/shared_mutex.hpp>

namespace rack{
template<typename numtype>
class FixArray {
public:

    /**
    *@author lfc
    *@date 16-10-5:上午9:51
    *@version 3.0
    *@brief 构造函数，初始化内成员，注意需要指明size大小
    *@param int 固定数组大小
    *@return 无
    *@note 需要指明数组size大小
    *@warning need to warn
    */
    FixArray(int size_) : array_(nullptr),size(size_),index(0),real_size(0){
        array_ = new numtype[size];
    }

    FixArray() : array_(nullptr),size(100), index(0),real_size(0) {

    }

    /**
    *@author lfc
    *@date 16-10-5:上午9:52
    *@version 3.0
    *@brief 析构函数，将新建的数组删除
    *@param 无 无
    *@return 无
    *@note need to note
    *@warning need to warn
    */
    virtual ~FixArray() {
        if (array_) {
            delete[] array_;
            array_ = nullptr;
        }
    }

    /**
    *@author lfc
    *@date 16-10-5:上午9:53
    *@version 3.0
    *@brief 向数组内部存储数据，如果未满，realsize将+1,如果满，则删除最后一个。该部分，最后一个是index%size上一个
    *@param numtype 传入容器内的类型，可指定
    *@return void
    *@note need to note
    *@warning need to warn
    */
    void push_back(const numtype& element) {
        array_[(index) % size] = element;
        real_size++;
        index++;
        index = index % size;
        real_size = real_size < size ? real_size : size;
    }

    /**
    *@author lfc
    *@date 16-10-5:上午9:56
    *@version 3.0
    *@brief 逆序查找，这样第一个查找的是最近存入系统的，以存储时间顺序，向前查找
    *@param numtype 传入容器的类型 int 返回的索引
    *@return bool，是否找到
    *@note need to note
    *@warning need to warn
    */
    bool findIndex(numtype &element, int &index_) {
        int new_index = index + size - 1;
        for (int i = 0; i < size; ++i) {
            if (array_[(new_index - i) % size] == element) {
                index_ = i;
                return true;
            }
        }
        LOG(INFO)<<"cannot find the element!\n";
        index_ = -1;
        return false;
    }

    /**
    *@author lfc
    *@date 16-10-5:上午9:58
    *@version 3.0
    *@brief 返回指向array的引用，index为0时，表示最新插进去。如果未找到，返回0
    *@param int 要找的index，如果未找到，返回0
    *@return 返回array引用
    *@note need to note
    *@warning need to warn
    */
    numtype &operator[](int index_) {
        if (index_ >= size) {
            printf("error to get the index! will return array[0]\n");
            return array_[0];
        }
        int new_index = index + size - 1;
        return array_[(new_index - index_) % size];
    }

    /**
    *@author lfc
    *@date 16-10-5:上午10:00
    *@version 3.0
    *@brief 重新申请内存空间
    *@param int 申请空间大小
    *@return void
    *@note need to note
    *@warning need to warn
    */

    void resize(int size_) {
        size = size_;
        if (array_) {
            delete[] array_;
            array_ = nullptr;
        }
        array_ = new numtype[size];
        index = 0;
        real_size = 0;
    }


    /**
    *@author lfc
    *@date 16-10-5:上午10:01
    *@version 3.0
    *@brief 返回size大小，如果数组未满，则返回realsize
    *@param 无 无
    *@return int 数组size大小
    *@note need to note
    *@warning need to warn
    */
    int getSize() {
        return real_size < size ? real_size : size;
    }

    /**
    *@author lfc
    *@date 16-10-5:上午10:01
    *@version 3.0
    *@brief 返回size大小
    *@param 无 无
    *@return int 数组size大小
    *@note need to note
    *@warning need to warn
    */
    int arraySize() {
        return size;
    }

    /**
    *@author lfc
    *@date 16-10-5:上午10:03
    *@version 3.0
    *@brief 清除数组，重新建立size大小数组，类似于数组初始化
    *@param 无 无
    *@return 无
    *@note need to note
    *@warning need to warn
    */
    void clear() {
        if (array_) {
            delete[] array_;
            array_ = nullptr;
        }
        array_ = new numtype[size];
        index = 0;
        real_size = 0;
    }

    /**
    *@author lfc
    *@date 16-10-5:上午10:04
    *@version 3.0
    *@brief 判断数组是否满
    *@param 无 无
    *@return bool 如果未满，返回false，否则返回true
    *@note need to note
    *@warning need to warn
    */
    bool isfull() {
        return real_size >= size;
    }

private:
    numtype *array_;
    int64_t size;
    unsigned int index;
    int64_t real_size;
};
}




#endif //SROS_FIX_ARRAY_HPP
