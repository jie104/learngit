//
// Created by zxj on 2023/2/15.
//

#ifndef EXAMPLE_STRBLOB_H
#define EXAMPLE_STRBLOB_H

#include <iostream>
#include <memory>
#include <string>
#include <list>
#include <vector>
#include <algorithm>
#include <initializer_list>
#include "StrBlobPtr.hpp"

//对于StrBlob中的有缘声明来说，此前置声明是必要
class StrBlobPtr;

class StrBlob{
public:
    typedef std::vector<std::string>::size_type size_type;
    StrBlob();
    StrBlob(std::initializer_list<std::string> l);
    size_type size() const {return data->size();}
    bool empty()const {return data->empty();}
    //添加和删除元素
    void push_back(const std::string &t){data->push_back(t);}
    void pop_back();
    //元素访问
    std::string& front();
    std::string& back();
    std::string& front() const;
    std::string& back() const;
    std::shared_ptr<std::vector<std::string>> data;

private:
    //如果data[i]不合法，抛出异常
    void check(size_type i,const std::string &msg) const;

//    friend class StrBlobPtr;
//    //返回指向首元素和尾后元素的StrBlobPtr
//    StrBlobPtr begin(){return StrBlobPtr(*this);}
//    StrBlobPtr end(){
//        auto ret=StrBlobPtr(*this,data->size());
//        return ret;
//    }


};
#endif //EXAMPLE_STRBLOB_H

