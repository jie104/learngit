//
// Created by zxj on 2023/2/20.
//

#include "head_file/StrBlob.h"
#include <exception>
#include <stdexcept>
#include <iostream>


class StrBlobPtr{
public:
    StrBlobPtr():curr(0){}
    StrBlobPtr(StrBlob &a,size_t sz=0):
        wptr(a.data),curr(sz){ }

    std::string& deref() const{
        auto p= check(curr,"dereference past end");
        return (*p)[curr];
    }

    StrBlobPtr& incr(){
        check(curr,"increment past end of StrBlobPtr");
        ++curr;
        return *this;
    } //前缀递增

private:
    //若检查成功，check返回一个指向vector的shared_ptr
    std::shared_ptr<std::vector<std::string>>
        check(std::size_t i,const std::string &msg) const{
        auto ret=wptr.lock();   //检车vector是否存在
        if(!ret)
            throw std::runtime_error("unbond StrBlobPtr");
        if(i>=ret->size())
            throw std::out_of_range(msg);
        return ret; //否则返回指向vector的shared_ptr
    }
    //保存一个weak_ptr，意味着底层可能被销毁
    std::weak_ptr<std::vector<std::string>> wptr;
    std::size_t curr;   //在数组中的当前位置
};