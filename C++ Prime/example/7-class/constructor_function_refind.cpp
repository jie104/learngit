//
// Created by zxj on 2022/12/12.
//

#include <iostream>
#include <string>
#include <vector>

class ConstRef
{
public:
    ConstRef(int ii);
private:
    int i;
    const int ci;
    int &ri;
};

class X{
    int i;
    int j;
public:
    //i先被初始化，并试图用未定义的值j初始化i
    X(int val):j(val),i(j) {}
};
//错误：ci和ri必须被初始化
//ConstRef::ConstRef(int ii) {
//赋值
//    i=ii;
//    ci=ii;
//    ri=i;
//}

//正确：显式地初始化引用和const成员
//构造函数初始值列表只说明初始化成员，不限定初始化的具体执行顺序
//成员初始化的顺序与他们在类定义中出现顺序一致
ConstRef::ConstRef(int ii):i(ii),ci(ii),ri(i) {}
