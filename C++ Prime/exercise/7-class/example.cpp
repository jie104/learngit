//
// Created by zxj on 2022/12/18.
//
#include<vector>

class Example{
public:
    //必须在类的外部定义和初始化每个静态成员
    static double rate;
    static std::vector<double> vec;

    //类的静态成员不应该在类的内部初始化，但我们可以为静态成员提供const整数
    //类型的类内初始值，不过要求静态成员必须是字面值常量类型
    static const int vecSize=20;
};

double Example::rate=6.5;
std::vector<double> Example::vec(vecSize);


