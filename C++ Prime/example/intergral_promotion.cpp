#include <iostream>


int main()
{
//如果一个运算对象是无符号类型，另一个对象是带符号类型，其中的无符号类型不小于
//带符号类型，则带符号的运算对象转化为无符号
    unsigned int a=3;
    int b=-10;
    auto c=a+b;
    std::cout << c << std::endl;

//带符号类型大于无符号类型，此时转换的结果依赖于机器；如果无符号类型的所有值都能存在于
//该带符号类型，则无符号的运算对象转换成带符号类型，否则带符号类型对象转化为无符号类型
    long e=1;
    unsigned int f=-2;
    auto h=e+f;


}