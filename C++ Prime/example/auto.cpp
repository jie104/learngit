//2.5.2、auto类型说明

#include <iostream>

int main()
{
//auto让编译器通过初始值来推算变脸的类型，所以auto定义必须有初始值
    auto item=3.14;

//一条声明语句自己能有一个基本数据类型，所以该语句中的所有变量的初始
//基本类型都必须一样
    auto i=0,*a=&i;
    auto sz=0,pi=3.14;

//auto一般会忽略顶层const，同时低层const会被保留
    const int ci=i,&cr=ci;
    auto b=ci;  //b是一个整数(ci的顶层const特性被忽略掉了)
    auto c=cr;  //c是一个整数(cr是ci的别名，ci本身是一个顶层const)
    auto d=&i;  //d是一个整型指针(整数的地址就是指向整数的指针)
    auto e=&ci; //e是一个指向常量的指针(对常量对象取地址是一种低层const)

//如果希望推演出的类型是顶层const，则
    const auto f=ci;   //ci的推演类型是int，f是const int

//可以将引用的类型设为auto，引用原来的初始化规则适用
    auto &g=ci; //g是一个整型常量引用，绑定到ci
    auto &h=42; //不能为非常量引用绑定字面值
    const auto &j=42;   //可以为常量引用绑定字面值


    a=42;   //a是指向int指针，错误
    b=42;   //b是一个整数，正确
    c=42;   //c是整数，正确
    d=42;   //d是一个整型指针，错误
    e=42;   //e是一个低层const的int指针，错误
    f=42;   //f是一个顶层const的常量，错误

    {
        const int i=42;
        auto j=i;
        const auto &k=i;
        auto *p=&i;
        const auto j2=i,&k2=i;
    }

}