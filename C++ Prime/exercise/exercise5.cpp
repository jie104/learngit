//C++ Primer 2.4.3 exercise

#include <iostream>

int main() 
{
    int i=0;
    const int v2=3;
    int v1=v2;
    int *p1=&v1;
    int &r1=v1;
    const int *p2=&v2;
    const int *const p3=&i;
    const int &r2=v2;

    r1=v2;  //由于引用初始化已经绑定了初值，所以此处是属于拷贝赋值，与v2是不是const无关
    r1=4;
    std::cout << "v1: " << v1 << std::endl;
    p2=p1;
    //p1=p3;  //拷贝赋值要求基本变量类型与所赋值对象一致，初始化常量类型不要求(例如const int)
    p2=p3;  //赋值值只考虑底层const，不考虑顶层const
}

