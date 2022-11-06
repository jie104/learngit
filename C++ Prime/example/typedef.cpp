#include <iostream>

//类型别名
int main()
{
    //传统方法
    typedef double wages;
    typedef wages base,*p;

    //新标准
    using SI=double;

//和过去一样，const是对给定类型的修饰
    typedef char *pstring;
    const pstring cstr=0;   //cstr是指向char的常量指针，此时是顶层const
    const pstring *ps;      //ps是一个指针，他的对象是指向char的常量指针

//使用类型别名时，不能简单的理解为类型名替换他本来的样子
    const char *cstr=0;   //是对const pstring cstr的错误理解
    std::cout << cstr;



}
