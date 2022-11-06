#include <iostream>


int main()
{
//sizeof运算符返回一条表达式或一个类型名字所占的字符数

//exercise 4.28
    int a=1;
    float b=2.0;
    double c=3.0;
    int d[4]={1,2,3,4};

    std::cout << "sizeof(a): " << sizeof(a) << std::endl;;
    std::cout << "sizeof(b): " << sizeof(b) << std::endl;
    std::cout << "sizeof(c): " << sizeof(c) << std::endl;
    std::cout << "sizeof(d): " << sizeof(d)  << std::endl;

//exercise 4.29
    int x[10];
    int *p=x;
    std::cout << sizeof(x)/sizeof(*x) << std::endl;
    std::cout << sizeof(p)/sizeof(*p) << std::endl;

//sizeof 运算符返回一条表达式或一个类型名字所占字节数
    std::cout << sizeof(1<2) << std::endl;

}