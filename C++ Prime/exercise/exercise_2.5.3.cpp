#include <iostream>

int main()
{
//exercise 2.3.6
{
    int a=3,b=4;
    decltype(a) c=a;
    decltype((b)) d=a;
    ++c;
    ++d;
    std::cout << "a: " << a << std::endl;   //4
    std::cout << "b: " << b << std::endl;   //4
    std::cout << "c: " << c << std::endl;   //4
    std::cout << "d: " << d << std::endl;   //4
    
}

//exercise 2.3.7
{
    int a=3,b=4;
    decltype(a) c=a;
//赋值是一种会产生引用的一类典型表达式，引用的类型就是左值的类型
//如果i是int，则表达式i=x的类型是int&
    decltype(a=b) d=a;
    std::cout << "a: " << a << std::endl;   
    std::cout << "b: " << b << std::endl;   
    std::cout << "c: " << c << std::endl;   
    std::cout << "d: " << d << std::endl;   

}

}