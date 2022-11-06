#include <iostream>

int main()
{
    int a=3;
    int& b=a;   //引用

    std::cout << "b: " << b << std::endl;

    int& c=b;

    c=a;
    std::cout << "c: " << c << std::endl;
}