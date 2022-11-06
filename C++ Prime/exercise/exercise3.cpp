#include<memory>
#include <iostream>

void fun()
{
    int *d;
    int f;
    int e=f;
    std::cout << "the adress of d is " << d << std::endl;
    std::cout << "f is " << f << std::endl;
    std::cout << "e is " << e << std::endl;
}

int main()
{
    int a=2;
    int* b=&a;
    std::cout << "the adress of b is " << b << std::endl;

    int*c;  //默认初始化
    if (c==nullptr){
        std::cout << "c is nullptr" << std::endl;
    }
    std::cout << "the adress of c is " << c << std::endl;
    // fun();

    std::shared_ptr<int> d(new int(2));
    std::cout << "the adress of d is " << d << std::endl;
    std::cout << "*d= " << *d << std::endl;

    std::shared_ptr<double> d1; //共享指针默认初始化是空指针
    if (d1==nullptr){
        std::cout << "d1 is nullptr" << std::endl;
        std::cout << d1 << std::endl;
    }


}