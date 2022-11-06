#include <iostream>

int main()
{
    int *a=nullptr;   //初始化指针为空指针,写法：基本数据类型+类型修饰符*
    std::cout << "a: " << a << std::endl;
    int num=3;
    std::cout << "num: " << num << std::endl;
    a=&num;
    std::cout << "a: " << a << std::endl;
    *a=4;
    std::cout << "num: " << num << std::endl;
    int *d,e;
    d=&num; 
    e=1;
    std::cout << "d: " << d << std::endl;
    std::cout << "e: " << e << std::endl;

/******************指向指针的指针*********************/
    int ival=1024;
    int *pi=&ival;  //pi指向一个int型的数
    int **ppi=&pi;  //ppi指向一个int型的指针

    std::cout << "The value of ival\n"
              << "direct value: " << ival << "\n"
              << "indirect value: " << *pi << "\n"
              << "double indirect value: " << **ppi
              << std::endl;

/**************指向指针的引用***********************/
    int i=42;
    int *p;     //p是一个int型指针
    int *&r=p;  //r是一个对指针p的引用，从右往左读，较易理解

    r=&i;   //r引用一个指针，因此给r赋值&i，即令p指向i
    *r=0;   //解引用r得到i，即p指向的对象，将i的值改为0
}
