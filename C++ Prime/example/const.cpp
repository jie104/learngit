#include <iostream>

//const对象一旦创建后，其值就不能再改变，所以const对象必须初始化

int main()
{
    int errNUmb=0;
    int *const curErr=&errNUmb; //curErr将一直指向errNumb
    const double pi=3.14159;
    const double *const pip=&pi;    //pip是一个指向常量对象的常量指针

    *curErr=6;
    // pip=&errNUmb;
    // *pip=8;

    int i=0;
    const int ic=3;
    const int *const p3=&ic;

    const int *p1=&ic;
    int *const p2=p1;

    const int *const p3=&ic;
    // ic=p3;

    const int &a=3.14;  //只有const int 常量引用才允许初始化绑定其他基本数据类型

    //指针本身是一个对象，他可以指向另一个对象，因此指针本身是不是常量，及指针所指的是不是常量，是两个相互独立的问题
    int * b=&errNUmb;
    int * const c=&errNUmb;

    int& d=3;
    int f=d;

    i=ic;



}


int f(const int a){//函数参数列表的常量const 可以不初始化，注意与const对象区分
    std::cout << a;
}