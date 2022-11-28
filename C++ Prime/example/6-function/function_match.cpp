//
// Created by zxj on 2022/11/28.
//

#include <iostream>
void f();
void f(int);
void f(int,int);
void f(double,double=3.14);

//顶层const对函数重载不起作用，const引用是低层const
//低层const
int calc(int&,int&);
int calc(const int&,const int&);

//底层const
int calc(char*,char*);
int calc(const char*,const char*);

//顶层const,对函数重载不起作用，下面两个函数本质上是一个
int calc(char*,char*);
int calc(char* const,char* const);



int main()
{
//    f(2.56,42);//二义性
    f(42);
    f(42,0);
    f(2.56,3.14);
}

void f(){
    std::cout <<  1;
}

void f(int n){
    std::cout << n << std::endl;
}

void f(int n1,int n2){
    std::cout << n1 << " " << n2 << std::endl;
}

void f(double f,double f1){
    std::cout << f << " " << f1 << std::endl;
}



