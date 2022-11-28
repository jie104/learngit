//
// Created by zxj on 2022/11/27.
//

#include <iostream>
#include <string>
//函数重载时，顶层const不起作用
int calc(int,int);
int calc(const int,const int);

//函数重载时，除了返回类型不一样，其他一样，无法对函数进行区分
int get();
double get();

//可以重载，能够区分
int *reset(int *);
double *reset(double *);

std::string &read(const std::string & s);
void print(const std::string &);
void print(double); //重载print函数
void fooBar(int val);
int main()
{

}
std::string &read(const std::string & s){
    const std::string& a=s;
    return const_cast<std::string&>(a);
}

void fooBar(int val){
    bool read= false;   //新作用域：隐藏了外层的read
    std::string s=read();   //错误：read是一个布尔值，而非函数

    //通常来说，在局部作用域中声明函数不是一个好的选择
    void print(int);
    void print("Value: ");
    print(ival);
    print(3.14);
}

void print(const std::string & s){
    std::cout << s << std::endl;
}

void print(double k){
    std::cout << k << std::endl;
}
