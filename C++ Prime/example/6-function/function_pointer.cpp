//
// Created by zxj on 2022/11/28.
//
#include <iostream>
#include <string>

bool lengthCompare(const std::string& s1,const std::string& s2)
{
    return s1.size()>s2.size();
}

//指向不同函数类型的指针不存在转换规则
std::string::size_type sumLength(const std::string&,const std::string&);
bool cstringCompare(const char*,const char*);

//重载函数的指针
void ff(int*);
void ff(unsigned int);

//Func和Func2是函数类型
typedef bool Func(const std::string&,const std::string&);
    //使用类型指示符时，注意里面的函数名是否加调用运算符，加调用运算符，则返回函数类型
    //不加调用运算符则返回函数类型
    //decltype返回函数类型，不会将函数自动转换成指针类型
typedef decltype(lengthCompare) Func2;  //等价的类型

//FuncP和FuncP是指向函数的指针
typedef bool(*FuncP)(const std::string&,const std::string&);
typedef decltype(lengthCompare) *FuncP2;

//编译器自动将Func表示的函数类型转换成指针
void useBigger(const std::string&,const std::string&,Func);

void useBigger(const std::string&,const std::string&,FuncP2);

typedef int (*PF)(int*,int);
PF f1(int); //PF是指向函数的指针，f1返回指向函数的指针

int main()
{
//声明一个可以指向该函数的指针，只需要使用指针替换函数即可
    //pf是函数指针，函数指针指向某种特定类型，类型由它的返回类型和形式参类型共同决定
    //与函数名无关
    bool (*pf)(const std::string&,const std::string&);

    //把函数名作为一个值使用时，该函数自动转化为指针
    pf= lengthCompare;
    //pf1=&lengthCompare;

//可以直接使用指向函数的指针调用该函数，无需提前解引用指针
    bool b1=pf("hello","goodbye"); //调用lengthCompare函数
    bool b2=(*pf)("hello","goodbye");  //等价的调用
    bool b3= lengthCompare("hello","goodbye");

    pf=0;   //正确：pf不指向任何函数
//    pf=sumLength;   //错误：返回类型不匹配
//    pf=cstringCompare;  //错误：形参类型不匹配
    pf=lengthCompare;   //正确：函数和指针的类型精准匹配

    void (*pf1)(unsigned int)=ff;   //pf1指向(unsigned)

    //编译器通过指针类型决定选用哪个函数，指针类型必须与重载函数中的一个精确匹配
//    void (*pf2)(int)=ff;    //没有ff与该形参列表匹配
//    double (*pf3)(int*)=ff; //错误：ff和pf3的返回类型不匹配

}
