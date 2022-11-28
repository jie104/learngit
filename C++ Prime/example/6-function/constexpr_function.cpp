//
// Created by zxj on 2022/11/27.
//

#include <iostream>
#include <string>
#include <cassert>

//函数返回类型及所有形参的类型都是字面值类型，且函数体中必须有且只有一条return语句
//字面值类型是指编译时就能得到结果的类型，具体包括算术类型、引用和指针
constexpr int new_sz(){return 42;}
constexpr int foo=new_sz(); //正确：foo是一个常量表达式

//允许constexpr函数的返回值并非一个常量
//如果arg是常量表达式，则scale(arg)也是一个常量表达式
constexpr size_t scale(size_t cnt){return new_sz()*cnt;}

//当scale的实参是常量表达式时，它的返回值也是常量表达式，反之则不然
int arr[scale(2)];
int i=2;    //i不是常量表达式
int a2[scale(i)];   //错误：scale(i)不是常量表达式

inline bool isShorter(const std::string &s1,const std::string &s2){
    return s1.size() < s2.size();
}

std::string word;
int threshold;
//预处理变量
assert(word.size() >threshold);