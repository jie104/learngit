//
// Created by zxj on 2022/11/23.
//

#include <initializer_list>
#include <iostream>
#include <string>
void error_msg(std::initializer_list<std::string> i1);
void foo(int parm,...);//省略符形参
void foo(...);
std::string make_plural(size_t ctr,const std::string &word,const std::string &ending);
const std::string &shoterString(const std::string &s1,const std::string &s2);

struct vector{
    int x;
    int y;
    vector(int a,int b):x(a),y(b){}

};
const vector &compare(const int &n1,const int &n2);

int main()
{
    std::string expected="adsa";
    std::string actual="zcxzc";
//    if (expected!=actual){
//        error_msg({"functionX",expected,actual});
//
//    }
//    else{
//        error_msg({"functionX","okay"});
//    }
    std::cout << compare(1,2).x  << std::endl;
}

void error_msg(std::initializer_list<std::string> i1){
    for (auto beg=i1.begin();beg!=i1.end();++beg){
        std::cout << *beg << " ";
    }
    std::cout << std::endl;
}

//该函数的返回类型是string，意味着返回值将被拷贝到调用点
std::string make_plural(size_t ctr,const std::string &word,
                                    const std::string &ending){
    return (ctr>1)? word+ending:word;
}

//函数挑出两个string形参较短的那个并返回其引用
const std::string &shoterString(const std::string &s1,const std::string &s2){
    return s1.size() <= s2.size() ? s1:s2;
}

//不要返回局部对象的引用或指针，函数完成后，它所占用的存储空间也随之释放掉
//因此，函数终止意味着局部变量的引用将指向不再有效的内存区域
const vector &compare(const int &n1,const int &n2){
    int n=n1;
    vector v1=vector(n1,n2);
    vector & v=v1;
    return vector(n1,n2);
}